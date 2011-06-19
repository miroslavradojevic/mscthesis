%%% LOCALIZATION USING UnscentedKF (pitch included) %%%
%==========================================================================
%state consisits of:
%   original state
%                   x, y, z coordinates     (wrt to pool coord sys)
%                   yaw rotation around z   (wrt to pool coord sys)
%                   pitch rotation around y (wrt to pool coord sys)
%                   u, v, w linear speeds   (wrt to vehicle coord sys)
%                   vyaw rotation rate
%   noise variables
%                   nu   (lin acceleration along x) 
%                   nv   (lin acceleration along y)
%                   nw   (lin acceleration along z)
%                   nr   (yaw ang acceleration)
%                   nq   (pitch ang acceleration)
%==========================================================================
% imaging sonar is not used to measure distances
%
fclose('all'); clear all; close all; clc
addpath('utils'); % tool functions are available 
%LOAD DEBUGGING FILE=======================================================
FILE.log = fopen('logFile.txt', 'w');

%LOAD LOG FILES ===========================================================
experiment='log-files\_040825_1735_'; % location of logs

if isunix
    experiment=strrep(experiment,'\','/');
end

global FILE PARAM  PLOT CONFIG DATA whichDevice whichValues %SUBMAP % SECTOR VOTERS
whichDevice = 'NNN'; whichValues = [];

%configure which devices are active========================================
CONFIG.enableGPS = 1; % define whether device is used
CONFIG.enableMTI = 1;
CONFIG.enableDVL = 1;
%configure what they measure
CONFIG.valuesGPS = [1 1];     % [x y]
CONFIG.valuesMTI = [1 1 1 1]; % [yaw pitch yaw_rate pitch_rate]
CONFIG.valuesDVL = [1 0 0 1]; % [(uvw) yaw pitch z]

CONFIG.plotEllipses = 0;
CONFIG.count = 3000;

%FILE======================================================================
%Ids for the opened files
FILE.fid(1) = fopen([experiment 'DVL.log']);
%FILE.fid(2) = fopen([experiment 'IS.log']); % not used
FILE.fid(3) = fopen([experiment 'DGPS.log']);
FILE.fid(4) = fopen([experiment 'MTi.log']);

%STATE variances  =========================================================
PARAM.SDx    =  0.1;    %m
PARAM.SDy    =  0.1;    %m
PARAM.SDz    =  0.1;    %m
PARAM.SDzp    =  0.02;                  %m      depth (pressure)

PARAM.SDu    =  2.501;   %0.4  0.03       %m/s    velocity x bottom
PARAM.SDv    =  2.501;   %0.4  0.03       %m/s    velocity y bottom
PARAM.SDw    =  2.501;   %0.3;            %m/s    velocity z bottom
PARAM.SDyaw    = 0.21;   % 0.45          %rad    yaw
PARAM.SDmtiyaw = 0.01;  % for MTi especially
PARAM.SDvyaw     = 0.2;                   %rad/s  yaw rate
PARAM.SDpch   = 0.2;   % 0.45          %rad    pitch
PARAM.SDvpch  = 0.2;                   %rad/s  pitch rate
PARAM.chi2          =  chi2inv(0.95,2);

%%% model (v)
PARAM.SDuR     =  0.3;  % 0.3            %m/s^2   velocity x robot
PARAM.SDvR     =  0.3;                  %m/s^2   velocity y robot
PARAM.SDwR     =  0.3;  % 0.2            %m/s^2   velocity z robot
PARAM.SDvyawR  =  0.25; % 0.5 0.7        %rad/s^2  ang. velocity yaw robot
PARAM.SDvpchR  =  0.25;
%%% ----

%initialize state vector X - find x,y,yaw
states_nr = 10;
X(1) = 0;% x 
X(2) = 0;% y 
X(3) = 0;% z 
X(4) = 0;% yaw  -taken from MTi (priority), DVL, or set to zero (according to Girona)
X(5) = 0;% pitch-taken from MTi
X(6) = 0;% u(speed along x)
X(7) = 0;% v(speed along y)
X(8) = 0;% w(speed along z)
X(9) = 0;% r(angular speed around z)
X(10)= 0;% q(angular speed around y)
%%% model noise 
V(1) = 0;%nu(lin acceleration along x)
V(2) = 0;%nv(lin acceleration along y)
V(3) = 0;%nv(lin acceleration along z)
V(4) = 0;%nr(angular acceleration around yaw)
V(5) = 0;%nq(angular acceleration around pitch)
%%% measurement noise
N(1) = 0; % measurement of x
N(2) = 0;
N(3) = 0;
N(4) = 0;
N(5) = 0; 
N(6) = 0;
N(7) = 0;
N(8) = 0;
N(9) = 0;
N(10) = 0;

X=X'; V=V'; N=N';

%DGPS data===================================
device = 0; 
store_gps_n = []; store_gps_e = [];
%%% GPS is recieved occasionally to showcase the vehicle position and update 
% the Kalman filter (instead of the LBL measurement)
% at the beginning, position (x, y, z, yaw, pitch) is estimated
if FILE.fid(3)>0, %Waits the first GPS measurement to 
                  %refer to it as initial position before starting
    % GPS extracts latitude and longitude => x, y
    while device~=3 %searches for the first GPS measurement.
        [device] = read_dataset;
    end
    time = FILE.DGPS_tline(1); %obtains the time fo the first measurement
    init_time = time;
    DGPS = FILE.DGPS_tline(2:end);
    lat=floor(DGPS(:,1)/100)+((DGPS(:,1)/100)-floor(DGPS(:,1)/100))*100/60; %obtains latitude
    lon=floor(DGPS(:,2)/100)+((DGPS(:,2)/100)-floor(DGPS(:,2)/100))*100/60; %obtains longitude
    [gps_e,gps_n, utmzone] = deg2utm(lat,lon); %transform degrees to utm
    
    offset_gps_n = gps_n - X(1); offset_gps_e = gps_e - X(2);
    store_gps_n = [store_gps_n gps_n-offset_gps_n]; %store DGPS data before starting the mission
    store_gps_e = [store_gps_e gps_e-offset_gps_e]; % x=North y=East
end
% MTI has the priority when determining the yaw
if FILE.fid(4)>0, % %exist a MTI measurement file
    while device~=4 %searches for the soonest MTI measurement
        [device] = read_dataset;
    end
    time=FILE.MTI_tline(1); %obtains the measurement time
    MTI=FILE.MTI_tline(2:end); %obtains the first mti measurement
    confMes = [1 0 0 0]; % measure yaw
    [z,H,R] = MTI_measurement(MTI, confMes);  %obtain measurements from mti data   
elseif FILE.fid(1)>0 %Exist a DVL measurement
    while device~=1 %searchs for the first DVL measurement.
        [device] = read_dataset;
    end
    time=FILE.DVL_tline(1); %obtains the initial time
    DVL=FILE.DVL_tline(2:end); %obtains the first DVL measurement
    DVL([10 14])=0; %set velocity status to 0
    confMes = [0 0 0 0 1 0];%measure only yaw
    [z,H,R] = DVL_measurement(DVL, confMes); 
else %There is no orientation measurement available
    z=0';
    R=0.5.^2;
    [device] = read_dataset;
    if isfield(FILE,'IS_tline')
        time=FILE.IS_tline(1);
    elseif isfield(FILE,'DGPS_tline')
        time=FILE.DGPS_tline(1);
    end
end
X(4)= z; %initialize the yaw for the vehicle in world coordinates
Px=zeros(1,states_nr);
Px(4)=R; %initialize uncertainty for angular measurements
%Px=diag(P);
% model noise uncertainty
Pv=[PARAM.SDuR^2 PARAM.SDvR^2 PARAM.SDwR^2 PARAM.SDvyawR PARAM.SDvpchR];
% measurement noise uncer
Pn=[PARAM.SDx^2    PARAM.SDy^2   PARAM.SDz^2 ...
    PARAM.SDyaw^2  PARAM.SDpch^2 ...
    PARAM.SDu^2    PARAM.SDv^2   PARAM.SDw^2 ...
    PARAM.SDvyaw^2 PARAM.SDvpch^2];  

% join together
XA = [X; V; N];
PA = [Px Pv Pn];
PA = diag(PA);
 

%DEFINE INIT POSITION======================================================
ini_pos = [0; 0; z];
%store simulation data=====================================================
DATA.time=time; 
DATA.init_time=time;

%INITIALIZE FIGURES========================================================
scrsz = get(0,'ScreenSize');
%%% set 2d plot
PLOT.fig2d=figure('Position',[0 30 (scrsz(3)/2) (scrsz(4)-104)],'Renderer','zbuffer','doublebuffer','on');%(scrsz(3)/2)+20
axis2d = gca; %get the current axes handle
set(axis2d,'XDir','reverse'); %Invert X direction so Z looks downwards
hold(axis2d); 
set(PLOT.fig2d,'CurrentAxes',axis2d);
axis equal; %axis([-50 40 -20 50]); 
grid(axis2d);
xlabel('X (m)'); ylabel('Y (m)');

%%% set 3d plot
PLOT.fig3d=figure('Position',[(scrsz(3)/2) 30 (scrsz(3)/2) (scrsz(4)-104)],'Renderer','zbuffer','doublebuffer','on');
rotate3d(PLOT.fig3d);
axis3d = gca; %get the current axes handle
set(axis3d,'XDir','reverse','ZDir','reverse');
hold(axis3d);
set(PLOT.fig3d,'CurrentAxes',axis3d);
axis equal; %zlim([-0.1 0.4]); 
grid(axis3d);
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%MAIN======================================================================
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
flag_DVLread = 0; % to make balance in readings MTi/DVL
tic;
stop = 0;    % flag for stopping once the end of LOG files is reached
count = 0;   % counts the iterations (can be used for stopping as well)
while (stop==0) && (count<=CONFIG.count),  
    
    %X_prev=X;
    
    [device] = read_dataset; %read the next line in dataset
    DATA.x_mes = NaN; DATA.y_mes=NaN; DATA.z_mes=NaN;   DATA.yaw_mes=NaN; DATA.pitch_mes=NaN;
    DATA.u_mes = NaN; DATA.v_mes = NaN; DATA.w_mes = NaN; % initailize before potential measurement
                                                          % those
                                                          % interesting for
                                                          % plotting
    switch device
        % for each device Unsc. Kalman
        case 1 %DVL measurement available
            if CONFIG.enableDVL, %if available
                count = count + 1; flagMeasurementOccured = 1;
                %%%flag_DVLread = 1; % enable MTI after but only once
                % check for measurements 
                At=FILE.DVL_tline(1)-DATA.time(end); %time interval dT
                DATA.time(end+1)=FILE.DVL_tline(1); %store time for next iteration
                DVL=FILE.DVL_tline(2:end); %store sensor raw data
                if FILE.fid(4)<0 | (~CONFIG.enableMTI),%if there is no MTI available, or MTI excluded
                    confMes = [1 1 1 1];%obtain all measurements from DVL data
                    [z,H1,R1] = DVL_measurement(DVL, confMes); % z= [u; v; w; yaw; pitch; depth] 
                else % MTI available, no need to measure yaw
                    [z,H1,R1] = DVL_measurement(DVL, CONFIG.valuesDVL); % z = [u; v; w; depth];
                end
                                                    disp(['device ' num2str(device) ' read...   ']);
                %estimatew
                %[X,P]=KalmanFilter(X,P,At,z,H,R); %includes prediction & update
                [XA, PA] = UnscentedKalman(XA, PA, z, At);
                X = XA(1:10,1); P = PA(1:10, 1:10); %extract from extended state vector
                PLOT.fig2d; h = plot(axis2d,X(1),X(2),'mx');       drawnow;
                PLOT.fig3d;     plot3(axis3d,X(1),X(2),X(3),'mx'); drawnow;
                %jump{1}(end+1) = sqrt(sum((X(1:2)-X_prev(1:2)).^2));
            end
            if feof(FILE.fid(device))==1; %check for end of file
                fclose(FILE.fid(device)); %close file
                stop=1; %stop execution flag
            end
        case 2 % imaging sonar.. skipping it this time
            %nothing
        case 3 %DGPS measurement available - play as ground truth
            count = count + 1; flagMeasurementOccured = 1;
            At=FILE.DGPS_tline(1)-DATA.time(end);
            DATA.time(end+1)=FILE.DGPS_tline(1);
            DGPS=FILE.DGPS_tline(2:end); %store sensor raw data
            confMes = CONFIG.valuesGPS;
            [z,dummyH,dummyR] = GPS_measurement(DGPS, confMes, offset_gps_n, offset_gps_e);
                                                disp(['device ' num2str(device) ' read...']);
                % plot reference positions in blue
                PLOT.fig2d;  plot(axis2d,DATA.x_mes,DATA.y_mes, 'bo'); drawnow;
                PLOT.fig3d;  plot3(axis3d,DATA.x_mes,DATA.y_mes, 0, 'bo'); drawnow;
            if CONFIG.enableGPS, % if available for estimation
                    %[X,P]=UnscentedKalman(X, P, z, H, At, R);
                    [XA, PA] = UnscentedKalman(XA, PA, z, At);
                    X = XA(1:10,1); P = PA(1:10, 1:10);
                    % plot robot positions in red
                    PLOT.fig2d; h = plot(axis2d,X(1),X(2),'rx');       drawnow;
                    PLOT.fig3d;     plot3(axis3d,X(1),X(2),X(3),'rx'); drawnow;
                    %jump{2}(end+1) = sqrt(sum((X(1:2)-X_prev(1:2)).^2));
            end
            if feof(FILE.fid(device))==1; %check for end of file
                fclose(FILE.fid(device)); %close file
                stop=1; %stop execution
            end
        case 4 %MTI measurement available
            
            if CONFIG.enableMTI,
                    %%%if flag_DVLread==1,
                
                count = count + 1; flagMeasurementOccured = 1;
                At=FILE.MTI_tline(1)-DATA.time(end); %time interval
                DATA.time(end+1)=FILE.MTI_tline(1); %store time for next iteration
                MTI=FILE.MTI_tline(2:end); %store sensor raw data
                confMes = CONFIG.valuesMTI; %measure yaw
                [z,H1,R1] = MTI_measurement(MTI, confMes); 
                                                    disp(['device ' num2str(device) ' read...']);
                %[X,P]=UnscentedKalman(X, P, z, H, At, R);
                [XA, PA] = UnscentedKalman(XA, PA, z, At);
                X = XA(1:10,1); P = PA(1:10, 1:10);
                PLOT.fig2d; h = plot(axis2d,X(1),X(2),'kx'); drawnow;
                PLOT.fig3d;     plot3(axis3d,X(1),X(2),X(3),'kx'); drawnow;
                %jump{3}(end+1) = sqrt(sum((X(1:2)-X_prev(1:2)).^2));
                
                    %%%flag_DVLread = 0; % stop next execution unless DVL happens meanwhile
                    %%%end
            end
            if feof(FILE.fid(device))==1; %check for end of file
                fclose(FILE.fid(device)); %close file
                stop=1; %stop execution
            end            
    end
    if count & flagMeasurementOccured, % store
        DATA.state(:,count)     = X;
        % fill the record with most recently obtained values
        DATA.state_mes(:,count) = ...
            [DATA.x_mes; DATA.y_mes; DATA.z_mes; DATA.yaw_mes; DATA.pitch_mes; DATA.u_mes; DATA.v_mes; DATA.w_mes];
        DATA.uncer(:,:,count)   = P;
    end
    if (mod(count, 40)==0) & count, % every once in a while
        PLOT.fig3d; hold on; grid on;
        plot_vehicle(X(1),X(2),X(3),X(4),X(5), axis3d); drawnow;
        if CONFIG.plotEllipses,
            if exist('hEllipse'), delete(hEllipse); end
            %PLOT.fig2d; 
            hEllipse = draw_ellipse([X(1) X(2)], P(1:2,1:2), 'k');  
            drawnow;
        end
    end
end
% check the localization error
ind1 = find(~isnan(DATA.state_mes(1,:)));
ind2 = find(~isnan(DATA.state_mes(2,:)));
DATA.distance = sqrt( (DATA.state(1,ind1)-DATA.state_mes(1,ind1)).^2 +...
                 (DATA.state(2,ind2)-DATA.state_mes(2,ind2)).^2 );

disp(['Total elapsed time: ' num2str(toc/60) ' (min)']);
save('UKF.mat', 'PARAM', 'DATA', 'CONFIG');
%%%PLOTTINGS---------------------------------------------------------------
plottings;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%END MAIN==================================================================
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fclose('all');
