close all; clear all; clc;

EXPORT_IMAGES = 1;
MAXIMIZE = 0;
InputMessages  =  importdata('logFile.input',  ',');
Observations   = csvread('logFile.obser');
OutputMessages  = importdata('logFile.output', ',');
GpsMessages     = importdata('logFile.gps', ',');
OldNav          = importdata('logFile.old', ',');
[name, value]   = textread('logFile.params', '%s %f',17); % read parameters used for ekf
% //indexing constants - state vector
% #define NORTH_INDEX       0
% #define EAST_INDEX        1
% #define DEPTH_INDEX       2
% #define ALTITUDE_INDEX    3
% #define SURGE_VEL_INDEX   4
% #define SWAY_VEL_INDEX    5
% #define HEAVE_VEL_INDEX   6
% #define YAW_INDEX         7
% #define PITCH_INDEX       8
% #define YAW_RATE_INDEX    9
% #define PITCH_RATE_INDEX 10

% extract observations
Obs=ones(size(Observations,1),11)*NaN;
for i = 1 : size(Observations, 1),
    MeasurementLen = Observations(i,1);
    if(Observations(i,1:3) == [1 0 0]),
        disp('nothing measured...')
    else
        MeasurementVec = Observations(i,2:MeasurementLen+1);
        MeasurementStates = Observations(i,MeasurementLen+2:2*MeasurementLen+1);
        for j = 1 : length(MeasurementStates),
            Obs(i,MeasurementStates(j)) = MeasurementVec(j);
        end
    end
end
time = cumsum(InputMessages(:,1));

LENGTH = floor(1.0 * size(OldNav,1));
GPSnav = OldNav(:,12:13);
INDEXES_o = find(abs(OldNav(1:LENGTH,5))<0.001);
INDEXES_x = find(abs(OldNav(1:LENGTH,5))>0.5);
INDEXES = [];
%INDEXES = [INDEXES; INDEXES_o];
INDEXES = [INDEXES; INDEXES_x];

%%%%%% UKF FILTERING OFFLINE ADDED %%%%%%
lambda = -0.5;
for i = 1 : size(Observations, 1),
        if(i==1),
            X(1:11) = InputMessages(1, 2:12);
            V(1:5)  = zeros(1,5);
            N(1:11) = zeros(1,11);
            X=X'; V=V'; N=N'; 
            Pv=[value(12)^2 value(13)^2 value(14)^2 value(15)^2 value(16)^2];
            Pn=[value(1)^2    value(2)^2   value(3)^2 value(4)^2 ...
                value(5)^2    value(6)^2 ...
                value(7)^2    value(8)^2   value(9)^2 ...
                value(10)^2   value(11)^2];
            Px=zeros(1,11); Px(find(X~=0))=1; Px = Px .* Pn;
            % join together
            XA = [X; V; N];
            PA = [Px Pv Pn];
            PA = diag(PA);
        end  
    At = InputMessages(i,1);    
    MeasurementLen = Observations(i,1);
    if(Observations(i,1:3) == [1 0 0]),
        disp('nothing measured... empty observation...');
        % should just do the prediction
        z = NaN; conf = [0];
        [XA, PA] = UnscentedKalman(XA, PA, z, At, conf, lambda);
    else
        % there was a measurement
        MeasurementVec = Observations(i,2:MeasurementLen+1);
        MeasurementStates = Observations(i,MeasurementLen+2:2*MeasurementLen+1);
        for j = 1 : length(MeasurementStates),
            Obs(i,MeasurementStates(j)) = MeasurementVec(j);
        end
        z = MeasurementVec;
        conf = MeasurementStates;
        %At = InputMessages(i,1);
        [XA, PA] = UnscentedKalman(XA, PA, z, At, conf, lambda);
    end
    X_ukf(i, :) = XA(1:11)'; 
    P_ukf(i,:) = diag(PA)';
end

figure;
%GPS
plot(OldNav(1:LENGTH,13), OldNav(1:LENGTH,12), 'kx');
hold on; axis equal; grid on;
%EKF
plot(OutputMessages(1:LENGTH,2), OutputMessages(1:LENGTH,1), 'g', 'LineWidth', 2);
%UKF
plot(X_ukf(1:LENGTH,2), X_ukf(1:LENGTH,1), 'b', 'LineWidth', 2);
%DR
plot(OldNav(1:LENGTH,2), OldNav(1:LENGTH,1), 'k-.'); 
xlabel('east [m]'); ylabel('north [m]'); 
%plot(GpsMessages(:,2), GpsMessages(:,1), 'ro');
legend( 'gps', 'ekf', 'ukf','dead recon');
if(MAXIMIZE)
    set(gcf, 'Position', get(0,'Screensize')); % Maximize figure.
end
if(EXPORT_IMAGES),
            set(gcf, 'Color', 'none');  
            export_fig squareWithGpsUkf-05.eps -eps -a1;
end
time = cumsum(InputMessages(:,1));
%%%%% NORTH vs EAST %%%%%%%%%%
figure;
subplot(511);
plot(time, OldNav(:,1), 'k-.'); hold on; grid on;%axis equal;
plot(time, Obs(:,1), 'kx');
plot(time, OutputMessages(:,1), 'g', 'LineWidth', 2);
%plot(time, X_ukf(:,1), 'b', 'LineWidth', 2);
%plot(time, OutputMessages(:,1)+2*sqrt(OutputMessages(:,12)),'k-.'); 
%plot(time, OutputMessages(:,1)-2*sqrt(OutputMessages(:,12)),'k-.');
legend('dead reckon','gps', 'ekf'); % , 'ukf'
xlabel('time [s]');  ylabel('north [m]');

subplot(512);
plot(time, OldNav(:,2), 'k-.'); hold on; grid on;%axis equal;
plot(time, Obs(:,2), 'kx');
plot(time, OutputMessages(:,2), 'g', 'LineWidth', 2);
%plot(time, X_ukf(:,2), 'b', 'LineWidth', 2);
%plot(time, OutputMessages(:,2)+2*sqrt(OutputMessages(:,13)),'k-.'); 
%plot(time, OutputMessages(:,2)-2*sqrt(OutputMessages(:,13)),'k-.');
legend('dead reckon','gps', 'ekf'); % , 'ukf'
xlabel('time [s]');  ylabel('east [m]');

subplot(513);
plot(time, (Obs(:,8))*180/pi, 'kx');  hold on; ylabel('yaw [deg]');
plot(time, OutputMessages(:,8) *180/pi, 'g', 'LineWidth', 2);
legend('measured', 'ekf');
xlabel('time [s]');  ylabel('heading [deg]');

subplot(514);
plot(time, Obs(:,5), 'kx');  hold on;
plot(time, OutputMessages(:,5), 'g', 'LineWidth', 2);
%plot(time, OutputMessages(:,5)+2*sqrt(OutputMessages(:,16)),'k-.'); 
%plot(time, OutputMessages(:,5)-2*sqrt(OutputMessages(:,16)),'k-.');
legend('measured', 'ekf');% , '2\sigma boundary'
ylabel('surge [m/s]');
% 
subplot(515);
plot(time, Obs(:,6), 'kx'); hold on;
plot(time, OutputMessages(:,6), 'g', 'LineWidth', 2);
% plot(time, OutputMessages(:,6)+2*sqrt(OutputMessages(:,17)),'k-.'); 
% plot(time, OutputMessages(:,6)-2*sqrt(OutputMessages(:,17)),'k-.');
legend('measured', 'ekf'); %, '2\sigma boundary'
ylabel(' sway [m/s]');

if(EXPORT_IMAGES),
            set(gcf, 'Color', 'none');  
            export_fig values.eps -eps -a1;
end
 
% % % % % % % %%%%%% YAW-YAW RATE TEST %%%%%%%%
figure;

subplot(411);
plot(time, (Obs(:,8))*180/pi, 'kx');  hold on; ylabel('yaw [deg]');
plot(time, OutputMessages(:,8) *180/pi, 'r');

% plot(time, outputmessages(:,8)*180/pi+2*sqrt(outputmessages(:,19)*180/pi),'k-.'); 
% plot(time, outputmessages(:,8)*180/pi-2*sqrt(outputmessages(:,19)*180/pi),'k-.');
% plot(time, (outputmessages(:,8)-obs(:,8)+2*pi)*180/pi, 'b.'); grid on;
legend('measured', 'filtered');
xlabel('time [s]');  ylabel('heading [deg]');

subplot(412);
plot(time, Obs(:,10), 'kx'); hold on; ylabel('yaw rate [rad/s]');
plot(time, OutputMessages(:,10), 'r'); grid on;
% plot(time, outputmessages(:,10)+2*sqrt(outputmessages(:,21)),'k-.'); 
% plot(time, outputmessages(:,10)-2*sqrt(outputmessages(:,21)),'k-.');
legend('measured', 'filtered'); % , '2\sigma boundary'

% % % % % % % if(EXPORT_IMAGES),
% % % % % % %     set(gcf, 'color', 'none');  
% % % % % % %     export_fig yaw-test.eps -eps -a1;
% % % % % % % end
% % % % % % % % % % % % % %%%%%% SURGE-SWAY RATE TEST %%%%%%%%
%figure;
subplot(413);
plot(time, Obs(:,5), 'kx');  hold on;
plot(time, OutputMessages(:,5), 'r');
%plot(time, OutputMessages(:,5)+2*sqrt(OutputMessages(:,16)),'k-.'); 
%plot(time, OutputMessages(:,5)-2*sqrt(OutputMessages(:,16)),'k-.');
legend('measured', 'filtered');% , '2\sigma boundary'
ylabel('surge [m/s]');
% 
subplot(414);
plot(time, Obs(:,6), 'kx'); hold on;
plot(time, OutputMessages(:,6), 'r');
% plot(time, OutputMessages(:,6)+2*sqrt(OutputMessages(:,17)),'k-.'); 
% plot(time, OutputMessages(:,6)-2*sqrt(OutputMessages(:,17)),'k-.');
legend('measured', 'filtered'); %, '2\sigma boundary'
ylabel(' sway [m/s]');
if(0)
    set(gcf, 'Position', get(0,'Screensize')); % Maximize figure.
end    
 if(EXPORT_IMAGES),
     set(gcf, 'Color', 'none');  
     export_fig dynamics.eps -eps -a1;
 end
% % % % % % % % % % % % % %%%%%% DEPTH TEST %%%%%%%%
% % % % % % % figure;
% % % % % % % subplot(211);
% % % % % % % plot(time, Obs(:,3), 'kx'); hold on;
% % % % % % % plot(time, OutputMessages(:,3), 'r');
% % % % % % % plot(time, OutputMessages(:,3)+2*sqrt(OutputMessages(:,14)),'k-.'); 
% % % % % % % plot(time, OutputMessages(:,3)-2*sqrt(OutputMessages(:,14)),'k-.');
% % % % % % % legend('measured', 'filtered', '2\sigma boundary');
% % % % % % % title(' depth [m] (pressure)');
% % % % % % % 
% % % % % % % subplot(212);
% % % % % % % plot(time, Obs(:,7), 'kx'); hold on;
% % % % % % % plot(time, OutputMessages(:,7), 'r');
% % % % % % % plot(time, OutputMessages(:,7)+2*sqrt(OutputMessages(:,18)),'k-.'); 
% % % % % % % plot(time, OutputMessages(:,7)-2*sqrt(OutputMessages(:,18)),'k-.');
% % % % % % % legend('measured', 'filtered', '2\sigma boundary');
% % % % % % % title(' heave vel [m/s] (pressure)');
% % % % % % % if(EXPORT_IMAGES),
% % % % % % %     set(gcf, 'color', 'none');  
% % % % % % %     export_fig dep-test.eps -eps -a1;
% % % % % % % end
% % % % % % %%%%%% ALTITUDE TEST %%%%%%%%
% % % % % % figure;
% % % % % % subplot(211);
% % % % % % plot(time, Obs(:,4), 'kx'); hold on;
% % % % % % plot(time, OutputMessages(:,4), 'r');
% % % % % % plot(time, OutputMessages(:,4)+2*sqrt(OutputMessages(:,15)),'k-.'); 
% % % % % % plot(time, OutputMessages(:,4)-2*sqrt(OutputMessages(:,15)),'k-.');
% % % % % % legend('measured', 'filtered', '2\sigma boundary');
% % % % % % title(' altitude [m] (dvl)');
% % % % % % 
% % % % % % subplot(212);
% % % % % % plot(time, Obs(:,7), 'kx'); hold on;
% % % % % % plot(time, OutputMessages(:,7), 'r');
% % % % % % plot(time, OutputMessages(:,7)+2*sqrt(OutputMessages(:,18)),'k-.'); 
% % % % % % plot(time, OutputMessages(:,7)-2*sqrt(OutputMessages(:,18)),'k-.');
% % % % % % legend('measured', 'filtered', '2\sigma boundary');
% % % % % % title(' heave vel [m/s] (pressure)');
% % % % % % if(EXPORT_IMAGES),
% % % % % %     set(gcf, 'Color', 'none');  
% % % % % %     export_fig alt-test.eps -eps -a1;
% % % % % % end
% % % % % % %close all;