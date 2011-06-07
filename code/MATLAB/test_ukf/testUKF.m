close all; clear all; clc;

EXPORT_IMAGES = 1;

InputMessages   =  importdata('logFile.input',  ',');
Observations    =  csvread('logFile.obser');
OutputMessages  =  importdata('logFile.output', ',');
OldNav          =  importdata('logFile.old', ',');
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
%%%%%% UKF FILTERING OFFLINE %%%%%%
lambda = -0.7;

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


%%%%%% NORTH-EAST TEST %%%%%%%%
% % % % fig3d=figure;%('Position',[(scrsz(3)/2) 30 (scrsz(3)/2) (scrsz(4)-104)],'Renderer','zbuffer','doublebuffer','on');
% % % % axis3d = gca; %get the current axes handle
% % % % rotate3d(fig3d);
% % % % axis3d = gca; %get the current axes handle
% % % % set(axis3d,'XDir','reverse','ZDir','reverse');
% % % % hold(axis3d);
% % % % set(fig3d,'CurrentAxes',axis3d);
% % % % axis equal; %zlim([-0.1 0.4]); 
% % % % grid(axis3d);
% % % % xlabel('east (m)'); ylabel('north (m)'); zlabel('depth (m)');
% % % % plot3(axis3d,OldNav(:,2), OldNav(:,1), OldNav(:,3), 'r.'); hold on;
% % % % lbl_measured = find(Observations(:,1)==2);
% % % % plot3(axis3d, Obs(:,2), Obs(:,1), zeros(size(Obs(:,1))), 'bo');
% % % % plot3(axis3d, OutputMessages(:,2), OutputMessages(:,1), OutputMessages(:,3), 'g'); axis equal;
% % % % legend('dead reckoning','lbl', 'filtered');
% % % % title('ekf localisation');

figure;
plot(OldNav(:,2), OldNav(:,1), 'r.'); hold on; grid on; axis equal;
plot(OutputMessages(:,2), OutputMessages(:,1), 'g', 'LineWidth', 2);
plot(X_ukf(:,2), X_ukf(:,1), 'b', 'LineWidth', 2);
legend('dead reckoning', 'ekf filtered', 'ukf filtered');
legend('Location', 'NorthWest');
xlabel('east [m]'); ylabel('north [m]');

if(EXPORT_IMAGES),
            set(gcf, 'Color', 'none');  
            export_fig compareUKF.eps -eps -a1;
end

% % % % time = cumsum(InputMessages(:,1));
%%%%%% YAW-YAW RATE TEST %%%%%%%%
% % % % figure;
% % % % subplot(211);
% % % % plot(time, (Obs(:,8))*180/pi, 'kx');  hold on; ylabel('yaw [deg]');
% % % % plot(time, OutputMessages(:,8) *180/pi, 'r');
% % % % 
% % % % plot(time, OutputMessages(:,8)*180/pi+2*sqrt(OutputMessages(:,19)*180/pi),'k-.'); 
% % % % plot(time, OutputMessages(:,8)*180/pi-2*sqrt(OutputMessages(:,19)*180/pi),'k-.');
% % % % 
% % % % plot(time, (OutputMessages(:,8)-Obs(:,8)+2*pi)*180/pi, 'b.'); grid on;
% % % % legend('measured', 'filtered');
% % % % title ('compass');

% % % % subplot(212);
% % % % plot(time, Obs(:,10), 'kx'); hold on; ylabel('yaw rate [rad/s]');
% % % % plot(time, OutputMessages(:,10), 'r'); grid on;
% % % % plot(time, OutputMessages(:,10)+2*sqrt(OutputMessages(:,21)),'k-.'); 
% % % % plot(time, OutputMessages(:,10)-2*sqrt(OutputMessages(:,21)),'k-.');
% % % % legend('measured', 'filtered', '2\sigma boundary');
% % % % title('gyroscope');
% % % % if(EXPORT_IMAGES),
% % % %     set(gcf, 'Color', 'none');  
% % % %     export_fig yaw-test.eps -eps -a1;
% % % % end
%%%%%% SURGE-SWAY RATE TEST %%%%%%%%
% % % % % figure;
% % % % % subplot(211);
% % % % % plot(time, Obs(:,5), 'kx');  hold on;
% % % % % plot(time, OutputMessages(:,5), 'r');
% % % % % plot(time, OutputMessages(:,5)+2*sqrt(OutputMessages(:,16)),'k-.'); 
% % % % % plot(time, OutputMessages(:,5)-2*sqrt(OutputMessages(:,16)),'k-.');
% % % % % legend('measured', 'filtered', '2\sigma boundary');
% % % % % title ('surge vel [m/s] (dvl)');
% % % % % 
% % % % % subplot(212);
% % % % % plot(time, Obs(:,6), 'kx'); hold on;
% % % % % plot(time, OutputMessages(:,6), 'r');
% % % % % plot(time, OutputMessages(:,6)+2*sqrt(OutputMessages(:,17)),'k-.'); 
% % % % % plot(time, OutputMessages(:,6)-2*sqrt(OutputMessages(:,17)),'k-.');
% % % % % legend('measured', 'filtered', '2\sigma boundary');
% % % % % title(' sway vel [m/s] (dvl)');
% % % % % if(EXPORT_IMAGES),
% % % % %     set(gcf, 'Color', 'none');  
% % % % %     export_fig vel-test.eps -eps -a1;
% % % % % end
%%%%%% DEPTH TEST %%%%%%%%
% % % % figure;
% % % % subplot(211);
% % % % plot(time, Obs(:,3), 'kx'); hold on;
% % % % plot(time, OutputMessages(:,3), 'r');
% % % % plot(time, OutputMessages(:,3)+2*sqrt(OutputMessages(:,14)),'k-.'); 
% % % % plot(time, OutputMessages(:,3)-2*sqrt(OutputMessages(:,14)),'k-.');
% % % % legend('measured', 'filtered', '2\sigma boundary');
% % % % title(' depth [m] (pressure)');
% % % % 
% % % % subplot(212);
% % % % plot(time, Obs(:,7), 'kx'); hold on;
% % % % plot(time, OutputMessages(:,7), 'r');
% % % % plot(time, OutputMessages(:,7)+2*sqrt(OutputMessages(:,18)),'k-.'); 
% % % % plot(time, OutputMessages(:,7)-2*sqrt(OutputMessages(:,18)),'k-.');
% % % % legend('measured', 'filtered', '2\sigma boundary');
% % % % title(' heave vel [m/s] (pressure)');
% % % % if(EXPORT_IMAGES),
% % % %     set(gcf, 'Color', 'none');  
% % % %     export_fig vel-test.eps -eps -a1;
% % % % end
%close all;
