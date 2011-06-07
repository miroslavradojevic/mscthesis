close all; clear all; clc;

EXPORT_IMAGES = 1;
MAXIMIZE = 1;
InputMessages  =  importdata('logFile.input',  ',');
Observations   = csvread('logFile.obser');
OutputMessages  = importdata('logFile.output', ',');
GpsMessages     = importdata('logFile.gps', ',');
OldNav          = importdata('logFile.old', ',');
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

LENGTH = floor(0.22 * size(OldNav,1));
GPSnav = OldNav(:,12:13);
INDEXES_o = find(abs(OldNav(1:LENGTH,5))<0.0001);
INDEXES_x = find(abs(OldNav(1:LENGTH,5))>0.1);
INDEXES = [];
%INDEXES = [INDEXES; INDEXES_o];
INDEXES = [INDEXES; INDEXES_x];
%%%%%% NORTH-EAST TEST %%%%%%%%
% % % % figure;%('Position',[(scrsz(3)/2) 30 (scrsz(3)/2) (scrsz(4)-104)],'Renderer','zbuffer','doublebuffer','on');
% % % % plot3(OldNav(:,2), OldNav(:,1), OldNav(:,3), 'y-.'); hold on; grid on; axis equal;
% % % % plot3(OldNav(:,2), OldNav(:,1), OldNav(:,3), 'r.');
% % % % set(gcf, 'Position', get(0,'Screensize')); % Maximize figure.
% % % % set(gca,'ZDir','reverse');
% % % % plot3(Obs(:,2), Obs(:,1), OldNav(:,3), 'bo');
% % % % plot3(OutputMessages(:,2), OutputMessages(:,1), OutputMessages(:,3), 'g', 'LineWidth', 3); hold on;
% % % % rotate3d(gcf);
% % % % xlabel('east [m]');  ylabel('north [m]'); zlabel('depth [m]');
% % % % legend('rejected outlier', 'median&dead recon', 'lbl', 'ekf');
% % % % if(EXPORT_IMAGES),
% % % %             set(gcf, 'Color', 'none');  
% % % %             export_fig ekfForLbl3d.eps -eps -a1;
% % % % end

figure;
%plot(OldNav(1:LENGTH,2), OldNav(1:LENGTH,1), 'k-.'); 
plot(OldNav(INDEXES,13), OldNav(INDEXES,12), 'kx');

hold on; axis equal; grid on;
% plot(OldNav(:,2), OldNav(:,1), 'r.');

%plot(OldNav(1:3:LENGTH,13), OldNav(1:3:LENGTH,12), 'y.-');

plot(OutputMessages(1:LENGTH,2), OutputMessages(1:LENGTH,1), 'g', 'LineWidth', 2);

xlabel('east [m]'); ylabel('north [m]'); 
%plot(GpsMessages(:,2), GpsMessages(:,1), 'ro');
legend( 'gps', 'ekf');
%title('ekf localisation');
if(MAXIMIZE)
    set(gcf, 'Position', get(0,'Screensize')); % Maximize figure.
end
if(EXPORT_IMAGES),
            set(gcf, 'Color', 'none');  
            export_fig square1NoGps.eps -eps -a1;
end

time = cumsum(InputMessages(:,1));
%%%%% NORTH vs EAST %%%%%%%%%%
% % % % % % % figure;
% % % % % % % subplot(211);
% % % % % % % plot(time, OldNav(:,1), 'r.'); hold on; grid on;axis equal;
% % % % % % % plot(time, Obs(:,1), 'bx');
% % % % % % % plot(time, OutputMessages(:,1), 'g');
% % % % % % % 
% % % % % % % plot(time, OutputMessages(:,1)+2*sqrt(OutputMessages(:,12)),'k-.'); 
% % % % % % % plot(time, OutputMessages(:,1)-2*sqrt(OutputMessages(:,12)),'k-.');
% % % % % % % 
% % % % % % % legend('dead reckoning','measured', 'filtered');
% % % % % % % xlabel('time [s]');  ylabel('north [m]');
% % % % % % % 
% % % % % % % subplot(212);
% % % % % % % plot(time, OldNav(:,2), 'r.'); hold on; grid on;axis equal;
% % % % % % % plot(time, Obs(:,2), 'bx');
% % % % % % % plot(time, OutputMessages(:,2), 'g');
% % % % % % % 
% % % % % % % %plot(time, OutputMessages(:,2)+2*sqrt(OutputMessages(:,13)),'k-.'); 
% % % % % % % %plot(time, OutputMessages(:,2)-2*sqrt(OutputMessages(:,13)),'k-.');
% % % % % % % 
% % % % % % % legend('dead reckoning','measured', 'filtered');
% % % % % % % xlabel('time [s]');  ylabel('east [m]');
% % % % % % % 
% % % % % % % if(EXPORT_IMAGES),
% % % % % % %             set(gcf, 'Color', 'none');  
% % % % % % %             export_fig north-east.eps -eps -a1;
% % % % % % % end
% % % % % % % 
% % % % % % % %%%%%% YAW-YAW RATE TEST %%%%%%%%
figure;

subplot(411);
plot(time(1:LENGTH), (Obs(1:LENGTH,8))*180/pi, 'kx');  hold on; ylabel('yaw [deg]');
plot(time(1:LENGTH), OutputMessages(1:LENGTH,8) *180/pi, 'r');

% plot(time, outputmessages(:,8)*180/pi+2*sqrt(outputmessages(:,19)*180/pi),'k-.'); 
% plot(time, outputmessages(:,8)*180/pi-2*sqrt(outputmessages(:,19)*180/pi),'k-.');
% plot(time, (outputmessages(:,8)-obs(:,8)+2*pi)*180/pi, 'b.'); grid on;
legend('measured', 'filtered');
%title ('compass');

subplot(412);
plot(time(1:LENGTH), Obs(1:LENGTH,10), 'kx'); hold on; ylabel('yaw rate [rad/s]');
plot(time(1:LENGTH), OutputMessages(1:LENGTH,10), 'r'); grid on;
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
plot(time(1:LENGTH), Obs(1:LENGTH,5), 'kx');  hold on;
plot(time(1:LENGTH), OutputMessages(1:LENGTH,5), 'r');
%plot(time, OutputMessages(:,5)+2*sqrt(OutputMessages(:,16)),'k-.'); 
%plot(time, OutputMessages(:,5)-2*sqrt(OutputMessages(:,16)),'k-.');
legend('measured', 'filtered');% , '2\sigma boundary'
ylabel('surge [m/s]');
% 
subplot(414);
plot(time(1:LENGTH), Obs(1:LENGTH,6), 'kx'); hold on;
plot(time(1:LENGTH), OutputMessages(1:LENGTH,6), 'r');
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