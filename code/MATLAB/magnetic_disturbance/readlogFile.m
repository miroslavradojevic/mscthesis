close all; clear all; clc;

EXPORT_IMAGES = 0;

InputMessages  =  importdata('logFile.input',  ',');
Observations   = csvread('logFile.obser');
OutputMessages  = importdata('logFile.output', ',');

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
%%%%%% NORTH-EAST TEST %%%%%%%%
fig3d=figure;%('Position',[(scrsz(3)/2) 30 (scrsz(3)/2) (scrsz(4)-104)],'Renderer','zbuffer','doublebuffer','on');
rotate3d(fig3d);
axis3d = gca; %get the current axes handle
set(axis3d,'XDir','reverse','ZDir','reverse');
hold(axis3d);
set(fig3d,'CurrentAxes',axis3d);
axis equal; %zlim([-0.1 0.4]); 
grid(axis3d);
xlabel('east (m)'); ylabel('north (m)'); zlabel('depth (m)');
plot3(axis3d,OldNav(:,2), OldNav(:,1), OldNav(:,3), 'r.'); hold on;
lbl_measured = find(Observations(:,1)==2);
plot3(axis3d, Obs(:,2), Obs(:,1), zeros(size(Obs(:,1))), 'bo');
plot3(axis3d, OutputMessages(:,2), OutputMessages(:,1), OutputMessages(:,3), 'g'); axis equal;
legend('dead reckoning','lbl', 'filtered');
title('ekf localisation');
if(EXPORT_IMAGES),
            set(gcf, 'Color', 'none');  
            export_fig north-east.eps -eps -a1;
end

time = cumsum(InputMessages(:,1));
%%%%%% YAW-YAW RATE TEST %%%%%%%%
figure;
subplot(211);
plot(time, (Obs(:,8))*180/pi, 'kx');  hold on; ylabel('yaw [deg]');
plot(time, OutputMessages(:,8) *180/pi, 'r');

plot(time, OutputMessages(:,8)*180/pi+2*sqrt(OutputMessages(:,19)*180/pi),'k-.'); 
plot(time, OutputMessages(:,8)*180/pi-2*sqrt(OutputMessages(:,19)*180/pi),'k-.');

plot(time, (OutputMessages(:,8)-Obs(:,8)+2*pi)*180/pi, 'b.'); grid on;
legend('measured', 'filtered');
title ('compass');

subplot(212);
plot(time, Obs(:,10), 'kx'); hold on; ylabel('yaw rate [rad/s]');
plot(time, OutputMessages(:,10), 'r'); grid on;
plot(time, OutputMessages(:,10)+2*sqrt(OutputMessages(:,21)),'k-.'); 
plot(time, OutputMessages(:,10)-2*sqrt(OutputMessages(:,21)),'k-.');
legend('measured', 'filtered', '2\sigma boundary');
title('gyroscope');
if(EXPORT_IMAGES),
    set(gcf, 'Color', 'none');  
    export_fig yaw-test.eps -eps -a1;
end
%%%%%% SURGE-SWAY RATE TEST %%%%%%%%
figure;
subplot(211);
plot(time, Obs(:,5), 'kx');  hold on;
plot(time, OutputMessages(:,5), 'r');
legend('measured', 'filtered');
title ('surge vel [m/s] (dvl)');

subplot(212);
plot(time, Obs(:,6), 'kx'); hold on;
plot(time, OutputMessages(:,6), 'r');
plot(time, OutputMessages(:,6)+2*sqrt(OutputMessages(:,17)),'k-.'); 
plot(time, OutputMessages(:,6)-2*sqrt(OutputMessages(:,17)),'k-.');
legend('measured', 'filtered');
title(' sway vel [m/s] (dvl)');
if(EXPORT_IMAGES),
    set(gcf, 'Color', 'none');  
    export_fig vel-test.eps -eps -a1;
end

% % % subplot(212);
% % % plot(time, InputMessages(:,9)); title ('sway vel [m/s]');hold on;
% % % plot(time, InputMessages(:,10), 'rx');
% % % plot(time, PredictedState(:,6), 'gx');
% figure; plot(CorrectedP(:,9));
% figure;
% plot(InputMessages(:,12),'m'); hold on;
% plot(InputMessages(:,13),'y');

%close all;