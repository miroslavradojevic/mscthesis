close all; clear all; clc;

EXPORT_IMAGES = 1;

InputMessages   = importdata('logFile.input', ',');
Observations    = csvread('logFile.obser');
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

LENGTH = floor(1.0 * size(OldNav,1));

%%%%%% NORTH-EAST TEST %%%%%%%%
figure;
plot(OldNav(1:LENGTH,2), OldNav(1:LENGTH,1), 'r.'); 
hold on; axis equal; grid on;
plot(OldNav(1:LENGTH,13), OldNav(1:LENGTH,12), 'bo');
plot(OutputMessages(1:LENGTH,2), OutputMessages(1:LENGTH,1), 'g', 'LineWidth', 2);
xlabel('east [m]'); ylabel('north [m]'); 
legend('dead recon', 'lbl', 'ekf');
legend('Location', 'NorthWest');
%set(gcf, 'Position', get(0,'Screensize')); % Maximize figure.
if(EXPORT_IMAGES),
            set(gcf, 'Color', 'none');  
            export_fig spiral2d.eps -eps -a1;
end
time = cumsum(InputMessages(:,1));
%%%%%% DEPTH TEST %%%%%%%%
START_FROM = 0.3*LENGTH;
figure;
subplot(211);
plot(time(START_FROM:LENGTH), Obs(START_FROM:LENGTH,3), 'kx'); hold on;
xlim( [time(floor(START_FROM)) time(floor(LENGTH)) ]);
plot(time(START_FROM:LENGTH), OutputMessages(START_FROM:LENGTH,3), 'r','LineWidth', 2.5);
%plot(time, OutputMessages(:,3)+2*sqrt(OutputMessages(:,14)),'k-.'); 
%plot(time, OutputMessages(:,3)-2*sqrt(OutputMessages(:,14)),'k-.');
legend('measured', 'filtered');
ylabel('depth [m]');
xlabel('time [s]');
subplot(212);
plot(time(START_FROM:LENGTH), Obs(START_FROM:LENGTH,7), 'kx'); hold on;

plot(time(START_FROM:LENGTH), OutputMessages(START_FROM:LENGTH,7), 'r','LineWidth', 2.5);
plot(time(START_FROM:LENGTH), OutputMessages(START_FROM:LENGTH,7)+2*sqrt(OutputMessages(START_FROM:LENGTH,18)),'k-.'); 
plot(time(START_FROM:LENGTH), OutputMessages(START_FROM:LENGTH,7)-2*sqrt(OutputMessages(START_FROM:LENGTH,18)),'k-.');
legend('Location', 'SouthEast');

legend('measured', 'filtered', '2\sigma boundary');
xlim( [time(floor(START_FROM)) time(floor(LENGTH)) ]);
ylabel('heave vel [m/s]');
xlabel('time [s]');
if(EXPORT_IMAGES),
     set(gcf, 'color', 'none');  
     export_fig spiral-depth.eps -eps -a1;
end