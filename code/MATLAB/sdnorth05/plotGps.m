close all; clear all; clc;

EXPORT_IMAGES = 1;

InputMessages  =  importdata('logFile.input', ',');
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

LENGTH = floor(1.0 * size(OldNav,1));
GPSnav = OldNav(:,12:13);
INDEXES_o = find(abs(OldNav(1:LENGTH,5))<0.05);
INDEXES_x = find(abs(OldNav(1:LENGTH,5))>0.8);
INDEXES = [];
%INDEXES = [INDEXES; INDEXES_o];
INDEXES = [INDEXES; INDEXES_x];
%%%%%% NORTH-EAST TEST %%%%%%%%
% % % % figure;%('Position',[(scrsz(3)/2) 30 (scrsz(3)/2) (scrsz(4)-104)],'Renderer','zbuffer','doublebuffer','on');
% % % % plot3(OldNav(:,2), OldNav(:,1), OldNav(:,3), 'y-.'); hold on; 
% % % % plot3(OldNav(:,2), OldNav(:,1), OldNav(:,3), 'r.');
% % % % set(gcf, 'Position', get(0,'Screensize')); % Maximize figure.
% % % % set(gca,'ZDir','reverse');
% % % % plot3(Obs(:,2), Obs(:,1), OldNav(:,3), 'bo');
% % % % plot3(OutputMessages(:,2), OutputMessages(:,1), OutputMessages(:,3), 'g', 'LineWidth', 3); hold on;
% % % % rotate3d(gcf);
% % % % xlabel('east [m]');  ylabel('north [m]'); zlabel('depth [m]');
% % % % legend('rejected outlier', 'median&dead recon', 'lbl', 'ekf');

figure;
plot(OldNav(:,13), OldNav(:,12), 'ko'); grid on; axis equal;
xlabel('east [m]');  ylabel('north [m]'); 
if(EXPORT_IMAGES),
            set(gcf, 'Color', 'none');  
            export_fig gps-signal.eps -eps -a1;
end