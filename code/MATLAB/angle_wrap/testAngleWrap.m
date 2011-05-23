function testAngleWrap
close all; clear all; clc;
EXPORT_FIGURES = 1;
Observations   = csvread('logFile.obser');
InputMessages  =  importdata('logFile.input',  ',');
OutputMessages  = importdata('logFile.output', ',');
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
ANGLE1  = Obs(:,8)*180/pi;
ANGLE2  = OutputMessages(:,8)*180/pi;
%%%%%% YAW-YAW RATE TEST %%%%%%%%
figure;
plot(time, ANGLE1, 'k.');  grid on; ylabel('angle 1 [deg]');
if (EXPORT_FIGURES),
    set(gcf, 'Color', 'none');  
    export_fig angle1.eps -eps -a1;
end
figure;
plot(time, ANGLE2, 'b.'); grid on; ylabel('angle 2 [deg]');
if (EXPORT_FIGURES),
    set(gcf, 'Color', 'none');  
    export_fig angle2.eps -eps -a1;
end
figure;
plot(time, ANGLE1-ANGLE2, 'g.'); grid on; ylabel('(angle 1 - angle 2) [deg]');
if (EXPORT_FIGURES),
    set(gcf, 'Color', 'none');  
    export_fig angle1-2.eps -eps -a1;
end
figure;
plot(time, angleWrap180(ANGLE1-ANGLE2), 'r.'); grid on; ylabel('angle wrap180(angle 1 - angle 2) [deg]');
if (EXPORT_FIGURES),
    set(gcf, 'Color', 'none');  
    export_fig wrap180angle1-2.eps -eps -a1;
end
figure;
plot(time, angleWrap360(ANGLE1-ANGLE2), 'g.'); grid on; ylabel('angle wrap360(angle 1 - angle 2) [deg]');
if (EXPORT_FIGURES),
    set(gcf, 'Color', 'none');  
    export_fig wrap360angle1-2.eps -eps -a1;
end
figure;
plot(time, angleWrap360(ANGLE1-ANGLE2), 'go'); grid on; ylabel('angle wrap360(angle 1 - angle 2) [deg]');
hold on;
plot(time, angleWrap180(ANGLE1-ANGLE2), 'r.');
legend('0,2\pi wrap', '-\pi,+\pi wrap');
if (EXPORT_FIGURES),
    set(gcf, 'Color', 'none');  
    export_fig wrapangle1-2.eps -eps -a1;
end
disp('end...');
close all;
end
function ang2 = angleWrap360(ang1)
    ang2 = ang1;
    for i = 1 : length(ang2),
        while ang2(i)<0,
            ang2(i) = ang2(i) + 360;
        end
        while ang2(i)>=360,
            ang2(i) = ang2(i) - 360;
        end
    end
end
function ang2 = angleWrap180(ang1)
    ang2 = ang1;
    for i = 1 : length(ang2),
        while ang2(i)<-180,
            ang2(i) = ang2(i) + 360;
        end
        while ang2(i)>=180,
            ang2(i) = ang2(i) - 360;
        end
    end
end