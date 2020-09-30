%% Task 005.A
clear all; clc;

%--- Inputs
%--- Assumptions
T = 10; % lbs
L = 10;  % lbs
plane_weight = 6;

Cd_sensor = 0.04; % unitless
Cd_plane = 0.027; % unitless
A_wing = 5; % ft^2

track_length = 3000; % ft
flight_time = 10*60; % seconds
rho = 0.002377; % slug/ft^3
rho_sensor = 1.05675; % slug/ft^3 --> Selected PC plastic 34 lb/ft^3

%---Governing constants
g_acceleration = 32.17405; %ft/s^2
M3_maxscore = 3;
length_to_diameter = 4;

%------------------ Equations
sensor_diameter = [1/12:0.5/12:85/12]; % ft
sensor_length = length_to_diameter * sensor_diameter;   % ft
num_laps = floor(linspace(1,20,length(sensor_diameter))); % unitless
A_sensor = pi/4 * sensor_diameter.^2; % ft^2
vel_range = track_length .* num_laps / flight_time; % ft/s

Vol_sensorhead = (2/3) * pi * (sensor_diameter./2).^3; % ft^3
Vol_sensorbody = pi * (sensor_diameter./2).^2;  % ft^3
sensor_weight = rho_sensor * (Vol_sensorhead + Vol_sensorbody) * g_acceleration; % lb

D_sensor = zeros(length(sensor_diameter), length(sensor_diameter));
D_plane = zeros(length(sensor_diameter), length(sensor_diameter));
Thrust = zeros(length(sensor_diameter), length(sensor_diameter));

for i=1:length(sensor_diameter)
    for j = 1:length(sensor_diameter)
        
        D_sensor(i,j) = 0.5 * Cd_sensor * pi / 4 * sensor_diameter(1,i)^2 * rho * (vel_range(1,j))^2;
        D_plane(i,j) = 0.5 * Cd_plane * A_wing * rho * (vel_range(1,j))^2;
        Thrust(i,j) = D_plane(i,j) + D_sensor(i,j);
    end
%     M3_our_range(1,i) =  num_laps(1,i) * sensor_length(1,i) * sensor_weight(1,i);
end


%--------% Velocity as function of sensor length %-------%
%------- Refer the deduction part to the attached pdf ------%
X = 0.5 * Cd_plane * A_wing * rho;
Y = (pi/8) * Cd_sensor * rho;
A = flight_time / track_length; % s*lap/ft
for i=1:length(sensor_length)
    max_velocity(1,i)  = sqrt(16*T /(16*X + Y*sensor_length(1,i)^2 ));
    max_laps(1,i) = A * sqrt(16*T /(16*X + Y*sensor_length(1,i)^2 ));
    max_laps_int(1,i) = floor(A * sqrt(16*T /(16*X + Y*sensor_length(1,i)^2 )));
end
%-------------------------------------------------------------%

subplot(3,2,1);
plot(12*sensor_length/4, max_velocity);
title('Velocity vs sensor length');
xlabel('sensor diameter, in');
ylabel('Velocity, ft/s');
xlim([0 10]);

subplot(3,2,2);
plot(12*sensor_length/4, max_laps);
title('# of laps vs sensor length');
xlabel('sensor diameter, in');
ylabel('# of Laps');
xlim([0 10]);

subplot(3,2,3);
plot(12*sensor_length/4, max_laps_int);
title('# of whole laps vs sensor length');
xlabel('sensor diameter, in');
ylabel('# of Whole Laps');
xlim([0 10]);

subplot(3,2,4);
plot(12*sensor_diameter,Thrust(169,:));
title('Thrust vs sensor length');
xlabel('sensor diameter, in');
ylabel('Thrust, lb');
xlim([0 10]);

subplot(3,2,5);
plot(12*sensor_diameter,sensor_weight+plane_weight);
title('Plane weight vs sensor length');
xlabel('sensor diameter, in');
ylabel('Plane Weight, lb');
ylim([0 15]);
