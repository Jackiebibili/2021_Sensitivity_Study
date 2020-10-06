%% Sensitivity Study on Mission 3 Score Multiplier %%

% Preparing Constants
Cd_sensor = 0.04; % unitless
Cd_plane = 0.027; % unitless
A_wing = 5; % ft^2

track_length = 3000; % ft
flight_time = 10*60; % seconds
rho = 0.002377; % slug/ft^3
rho_sensor = 1.05675; % slug/ft^3 --> Selected PC plastic 34 lb/ft^3

X = 0.5 * Cd_plane * A_wing * rho;
Y = (pi/8) * Cd_sensor * rho;
A = flight_time / track_length; % s*laps/ft

% Baseline variables
Mp = 6;                     %takeoff weight of the plane except the sensor
Ms = 1.5;                   %weight of the sensor
d  = 2/12;                  %diameter of the sensor
len = 8/12;                 %length of the sensor

d_span  = [-d/2:d/100:d/2];
sw_span = [-Ms/2:Ms/100:Ms/2];
pw_span = [-Mp/2:Mp/100:Mp/2];
sl_span = [-len/2:len/100:len/2];

vel = 85;                   %velocity, ft/s
T = 10;                     %thrust, lb
llap = 3000;                %circumference of track, ft/lap
FT = 600;                   %flight time,  sec

% M3 score multiplier baseline at the baseline of sensor diameter %
% Baseline score multiplier
num_laps_base = floor(1/(llap * (Mp + Ms)) * (T * vel * FT - (X + Y * d^2)/3 * vel^3 * FT));
M3_multi_base = Ms * 4 * d * num_laps_base;

% vary sensor diameter
for i=1:length(d_span)
    num_laps_calc = floor(1/(llap * (Mp + Ms)) * (T * vel * FT - (X + Y * (d + d_span(1, i))^2)/3 * vel^3 * FT));  
    M3_multi_1(1,i) = Ms * 4 * (d + d_span(1,i)) * num_laps_calc;
    M3_multi_diameter_ratio(1,i) =  (M3_multi_1(1,i) - M3_multi_base) / M3_multi_base;
end

%vary sensor weight
for i=1:length(sw_span)
     num_laps_calc = floor(1/(llap * (Mp + (Ms + sw_span(1, i)))) * (T * vel * FT - (X + Y * d^2)/3 * vel^3 * FT)); 
     M3_multi_2(1,i) = (Ms + sw_span(1,i)) * 4 * d * num_laps_calc;
     M3_multi_sensor_weight_ratio(1,i) =  (M3_multi_2(1,i) - M3_multi_base) / M3_multi_base;
end

for i=1:length(pw_span)
     num_laps_calc = floor(1/(llap * ((Mp + pw_span(1, i)) + Ms)) * (T * vel * FT - (X + Y * d^2)/3 * vel^3 * FT));
     M3_multi_3(1,i) = Ms * 4 * d * num_laps_calc;
     M3_multi_plane_weight_ratio(1,i) = (M3_multi_3(1,i) - M3_multi_base) / M3_multi_base;
end

for i=1:length(sl_span)
     num_laps_calc = floor(1/(llap * (Mp + Ms)) * (T * vel * FT - (X + Y * d^2)/3 * vel^3 * FT));
     M3_multi_4(1,i) = Ms * (len + sl_span(1, i)) * num_laps_calc;
     M3_multi_sensor_length_ratio(1,i) = (M3_multi_4(1,i) - M3_multi_base) / M3_multi_base;
end


subplot(2,1,1);
plot(100 * d_span(1,:) / d, 100 * M3_multi_diameter_ratio(1,:), 'DisplayName', 'M3-sensor-diameter');
hold on;
plot(100 * sw_span(1,:) / Ms, 100 * M3_multi_sensor_weight_ratio(1,:), 'DisplayName', 'M3-sensor-weight');
hold on;
plot(100 * pw_span(1,:) / Mp, 100 * M3_multi_plane_weight_ratio(1,:), 'DisplayName', 'M3-plane-weight');
hold on;
plot(100 * sl_span(1,:) / len, 100 * M3_multi_sensor_length_ratio(1,:), 'DisplayName', 'M3-sensor-length');
hold off;
legend('show');
title('M3-multiplier percent change vs Parameters');
xlabel('Percent change in parameters');
ylabel('M3_multiplier percent change');
xlim([-50 50]);



