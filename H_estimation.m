%Observer using Kalman filter based Height estimation. 
%Sensors fused are Barometer and Accel z axis data 
%Future extention can be done by adding the GPS data.
%For simulation, get the pressure sensor raw data
%Extract the raw H parameter 
%Build the State Space model for the H estimation 

%TO DO's
%Tilt correction 

%% Function to obtain the raw data from the pressure sensor 
%Obtain the raw data from the pressure sensor and accel_z
%BMP280 = load('PressureAlt.mat');
BMP280 = load('H_data/Height_ESTO.mat');
BMP280.Height_Pressure = BMP280.Book1; BMP280 = rmfield(BMP280, "Book1");
BMP280.Height_Pressure = table2array(BMP280.Height_Pressure);
Length = round(size(BMP280.Height_Pressure(:,1)));

%Initializers
Pressure_Z = zeros;
height_p = zeros;
velocity_p = zeros;
Accel_Z = zeros;
az = zeros;
estimate_height = zeros;
estimate_velocity = zeros;
roll = zeros;
pitch = zeros;
Time = zeros;

%Time 
T = 1; 
Time(1) = T;
%Sampling period
Ts = 0.1;

P0 = BMP280.Height_Pressure(1,1);

for i=1:Length(1)
   
   %Extract the raw pressure data 
   %Pressure_Z(i) = mean(BMP280.PressureAccelData(i,1));
   if(i > 1)
       %Sample time 
    %   Time(1) = Ts; %for now 
        Time(i) = Time(i-1) + T;
       %P0 = 1008; %hpa
   %else
    end
   %P = Pressure_Z(i);
   %Convert the pressure to height equation
   %height_p(i) = ((Ts + 273.15)/0.0065)*((1 - (BMP280.Height_Pressure(i,1)/BMP280.Height_Pressure(1,1))^(0.1903))); 
   Pp = BMP280.Height_Pressure(i,1);
   height_p(i) = 44330 * (1.0 - (Pp / P0)^ 0.1903);
   %Also obtain the veloctiy from the barometer
   %height_p(i) = mean(BMP280.Height_Pressure(i,1));
   velocity_p(i) = (height_p(i) - height_p(1))/Time(i);

   Accel_Z(i) = mean(BMP280.Height_Pressure(i,2));
   az(i) = Accel_Z(i);



% Tilt correction
% Import the estimated roll and pithc from the EKF 2DOF
% roll(i) = deg2rad(Roll_H(i));
% pitch(i) = deg2rad(Pitch_H(i));
% 
% get roll and pitch for tilt correction
% x = sin(roll(i));
% y = sin(pitch(i));
% Z = sqrt(x^2 + y^2);
% accel_corr = sqrt(1 - Z^2);
% az(i) = az(i)- accel_corr;

end

%% Sensor Fusion 
%Initialization 
Pp = eye(2);
%
measurement_noise = 1e-1 * ones(1,2);
R = diag(measurement_noise);
%
process_noise = 1e-3 * ones(1,2);
Q = process_noise * [Ts^4/4, Ts^3/2;
     Ts^3/2, Ts^2];
%Inital state
Xp = [ 0; 0];  %assumptionaka starting ground 


%State space model 
%State transisition matrix 
A = [1 Ts;
     0 1];
%Input Matrix
B = [Ts^2/2; Ts];
%State Observer
C = [1 0;
     0 0];
 
for i=1:Length 

   %Estimated State update equation 
   X = A*Xp + B*az(i);
   %Predicted State update equation 
   Pp = A*Pp*A' + Q;

   %Measurement model
   Y = [ height_p(i); 0];
   %Corrector equation 
   S = Y - C*X;  %converge to zero
   %Kalman Gain 
   %Sp = C*Pp*C' + R;
   Sp = 1/(Pp(1,1) + R(1,1));
   K = Pp*C*Sp;

   %Update state equation 
   Xp = Xp + K*S;
   %Update Predicted equation
   Pp = (eye(2) - K*C)*Pp;

   %Output
   estimate_height(i) = Xp(1);
   estimate_velocity(i) = Xp(2);
   
end

%% Plots 
figure;
plot(Time, height_p, Time, estimate_height);
legend('Height_Baro', 'Height_KF', 'FontSize', 10);
xlabel('t/s', 'FontSize', 20);
ylabel('Height', 'FontSize', 20);
title('Height Estimation', 'FontSize', 20);

figure(2);
plot(Time, velocity_p, Time, estimate_velocity);
legend('Velocity_Baro', 'Velocity_KF', 'FontSize', 10);
xlabel('t/s', 'FontSize', 10);
ylabel('Velocity', 'FontSize', 10);
title('Veloctiy Estimation', 'FontSize', 20);
%%END








