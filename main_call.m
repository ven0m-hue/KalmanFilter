% Quaternion based Extended Kalman Filter using MPU 9250 
% Goals : State estimation, narrow down the uncertanity by fusing the
% sensor data 
% Create a function that takes in the IMU raw data and sampling frequency
% along with the other neccessary inputs. Will explain in the future.
% Whihc spits out the roll pitch yaw.
% This then is sent to simulink function block, whihc inturn goes into the
% simsacpe model.

%% 
% Includes 
addpath('utils');
addpath('datafiles');
%% Read the logged data 

%Data read from the Hardware
%Uncomment when using the hardware 

MPU9250 = table2array(MPU9250data);
sensor_data = MPU9250;
%Data read from the Mobile sensor
%Uncomment when using the Mobile sensor 
%F1 = load('90.mat');
%sensor_data = F1;
data_num=round(size(sensor_data,1)); % get the length 
L = data_num - 2; % matrix length(row) --> drone data_log: 7600

    X_ACCL = zeros; 
    Y_ACCL = zeros;
    Z_ACCL = zeros;
    X_GYRO = zeros;
    Y_GYRO = zeros;
    Z_GYRO = zeros;
    X_MAG  = zeros;
    Y_MAG  = zeros;
    Z_MAG  = zeros;
  
%%
%This block is only if the data is logged in from the mobile device. Uncomment when using this.   
% j=0;
%     for i = 1:L
%     j = j+1;
%     T = timetable2table(F1.AngularVelocity);    
%     X_GYRO(j,1) = T.X(i);
%     Y_GYRO(j,1) = T.Y(i);
%     Z_GYRO(j,1) = T.Z(i);
% 
%     T = timetable2table(F1.Acceleration);    
%     X_ACCL(j,1) = T.X(i);
%     Y_ACCL(j,1) = T.Y(i);
%     Z_ACCL(j,1) = T.Z(i);
% 
%     T = timetable2table(F1.MagneticField);   
%     X_MAG(j,1) = T.X(i);
%     Y_MAG(j,1) = T.Y(i);
%     Z_MAG(j,1) = T.Z(i);
%     end
    
%    
%%
%This block is only if the data is logged in from the Hardware device. Uncomment when using this.   
j = 0;
for i=3:data_num  % for time being 
    
    j=j+1;
    % just calling the logged data and placing it in a matrix 
    X_ACCL(j,1) = mean(sensor_data(i,1));  % Depends on how datas are logged 
    Y_ACCL(j,1) = mean(sensor_data(i,2));
    Z_ACCL(j,1) = -mean(sensor_data(i,3));
    X_GYRO(j,1) = mean(sensor_data(i,4));  %
    Y_GYRO(j,1) = mean(sensor_data(i,5));
    Z_GYRO(j,1) = mean(sensor_data(i,6));
    X_MAG(j,1) = mean(sensor_data(i,7));
    Y_MAG(j,1) = mean(sensor_data(i,8));
    Z_MAG(j,1) = mean(sensor_data(i,9));

end

%init()

Time = zeros(L,1);
Time(1,1) = 0.007; % Sampling Period (Real)   
gyro_bias = [ -3.1; -3.1; 0.];  % Gyro bias
% Organize 
%Time = load('Time.mat');
acc = [X_ACCL, Y_ACCL, Z_ACCL];  % Accel data

gyro = [X_GYRO - gyro_bias(1), Y_GYRO - gyro_bias(2), Z_GYRO - gyro_bias(3)]; % Gyro data


mag = [X_MAG, Y_MAG, Z_MAG]; % Mag data for Yaw 


% Call the function 
[acc_roll, acc_pitch, mag_yaw, estimate_roll, estimate_pitch, estimate_yaw,Time] = FusionKalman(acc,gyro_bias,gyro,mag,Time,L);
%%
%Plots
%    %ROLL
%    figure;
%    plot(Time,acc_roll,Time,estimate_roll);
%    legend('roll-acc','roll-ekf','FontSize',10);
%    xlabel('t / s','FontSize',20)
%    ylabel('roll','FontSize',20)
%    title('roll','FontSize',20);
%    %PITCH 
%    figure;
%    plot(Time,acc_pitch,Time,estimate_pitch);
%    legend('pitch-acc','pitch-ekf','FontSize',10);
%    xlabel('t / s','FontSize',20)
%    ylabel('pitch','FontSize',20)
%    title('pitch','FontSize',20);
   plot(Time,acc(1),Time,Z_GYRO);
   legend('acc','gyro','FontSize',10);
   xlabel('t / s','FontSize',20)
   ylabel('roll','FontSize',20)
   title('roll','FontSize',20);
   shg
  %YAW
%   figure(3);
%   plot(Time,mag_yaw,Time,360 - estimate_yaw);
%   legend('yaw-mag','yaw-ekf','FontSize',10);
%   xlabel('t / s','FontSize',20)
%   ylabel('yaw','FontSize',20)
%   title('yaw','FontSize',20);

% end 

%%
%For Simulink only
acc_cat = cat(2,Time,acc); % concat with time for simulink
gyro_cat = cat(2,Time,gyro); % concat with time for simulink
mag_cat = cat(2,Time,mag);% concat with time for simulink

roll_cat = cat(2,Time,estimate_roll);
pitch_cat = cat(2,Time,estimate_pitch);
yaw_cat = cat(2,Time,estimate_yaw);

%end
save main_call.mat



