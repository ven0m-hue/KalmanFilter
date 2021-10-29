function [acc_roll, acc_pitch, mag_yaw, estimate_roll, estimate_pitch, estimate_yaw,Time] = FusionKalman(acc,gyro_bias,gyro,mag,Time,L)

rad2deg = 180/pi;
deg2rad = pi/180;

% inits()
    % Process Covariance Mat 
    w_process_noise = 1e-6 * ones(1, 4);
    w_bias_process_noise = 1e-8 * ones(1,3);
    Q = diag([w_process_noise, w_bias_process_noise]);
    
    % Measurement Covariance Mat
    a_measure_noise = 1e-1 * ones(1, 3);
    R = diag(a_measure_noise);
    
    % Estimate Covariance Mat 
    p1 = 1 * ones(1, 4);
    p2 = 1 * ones(1, 3);
    P = diag([p1, p2]);
    
    % Although not part of Klaman init, but for better initialization.
    % init one set of euler angel
    [roll_init, pitch_init, yaw_init] = acc2euler(acc(1, :), mag(1, :));
    roll_init1 = roll_init * rad2deg;
    pitch_init1 = pitch_init * rad2deg;
    yaw_init1 = 360 - yaw_init * rad2deg;
    disp("Euler's 1st set of RPY");
    disp(roll_init1);
    disp(pitch_init1);
    disp(yaw_init1);
    
    % init quat with the 1st set of obtained euler 
    q_init = zeros(4, 1);
    [q_init(1), q_init(2), q_init(3), q_init(4)]= euler2quat(roll_init, pitch_init, yaw_init);
    disp("euler initialized..");
    disp(q_init);
    [roll_init, pitch_init, yaw_init] = quat2euler(q_init);
    disp("Quaternion initialized..");
    disp(roll_init);
    disp(pitch_init);
    disp(360 - yaw_init);
    
    % init the state vector 
    
    state_vector = [q_init; gyro_bias];
    
    % Measurement matrix 
    Z = zeros(L, 3);
    
    %Pre-initialize the output 
    estimate_roll = zeros(L, 1);
    acc_roll = zeros(L, 1);
    estimate_pitch = zeros(L, 1);
    acc_pitch = zeros(L, 1);
      
    Q1 = diag([1e-6, 1e-8]);
    R1 = 0.1;
    
    Z1 = zeros(L);
    estimate_yaw = zeros(L, 1);
    mag_yaw = zeros(L, 1);
 
    %% This is where the magic happens
    for k=1:L  
    Ts = 0.00001; % fake sampling freq
    T = 0.007;     % actual sampling freq
    
    if(k > 1)
        Time(k, 1) = Time(k-1, 1) + T;
    end
    
     %% Calcualte the RYP directily using the raw data 
    [acc_roll(k, 1), acc_pitch(k, 1), mag_yaw(k, 1)] = acc2euler(acc(k, :), mag(k,:));
    acc_roll(k) = acc_roll(k, 1) * rad2deg;
    acc_pitch(k) = acc_pitch(k, 1) * rad2deg;
    mag_yaw(k) = 360 - mag_yaw(k, 1) * rad2deg;
    
    % Normalize the vectors   
    Z(k, :) = acc(k, :) / norm(acc(k, :));
    %end 
    %% Process block 
    % modelling the gyro bias 
    gyro_x_bias_current = state_vector(5);
    gyro_y_bias_current = state_vector(6);
    gyro_z_bias_current = state_vector(7);
    
    % True values, according to our initial assumtion  
    w_truth = [gyro(k, 1)-gyro_x_bias_current, gyro(k, 2)-gyro_y_bias_current, gyro(k, 3)-gyro_z_bias_current];
    
    omega = [0, -w_truth(1), -w_truth(2), -w_truth(3);
                w_truth(1), 0, w_truth(3), -w_truth(2);
                w_truth(2), -w_truth(3), 0, w_truth(1);
                w_truth(3), w_truth(2), -w_truth(1), 0];
    
    % Quaternion initialization [q0 q1 q2 q3 bp bq br]        
    quat = [state_vector(1); state_vector(2); state_vector(3); state_vector(4)];
    
    q_new = quat + 0.5 * Ts * omega * quat;   
    
    w_bias_new = [gyro_x_bias_current; gyro_y_bias_current; gyro_z_bias_current];
    next_state_vector = [q_new; w_bias_new];

    %Normalize 
    next_state_vector(1:4) = next_state_vector(1:4) / norm(next_state_vector(1:4));
    
    % State trainsistion vector 
    
    line = [quat(2), quat(3), quat(4);
            -quat(1), quat(4), -quat(3);
            -quat(4), -quat(1), quat(2);
            quat(3), -quat(2), -quat(1)];
        
    % Jacobian to liearize the model     
    Ak = [omega, line;
          zeros(3, 7)];
    % Linearized state transistion model   
    F = eye(7) + 0.5 * Ts * Ak;
    
    %Computing the error covariance martrix 
    P_next = F * P * F' + Q;
    
    %% Fusion block 
    
    % Jacobian Observation Matrix 
    % For now only accel data is considered, mag data are extracted
    % seperatley 
    
    Hk = 2 * [next_state_vector(3), -next_state_vector(4), next_state_vector(1), -next_state_vector(2);
            -next_state_vector(2), -next_state_vector(1), -next_state_vector(4), -next_state_vector(3);
            -next_state_vector(1), next_state_vector(2), next_state_vector(3), -next_state_vector(4)];
    H = [Hk, zeros(3)];   
   
    % Computing the KALMAN GAIN
    %Auxillary
    SP = H * P_next * H' + R;
    SP_inv = SP^(-1);
    
    %Kalman gain 
    K = P_next * H'* SP_inv;
    %
    
    % re-e
    X_ = next_state_vector(1:4);
    
    % Observation model
    hk = [-2*(X_(2)*X_(4)-X_(1)*X_(3));
          -2*(X_(1)*X_(2)+X_(3)*X_(4));
          -(X_(1)^2+X_(4)^2-X_(2)^2-X_(3)^2)];
    
    % error variance 
    S = Z(k, :)' - hk;
    
    % Predicted state matrix 
    state_vector = next_state_vector + K * S;
    %Normalize 
    state_vector(1:4) = state_vector(1:4) / norm(state_vector(1:4));
    
    % Update predicted error co-vairnace matrix 
    P = (eye(7) - K * H) * P_next;
    
   %% extract quaternions and convert them to RPY (NED)
    estimate_q = [state_vector(1), state_vector(2), state_vector(3), state_vector(4)];
    [estimate_roll(k, 1), estimate_pitch(k, 1), estimate_yaw(k, 1)] = quat2euler(estimate_q);
    
    
    %% 
    % 1-D model for the YAW alone for better approximation
    % Mag model to compute the better yaw estimation 
    % Where, yaw initialization 
    %X1 = [yaw_init*deg2rad;0];
    %Q1 = diag([1e-6, 1e-8]);
    %R1 = 0.1;
    
    %Z1 = zeros(L);
    %estimate_yaw = zeros(L, 1);
    %mag_yaw = zeros(L, 1);
    A = [1,  -Ts;
         0,  1];
     
    B = [Ts;0];
    %Since yaw motion is concerned with the respect to Z-axis
    w_z = gyro(k, 3);
    
    H1 = [1,0];
    
    I = eye(2);
  
    X_next = A*X1 + B*w_z;
    %
    P1_next = A*P1*A' + Q1;
    % Refined roll and pitch to find the yaw.
    Z1(k) = getYaw(mag(k,:), estimate_roll(k, 1)*deg2rad, estimate_pitch(k, 1)*deg2rad);
    
    %
    S1 = Z1(k) - H1*X_next;
    
    % 
    SP1 = H1 * P1_next * H1' + R1;
    K1 = P1_next * H1' * SP1^(-1);
    
    % update state 
    X1 = X1 + K1*S1;
    
    % Update co-variance mat 
    P1 = (I - K1*H1)*P1_next;
    
    estimate_yaw(k, 1) = X1(1)*rad2deg;
    
    end
    
    
end