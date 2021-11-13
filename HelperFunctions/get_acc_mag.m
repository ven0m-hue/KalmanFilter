% NED frame of reference.
function [roll, pitch, yaw] = get_acc_mag(acce_data, mag_data)
    ax = acce_data(1)/norm(acce_data);
    ay = acce_data(2)/norm(acce_data);
    az = acce_data(3)/norm(acce_data);
    roll = atan2(-ay, -az);
    pitch = atan2(ax, sqrt(ay*ay + az*az));
    
	yaw = yawCorrection(mag_data, roll, pitch);
    
    if(yaw < 0)
        yaw = yaw+360*pi/180;
    end
end