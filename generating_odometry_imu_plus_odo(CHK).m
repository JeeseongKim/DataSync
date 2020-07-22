close all
clear all
clc

%% load
buffer = load('191125/snu07/imu_data');
imu_data = buffer.imu_data;

buffer = load('191125/snu07/odo_data');
odo_data = buffer.odo_data;

%% initial
imu_ro_st = 4159;
imu_ro_en = 1288998;

odo_ro_st = 5718;
odo_ro_en = 1294779;

% timing 비율
ratio_odo_over_imu = (odo_ro_en - odo_ro_st)/(imu_ro_en - imu_ro_st);

% % start 시점
% imu_st = 1000;
% imu_en = 1000;

%% generating odometry
result = [];
x = 0;
y = 0;
[h_imu, w_imu] = size(imu_data);
[h_odo, w_odo] = size(odo_data);
j_odo = 1;
for i=1:h_imu
    if(imu_data(i,2)>(imu_ro_st - 5))&&(imu_data(i,2)<(imu_ro_en+20))
        time_odo = (imu_data(i,2) - imu_ro_st)*ratio_odo_over_imu + odo_ro_st;
        for j=j_odo:h_odo
            if(odo_data(j,2) > time_odo)
                j_odo=j;
                dt = odo_data(j,2)-odo_data(j-1,2);
                %vx = sqrt((odo_data(j,3)-odo_data(j-1,3))^2+(odo_data(j,4)-odo_data(j-1,4))^2)/dt;
                rel_pose = minus_operator(odo_data(j-1,3:5), odo_data(j,3:5));
                vx = sign(rel_pose(1))*sqrt((odo_data(j,3)-odo_data(j-1,3))^2+(odo_data(j,4)-odo_data(j-1,4))^2)/dt;
                break;
            end
        end
        dt2 = imu_data(i,2) - imu_data(i-1,2);
        x = x + cosd(imu_data(i,4))*vx*dt2;
        y = y + sind(imu_data(i,4))*vx*dt2;
        
        result = [result; imu_data(i,1:2), x, y, imu_data(i,4), imu_data(i,7)];
    end
    i
end

imu_odo_data=result;
save('imu_odo_data','imu_odo_data')