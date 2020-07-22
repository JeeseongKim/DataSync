close all;
clear all;
clc;

%% data 가져오기
fid = fopen('D:\git\SLAM_JSK_original\data_sync\IMU_data_snu03.txt','r');
imu_data = fscanf(fid, '%f %f %f %f %f %f %f %f %f %f %f\n', [11 inf]); %index timestamp acc_x acc_y acc_z angacc_x angacc_y angacc_z mag_x mag_y mag_z

fid_ = fopen('D:\git\SLAM_JSK_original\data_sync\Odo_data_snu03.txt','r');
enc_data = fscanf(fid_, '%f %f %f %f %f \n', [5 inf]); %index timestamp x y theta

imu_data = imu_data';
enc_data = enc_data';

tmp_mean_x = mean(imu_data(1:100,6));
imu_data(:,6) = imu_data(:,6) - tmp_mean_x;
imu_data(:,6) = imu_data(:,6) / 32.8;

tmp_mean_y = mean(imu_data(1:100,7));
imu_data(:,7) = imu_data(:,7) - tmp_mean_y;
imu_data(:,7) = imu_data(:,7) / 32.8;

tmp_mean_z = mean(imu_data(1:100,8));
imu_data(:,8) = imu_data(:,8) - tmp_mean_z;
imu_data(:,8) = imu_data(:,8) / 32.8;

imu_data(:,2) = imu_data(:,2) * 0.001;
enc_data(:,2) = enc_data(:,2) * 0.001;

%%
% 0 에서 1 넘어가는 시점을 시작점으로 맞춤
%snu1: timestamp_ratio = 1.003337;
%snu3: timestamp_ratio = 1.003371;
%snu5: timestamp_ratio = 1.003340;
%snu7: timestamp_ratio = 1.003285;

timestamp_ratio = 1.003371;
imu_data(:,2) = imu_data(:,2) * timestamp_ratio;

imu_size = size(imu_data);
enc_size = size(enc_data);

new_imu_data = [];
new_enc_data = [];

%snu1: 450/94
%snu3: 451/97
%snu5: 413/95
%snu7: 415/99


new_imu_data = imu_data(451:(imu_size(1))-1, :);
new_enc_data = enc_data(97:(enc_size(1))-1, :);

new_imu_size = size(new_imu_data);
new_imu_size = new_imu_size(1);
new_enc_size = size(new_enc_data);
new_enc_size = new_enc_size(1);

%%
dt_imu=[];
dt_imu(1,1) = 0;
for i=2:1:new_imu_size
    dt_imu(i,1) = (new_imu_data(i,2) - new_imu_data(i-1,2));
end

dt_enc=[];
dt_enc(1,1) = 0;
for i=2:1:new_enc_size
    dt_enc(i,1) = (new_enc_data(i,2) - new_enc_data(i-1,2));
end

new_imu_data(:,12) = dt_imu; %dt_imu
new_enc_data(:,6) = dt_enc; %dt_odo

dt_imu_size = size(dt_imu);
dt_enc_size = size(dt_enc);

%% 시작점 기준 dt
for_sync_dt_imu=[];
for_sync_dt_enc=[];

for i=1:1:new_imu_size
    for_sync_dt_imu(i,1) = (new_imu_data(i,2) - new_imu_data(1,2));
end

for i=1:1:new_enc_size
    for_sync_dt_enc(i,1) = (new_enc_data(i,2) - new_enc_data(1,2));
end

%% odo_velocity 구하기
new_enc_data(1,7) = 0; % x방향 속도
new_enc_data(1,8) = 0; % y방향 속도

for i=2:1:new_enc_size
    new_enc_data(i, 7) = (new_enc_data(i,3) - new_enc_data(i-1,3)) / new_enc_data(i,6); % x방향 속도
    new_enc_data(i, 8) = (new_enc_data(i,4) - new_enc_data(i-1,4)) / new_enc_data(i,6); % y방향 속도
end

%% Encoder data를 IMU data에 대응시키기
data_tmp_imu = [];
data_tmp_imu(:,1) = for_sync_dt_imu(:,1); % for_sync_dt_imu
data_tmp_imu_size = size(data_tmp_imu);
data_tmp_imu_size = data_tmp_imu_size(1);

data_tmp_enc = [];
data_tmp_enc(:,1) = for_sync_dt_enc(:,1); % for_sync_dt_enc
data_tmp_enc(:,2) = new_enc_data(:,7); % x방향 속도
data_tmp_enc(:,3) = new_enc_data(:,8); % y방향 속도
data_tmp_enc_size = size(data_tmp_enc);
data_tmp_enc_size = data_tmp_enc_size(1);

sync_matched_data = []; %dt_imu dt_enc x_vel_enc y_vel_enc
sync_matched_data(1,1) = data_tmp_imu(1,1);
sync_matched_data(1,2) = data_tmp_enc(1,1);
sync_matched_data(1,3) = data_tmp_enc(1,2);
sync_matched_data(1,4) = data_tmp_enc(1,3);

j=2;
for i=2:1:data_tmp_imu_size
    if(data_tmp_enc(j,1) > data_tmp_imu(i,1))
        comp1 = abs(data_tmp_enc(j-1,1) - data_tmp_imu(i,1)); 
        comp2 = abs(data_tmp_enc(j,1) - data_tmp_imu(i,1));            

        if(comp1 < comp2) %comp1 제일 작을 경우
            sync_matched_data(i,1) = data_tmp_imu(i,1);
            sync_matched_data(i,2) = data_tmp_enc(j-1,1);
            sync_matched_data(i,3) = data_tmp_enc(j-1,2);
            sync_matched_data(i,4) = data_tmp_enc(j-1,3);

        else %comp2 제일 작을 경우
            sync_matched_data(i,1) = data_tmp_imu(i,1);
            sync_matched_data(i,2) = data_tmp_enc(j,1);
            sync_matched_data(i,3) = data_tmp_enc(j,2);
            sync_matched_data(i,4) = data_tmp_enc(j,3);
        end
        
    else
        if(j==data_tmp_enc_size)
            break;
        end
        j=j+1;
        comp1 = abs(data_tmp_enc(j-1,1) - data_tmp_imu(i,1)); 
        comp2 = abs(data_tmp_enc(j,1) - data_tmp_imu(i,1));            

        if(comp1 < comp2) %comp1 제일 작을 경우
            sync_matched_data(i,1) = data_tmp_imu(i,1);
            sync_matched_data(i,2) = data_tmp_enc(j-1,1);
            sync_matched_data(i,3) = data_tmp_enc(j-1,2);
            sync_matched_data(i,4) = data_tmp_enc(j-1,3);
            
        else %comp2 제일 작을 경우
            sync_matched_data(i,1) = data_tmp_imu(i,1);
            sync_matched_data(i,2) = data_tmp_enc(j,1);
            sync_matched_data(i,3) = data_tmp_enc(j,2);
            sync_matched_data(i,4) = data_tmp_enc(j,3);
        end
        
    end
end

sync_size = size(sync_matched_data);
sync_size = sync_size(1);

%% imu 기준 이동거리 (x,y)
imu_len = []; 
imu_len(1,1) = 0;
imu_len(1,2) = 0;
for i=2:1:sync_size
    imu_len(i,1) = sync_matched_data(i,3) * dt_imu(i,1); %x% imu_len(i-1,1) + sync_matched_data(i,3) * sync_matched_data(i,1); %x
    imu_len(i,2) = sync_matched_data(i,4) * dt_imu(i,1); %imu_len(i-1,2) + sync_matched_data(i,4) * sync_matched_data(i,1); %y
    imu_len(i,3) = sqrt((imu_len(i,1)^2) + (imu_len(i,2)^2)); 
end
%% get theta_z 
theta_xyz = [];
theta_xyz(1,1) = 0; theta_xyz(1,2) = 0; theta_xyz(1,3) = 0;

for i=2:1:new_imu_size
    theta_xyz(i,1) = theta_xyz(i-1,1) + (new_imu_data(i,6) * new_imu_data(i,12));
    theta_xyz(i,2) = theta_xyz(i-1,2) + (new_imu_data(i,7) * new_imu_data(i,12));
    theta_xyz(i,3) = theta_xyz(i-1,3) + (new_imu_data(i,8) * new_imu_data(i,12));
    
    theta_xyz(i,1) = normAngle(theta_xyz(i,1));
    theta_xyz(i,2) = normAngle(theta_xyz(i,2));
    theta_xyz(i,3) = normAngle(theta_xyz(i,3));
end
%% final odometry 
final_3d_odo = [];
final_3d_odo(1,1) = 0; % x odo
final_3d_odo(1,2) = 0; % y odo
final_3d_odo(1,3) = 0; % z odo

for i=2:1:sync_size
    % Lie Method
    w = [theta_xyz(i,1);
         theta_xyz(i,2);
         theta_xyz(i,3);];
     
    w(1)= w(1) * pi() / 180;
    w(2)= w(2) * pi() / 180;
    w(3)= w(3) * pi() / 180;

    w_hat = [0       -w(3)    w(2);
             w(3)     0      -w(1);
            -w(2)     w(1)     0;];

    w_norm = norm(w);

    if (w_norm == 0)
        R = eye(3,3);

    else
        w_w_norm = w_hat/w_norm;
        R = eye(3,3) + w_w_norm * sin(w_norm) + w_w_norm * w_w_norm *(1-cos(w_norm)); %this R = robot to world 
    end

    len_all = [];
    len_all(1,1) = (R(1,1) * imu_len(i,3));
    len_all(1,2) = (R(2,1) * imu_len(i,3));
    len_all(1,3) = (R(3,1) * imu_len(i,3));
    
    final_3d_odo(i,1) = final_3d_odo(i-1,1) + len_all(1,1);
    final_3d_odo(i,2) = final_3d_odo(i-1,2) + len_all(1,2);
    final_3d_odo(i,3) = final_3d_odo(i-1,3) + len_all(1,3);
        
end

%%
figure(1)
x = final_3d_odo(:,1);
y = final_3d_odo(:,2);
z = final_3d_odo(:,3);
plot3(x,y,z)

figure(2)
x_2d  = final_3d_odo(:,1);
y_2d = final_3d_odo(:,2);
plot(x,y)



