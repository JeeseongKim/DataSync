close all;
clear all;
clc;

%% data 가져오기
fid = fopen('D:\git\log\310_2_2\IMU_data.txt','r');
% fid = fopen('D:\git\log\200121_RGB_1\IMU_data.txt','r');
imu_data = fscanf(fid, '%f %f %f %f %f %f %f %f %f %f %f\n', [11 inf]); %index timestamp acc_x acc_y acc_z angacc_x angacc_y angacc_z mag_x mag_y mag_z

fid_ = fopen('D:\git\log\310_2_2\Odo_data.txt','r');
% fid_ = fopen('D:\git\log\200121_RGB_1\Odo_data.txt','r');
enc_data = fscanf(fid_, '%f %f %f %f %f \n', [5 inf]); %index timestamp x y theta

imu_data = imu_data';
enc_data = enc_data';

tmp_mean = mean(imu_data(1:100,8));
imu_data(:,8) = imu_data(:,8) - tmp_mean;
imu_data(:,8) = imu_data(:,8) / 32.8;

imu_data(:,2) = imu_data(:,2) * 0.001;
enc_data(:,2) = enc_data(:,2) * 0.001;

%% 지자계 데이터 처리
%mag offset
min_x = min(imu_data(:,10));
max_x = max(imu_data(:,10));
offset_x = (min_x + max_x) * 0.5;

min_y = min(imu_data(:,11));
max_y = max(imu_data(:,11));
offset_y = (min_y + max_y) * 0.5;

imu_data(:,10) = imu_data(:,10) - offset_x;
imu_data(:,11) = imu_data(:,11) - offset_y;

%%
% 0 에서 1 넘어가는 시점을 시작점으로 맞춤
timestamp_ratio = 1.003351796; % odo(end - start)/ imu(end - start)
imu_data(:,2) = imu_data(:,2) * timestamp_ratio;

imu_size = size(imu_data);
enc_size = size(enc_data);

new_imu_data = [];
new_enc_data = [];

new_imu_data = imu_data(343:(imu_size(1))-1, :);
new_enc_data = enc_data(97:(enc_size(1))-1, :);

new_imu_size = size(new_imu_data);
new_imu_size = new_imu_size(1);
new_enc_size = size(new_enc_data);
new_enc_size = new_enc_size(1);

%%
mag_yaw = [];
for i=1:1:new_imu_size
    mag_x = new_imu_data(i,10);
    mag_y = new_imu_data(i,11);
    
    mag_yaw(i,1) = atan2(mag_y,mag_x) * 180 / pi;
    mag_yaw(i,1) = mag_yaw(i,1) * 180 / pi;    
end

%%
mag_sz = size(mag_yaw);
mag_sz = mag_sz(1);

for i=1:1:mag_sz
    mag_yaw(i,1) = normAngle(mag_yaw(i,1));
end
mag_yaw(:,1) = mag_yaw(:,1) - mag_yaw(1,1);


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

for i=1:1:new_enc_size
    new_enc_data(i,5) = new_enc_data(i,5) * 0.1; %degree 
end

%%
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

%% 이동거리 구하기
dist_enc_sofar = [];
dist_enc_sofar(1,1) = 0;
dist_enc_sofar(1,2) = 0;
dist_enc_sofar(1,3) = new_enc_data(1,5);

% dist_robot=[];
dist_1 = 0;

for i=2:1:new_enc_size
    dist_enc_sofar(i, 1) = (new_enc_data(i,3) - new_enc_data(i-1,3)); % x방향 이동거리
    dist_enc_sofar(i, 2) = (new_enc_data(i,4) - new_enc_data(i-1,4)); % y방향 이동거리
    dist_enc_sofar(i, 3) = new_enc_data(i-1,5); %현재 theta
    
    dist_1 = sqrt(dist_enc_sofar(i, 1) * dist_enc_sofar(i, 1) + dist_enc_sofar(i, 2) * dist_enc_sofar(i, 2))+dist_1;
    
%     dist_robot(i,1) = dist_enc_sofar(i, 1)*cosd(dist_enc_sofar(i, 3)) - dist_enc_sofar(i, 2) * sind(dist_enc_sofar(i, 3));
%     dist_robot(i,2) = dist_enc_sofar(i, 1)*sind(dist_enc_sofar(i, 3)) + dist_enc_sofar(i, 2) * cosd(dist_enc_sofar(i, 3));
%     if (dist_robot(i,1) < dist_robot(i-1,1))
%         dist_1 = -sqrt(dist_enc_sofar(i, 1) * dist_enc_sofar(i, 1) + dist_enc_sofar(i, 2) * dist_enc_sofar(i, 2))+dist_1;
%     else
%         dist_1 = sqrt(dist_enc_sofar(i, 1) * dist_enc_sofar(i, 1) + dist_enc_sofar(i, 2) * dist_enc_sofar(i, 2))+dist_1;
%     end
end

%% Encoder data를 IMU data에 대응시키기

data_tmp_imu = [];
data_tmp_imu(:,1) = new_imu_data(:,1); %image index
data_tmp_imu(:,2) = for_sync_dt_imu(:); % for_sync_dt_imu
data_tmp_imu_size = size(data_tmp_imu);
data_tmp_imu_size = data_tmp_imu_size(1);

data_tmp_enc = [];
data_tmp_enc(:,1) = new_enc_data(:,1); %image index
data_tmp_enc(:,2) = for_sync_dt_enc(:,1); % for_sync_dt_enc
data_tmp_enc(:,3) = new_enc_data(:,7); % x방향 속도
data_tmp_enc(:,4) = new_enc_data(:,8); % y방향 속도
data_tmp_enc_size = size(data_tmp_enc);
data_tmp_enc_size = data_tmp_enc_size(1);

sync_matched_data = []; % image_index dt_imu dt_enc x_vel_enc y_vel_enc mag_yaw
sync_matched_data(1,1) = data_tmp_imu(1,1);
sync_matched_data(1,2) = data_tmp_imu(1,2);
sync_matched_data(1,3) = data_tmp_enc(1,2);
sync_matched_data(1,4) = data_tmp_enc(1,3);
sync_matched_data(1,5) = data_tmp_enc(1,4);
sync_matched_data(1,6) = mag_yaw(1,1);
sync_matched_data(1,7) = new_enc_data(1,5);

j=2;
for i=2:1:data_tmp_imu_size
    if(data_tmp_enc(j,2) > data_tmp_imu(i,2))
        comp1 = abs(data_tmp_enc(j-1,2) - data_tmp_imu(i,2)); 
        comp2 = abs(data_tmp_enc(j,2) - data_tmp_imu(i,2));            

        if(comp1 < comp2) %comp1 제일 작을 경우
            sync_matched_data(i,1) = data_tmp_imu(i,1);
            sync_matched_data(i,2) = data_tmp_imu(i,2);
            sync_matched_data(i,3) = data_tmp_enc(j-1,2);
            sync_matched_data(i,4) = data_tmp_enc(j-1,3);
            sync_matched_data(i,5) = data_tmp_enc(j-1,4);
            sync_matched_data(i,6) = mag_yaw(i,1);
            sync_matched_data(i,7) = new_enc_data(j-1,5);

        else %comp2 제일 작을 경우
            sync_matched_data(i,1) = data_tmp_imu(i,1);
            sync_matched_data(i,2) = data_tmp_imu(i,2);
            sync_matched_data(i,3) = data_tmp_enc(j,2);
            sync_matched_data(i,4) = data_tmp_enc(j,3);
            sync_matched_data(i,5) = data_tmp_enc(j,4);
            sync_matched_data(i,6) = mag_yaw(i,1);
            sync_matched_data(i,7) = new_enc_data(j,5);
        end
        
    else
        if(j==data_tmp_enc_size)
            break;
        end
        j=j+1;
        comp1 = abs(data_tmp_enc(j-1,2) - data_tmp_imu(i,2)); 
        comp2 = abs(data_tmp_enc(j,2) - data_tmp_imu(i,2));            

        if(comp1 < comp2) %comp1 제일 작을 경우
            sync_matched_data(i,1) = data_tmp_imu(i,1);
            sync_matched_data(i,2) = data_tmp_imu(i,2);
            sync_matched_data(i,3) = data_tmp_enc(j-1,2);
            sync_matched_data(i,4) = data_tmp_enc(j-1,3);
            sync_matched_data(i,5) = data_tmp_enc(j-1,4);
            sync_matched_data(i,6) = mag_yaw(i,1);
            sync_matched_data(i,7) = new_enc_data(j-1,5);
            
        else %comp2 제일 작을 경우
            sync_matched_data(i,1) = data_tmp_imu(i,1);
            sync_matched_data(i,2) = data_tmp_imu(i,2);
            sync_matched_data(i,3) = data_tmp_enc(j,2);
            sync_matched_data(i,4) = data_tmp_enc(j,3);
            sync_matched_data(i,5) = data_tmp_enc(j,4);
            sync_matched_data(i,6) = mag_yaw(i,1);
            sync_matched_data(i,7) = new_enc_data(j,5);
        end
        
    end
end

sync_size = size(sync_matched_data);
sync_size = sync_size(1);

%% imu 기준 이동거리 (x,y)
imu_len = []; 
imu_len(1,1) = 0;
imu_len(1,2) = 0;
backOrFront = [];
backOrFront(1,1) = sync_matched_data(1,7);
robot_dist=[];

for i=2:1:sync_size
    imu_len(i,1) = sync_matched_data(i,4) * dt_imu(i,1); 
    imu_len(i,2) = sync_matched_data(i,5) * dt_imu(i,1); 
    backOrFront(i,1) = sync_matched_data(i,7);
    
%     robot_dist(i,1) = imu_len(i,1) * cosd(backOrFront(i,1)) - imu_len(i,2) * sind(backOrFront(i,1));
%     robot_dist(i,2) = imu_len(i,1) * sind(backOrFront(i,1)) + imu_len(i,2) * cosd(backOrFront(i,1));
    robot_dist(i,1) = imu_len(i,1) * cosd(backOrFront(i,1));
    robot_dist(i,2) = imu_len(i,2) * cosd(backOrFront(i,1));
    
    if (robot_dist(i,1) < robot_dist(i-1,1))
        imu_len(i,3) = - sqrt((imu_len(i,1)^2) + (imu_len(i,2)^2)); 
    else
        imu_len(i,3) = sqrt((imu_len(i,1)^2) + (imu_len(i,2)^2)); 
    end
end

%% get theta_z
theta_z = [];
theta_z(1,1) = 0;
for i=2:1:new_imu_size
    theta_z(i,1) = theta_z(i-1,1) + (new_imu_data(i,8) * new_imu_data(i,12));
    theta_z(i,1) = normAngle(theta_z(i,1));
end

%% Final odo
final_2d_odo = [];
final_2d_odo(1,1) = 0; % image index
final_2d_odo(1,2) = 0; % timestamp
final_2d_odo(1,3) = 0; % x odo
final_2d_odo(1,4) = 0; % y odo
final_2d_odo(1,5) = 0; % theta odo
final_2d_odo(1,6) = 0; % mag_theta_z (yaw)

for i=2:1:sync_size
    final_2d_odo(i,1) = sync_matched_data(i,1);
    final_2d_odo(i,2) = sync_matched_data(i,2);
    final_2d_odo(i,3) = final_2d_odo(i-1,3) + (imu_len(i,3)) * cosd(theta_z(i,1)); 
    final_2d_odo(i,4) = final_2d_odo(i-1,4) + (imu_len(i,3)) * sind(theta_z(i,1));

%     final_2d_odo(i,3) = final_2d_odo(i-1,3) + (imu_len(i,3)) * cosd(sync_matched_data(i,6)); 
%     final_2d_odo(i,4) = final_2d_odo(i-1,4) + (imu_len(i,3)) * sind(sync_matched_data(i,6));

    final_2d_odo(i,5) = theta_z(i,1); 
    final_2d_odo(i,6) = sync_matched_data(i,6); 
end

final_sz = size(final_2d_odo);
final_sz = final_sz(1);
final_2d_odo = final_2d_odo(2:final_sz,:);

%% plot
figure(1)
x=[]; y=[];

x = final_2d_odo(:,3);
y = final_2d_odo(:,4);
plot(x,y)

figure(2)
plot(final_2d_odo(:,5))

% figure(3)
% plot(final_2d_odo(:,6))

%% make imu_plus_odo
[final_size, tmp] = size(final_2d_odo);
j=1;

imu_plus_odo=[];
imu_plus_odo(1,1) = final_2d_odo(1,1);
imu_plus_odo(1,2) = final_2d_odo(1,2);
imu_plus_odo(1,3) = final_2d_odo(1,3);
imu_plus_odo(1,4) = final_2d_odo(1,4);
imu_plus_odo(1,5) = final_2d_odo(1,5);
imu_plus_odo(1,6) = final_2d_odo(1,6);

for i=1:1:final_size - 1
    if(final_2d_odo(i,1) < final_2d_odo(i+1,1))
        j=j+1;
        imu_plus_odo(j,1) = final_2d_odo(i+1,1); %image index
        imu_plus_odo(j,2) = final_2d_odo(i+1,2); % timestamp
        imu_plus_odo(j,3) = final_2d_odo(i+1,3); % x
        imu_plus_odo(j,4) = final_2d_odo(i+1,4); % y
        imu_plus_odo(j,5) = final_2d_odo(i+1,5); % theta_z (by imu)
        imu_plus_odo(j,6) = final_2d_odo(i+1,6); % theta_z (by mag)
    end
end

imu_plus_odo(:,5) =imu_plus_odo(:,5) - imu_plus_odo(1,5);
imu_plus_odo(:,6) =imu_plus_odo(:,6) - imu_plus_odo(1,6);

% figure(3)
% plot(imu_plus_odo(:,2), imu_plus_odo(:,3));

fid = fopen('imu_plus_odo.txt','w');
for i = 1:length(imu_plus_odo)
    fprintf(fid,'%f\t%f\t%f\t%f\t%f\t%f\n',imu_plus_odo(i,1), imu_plus_odo(i,2), imu_plus_odo(i,3), imu_plus_odo(i,4), imu_plus_odo(i,5), imu_plus_odo(i,6));
end

fclose(fid);

%% SO error

start_x = final_2d_odo(1,3);
start_y = final_2d_odo(1,4);
start_pt = sqrt(start_x*start_x + start_y * start_y);

end_x = final_2d_odo(final_size,3);
end_y = final_2d_odo(final_size,4);
end_pt = sqrt(end_x*end_x + end_y * end_y);

SOerror = end_pt - start_pt;
SOerror = SOerror/1000

