filename = 'C:/Users/Nick/Documents/Thesis/code/cpp/ardrone_slam/ardrone_slam/dataset/031/output.yaml';

[data_alt, data_or, data_accel, data_vel] = loadDataset(filename);

data2_alt = zeros(1,2);
data2_or  = zeros(1,4);
data2_accel=zeros(1,4);
data2_vel = zeros(1,4);

%filename = 'C:/Users/Nick/Documents/Thesis/code/cpp/dataset_collector/dataset_collector/dataset/005/output.yaml';

%[data2_alt, data2_or, data2_accel, data2_vel] = loadDataset(filename);


%%


% ALT
figure();
plot(data_alt(:,1), data_alt(:,2));
hold on
plot(data2_alt(:,1), data2_alt(:,2), 'Color', 'red');
title('Altitude (mm)');



% OR
yrange = [-300, 300];
figure();
subplot(3,1,1), plot(data_or(:,1), data_or(:,2)), hold on, plot(data2_or(:,1), data2_or(:,2), 'Color', 'red');
ylim(yrange);
title('X orientation (milli-deg)');


% y
subplot(3,1,2), plot(data_or(:,1), data_or(:,3)), hold on, plot(data2_or(:,1), data2_or(:,3), 'Color', 'red');
ylim(yrange);
title('Y orientation (milli-deg)');

% z
subplot(3,1,3), plot(data_or(:,1), data_or(:,4)), hold on, plot(data2_or(:,1), data2_or(:,4), 'Color', 'red');
%ylim([-180000 180000]);
title('Z orientation (milli-deg)');


std(data_or(:,2))
std(data_or(:,3))

%%


% ACCEL
yrange = [-500, 500];
figure();

data_accel(1:300,1)
mean(data_accel(1:300,4))

subplot(3,1,1), plot(data_accel(:,1), data_accel(:,2)), hold on, plot(data2_accel(:,1), data2_accel(:,2), 'Color', 'red');
%ylim([2100 2120]);
%xlim([0 20]);
title('X acceleration (mg)');

% y
subplot(3,1,2), plot(data_accel(:,1), data_accel(:,3)), hold on, plot(data2_accel(:,1), data2_accel(:,3), 'Color', 'red');
%ylim(yrange);
%xlim([0 60]);
title('Y acceleration (mg)');

% z
subplot(3,1,3), plot(data_accel(:,1), data_accel(:,4)), hold on, plot(data2_accel(:,1), data2_accel(:,4), 'Color', 'red');
%ylim(yrange);
title('Z acceleration (mg)');

%%

% VEL
yrange = [-5000, 5000];
figure();

subplot(3,1,1), plot(data_vel(:,1), data_vel(:,2)), hold on, plot(data2_vel(:,1), data2_vel(:,2), 'Color', 'red');
%ylim(yrange);
title('X velocity (mm/s)');

% y
subplot(3,1,2), plot(data_vel(:,1), data_vel(:,3)), hold on, plot(data2_vel(:,1), data2_vel(:,3), 'Color', 'red');
ylim(yrange);
title('Y velocity (mm/s)');

% z
subplot(3,1,3), plot(data_vel(:,1), data_vel(:,4)), hold on, plot(data2_vel(:,1), data2_vel(:,4), 'Color', 'red');
ylim(yrange);
title('Z velocity (mm/s)');
