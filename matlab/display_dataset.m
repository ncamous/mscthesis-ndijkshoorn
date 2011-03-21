filename = 'C:/Users/Nick/Documents/Thesis/code/cpp/dataset_collector/dataset_collector/dataset/004/output.yaml';

[data_alt, data_or, data_accel, data_vel] = loadDataset(filename);

data2_alt = zeros(1,2);
data2_or  = zeros(1,4);
data2_accel=zeros(1,4);
data2_vel = zeros(1,4);

filename = 'C:/Users/Nick/Documents/Thesis/code/cpp/dataset_collector/dataset_collector/dataset/005/output.yaml';

[data2_alt, data2_or, data2_accel, data2_vel] = loadDataset(filename);


%%


% ALT
figure();
plot(data_alt(:,1), data_alt(:,2));
hold on
plot(data2_alt(:,1), data2_alt(:,2), 'Color', 'red');
title('Altitude');



% OR
yrange = [-30000, 30000];
figure();
subplot(3,1,1), plot(data_or(:,1), data_or(:,2)), hold on, plot(data2_or(:,1), data2_or(:,2), 'Color', 'red');
ylim(yrange);
title('X orientation');


% y
subplot(3,1,2), plot(data_or(:,1), data_or(:,3)), hold on, plot(data2_or(:,1), data2_or(:,3), 'Color', 'red');
ylim(yrange);
title('Y orientation');

% z
subplot(3,1,3), plot(data_or(:,1), data_or(:,4)), hold on, plot(data2_or(:,1), data2_or(:,4), 'Color', 'red');
ylim([-180000 180000]);
title('Z orientation');



% ACCEL
yrange = [-500, 500];
figure();

subplot(3,1,1), plot(data_accel(:,1), data_accel(:,2)), hold on, plot(data2_accel(:,1), data2_accel(:,2), 'Color', 'red');
ylim(yrange);
title('X acceleration');

% y
subplot(3,1,2), plot(data_accel(:,1), data_accel(:,3)), hold on, plot(data2_accel(:,1), data2_accel(:,3), 'Color', 'red');
ylim(yrange);
title('Y acceleration');

% z
subplot(3,1,3), plot(data_accel(:,1), data_accel(:,4)), hold on, plot(data2_accel(:,1), data2_accel(:,4), 'Color', 'red');
ylim(yrange);
title('Z acceleration');



% VEL
yrange = [-3000, 3000];
figure();

subplot(3,1,1), plot(data_vel(:,1), data_vel(:,2)), hold on, plot(data2_vel(:,1), data2_vel(:,2), 'Color', 'red');
ylim(yrange);
title('X velocity');

% y
subplot(3,1,2), plot(data_vel(:,1), data_vel(:,3)), hold on, plot(data2_vel(:,1), data2_vel(:,3), 'Color', 'red');
ylim(yrange);
title('Y velocity');

% z
subplot(3,1,3), plot(data_vel(:,1), data_vel(:,4)), hold on, plot(data2_vel(:,1), data2_vel(:,4), 'Color', 'red');
ylim(yrange);
title('Z velocity');
