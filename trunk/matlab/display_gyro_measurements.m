filename = 'C:/Users/Nick/Documents/Thesis/code/cpp/dataset_collector/dataset_collector/gyros.csv';

yrange = [-150, 150];

M = csvread(filename)';

% x
subplot(3,1,1), plot(M(1,:));
ylim(yrange);
title('X acceleration');

% y
subplot(3,1,2), plot(M(2,:));
ylim(yrange);
title('Y acceleration');

% z
subplot(3,1,3), plot(M(3,:));
ylim(yrange);
title('Z acceleration');

%%
figure(2);

yrange = [-2000, 2000];

subplot(3,1,1), plot(M(4,:));
ylim(yrange);
title('X vel');

% y
subplot(3,1,2), plot(M(5,:));
ylim(yrange);
title('Y vel');

% z
subplot(3,1,3), plot(M(6,:));
ylim(yrange);
title('Z vel');