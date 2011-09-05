filename = 'C:/Users/Nick/Documents/Thesis/code/cpp/ardrone_slam/ardrone_slam/dataset/012/output.yaml';

yrange = [-700, 700];
xrange = [0, 500];

M = csvread(filename)';

mean(M(4,:))
mean(M(5,:))
mean(M(6,:))

%%

% x
subplot(3,1,1), plot(M(1,:));

%ylim(yrange);
%xlim(xrange);
title('X or (mili-deg)');

% y
subplot(3,1,2), plot(M(2,:));
%ylim(yrange);
%xlim(xrange);
title('Y or (mili-deg)');

% z
subplot(3,1,3), plot(M(3,:));
%ylim(yrange);
%xlim(xrange);
title('Z or (mili-deg)');

%%
figure(2);

yrange = [-2000, 2000];
xrange = [0, 500];

subplot(3,1,1), plot(M(4,:));
%ylim(yrange);
%xlim(xrange);

X = [];
Y = [];

%for j=300:300:length(M)
%    X = [X j-150];
%    Y = [Y mean(M(4, j-299:j))];
%end

%subplot(3,1,1), plot(X, Y, 'd');


title('X accel');

% y
subplot(3,1,2), plot(M(5,:));
%ylim(yrange);
%xlim(xrange);
title('Y accel');

% z
subplot(3,1,3), plot(M(6,:));
%ylim(yrange);
%xlim(xrange);
title('Z accel');