filename = 'C:/Users/Nick/Documents/Thesis/code/cpp/ardrone_slam/ardrone_slam/dataset/exp6/error_log.txt';
filename2 = 'C:/Users/Nick/Documents/Thesis/code/cpp/ardrone_slam/ardrone_slam/dataset/exp5/error_log.txt';

yrange = [0, 3500];
xrange = [0, 742];

M = csvread(filename)';
M2 = csvread(filename2)';


% x
plot(M(1,:), M(2,:),'-.b');
hold on
plot(M(1,:), M(3,:),'-.r');
%plot(M(1,:), M(4,:),':b');

plot(M2(1,:), M2(2,:),'-b');
plot(M2(1,:), M2(3,:),'-r');
%plot(M2(1,:), M2(4,:),'b');
hold off;


ylim(yrange);
xlim(xrange);
h = legend('X error (without localization)','Y error (without localization)', 'X error (with localization)','Y error (with localization)')
rect = [0.25, 0.65, .1, .1];
set(h, 'Position', rect);
xlabel('Time (s)');
ylabel('Error (mm)');



%%
figure('Position',[1 1 1100 1100])

filename = 'C:/Users/Nick/Documents/Thesis/code/cpp/ardrone_slam/ardrone_slam/dataset/exp5/loc_log.txt';

%xrange = [-3000, 3000];
yrange = [-10000, 10000];
xrange = [-10000, 10000];

ha = axes('units','normalized', 'position',[0 0 1 1]);

M = csvread(filename)';

I=imread('C:/Users/Nick/Documents/Thesis/code/cpp/ardrone_slam/ardrone_slam/dataset/exp3mapping/visual_map_canvas.png', 'BackgroundColor', [1 1 1]);
size(I)
hi = imagesc(I);

% x
%set(ha,'handlevisibility','off', 'visible','off')
axes('position',[0.0,0.0,1.0,1.0]);
hold on;
plot(M(2,:),M(3,:), '.', 'MarkerEdgeColor','red');
set(gca,'color','none');
axis([-10000 10000 -10000 10000]);