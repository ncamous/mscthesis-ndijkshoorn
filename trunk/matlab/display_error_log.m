filename = 'C:/Users/Nick/Documents/Thesis/code/cpp/ardrone_slam/ardrone_slam/dataset/067/error_log.txt';

M = csvread(filename)';

figure();
plot(M(1,:), M(5,:),':b');
%ylim([-2000 2000]);
hold on
plot(M(1,:), M(4,:),':r');
plot(M(1,:), M(3,:),':g');
plot(M(1,:), M(2,:)*0.1,'-c');
legend('alt', 'vel', 'accel', 'vel (KF)');
hold off;



%plot(M(1,:), M(4,:),':b');

%plot(M(1,:), abs(M(3,:)),':r');

%plot(M(1,:), abs(M(3,:)),':r');

%y = decimate(abs(M(3,:)),50);
%plot(M(1,1:50:2154), y, ':r');
%plot(M(1,:), 200, ':y');

%plot(data_alt(:,1), data_alt(:,2), 'Color', 'b');
%plot(M(1,:), M(5,:), 'Color', 'b');

%%

% velocity
Xrange = 2800:3300;

X = diff(M(1,Xrange));
X = [X 0];
Y = cumsum(M(3,Xrange) .* X);
plot(M(1,Xrange), Y,':b');

% distance
Y2 = cumsum(Y(1,:) .* -X);
plot(M(1,Xrange), Y2,'r');

Y2(1, length(Y2) - 1)



% velocity
Xrange = 3400:3900;

X = diff(M(1,Xrange));
X = [X 0];
Y = cumsum(M(3,Xrange) .* X);
plot(M(1,Xrange), Y,':b');

% distance
Y3 = cumsum(Y(1,:) .* -X);
plot(M(1,Xrange), Y3,'r');

Y3(1, length(Y3) - 1)



%plot(M(1,:), M(5,:),'-r');
%plot(M(1,:), M(6,:),'-g');
%plot(M(1,:), M(7,:),'-b');

%plot(M2(1,:), M2(2,:),'-b');
%plot(M2(1,:), M2(3,:),'-r');
%plot(M2(1,:), M2(4,:),'b');
hold off;


%ylim(yrange);
%xlim(xrange);
%h = legend('X error (without localization)','Y error (without localization)', 'X error (with localization)','Y error (with localization)')
%rect = [0.25, 0.65, .1, .1];
%set(h, 'Position', rect);
%xlabel('Time (s)');
%ylabel('Error (mm)');



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