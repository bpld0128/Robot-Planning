r = 0.2;  %ask user for input
x1 = 0:250;
y1 = 0:150;
[x,y] = meshgrid(x1,y1);

%Rectangle
cond1r = x>50 & x<100 & y<112.5 & y>67.5;

%circle
cond1c = (x-190).^2 + (y-130).^2 <= 225;

% polygon
cond1p = ((25*y + 41*x > 6525) & (19*y + 2*x < 1314) & (x <= 163 & y > 15)) | ((38*x -7*y > 5830) & (23*y + 38*x < 8530) & (37*x -20*y < 6101) & (x > 163 & y > 15)) ;

%Ellipse
cond1e = ((x-140).^2)/225 + ((y-120).^2)/36 <= 1;

map = ones(size(x));
map((cond1r) | cond1c | cond1p | cond1e) = 0;

%plot the grid for a point robot
imshow(map, 'xdata', x1, 'ydata', y1)
axis on
grid on
set(gca, 'ydir', 'normal')
set(gca, 'xtick', 0:10:250)
set(gca, 'ytick', 0:10:150)
title('Obstacle Space')
grid off
grid on
xlabel('X in mm')
ylabel('Y in mm')
