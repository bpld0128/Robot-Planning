%% Author : Himanshu Singhal
%Run A* Algorithm for a rigid robot on a given map
% A* is a generalized form of the Dijkstra search algorithm. A* is faster,
% and finds the optimal path because it uses an heuristic function to
% calculate the distance. It explores only the best nodes and not all the
% nodes

profile on
clear all
close all
clc

%Define the map size
x1 = 0:250;
y1 = 0:250;

%Obtain the intitial and goal points
start_x = input('Enter the value of x coordinate for start:');
start_y = input('Enter the value of y coordinate for start:');
dest_x = input('Enter the value of x coordinate for dest:');
dest_y = input('Enter the value of y coodinate for dest;');

%Convert float to integers
s_x = round(start_x);
s_y = round(start_y);
d_x = round(dest_x);
d_y = round(dest_y);

%obtain the final start and dest. points
start_point = [s_y+1, s_x+1]; % ask for input from the user
dest_point = [d_y+1, d_x+1]; %ask for input from the user

%Obtain the Robot Parameters
clearance = input('Enter the clearance = ');
rob_clear = ceil(clearance);
radius = input('Enter the radius of the robot = ');
rob_rad = ceil(radius); %ask user for input
offset = rob_rad + rob_clear;

%Create the map
[X,Y] = meshgrid(x1,y1);
map = ones(size(X));

%Check if the start/goal point is within the map
if   start_x<0 || start_x>250 || start_y<0 || start_y>150 || dest_x<0 || dest_x>250 || dest_y<0 || dest_y>150
     disp('Start_Point/Dest_Point does not lie on the specified map')
     path = [];
     nodeExplored = 0;
     inside = 0;
     return
else
    inside = 1;
end

%Check if the start/goal point is equal
if (start_x == dest_x) && (start_y == dest_y)
    disp('Start node & Destination node are equal')
     path = [];
     nodeExplored = 0;
     not_equal = 0;
     return
else
    not_equal = 1;
end

%Run the function
if inside == 1 && not_equal == 1 
[path, nodeExplored] = AStarGrid (map, start_point, dest_point,offset);
return
end

function [path,nodeExplored] = AStarGrid (input_map, start_point, dest_point,offset)
% Run A* algorithm on a grid.
%r = input('Enter the resolution = ');

%create the color map for distinctions
 cmap = [1 1 1; ...  % white - Unvisited node
    0 0 0; ...       % Black - Obstacle
    1 0 0; ...       % Red - Destination node
    0 0 1; ...       % Blue - Adjacent node
    0 1 0; ...       % Green - Start node
    1 1 0; ...       % Yellow - Visited node
    0.5 0.5 0.5];    % Gray - Optimal path

colormap(cmap);

% Create map for every iteration
drawMapEveryTime = true;

[nrows, ncols] = size(input_map);

% Create a table that keeps track of the state of each grid cell
map = zeros(nrows,ncols);

% Generate linear indices of start and dest nodes
start_point_lin = sub2ind(size(map), start_point(1), start_point(2));
dest_point_lin  = sub2ind(size(map), dest_point(1),  dest_point(2));

% Produce a full grid
parent = zeros(nrows,ncols);

% obtain the X and Y values
[x, y] = meshgrid (1:ncols, 1:nrows);

%clearance (creating a boundary wall)
c1cl = (x<=offset & x>=0) | (x>=(250-offset) & x<=250) | (y>=0 & y<=offset) | (y<=150 & y>=(150-offset));

%Rectangle
c1r = x>50 & x<100 & y<112.5 & y>67.5;

%rectable_enlarged
c1re = (x-50).^2 + (y-67.5).^2 <= (offset).^2 | (x-50).^2 + (y-112.5).^2 <= (offset).^2 | (x-100).^2 + (y-67.5).^2 <= (offset).^2 | (x-100).^2 + (y-112.5).^2 <= (offset).^2 ;
c2re = (x >= (50-offset) & x <=(100+offset) & y <=112.5 & y >= 67.5) |(x <=100 & x >= 50 & y <= (112.5+offset) & y >= (67.5-offset)) ;    
  
%circle
rc = 15;
xic = 190;
yic = 130;
th = 0:pi/50:2*pi;
xc = rc * cos(th) + xic;
yc = rc * sin(th) + yic;
c1c = (x-190).^2 + (y-130).^2 <= 225;

%circle_enlarged
c1ce = (x-190).^2 + (y-130).^2 <= (rc+offset).^2;

%Ellipse
ae = 15;
be = 6;
xie = 140;
yie = 120;
xe = ae * cos(th) + xie;
ye = be * sin(th) + yie;
c1e = ((x-140).^2)/225 + ((y-120).^2)/36 <= 1;

%ellipse enlarged
c1ee = ((x-140).^2)/(ae+offset).^2 + ((y-120).^2)/(be+offset).^2 <= 1;

% polygon
c1p = ((25*y + 41*x > 6525) & (19*y + 2*x < 1314) & (x <= 163 & y > 15)) | ((38*x -7*y > 5830) & (23*y + 38*x < 8530) & (37*x -20*y < 6101) & (x > 163 & y > 15)) ;

%polygon enlarged
c1pe = (x-125).^2 + (y-56).^2 <=(offset).^2 | (x-150).^2 + (y-15).^2 <= (offset).^2 | (x-163).^2 + (y-52).^2 <= (offset).^2;
c2pe = (x-173).^2 + (y-15).^2 <= (offset).^2 | (x-193).^2 + (y-52).^2 <= (offset).^2 | (x-170).^2 + (y-90).^2 <= (offset).^2;
c3pe = (y <= 15 & y > 15-(offset) & x > 150 & x < 173);

%find the cficients of the offset line where it meets the circle to get
%a round edge
merge = ceil((offset)/2);

cf1 = line1(37,-20,173,15,193,52,offset);
cf2 = line2(38,23,193,52,170,90,offset);
cf3 = line3(38,-7,170,90,163,52,offset);
cf4 = line4(2,19,163,52,125,56,offset);
cf5 = line5(41,25,125,56,150,15,offset);

c4pe = (37*x-20*y>=6101)&(37*x-20*y<=cf1)&(y>=15-merge)&(y<=52-merge);
c5pe = (38*x+23*y>=8530)&(38*x+23*y<=cf2)&(y>=52+merge)&(y<=90+merge);
c6pe = (38*x-7*y<=5830)&(38*x-7*y>=cf3)&(y<=90+merge)&(y>=52+merge);
c7pe = (2*x+19*y>=1314)&(2*x+19*y<=cf4)&(x<=163+merge)&(x>=125-merge);
c8pe = (41*x+25*y<=6525)&(41*x+25*y>=cf5)&(y>=15-merge)&(y<=56-merge);

map = ones(size(x));
map(c1r | c1c | c1p | c1e | c1re | c2re | c1ee | c1pe | c2pe | c3pe | c4pe | c5pe | c6pe | c7pe | c8pe | c1ce | c1cl) = 2;

function c = line1(x_cf, y_cf, x1,y1,x2,y2,offset)
dx=offset;
dy=1;
            n_x1 = x1+dx;
            n_x2 = x2+dx;
            n_y1 = y1-dy;
            n_y2 = y2-dy;
            c = y_cf*n_y1 + x_cf*n_x1;
end

function c = line2(x_cf, y_cf, x1,y1,x2,y2,offset)
dx=offset;
dy=1;
            n_x1 = x1+dx;
            n_x2 = x2+dx;
            n_y1 = y1+dy;
            n_y2 = y2+dy;
            c = y_cf*n_y1 + x_cf*n_x1;

end

function c = line3(x_cf, y_cf, x1,y1,x2,y2,offset)
dx=offset;
dy=1;
            n_x1 = x1-dx;
            n_x2 = x2-dx;
            n_y1 = y1+dy;
            n_y2 = y2+dy;
            c = y_cf*n_y1 + x_cf*n_x1;       
end

function c = line5(x_cf, y_cf, x1,y1,x2,y2,offset)
dx=offset;
dy=1;
            n_x1 = x1-dx;
            n_x2 = x2-dx;
            n_y1 = y1-dy;
            n_y2 = y2-dy;
            c = y_cf*n_y1 + x_cf*n_x1;
end 

function c = line4(x_cf, y_cf, x1,y1,x2,y2,offset)
dx=1;
dy=offset;
            n_x1 = x1-dx;
            n_x2 = x2-dx;
            n_y1 = y1+dy;
            n_y2 = y2+dy;
            c = y_cf*n_y1 + x_cf*n_x1;       
end

%check if the start/goal point is on the obstacle space
if (map(start_point_lin) == 2 || map(dest_point_lin) == 2)
     disp('Start_point/Dest_point lie on the obstacle')
     path = [];
     nodeExplored = 0;
     return
end

% create destination points
xd = dest_point(1);
yd = dest_point(2);

% Evaluate Heuristic function, H, for each grid cell
% Euclidean distance
dx = abs(x-xd);
dy = abs(y-yd);
H = sqrt(dx.^2 + dy.^2);
H = H';  %cost from the current point to the goal point

% Initialize cost arrays to infinity
% f(x,y) = g(x,y) + h(x,y);
f = Inf(nrows,ncols); % total cost
g = Inf(nrows,ncols); % cost from the start point to the current point

% For the start point, the cost g(x,y) is 0
g(start_point_lin) = 0;
f(start_point_lin) = H(start_point_lin);

% Initialize the nodes explored
nodeExplored = 0;

% Main Loop
while true
    
    % Mark the start and destination point
    map(start_point_lin) = 5;
    map(dest_point_lin) = 3;
    
    %Uncomment this to see visualization
%      if (drawMapEveryTime)
%         image(map);
%         set(gca, 'ydir', 'normal')
%         set(gca, 'xtick', 0:10:250)
%         set(gca, 'ytick', 0:10:150)
%         xlabel('X in mm')
%         ylabel('Y in mm')
%         title('A* Algorithm')
%         grid on;
%         axis image;
%         axis([1 251 1 151])
%         drawnow
%     end
    
    % Compare the current node and the minimum f
    [min_f, cur_node] = min(f(:));
    
    % Break if the destination node is reached or mininmum f is infinite(Destination does not exist)
    if ((cur_node == dest_point_lin) || isinf(min_f))
        break;
    end
    
    % Update input_map
    map(cur_node) = 6;
    f(cur_node) = Inf; 
    
    % Compute row, column coordinates of current node
    [i, j] = ind2sub(size(f), cur_node);
   
    % Visit all of the neighbors around the current node and update the
    % entries in the map, f, g and parent arrays
    m = 0;
    n = 0;
    if (i>=1 && i<nrows) %% UP
        m = i+1; 
        n = j;
        if (map(m,n)~=2 && map(m,n)~=5)
             if g(m,n) > (g(i,j) + 1)
                g(m,n) = g(i,j) + 1;
                f(m,n) = g(i,j) + H(m,n);
                map(m,n) = 4;
                parent(m,n) = cur_node;
            end
        end
    end
    
    if (i>1 && i<=nrows) % DOWN
        m = i-1; 
        n = j;
        if (map(m,n)~=2 && map(m,n)~=5)
            if g(m,n) > (g(i,j) + 1)
                g(m,n) = g(i,j) + 1;
                f(m,n) = g(m,n) + H(m,n);
                map(m,n) = 4;
                parent(m,n) = cur_node;
            end
        end
    end
    if (j>1 && j<=ncols) % LEFT
        n = j-1;
        m = i;
        if (map(m,n)~=2 && map(m,n)~=5)
            if g(m,n) > (g(i,j) + 1)
                g(m,n) = g(i,j) + 1;
                f(m,n) = g(m,n) + H(m,n);
                map(m,n) = 4;
                parent(m,n) = cur_node;
            end
        end
    end
    if (j>=1 && j<ncols) %RIGHT
        n =j+1;
        m = i;
        if (map(m,n)~=2 && map(m,n)~=5)
            if g(m,n) > (g(i,j) + 1)
                g(m,n) = g(i,j) + 1;
                f(m,n) = g(m,n) + H(m,n);
                map(m,n) = 4;
                parent(m,n) = cur_node;
            end
        end
    end
    if (i>=1 && i<nrows && j>=1 && j<ncols) %Up Right
        m = i+1;
        n = j+1;
        if (map(m,n)~=2 && map(m,n)~=5)
            if g(m,n) > (g(i,j) + sqrt(2))
                g(m,n) = g(i,j) + sqrt(2);
                f(m,n) = g(m,n) + H(m,n);
                map(m,n) = 4;
                parent(m,n) = cur_node;
            end
        end
    end
    if (i>1 && i<=nrows && j>=1 && j<ncols) %Down Right
        m =i-1;
        n = j+1;
        if (map(m,n)~=2 && map(m,n)~=5)
            if g(m,n) > (g(i,j) + sqrt(2))
                g(m,n) = g(i,j) + sqrt(2);
                f(m,n) = g(m,n) + H(m,n);
                map(m,n) = 4;
                parent(m,n) = cur_node;
            end
        end
    end
     if (i>1 && i<=nrows && j>1 && j<=ncols) %Down Left
        m =i-1;
        n = j-1;
        if (map(m,n)~=2 && map(m,n)~=5)
            if g(m,n) > (g(i,j) + sqrt(2))
                g(m,n) = g(i,j) + sqrt(2);
                f(m,n) = g(m,n) + H(m,n);
                map(m,n) = 4;
                parent(m,n) = cur_node;
            end
        end
    end
     if (i>=1 && i<nrows && j>1 && j<=ncols) %Up Left
        m =i+1;
        n = j-1;
        if (map(m,n)~=2 && map(m,n)~=5)
            if g(m,n) > (g(i,j) + sqrt(2))
                g(m,n) = g(i,j) + sqrt(2);
                f(m,n) = g(m,n) + H(m,n);
                map(m,n) = 4;
                parent(m,n) = cur_node;
            end
        end
    end
    nodeExplored = nodeExplored + 1;
    disp(nodeExplored)
    f(cur_node) = Inf;
     
end

% Construct path from start to dest by following the parent links
if (isinf(f(dest_point_lin)))
    path = [];
    disp('Path Not Found')
else
    path = (dest_point_lin);
    disp('Path Found')
    
    while (parent(path(1)) ~= 0)
        path = [parent(path(1)), path];
    end

    % Visualize the map and the path
    for k = 2:length(path) - 1        
        map(path(k)) = 7;
        image(map);
        set(gca, 'ydir', 'normal')
        set(gca, 'xtick', 0:10:250)
        set(gca, 'ytick', 0:10:150)
        grid off
        axis on
        grid on
        hold on
%         %plot the rectangle
%         plot([50 50], [67.5 112.5], 'y')
%         plot([50 100], [67.5 67.5], 'y')
%         plot([100 100], [67.5 112.5], 'y')
%         plot([50 100], [112.5 112.5], 'y')
%         %plot the circle
%         plot(xc,yc, 'y')
%         %plot the ellipse
%         plot(xe,ye, 'y')
%         %plot the polygon
%         plot([150 125], [15 56], 'y')
%         plot([150 173], [15 15], 'y')
%         plot([173 193], [15 52], 'y')
%         plot([170 193], [90 52], 'y')
%         plot([125 163], [56 52], 'y')
%         plot([170 163], [90 52], 'y')
        hold on
        xlabel('X in mm')
        ylabel('Y in mm')
        title('A* Algorithm for Rigid Robot')
        axis image
        axis([-1 251 -1 151])
        drawnow
    end
end
profile viewer
end
