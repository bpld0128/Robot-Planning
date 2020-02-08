%% Author : Himanshu Singhal
%Run A* Algorithm for Point Robot on a given map
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

%Create the map
[x,y] = meshgrid(x1,y1);
map = ones(size(x));

%Check if the start/goal point is within the map
if   start_x<0 || start_x>250 || start_y<0 || start_y>150 || dest_x<0 || dest_x>250 && dest_y<0 || dest_y>150
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
[path, nodeExplored] = AStarGrid (map, start_point, dest_point);
return
end

function [path,nodeExplored] = AStarGrid (input_map, start_point, dest_point)
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
[X, Y] = meshgrid (1:ncols, 1:nrows);

%Rectangle
cond1r = X>50 & X<100 & Y<112.5 & Y>67.5;

%circle
cond1c = (X-190).^2 + (Y-130).^2 <= 225;

% polygon
cond1p = ((25*Y + 41*X > 6525) & (19*Y + 2*X < 1314) & (X <= 163 & Y > 15)) | ((38*X -7*Y > 5830) & (23*Y + 38*X < 8530) & (37*X -20*Y < 6101) & (X > 163 & Y > 15)) ;

%Ellipse
cond1e = ((X-140).^2)/225 + ((Y-120).^2)/36 <= 1;

map = ones(size(X));
map((cond1r) | cond1c | cond1p | cond1e) = 2;

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
dx = abs(X-xd);
dy = abs(Y-yd);
H = sqrt(dx.^2 + dy.^2);
H = H';  %cost from the current point to the goal point

% Initialize cost arrays
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
        xlabel('X in mm')
        ylabel('Y in mm')
        title('A* Algorithm for Point Robot')
        grid on;
        axis image;
        axis([1 251 1 151])
        drawnow
    end
end
profile viewer
end
