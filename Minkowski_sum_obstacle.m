clc
clear all

r = input('Enter the resolution = ');
x1 = 0:250;
y1 = 0:150;
[x,y] = meshgrid(x1,y1);
rob_clear = input('Enter the clearance = ');
diam = input('Enter the diameter of the robot = ');
rob_dim = ceil(diam/2); %ask user for input
offset = rob_dim + rob_clear;

%clearance (creating a boundary wall)
c1cl = (x<=rob_clear & x>=0) | (x>=(250-rob_clear) & x<=250) | (y>=0 & y<=rob_clear) | (y<=150 & y>=(150-rob_clear));

%Rectangle
c1r = x>50 & x<100 & y<112.5 & y>67.5;

%rectable_enlarged
c1re = (x-50).^2 + (y-67.5).^2 <= (offset).^2 | (x-50).^2 + (y-112.5).^2 <= (offset).^2 | (x-100).^2 + (y-67.5).^2 <= (offset).^2 | (x-100).^2 + (y-112.5).^2 <= (offset).^2 ;
c2re = (x >= (50-rob_dim-rob_clear) & x <=(100+offset) & y <=112.5 & y >= 67.5) |(x <=100 & x >= 50 & y <= (112.5+offset) & y >= (67.5-rob_dim-rob_clear)) ;    
  
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

cf1 = plotline1(37,-20,173,15,193,52,offset);
cf2 = plotline2(38,23,193,52,170,90,offset);
cf3 = plotline3(38,-7,170,90,163,52,offset);
cf4 = plotline3(2,19,163,52,125,56,offset);
cf5 = plotline4(41,25,125,56,150,15,offset);

c4pe = (37*x-20*y>=6101)&(37*x-20*y<=cf1)&(y>=15-merge)&(y<=52-merge);
c5pe = (38*x+23*y>=8530)&(38*x+23*y<=cf2)&(y>=52+merge)&(y<=90+merge);
c6pe = (38*x-7*y<=5830)&(38*x-7*y>=cf3)&(y<=90+merge)&(y>=52+merge);
c7pe = (2*x+19*y>=1314)&(2*x+19*y<=cf4)&(x<=163+merge)&(x>=125-merge);
c8pe = (41*x+25*y<=6525)&(41*x+25*y>=cf5)&(y>=15-merge)&(y<=56-merge);

map = ones(size(x));
map(c1r | c1c | c1p | c1e | c1re | c2re | c1ee | c1pe | c2pe | c3pe | c4pe | c5pe | c6pe | c7pe | c8pe | c1ce | c1cl) = 0;

%plot the grid for a point robot
imshow(map, 'xdata', x1, 'ydata', y1)
grid on
set(gca, 'ydir', 'normal')
set(gca, 'xtick', 0:r:250)
set(gca, 'ytick', 0:r:150)
grid off
axis on
grid on
hold on
%plot the rectangle
plot([50 50], [67.5 112.5], 'y')
plot([50 100], [67.5 67.5], 'y')
plot([100 100], [67.5 112.5], 'y')
plot([50 100], [112.5 112.5], 'y')
%plot the circle
plot(xc,yc, 'y')
%plot the ellipse
plot(xe,ye, 'y')
%plot the polygon
plot([150 125], [15 56], 'y')
plot([150 173], [15 15], 'y')
plot([173 193], [15 52], 'y')
plot([170 193], [90 52], 'y')
plot([125 163], [56 52], 'y')
plot([170 163], [90 52], 'y')
hold on
xlabel('X in mm')
ylabel('Y in mm')
axis image
axis([1 251 1 151])

%equation of the line ax+by+c = 0
function intercept = plotline1(x_cf, y_cf, x1,y1,x2,y2,offset)
m = x_cf/y_cf;
a=0;
b=0;
dist = 0;
min_dist= Inf;
intercept=Inf;
centre = round(offset/2);
    for back = centre:-1:1
        for front = centre:1: offset
            new_dist = sqrt(front^2+back^2);
            if new_dist >= offset
            a = front;
            b = back;
            x1_new = x1+a;
            x2_new = x2+a;
            y1_new = y1-b;
            y2_new = y2-b;
            c1 = y_cf*y2+x_cf*x2;
            c2 = y_cf*y1_new + x_cf*x1_new;
            dist =abs(c1-c2)/(sqrt(x_cf^2+y_cf^2));
                if dist>=offset
                    if dist < min_dist
                       min_dist = dist; 
                       intercept = c2;
                    end
                end
            end
        end
    end
    
    if intercept== Inf
    intercept = round(offset*(sqrt(x_cf^2+y_cf^2))+c1);
end
end

function intercept = plotline2(x_cf, y_cf, x1,y1,x2,y2,offset)
m = x_cf/y_cf;
a=0;
b=0;
dist = 0;
min_dist= Inf;
intercept=Inf;
centre = round(offset/2);
    for back = centre:-1:1
        for front = centre:1: offset
            new_dist = sqrt(front^2+back^2);
            if new_dist >= offset
            a = front;
            b = back;
            x1_new = x1+a;
            x2_new = x2+a;
            y1_new = y1+b;
            y2_new = y2+b;
            c1 = (y_cf*y2+x_cf*x2);
            c2 = y_cf*y1_new + x_cf*x1_new;
            dist =abs(c1-c2)/(sqrt(x_cf^2+y_cf^2));
                if dist>=offset
                    if dist < min_dist
                       min_dist = dist; 
                       intercept = c2;
                    end
                end
            end
        end
    end   
if intercept== Inf
    intercept = round(offset*(sqrt(x_cf^2+y_cf^2))+c1);
end
end

function intercept = plotline3(x_cf, y_cf, x1,y1,x2,y2,offset)
intercept= Inf;
m = x_cf/y_cf;
a=0;
b=0;
min_dist= Inf;
intercept=Inf;
dist = 0;
centre = round(offset/2);
    for back = centre:-1:1
        for front = centre:1: offset
            new_dist = sqrt(front^2+back^2);
            if new_dist >= offset
            a = front;
            b = back;
            x1_new = x1-a;
            x2_new = x2-a;
            y1_new = y1+b;
            y2_new = y2+b;
            c1 = (y_cf*y2+x_cf*x2);
            c2 = y_cf*y1_new + x_cf*x1_new;
            dist =abs(c1-c2)/(sqrt(x_cf^2+y_cf^2));
                if dist>=offset
                    if dist < min_dist
                       min_dist = dist; 
                       intercept = c2;
                    end
                end
            end
        end
    end
    
if intercept== Inf
    intercept = round(offset*(sqrt(x_cf^2+y_cf^2))+c1);
end
end

function intercept = plotline4(x_cf, y_cf, x1,y1,x2,y2,offset)
 intercept= Inf;
m = x_cf/y_cf;
a=0;
b=0;
min_dist= Inf;
intercept=Inf;
dist = 0;
centre = round(offset/2);
    for back = centre:-1:1
        for front = centre:1: offset
            new_dist = sqrt(front^2+back^2);

            if new_dist >= offset
            a = front;
            b = back;
            x1_new = x1-a;
            x2_new = x2-a;
            y1_new = y1-b;
            y2_new = y2-b;
            c1 = (y_cf*y2+x_cf*x2);
            c2 = y_cf*y1_new + x_cf*x1_new;
            dist =abs(c1-c2)/(sqrt(x_cf^2+y_cf^2));
                if dist>=offset
                    if dist < min_dist
                       min_dist = dist; 
                       intercept = c2;
                    end
                end
            end
        end
    end 
    
if intercept== Inf
    intercept = round(offset*(sqrt(x_cf^2+y_cf^2))+c1);
end
end


