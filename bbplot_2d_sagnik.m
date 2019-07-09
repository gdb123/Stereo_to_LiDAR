in = imread("D:\Sagnik\RAW YOLO\0000000283_2.png");
imshow(in);
points = 200*rand(50,2);
hold on
%plot(points(:,1),points(:,2),'r*');
% Now use min/max to determine bounding rectangle
%{
min_y = 165;
max_y = 347;
min_x = 226;
max_x = 313;

min_x = 808;
max_x = 871;
min_y = 163;
max_y = 339;

min_x = 94;
max_x = 225;
min_y = 269;
max_y = 349;
%}

min_x = 1115;
max_x = 1231;
min_y = 147;
max_y = 363;
x = round((max_x + min_x)/2);
y = round((max_y + min_y)/2); 

% Use rectangle to draw bounding rectangle
plot(min_x ,min_y,'r*')
plot(max_x, min_y,'r*')
plot(max_x,max_y,'r*');
plot(min_x,max_y,'r*');
%rectangle('Position',[min_x min_y (max_x-min_x) (max_y-min_y)],'EdgeColor','r');