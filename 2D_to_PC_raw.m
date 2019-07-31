% clear all;
close all;
clc;

%%%%%% Reading Calib data needed for projection
cam       = 2; % 0-based index
fram     = 000000; % 0-based index
calib_dir = 'D:\Sagnik\2011_09_26';
base_dir  = 'D:\Sagnik';

calib = loadCalibrationCamToCam(fullfile(calib_dir,'calib_cam_to_cam.txt'));
Tr_velo_to_cam = loadCalibrationRigid(fullfile(calib_dir,'calib_velo_to_cam.txt'));

R_cam_to_rect = eye(4);
R_cam_to_rect(1:3,1:3) = calib.R_rect{1};
P_velo_to_img = calib.P_rect{cam+1}*R_cam_to_rect*Tr_velo_to_cam;
leftI = imread('D:\Sagnik\RAW YOLO\0000000283_2.png');
rightI = imread('D:\Sagnik\RAW YOLO\0000000283_3.png');
frameLeftGray  = rgb2gray(leftI);
frameRightGray = rgb2gray(rightI);

%Bounding Box of Object
min_x = 1115;
max_x = 1231;
min_y = 147;
max_y = 363;

x = round((max_x + min_x)/2);
y = round((max_y + min_y)/2); 


tic,
disparityMap = disparity(frameLeftGray, frameRightGray, 'DisparityRange', [0 96], 'BlockSize',5);
toc
% Converting disparity to depth by triangulation formula
depthM = (7.2153e+02 * 0.540).* ones(size(disparityMap))./disparityMap;


%%%%%% getting inverse transform
P_rect_inv = pinv(calib.P_rect{cam+1});
R_rect_inv = pinv(R_cam_to_rect);
Tr_velo_2_cam_inv = pinv(Tr_velo_to_cam);
P_img_to_velo = Tr_velo_2_cam_inv*R_rect_inv*P_rect_inv;
%imshow(P_img_to_velo);


%Full Cloud image
depthSize = size(depthM,1) * size(depthM,2);
depthVec = reshape(depthM, [1 depthSize]);            % reshaping stereo depth map to vector
[I1, J1] = ind2sub(size(depthM),1:depthSize);         % getting pixel coordinates
I1 = I1 .* depthVec;                                 % i * d
J1 = J1 .* depthVec;                                 % j * d
Pximg = [J1; I1; depthVec]';                         % creating vector[i*d j*d d]
Pst_temp = P_img_to_velo*Pximg';
p=Pst_temp';



% for bin files
fid = fopen('D:\Sagnik\RAW YOLO\0000000283.bin','rb');
velo = fread(fid,[4 inf],'single')';
%velo = velo(1:5:end,:); % remove every 5th point for display speed
fclose(fid);

figure,pcshow(p(:,1:3));
hold on;
plot3(im2(:,1),im2(:,2),im2(:,3),'r*');
hold on;
