clear all; close all; dbstop error;

% error threshold
tau = [3 0.05];



%第6张5.png
%对于PSNET
I=imread('F:\tum course\capture 3d\project\testyqy\dsparity\nn\disp\left disparity map5.png');
D_est = double(I)/256;

% %对于BM
% I=imread('F:\tum course\capture 3d\project\testyqy\dsparity\BM\left disparity\left disparity map5.jpg');
% D_est = double(I);

%对于SGM
% I=imread('F:\tum course\capture 3d\project\testyqy\dsparity\SGM\left disparity\left disparity map5.jpg');
% D_est = double(I);

%真值
D_gt  = disp_read('F:\tum course\capture 3d\project\testyqy\dsparity\disparity_gt\5.png');
d_err = disp_error(D_gt,D_est,tau);
D_err = disp_error_image(D_gt,D_est,tau);

figure,imshow([disp_to_color([D_est;D_gt]);D_err]);
title(sprintf('Disparity Error: %.2f %%',d_err*100));
