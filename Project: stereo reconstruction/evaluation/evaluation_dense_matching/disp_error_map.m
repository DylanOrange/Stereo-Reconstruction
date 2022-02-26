function [E,D_gt_val] = disp_error_map (D_gt,D_est)

D_gt_val = D_gt>=0; %找到>0的坐标
E = abs(D_gt-D_est);
E(D_gt_val==0) = 0; % 将invalid 的Error pixel 变为0
