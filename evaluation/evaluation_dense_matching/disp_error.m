function d_err = disp_error (D_gt,D_est,tau)

E = abs(D_gt-D_est); %求深度差
% n_err   = length(find(D_gt>0  & E./abs(D_gt)>tau(2)));
n_err   = length(find(D_gt>0 & E>tau(1) & E./abs(D_gt)>tau(2)));%%找出视距误差大于3且大于%5的点
n_total = length(find(D_gt>0));%所有可测点
d_err = n_err/n_total;
