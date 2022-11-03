function D_err = disp_error_image (D_gt,D_est,tau,dilate_radius)

if nargin==3 %%nargin是用来判断输入变量个数的函数，若未输入radius则 默认1
  dilate_radius = 1;
end

[E,D_val] = disp_error_map (D_gt,D_est); % 找出gt大于0的坐标记录于D_val，并将E invalid =0
E = min(E/tau(1),(E./abs(D_gt))/tau(2)); % E取其中最小的（E/3，和E/0.05Dgt )  

cols = error_colormap();

D_err = zeros([size(D_gt) 3]);
for i=1:size(cols,1)
  [v,u] = find(D_val > 0 & E >= cols(i,1) & E <= cols(i,2));
  D_err(sub2ind(size(D_err),v,u,1*ones(length(v),1))) = cols(i,3);
  D_err(sub2ind(size(D_err),v,u,2*ones(length(v),1))) = cols(i,4);
  D_err(sub2ind(size(D_err),v,u,3*ones(length(v),1))) = cols(i,5);
end

D_err = imdilate(D_err,strel('disk',dilate_radius));
