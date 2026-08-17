syms txf tyf ttf real

% LIDAR relative pose
syms lx ly lt real

% wall normal and offset
syms nx ny wk real

lp = [txf; tyf] + rot2(ttf) * [lx; ly];
lr = ttf + lt;

% distance to wall along ray
h = simplify(abs(nx*lp(1) + ny*lp(2) - wk) / abs(dot([cos(lr); sin(lr)], [nx; ny])))

H = jacobian(h, [txf tyf ttf])
