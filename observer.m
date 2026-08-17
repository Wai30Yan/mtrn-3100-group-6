% State vector
syms tx ty tt real
% Input vector
syms dwl dwr real
% Constants
syms rd bw positive
% encoder variance
syms ee positive


dx = rd / 2 * (dwl + dwr);
dw = rd / bw * (-dwl + dwr);
tta = tt + dw/2;

txn = tx + dx*cos(tta);
tyn = ty + dx*sin(tta);
ttn = tt + dw;

f = [txn; tyn; ttn]
fj = jacobian(f, [tx, ty, tt])
fju = jacobian(f, [dwl, dwr])
