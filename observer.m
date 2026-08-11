syms txf tyf ttf
syms vxf vyf vtf
syms bxf byf btf
% Redundant states
syms axf ayf atf

% l is the distance between the IMU and center of rotation
syms dt rd bw ix iy

txi = txf - vxf*dt + axf*dt^2/2;
tyi = tyf - vyf*dt + ayf*dt^2/2;
tti = ttf - vtf*dt + atf*dt^2/2;

vxi = vxf - axf*dt;
vyi = vyf - ayf*dt;
vti = vtf - atf*dt;

bxi = bxf;
byi = byf;
bti = btf;

% Average values to mix in the acceleration more
vxa = (vxi + vxf) / 2;
vya = (vyi + vyf) / 2;
vta = (vti + vtf) / 2;
tta = (tti + ttf) / 2;

% Velocity and acceleration in the robot FoR
vg = rot2(-tta)*[vxa; vya];
ag = rot2(-tta)*[axf; ayf];

% Encoder velocities
dwl = 1/rd * (2*vg(1) - bw*vta) * dt;
dwr = 1/rd * (2*vg(1) + bw*vta) * dt;
% Forced to zero to bound sliding
vgy = vg(2);

% Take into account centripetal and tangential acceleration
ax = ag(1) + ix*vtf^2 + iy*atf + bxf;
ay = ag(2) + iy*vtf^2 + ix*atf + byf;
gz = vtf + btf;

f = [txi; tyi; tti; vxi; vyi; vti; bxi; byi; bti; dwl; dwr; vgy; ax; ay; gz;]
w = jacobian(f, [txf, tyf, ttf, vxf, vyf, vtf, bxf, byf, btf, axf, ayf, atf])
