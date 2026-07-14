syms tx1 ty1 tt1
syms vx1 vy1 vt1
syms bx1 by1 bt1
% Redundant states
syms ax1 ay1 at1

% l is the distance between the IMU and center of rotation
syms dt r b ix iy

tx0 = tx1 - vx1*dt + ax1*dt^2/2;
ty0 = ty1 - vy1*dt + ay1*dt^2/2;
tt0 = tt1 - vt1*dt + at1*dt^2/2;

vx0 = vx1 - ax1*dt;
vy0 = vy1 - ay1*dt;
vt0 = vt1 - at1*dt;

bx0 = bx1;
by0 = by1;
bt0 = bt1;

% Average values to mix in the acceleration more
vxa = (vx0 + vx1) / 2;
vya = (vy0 + vy1) / 2;
tta = (tt0 + tt1) / 2;

% Velocity and acceleration in the robot FoR
vg = rot2(-tta)*[vxa; vya];
ag = rot2(-tta)*[ax1; ay1];

% Encoder velocities
dwl = 1/r * (2*vg(1) - b*vt1) * dt;
dwr = 1/r * (2*vg(1) + b*vt1) * dt;
% Forced to zero to bound sliding
vgy = vg(1);

% Take into account centripetal and tangential acceleration
ax = ag(1) + ix*vt1^2 + iy*at1 + bx1;
ay = ag(1) + iy*vt1^2 + ix*at1 + by1;
gz = vt1 + bt1;

f = [tx0; ty0; tt0; vx0; vy0; vt0; bx0; by0; bt0; dwl; dwr; vgy; ax; ay; gz;]
jacobian(f, [tx1, ty1, tt1, vx1, vy1, vt1, bx1, by1, bt1, ax1, ay1, at1])
