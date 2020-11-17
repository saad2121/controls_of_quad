% dynamic model for drone position

function ddX = dronePos(q,U,d_X,droneparam)

phi = q(1);
theta = q(2);
si = q(3);

d_x = d_X(1);
d_y = d_X(2);
d_z = d_X(3);

m = droneparam.m;
g = droneparam.g;
Kf = droneparam.Kf;

cp = cos(phi);  sp = sin(phi);
ct = cos(theta);    st = sin(theta);
cs = cos(si);   ss = sin(si);

dd_x = ((cp*st*cs + sp*ss) * U(1) - Kf * d_x ) / m;
dd_y = ((cp*st*ss - sp*cs) * U(1) - Kf * d_y ) / m;
dd_z = ((cp*ct) * U(1) - m * g - Kf * d_z ) / m;

ddX = [dd_x;dd_y;dd_z];