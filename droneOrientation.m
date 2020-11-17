% dynamic model for orientation 

function ddq = droneOrientation(q,dq,U,droneparam)

phi = q(1);
theta = q(2);
si = q(3);

d_phi = dq(1);
d_theta = dq(2);
d_si = dq(3);

l = droneparam.l;
Ix = droneparam.Ix;
Iy = droneparam.Iy;
Iz = droneparam.Iz;

dd_phi = (d_theta*d_si*(Iy-Iz) + l*U(2))/Ix;
dd_theta = (d_phi*d_si*(Iz-Ix) + l*U(3))/Iy;
dd_si = (d_theta*d_phi*(Ix-Iy) + U(4))/Iz;

ddq = [dd_phi; dd_theta; dd_si];