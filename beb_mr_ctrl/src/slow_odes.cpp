#include "beb_mr_ctrl/syst_odes.h"


namespace beb_mr_ctrl {

void SystODEs::SlowSystem()
{ 
	double x, y, z, phi, theta, psi, p, q, r;
	//double U1, U2, cx, cy, cz, thetat, phit;
	//double U1, U2, phit;

	// Slow subsystem equations
   // asignacion de variables de estado arreglo x(noEdos)
   // a variables del cuadrotor
   x = dbly[0];
   y = dbly[1];
	z = dbly[2];
   phi = dbly[6];
   theta = dbly[7];
   psi = dbly[8];
   p = dbly[9];
   q = dbly[10];
   r = dbly[11];

   // modelo del cuadrotor
   /*dblDydt[0] = w;         //zpunto
   // Control PD para z(t)
   U1 = kp1*(zt - z) - kd1*dblDydt[0];
   dblDydt[1] = (U1*cos(phi)*cos(theta) - m*g)/m;  //wpunto
   dblDydt[2] = (p*cos(theta) + q*sin(phi)*sin(theta) + r*cos(phi)*sin(theta))/cos(theta); //phipunto
   dblDydt[3] = q*cos(phi) + r*sin(phi); //thetapunto
   dblDydt[4] = (q*sin(phi) + r*cos(phi))/cos(theta); //psipunto*/

	dblDydt[0] = dbly[3];
	dblDydt[1] = dbly[4];
	dblDydt[2] = dbly[5];
	/*cx = m*((1 + kp1*kv1)*(xt - x) - (kp1 + kv1)*dblDydt[0]);
	cy = m*((1 + kp2*kv2)*(yt - y) - (kp2 + kv2)*dblDydt[1]);
	cz = m*((1 + kp3*kv3)*(zt - z) - (kp3 + kv3)*dblDydt[2]);
	thetat = atan2(cy*sin(psi) + cx*cos(psi), cz + m*g);
	phit = atan2((cx*sin(psi) - cy*cos(psi))*cos(thetat), cz + m*g);
	U1 = (cz + m*g) / (cos(thetat)*cos(phit));*/
	//dblDydt[3] = (sin(psi)*sin(phi) + cos(psi)*sin(theta)*cos(phi))*U1/m - kd*dbly[3];
	//dblDydt[4] = (sin(psi)*sin(theta)*cos(phi) - cos(psi)*sin(phi))*U1/m - kd*dbly[4];
	dblDydt[5] = cos(theta)*cos(phi)*dblInputs[0]/mass - grav - kd*dbly[5];
	dblDydt[6] = p + r*cos(phi)*tan(theta) + q*sin(phi)*tan(theta);
	dblDydt[7] = q*cos(phi) - r*sin(phi);
	dblDydt[8] = (r*cos(phi) + q*sin(phi)) / cos(theta);

   //U2 = q*r*(Iz - Iy)*Ix + Ix*(-dblDydt[6] * (kn1 + ko1) - (1 + kn1*ko1)*(phi - phit) + (dblDydt[8] * (kn3 + ko1) + (1 + kn3*ko1)*(psi - psit))*sin(theta));
   dblDydt[9] = (dblInputs[1] + q*r*(Iyy - Izz) - JTP*q*Omega) / Ixx;
}

}  // namespace bebop_mr_ctrl

