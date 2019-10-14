#include <math.h>
#include "SystODEs.h"
#include "parametrosQuadrotor.h"

void SystODEs::FastSystem()
{
	double phi, theta, psi, p, q, r;
	double U1, U3, U4;
	double x, y, z, cx, cy, cz, phit, thetat;

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

    /*U2 = kp2*(phit - phi) - kd2*dblDydt[2];
    U3 = kp3*(thetat - theta) - kd3*dblDydt[3];
    U4 = kp4*(psit - psi) - kd4*dblDydt[4];
	dblDydt[5] = (raizdos*l*U2 + q*r*(Ix - Iz) - JTP*q*Omega)/Ix;  //ppunto
	dblDydt[6] = (raizdos*l*U3 + p*r*(Iz - Ix) - JTP*q*Omega)/Iy;  //qpunto
	dblDydt[7] = (raizdos*U4 + q*p*(Ix - Iy))/Iz;                  //rpunto*/

	cx = m*((1 + kp1*kv1)*(xt - x) - (kp1 + kv1)*dblDydt[0]);
	cy = m*((1 + kp2*kv2)*(yt - y) - (kp2 + kv2)*dblDydt[1]);
	cz = m*((1 + kp3*kv3)*(zt - z) - (kp3 + kv3)*dblDydt[2]);
	thetat = atan2(cy*sin(psi) + cx*cos(psi), cz + m*g);
	phit = atan2((cx*sin(psi) - cy*cos(psi))*cos(thetat), cz + m*g);
	U1 = (cz + m*g) / (cos(thetat)*cos(phit));

	dblDydt[3] = (sin(psi)*sin(phi) + cos(psi)*sin(theta)*cos(phi))*U1/m - kd*dbly[3];
	dblDydt[4] = (sin(psi)*sin(theta)*cos(phi) - cos(psi)*sin(phi))*U1/m - kd*dbly[4];

	//U2 = q*r*(Iz - Iy)*Ix + Ix*(-dblDydt[6] * (kn1 + ko1) - (1 + kn1*ko1)*(phi - phit) + (dblDydt[8] * (kn3 + ko1) + (1 + kn3*ko1)*(psi - psit))*sin(theta));
	U3 = p*r*(Ix - Iz)*Iy + Iy*((-(dblDydt[7] * (kn2 + ko2) + (1 + kn2*ko2)*(theta - thetat)))*cos(phi) - (dblDydt[8] * (kn3 + ko2) + (1 + kn3*ko2)*(psi - psit))*cos(theta)*sin(phi));
	U4 = p*q*(Iy - Ix)*Iz + Iz*((-(dblDydt[8] * (kn3 + ko3) + (1 + kn3*ko3)*(psi - psit)))*cos(phi)*cos(theta) + (dblDydt[7] * (kn2 + ko3) + (1 + kn2*ko3)*(theta - thetat))*sin(phi));
	//dblDydt[9] = (U2 + q*r*(Iy - Iz) - JTP*q*Omega) / Ix; 
	dblDydt[10] = (U3 + p*r*(Iz - Ix) + JTP*p*Omega) / Iy;
	dblDydt[11] = (U4 + p*q*(Ix - Iy)) / Iz;
}
