#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <cstdio>
#include <time.h>
#include <fstream>

using namespace std;

#include "rk4.hpp"
#include "params.h"

int main();
//void rk4_test();
//double rk4_test_f(double t, double u);
void qr_test();
double *qr_f(double t, int n, double s[]);

void Display(double t, int n, double s[]);
int FrecImp(double ti, double tf, double dt, int dp);


//****************************************************************************
//****************************************************************************
//  Purpose:
//    MAIN is the main program for RK4_PRB.
//
//  Discussion:
//    RK4_PRB tests the RK4 library.
//
//  Licensing:
//    This code is distributed under the GNU LGPL license. 
//
//  Modified:
//    20 January 2019
//
//  Author:
//    Antonio Matus, original from John Burkardt
//
int main()
{
	timestamp();
	cout << "\n";
	cout << "RK4_PRB\n";
	cout << "  C++ version\n";
	cout << "  Test the RK4 library.\n";

	//  Set output formatting.
	cout.setf(ios::scientific);     // Use scientific notation.
	cout.precision(6);              // Show six digits.

	//rk4_test();
	//rk4vec_test();
	qr_test();
	//
	//  Terminate.
	//
	cout << "\n";
	cout << "RK4_PRB\n";
	cout << "  Normal end of execution.\n";
	cout << "\n";
	timestamp();

	std::getchar();

	return 0;
}


//****************************************************************************
//  Purpose:
//    QR_TEST integrates numerically the quadrotor ODEs
//
//  Licensing:
//    This code is distributed under the GNU LGPL license. 
//
//  Modified:
//    20 January 2019
//
//  Author:
//    Antonio Matus
//
void qr_test()
{
	double dt = 0.001;
	int i;
	int n = 12;
	double t0;
	double t1;
	double tmax = 5.0;
	double *s0;
	double *s1;

	int iGridPoints = FrecImp(0, tmax, dt, 1250);
	int iOutFrecuency = 0;

	cout << "\n";
	cout << "QR_TEST\n";
	//cout << "  RK4VEC takes a Runge Kutta step for a vector ODE.\n";

	cout << "\n";
	cout << "       T       x       y       z\n";
	cout << "\n";
	t0 = 0.0;

	s0 = new double[n];
	s0[0] = 0.0;
	s0[1] = 0.0;
	s0[2] = 0.0;
	s0[3] = 0.0;
	s0[4] = 0.0;
	s0[5] = 0.0;
	s0[6] = 0.0;
	s0[7] = 0.0;
	s0[8] = 0.0;
	s0[9] = 0.0;
	s0[10] = 0.0;
	s0[11] = 0.0;

	Display(t0, n, s0);

	clock_t begin = clock();

	for (; ; )
	{
		//
		//  Print (T0,U0).
		//
		/*cout << "  " << setw(14) << t0
			<< "  " << setw(14) << s0[0]
			<< "  " << setw(14) << s0[1]
			<< "  " << setw(14) << s0[2] << "\n";*/
		//iOutFrecuency++;
		if (iOutFrecuency == iGridPoints)
		{
			// Analytical solution and relative error
			Display(t0, n, s0);
			iOutFrecuency = 0;
		}
		iOutFrecuency++;
		//
		//  Stop if we've exceeded TMAX.
		//
		if (tmax <= t0)
		{
			break;
		}
		//
		//  Otherwise, advance to time T1, and have RK4 estimate 
		//  the solution U1 there.
		//
		t1 = t0 + dt;
		s1 = rk4vec(t0, n, s0, dt, qr_f);
		//
		//  Shift the data to prepare for another step.
		//
		t0 = t1;
		for (i = 0; i < n; i++)
		{
			s0[i] = s1[i];
		}
		delete[] s1;
	}

	clock_t end = clock();
	cout << "Time elapsed: " << (end - begin) * 1000 / CLOCKS_PER_SEC << " ms \n\n";

	return;
}


//****************************************************************************
//  Purpose:
//    QR_F evaluates the right hand side of the quadrotor ODEs.
//
//  Licensing:
//    This code is distributed under the GNU LGPL license. 
//
//  Modified:
//    20 January 2019
//
//  Author:
//    Antonio Matus
//
//  Parameters:
//    Input, double T, the current time.
//    Input, int N, the dimension of the system.
//    Input, double S[N], the current solution value.
//    Output, double QR_F[N], the value of the derivative, dS/dT.
//
double *qr_f(double t, int n, double s[])
{
	double *sprime;

	sprime = new double[n];

	double x, y, z, phi, theta, psi, p, q, r;
	double u1, u2, u3, u4, cx, cy, cz, thetat, phit;

	x = s[0];
	y = s[1];
	z = s[2];
	phi = s[6];
	theta = s[7];
	psi = s[8];
	p = s[9];
	q = s[10];
	r = s[11];

	sprime[0] = s[3];
	sprime[1] = s[4];
	sprime[2] = s[5];
	cx = m*((1 + kp1*kv1)*(xt - x) - (kp1 + kv1)*sprime[0]);
	cy = m*((1 + kp2*kv2)*(yt - y) - (kp2 + kv2)*sprime[1]);
	cz = m*((1 + kp3*kv3)*(zt - z) - (kp3 + kv3)*sprime[2]);
	thetat = atan2(cy*sin(psi) + cx*cos(psi), cz + m*g);
	phit = atan2((cx*sin(psi) - cy*cos(psi))*cos(thetat), cz + m*g);
	u1 = (cz + m*g) / (cos(thetat)*cos(phit));
	sprime[3] = (sin(psi)*sin(phi) + cos(psi)*sin(theta)*cos(phi))*u1 / m - kd*s[3];
	sprime[4] = (sin(psi)*sin(theta)*cos(phi) - cos(psi)*sin(phi))*u1 / m - kd*s[4];
	sprime[5] = cos(theta)*cos(phi)*u1 / m - g - kd*s[5];
	sprime[6] = p + r*cos(phi)*tan(theta) + q*sin(phi)*tan(theta);
	sprime[7] = q*cos(phi) - r*sin(phi);
	sprime[8] = (r*cos(phi) + q*sin(phi)) / cos(theta);
	u2 = q*r*(Iz - Iy)*Ix + Ix*(-sprime[6] * (kn1 + ko1) - (1 + kn1*ko1)*(phi - phit) + (sprime[8] * (kn3 + ko1) + (1 + kn3*ko1)*(psi - psit))*sin(theta));
	u3 = p*r*(Ix - Iz)*Iy + Iy*((-(sprime[7] * (kn2 + ko2) + (1 + kn2*ko2)*(theta - thetat)))*cos(phi) - (sprime[8] * (kn3 + ko2) + (1 + kn3*ko2)*(psi - psit))*cos(theta)*sin(phi));
	u4 = p*q*(Iy - Ix)*Iz + Iz*((-(sprime[8] * (kn3 + ko3) + (1 + kn3*ko3)*(psi - psit)))*cos(phi)*cos(theta) + (sprime[7] * (kn2 + ko3) + (1 + kn2*ko3)*(theta - thetat))*sin(phi));
	sprime[9] = (u2 + q*r*(Iy - Iz) - JTP*q*Omega) / Ix;
	sprime[10] = (u3 + p*r*(Iz - Ix) + JTP*p*Omega) / Iy;
	sprime[11] = (u4 + p*q*(Ix - Iy)) / Iz;

	return sprime;
}


void Display(double t, int n, double s[])
{
	cout << t << setw(15) << s[0] << setw(15) << s[1] << setw(15) << s[2] << endl;

	// Out file data
	ofstream OutData("Datos.dat", ios::app);

	OutData.width(20);
	OutData.setf(ios::scientific, ios::floatfield);
	OutData << t;

	for (int i = 0; i < n; i++)
	{
		// state variables
		OutData.width(20);
		OutData.setf(ios::scientific, ios::floatfield);
		OutData << s[i];
	}

	OutData << endl;
}


int FrecImp(double ti, double tf, double dt, int dp)
{
	double dblTemporal;
	int iGridPoints;

	// Total de puntos en la malla
	dblTemporal = ceil((tf - ti) / dt);
	iGridPoints = int(dblTemporal);
	if (dp < iGridPoints)
	{
		// Estimamos la frecuencia de salida
		dblTemporal = ceil(double(iGridPoints) / double(dp));
		iGridPoints = int(dblTemporal);
	}
	else
		// Se despliegan todos los puntos
		iGridPoints = 1;

	return iGridPoints;
}