// clMultiIntrp.cpp : Defines the entry point for the console application.
//

#include <iostream>
#include <iomanip>
#include <fstream>
#include <math.h>
#include "SystODEs.h"
#include "parametrosQuadrotor.h"

using namespace std;


// Constructor
SystODEs::SystODEs()
{
    // Default constructor

   // Calculamos el total de edo's
    initFast = 5;
    iendFast = 7;
    initSlow = 0;
    iendSlow = 4;

    // Calculamos el total de edo's
    int iTotalEDOs = (iendFast - initFast + 1) + (iendSlow - initSlow + 1);
	// Initial conditions
	// Initial conditions
    dbly[0] = 0.0;               //Valor de la condicion inicial 1
    dbly[1] = 0.0;               //Valor de la condicion inicial 2
    dbly[2] = 0.785398163397448; //Valor de la condicion inicial 3 (pi/4)
    dbly[3] = 0.785398163397448; //Valor de la condicion inicial 4
    dbly[4] = 0.785398163397448; //Valor de la condicion inicial 5
    dbly[5] = 0.0;               //Valor de la condicion inicial 6
    dbly[6] = 0.0;               //Valor de la condicion inicial 6
    dbly[7] = 0.0;               //Valor de la condicion inicial 8


	dblSlowStep = 0.1;
	iMultirateFactor = 8;
	dblFastStep = dblSlowStep / double(iMultirateFactor);
	dblTime = 0.0;
	dblSimTime = 5.0;
	iDisplayPoints = 100;	

}


// Constructor
SystODEs::SystODEs(double dblLstep, double dbliTime, double dblStime,
                   int iFast, int ifFast, int iSlow, int ifSlow,
				   int iMfactor, int iPuntos)
{

    // Calculamos el total de edo's
    initFast = iFast;
    iendFast = ifFast;
    initSlow = iSlow;
    iendSlow = ifSlow;
    
    int iTotalEDOs = (iendFast - initFast + 1) + (iendSlow - initSlow + 1);

	// Initial conditions
    dbly[0] = 0.0;               //Valor de la condicion inicial 1
    dbly[1] = 0.0;               //Valor de la condicion inicial 2
    dbly[2] = 0.0;				 //Valor de la condicion inicial 3 
    dbly[3] = 0.0;				 //Valor de la condicion inicial 4
    dbly[4] = 0.0;				 //Valor de la condicion inicial 5
    dbly[5] = 0.0;               //Valor de la condicion inicial 6
    dbly[6] = 0.0;               //Valor de la condicion inicial 7 0.785398163397448 (pi/4)
    dbly[7] = 0.0;               //Valor de la condicion inicial 8
	dbly[8] = 0.0;				 //Valor de la condicion inicial 9
	dbly[9] = 0.0;               //Valor de la condicion inicial 10
	dbly[10] = 0.0;               //Valor de la condicion inicial 11
	dbly[11] = 0.0;               //Valor de la condicion inicial 12

	// Ponemos en ceros todas las derivadas
	for (int i = 0; i < iTotalEDOs; i++)
	{
		dblDydt[i] = 0.0;
	}

	// Calculamos el paso para el sistema rapido
	dblSlowStep = dblLstep;
	dblFastStep = dblSlowStep / double(iMfactor);

	dblTime = dbliTime;
	iDisplayPoints = iPuntos;
 	dblSimTime = dblStime;
	iMultirateFactor = iMfactor;
}


double SystODEs::ActTime() const
{
	// Actual simulation time
	return dblTime;
}


double SystODEs::TotalSimTime() const
{
	// Total simulation time
	return dblSimTime;

}


////////////////////////////////////////////////////////////////////
//
// Forward Euler Multirate
// Strategy based on linear interpolation
// Esta versión interpola las derivadas de la variables de estado
// This version interpolate the derivative of the state variables.
//
// Inputs:
//
//		iMultirateFactor         multirate factor
//		dblTime					 actual simulation time
//		dblFastStep              fast integration step
//		dbly                     fast component
//		dblz                     slow component
//		dblDydt                  derivate fast component
//		dblDzdt                  derivate slow component
//
// Output:
//
//		dblTime                  actual simulation time
//		dbly                     fast component
//		dblz                     slow component
//		dblDydt                  derivate fast component
//		dblDzdt                  derivate slow component
//
//////////////////////////////////////////////////////////////////////
void SystODEs::ForwardEulerMultirateInt()
{
	  
    double ySlowsave[11], dydtSlowsave[11], TimeSavei, TimeSavef;
	
	// We save initial and final time of the cycle tn to tn+k
	TimeSavei = dblTime;
	TimeSavef = dblTime + dblSlowStep;

	// we save state variables and derivatives of the slow system at tn
    for (int i = initSlow; i <= iendSlow; i++)
	{
        ySlowsave[i] = dbly[i];        // state variables
		dydtSlowsave[i] = dblDydt[i];  // derivatives variables
	}

	// we integrate the slow sytem fron tn to tn+k 
	for (int i = initSlow; i <= iendSlow; i++)
		dbly[i] = dbly[i] + dblSlowStep * dblDydt[i];
		
	SlowSystem(); // actualizamos al tiempo final las derivadas

	/*for ( int i = initSlow; i <= iendSlow; i++)
	    cout << "No. derivada = " << setw(15)<< i <<
		"Derivada en tiempo final = "<< setw(15) << dblDydt[i] << endl;
	*/


	for (int i = 0; i < iMultirateFactor; i++)  // begin cycle of integration
	{
        for (int j = initFast; j <= iendFast; j++)
            // we integrate the fast componentes at tn+i
		    dbly[j] += dblFastStep * dblDydt[j];

		double tmp = dblTime + dblFastStep;  // iterpolation time
        for (int j = initSlow; j <= iendSlow; j++)
		{
            // linear interpolation slow components
		    dbly[j] = ySlowsave[j] + double(i + 1) * dblFastStep * dydtSlowsave[j];
			// linear interpolation derivatives slow componentes			
			dblDydt[j] = dydtSlowsave[j] + (( dblDydt[j] - dydtSlowsave[j]) / (TimeSavef - TimeSavei)) * (tmp - TimeSavei);

			//cout << "Factor multirate = " << i 
			//	 << " Derivada interpolada " << setw(10) << j << setw(15) << dblDydt[j] << endl;
		}

        dblTime += dblFastStep; // update time
		FastSystem();	// update fast system
	} // end cycle

	// Update slow system
	//SlowSystem();
}


////////////////////////////////////////////////////////////////////
//
// Forward Euler Multirate
// Strategy based on advance information
// Inputs:
//  k         multirate factor
//  Tiempo    actual simulation time
//  h         fast integration step
//  y         fast component
//  z         slow component
//  dydt      derivate fast component
//  dzdt      derivate slow component
//
//  Output:
//  Tiempo    actual simulation time
//  y         fast component
//  z         slow component
//  dydt      derivate fast component
//  dzdt      derivate slow component
void SystODEs::ForwardEulerMultirateAndvn()
{
	int i;
    int j;

    for (i = initSlow; i <= iendSlow; i++)
        // advance information of slow sytem
        dbly[i] += double(iMultirateFactor) * dblFastStep * dblDydt[i];

	for (i = 0; i < iMultirateFactor; i++)  // begin cycle
	{
        for (j = initFast; j <= iendFast; j++)
		    dbly[j] += dblFastStep * dblDydt[j];   // fast component

		dblTime += dblFastStep;
        FastSystem();                       // update fast system
		
	} // end cycle

	// Update systems
	SlowSystem();
	//FastSystem();    // creo que no es necesario esta llamada
}


////////////////////////////////////////////////////////////////////
//
// Forward Euler Multirate
// Strategy based on back information
// Inputs:
//  k         multirate factor
//  Tiempo    actual simulation time
//  h         fast integration step
//  y         fast component
//  z         slow component
//  dydt      derivate fast component
//  dzdt      derivate slow component
//
//  Output:
//  Tiempo    actual simulation time
//  y         fast component
//  z         slow component
//  dydt      derivate fast component
//  dzdt      derivate slow component
void SystODEs::ForwardEulerMultirateBack()
{
	int i;
    int j;

	for (i = 0; i < iMultirateFactor; i++) // begin cycle
	{
        for (j = initFast; j <= iendFast; j++)
            // integrate fast component
		    dbly[j] += dblFastStep * dblDydt[j];

		// updtate time
		dblTime += dblFastStep;
		FastSystem();   // update fast system		
	} // end cycle

    for (i = initSlow; i <= iendSlow; i++)
        // we integrate slow component
	    dbly[i] += double(iMultirateFactor) * dblFastStep * dblDydt[i];

	// Update systems
	SlowSystem();
	//FastSystem();
}


void SystODEs::Display()
{
	cout << dblTime << setw(15) << dbly[0] << setw(15) << dbly[1] << setw(15) << dbly[2] << endl;

	// Out file data
	ofstream OutData("Datos.dat", ios::app);

	OutData.width(20);
	OutData.setf(ios::scientific, ios::floatfield);
	OutData << dblTime;

	for (int i = 0; i < 12; i++)
	{
		// state variables
		OutData.width(20);
		OutData.setf(ios::scientific, ios::floatfield);
		OutData << dbly[i];
	}

	OutData << endl;
}


void SystODEs::FastSystem()
{
    //#include "parametrosQuadrotor.h"

	double phi, theta, psi, p, q, r;
	double U2, U3, U4;
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

	U2 = q*r*(Iz - Iy)*Ix + Ix*(-dblDydt[6] * (kn1 + ko1) - (1 + kn1*ko1)*(phi - phit) + (dblDydt[8] * (kn3 + ko1) + (1 + kn3*ko1)*(psi - psit))*sin(theta));
	U3 = p*r*(Ix - Iz)*Iy + Iy*((-(dblDydt[7] * (kn2 + ko2) + (1 + kn2*ko2)*(theta - thetat)))*cos(phi) - (dblDydt[8] * (kn3 + ko2) + (1 + kn3*ko2)*(psi - psit))*cos(theta)*sin(phi));
	U4 = p*q*(Iy - Ix)*Iz + Iz*((-(dblDydt[8] * (kn3 + ko3) + (1 + kn3*ko3)*(psi - psit)))*cos(phi)*cos(theta) + (dblDydt[7] * (kn2 + ko3) + (1 + kn2*ko3)*(theta - thetat))*sin(phi));
	dblDydt[9] = (U2 + q*r*(Iy - Iz) - JTP*q*Omega) / Ix; 
	dblDydt[10] = (U3 + p*r*(Iz - Ix) + JTP*p*Omega) / Iy;
	dblDydt[11] = (U4 + p*q*(Ix - Iy)) / Iz;

}


void SystODEs::SlowSystem()
{
    //#include "parametrosQuadrotor.h"

	double x, y, z, phi, theta, psi, p, q, r;
	double U1, cx, cy, cz, thetat, phit;

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
	cx = m*((1 + kp1*kv1)*(xt - x) - (kp1 + kv1)*dblDydt[0]);
	cy = m*((1 + kp2*kv2)*(yt - y) - (kp2 + kv2)*dblDydt[1]);
	cz = m*((1 + kp3*kv3)*(zt - z) - (kp3 + kv3)*dblDydt[2]);
	thetat = atan2(cy*sin(psi) + cx*cos(psi), cz + m*g);
	phit = atan2((cx*sin(psi) - cy*cos(psi))*cos(thetat), cz + m*g);
	U1 = (cz + m*g) / (cos(thetat)*cos(phit));
	dblDydt[3] = (sin(psi)*sin(phi) + cos(psi)*sin(theta)*cos(phi))*U1/m - kd*dbly[3];
	dblDydt[4] = (sin(psi)*sin(theta)*cos(phi) - cos(psi)*sin(phi))*U1/m - kd*dbly[4];
	dblDydt[5] = cos(theta)*cos(phi)*U1/m - g - kd*dbly[5];
	dblDydt[6] = p + r*cos(phi)*tan(theta) + q*sin(phi)*tan(theta);
	dblDydt[7] = q*cos(phi) - r*sin(phi);
	dblDydt[8] = (r*cos(phi) + q*sin(phi)) / cos(theta);

}


int SystODEs::FrecImp()
{
	double dblTemporal;
	int iGridPoints;

	// Total de puntos en la malla
    dblTemporal = ceil((dblSimTime - dblTime) / (iMultirateFactor*dblFastStep));
	iGridPoints = int(dblTemporal);
	if (iDisplayPoints < iGridPoints )
	{
		// Estimamos la frecuencia de salida
		dblTemporal = ceil(double(iGridPoints) / double(iDisplayPoints));
		iGridPoints = int(dblTemporal);
	}
	else
		// Se despliegan todos los puntos
		iGridPoints = 1;

	return iGridPoints;
}
