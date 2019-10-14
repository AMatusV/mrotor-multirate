// clMultiIntrp.cpp : Defines the entry point for the console application.

//#include <iostream>
//#include <iomanip>
//#include <fstream>
#include "beb_mr_ctrl/syst_odes.h"
//#include "beb_mr_ctrl/params_quadrotor.h"

//using namespace std;


namespace beb_mr_ctrl {

// AMV
SystODEs::SystODEs(int iMfactor)
{
   // Calculamos el total de edo's
   iMultirateFactor = iMfactor;
   iNumberInputs = 4;

   iNumberSlow = 8;
   iNumberFast = 4;    
   iTotalEDOs = iNumberSlow + iNumberFast;
    
	// Ponemos en ceros todas las derivadas
	for (int i = 0; i < iTotalEDOs; i++)
	{
		dbly[i] = 0.0;     // inittial condition state variable
		dblDydt[i] = 0.0;  // clean array of derivative
	}

   for (int i = 0; i < iNumberInputs; i++) 
      dblInputs[i] = 0.0;

	// Definimos los indices para las componentes lentas
	iSlowComp[0] = 0; iSlowComp[1] = 1; iSlowComp[2] = 2;
   iSlowComp[3] = 5; iSlowComp[4] = 6, iSlowComp[5] = 7;
	iSlowComp[6] = 8; iSlowComp[7] = 9;

	// Definimos los indices para las componentes rapidas
	iFastComp[0] = 3; iFastComp[1] = 4; iFastComp[2] = 10;
	iFastComp[3] = 11;
   
	// Calculamos el paso para el sistema rapido
	/*dblSlowStep = dblLstep;
	dblFastStep = dblSlowStep / double(iMfactor);

	dblTime = dbliTime;
	iDisplayPoints = iPuntos;
 	dblSimTime = dblStime;
	iMultirateFactor = iMfactor;*/
}

// AMV
void SystODEs::Reset(const double* states, const double* inputs)
{
   for (int i = 0; i < iTotalEDOs; i++) {
		dbly[i] = states[i];
		dblDydt[i] = 0.0;    // clean derivatives array
	}
   
   for (int i = 0; i < iNumberInputs; i++) {
      dblInputs[i] = inputs[i];
   }
}

// AMV
void SystODEs::Reset()
{
   for (int i = 0; i < iTotalEDOs; i++) {
		dbly[i] = 0.0;     // inittial condition state variable
		dblDydt[i] = 0.0;  // clean derivatives array
	}

   for (int i = 0; i < iNumberInputs; i++) {
      dblInputs[i] = 0.0;
   }
}

// AMV
void SystODEs::Simulate(const double& duration_s, const double& dt_s)
{
   if (duration_s < 0.0) {
      throw std::runtime_error("Negative duration");
   }

   dblTime = 0.0;
   dblSlowStep = dt_s;
   dblFastStep = dblSlowStep / double(iMultirateFactor);

   while (dblTime < duration_s) {
      ForwardEulerMultirateAndvn();
      //dblTime += dt_s;
   }
}

// AMV
void SystODEs::Simulate(const double& duration_s, const double& dt_s,
                        const double* states, const double* inputs)
{
  Reset(states, inputs);
  Simulate(duration_s, dt_s);
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
	double intrpConst[11], deltaTiempo;
	
	// We save initial and final time of the cycle tn to tn+k
	TimeSavei = dblTime;
	TimeSavef = dblTime + dblSlowStep;

	// we save state variables and derivatives of the slow system at tn
   for (int i = 0; i <= 7; i++)
	{
      ySlowsave[iSlowComp[i]] = dbly[iSlowComp[i]];        // state variables
		dydtSlowsave[iSlowComp[i]] = dblDydt[iSlowComp[i]];  // derivatives variables
	}

	// we integrate the slow system fron tn to tn+k
	for (int i = 0; i <= 7; i++)
		dbly[iSlowComp[i]] = dbly[iSlowComp[i]] + dblSlowStep * dblDydt[iSlowComp[i]];
	
	dblTime = TimeSavef; // actualizamos el tiempo para el sistema lento t = tn+k
	SlowSystem();        // actualizamos al tiempo final las derivadas sistema lento
	dblTime = TimeSavei; // regresamos el valor del tiempo al inicio del ciclo

	// Ponemos en un arreglo la parte que se mantiene constante en la interpolación
	deltaTiempo = TimeSavef - TimeSavei;
	for (int i = 0; i <= 7; i++)
	{
		intrpConst[i] = (( dblDydt[iSlowComp[i]] - dydtSlowsave[iSlowComp[i]]) / deltaTiempo);
	}
	
	for (int i = 0; i < iMultirateFactor; i++)  // begin cycle of integration
	{
        for (int j = 0; j <= 3; j++)
            // we integrate the fast componentes at tn+i
		    dbly[iFastComp[j]] += dblFastStep * dblDydt[iFastComp[j]];

		double tmp = dblTime + dblFastStep;  // iterpolation time
      for (int j = 0; j <= 7; j++)
		{
         // linear interpolation slow state variables
		   dbly[iSlowComp[j]] = ySlowsave[iSlowComp[j]] + double(i + 1) * dblFastStep * dydtSlowsave[iSlowComp[j]];
			// linear interpolation slow state derivative variable			
			dblDydt[iSlowComp[j]] = dydtSlowsave[iSlowComp[j]] + intrpConst[j] * (tmp - TimeSavei);
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
// Strategy based on forward (advance) information
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

void SystODEs::ForwardEulerMultirateAndvn()  // AMV
{

    for (int i = 0; i < iNumberSlow; i++)
        // Slow sytem
        //dbly[i] += double(iMultirateFactor) * dblFastStep * dblDydt[i];
	    dbly[iSlowComp[i]] += dblSlowStep * dblDydt[iSlowComp[i]];  // Forward Euler


	for (int i = 0; i < iMultirateFactor; i++)  // begin cycle
	{
		// Fast System
      for (int j = 0; j < iNumberFast; j++)
		    dbly[iFastComp[j]] += dblFastStep * dblDydt[iFastComp[j]];   // fast component

		dblTime += dblFastStep;
      FastSystem();                       // update fast system
		
	} // end cycle

	// Actualizamos las derivadas del sistema lento
	SlowSystem();
	// No es necesario actualizar el rapido
}


////////////////////////////////////////////////////////////////////
//
// Forward Euler Multirate
// Strategy based on backward information
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
	for (int i = 0; i < iMultirateFactor; i++) // begin cycle
	{
        for (int j = 0; j <= 3; j++)
            // integrate fast component
		    dbly[iFastComp[j]] += dblFastStep * dblDydt[iFastComp[j]];

		// updtate time
		dblTime += dblFastStep;
		FastSystem();   // update fast system		
	} // end cycle

    for (int i = 0; i <= 7; i++)
        // we integrate slow component
	    //dbly[i] += double(iMultirateFactor) * dblFastStep * dblDydt[i];
	    dbly[iSlowComp[i]] += dblSlowStep * dblDydt[iSlowComp[i]];

	// Update systems
	SlowSystem();
	//FastSystem();
}


void SystODEs::Display()
{
	std::cout << dblTime << std::setw(15) << dbly[0] << std::setw(15) << dbly[1] << std::setw(15) 
             << dbly[2] << std::endl;

	// Out file data
	std::ofstream OutData("Datos.dat", std::ios::app);

	OutData.width(20);
	OutData.setf(std::ios::scientific, std::ios::floatfield);
	OutData << dblTime;

	for (int i = 0; i < 12; i++)
	{
		// state variables
		OutData.width(20);
		OutData.setf(std::ios::scientific, std::ios::floatfield);
		OutData << dbly[i];
	}

	OutData << std::endl;
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

}  // namespace bebop_mr_ctrl

