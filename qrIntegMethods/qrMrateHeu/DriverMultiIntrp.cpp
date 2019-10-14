
#include <iostream>
#include <iomanip>
#include <fstream>
#include <math.h>
//#include <time.h>
#include <chrono> 
#include "SystODEs.h"

using namespace std;
using namespace std::chrono;


int main(int argc, char* argv[])
{
	ofstream ofs;
	ofs.open("Datos.dat", ofstream::out | ofstream::trunc);
	ofs.close();

	// Construcctor parameters:
	//             1) integration step of the slow system,
	//             2) Simulation initial time,
	//             3) Simulation final time,
	//             4) Position of the first ode of the fast system,
	//             5) Position of the last ode of the fast system,
	//             6) Position of the first ode of the slow system,
	//             7) Position of the last ode of the slow system,
	//             8) Multirate factor,
	//             9) Number of variables that you want to display

	SystODEs Quadrotor(0.01, 0.0, 5.0, 9, 11, 0, 8, 4, 6000);
	//SystODEs Quadrotor(0.004, 0.0, 5.0, 9, 11, 0, 8, 4, 625);
	//SystODEs Quadrotor(0.002, 0.0, 5.0, 9, 11, 0, 8, 1, 625);

	double dblTemporal;
	int iMetodo = 1;	// 1 : Euler linear interpolation
	                    // 2 : Euler forward information
	                    // 3 : Euler back information
	int iGridPoints;
	int iOutFrecuency = 0;

	//  Set output formatting.
    cout.setf(ios::scientific);     // Use scientific notation.
    cout.precision(6);              // Show six digits.

	// Estimamos la frecuencia de impresion
	// en funcion del numero de datos que
	// requiere el usuario

	iGridPoints = Quadrotor.FrecImp();
	//iGridPoints = 1;

	//clock_t begin = clock();
	auto start = high_resolution_clock::now();

    // Evaluamos al tiempo inicial
	Quadrotor.SlowSystem();
	Quadrotor.FastSystem();

	// Impresion de las condiciones iniciales
	//Quadrotor.Display();

	// Simulation
	dblTemporal = Quadrotor.TotalSimTime();
	while (Quadrotor.ActTime() <= dblTemporal)	//Ciclo simulacion
	{
		/*switch(iMetodo)
		{
			case 1:
				// Linear interpolation
				Quadrotor.ForwardEulerMultirateInt();
				break;
			case 2:
				// Forward information
				Quadrotor.ForwardEulerMultirateAndvn();
				break;
			case 3:
				// Back information
				Quadrotor.ForwardEulerMultirateBack();
			default:
				break;
		}

		iOutFrecuency++;			// Frecuencia de salida de datos
		if (iOutFrecuency == iGridPoints)
		{
			// Analytical solution and relative error
			Quadrotor.Display();
			iOutFrecuency = 0;
		}*/

		Quadrotor.ForwardEulerMultirateInt();
		//Quadrotor.ForwardEulerMultirateBack();
		//Quadrotor.ForwardEulerMultirateAndvn();

	} //end cycle

	//clock_t end = clock();
	//cout << "Time elapsed: " << double((end - begin)*1000/CLOCKS_PER_SEC) << " ms \n\n";
	auto stop = high_resolution_clock::now();
	auto duration = duration_cast<microseconds>(stop - start);
	cout << "Time elapsed: " << duration.count() << " microsec \n\n";

	getchar();
	return 0;
}
