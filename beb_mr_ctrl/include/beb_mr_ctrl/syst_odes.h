#ifndef SYST_ODES_H
#define SYST_ODES_H

#include <iostream>
#include <iomanip>
#include <fstream>
#include <math.h>
#include "params_quadrotor.h"

#ifndef CLAMP
#define CLAMP(x, l, h) (((x) > (h)) ? (h) : (((x) < (l)) ? (l) : (x)))
#endif

#ifndef FILTER_SMALL_VALS
#define FILTER_SMALL_VALS(x, eps) (x = ((fabs((x)) < (eps)) ? 0.0 : (x)))
#endif

namespace beb_mr_ctrl
{

class SystODEs
{
public:
	double dbly[12];		   // state variables
	double dblDydt[12];		// derivate state variables
	double dblFastStep;		// fast step
	double dblSlowStep;     // slow step
	double dblTime;			// actual Time
	double dblSimTime;		// Total simulation time
	int iSlowComp[8];       // slow components index
	int iFastComp[4];       // fast components index
	int iMultirateFactor;	// multirate factor
   int iNumberSlow;        // no. componentes lentas
   int iNumberFast;        // no. componentes rapidas
	int iDisplayPoints;     // number of point to display

   // AMV
   int iTotalEDOs;
   int iNumberInputs;
   double dblInputs[4];
   double foo;

   // Constructor;
	SystODEs(int iMfactor);
   // Forward Euler Multirate linear interpolation
	void ForwardEulerMultirateInt();
   // Forward Euler Multirate forward information
	void ForwardEulerMultirateAndvn();
   // Forward Euler Multirate backward information
	void ForwardEulerMultirateBack();
	void SlowSystem();
	void FastSystem();
	void Display();
	double TotalSimTime() const;
	double ActTime() const;
	int FrecImp();

   // AMV
   inline double * GetStates() {return dbly;}
   inline double * GetInputs() {return dblInputs;}
   inline int GetNumberStates() const {return iTotalEDOs;}
   void Reset(const double* states, const double* inputs);
   void Reset();
   void Simulate(const double& duration_s, const double& dt_s);
   void Simulate(const double& duration_s, const double& dt_s,
                 const double* states, const double* inputs);

};

}  // namespace bebop_mr_ctrl

#endif
