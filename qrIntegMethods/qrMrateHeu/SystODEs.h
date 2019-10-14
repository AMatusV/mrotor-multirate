class SystODEs
{
  public:
	double dbly[12];		   // state variables
	double dblDydt[12];		   // derivate state variables
	double dblFastStep;		   // fast step
	double dblSlowStep;        // slow step
	double dblTime;			   // actual Time
	double dblSimTime;		   // Total simulation time
	int iMultirateFactor;	   // multirate factor
    int iNumberSlow;           // no. componentes lentas
    int iNumberFast;           // no. componentes rapidas
	int iDisplayPoints;        // number of point to display
    int initFast;
    int iendFast;
    int initSlow;
    int iendSlow;
   //public:
	SystODEs();	// defaul constructor
    // contructor
	SystODEs(double, double, double, int, int, int, int, int, int);
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
};
