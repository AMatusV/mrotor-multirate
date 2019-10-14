//#include <math.h>

#ifndef M_PI
#    define M_PI 3.14159265358979323846
#endif

  // Parametros del modelo del Cuadrotor

  const double Ix = 0.00223;
  const double Iy = 0.00299;
  const double Iz = 0.00480;
  const double m = 0.4875;
  const double g = 9.81;
  //const double l = 0.24;
  const double JTP = 0.0;   // neglected for now
  const double Omega = 1.0;
  const double kd = 0.1;
  //const double raizdos = sqrt(double(2.0));

  // Setpoints (valores de referecia)
  const double xt = 1.0;
  const double yt = 1.0;
  const double zt = 1.0;
  //const double psit = M_PI / double(6.0);
  const double psit = 0.0;


  // Valor de las ganancias del controlador de posición
  const double kp1 = 1.62;
  const double kp2 = 1.62;
  const double kp3 = 2.56;
  const double kv1 = 1.6;
  const double kv2 = 1.6;
  const double kv3 = 2.56;

  // Valor de las ganancias del controlador de orientación
  const double kn1 = 5.38;
  const double kn2 = 5.38;
  const double kn3 = 0.5;
  const double ko1 = 5.38;
  const double ko2 = 5.38;
  const double ko3 = 0.4;

  // Variables internas de control
  //double cx, cy, cz;
  //double phit, thetat;