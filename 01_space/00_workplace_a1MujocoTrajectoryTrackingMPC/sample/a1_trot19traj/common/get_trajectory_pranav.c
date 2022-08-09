void get_trajectory(double ttime,double ts,double tend,
                    double *xref,double *yref,
                    double *xdotref,double *ydotref,
                    double *thetaref,double *thetadotref,
                    double *theta_0,double a,double x_center,double y_center,
                    double *x_ref_temp, double *y_ref_temp)
{
  double tau = 2*pi*(ttime-ts)/tend;
  double  den = 1+sin(tau)*sin(tau);
  double den2 = den*den;
  double b = 2*pi/tend;
  *xref = x_center + a*cos(tau)/den; //--- pranav origin
  *yref = y_center + a*cos(tau)*sin(tau)/den; //--- pranav origin

  // *xref = -1*(x_center + a*cos(tau)/den); // chunming --- flip the curve
  // *yref = -1*(y_center + a*cos(tau)*sin(tau)/den); //chunming --- flip the curve

  // *xref = -1*(x_center + a*cos(tau)/den + 1); // chunming --- flip the curve and move to positive area
  // *yref = -1*(y_center + a*cos(tau)*sin(tau)/den + 1); //chunming --- flip the curve and move to positive area

  // *xref = -1*(x_center + a*cos(tau)/den + 0.012134);// chunming --- verify the influence made by the initial guess
  // *yref = -1*(y_center + a*cos(tau)*sin(tau)/den - 0.016342); // chunming --- verify the influence made by the initial guess
 
  *xdotref = (a*b*sin(tau)*(sin(tau)*sin(tau) - 3))/den2;
  *ydotref = -(a*b*(3*sin(tau)*sin(tau) - 1))/den2;

  double xdot = *xdotref;
  double ydot = *ydotref;
  double epsilon = 1e-6;
  double xddot = a*(b*b)*cos(tau)*1.0/pow(pow(cos(tau),2.0)-2.0,3.0)*(pow(cos(tau),2.0)*1.0E+1+pow(cos(tau),4.0)-8.0);
  double yddot = a*(b*b)*cos(tau)*sin(tau)*1.0/pow(pow(sin(tau),2.0)+1.0,3.0)*(pow(cos(tau),2.0)*3.0+2.0)*-2.0;

  double theta = atan2(ydot,xdot);
  double sec_theta = 1/cos(theta);
  double num3 = xdot*yddot - ydot*xddot;
  double den3 = (xdot*xdot+epsilon);
  double thetadot;

  //constant reference
    // *thetaref = M_PI/2;
    // *thetadotref = 0;
    if (flag_constant)
      thetadot = 0;
    else
      thetadot = (1/(sec_theta*sec_theta))*(num3/den3);

    //changing reference
   *thetadotref = thetadot;
   *theta_0 = *theta_0 + t_stance*thetadot;
   *thetaref = *theta_0;
   //printf("%f %f %f \n", *thetaref,*thetadotref,theta);

   // chunming --- add poly traj. here


}
