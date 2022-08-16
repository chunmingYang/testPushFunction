void get_trajectory(double ttime,double ts,double tend,
                    double *xref,double *yref, double *thetaref,
                    double *xdot_ref, double *ydot_ref, double *thetadot_ref)
{
    double t0 = ts;
    double tf = tend;
    double y0 = 0;
    double yf = 1;
    double a0 = (yf*t0*t0*(3*tf-t0) + y0*tf*tf*(tf-3*t0))/((tf-t0)*(tf-t0)*(tf-t0));
    double a1 = 6*t0*tf*(y0-yf)/((tf-t0)*(tf-t0)*(tf-t0));
    double a2 = 3*(t0+tf)*(yf-y0)/((tf-t0)*(tf-t0)*(tf-t0));
    double a3 = 2*(y0-yf)/((tf-t0)*(tf-t0)*(tf-t0));
    double y = a0 + a1*ttime + a2*ttime*ttime + a3*ttime*ttime*ttime;
    double ydot = a1 + 2*a2*ttime + 3*a3*ttime*ttime;
    *xref = y;
    *yref = y;
    *thetaref = 1.57;

    *xdot_ref = ydot;
    *ydot_ref = ydot;
    *thetadot_ref = 0;
}                    