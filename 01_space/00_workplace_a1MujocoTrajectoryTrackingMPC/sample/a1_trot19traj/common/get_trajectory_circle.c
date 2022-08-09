void get_trajectory(double ttime, double ts, double tend,
                    double *x_ref, double *y_ref,
                    double *xdot_ref, double *ydot_ref)
{
    double radius_circle = 1;
    double omega_circle = pi/(tend - ts);
    double v_circle = omega_circle*radius_circle;
    double theta_circle = omega_circle*ttime;

    *x_ref = radius_circle - cos(theta_circle);
    *y_ref = sin(theta_circle);
    *xdot_ref = v_circle*sin(theta_circle);
    *ydot_ref = v_circle*cos(theta_circle);
}