#include <stdio.h>
#include <math.h>
#define pi M_PI
#include "nlopt.h"
#include <sys/time.h>

// MPC parameters
#define ts 0.002  // step time
#define T 5  // receding horizon
#define iter 60/0.002  // iteration timess
double Q[9] = {10,0,0, 0,10,0, 0,0,10};  // quadrutic function parameters can be tuned
double R[9] = {0,0,0, 0,0,0, 0,0,0};  // quadrutic function parameters can be tuned

// initialization
double X[3] = {0, -0.2, 0};  // states initialization (here choose the "center")
double Xref[3] = {0, 0, 0};
double U[15] = {};  // we have the special index format here (counts by column)

// save data initialization
FILE *fid;
char path[] = "../sample/data.csv";

// data saving function initialization --- adding labels
void init_save_data()
{
    fprintf(fid, "x,y,theta,speed,xref,yref");
    fprintf(fid, "\n");
}

// data saving main function
void save_data(double x, double y, double theta, double speed, double xref, double yref)
{
    fprintf(fid, "%f, %f, %f, %f, %f, %f", x, y, theta, speed, xref, yref);
    fprintf(fid, "\n");
}

// cost function setup for nlopt optimizer
double mycost(unsigned n, const double *U)
{
    // manual iteration initialization --- the total iteration time decided by the receding horizon
    double XX[3] = {X[0], X[1], X[2]}; // using local variables to avoid error
    double X_Xref[3];                  // X-Xref
    double mul[3];                     // transpose(X-Xref)*Q
    double cost;                       // cost initialization
    double cost_state;                 // transpose(X-Xref)*Q*(X-Xref)
    double A[9];                       // this diff car model doesn't have A matrix
    double B[9];                       // B matrix of the state space model of diff car
    double B_U[3];                     // B*U
    double Xdot[3];                    // state velocity for the states updating
    
    // iteration 01
    X_Xref[0] = XX[0]-Xref[0];
    X_Xref[1] = XX[1]-Xref[1];
    X_Xref[2] = XX[2]-Xref[2];
    mul[0] = X_Xref[0]*Q[0]+X_Xref[1]*Q[3]+X_Xref[2]*Q[6];
    mul[1] = X_Xref[0]*Q[1]+X_Xref[1]*Q[4]+X_Xref[2]*Q[7];
    mul[2] = X_Xref[0]*Q[2]+X_Xref[1]*Q[5]+X_Xref[2]*Q[8];
    cost_state = X_Xref[0]*mul[0]+mul[1]*X_Xref[1]+mul[2]*X_Xref[2];
    cost = cost + cost_state;
    B[0] = cos(XX[2]);
    B[1] = -sin(XX[2]);
    B[2] = 0;
    B[3] = sin(XX[2]);
    B[4] = cos(XX[2]);
    B[5] = 0;
    B[6] = 0;
    B[7] = 0;
    B[8] = 1;
    B_U[0] = B[0]*U[0]+B[1]*U[1]+B[2]*U[2];
    B_U[1] = B[3]*U[0]+B[4]*U[1]+B[5]*U[2];
    B_U[2] = B[6]*U[0]+B[7]*U[1]+B[8]*U[2];
    Xdot[0] = B_U[0];
    Xdot[1] = B_U[1];
    Xdot[2] = B_U[2];
    XX[0] = XX[0]+ts*Xdot[0];
    XX[1] = XX[1]+ts*Xdot[1];
    XX[2] = XX[2]+ts*Xdot[2];

    // iteration 02
    X_Xref[0] = XX[0]-Xref[0];
    X_Xref[1] = XX[1]-Xref[1];
    X_Xref[2] = XX[2]-Xref[2];
    mul[0] = X_Xref[0]*Q[0]+X_Xref[1]*Q[3]+X_Xref[2]*Q[6];
    mul[1] = X_Xref[0]*Q[1]+X_Xref[1]*Q[4]+X_Xref[2]*Q[7];
    mul[2] = X_Xref[0]*Q[2]+X_Xref[1]*Q[5]+X_Xref[2]*Q[8];
    cost_state = X_Xref[0]*mul[0]+mul[1]*X_Xref[1]+mul[2]*X_Xref[2];
    cost = cost + cost_state;
    B[0] = cos(XX[2]);
    B[1] = -sin(XX[2]);
    B[2] = 0;
    B[3] = sin(XX[2]);
    B[4] = cos(XX[2]);
    B[5] = 0;
    B[6] = 0;
    B[7] = 0;
    B[8] = 1;
    B_U[0] = B[0]*U[3]+B[1]*U[4]+B[2]*U[5];
    B_U[1] = B[3]*U[3]+B[4]*U[4]+B[5]*U[5];
    B_U[2] = B[6]*U[3]+B[7]*U[4]+B[8]*U[5];
    Xdot[0] = B_U[0];
    Xdot[1] = B_U[1];
    Xdot[2] = B_U[2];
    XX[0] = XX[0]+ts*Xdot[0];
    XX[1] = XX[1]+ts*Xdot[1];
    XX[2] = XX[2]+ts*Xdot[2];

    // iteration 03
    X_Xref[0] = XX[0]-Xref[0];
    X_Xref[1] = XX[1]-Xref[1];
    X_Xref[2] = XX[2]-Xref[2];
    mul[0] = X_Xref[0]*Q[0]+X_Xref[1]*Q[3]+X_Xref[2]*Q[6];
    mul[1] = X_Xref[0]*Q[1]+X_Xref[1]*Q[4]+X_Xref[2]*Q[7];
    mul[2] = X_Xref[0]*Q[2]+X_Xref[1]*Q[5]+X_Xref[2]*Q[8];
    cost_state = X_Xref[0]*mul[0]+mul[1]*X_Xref[1]+mul[2]*X_Xref[2];
    cost = cost + cost_state;
    B[0] = cos(XX[2]);
    B[1] = -sin(XX[2]);
    B[2] = 0;
    B[3] = sin(XX[2]);
    B[4] = cos(XX[2]);
    B[5] = 0;
    B[6] = 0;
    B[7] = 0;
    B[8] = 1;
    B_U[0] = B[0]*U[6]+B[1]*U[7]+B[2]*U[8];
    B_U[1] = B[3]*U[6]+B[4]*U[7]+B[5]*U[8];
    B_U[2] = B[6]*U[6]+B[7]*U[7]+B[8]*U[8];
    Xdot[0] = B_U[0];
    Xdot[1] = B_U[1];
    Xdot[2] = B_U[2];
    XX[0] = XX[0]+ts*Xdot[0];
    XX[1] = XX[1]+ts*Xdot[1];
    XX[2] = XX[2]+ts*Xdot[2];

    // iteration 04
    X_Xref[0] = XX[0]-Xref[0];
    X_Xref[1] = XX[1]-Xref[1];
    X_Xref[2] = XX[2]-Xref[2];
    mul[0] = X_Xref[0]*Q[0]+X_Xref[1]*Q[3]+X_Xref[2]*Q[6];
    mul[1] = X_Xref[0]*Q[1]+X_Xref[1]*Q[4]+X_Xref[2]*Q[7];
    mul[2] = X_Xref[0]*Q[2]+X_Xref[1]*Q[5]+X_Xref[2]*Q[8];
    cost_state = X_Xref[0]*mul[0]+mul[1]*X_Xref[1]+mul[2]*X_Xref[2];
    cost = cost + cost_state;
    B[0] = cos(XX[2]);
    B[1] = -sin(XX[2]);
    B[2] = 0;
    B[3] = sin(XX[2]);
    B[4] = cos(XX[2]);
    B[5] = 0;
    B[6] = 0;
    B[7] = 0;
    B[8] = 1;
    B_U[0] = B[0]*U[9]+B[1]*U[10]+B[2]*U[11];
    B_U[1] = B[3]*U[9]+B[4]*U[10]+B[5]*U[11];
    B_U[2] = B[6]*U[9]+B[7]*U[10]+B[8]*U[11];
    Xdot[0] = B_U[0];
    Xdot[1] = B_U[1];
    Xdot[2] = B_U[2];
    XX[0] = XX[0]+ts*Xdot[0];
    XX[1] = XX[1]+ts*Xdot[1];
    XX[2] = XX[2]+ts*Xdot[2];

    // iteration 05
    X_Xref[0] = XX[0]-Xref[0];
    X_Xref[1] = XX[1]-Xref[1];
    X_Xref[2] = XX[2]-Xref[2];
    mul[0] = X_Xref[0]*Q[0]+X_Xref[1]*Q[3]+X_Xref[2]*Q[6];
    mul[1] = X_Xref[0]*Q[1]+X_Xref[1]*Q[4]+X_Xref[2]*Q[7];
    mul[2] = X_Xref[0]*Q[2]+X_Xref[1]*Q[5]+X_Xref[2]*Q[8];
    cost_state = X_Xref[0]*mul[0]+mul[1]*X_Xref[1]+mul[2]*X_Xref[2];
    cost = cost + cost_state;
    B[0] = cos(XX[2]);
    B[1] = -sin(XX[2]);
    B[2] = 0;
    B[3] = sin(XX[2]);
    B[4] = cos(XX[2]);
    B[5] = 0;
    B[6] = 0;
    B[7] = 0;
    B[8] = 1;
    B_U[0] = B[0]*U[12]+B[1]*U[13]+B[2]*U[14];
    B_U[1] = B[3]*U[12]+B[4]*U[13]+B[5]*U[14];
    B_U[2] = B[6]*U[12]+B[7]*U[13]+B[8]*U[14];
    Xdot[0] = B_U[0];
    Xdot[1] = B_U[1];
    Xdot[2] = B_U[2];
    XX[0] = XX[0]+ts*Xdot[0];
    XX[1] = XX[1]+ts*Xdot[1];
    XX[2] = XX[2]+ts*Xdot[2];


    // X_Xref[0] = XX[0]-Xref[0];
    // X_Xref[1] = XX[1]-Xref[1];
    // X_Xref[2] = XX[2]-Xref[2];
    // mul[0] = X_Xref[0]*Q[0]+X_Xref[1]*Q[3]+X_Xref[2]*Q[6];
    // mul[1] = X_Xref[0]*Q[1]+X_Xref[1]*Q[4]+X_Xref[2]*Q[7];
    // mul[2] = X_Xref[0]*Q[2]+X_Xref[1]*Q[5]+X_Xref[2]*Q[8];
    // cost_state = X_Xref[0]*mul[0]+mul[1]*X_Xref[1]+mul[2]*X_Xref[2];
    // cost = cost + cost_state;
    return cost;
}

// simple poly traj.
// void get_trajectory(double *xref, double *yref, double *thetaref, double ttime,
// double *xdotref)
// {
//     double t0 = 0;
//     double tf = ts*iter;
//     double y0 = 0;
//     double yf = 0.9;
//     double a0 = (yf*t0*t0*(3*tf-t0) + y0*tf*tf*(tf-3*t0))/((tf-t0)*(tf-t0)*(tf-t0));
//     double a1 = 6*t0*tf*(y0-yf)/((tf-t0)*(tf-t0)*(tf-t0));
//     double a2 = 3*(t0+tf)*(yf-y0)/((tf-t0)*(tf-t0)*(tf-t0));
//     double a3 = 2*(y0-yf)/((tf-t0)*(tf-t0)*(tf-t0));
//     double x = 0;
//     double y = a0 + a1*ttime + a2*ttime*ttime + a3*ttime*ttime*ttime;
//     double ydot = a1 + 2*a2*ttime + 3*a3*ttime*ttime;
//     *xref = y;
//     *xdotref = ydot;
//     *yref = 0;
//     *thetaref = 0;
// }

// pranav's "8" traj. --- provide x_ref, y_ref
void get_trajectory(double ttime, double *xref, double *yref, double *xdotref, double *ydotref)
{
    double tstart = 0;
    double tend = ts*iter;
    double tau = 2*pi*(ttime-tstart)/tend;
    double den = 1+sin(tau)*sin(tau);
    double den2 = den*den;
    double b = 2*pi/tend;
    double a = 2;  // from pranv's code, variavle for "8" traj.
    double x_center = -a;  // from pranv's code, variavle for "8" traj.
    double y_center = 0;// from pranv's code, variavle for "8" traj.
    *xref = x_center + a*cos(tau)/den;
    *yref = y_center + a*cos(tau)*sin(tau)/den - 0.2;
    *xdotref = (a*b*sin(tau)*(sin(tau)*sin(tau) - 3))/den2;
    *ydotref = -(a*b*(3*sin(tau)*sin(tau) - 1))/den2;
}

// optimization main loop
void main()
{
    // some initialization iteration states update and nlopt parameters
    double B[9] = {};
    double B_U[3] = {};
    double Xdot[3] = {};
    unsigned n = 15;  // number of decision variables
    // double lb[] = {-50,-50,-50,   -50,-50,-50,   -50,-50,-50,   -50,-50,-50,   -50,-50,-50};  // low bound for decision variables
    // double ub[] = {50,50,50,   50,50,50,   50,50,50,   50,50,50,   50,50,50};  // up bound for decision variables
    double lb_var = -0.15;
    double up_var = 0.15;
    double lb[] = {lb_var,lb_var,lb_var,   lb_var,lb_var,lb_var,   lb_var,lb_var,lb_var,   lb_var,lb_var,lb_var,   lb_var,lb_var,lb_var};  // low bound for decision variables
    double ub[] = {up_var,up_var,up_var,   up_var,up_var,up_var,   up_var,up_var,up_var,   up_var,up_var,up_var,   up_var,up_var,up_var};  // up bound for decision variables
    double minf;  // final optimized J result
    double U[15] = {0,0,0,   0,0,0,   0,0,0,   0,0,0,   0,0,0};
    nlopt_opt opt;
    double xref;
    double xdotref;
    double yref;
    double ydotref;
    double thetaref;
    double thetadotref = 0;

    // install data saving function
    fid = fopen(path, "w");
    init_save_data();
    // save_data(X[0], X[2], X[3]); // load the states initial values

    // for loop here --- logic reference from the matlab mpc template
    for (int i=0; i<iter; i++)
    {
        // ********************Start measuring time
        struct timeval begin, end;
        gettimeofday(&begin, 0);
        // ****************************************

        // the states update with Xdotref
        get_trajectory(ts*i, &xref, &yref, &xdotref, &ydotref);
        Xref[0] = xref;
        Xref[1] = yref;
        Xref[2] = 0;

        opt = nlopt_create(NLOPT_LN_COBYLA, n);                                              // generate the optimization problem using nlopt api
        nlopt_set_lower_bounds(opt, lb);                                                     // set the low bound to optimizer
        nlopt_set_upper_bounds(opt, ub);                                                     // set the up bound to optimizer
        nlopt_set_min_objective(opt, mycost, NULL);                                          // set the cost function
        nlopt_set_xtol_rel(opt, 1e-4);                                                       // set the optimizer valve to converge
        if (nlopt_optimize(opt, U, &minf) < 0) 
        {
            // printf("nlopt failed!\n");
        }
        else 
        {
            // printf("we have the optimized min(J) is %f\n", minf);
        //     printf("in this case we have corresponding decision variables are:\n");
        //     printf("*****************\n");
        //     printf("%f %f %f\n%f %f %f\n%f %f %f\n%f %f %f\n%f %f %f\n", U[0],U[1],U[2],U[3],U[4],U[5],U[6],U[7],U[8],U[9],U[10],U[11],U[12],U[13],U[14]);
        //     printf("*****************\n");
        }
        B[0] = cos(X[2]);  // state update
        B[1] = -sin(X[2]);
        B[2] = 0;
        B[3] = sin(X[2]);
        B[4] = cos(X[2]);
        B[5] = 0;
        B[6] = 0;
        B[7] = 0;
        B[8] = 1;
        // the states update without Xdotref
        Xdot[0] = xdotref + B[0]*U[0]+B[1]*U[1]+B[2]*U[2];
        Xdot[1] = ydotref + B[3]*U[0]+B[4]*U[1]+B[5]*U[2];
        // Xdot[2] = 0;
        Xdot[2] = thetadotref + B[6]*U[0]+B[7]*U[1]+B[8]*U[2];
        // Xdot[0] = B[0]*U[0]+B[1]*U[1]+B[2]*U[2]; // without the linear term
        // Xdot[1] = B[3]*U[0]+B[4]*U[1]+B[5]*U[2];
        // Xdot[2] = B[6]*U[0]+B[7]*U[1]+B[8]*U[2];
        X[0] = X[0] + ts*Xdot[0];
        X[1] = X[1] + ts*Xdot[1];
        X[2] = X[2] + ts*Xdot[2];
        // printf("states: %f %f %f\n", X[0], X[1], X[2]);

        // warm start U already being update along with the for loop, already have the warm start

        // save_data(X[0], X[1], X[2]);


        // ********************Stop measuring time and calculate the elapsed time
        gettimeofday(&end, 0);
        long seconds = end.tv_sec - begin.tv_sec;
        long microseconds = end.tv_usec - begin.tv_usec;
        double elapsed = seconds + microseconds*1e-6;
        printf("Time measured: %.3f milli-seconds.\n", elapsed*1e3);
        // ********************Stop measuring time and calculate the elapsed time

        save_data(X[0], X[1], X[2], elapsed*1e3, xref, yref); // save data after one iteration

    }
    nlopt_destroy(opt);
    fclose(fid);
}