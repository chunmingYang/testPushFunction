double setpt[nact]={0};
int step_no = 0;
int prev_step = 0;
double theta0 = 0;
double theta_ref0 = 0;
double Kp_ik = 2;
int flag_constant = 1; //1 for constant yaw and 0 for varying yaw
double px = 0.05;
double py = 0;
#define t_curve 60
double tend = t_curve;
double a = 2;
#include <sys/time.h>
#include "utility_matrix.c"
#include "set_model_state.c"
#include "poly_trajectory.c"
#include "get_Jac.c"
#include "get_kinematics2.c"
#include "inverse_kinematics_3D_analytic.c"
#include "control_params.h"
#include "get_trq_stance.c"
#include "get_trq_swing.c"
#include "set_limit.c"
#include "leg_state_machine.c"
#include "get_stance_forces.c"
#include "get_trajectory.c"
#include "nlopt.h"
FILE *fidmat,*fidC; //,*fidC2;
char pathmat[] = "../sample/a1_trot19traj/data.m";
char pathC[] = "../sample/a1_trot19traj/data.txt";
int flag_openmat;
int flag_C;

// ************ Change this as per the problem being solved *********//
#define DATA_PTS 3 //Set this based on columns in data file
float data[5000][DATA_PTS]; //Structure that will store the data.
int STEPS;
int COUNTER=0;
// **************************************************************** //

int fsm[4] = {0};
void init_controller(int motiontime, double quat_act[4], double omega_act[3],
                     double q_act[nact], double u_act[nact])
{
  int i,j;
  sdinit(); sdprinterr(stderr);
  for (j=0;j<4;j++)
    fsm[j] = fsm_init;
  fidmat = fopen(pathmat,"w");
  fprintf(fidmat,"all_data = [ \n");
  flag_openmat = 1;
  fidC = fopen(pathC,"w");
  flag_C = 1;
  double ang1,ang2, theta;
  double dc[3][3];
  sdquat2dc(quat_act[1],quat_act[2],quat_act[3],quat_act[0],dc); sdprinterr(stderr);
  sddc2ang(dc,&ang1,&ang2,&theta); sdprinterr(stderr);
  theta0 = theta;
  theta_ref0 = theta;
}









































double mycost(unsigned n, const double *U, double *null_1, void *null_2)  // here double *null_1 and void *null_2 are for clear the warning in the terminal
{
  double ts = 0.002;  // step time

  // (1) chunming --- care about not central but the other linear point p as the system states
  // double XX[3] = {x_p_global, y_p_global, theta0_global}; // point p states feedbacks from sensors

  // (2) chunming --- care about the central point c as the system states
  double XX[3] = {x_c_global, y_c_global, theta0_global};

  double Xref[3] = {x_ref_global, y_ref_global, theta0_ref_global}; 
  double X_Xref[3];  // X-Xref
  double mul[3];  // transpose(X-Xref)*Q
  double cost;  // cost initialization
  double cost_state;  // transpose(X-Xref)*Q*(X-Xref)

  // chunming --- i didn't optimize the controls here since from my last experiment, considering "controls" here will worse the optimization
  double cost_controls;  // chunming --- added for tracking problem, the controls regulation transpose(U)*R*U

  double A[9];  // this diff. car model from pranav doesn't have A matrix
  double B[9];  // B matrix of the state space model of diff. car from pranav
  double B_U[3];  // B*U
  double Xdot[3];  // state velocity
  double Q[9] = {1,0,0, 0,1,0, 0,0,0};  // quadrutic function parameters can be tuned
  double R[9] = {1,1,1};  // quadratic function parameters can be tuned

  // iteration 01 *******************************************************************************
  X_Xref[0] = XX[0]-Xref[0];
  X_Xref[1] = XX[1]-Xref[1];
  X_Xref[2] = XX[2]-Xref[2];
  mul[0] = X_Xref[0]*Q[0]+X_Xref[1]*Q[3]+X_Xref[2]*Q[6];
  mul[1] = X_Xref[0]*Q[1]+X_Xref[1]*Q[4]+X_Xref[2]*Q[7];
  mul[2] = X_Xref[0]*Q[2]+X_Xref[1]*Q[5]+X_Xref[2]*Q[8];
  cost_state = X_Xref[0]*mul[0]+mul[1]*X_Xref[1]+mul[2]*X_Xref[2];
  cost = cost + cost_state;
    
  // (1) chunming --- "B" matrix without linear term
  B[0] = cos(XX[2]);
  B[1] = -sin(XX[2]);
  B[2] = 0;
  B[3] = sin(XX[2]);
  B[4] = cos(XX[2]);
  B[5] = 0;
  B[6] = 0;
  B[7] = 0;
  B[8] = 1;

  // (2) chunming --- "B" matrix with linear term equation from chunming
  // double lamdaaaaa = 0.7854; // lamdaaaaa = actan(y_q/x_q) here x_q=y_q=0.1
  // double radius = 0.1414; // radius = sqrt(x_q^2 + y_q^2)
  // B[0] = cos(XX[2]);
  // B[1] = -sin(XX[2]);
  // B[2] = -radius*sin(lamdaaaaa);
  // B[3] = sin(XX[2]);
  // B[4] = cos(XX[2]);
  // B[5] = radius*cos(lamdaaaaa);
  // B[6] = 0;
  // B[7] = 0;
  // B[8] = 1;

  // (3) chunming --- "B" matrix with linear term equation from pranav
  // double x_p_in_cost = XX[0];
  // double y_p_in_cost = XX[1];
  // B[0] = cos(XX[2]);
  // B[1] = -sin(XX[2]);
  // B[2] = cos(pi+XX[2])*x_p_in_cost - sin(pi+XX[2])*y_p_in_cost;
  // B[3] = sin(XX[2]);
  // B[4] = cos(XX[2]);
  // B[5] = sin(pi+XX[2])*x_p_in_cost + cos(pi+XX[2])*y_p_in_cost;
  // B[6] = 0;
  // B[7] = 0;
  // B[8] = 1;

  B_U[0] = B[0]*U[0]+B[1]*U[1]+B[2]*U[2];
  B_U[1] = B[3]*U[0]+B[4]*U[1]+B[5]*U[2];
  B_U[2] = B[6]*U[0]+B[7]*U[1]+B[8]*U[2];
  Xdot[0] = B_U[0];
  Xdot[1] = B_U[1];
  Xdot[2] = B_U[2];
  XX[0] = XX[0]+ts*Xdot[0];
  XX[1] = XX[1]+ts*Xdot[1];
  XX[2] = XX[2]+ts*Xdot[2];
  // ********************************************************************************************



  // iteration 02 *******************************************************************************
  X_Xref[0] = XX[0]-Xref[0];
  X_Xref[1] = XX[1]-Xref[1];
  X_Xref[2] = XX[2]-Xref[2];
  mul[0] = X_Xref[0]*Q[0]+X_Xref[1]*Q[3]+X_Xref[2]*Q[6];
  mul[1] = X_Xref[0]*Q[1]+X_Xref[1]*Q[4]+X_Xref[2]*Q[7];
  mul[2] = X_Xref[0]*Q[2]+X_Xref[1]*Q[5]+X_Xref[2]*Q[8];
  cost_state = X_Xref[0]*mul[0]+mul[1]*X_Xref[1]+mul[2]*X_Xref[2];

  // chunming --- i didn't optimize the controls here since from my last experiment, considering "controls" here will worse the optimization
  // cost_controls = U[0]*R[0]*U[0] + U[1]*R[1]*U[1] + U[1]*R[2]*U[1]; // chunming --- added for tracking problem --- did not solve the tracking problem even performance worse

  cost = cost + cost_state + cost_controls;
  
  // (1) chunming --- "B" matrix without linear term
  B[0] = cos(XX[2]);
  B[1] = -sin(XX[2]);
  B[2] = 0;
  B[3] = sin(XX[2]);
  B[4] = cos(XX[2]);
  B[5] = 0;
  B[6] = 0;
  B[7] = 0;
  B[8] = 1;

  // (2) chunming --- "B" matrix with linear term equation from chunming
  // double lamdaaaaa = 0.7854; // lamdaaaaa = actan(y_q/x_q) here x_q=y_q=0.1
  // double radius = 0.1414; // radius = sqrt(x_q^2 + y_q^2)
  // B[0] = cos(XX[2]);
  // B[1] = -sin(XX[2]);
  // B[2] = -radius*sin(lamdaaaaa);
  // B[3] = sin(XX[2]);
  // B[4] = cos(XX[2]);
  // B[5] = radius*cos(lamdaaaaa);
  // B[6] = 0;
  // B[7] = 0;
  // B[8] = 1;

  // (3) chunming --- "B" matrix with linear term equation from pranav
  // double x_p_in_cost = XX[0];
  // double y_p_in_cost = XX[1];
  // B[0] = cos(XX[2]);
  // B[1] = -sin(XX[2]);
  // B[2] = cos(pi+XX[2])*x_p_in_cost - sin(pi+XX[2])*y_p_in_cost;
  // B[3] = sin(XX[2]);
  // B[4] = cos(XX[2]);
  // B[5] = sin(pi+XX[2])*x_p_in_cost + cos(pi+XX[2])*y_p_in_cost;
  // B[6] = 0;
  // B[7] = 0;
  // B[8] = 1;

  B_U[0] = B[0]*U[3]+B[1]*U[4]+B[2]*U[5];
  B_U[1] = B[3]*U[3]+B[4]*U[4]+B[5]*U[5];
  B_U[2] = B[6]*U[3]+B[7]*U[4]+B[8]*U[5];
  Xdot[0] = B_U[0];
  Xdot[1] = B_U[1];
  Xdot[2] = B_U[2];
  XX[0] = XX[0]+ts*Xdot[0];
  XX[1] = XX[1]+ts*Xdot[1];
  XX[2] = XX[2]+ts*Xdot[2];
  // ********************************************************************************************



  // iteration 03 *******************************************************************************
  X_Xref[0] = XX[0]-Xref[0];
  X_Xref[1] = XX[1]-Xref[1];
  X_Xref[2] = XX[2]-Xref[2];
  mul[0] = X_Xref[0]*Q[0]+X_Xref[1]*Q[3]+X_Xref[2]*Q[6];
  mul[1] = X_Xref[0]*Q[1]+X_Xref[1]*Q[4]+X_Xref[2]*Q[7];
  mul[2] = X_Xref[0]*Q[2]+X_Xref[1]*Q[5]+X_Xref[2]*Q[8];
  cost_state = X_Xref[0]*mul[0]+mul[1]*X_Xref[1]+mul[2]*X_Xref[2];

  // chunming --- i didn't optimize the controls here since from my last experiment, considering "controls" here will worse the optimization
  // cost_controls = U[0]*R[0]*U[0] + U[1]*R[1]*U[1] + U[1]*R[2]*U[1]; // chunming --- added for tracking problem --- did not solve the tracking problem even performance worse
  
  cost = cost + cost_state + cost_controls;
  
  // (1) chunming --- "B" matrix without linear term
  B[0] = cos(XX[2]);
  B[1] = -sin(XX[2]);
  B[2] = 0;
  B[3] = sin(XX[2]);
  B[4] = cos(XX[2]);
  B[5] = 0;
  B[6] = 0;
  B[7] = 0;
  B[8] = 1;

  // (2) chunming --- "B" matrix with linear term equation from chunming
  // double lamdaaaaa = 0.7854; // lamdaaaaa = actan(y_q/x_q) here x_q=y_q=0.1
  // double radius = 0.1414; // radius = sqrt(x_q^2 + y_q^2)
  // B[0] = cos(XX[2]);
  // B[1] = -sin(XX[2]);
  // B[2] = -radius*sin(lamdaaaaa);
  // B[3] = sin(XX[2]);
  // B[4] = cos(XX[2]);
  // B[5] = radius*cos(lamdaaaaa);
  // B[6] = 0;
  // B[7] = 0;
  // B[8] = 1;

  // (3) chunming --- "B" matrix with linear term equation from pranav
  // double x_p_in_cost = XX[0];
  // double y_p_in_cost = XX[1];
  // B[0] = cos(XX[2]);
  // B[1] = -sin(XX[2]);
  // B[2] = cos(pi+XX[2])*x_p_in_cost - sin(pi+XX[2])*y_p_in_cost;
  // B[3] = sin(XX[2]);
  // B[4] = cos(XX[2]);
  // B[5] = sin(pi+XX[2])*x_p_in_cost + cos(pi+XX[2])*y_p_in_cost;
  // B[6] = 0;
  // B[7] = 0;
  // B[8] = 1;

  B_U[0] = B[0]*U[6]+B[1]*U[7]+B[2]*U[8];
  B_U[1] = B[3]*U[6]+B[4]*U[7]+B[5]*U[8];
  B_U[2] = B[6]*U[6]+B[7]*U[7]+B[8]*U[8];
  Xdot[0] = B_U[0];
  Xdot[1] = B_U[1];
  Xdot[2] = B_U[2];
  XX[0] = XX[0]+ts*Xdot[0];
  XX[1] = XX[1]+ts*Xdot[1];
  XX[2] = XX[2]+ts*Xdot[2];
  // ********************************************************************************************



  // iteration 04 *******************************************************************************
  X_Xref[0] = XX[0]-Xref[0];
  X_Xref[1] = XX[1]-Xref[1];
  X_Xref[2] = XX[2]-Xref[2];
  mul[0] = X_Xref[0]*Q[0]+X_Xref[1]*Q[3]+X_Xref[2]*Q[6];
  mul[1] = X_Xref[0]*Q[1]+X_Xref[1]*Q[4]+X_Xref[2]*Q[7];
  mul[2] = X_Xref[0]*Q[2]+X_Xref[1]*Q[5]+X_Xref[2]*Q[8];
  cost_state = X_Xref[0]*mul[0]+mul[1]*X_Xref[1]+mul[2]*X_Xref[2];
  
  // chunming --- i didn't optimize the controls here since from my last experiment, considering "controls" here will worse the optimization
  // cost_controls = U[0]*R[0]*U[0] + U[1]*R[1]*U[1] + U[1]*R[2]*U[1]; // chunming --- added for tracking problem --- did not solve the tracking problem even performance worse
  
  cost = cost + cost_state + cost_controls;
  
  // (1) chunming --- "B" matrix without linear term
  B[0] = cos(XX[2]);
  B[1] = -sin(XX[2]);
  B[2] = 0;
  B[3] = sin(XX[2]);
  B[4] = cos(XX[2]);
  B[5] = 0;
  B[6] = 0;
  B[7] = 0;
  B[8] = 1;

  // (2) chunming --- "B" matrix with linear term equation from chunming
  // double lamdaaaaa = 0.7854; // lamdaaaaa = actan(y_q/x_q) here x_q=y_q=0.1
  // double radius = 0.1414; // radius = sqrt(x_q^2 + y_q^2)
  // B[0] = cos(XX[2]);
  // B[1] = -sin(XX[2]);
  // B[2] = -radius*sin(lamdaaaaa);
  // B[3] = sin(XX[2]);
  // B[4] = cos(XX[2]);
  // B[5] = radius*cos(lamdaaaaa);
  // B[6] = 0;
  // B[7] = 0;
  // B[8] = 1;

  // (3) chunming --- "B" matrix with linear term equation from pranav
  // double x_p_in_cost = XX[0];
  // double y_p_in_cost = XX[1];
  // B[0] = cos(XX[2]);
  // B[1] = -sin(XX[2]);
  // B[2] = cos(pi+XX[2])*x_p_in_cost - sin(pi+XX[2])*y_p_in_cost;
  // B[3] = sin(XX[2]);
  // B[4] = cos(XX[2]);
  // B[5] = sin(pi+XX[2])*x_p_in_cost + cos(pi+XX[2])*y_p_in_cost;
  // B[6] = 0;
  // B[7] = 0;
  // B[8] = 1;

  B_U[0] = B[0]*U[9]+B[1]*U[10]+B[2]*U[11];
  B_U[1] = B[3]*U[9]+B[4]*U[10]+B[5]*U[11];
  B_U[2] = B[6]*U[9]+B[7]*U[10]+B[8]*U[11];
  Xdot[0] = B_U[0];
  Xdot[1] = B_U[1];
  Xdot[2] = B_U[2];
  XX[0] = XX[0]+ts*Xdot[0];
  XX[1] = XX[1]+ts*Xdot[1];
  XX[2] = XX[2]+ts*Xdot[2];
  // ********************************************************************************************



  // iteration 05 *******************************************************************************
  X_Xref[0] = XX[0]-Xref[0];
  X_Xref[1] = XX[1]-Xref[1];
  X_Xref[2] = XX[2]-Xref[2];
  mul[0] = X_Xref[0]*Q[0]+X_Xref[1]*Q[3]+X_Xref[2]*Q[6];
  mul[1] = X_Xref[0]*Q[1]+X_Xref[1]*Q[4]+X_Xref[2]*Q[7];
  mul[2] = X_Xref[0]*Q[2]+X_Xref[1]*Q[5]+X_Xref[2]*Q[8];
  cost_state = X_Xref[0]*mul[0]+mul[1]*X_Xref[1]+mul[2]*X_Xref[2];
  
  // chunming --- i didn't optimize the controls here since from my last experiment, considering "controls" here will worse the optimization
  // cost_controls = U[0]*R[0]*U[0] + U[1]*R[1]*U[1] + U[1]*R[2]*U[1]; // chunming --- added for tracking problem --- did not solve the tracking problem even performance worse
  
  cost = cost + cost_state + cost_controls;
  
  // (1) chunming --- "B" matrix without linear term
  // B[0] = cos(XX[2]);
  // B[1] = -sin(XX[2]);
  // B[2] = 0;
  // B[3] = sin(XX[2]);
  // B[4] = cos(XX[2]);
  // B[5] = 0;
  // B[6] = 0;
  // B[7] = 0;
  // B[8] = 1;

  // (2) chunming --- "B" matrix with linear term equation from chunming
  // double lamdaaaaa = 0.7854; // lamdaaaaa = actan(y_q/x_q) here x_q=y_q=0.1
  // double radius = 0.1414; // radius = sqrt(x_q^2 + y_q^2)
  // B[0] = cos(XX[2]);
  // B[1] = -sin(XX[2]);
  // B[2] = -radius*sin(lamdaaaaa);
  // B[3] = sin(XX[2]);
  // B[4] = cos(XX[2]);
  // B[5] = radius*cos(lamdaaaaa);
  // B[6] = 0;
  // B[7] = 0;
  // B[8] = 1;

  // (3) chunming --- "B" matrix with linear term equation from pranav
  // double x_p_in_cost = XX[0];
  // double y_p_in_cost = XX[1];
  // B[0] = cos(XX[2]);
  // B[1] = -sin(XX[2]);
  // B[2] = cos(pi+XX[2])*x_p_in_cost - sin(pi+XX[2])*y_p_in_cost;
  // B[3] = sin(XX[2]);
  // B[4] = cos(XX[2]);
  // B[5] = sin(pi+XX[2])*x_p_in_cost + cos(pi+XX[2])*y_p_in_cost;
  // B[6] = 0;
  // B[7] = 0;
  // B[8] = 1;

  B_U[0] = B[0]*U[12]+B[1]*U[13]+B[2]*U[14];
  B_U[1] = B[3]*U[12]+B[4]*U[13]+B[5]*U[14];
  B_U[2] = B[6]*U[12]+B[7]*U[13]+B[8]*U[14];
  Xdot[0] = B_U[0];
  Xdot[1] = B_U[1];
  Xdot[2] = B_U[2];
  XX[0] = XX[0]+ts*Xdot[0];
  XX[1] = XX[1]+ts*Xdot[1];
  XX[2] = XX[2]+ts*Xdot[2];
  // ********************************************************************************************



  // *********************** final cost sum for iteration 5 *****************************************************
  X_Xref[0] = XX[0]-Xref[0];
  X_Xref[1] = XX[1]-Xref[1];
  X_Xref[2] = XX[2]-Xref[2];
  mul[0] = X_Xref[0]*Q[0]+X_Xref[1]*Q[3]+X_Xref[2]*Q[6];
  mul[1] = X_Xref[0]*Q[1]+X_Xref[1]*Q[4]+X_Xref[2]*Q[7];
  mul[2] = X_Xref[0]*Q[2]+X_Xref[1]*Q[5]+X_Xref[2]*Q[8];
  cost_state = X_Xref[0]*mul[0]+mul[1]*X_Xref[1]+mul[2]*X_Xref[2];
  
  // chunming --- i didn't optimize the controls here since from my last experiment, considering "controls" here will worse the optimization
  // cost_controls = U[0]*R[0]*U[0] + U[1]*R[1]*U[1] + U[1]*R[2]*U[1]; // chunming --- added for tracking problem --- did not solve the tracking problem even performance worse
  
  cost = cost + cost_state + cost_controls;
  // **********************************************************************************************

  return cost;
}





































void mpc_controller(double *inputs_vx, double *inputs_vy, double *inputs_omega)
{
  // (1) chunming --- decision variables determined by 5 MPC receding horizon 3*5=15
  unsigned n = 15;
  // (2) chunming --- decision variables determined by 2 MPC receding horizon 3*2=6
  // unsigned n = 6;
  
  // (1) chunming --- boundary of decision variable
  // double lb_var = -0.15;
  // double up_var = 0.15;
  // (2) chunming --- boundary of decision variable
  double lb_var = -50;
  double up_var = 50;
  
  // (1) chunming --- control the time horizon here to control the numbers of decision variable 6
  // double lb[] = {lb_var,lb_var,lb_var,   lb_var,lb_var,lb_var};
  // double ub[] = {up_var,up_var,up_var,   up_var,up_var,up_var};
  // (2) chunming --- control the time horizon here to control the numbers of decision variable 15
  double lb[] = {lb_var,lb_var,lb_var,   lb_var,lb_var,lb_var,   lb_var,lb_var,lb_var,   lb_var,lb_var,lb_var,   lb_var,lb_var,lb_var};  // low bound for decision variables
  double ub[] = {up_var,up_var,up_var,   up_var,up_var,up_var,   up_var,up_var,up_var,   up_var,up_var,up_var,   up_var,up_var,up_var};  // up bound for decision variables
  
  double minf;
  
  double U[15] = {};  // decision variables

  // chunming --- warm start format, "u_warmstart(i)" is the gloabal variable stated in the front "a1_main.c"
  // double U[6] = {u_warmstart1, u_warmstart2, u_warmstart3, u_warmstart4, u_warmstart5, u_warmstart6}; // final optimized J result

  nlopt_opt opt;  // nlopt callout install

  // main nlopt loop
  opt = nlopt_create(NLOPT_LN_COBYLA, n);  // generate the optimization problem using nlopt api
  nlopt_set_lower_bounds(opt, lb);  // set the low bound to optimizer
  nlopt_set_upper_bounds(opt, ub);  // set the up bound to optimizer
  nlopt_set_min_objective(opt, mycost, NULL);  // set the cost function
  nlopt_set_xtol_rel(opt, 1e-4);  // set the optimizer valve to converge
  if (nlopt_optimize(opt, U, &minf) < 0) 
  {
    // printf("nlopt failed!\n");
  }
  else 
  {
    printf("nlopt success!\n");
  }
  nlopt_destroy(opt);

  // final output
  *inputs_vx = U[0];
  *inputs_vy = U[1];
  *inputs_omega = U[2];

  // chumming --- warm start format
  // u_warmstart1 = U[0];
  // u_warmstart2 = U[1];
  // u_warmstart3 = U[2];
  // u_warmstart4 = U[3];
  // u_warmstart5 = U[4];
  // u_warmstart6 = U[5];
}


























void my_controller(int motiontime,double quat_act[4], double omega_act[3],
                   double q_act[nact], double u_act[nact], double tau[nact],
                   double Kp[nact],double Kd[nact],
                   double q_ref[nact], double u_ref[nact],double tauEst[nact],
                   uint16_t footForce[4], double bodyWorldPos[3])
{
  int i,j;
  double tau_r[3]={0}, kp_r[3]={0}, kd_r[3] = {0};
  double F[4][3]={0};
  double ttime = motiontime*dtime;
  get_stance_forces(quat_act,omega_act,q_act,u_act,F);
  for (j=0;j<4;j++)
  {
    double qr_ref[3]={0}, ur_ref[3]={0};
    double qr_act[3]={0}, ur_act[3]={0};
    for (i=0;i<3;i++)
    {
      qr_act[i]=q_act[index_leg_act[j][i]];
      ur_act[i]=u_act[index_leg_act[j][i]];
    }
    double force[3] = {0};
    for (i=0;i<3;i++)
      force[i] = F[j][i];
    fsm[j] = leg_state_machine(j,fsm[j],motiontime,quat_act,omega_act,qr_act,ur_act,qr_ref,ur_ref,tau_r,kp_r,kd_r,footForce[j],force);
    double ts = 4; // > t_free + t_trans + t_stance
    if (ttime >= ts && ttime <= ts+tend) //ensures trajectory is provided after ts seconds
    {
      if (prev_step < step_no) //ensures values are set once per step
      {
        prev_step = step_no;
        double x_ref, y_ref;
        double xdot_ref, ydot_ref;
        double theta_ref, thetadot_ref;
        double x_c, y_c,theta;
        double ang1,ang2;
        double dc[3][3];

        // *************** install reference trajectory ***************************
        // (1) pranav --- "8" curve
        double x_center = -a;
        double y_center = 0;
        get_trajectory(ttime,ts,tend,&x_ref,&y_ref,
                        &xdot_ref,&ydot_ref,
                        &theta_ref,&thetadot_ref,
                        &theta_ref0,a,x_center,y_center);

        // (2) chunming --- simple poly characteristic straight line
        // get_trajectory(ttime,ts,tend,&x_ref,&y_ref,&theta_ref,&xdot_ref, &ydot_ref, &thetadot_ref);

        // (3) chunming --- half circle trajectory
        // get_trajectory(ttime, ts, tend,
        //                &x_ref, &y_ref,
        //                &xdot_ref, &ydot_ref);
        // *************************************************************************

        // chunming --- install the reference trajectory information to the global to feed outside functions
        x_ref_global = x_ref;
        xdot_ref_global = xdot_ref;
        y_ref_global = y_ref;
        ydot_ref_global = ydot_ref;
        theta0_ref_global = theta_ref;
        theta0dot_ref_global = thetadot_ref;
        
        sdquat2dc(quat_act[1],quat_act[2],quat_act[3],quat_act[0],dc); sdprinterr(stderr);
        sddc2ang(dc,&ang1,&ang2,&theta); sdprinterr(stderr);
        if (flag_constant)
          theta0 = theta;
        else
          theta0 += t_stance*omega_act[2];
        double c = cos(theta);
        double s = sin(theta);

        //get world coordinates of robot's center of mass
        x_c = bodyWorldPos[0];
        y_c = bodyWorldPos[1];
        //get position of the linear point P
        double x_p = x_c + c*px - s*py;
        double y_p = y_c + s*px + c*py;

        // (1) chunming --- install the point p information to the global in order to feed the "mycost"
        x_p_global = x_p;
        y_p_global = y_p;
        theta0_global = theta0;

        // (2) chunming --- install the point c information to the global in order to feed the "mycost"
        x_c_global = x_c;
        y_c_global = y_c;

        


























        // pranav --- flag_mpc "1" using pranav's controller, "2" using chunming's controller
        int flag_mpc = 0;
        if (flag_mpc!=0)
        {
          double xdot_p = xdot_ref + Kp_ik*(x_ref - x_p);  // Xdot = Xdot_ref + Kp * (X- X_ref) //feedback linearization
          double ydot_p = ydot_ref + Kp_ik*(y_ref - y_p);
          double thetadot_p = thetadot_ref + Kp_ik*(theta_ref - theta0);
          double a11 = c; double a12 = s; double a13 = py;  // Xdot = B*U where U = [vx; vy; omega]
          double a21 = -s; double a22 =  c; double a23 = -px;
          double a31 = 0;  double a32 = 0; double a33 = 1;
          vx =    a11*xdot_p + a12*ydot_p + a13*thetadot_p;
          vy =    a21*xdot_p + a22*ydot_p + a23*thetadot_p;
          omega = a31*xdot_p + a32*ydot_p + a33*thetadot_p;
        }
        else
        {
          double inputs_vx;  // output from mpc
          double inputs_vy;  // output from mpc
          double inputs_omega;  // output from mpc

          // ***********Start measuring time *****************
          struct timeval begin, end;
          gettimeofday(&begin, 0);
          // *************************************************

          mpc_controller(&inputs_vx, &inputs_vy, &inputs_omega);
          
          // ********************Stop measuring time and calculate the elapsed time **************************
          gettimeofday(&end, 0);
          long seconds = end.tv_sec - begin.tv_sec;
          long microseconds = end.tv_usec - begin.tv_usec;
          double elapsed = seconds + microseconds*1e-6;
          // printf("Time measured: %.3f milli-seconds.\n", elapsed*1e3);
          nloptSpeed = elapsed*1e3;
          // **************************************************************************************************



          // ************* optimization result from mpc **************************
          // (1) chunming --- directly use the mpc output
          // vx = inputs_vx;
          // vy = inputs_vy;
          // omega = inputs_omega;
        
          // (2) chunming --- considering the feedforward term since "inputs_vx/vy/omega" can be too small to use
          // vx = xdot_ref_global + inputs_vx;
          // vy = ydot_ref_global + inputs_vy;
          // // // omega = theta0dot_ref_global + inputs_omega;
          // omega = 0;

          // (3) chunming --- increase the "controls" boundary range then add a gain to regulate
          double kp_mpc = 0.006;
          vx = xdot_ref_global + kp_mpc*inputs_vx;
          vy = ydot_ref_global + kp_mpc*inputs_vy;
          // *********************************************************************
        }
      }
    }
    
    
    
    
    












































    
    
    else if (ttime > ts+tend)
    {
      if (flag_openmat==1)
      {
        fprintf(fidmat,"];");
        fclose(fidmat);
        flag_openmat = 0;
      }

      if (flag_C==1)
      {
        //fprintf(fidmat,"];");
        fclose(fidC);
        flag_C = 0;
      }

      vx = 0;
      omega = 0;
      vy = 0;
    }
    for (i=0;i<3;i++)
    {
      q_ref[index_leg_act[j][i]] = qr_ref[i];
      u_ref[index_leg_act[j][i]] = ur_ref[i];
      tau[index_leg_act[j][i]] = tau_r[i];
      Kp[index_leg_act[j][i]] = kp_r[i];
      Kd[index_leg_act[j][i]] = kd_r[i];
    }
  }
}
