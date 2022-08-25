#include <stdio.h>
#include <math.h>
#define GROUND	(-1)            // information from a1_i
#define BODY_THIGH	0           // information from a1_i
#define BODY_CALF	1           // information from a1_i
#define JOINT_GND_THIGH 0       // information from a1_i
#define JOINT_THIGH_CALF 1      // information from a1_i
#define NQ 2                    // information from a1_i
#define NU 2                    // information from a1_i
#define NSTATE (NQ+NU)
#define CTOL		0.0	        /* constraint tolerance */
#define INT_TOL		1e-10	    /* integration tolerance */
#define TIME        10          /* total simulation timr */
#define DT          0.01        /* Step size */
#define NSTEPS		TIME/DT     /* Number of time steps */

// data collection setting
FILE *fid;
char path[]="data_sd.csv";
void init_save_data()
{
	fprintf(fid, "t,");
	fprintf(fid, "pe,ke,te,");
	fprintf(fid, "q1,q2");
	fprintf(fid, "\n");
}
void save_data(double t, double pe, double ke, double te, double q1, double q2)
{
	fprintf(fid, "%f,", t);
	fprintf(fid, "%f,%f,%f,", pe, ke, te);
	fprintf(fid, "%f,%f", q1, q2);
	fprintf(fid, "\n");
}

// main loop
int main()
{
	double t;
	int flag=1, err, i; 
	double q[2], u[2], state[NSTATE], dstate[NSTATE];
	double lm[3], am[3], ke, pe, te, g[3];
	double comass[3]={0,0,0}, mass_thigh, mass_calf, pos_thigh[3], pos_calf[3], vel_thigh[3], vel_calf[3];
	
	// initialization
	a1init();               // model initialization
	state[0] = 90*M_PI/180; // local joint1 angle    // using "pi" as "M_PI" in C
	state[1] = 0;           // local joint2 angle
	state[2] = 0;
	state[3] = 0;
	q[0] = state[0];
	q[1] = state[1];
	u[0] = state[2];
	u[1] = state[3];
	a1state(t, q, u);       // state update
	fid = fopen(path, "w");
	init_save_data();
	a1getmass(BODY_THIGH, &mass_thigh);
	a1getmass(BODY_CALF, &mass_calf);
	a1getgrav(g);
	a1mom(lm, am, &ke);
	a1pos(BODY_THIGH, comass, pos_thigh);
	a1pos(BODY_CALF, comass, pos_calf);
	pe = mass_thigh*-g[1]*pos_thigh[1] + mass_calf*-g[1]*pos_calf[1];
	te = pe + ke;
	save_data(t, pe, ke, te, state[0], state[1]);

	// iteration routine
	for (i = 0; i < (NSTEPS);i++)
	{
		a1motion(&t,state,dstate,DT,CTOL,INT_TOL,&flag,&err);   // format like ode45
			if (err) printf("*** at t=%g got err=%d\n", t, err);

		// state estimation
		a1mom(lm, am, &ke);
		a1pos(BODY_THIGH, comass, pos_thigh);
		a1pos(BODY_CALF, comass, pos_calf);
		pe = mass_thigh*-g[1]*pos_thigh[1] + mass_calf*-g[1]*pos_calf[1];
		te = pe + ke;

		// checking 
		printf("****************\n");
		printf("time: %f\n", t);
		printf("pe, ke, te: %f %f %f\n", pe, ke, te);
		printf("theta1, theta2: %f %f\n", state[0], state[1]);
		printf("****************\n");

		// save data
		save_data(t, pe, ke, te, state[0], state[1]);
	}
	fclose(fid);
}

void a1uforce(double t, double *q, double *u)
{
}
