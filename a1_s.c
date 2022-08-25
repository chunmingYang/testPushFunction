/*
Generated 23-Mar-2022 17:15:19 by SD/FAST, Kane's formulation
(sdfast B.2.8 #30123) on machine ID unknown
Copyright (c) 1990-1997 Symbolic Dynamics, Inc.
Copyright (c) 1990-1997 Parametric Technology Corp.
RESTRICTED RIGHTS LEGEND: Use, duplication, or disclosure by the U.S.
Government is subject to restrictions as set forth in subparagraph
(c)(1)(ii) of the Rights in Technical Data and Computer Software
clause at DFARS 52.227-7013 and similar clauses in the FAR and NASA
FAR Supplement.  Symbolic Dynamics, Inc., Mountain View, CA 94041
*/
#include <math.h>

/* These routines are passed to a1root. */

void a1posfunc(double vars[2],
    double param[1],
    double resid[1])
{
    int i;
    double pos[2],vel[2];

    for (i = 0; i < 2; i++) {
        vel[i] = 0.;
    }
    a1ang2st(vars,pos);
    a1state(param[0],pos,vel);
    a1perr(resid);
}

void a1velfunc(double vars[2],
    double param[3],
    double resid[1])
{

    a1state(param[2],param,vars);
    a1verr(resid);
}

void a1statfunc(double vars[2],
    double param[3],
    double resid[2])
{
    double pos[2],qdotdum[2];

    a1ang2st(vars,pos);
    a1state(param[2],pos,param);
    a1uforce(param[2],pos,param);
    a1perr(resid);
    a1deriv(qdotdum,&resid[0]);
}

void a1stdyfunc(double vars[4],
    double param[1],
    double resid[2])
{
    double pos[2],qdotdum[2];

    a1ang2st(vars,pos);
    a1state(param[0],pos,&vars[2]);
    a1uforce(param[0],pos,&vars[2]);
    a1perr(resid);
    a1verr(&resid[0]);
    a1deriv(qdotdum,&resid[0]);
}

/* This routine is passed to the integrator. */

void a1motfunc(double time,
    double state[4],
    double dstate[4],
    double param[1],
    int *status)
{

    a1state(time,state,&state[2]);
    a1uforce(time,state,&state[2]);
    a1deriv(dstate,&dstate[2]);
    *status = 0;
}

/* This routine performs assembly analysis. */

void a1assemble(double time,
    double state[4],
    int lock[2],
    double tol,
    int maxevals,
    int *fcnt,
    int *err)
{
    double perrs[1],param[1];
    int i;

    a1gentime(&i);
    if (i != 171519) {
        a1seterr(50,42);
    }
    param[0] = time;
    *err = 0;
    *fcnt = 0;
    a1posfunc(state,param,perrs);
    *fcnt = *fcnt+1;
}

/* This routine performs initial velocity analysis. */

void a1initvel(double time,
    double state[4],
    int lock[2],
    double tol,
    int maxevals,
    int *fcnt,
    int *err)
{
    double verrs[1],param[3];
    int i;

    a1gentime(&i);
    if (i != 171519) {
        a1seterr(51,42);
    }
    for (i = 0; i < 2; i++) {
        param[i] = state[i];
    }
    param[2] = time;
    *err = 0;
    *fcnt = 0;
    a1velfunc(&state[2],param,verrs);
    *fcnt = *fcnt+1;
}

/* This routine performs static analysis. */

void a1static(double time,
    double state[4],
    int lock[2],
    double ctol,
    double tol,
    int maxevals,
    int *fcnt,
    int *err)
{
    double resid[2],param[3],jw[4],dw[32],rw[32];
    int iw[16],rooterr,i;

    a1gentime(&i);
    if (i != 171519) {
        a1seterr(52,42);
    }
    for (i = 0; i < 2; i++) {
        param[i] = state[2+i];
    }
    param[2] = time;
    a1root(a1statfunc,state,param,2,2,2,lock,
      ctol,tol,maxevals,jw,dw,rw,iw,resid,fcnt,&rooterr);
    a1statfunc(state,param,resid);
    *fcnt = *fcnt+1;
    if (rooterr == 0) {
        *err = 0;
    } else {
        if (*fcnt >= maxevals) {
            *err = 2;
        } else {
            *err = 1;
        }
    }
}

/* This routine performs steady motion analysis. */

void a1steady(double time,
    double state[4],
    int lock[4],
    double ctol,
    double tol,
    int maxevals,
    int *fcnt,
    int *err)
{
    double resid[2],param[1];
    double jw[8],dw[72],rw[50];
    int iw[24],rooterr,i;

    a1gentime(&i);
    if (i != 171519) {
        a1seterr(53,42);
    }
    param[0] = time;
    a1root(a1stdyfunc,state,param,2,4,2,lock,
      ctol,tol,maxevals,jw,dw,rw,iw,resid,fcnt,&rooterr);
    a1stdyfunc(state,param,resid);
    *fcnt = *fcnt+1;
    if (rooterr == 0) {
        *err = 0;
    } else {
        if (*fcnt >= maxevals) {
            *err = 2;
        } else {
            *err = 1;
        }
    }
}

/* This routine performs state integration. */

void a1motion(double *time,
    double state[4],
    double dstate[4],
    double dt,
    double ctol,
    double tol,
    int *flag,
    int *err)
{
    static double step;
    double work[24],ttime,param[1];
    int vintgerr,which,ferr,i;

    a1gentime(&i);
    if (i != 171519) {
        a1seterr(54,42);
    }
    param[0] = ctol;
    ttime = *time;
    if (*flag != 0) {
        a1motfunc(ttime,state,dstate,param,&ferr);
        step = dt;
        *flag = 0;
    }
    if (step <= 0.) {
        step = dt;
    }
    a1vinteg(a1motfunc,&ttime,state,dstate,param,dt,&step,4,tol,work,&vintgerr,&
      which);
    *time = ttime;
    *err = vintgerr;
}

/* This routine performs state integration with a fixed-step integrator. */

void a1fmotion(double *time,
    double state[4],
    double dstate[4],
    double dt,
    double ctol,
    int *flag,
    double *errest,
    int *err)
{
    double work[16],ttime,param[1];
    int ferr,i;

    a1gentime(&i);
    if (i != 171519) {
        a1seterr(55,42);
    }
    param[0] = ctol;
    *err = 0;
    ttime = *time;
    if (*flag != 0) {
        a1motfunc(ttime,state,dstate,param,&ferr);
        *flag = 0;
    }
    a1finteg(a1motfunc,&ttime,state,dstate,param,dt,4,work,errest,&ferr);
    if (ferr != 0) {
        *err = 1;
    }
    *time = ttime;
}
