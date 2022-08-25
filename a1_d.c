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


ROADMAP (a1.sd)

Bodies        Inb
No  Name      body Joint type  Coords q
--- --------- ---- ----------- ----------------
 -1 $ground                                    
  0 body1      -1  Pin           0             
  1 body2       0  Pin           1             

*/
#include <math.h>
#include <stdio.h>

typedef struct {
    int ground_,nbod_,ndof_,ncons_,nloop_,nldof_,nloopc_,nball_,nlball_,npres_,
      nuser_;
    int jtype_[2],inb_[2],outb_[2],njntdof_[2],njntc_[2],njntp_[2],firstq_[2],
      ballq_[2],firstm_[2],firstp_[2];
    int trans_[2];
} a1gtopo_t;
#define ground (a1gtopo.ground_)
#define nbod (a1gtopo.nbod_)
#define ndof (a1gtopo.ndof_)
#define ncons (a1gtopo.ncons_)
#define nloop (a1gtopo.nloop_)
#define nldof (a1gtopo.nldof_)
#define nloopc (a1gtopo.nloopc_)
#define nball (a1gtopo.nball_)
#define nlball (a1gtopo.nlball_)
#define npres (a1gtopo.npres_)
#define nuser (a1gtopo.nuser_)
#define jtype (a1gtopo.jtype_)
#define inb (a1gtopo.inb_)
#define outb (a1gtopo.outb_)
#define njntdof (a1gtopo.njntdof_)
#define njntc (a1gtopo.njntc_)
#define njntp (a1gtopo.njntp_)
#define firstq (a1gtopo.firstq_)
#define ballq (a1gtopo.ballq_)
#define firstm (a1gtopo.firstm_)
#define firstp (a1gtopo.firstp_)
#define trans (a1gtopo.trans_)

typedef struct {
    double grav_[3],mk_[2],ik_[2][3][3],pin_[2][3];
    double rk_[2][3],ri_[2][3],pres_[2],stabvel_,stabpos_;
    int mfrcflg_,roustate_,vpkflg_,inerflg_,mmflg_,mmlduflg_,wwflg_,ltauflg_,
      fs0flg_,ii_,mmap_[2];
    int gravq_[3],mkq_[2],ikq_[2][3][3],pinq_[2][3],rkq_[2][3],riq_[2][3],presq_
      [2],stabvelq_,stabposq_;
    double mtot_,psmkg_,rhead_[2][3],rcom_[2][3],mkrcomt_[2][3][3],psikg_[3][3],
      psrcomg_[3],psrkg_[3],psrig_[3],psmk_[2],psik_[2][3][3],psrcom_[2][3],
      psrk_[2][3],psri_[2][3];
} a1ginput_t;
#define grav (a1ginput.grav_)
#define mk (a1ginput.mk_)
#define ik (a1ginput.ik_)
#define pin (a1ginput.pin_)
#define rk (a1ginput.rk_)
#define ri (a1ginput.ri_)
#define pres (a1ginput.pres_)
#define stabvel (a1ginput.stabvel_)
#define stabpos (a1ginput.stabpos_)
#define rhead (a1ginput.rhead_)
#define rcom (a1ginput.rcom_)
#define psrcomg (a1ginput.psrcomg_)
#define psrcom (a1ginput.psrcom_)
#define mkrcomt (a1ginput.mkrcomt_)
#define psmk (a1ginput.psmk_)
#define psik (a1ginput.psik_)
#define psrk (a1ginput.psrk_)
#define psri (a1ginput.psri_)
#define psmkg (a1ginput.psmkg_)
#define psikg (a1ginput.psikg_)
#define psrkg (a1ginput.psrkg_)
#define psrig (a1ginput.psrig_)
#define mtot (a1ginput.mtot_)
#define mfrcflg (a1ginput.mfrcflg_)
#define roustate (a1ginput.roustate_)
#define vpkflg (a1ginput.vpkflg_)
#define inerflg (a1ginput.inerflg_)
#define mmflg (a1ginput.mmflg_)
#define mmlduflg (a1ginput.mmlduflg_)
#define wwflg (a1ginput.wwflg_)
#define ltauflg (a1ginput.ltauflg_)
#define fs0flg (a1ginput.fs0flg_)
#define ii (a1ginput.ii_)
#define mmap (a1ginput.mmap_)
#define gravq (a1ginput.gravq_)
#define mkq (a1ginput.mkq_)
#define ikq (a1ginput.ikq_)
#define pinq (a1ginput.pinq_)
#define rkq (a1ginput.rkq_)
#define riq (a1ginput.riq_)
#define presq (a1ginput.presq_)
#define stabvelq (a1ginput.stabvelq_)
#define stabposq (a1ginput.stabposq_)

typedef struct {
    double curtim_,q_[2],qn_[2],u_[2],cnk_[2][3][3],cnb_[2][3][3];
    double rnk_[2][3],vnk_[2][3],wk_[2][3],rnb_[2][3],vnb_[2][3],wb_[2][3],
      wbrcom_[2][3],com_[3],rnkg_[3];
    double Cik_[2][3][3],rikt_[2][3][3],Iko_[2][3][3],mkrk_[2][3][3],Cib_[2][3][
      3];
    double Wkk_[2][3],Vkk_[2][3],dik_[2][3],rpp_[2][3],rpk_[2][3],rik_[2][3],
      rik2_[2][3];
    double rpri_[2][3],Wik_[2][3],Vik_[2][3],Wirk_[2][3],rkWkk_[2][3],Wkrpk_[2][
      3],VikWkr_[2][3];
    double perr_[1],verr_[1],aerr_[1],mult_[1],ufk_[2][3],utk_[2][3],mfk_[2][3],
      mtk_[2][3];
    double utau_[2],mtau_[2],uacc_[2],uvel_[2],upos_[2];
    double s0_,c0_,s1_,c1_;
} a1gstate_t;
#define curtim (a1gstate.curtim_)
#define q (a1gstate.q_)
#define qn (a1gstate.qn_)
#define u (a1gstate.u_)
#define cnk (a1gstate.cnk_)
#define cnb (a1gstate.cnb_)
#define rnkg (a1gstate.rnkg_)
#define rnk (a1gstate.rnk_)
#define rnb (a1gstate.rnb_)
#define vnk (a1gstate.vnk_)
#define vnb (a1gstate.vnb_)
#define wk (a1gstate.wk_)
#define wb (a1gstate.wb_)
#define com (a1gstate.com_)
#define Cik (a1gstate.Cik_)
#define Cib (a1gstate.Cib_)
#define rikt (a1gstate.rikt_)
#define Iko (a1gstate.Iko_)
#define mkrk (a1gstate.mkrk_)
#define Wkk (a1gstate.Wkk_)
#define Vkk (a1gstate.Vkk_)
#define dik (a1gstate.dik_)
#define rpp (a1gstate.rpp_)
#define rpk (a1gstate.rpk_)
#define rik (a1gstate.rik_)
#define rik2 (a1gstate.rik2_)
#define rpri (a1gstate.rpri_)
#define Wik (a1gstate.Wik_)
#define Vik (a1gstate.Vik_)
#define Wirk (a1gstate.Wirk_)
#define rkWkk (a1gstate.rkWkk_)
#define Wkrpk (a1gstate.Wkrpk_)
#define VikWkr (a1gstate.VikWkr_)
#define wbrcom (a1gstate.wbrcom_)
#define perr (a1gstate.perr_)
#define verr (a1gstate.verr_)
#define aerr (a1gstate.aerr_)
#define mult (a1gstate.mult_)
#define ufk (a1gstate.ufk_)
#define utk (a1gstate.utk_)
#define utau (a1gstate.utau_)
#define mfk (a1gstate.mfk_)
#define mtk (a1gstate.mtk_)
#define mtau (a1gstate.mtau_)
#define uacc (a1gstate.uacc_)
#define uvel (a1gstate.uvel_)
#define upos (a1gstate.upos_)
#define s0 (a1gstate.s0_)
#define c0 (a1gstate.c0_)
#define s1 (a1gstate.s1_)
#define c1 (a1gstate.c1_)

typedef struct {
    double fs0_[2],qdot_[2],Otk_[2][3],Atk_[2][3],AiOiWi_[2][3],Fstar_[2][3];
    double Tstar_[2][3],Fstark_[2][3],Tstark_[2][3],IkWk_[2][3],WkIkWk_[2][3],
      gk_[2][3],IkbWk_[2][3],WkIkbWk_[2][3];
    double w0w0_[2],w1w1_[2],w2w2_[2],w0w1_[2],w0w2_[2],w1w2_[2];
    double w00w11_[2],w00w22_[2],w11w22_[2],ww_[1][1],qraux_[1];
    double mm_[2][2],mlo_[2][2],mdi_[2],IkWpk_[2][2][3],works_[2],workss_[2][2];
    double Wpk_[2][2][3],Vpk_[2][2][3],VWri_[2][2][3];
    int wmap_[1],multmap_[1],jpvt_[1],wsiz_,wrank_;
} a1glhs_t;
#define qdot (a1glhs.qdot_)
#define Otk (a1glhs.Otk_)
#define Atk (a1glhs.Atk_)
#define AiOiWi (a1glhs.AiOiWi_)
#define Fstar (a1glhs.Fstar_)
#define Tstar (a1glhs.Tstar_)
#define fs0 (a1glhs.fs0_)
#define Fstark (a1glhs.Fstark_)
#define Tstark (a1glhs.Tstark_)
#define IkWk (a1glhs.IkWk_)
#define IkbWk (a1glhs.IkbWk_)
#define WkIkWk (a1glhs.WkIkWk_)
#define WkIkbWk (a1glhs.WkIkbWk_)
#define gk (a1glhs.gk_)
#define w0w0 (a1glhs.w0w0_)
#define w1w1 (a1glhs.w1w1_)
#define w2w2 (a1glhs.w2w2_)
#define w0w1 (a1glhs.w0w1_)
#define w0w2 (a1glhs.w0w2_)
#define w1w2 (a1glhs.w1w2_)
#define w00w11 (a1glhs.w00w11_)
#define w00w22 (a1glhs.w00w22_)
#define w11w22 (a1glhs.w11w22_)
#define ww (a1glhs.ww_)
#define qraux (a1glhs.qraux_)
#define mm (a1glhs.mm_)
#define mlo (a1glhs.mlo_)
#define mdi (a1glhs.mdi_)
#define IkWpk (a1glhs.IkWpk_)
#define works (a1glhs.works_)
#define workss (a1glhs.workss_)
#define Wpk (a1glhs.Wpk_)
#define Vpk (a1glhs.Vpk_)
#define VWri (a1glhs.VWri_)
#define wmap (a1glhs.wmap_)
#define multmap (a1glhs.multmap_)
#define jpvt (a1glhs.jpvt_)
#define wsiz (a1glhs.wsiz_)
#define wrank (a1glhs.wrank_)

typedef struct {
    double fs_[2],udot_[2],tauc_[2],dyad_[2][3][3],fc_[2][3],tc_[2][3];
    double ank_[2][3],onk_[2][3],Onkb_[2][3],AOnkri_[2][3],Ankb_[2][3],AnkAtk_[2
      ][3],anb_[2][3],onb_[2][3],dyrcom_[2][3];
    double ffk_[2][3],ttk_[2][3],fccikt_[2][3],ffkb_[2][3],ttkb_[2][3];
} a1grhs_t;
#define fs (a1grhs.fs_)
#define udot (a1grhs.udot_)
#define ank (a1grhs.ank_)
#define anb (a1grhs.anb_)
#define onk (a1grhs.onk_)
#define onb (a1grhs.onb_)
#define Onkb (a1grhs.Onkb_)
#define AOnkri (a1grhs.AOnkri_)
#define Ankb (a1grhs.Ankb_)
#define AnkAtk (a1grhs.AnkAtk_)
#define dyrcom (a1grhs.dyrcom_)
#define ffk (a1grhs.ffk_)
#define ttk (a1grhs.ttk_)
#define fccikt (a1grhs.fccikt_)
#define ffkb (a1grhs.ffkb_)
#define ttkb (a1grhs.ttkb_)
#define dyad (a1grhs.dyad_)
#define fc (a1grhs.fc_)
#define tc (a1grhs.tc_)
#define tauc (a1grhs.tauc_)

typedef struct {
    double temp_[3000],tmat1_[3][3],tmat2_[3][3],tvec1_[3],tvec2_[3],tvec3_[3],
      tvec4_[3],tvec5_[3];
    double tsc1_,tsc2_,tsc3_;
} a1gtemp_t;
#define temp (a1gtemp.temp_)
#define tmat1 (a1gtemp.tmat1_)
#define tmat2 (a1gtemp.tmat2_)
#define tvec1 (a1gtemp.tvec1_)
#define tvec2 (a1gtemp.tvec2_)
#define tvec3 (a1gtemp.tvec3_)
#define tvec4 (a1gtemp.tvec4_)
#define tvec5 (a1gtemp.tvec5_)
#define tsc1 (a1gtemp.tsc1_)
#define tsc2 (a1gtemp.tsc2_)
#define tsc3 (a1gtemp.tsc3_)

a1gtopo_t a1gtopo = {
/*  Topological information
*/
    /* ground */ 1,
    /* nbod */ 2,
    /* ndof */ 2,
    /* ncons */ 0,
    /* nloop */ 0,
    /* nldof */ 0,
    /* nloopc */ 0,
    /* nball */ 0,
    /* nlball */ 0,
    /* npres */ 0,
    /* nuser */ 0,
    /* jtype[0] */ 1,
    /* jtype[1] */ 1,
    /* inb[0] */ -1,
    /* inb[1] */ 0,
    /* outb[0] */ 0,
    /* outb[1] */ 1,
    /* njntdof[0] */ 1,
    /* njntdof[1] */ 1,
    /* njntc[0] */ 0,
    /* njntc[1] */ 0,
    /* njntp[0] */ 0,
    /* njntp[1] */ 0,
    /* firstq[0] */ 0,
    /* firstq[1] */ 1,
    /* ballq[0] */ -104,
    /* ballq[1] */ -104,
    /* firstm[0] */ -1,
    /* firstm[1] */ -1,
    /* firstp[0] */ -1,
    /* firstp[1] */ -1,
    /* trans[0] */ 0,
    /* trans[1] */ 0,
};
a1ginput_t a1ginput = {
/* Model parameters from the input file */

/* gravity */
    /* grav[0] */ 0.,
    /* grav[1] */ -9.8,
    /* grav[2] */ 0.,

/* mass */
    /* mk[0] */ 1.013,
    /* mk[1] */ .226,

/* inertia */
    /* ik[0][0][0] */ .01013,
    /* ik[0][0][1] */ 0.,
    /* ik[0][0][2] */ 0.,
    /* ik[0][1][0] */ 0.,
    /* ik[0][1][1] */ 0.,
    /* ik[0][1][2] */ 0.,
    /* ik[0][2][0] */ 0.,
    /* ik[0][2][1] */ 0.,
    /* ik[0][2][2] */ .01013,
    /* ik[1][0][0] */ .02034,
    /* ik[1][0][1] */ 0.,
    /* ik[1][0][2] */ 0.,
    /* ik[1][1][0] */ 0.,
    /* ik[1][1][1] */ 0.,
    /* ik[1][1][2] */ 0.,
    /* ik[1][2][0] */ 0.,
    /* ik[1][2][1] */ 0.,
    /* ik[1][2][2] */ .02034,

/* tree hinge axis vectors */
    /* pin[0][0] */ 0.,
    /* pin[0][1] */ 0.,
    /* pin[0][2] */ 1.,
    /* pin[1][0] */ 0.,
    /* pin[1][1] */ 0.,
    /* pin[1][2] */ 1.,

/* tree bodytojoint vectors */
    /* rk[0][0] */ 0.,
    /* rk[0][1] */ .1,
    /* rk[0][2] */ 0.,
    /* rk[1][0] */ 0.,
    /* rk[1][1] */ .1,
    /* rk[1][2] */ 0.,

/* tree inbtojoint vectors */
    /* ri[0][0] */ 0.,
    /* ri[0][1] */ 0.,
    /* ri[0][2] */ 0.,
    /* ri[1][0] */ 0.,
    /* ri[1][1] */ -.1,
    /* ri[1][2] */ 0.,

/* tree prescribed motion */
    /* pres[0] */ 0.,
    /* pres[1] */ 0.,

/* stabilization parameters */
    /* stabvel */ 0.,
    /* stabpos */ 0.,

/* miscellaneous */
    /* mfrcflg */ 0,
    /* roustate */ 0,
    /* vpkflg */ 0,
    /* inerflg */ 0,
    /* mmflg */ 0,
    /* mmlduflg */ 0,
    /* wwflg */ 0,
    /* ltauflg */ 0,
    /* fs0flg */ 0,
    /* ii */ 0,
    /* mmap[0] */ 0,
    /* mmap[1] */ 1,

/* Which parameters were "?" (1) or "<nominal>?" (3) */
    /* gravq[0] */ 0,
    /* gravq[1] */ 0,
    /* gravq[2] */ 0,
    /* mkq[0] */ 0,
    /* mkq[1] */ 0,
    /* ikq[0][0][0] */ 0,
    /* ikq[0][0][1] */ 0,
    /* ikq[0][0][2] */ 0,
    /* ikq[0][1][0] */ 0,
    /* ikq[0][1][1] */ 0,
    /* ikq[0][1][2] */ 0,
    /* ikq[0][2][0] */ 0,
    /* ikq[0][2][1] */ 0,
    /* ikq[0][2][2] */ 0,
    /* ikq[1][0][0] */ 0,
    /* ikq[1][0][1] */ 0,
    /* ikq[1][0][2] */ 0,
    /* ikq[1][1][0] */ 0,
    /* ikq[1][1][1] */ 0,
    /* ikq[1][1][2] */ 0,
    /* ikq[1][2][0] */ 0,
    /* ikq[1][2][1] */ 0,
    /* ikq[1][2][2] */ 0,
    /* pinq[0][0] */ 0,
    /* pinq[0][1] */ 0,
    /* pinq[0][2] */ 0,
    /* pinq[1][0] */ 0,
    /* pinq[1][1] */ 0,
    /* pinq[1][2] */ 0,
    /* rkq[0][0] */ 0,
    /* rkq[0][1] */ 0,
    /* rkq[0][2] */ 0,
    /* rkq[1][0] */ 0,
    /* rkq[1][1] */ 0,
    /* rkq[1][2] */ 0,
    /* riq[0][0] */ 0,
    /* riq[0][1] */ 0,
    /* riq[0][2] */ 0,
    /* riq[1][0] */ 0,
    /* riq[1][1] */ 0,
    /* riq[1][2] */ 0,
    /* presq[0] */ 0,
    /* presq[1] */ 0,
    /* stabvelq */ 3,
    /* stabposq */ 3,

/* End of values from input file */

};
a1gstate_t a1gstate;
a1glhs_t a1glhs;
a1grhs_t a1grhs;
a1gtemp_t a1gtemp;


void a1init(void)
{
/*
Initialization routine


 This routine must be called before the first call to sdstate(), after
 supplying values for any `?' parameters in the input.
*/
    double sumsq,norminv;
    int i,j,k;


/* Check that all `?' parameters have been assigned values */

    for (k = 0; k < 3; k++) {
        if (gravq[k] == 1) {
            a1seterr(7,25);
        }
    }
    for (k = 0; k < 2; k++) {
        if (mkq[k] == 1) {
            a1seterr(7,26);
        }
        for (i = 0; i < 3; i++) {
            if (rkq[k][i] == 1) {
                a1seterr(7,29);
            }
            if (riq[k][i] == 1) {
                a1seterr(7,30);
            }
            for (j = 0; j < 3; j++) {
                if (ikq[k][i][j] == 1) {
                    a1seterr(7,27);
                }
            }
        }
    }
    for (k = 0; k < 2; k++) {
        for (i = 0; i < 3; i++) {
            if (pinq[k][i] == 1) {
                a1seterr(7,28);
            }
        }
    }

/* Normalize pin vectors if necessary */


/* Zero out Vpk and Wpk */

    for (i = 0; i < 2; i++) {
        for (j = i; j <= 1; j++) {
            for (k = 0; k < 3; k++) {
                Vpk[i][j][k] = 0.;
                Wpk[i][j][k] = 0.;
            }
        }
    }

/* Compute pseudobody-related constants */

    rcom[0][0] = 0.;
    rcom[0][1] = 0.;
    rcom[0][2] = 0.;
    rcom[1][0] = 0.;
    rcom[1][1] = 0.;
    rcom[1][2] = 0.;

/* Compute mass properties-related constants */

    mtot = 1.239;
    a1serialno(&i);
    if (i != 30123) {
        a1seterr(7,41);
    }
    roustate = 1;
}

/* Convert state to form using 1-2-3 Euler angles for ball joints. */

void a1st2ang(double st[2],
    double stang[2])
{
    int i;

    for (i = 0; i < 2; i++) {
        stang[i] = st[i];
    }
}

/* Convert 1-2-3 form of state back to Euler parameters for ball joints. */

void a1ang2st(double stang[2],
    double st[2])
{
    int i;

    for (i = 0; i < 2; i++) {
        st[i] = stang[i];
    }
}

/* Normalize Euler parameters in state. */

void a1nrmsterr(double st[2],
    double normst[2],
    int routine)
{
    int i;

    for (i = 0; i < 2; i++) {
        normst[i] = st[i];
    }
}

void a1normst(double st[2],
    double normst[2])
{

    a1nrmsterr(st,normst,0);
}

void a1state(double timein,
    double qin[2],
    double uin[2])
{
/*
Compute kinematic information and store it in sdgstate.

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
    int i,j,qchg,uchg;

    if ((roustate != 1) && (roustate != 2) && (roustate != 3)) {
        a1seterr(8,22);
        return;
    }
    if (roustate == 1) {
        for (i = 0; i < 2; i++) {
            if (presq[i] == 1) {
                a1seterr(8,31);
            }
        }
    }
/*
See if time or any qs have changed since last call
*/
    if ((roustate != 1) && (timein == curtim)) {
        qchg = 0;
        for (i = 0; i < 2; i++) {
            if (qin[i] != q[i]) {
                qchg = 1;
                break;
            }
        }
    } else {
        qchg = 1;
    }
/*
If time and qs are unchanged, check us
*/
    if (qchg == 0) {
        uchg = 0;
        for (i = 0; i < 2; i++) {
            if (uin[i] != u[i]) {
                uchg = 1;
                break;
            }
        }
    } else {
        uchg = 1;
    }
    curtim = timein;
    roustate = 2;
    if (qchg == 0) {
        goto skipqs;
    }
/*
Position-related variables need to be computed
*/
    vpkflg = 0;
    mmflg = 0;
    mmlduflg = 0;
    wwflg = 0;
    for (i = 0; i < 2; i++) {
        q[i] = qin[i];
    }
/*
Compute sines and cosines of q
*/
    s0 = sin(q[0]);
    c0 = cos(q[0]);
    s1 = sin(q[1]);
    c1 = cos(q[1]);
/*
Compute across-axis direction cosines Cik
*/
/*
Compute across-joint direction cosines Cib
*/
/*
Compute gravity
*/
    gk[1][0] = -(9.8*((s0*c1)+(s1*c0)));
    gk[1][1] = (9.8*((s0*s1)-(c0*c1)));
/*
Compute cnk & cnb (direction cosines in N)
*/
    cnk[1][0][0] = ((c0*c1)-(s0*s1));
    cnk[1][0][1] = -((s0*c1)+(s1*c0));
    cnk[1][1][0] = ((s0*c1)+(s1*c0));
    cnk[1][1][1] = ((c0*c1)-(s0*s1));
    cnb[0][0][0] = c0;
    cnb[0][0][1] = -s0;
    cnb[0][0][2] = 0.;
    cnb[0][1][0] = s0;
    cnb[0][1][1] = c0;
    cnb[0][1][2] = 0.;
    cnb[0][2][0] = 0.;
    cnb[0][2][1] = 0.;
    cnb[0][2][2] = 1.;
    cnb[1][0][0] = cnk[1][0][0];
    cnb[1][0][1] = cnk[1][0][1];
    cnb[1][0][2] = 0.;
    cnb[1][1][0] = cnk[1][1][0];
    cnb[1][1][1] = cnk[1][1][1];
    cnb[1][1][2] = 0.;
    cnb[1][2][0] = 0.;
    cnb[1][2][1] = 0.;
    cnb[1][2][2] = 1.;
/*
Compute q-related auxiliary variables
*/
    rik[1][1] = -(.1+(.1*c1));
/*
Compute rnk & rnb (mass center locations in N)
*/
    rnk[1][0] = ((.2*s0)-(.1*cnk[1][0][1]));
    rnk[1][1] = -((.1*cnk[1][1][1])+(.2*c0));
    rnb[0][0] = (.1*s0);
    rnb[0][1] = -(.1*c0);
    rnb[0][2] = 0.;
    rnb[1][0] = rnk[1][0];
    rnb[1][1] = rnk[1][1];
    rnb[1][2] = 0.;
/*
Compute com (system mass center location in N)
*/
    com[0] = (.807102502017756*((.1013*s0)+(.226*rnk[1][0])));
    com[1] = (.807102502017756*((.226*rnk[1][1])-(.1013*c0)));
    com[2] = 0.;
    skipqs: ;
    if (uchg == 0) {
        goto skipus;
    }
/*
Velocity-related variables need to be computed
*/
    inerflg = 0;
    for (i = 0; i < 2; i++) {
        u[i] = uin[i];
    }
/*
Compute u-related auxiliary variables
*/
/*
Compute wk & wb (angular velocities)
*/
    wk[1][2] = (u[0]+u[1]);
    wb[0][0] = 0.;
    wb[0][1] = 0.;
    wb[0][2] = u[0];
    wb[1][0] = 0.;
    wb[1][1] = 0.;
    wb[1][2] = wk[1][2];
/*
Compute auxiliary variables involving wk
*/
/*
Compute temporaries for use in SDRHS
*/
    w2w2[0] = (u[0]*u[0]);
    w2w2[1] = (wk[1][2]*wk[1][2]);
/*
Compute vnk & vnb (mass center linear velocities in N)
*/
    vnk[0][0] = (.1*(u[0]*c0));
    vnk[0][1] = (.1*(u[0]*s0));
    vnk[1][0] = ((.1*(cnk[1][0][0]*wk[1][2]))+(vnk[0][0]+(.1*(u[0]*c0))));
    vnk[1][1] = ((.1*(cnk[1][1][0]*wk[1][2]))+(vnk[0][1]+(.1*(u[0]*s0))));
    vnb[0][0] = vnk[0][0];
    vnb[0][1] = vnk[0][1];
    vnb[0][2] = 0.;
    vnb[1][0] = vnk[1][0];
    vnb[1][1] = vnk[1][1];
    vnb[1][2] = 0.;
/*
Compute qdot (kinematical equations)
*/
    qdot[0] = u[0];
    qdot[1] = u[1];
    skipus: ;
/*
Initialize applied forces and torques to zero
*/
    for (i = 0; i < 2; i++) {
        for (j = 0; j < 3; j++) {
            ufk[i][j] = 0.;
            utk[i][j] = 0.;
        }
    }
    for (i = 0; i < 2; i++) {
        utau[i] = 0.;
    }
    ltauflg = 0;
    fs0flg = 0;
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain   22 adds/subtracts/negates
                     41 multiplies
                      0 divides
                     75 assignments
*/
}

void a1qdot(double oqdot[2])
{
/*
Return position coordinate derivatives for tree joints.
*/
    int i;

    if ((roustate != 2) && (roustate != 3)) {
        a1seterr(63,23);
        return;
    }
    for (i = 0; i <= 1; i++) {
        oqdot[i] = qdot[i];
    }
}

void a1u2qdot(double uin[2],
    double oqdot[2])
{
/*
Convert velocities to qdots.
*/
    int i;

    if ((roustate != 2) && (roustate != 3)) {
        a1seterr(64,23);
        return;
    }
    for (i = 0; i <= 1; i++) {
        oqdot[i] = uin[i];
    }
/*
 Used -0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain    0 adds/subtracts/negates
                      0 multiplies
                      0 divides
                      2 assignments
*/
}

void a1psstate(double lqin[1])
{

    if (roustate != 2) {
        a1seterr(9,23);
        return;
    }
}

void a1dovpk(void)
{

    if (vpkflg == 0) {
/*
Compute Wpk (partial angular velocities)
*/
        Wpk[0][0][2] = 1.;
        Wpk[0][1][2] = 1.;
        Wpk[1][1][2] = 1.;
/*
Compute Vpk (partial velocities)
*/
        Vpk[0][0][0] = .1;
        Vpk[0][1][0] = (.1+(.2*c1));
        Vpk[0][1][1] = -(.2*s1);
        Vpk[1][1][0] = .1;
        vpkflg = 1;
    }
/*
 Used -0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain    2 adds/subtracts/negates
                      2 multiplies
                      0 divides
                      7 assignments
*/
}

void a1doltau(void)
{

/*
Compute effect of loop hinge torques
*/
/*
 Used -0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain    0 adds/subtracts/negates
                      0 multiplies
                      0 divides
                      0 assignments
*/
}

void a1doiner(void)
{

/*
Compute inertial accelerations and related temps
*/
    if (inerflg == 0) {
/*
Compute Otk (inertial angular acceleration)
*/
/*
Compute Atk (inertial linear acceleration)
*/
        Atk[0][1] = (.1*(u[0]*u[0]));
        AiOiWi[1][1] = (Atk[0][1]+(.1*(u[0]*u[0])));
        Atk[1][0] = (AiOiWi[1][1]*s1);
        Atk[1][1] = ((.1*(wk[1][2]*wk[1][2]))+(AiOiWi[1][1]*c1));
        inerflg = 1;
    }
/*
 Used -0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain    2 adds/subtracts/negates
                      8 multiplies
                      0 divides
                      4 assignments
*/
}

void a1dofs0(void)
{

/*
Compute effect of all applied loads
*/
    if (fs0flg == 0) {
        a1doltau();
        a1doiner();
/*
Compute Fstar (forces)
*/
        Fstar[0][0] = ((9.9274*s0)-ufk[0][0]);
        Fstar[0][1] = ((1.013*(Atk[0][1]+(9.8*c0)))-ufk[0][1]);
        Fstar[1][0] = ((.226*(Atk[1][0]-gk[1][0]))-ufk[1][0]);
        Fstar[1][1] = ((.226*(Atk[1][1]-gk[1][1]))-ufk[1][1]);
/*
Compute Tstar (torques)
*/
/*
Compute fs0 (RHS ignoring constraints)
*/
        a1dovpk();
        fs0[0] = (utau[0]-(((.1*Fstar[0][0])-utk[0][2])+(((Fstar[1][0]*
          Vpk[0][1][0])-(.2*(Fstar[1][1]*s1)))-utk[1][2])));
        fs0[1] = (utau[1]-((.1*Fstar[1][0])-utk[1][2]));
        fs0flg = 1;
    }
/*
 Used -0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain   14 adds/subtracts/negates
                     10 multiplies
                      0 divides
                      6 assignments
*/
}

void a1domm(int routine)
{
    int dumroutine,errnum;
    int i;

    if (mmflg == 0) {
/*
Compute mass matrix (MM)
*/
        a1dovpk();
        mm[0][0] = (.0406+(.226*((.04*(s1*s1))+(Vpk[0][1][0]*Vpk[0][1][0]))));
        mm[0][1] = (.02034+(.0226*Vpk[0][1][0]));
        mm[1][1] = .0226;
/*
Check for singular mass matrix
*/
        for (i = 0; i < 2; i++) {
            if (mm[i][i] < 1e-13) {
                a1seterr(routine,47);
            }
        }
        a1error(&dumroutine,&errnum);
        if (errnum == 0) {
            mmflg = 1;
        }
    }
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain    3 adds/subtracts/negates
                      5 multiplies
                      0 divides
                      3 assignments
*/
}

void a1dommldu(int routine)
{
    int i;
    int dumroutine,errnum;

    if (mmlduflg == 0) {
        a1domm(routine);
/*
Numerically decompose the mass matrix
*/
        a1ldudcomp(2,2,mmap,1e-13,workss,works,mm,mlo,mdi);
/*
Check for singular mass matrix
*/
        for (i = 0; i < 2; i++) {
            if (mdi[i] <= 1e-13) {
                a1seterr(routine,47);
            }
        }
        a1error(&dumroutine,&errnum);
        if (errnum == 0) {
            mmlduflg = 1;
        }
    }
}

void a1lhs(int routine)
{
/* Compute all remaining state- and force-dependent quantities
*/

    roustate = 2;
    a1dommldu(routine);
    a1dofs0();
}

void a1mfrc(double imult[1])
{

}

void a1equivht(double tau[2])
{
/* Compute tree hinge torques to match effect of applied loads
*/
    double fstareq[2][3],tstareq[2][3];

    if ((roustate != 2) && (roustate != 3)) {
        a1seterr(56,23);
        return;
    }
/*
Compute fstareq (forces)
*/
    fstareq[0][0] = ((9.9274*s0)-ufk[0][0]);
    fstareq[0][1] = ((9.9274*c0)-ufk[0][1]);
    fstareq[1][0] = -(ufk[1][0]+(.226*gk[1][0]));
    fstareq[1][1] = -(ufk[1][1]+(.226*gk[1][1]));
/*
Compute tstareq (torques)
*/
/*
Compute taus (RHS ignoring constraints and inertial forces)
*/
    a1dovpk();
    tau[0] = (utau[0]-(((.1*fstareq[0][0])-utk[0][2])+(((fstareq[1][0]*
      Vpk[0][1][0])-(.2*(fstareq[1][1]*s1)))-utk[1][2])));
    tau[1] = (utau[1]-((.1*fstareq[1][0])-utk[1][2]));
/*
Op counts below do not include called subroutines
*/
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain   13 adds/subtracts/negates
                      9 multiplies
                      0 divides
                      6 assignments
*/
}

void a1fs0(void)
{

/*
Compute Fs (ignoring multiplier forces)
*/
    fs[0] = fs0[0];
    fs[1] = fs0[1];
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain    0 adds/subtracts/negates
                      0 multiplies
                      0 divides
                      2 assignments
*/
}

void a1fsmult(void)
{
    int i;

/*
Compute Fs (multiplier-generated forces only)
*/
    for (i = 0; i < 2; i++) {
        fs[i] = 0.;
    }
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain    0 adds/subtracts/negates
                      0 multiplies
                      0 divides
                      2 assignments
*/
}

void a1fsfull(void)
{

/*
Compute Fs (including all forces)
*/
    a1fsmult();
    fs[0] = (fs[0]+fs0[0]);
    fs[1] = (fs[1]+fs0[1]);
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain    2 adds/subtracts/negates
                      0 multiplies
                      0 divides
                      2 assignments
*/
}

void a1fsgenmult(void)
{
    int i;

/*
Compute Fs (generic multiplier-generated forces)
*/
    for (i = 0; i < 2; i++) {
        fs[i] = 0.;
    }
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain    0 adds/subtracts/negates
                      0 multiplies
                      0 divides
                      2 assignments
*/
}

void a1fsgenfull(void)
{

/*
Compute Fs (incl generic mult & other forces)
*/
    a1fsgenmult();
    fs[0] = (fs[0]+fs0[0]);
    fs[1] = (fs[1]+fs0[1]);
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain    2 adds/subtracts/negates
                      0 multiplies
                      0 divides
                      2 assignments
*/
}

void a1fulltrq(double udotin[2],
    double multin[1],
    double trqout[2])
{
/* Compute hinge torques which would produce indicated udots
*/
    double fstarr[2][3],tstarr[2][3],Otkr[2][3],Atir[2][3],Atkr[2][3];

    if ((roustate != 2) && (roustate != 3)) {
        a1seterr(61,23);
        return;
    }
/*
Account for inertial accelerations and supplied udots
*/
    Otkr[1][2] = (udotin[0]+udotin[1]);
    Atkr[0][1] = (.1*(u[0]*u[0]));
    Atir[1][1] = (Atkr[0][1]+(.1*(u[0]*u[0])));
    Atkr[1][0] = ((.1*Otkr[1][2])+((.2*(udotin[0]*c1))+(Atir[1][1]*s1)));
    Atkr[1][1] = ((.1*(wk[1][2]*wk[1][2]))+((Atir[1][1]*c1)-(.2*(udotin[0]*s1)))
      );
/*
Accumulate all forces and torques
*/
    fstarr[0][0] = (ufk[0][0]-(1.013*((.1*udotin[0])+(9.8*s0))));
    fstarr[0][1] = (ufk[0][1]-(1.013*(Atkr[0][1]+(9.8*c0))));
    fstarr[1][0] = (ufk[1][0]+(.226*(gk[1][0]-Atkr[1][0])));
    fstarr[1][1] = (ufk[1][1]+(.226*(gk[1][1]-Atkr[1][1])));
    tstarr[0][2] = (utk[0][2]-(.01013*udotin[0]));
    tstarr[1][2] = (utk[1][2]-(.02034*Otkr[1][2]));
/*
Now calculate the torques
*/
    a1dovpk();
    trqout[0] = -(utau[0]+((tstarr[0][2]+(.1*fstarr[0][0]))+(tstarr[1][2]+((
      fstarr[1][0]*Vpk[0][1][0])-(.2*(fstarr[1][1]*s1))))));
    trqout[1] = -(utau[1]+(tstarr[1][2]+(.1*fstarr[1][0])));
/*
Op counts below do not include called subroutines
*/
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain   25 adds/subtracts/negates
                     27 multiplies
                      0 divides
                     13 assignments
*/
}

void a1comptrq(double udotin[2],
    double trqout[2])
{
/* Compute hinge torques to produce these udots, ignoring constraints
*/
    double multin[1];

    if ((roustate != 2) && (roustate != 3)) {
        a1seterr(60,23);
        return;
    }
    a1fulltrq(udotin,multin,trqout);
}

void a1multtrq(double multin[1],
    double trqout[2])
{
/* Compute hinge trqs which would be produced by these mults.
*/
    int i;

    if ((roustate != 2) && (roustate != 3)) {
        a1seterr(65,23);
        return;
    }
    for (i = 0; i < 2; i++) {
        trqout[i] = 0.;
    }
}

void a1rhs(void)
{
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

/*
Compute hinge torques for tree hinges
*/
    tauc[0] = utau[0];
    tauc[1] = utau[1];
    a1doiner();
/*
Compute onk & onb (angular accels in N)
*/
    Onkb[1][2] = (udot[0]+udot[1]);
    onb[0][0] = 0.;
    onb[0][1] = 0.;
    onb[0][2] = udot[0];
    onb[1][0] = 0.;
    onb[1][1] = 0.;
    onb[1][2] = Onkb[1][2];
/*
Compute acceleration dyadics
*/
    dyad[0][0][0] = -w2w2[0];
    dyad[0][0][1] = -udot[0];
    dyad[0][0][2] = 0.;
    dyad[0][1][0] = udot[0];
    dyad[0][1][1] = -w2w2[0];
    dyad[0][1][2] = 0.;
    dyad[0][2][0] = 0.;
    dyad[0][2][1] = 0.;
    dyad[0][2][2] = 0.;
    dyad[1][0][0] = -w2w2[1];
    dyad[1][0][1] = -Onkb[1][2];
    dyad[1][0][2] = 0.;
    dyad[1][1][0] = Onkb[1][2];
    dyad[1][1][1] = -w2w2[1];
    dyad[1][1][2] = 0.;
    dyad[1][2][0] = 0.;
    dyad[1][2][1] = 0.;
    dyad[1][2][2] = 0.;
/*
Compute ank & anb (mass center linear accels in N)
*/
    Ankb[1][0] = ((.1*Onkb[1][2])+(.2*(udot[0]*c1)));
    Ankb[1][1] = -(.2*(udot[0]*s1));
    ank[0][0] = ((.1*(udot[0]*c0))-(Atk[0][1]*s0));
    ank[0][1] = ((.1*(udot[0]*s0))+(Atk[0][1]*c0));
    AnkAtk[1][0] = (Ankb[1][0]+Atk[1][0]);
    AnkAtk[1][1] = (Ankb[1][1]+Atk[1][1]);
    ank[1][0] = ((AnkAtk[1][0]*cnk[1][0][0])+(AnkAtk[1][1]*cnk[1][0][1]));
    ank[1][1] = ((AnkAtk[1][0]*cnk[1][1][0])+(AnkAtk[1][1]*cnk[1][1][1]));
    anb[0][0] = ank[0][0];
    anb[0][1] = ank[0][1];
    anb[0][2] = 0.;
    anb[1][0] = ank[1][0];
    anb[1][1] = ank[1][1];
    anb[1][2] = 0.;
    roustate = 3;
/*
 Used -0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain   15 adds/subtracts/negates
                     15 multiplies
                      0 divides
                     41 assignments
*/
}

void a1massmat(double mmat[2][2])
{
/* Return the system mass matrix (LHS)
*/
    int i,j;

    if ((roustate != 2) && (roustate != 3)) {
        a1seterr(57,23);
        return;
    }
    a1domm(57);
    for (i = 0; i < 2; i++) {
        for (j = i; j <= 1; j++) {
            mmat[i][j] = mm[i][j];
            mmat[j][i] = mm[i][j];
        }
    }
}

void a1frcmat(double fmat[2])
{
/* Return the system force matrix (RHS), excluding constraints
*/
    int i;

    if ((roustate != 2) && (roustate != 3)) {
        a1seterr(58,23);
        return;
    }
    a1dofs0();
    for (i = 0; i < 2; i++) {
        fmat[i] = fs0[i];
    }
}

void a1pseudo(double lqout[1],
    double luout[1])
{
/*
Return pseudo-coordinates for loop joints.

*/
/*
There are no loop joints in this system.

*/
}

void a1psqdot(double lqdout[1])
{
/*
Return pseudo-coordinate derivatives for loop joints.

*/
/*
There are no loop joints in this system.

*/
}

void a1psudot(double ludout[1])
{
/*
Return pseudo-coordinate accelerations for loop joints.

*/
/*
There are no loop joints in this system.

*/
}

void a1perr(double errs[1])
{

}

void a1verr(double errs[1])
{

}

void a1aerr(double errs[1])
{

}
int 
a1chkbnum(int routine,
    int bnum)
{

    if ((bnum < -1) || (bnum > 1)) {
        a1seterr(routine,15);
        return 1;
    }
    return 0;
}
int 
a1chkjnum(int routine,
    int jnum)
{

    if ((jnum < 0) || (jnum > 1)) {
        a1seterr(routine,16);
        return 1;
    }
    return 0;
}
int 
a1chkucnum(int routine,
    int ucnum)
{

    if ((ucnum < 0) || (ucnum > -1)) {
        a1seterr(routine,21);
        return 1;
    }
    return 0;
}
int 
a1chkjaxis(int routine,
    int jnum,
    int axnum)
{
    int maxax;

    if (a1chkjnum(routine,jnum) != 0) {
        return 1;
    }
    if ((axnum < 0) || (axnum > 6)) {
        a1seterr(routine,17);
        return 1;
    }
    maxax = njntdof[jnum]-1;
    if ((jtype[jnum] == 4) || (jtype[jnum] == 6) || (jtype[jnum] == 21)) {
        maxax = maxax+1;
    }
    if (axnum > maxax) {
        a1seterr(routine,18);
        return 1;
    }
    return 0;
}
int 
a1chkjpin(int routine,
    int jnum,
    int pinno)
{
    int maxax,pinok;

    if (a1chkjnum(routine,jnum) != 0) {
        return 1;
    }
    if ((pinno < 0) || (pinno > 5)) {
        a1seterr(routine,17);
        return 1;
    }
    if (njntdof[jnum] >= 3) {
        maxax = 2;
    } else {
        maxax = njntdof[jnum]-1;
    }
    if (jtype[jnum] == 4) {
        maxax = -1;
    }
    if (jtype[jnum] == 7) {
        maxax = 0;
    }
    pinok = 0;
    if (pinno <= maxax) {
        pinok = 1;
    }
    if (pinok == 0) {
        a1seterr(routine,18);
        return 1;
    }
    return 0;
}
int 
a1indx(int joint,
    int axis)
{
    int offs,gotit;

    if (a1chkjaxis(36,joint,axis) != 0) {
        return 0;
    }
    gotit = 0;
    if (jtype[joint] == 4) {
        if (axis == 3) {
            offs = ballq[joint];
            gotit = 1;
        }
    } else {
        if ((jtype[joint] == 6) || (jtype[joint] == 21)) {
            if (axis == 6) {
                offs = ballq[joint];
                gotit = 1;
            }
        }
    }
    if (gotit == 0) {
        offs = firstq[joint]+axis;
    }
    return offs;
}

void a1presacc(int joint,
    int axis,
    double prval)
{

}

void a1presvel(int joint,
    int axis,
    double prval)
{

}

void a1prespos(int joint,
    int axis,
    double prval)
{

}

void a1getht(int joint,
    int axis,
    double *torque)
{

    if (a1chkjaxis(30,joint,axis) != 0) {
        return;
    }
    if (roustate != 3) {
        a1seterr(30,24);
        return;
    }
    *torque = tauc[a1indx(joint,axis)];
}

void a1hinget(int joint,
    int axis,
    double torque)
{

    if (a1chkjaxis(10,joint,axis) != 0) {
        return;
    }
    if (roustate != 2) {
        a1seterr(10,23);
        return;
    }
    if (mfrcflg != 0) {
        mtau[a1indx(joint,axis)] = mtau[a1indx(joint,axis)]+torque;
    } else {
        fs0flg = 0;
        utau[a1indx(joint,axis)] = utau[a1indx(joint,axis)]+torque;
    }
}

void a1pointf(int body,
    double point[3],
    double force[3])
{
    double torque[3];

    if (a1chkbnum(11,body) != 0) {
        return;
    }
    if (roustate != 2) {
        a1seterr(11,23);
        return;
    }
    if (body == -1) {
        return;
    }
    torque[0] = point[1]*force[2]-point[2]*force[1];
    torque[1] = point[2]*force[0]-point[0]*force[2];
    torque[2] = point[0]*force[1]-point[1]*force[0];
    if (mfrcflg != 0) {
        mfk[body][0] = mfk[body][0]+force[0];
        mtk[body][0] = mtk[body][0]+torque[0];
        mfk[body][1] = mfk[body][1]+force[1];
        mtk[body][1] = mtk[body][1]+torque[1];
        mfk[body][2] = mfk[body][2]+force[2];
        mtk[body][2] = mtk[body][2]+torque[2];
    } else {
        fs0flg = 0;
        ufk[body][0] = ufk[body][0]+force[0];
        utk[body][0] = utk[body][0]+torque[0];
        ufk[body][1] = ufk[body][1]+force[1];
        utk[body][1] = utk[body][1]+torque[1];
        ufk[body][2] = ufk[body][2]+force[2];
        utk[body][2] = utk[body][2]+torque[2];
    }
}

void a1bodyt(int body,
    double torque[3])
{

    if (a1chkbnum(12,body) != 0) {
        return;
    }
    if (roustate != 2) {
        a1seterr(12,23);
        return;
    }
    if (body == -1) {
        return;
    }
    if (mfrcflg != 0) {
        mtk[body][0] = mtk[body][0]+torque[0];
        mtk[body][1] = mtk[body][1]+torque[1];
        mtk[body][2] = mtk[body][2]+torque[2];
    } else {
        fs0flg = 0;
        utk[body][0] = utk[body][0]+torque[0];
        utk[body][1] = utk[body][1]+torque[1];
        utk[body][2] = utk[body][2]+torque[2];
    }
}

void a1doww(int routine)
{

    roustate = 2;
    if (wwflg == 0) {
        wwflg = 1;
    }
/*
 Used -0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain    0 adds/subtracts/negates
                      0 multiplies
                      0 divides
                      0 assignments
*/
}

void a1xudot0(int routine,
    double oudot0[2])
{
/*
Compute unconstrained equations
*/
    int i;

    a1lhs(routine);
/*
Solve equations for udots
*/
    a1fs0();
    a1ldubslv(2,2,mmap,works,mlo,mdi,fs,udot);
    for (i = 0; i <= 1; i++) {
        oudot0[i] = udot[i];
    }
/*
 Used -0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain    0 adds/subtracts/negates
                      0 multiplies
                      0 divides
                      2 assignments
*/
}

void a1udot0(double oudot0[2])
{

    if ((roustate != 2) && (roustate != 3)) {
        a1seterr(66,23);
        return;
    }
    a1xudot0(66,oudot0);
}

void a1setudot(double iudot[2])
{
/*
Assign udots and advance to stage Dynamics Ready
*/
    int i;

    if ((roustate != 2) && (roustate != 3)) {
        a1seterr(68,23);
        return;
    }
    for (i = 0; i <= 1; i++) {
        udot[i] = iudot[i];
    }
    a1rhs();
}

void a1xudotm(int routine,
    double imult[1],
    double oudotm[2])
{
/*
Compute udots due only to multipliers
*/
    int i;

    a1lhs(routine);
    for (i = 0; i <= 1; i++) {
        udot[i] = 0.;
    }
    for (i = 0; i <= 1; i++) {
        oudotm[i] = udot[i];
    }
/*
 Used -0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain    0 adds/subtracts/negates
                      0 multiplies
                      0 divides
                      4 assignments
*/
}

void a1udotm(double imult[1],
    double oudotm[2])
{

    if ((roustate != 2) && (roustate != 3)) {
        a1seterr(67,23);
        return;
    }
    a1xudotm(67,imult,oudotm);
}

void a1deriv(double oqdot[2],
    double oudot[2])
{
/*
This is the derivative section for a 2-body ground-based
system with 2 hinge degree(s) of freedom.
*/
    int i;
    double udot0[2],udot1[2];

    if ((roustate != 2) && (roustate != 3)) {
        a1seterr(17,23);
        return;
    }
    if (stabvelq == 1) {
        a1seterr(17,32);
    }
    if (stabposq == 1) {
        a1seterr(17,33);
    }
    wsiz = 0;
/*
Compute unconstrained equations
*/
    a1xudot0(17,udot0);
    a1rhs();
    wrank = 0;
    for (i = 0; i <= 1; i++) {
        oqdot[i] = qdot[i];
    }
    for (i = 0; i <= 1; i++) {
        oudot[i] = udot[i];
    }
/*
 Used -0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain    0 adds/subtracts/negates
                      0 multiplies
                      0 divides
                      4 assignments
*/
}
/*
Compute residuals for use with DAE integrator
*/

void a1resid(double eqdot[2],
    double eudot[2],
    double emults[1],
    double resid[4])
{
    int i;
    double uderrs[2];

    if ((roustate != 2) && (roustate != 3)) {
        a1seterr(16,23);
        return;
    }
    if (stabposq == 1) {
        a1seterr(16,33);
    }
    a1fulltrq(eudot,emults,uderrs);
    for (i = 0; i < 2; i++) {
        resid[i] = eqdot[i]-qdot[i];
    }
    for (i = 0; i < 2; i++) {
        resid[2+i] = uderrs[i];
    }
    for (i = 0; i < 2; i++) {
        udot[i] = eudot[i];
    }
    a1rhs();
/*
 Used -0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain    2 adds/subtracts/negates
                      0 multiplies
                      0 divides
                      6 assignments
*/
}

void a1mult(double omults[1],
    int *owrank,
    int omultmap[1])
{

    if (roustate != 3) {
        a1seterr(34,24);
        return;
    }
    *owrank = wrank;
}

void a1reac(double force[2][3],
    double torque[2][3])
{
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

    if (roustate != 3) {
        a1seterr(31,24);
        return;
    }
/*
Compute reaction forces for non-weld tree joints
*/
    fc[1][0] = ((.226*(AnkAtk[1][0]-gk[1][0]))-ufk[1][0]);
    fc[1][1] = ((.226*(AnkAtk[1][1]-gk[1][1]))-ufk[1][1]);
    fc[1][2] = -ufk[1][2];
    tc[1][0] = -(utk[1][0]+(.1*fc[1][2]));
    tc[1][1] = -utk[1][1];
    tc[1][2] = ((.02034*Onkb[1][2])-(utk[1][2]-(.1*fc[1][0])));
    fccikt[1][0] = ((fc[1][0]*c1)-(fc[1][1]*s1));
    fccikt[1][1] = ((fc[1][0]*s1)+(fc[1][1]*c1));
    fccikt[1][2] = fc[1][2];
    ffk[0][0] = (ufk[0][0]-fccikt[1][0]);
    ffk[0][1] = (ufk[0][1]-fccikt[1][1]);
    ffk[0][2] = (ufk[0][2]-fccikt[1][2]);
    ttk[0][0] = (utk[0][0]-(((tc[1][0]*c1)-(tc[1][1]*s1))-(.1*fccikt[1][2])));
    ttk[0][1] = (utk[0][1]-((tc[1][0]*s1)+(tc[1][1]*c1)));
    ttk[0][2] = (utk[0][2]-(tc[1][2]+(.1*fccikt[1][0])));
    fc[0][0] = ((1.013*((.1*udot[0])+(9.8*s0)))-ffk[0][0]);
    fc[0][1] = ((1.013*(Atk[0][1]+(9.8*c0)))-ffk[0][1]);
    fc[0][2] = -ffk[0][2];
    tc[0][0] = -(ttk[0][0]+(.1*fc[0][2]));
    tc[0][1] = -ttk[0][1];
    tc[0][2] = ((.01013*udot[0])-(ttk[0][2]-(.1*fc[0][0])));
    force[0][0] = fc[0][0];
    torque[0][0] = tc[0][0];
    force[0][1] = fc[0][1];
    torque[0][1] = tc[0][1];
    force[0][2] = fc[0][2];
    torque[0][2] = tc[0][2];
    force[1][0] = fc[1][0];
    torque[1][0] = tc[1][0];
    force[1][1] = fc[1][1];
    torque[1][1] = tc[1][1];
    force[1][2] = fc[1][2];
    torque[1][2] = tc[1][2];
/*
Compute reaction forces for tree weld joints
*/
/*
 Used -0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain   32 adds/subtracts/negates
                     23 multiplies
                      0 divides
                     33 assignments
*/
}

void a1mom(double lm[3],
    double am[3],
    double *ke)
{
/*
Compute system linear and angular momentum, and kinetic energy.

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
    double lk[2][3],hnk[2][3];

    if ((roustate != 2) && (roustate != 3)) {
        a1seterr(19,23);
        return;
    }
    lm[0] = ((.226*vnk[1][0])+(1.013*vnk[0][0]));
    lm[1] = ((.226*vnk[1][1])+(1.013*vnk[0][1]));
    lm[2] = 0.;
    am[0] = 0.;
    am[1] = 0.;
    am[2] = ((((.01013*u[0])+(.1013*((vnk[0][0]*c0)+(vnk[0][1]*s0))))+((.02034*
      wk[1][2])+(.226*((rnk[1][0]*vnk[1][1])-(rnk[1][1]*vnk[1][0])))))-((com[0]*
      lm[1])-(com[1]*lm[0])));
    *ke = (.5*(((.01013*(u[0]*u[0]))+(1.013*((vnk[0][0]*vnk[0][0])+(vnk[0][1]*
      vnk[0][1]))))+((.02034*(wk[1][2]*wk[1][2]))+(.226*((vnk[1][0]*vnk[1][0])+(
      vnk[1][1]*vnk[1][1]))))));
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain   14 adds/subtracts/negates
                     25 multiplies
                      0 divides
                      7 assignments
*/
}

void a1sys(double *mtoto,
    double cm[3],
    double icm[3][3])
{
/*
Compute system total mass, and instantaneous center of mass and
inertia matrix.

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
    double ikcnkt[2][3][3];

    if ((roustate != 2) && (roustate != 3)) {
        a1seterr(20,23);
        return;
    }
    *mtoto = 1.239;
    cm[0] = com[0];
    cm[1] = com[1];
    cm[2] = 0.;
    icm[0][0] = (((.02026*(c0*c0))+((.02034*(cnk[1][0][0]*cnk[1][0][0]))+(.226*(
      rnk[1][1]*rnk[1][1]))))-(1.239*(com[1]*com[1])));
    icm[0][1] = ((1.239*(com[0]*com[1]))+((.02026*(s0*c0))+((.02034*(
      cnk[1][0][0]*cnk[1][1][0]))-(.226*(rnk[1][0]*rnk[1][1])))));
    icm[0][2] = 0.;
    icm[1][0] = icm[0][1];
    icm[1][1] = (((.02026*(s0*s0))+((.02034*(cnk[1][1][0]*cnk[1][1][0]))+(.226*(
      rnk[1][0]*rnk[1][0]))))-(1.239*(com[0]*com[0])));
    icm[1][2] = 0.;
    icm[2][0] = icm[0][2];
    icm[2][1] = icm[1][2];
    icm[2][2] = (.0406+((.226*((rnk[1][0]*rnk[1][0])+(rnk[1][1]*rnk[1][1])))-(
      1.239*((com[0]*com[0])+(com[1]*com[1])))));
/*
 Used -0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain   13 adds/subtracts/negates
                     30 multiplies
                      0 divides
                     13 assignments
*/
}

void a1pos(int body,
    double pt[3],
    double loc[3])
{
/*
Return inertial frame location of a point on a body.

*/
    double pv[3];

    if (a1chkbnum(21,body) != 0) {
        return;
    }
    if ((roustate != 2) && (roustate != 3)) {
        a1seterr(21,23);
        return;
    }
    if (body  ==  -1) {
        loc[0] = pt[0];
        loc[1] = pt[1];
        loc[2] = pt[2];
    } else {
        pv[0] = rnb[body][0]+pt[0]*cnb[body][0][0]+pt[1]*cnb[body][0][1]+pt[2]*
          cnb[body][0][2];
        pv[1] = rnb[body][1]+pt[0]*cnb[body][1][0]+pt[1]*cnb[body][1][1]+pt[2]*
          cnb[body][1][2];
        pv[2] = rnb[body][2]+pt[0]*cnb[body][2][0]+pt[1]*cnb[body][2][1]+pt[2]*
          cnb[body][2][2];
        loc[0] = pv[0];
        loc[1] = pv[1];
        loc[2] = pv[2];
    }
}

void a1vel(int body,
    double pt[3],
    double velo[3])
{
/*
Return inertial frame velocity of a point on a body.

*/
    double pv[3];

    if (a1chkbnum(22,body) != 0) {
        return;
    }
    if ((roustate != 2) && (roustate != 3)) {
        a1seterr(22,23);
        return;
    }
    if (body  ==  -1) {
        velo[0] = 0.;
        velo[1] = 0.;
        velo[2] = 0.;
    } else {
        pv[0] = wb[body][1]*pt[2]-wb[body][2]*pt[1];
        pv[1] = wb[body][2]*pt[0]-wb[body][0]*pt[2];
        pv[2] = wb[body][0]*pt[1]-wb[body][1]*pt[0];
        velo[0] = vnb[body][0]+pv[0]*cnb[body][0][0]+pv[1]*cnb[body][0][1]+pv[2]
          *cnb[body][0][2];
        velo[1] = vnb[body][1]+pv[0]*cnb[body][1][0]+pv[1]*cnb[body][1][1]+pv[2]
          *cnb[body][1][2];
        velo[2] = vnb[body][2]+pv[0]*cnb[body][2][0]+pv[1]*cnb[body][2][1]+pv[2]
          *cnb[body][2][2];
    }
}

void a1orient(int body,
    double dircos[3][3])
{
/*
Return orientation of body w.r.t. ground frame.

*/

    if (a1chkbnum(23,body) != 0) {
        return;
    }
    if ((roustate != 2) && (roustate != 3)) {
        a1seterr(23,23);
        return;
    }
    if (body == -1) {
        dircos[0][0] = 1.;
        dircos[0][1] = 0.;
        dircos[0][2] = 0.;
        dircos[1][0] = 0.;
        dircos[1][1] = 1.;
        dircos[1][2] = 0.;
        dircos[2][0] = 0.;
        dircos[2][1] = 0.;
        dircos[2][2] = 1.;
    } else {
        dircos[0][0] = cnb[body][0][0];
        dircos[0][1] = cnb[body][0][1];
        dircos[0][2] = cnb[body][0][2];
        dircos[1][0] = cnb[body][1][0];
        dircos[1][1] = cnb[body][1][1];
        dircos[1][2] = cnb[body][1][2];
        dircos[2][0] = cnb[body][2][0];
        dircos[2][1] = cnb[body][2][1];
        dircos[2][2] = cnb[body][2][2];
    }
}

void a1angvel(int body,
    double avel[3])
{
/*
Return angular velocity of the body.

*/

    if (a1chkbnum(24,body) != 0) {
        return;
    }
    if ((roustate != 2) && (roustate != 3)) {
        a1seterr(24,23);
        return;
    }
    if (body == -1) {
        avel[0] = 0.;
        avel[1] = 0.;
        avel[2] = 0.;
    } else {
        avel[0] = wb[body][0];
        avel[1] = wb[body][1];
        avel[2] = wb[body][2];
    }
}

void a1trans(int frbod,
    double ivec[3],
    int tobod,
    double ovec[3])
{
/*
Transform ivec from frbod frame to tobod frame.

*/
    double pv[3];

    if (a1chkbnum(25,frbod) != 0) {
        return;
    }
    if (a1chkbnum(25,tobod) != 0) {
        return;
    }
    if ((roustate != 2) && (roustate != 3)) {
        a1seterr(25,23);
        return;
    }
    if (frbod == tobod) {
        a1vcopy(ivec,ovec);
        return;
    }
    if (frbod == -1) {
        a1vcopy(ivec,pv);
        ovec[0] = pv[0]*cnb[tobod][0][0]+pv[1]*cnb[tobod][1][0]+pv[2]*cnb[tobod
          ][2][0];
        ovec[1] = pv[0]*cnb[tobod][0][1]+pv[1]*cnb[tobod][1][1]+pv[2]*cnb[tobod
          ][2][1];
        ovec[2] = pv[0]*cnb[tobod][0][2]+pv[1]*cnb[tobod][1][2]+pv[2]*cnb[tobod
          ][2][2];
        return;
    }
    if (tobod == -1) {
        a1vcopy(ivec,pv);
        ovec[0] = pv[0]*cnb[frbod][0][0]+pv[1]*cnb[frbod][0][1]+pv[2]*cnb[frbod
          ][0][2];
        ovec[1] = pv[0]*cnb[frbod][1][0]+pv[1]*cnb[frbod][1][1]+pv[2]*cnb[frbod
          ][1][2];
        ovec[2] = pv[0]*cnb[frbod][2][0]+pv[1]*cnb[frbod][2][1]+pv[2]*cnb[frbod
          ][2][2];
        return;
    }
    pv[0] = ivec[0]*cnb[frbod][0][0]+ivec[1]*cnb[frbod][0][1]+ivec[2]*cnb[frbod
      ][0][2];
    pv[1] = ivec[0]*cnb[frbod][1][0]+ivec[1]*cnb[frbod][1][1]+ivec[2]*cnb[frbod
      ][1][2];
    pv[2] = ivec[0]*cnb[frbod][2][0]+ivec[1]*cnb[frbod][2][1]+ivec[2]*cnb[frbod
      ][2][2];
    ovec[0] = pv[0]*cnb[tobod][0][0]+pv[1]*cnb[tobod][1][0]+pv[2]*cnb[tobod][2][
      0];
    ovec[1] = pv[0]*cnb[tobod][0][1]+pv[1]*cnb[tobod][1][1]+pv[2]*cnb[tobod][2][
      1];
    ovec[2] = pv[0]*cnb[tobod][0][2]+pv[1]*cnb[tobod][1][2]+pv[2]*cnb[tobod][2][
      2];
}

void a1rel2cart(int coord,
    int body,
    double point[3],
    double linchg[3],
    double rotchg[3])
{
/* Return derivative of pt loc and body orient w.r.t. hinge rate
*/
    int x,i,gnd;
    double lin[3],pv[3];

    if ((coord < 0) || (coord > 1)) {
        a1seterr(59,45);
        return;
    }
    if (a1chkbnum(59,body) != 0) {
        return;
    }
    if ((roustate != 2) && (roustate != 3)) {
        a1seterr(59,23);
        return;
    }
    gnd = -1;
    if (body == gnd) {
        x = -1;
    } else {
        x = firstq[body]+njntdof[body]-1;
    }
    if (x < coord) {
        a1vset(0.,0.,0.,linchg);
        a1vset(0.,0.,0.,rotchg);
        return;
    }
    a1dovpk();
    for (i = 0; i < 3; i++) {
        rotchg[i] = Wpk[coord][x][i];
        lin[i] = Vpk[coord][x][i];
    }
    if (body == gnd) {
        a1vcopy(point,pv);
    } else {
        pv[0] = rcom[body][0]+point[0];
        pv[1] = rcom[body][1]+point[1];
        pv[2] = rcom[body][2]+point[2];
    }
    a1vcross(rotchg,pv,linchg);
    a1vadd(linchg,lin,linchg);
}

void a1acc(int body,
    double pt[3],
    double accel[3])
{
/*
Return linear acceleration a point of the specified body.

*/
    double pv[3];

    if (a1chkbnum(32,body) != 0) {
        return;
    }
    if (roustate != 3) {
        a1seterr(32,24);
        return;
    }
    if (body  ==  -1) {
        accel[0] = 0.;
        accel[1] = 0.;
        accel[2] = 0.;
    } else {
        pv[0] = pt[0]*dyad[body][0][0]+pt[1]*dyad[body][0][1]+pt[2]*dyad[body][0
          ][2];
        pv[1] = pt[0]*dyad[body][1][0]+pt[1]*dyad[body][1][1]+pt[2]*dyad[body][1
          ][2];
        pv[2] = pt[0]*dyad[body][2][0]+pt[1]*dyad[body][2][1]+pt[2]*dyad[body][2
          ][2];
        accel[0] = anb[body][0]+pv[0]*cnb[body][0][0]+pv[1]*cnb[body][0][1]+pv[2
          ]*cnb[body][0][2];
        accel[1] = anb[body][1]+pv[0]*cnb[body][1][0]+pv[1]*cnb[body][1][1]+pv[2
          ]*cnb[body][1][2];
        accel[2] = anb[body][2]+pv[0]*cnb[body][2][0]+pv[1]*cnb[body][2][1]+pv[2
          ]*cnb[body][2][2];
    }
}

void a1angacc(int body,
    double aacc[3])
{
/*
Return angular acceleration of the body.

*/

    if (a1chkbnum(33,body) != 0) {
        return;
    }
    if (roustate != 3) {
        a1seterr(33,24);
        return;
    }
    if (body == -1) {
        aacc[0] = 0.;
        aacc[1] = 0.;
        aacc[2] = 0.;
    } else {
        aacc[0] = onb[body][0];
        aacc[1] = onb[body][1];
        aacc[2] = onb[body][2];
    }
}

void a1grav(double gravin[3])
{

    a1seterr(1,19);
    roustate = 0;
}

void a1mass(int body,
    double massin)
{

    if (a1chkbnum(2,body) != 0) {
        return;
    }
    if (body == -1) {
        a1seterr(2,15);
        return;
    }
    if (mkq[body] != 0) {
        mk[body] = massin;
        mkq[body] = 3;
    } else {
        a1seterr(2,19);
    }
    roustate = 0;
}

void a1iner(int body,
    double inerin[3][3])
{
    int anyques;

    if (a1chkbnum(3,body) != 0) {
        return;
    }
    if (body == -1) {
        a1seterr(3,15);
        return;
    }
    anyques = 0;
    if (ikq[body][0][0]  !=  0) {
        ik[body][0][0] = inerin[0][0];
        ikq[body][0][0] = 3;
        anyques = 1;
    }
    if (ikq[body][0][1]  !=  0) {
        ik[body][0][1] = inerin[0][1];
        ikq[body][0][1] = 3;
        ik[body][1][0] = inerin[0][1];
        ikq[body][1][0] = 3;
        anyques = 1;
    }
    if (ikq[body][0][2]  !=  0) {
        ik[body][0][2] = inerin[0][2];
        ikq[body][0][2] = 3;
        ik[body][2][0] = inerin[0][2];
        ikq[body][2][0] = 3;
        anyques = 1;
    }
    if (ikq[body][1][1]  !=  0) {
        ik[body][1][1] = inerin[1][1];
        ikq[body][1][1] = 3;
        anyques = 1;
    }
    if (ikq[body][1][2]  !=  0) {
        ik[body][1][2] = inerin[1][2];
        ikq[body][1][2] = 3;
        ik[body][2][1] = inerin[1][2];
        ikq[body][2][1] = 3;
        anyques = 1;
    }
    if (ikq[body][2][2]  !=  0) {
        ik[body][2][2] = inerin[2][2];
        ikq[body][2][2] = 3;
        anyques = 1;
    }
    if (anyques == 0) {
        a1seterr(3,19);
    }
    roustate = 0;
}

void a1btj(int joint,
    double btjin[3])
{
    int anyques;

    if (a1chkjnum(4,joint) != 0) {
        return;
    }
    anyques = 0;
    if (rkq[joint][0]  !=  0) {
        rk[joint][0] = btjin[0];
        rkq[joint][0] = 3;
        anyques = 1;
    }
    if (rkq[joint][1]  !=  0) {
        rk[joint][1] = btjin[1];
        rkq[joint][1] = 3;
        anyques = 1;
    }
    if (rkq[joint][2]  !=  0) {
        rk[joint][2] = btjin[2];
        rkq[joint][2] = 3;
        anyques = 1;
    }
    if (anyques == 0) {
        a1seterr(4,19);
    }
    roustate = 0;
}

void a1itj(int joint,
    double itjin[3])
{
    int anyques;

    if (a1chkjnum(5,joint) != 0) {
        return;
    }
    anyques = 0;
    if (riq[joint][0]  !=  0) {
        ri[joint][0] = itjin[0];
        riq[joint][0] = 3;
        anyques = 1;
    }
    if (riq[joint][1]  !=  0) {
        ri[joint][1] = itjin[1];
        riq[joint][1] = 3;
        anyques = 1;
    }
    if (riq[joint][2]  !=  0) {
        ri[joint][2] = itjin[2];
        riq[joint][2] = 3;
        anyques = 1;
    }
    if (anyques == 0) {
        a1seterr(5,19);
    }
    roustate = 0;
}

void a1pin(int joint,
    int pinno,
    double pinin[3])
{
    int anyques,offs;

    if (a1chkjpin(6,joint,pinno) != 0) {
        return;
    }
    anyques = 0;
    offs = firstq[joint]+pinno;
    if (jtype[joint] == 21) {
        offs = offs+3;
    }
    if (jtype[joint] == 11) {
        offs = offs+1;
    }
    if (pinq[offs][0]  !=  0) {
        pin[offs][0] = pinin[0];
        pinq[offs][0] = 3;
        anyques = 1;
    }
    if (pinq[offs][1]  !=  0) {
        pin[offs][1] = pinin[1];
        pinq[offs][1] = 3;
        anyques = 1;
    }
    if (pinq[offs][2]  !=  0) {
        pin[offs][2] = pinin[2];
        pinq[offs][2] = 3;
        anyques = 1;
    }
    if (anyques == 0) {
        a1seterr(6,19);
    }
    roustate = 0;
}

void a1pres(int joint,
    int axis,
    int presin)
{
    int anyques;

    if (a1chkjaxis(37,joint,axis) != 0) {
        return;
    }
    if ((presin != 0) && (presin != 1)) {
        a1seterr(37,20);
    }
    anyques = 0;
    if (presq[a1indx(joint,axis)]  !=  0) {
        if (presin  !=  0) {
            pres[a1indx(joint,axis)] = 1.;
        } else {
            pres[a1indx(joint,axis)] = 0.;
        }
        presq[a1indx(joint,axis)] = 3;
        anyques = 1;
    }
    if (anyques == 0) {
        a1seterr(37,19);
    }
    wwflg = 0;
}

void a1conschg(void)
{

    wwflg = 0;
}

void a1stab(double velin,
    double posin)
{

    stabvel = velin;
    stabvelq = 3;
    stabpos = posin;
    stabposq = 3;
}

void a1getgrav(double gravout[3])
{

    gravout[0] = grav[0];
    gravout[1] = grav[1];
    gravout[2] = grav[2];
}

void a1getmass(int body,
    double *massout)
{

    if (a1chkbnum(40,body) != 0) {
        return;
    }
    if (body == -1) {
        a1seterr(40,15);
        return;
    }
    *massout = mk[body];
}

void a1getiner(int body,
    double inerout[3][3])
{

    if (a1chkbnum(41,body) != 0) {
        return;
    }
    if (body == -1) {
        a1seterr(41,15);
        return;
    }
    inerout[0][0] = ik[body][0][0];
    inerout[0][1] = ik[body][0][1];
    inerout[0][2] = ik[body][0][2];
    inerout[1][0] = ik[body][1][0];
    inerout[1][1] = ik[body][1][1];
    inerout[1][2] = ik[body][1][2];
    inerout[2][0] = ik[body][2][0];
    inerout[2][1] = ik[body][2][1];
    inerout[2][2] = ik[body][2][2];
}

void a1getbtj(int joint,
    double btjout[3])
{

    if (a1chkjnum(42,joint) != 0) {
        return;
    }
    btjout[0] = rk[joint][0];
    btjout[1] = rk[joint][1];
    btjout[2] = rk[joint][2];
}

void a1getitj(int joint,
    double itjout[3])
{

    if (a1chkjnum(43,joint) != 0) {
        return;
    }
    itjout[0] = ri[joint][0];
    itjout[1] = ri[joint][1];
    itjout[2] = ri[joint][2];
}

void a1getpin(int joint,
    int pinno,
    double pinout[3])
{
    int offs;

    if (a1chkjpin(44,joint,pinno) != 0) {
        return;
    }
    offs = firstq[joint]+pinno;
    if (jtype[joint] == 21) {
        offs = offs+3;
    }
    if (jtype[joint] == 11) {
        offs = offs+1;
    }
    pinout[0] = pin[offs][0];
    pinout[1] = pin[offs][1];
    pinout[2] = pin[offs][2];
}

void a1getpres(int joint,
    int axis,
    int *presout)
{

    if (a1chkjaxis(45,joint,axis) != 0) {
        return;
    }
    if (pres[a1indx(joint,axis)]  !=  0.) {
        *presout = 1;
    } else {
        *presout = 0;
    }
}

void a1getstab(double *velout,
    double *posout)
{

    *velout = stabvel;
    *posout = stabpos;
}

void a1info(int info[50])
{

    info[0] = ground;
    info[1] = nbod;
    info[2] = ndof;
    info[3] = ncons;
    info[4] = nloop;
    info[5] = nldof;
    info[6] = nloopc;
    info[7] = nball;
    info[8] = nlball;
    info[9] = npres;
    info[10] = nuser;
    info[11] = 0;
/* info entries from 12-49 are reserved */
}

void a1jnt(int joint,
    int info[50],
    int tran[6])
{
    int i,offs;

    if (a1chkjnum(48,joint) != 0) {
        return;
    }
    info[0] = jtype[joint];
    info[1] = 0;
    offs = 0;
    info[2] = inb[joint];
    info[3] = outb[joint];
    info[4] = njntdof[joint];
    info[5] = njntc[joint];
    info[6] = njntp[joint];
    info[7] = firstq[joint];
    info[8] = ballq[joint];
    info[9] = firstm[joint];
    info[10] = firstp[joint];
/* info entries from 11-49 are reserved */

    for (i = 0; i <= 5; i++) {
        if (i  <  njntdof[joint]) {
            tran[i] = trans[offs+firstq[joint]+i];
        } else {
            tran[i] = -1;
        }
    }
}

void a1cons(int consno,
    int info[50])
{

    if (a1chkucnum(49,consno) != 0) {
        return;
    }
/* There are no user constraints in this problem. */
}

void a1gentime(int *gentm)
{

    *gentm = 171519;
}
/*
Done. CPU seconds used: 0.01  Memory used: 1736704 bytes.
Equation complexity:
  sdstate:    22 adds    41 multiplies     0 divides    75 assignments
  sdderiv:    41 adds    48 multiplies     2 divides   103 assignments
  sdresid:    44 adds    44 multiplies     0 divides    67 assignments
  sdreac:     32 adds    23 multiplies     0 divides    33 assignments
  sdmom:      14 adds    25 multiplies     0 divides     7 assignments
  sdsys:      13 adds    30 multiplies     0 divides    13 assignments
*/
