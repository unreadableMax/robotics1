#include <cmath>

#include "def_usrmod.hpp"

#define  NMOS   1  /* Number of phases (Model Stages) */
#define  NP     0  /* Number of parameters */
#define  NRC    0  /* Number of coupled constraints */
#define  NRCE   0  /* Number of coupled equality constraints */

#define  NXD    2  /* Number of differential states */
#define  NXA    0  /* Number of algebraic states */
#define  NU     0  /* Number of controls */
#define  NPR    0  /* Number of local parameters */

#define  NRD_S  1  /* Number of constraints at the start point */
#define  NRDE_S 1  /* Number of equality constraints at the start points */

#define  NRD_E  1  /* Number of constraints at the end point */
#define  NRDE_E 1  /* Number of equality constraints at the end point */

//vary this for exercise 2.
static double lambda1 = 15.0;

/** \brief Right hand side of the differential equation */
/*t, xd, xa, u, and p are double pointers to the 
current time, differential and algebraic states, controls, and model parameters*/
static void ffcn(double *t, double *xd, double *xa, double *u,
  double *p, double *rhs, double *rwh, long *iwh, InfoPtr *info)
{
  rhs[0] =xd[1];  //v
  rhs[1] =-xd[0]; //-x
}

/** \brief Constraints at the start point */
static void rdfcn_s(double *ts, double *sd, double *sa, double *u,
  double *p, double *pr, double *res, long *dpnd, InfoPtr *info)
{
  if (*dpnd) {
    *dpnd = RFCN_DPND(0, *sd, 0, 0, 0, 0);
    return;
  }

  res[0] = 0.0;
}

/** \brief Constraints at the end point */
static void rdfcn_e(double *ts, double *sd, double *sa, double *u,
  double *p, double *pr, double *res, long *dpnd, InfoPtr *info)
{
  if (*dpnd) {
    *dpnd = RFCN_DPND(0, *sd, 0, 0, 0, 0);
    return;
  }

  res[0] = 0.0;
}

/** \brief Entry point for the muscod application */
extern "C" void def_model(void);
void def_model(void)
{
	/* Define problem dimensions */
	def_mdims(NMOS, NP, NRC, NRCE);
	/* Define the first (and only) phase */
	def_mstage(
			0,
			NXD, NXA, NU,
			NULL, NULL,
			0, 0, 0, NULL, ffcn, NULL,
			NULL, NULL
			);
	/* Define constraints at the start point */
	def_mpc(0, "Start Point", NPR, NRD_S, NRDE_S, rdfcn_s, NULL);
	/* Define constraints at the end point */
	def_mpc(0, "End Point", NPR, NRD_E, NRDE_E, rdfcn_e, NULL);
}

