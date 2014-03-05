/**********************************************************************
* Network messaging library for ARSENL Aerial Combat Swarms
* Developed by Mike Clement, Winter 2013
* 
* geo.c - convert geodetic coordinates and local tangent plane
* 
* Notes:
*  - 
**********************************************************************/
#ifndef _ACS_GEO
#define _ACS_GEO

#include <math.h>  /* Evil, huge glibc math library */

/* Structure that defines a specific LTP and holds pre-evaluated
   matrices to expedite repetitive computation */
struct acs_geo {
  double	lla[3];		/* LLA of origin point */
  double	ecef[3];	/* ECEF-r of origin point (calculated) */
  double	R[3][3];	/* Rotational matrix (calculated) */
};

void acs_geo_init(struct acs_geo *geo, 
		  double lat, double lon, double hae);

void acs_geo_llatoltp(struct acs_geo *geo,
		      double *east, double *north, double *up,
		      double lat, double lon, double hae);

void acs_geo_ltptolla(struct acs_geo *geo,
		      double *lat, double *lon, double *hae,
		      double east, double north, double up);

#endif
