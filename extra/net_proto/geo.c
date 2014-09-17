/**********************************************************************
* Network messaging library for ARSENL Aerial Combat Swarms
* Developed by Mike Clement, Winter 2013
* 
* geo.c - convert between geodetic (ECEF-g) and local tangent plane (LTP)
* 
* Notes:
*  - Uses HAE not MSL for altitude
*  - LTP defined as east-north-up, so playing field is oriented accordingly
*  - This whole thing is mostly untested and the accuracy/precision
*    has not be characterized; use with caution!
*  - See the following references for details on the math:
*    "Data Transformations of GPS Positions" (u-blox ag, July 1999)
*    "Conversion of Geodetic coordinates to the Local Tangent Plane"
*    (Portland State Aerospace Society, September 2007)
**********************************************************************/
#include "geo.h"

/* Abstracting math operations for future non-glibc environments */
#define _SQRT(x)	sqrt(x)
#define _POW(x,y)	pow(x,y)
#define _SIN(x)		sin(x)
#define _COS(x)		cos(x)
#define _ATAN2(x,y)	atan2(x,y)

/* Define constants per WGS84 specification */
const double _ACS_GEO_PI = 3.1415926535898;
const double _ACS_GEO_A	 = 6378137.0;
const double _ACS_GEO_B	 = 6356752.3142;
const double _ACS_GEO_F	 = 0.0033528107;
			   /* (_ACS_GEO_A - _ACS_GEO_B) / _ACS_GEO_A */
			   /* or 1/298.257223563 */
const double _ACS_GEO_E	 = 0.081819191;
			   /* _SQRT(_ACS_GEO_F * (2.0 - _ACS_GEO_F)) */
			   /* or _SQRT((A^2 +B^2) / A^2) */
const double _ACS_GEO_E2 = 0.082099438;
			   /* or _SQRT((A^2 +B^2) / A^2) */

/* Convert degress to radians */
double _acs_geo_degtorad(double x) {
  return x * _ACS_GEO_PI / 180;
}

/* Convert radians to degress */
double _acs_geo_radtodeg(double x) {
  return x * 180 / _ACS_GEO_PI;
}

/* Determine the normal distance to the ellipse surface 
   for a latitude (in radians) */
double _acs_geo_natlat(double lat) {
  return _ACS_GEO_A / _SQRT(1 - _POW(_ACS_GEO_E * _SIN(lat), 2));
}

/*
* Step 1: LLA <-> ECEF-r
*/

/* Convert LLA (ECEF-g in radians/meters) to ECEF-r (rectangular) */
void _acs_geo_llatoecef(double ecef[], double lla[]) {
  /* TODO: Confirm that N is based on point latitude vice origin latitude */
  double N = _acs_geo_natlat(lla[0]);
  
  ecef[0] = (lla[2] + N) * _COS(lla[0]) * _COS(lla[1]);
  ecef[1] = (lla[2] + N) * _COS(lla[0]) * _SIN(lla[1]);
  ecef[2] = (lla[2] + N * (1 - _ACS_GEO_E * _ACS_GEO_E)) * _SIN(lla[0]);
  
  return;
}

/* Convert ECEF-r (rectangular) to LLA (ECEF-g in radians/meters) */
void _acs_geo_eceftolla(double lla[], double ecef[]) {
  double p = _SQRT(ecef[0] * ecef[0] + ecef[1] * ecef[1]);
  double theta = _ATAN2(ecef[2] * _ACS_GEO_A, p * _ACS_GEO_B);
  
  lla[0] = _ATAN2(ecef[2] + (_POW(_ACS_GEO_E2, 2) *
  			     _ACS_GEO_B * _POW(_SIN(theta), 3)),
		  p - (_POW(_ACS_GEO_E, 2) *
  			    _ACS_GEO_A * _POW(_COS(theta), 3)));
  lla[1] = _ATAN2(ecef[1], ecef[0]);
  lla[2] = (p / _COS(lla[0])) - _acs_geo_natlat(lla[0]);
  
  return;
}

/*
* Step 2: ECEF-r <-> LTP
*/

/* Initialize an LTP structure for a given origin */
void acs_geo_init(struct acs_geo *geo, 
		  double lat, double lon, double hae) {
  /* Convert Lat/Lon to radians */
  lat = _acs_geo_degtorad(lat);
  lon = _acs_geo_degtorad(lon);
  
  /* Store origin in lat/lon/alt in radians/meters */
  geo->lla[0] = lat;
  geo->lla[1] = lon;
  geo->lla[2] = hae;
  
  /* Compute geo origin in ECEF-r */
  _acs_geo_llatoecef(geo->ecef, geo->lla);
  
  /* Compute rotational matrix for ECEF-r <-> geo */
  geo->R[0][0] = -1.0 * _SIN(lon);
  geo->R[0][1] = _COS(lon);
  geo->R[0][2] = 0;
  geo->R[1][0] = -1.0 * _SIN(lat) * _COS(lon);
  geo->R[1][1] = -1.0 * _SIN(lat) * _SIN(lon);
  geo->R[1][2] = _COS(lat);
  geo->R[2][0] = _COS(lat) * _COS(lon);
  geo->R[2][1] = _COS(lat) * _SIN(lon);
  geo->R[2][2] = _SIN(lat);
  
  return;
}

/* Multiply a 3x3 matrix (or its transpose if 1) by a 3x1 vector */
void _acs_geo_mult33mtrx31vec(double v[3], double M[3][3], double v0[3],
			      unsigned char transpose) {
  int i, k, *r, *c;
  
  /* Use pointers to map rows and columns to get R[i][k] or R[k][i] */
  if (transpose) {
    r = &k;
    c = &i;
  } else {
    r = &i;
    c = &k;
  }
  
  /* Initialize the output vector */
  v[0] = v[1] = v[2] = 0;
  
  /* Perform the multiplication */
  for (i=0; i<3; i++) {
    for (k=0; k<3; k++) {
      v[i] += M[*r][*c] * v0[k];
    }
  }
}



/* Convert coords in ECEF-r (rectangular) to an LTP */
void _acs_geo_eceftoltp(struct acs_geo *geo,
			double ltp[3], double ecef[3]) {
  /* Shift ECEF coords to origin of LTP */
  double shifted[3];
  shifted[0] = ecef[0] - geo->ecef[0];
  shifted[1] = ecef[1] - geo->ecef[1];
  shifted[2] = ecef[2] - geo->ecef[2];
  
  /* Rotate axes into LTP with east-north-up coords */
  _acs_geo_mult33mtrx31vec(ltp, geo->R, shifted, 0);
  
  return;
}

/* Convert coords in an LTP to ECEF-r (rectangular) */
void _acs_geo_ltptoecef(struct acs_geo *geo,
			double ecef[3], double ltp[3]) {
  /* Rotate axes back into alignment with ECEF-r */
  double shifted[3];
  _acs_geo_mult33mtrx31vec(shifted, geo->R, ltp, 1);
  
  /* Shift coords back to ECEF origin */
  ecef[0] = shifted[0] + geo->ecef[0];
  ecef[1] = shifted[1] + geo->ecef[1];
  ecef[2] = shifted[2] + geo->ecef[2];
  
  return;
}

/*
* Combine Step 1 and Step 2
*/

/* Convert LLA (lat, lon, alt) to LTP (east, north, up) 
   for some initialized LTP *geo */
void acs_geo_llatoltp(struct acs_geo *geo,
		      double *east, double *north, double *up,
		      double lat, double lon, double hae) {
  /* Convert Lat/Lon to radians */
  double lla[3];
  lla[0] = _acs_geo_degtorad(lat);
  lla[1] = _acs_geo_degtorad(lon);
  lla[2] = hae;
  
  /* First convert LLA (ECEF-g) into ECEF-r */
  double ecef[3];
  _acs_geo_llatoecef(ecef, lla);
  
  /* Then convert ECEF-r into LTP */
  double ltp[3];
  _acs_geo_eceftoltp(geo, ltp, ecef);
  
  *east = ltp[0];
  *north = ltp[1];
  *up = ltp[2];
  
  return;
}

/* Convert LTP to LLA for some initialized LTP *geo */
void acs_geo_ltptolla(struct acs_geo *geo,
		      double *lat, double *lon, double *hae,
		      double east, double north, double up) {
  double ltp[3];
  ltp[0] = east;
  ltp[1] = north;
  ltp[2] = up;
  
  /* First convert LTP into ECEF-r */
  double ecef[3];
  _acs_geo_ltptoecef(geo, ecef, ltp);
  
  /* Then convert ECEF-r into LLA (ECEF-g) */
  double lla[3];
  _acs_geo_eceftolla(lla, ecef);
  
  /* Convert Lat/Lon to degrees */
  *lat = _acs_geo_radtodeg(lla[0]);
  *lon = _acs_geo_radtodeg(lla[1]);
  *hae = lla[2];
  
  return;
}

/* Test code - compile with -lm if using GLIBC math library */
#include <stdio.h>
int main(int argc, char **argv) {
  double lat0 = 35.123456, lon0 = -120.987654, hae0 = 500.00;
  double  lat = 35.123789,  lon = -120.987321,  hae = 500.00;
  
  printf("Lat %f Lon %f Hae %f\n", lat, lon, hae);
  
  struct acs_geo geo;
  acs_geo_init(&geo, lat0, lon0, hae0);
  
  double east, north, up;
  acs_geo_llatoltp(&geo, &east, &north, &up, lat, lon, hae);
  printf("East %f North %f Up %f\n", east, north, up);
  
  acs_geo_ltptolla(&geo, &lat, &lon, &hae, east, north, up);
  printf("Lat %f Lon %f Hae %f\n", lat, lon, hae);
  
  return 0;
}











