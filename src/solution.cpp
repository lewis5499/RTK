/*------------------------------------------------------------------------------
* solution.cpp : RTK software common functions
*
*          Copyright (C) 2024 by H.Z. Liu, All rights reserved.
*
* options : none
*
* references :  [1]"RTK_Structs.h"
*
* version : $Revision: 1.8 $ $Date: 2024 / 04 / 09  20:30:32 $
*
* history : 2023 / 11 / 03   1.0
*           2023 / 11 / 05   1.1
*           2023 / 11 / 06   1.2
*           2023 / 11 / 24   1.3
*           2023 / 12 / 11   1.4
*           2023 / 12 / 14   1.5
*           2023 / 12 / 25   1.6
*           2024 / 03 / 18   1.7
*           2024 / 04 / 09   1.8 new
*-----------------------------------------------------------------------------*/
#include "matrix.h"
#include "rtklib.h"
#include "trace.h"
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>

using namespace matrix;

lsq_t lsqParam;
ekf_t ekfParam;
extern char *rest;
extern int lambda(int n, int m, const double *a, const double *Q, double *F, double *s);

/* Convert Common Time to MJD --------------------------------------------------
* convert Common Time to MJD
* args   : const COMMONTIME * CT    I   to be converted
*          MJDTIME * MJDT           O   target TIME
* return : bool				        O   success or failure
*-----------------------------------------------------------------------------*/
bool CT2MJD(const utime_t *CT, mjdtime_t *MJDT){
	int y, m = 0;
	//First determine whether CT is initialized/available.
	//If the month is January or February, subtract 1 from the year and add 12 to the month.
	if (CT->Month == 0 || CT->Day == 0){
		return false;
	} else if (CT->Month <= 2){
		y = CT->Year - 1;
		m = CT->Month + 12;
	} else {
		y = CT->Year;
		m = CT->Month;
	}
	
	//30.6001:Avoid loss of computer accuracy
	MJDT->Days = (int)(365.25*y) + (int)(30.6001*(m + 1)) + int(CT->Day) - 679019;//1720981.5-2400000.5
	MJDT->FracDay = (CT->Hour + CT->Minute / 60.0 + CT->Second / 3600.0) / 24.0;

	return true;
}

/* Convert MJD to Common Time --------------------------------------------------
* convert MJD to Common Time
* args   : const MJDTIME * MJDT     I   to be converted
*          COMMONTIME * CT          O   target TIME
* return : bool				        O   success or failure
*-----------------------------------------------------------------------------*/
bool MJD2CT(const mjdtime_t *MJDT, utime_t *CT){
	if (MJDT->Days == 0 && abs(MJDT->FracDay) < EPSILON){
		return false;
	}            //2400000.5+0.5, a is the integer part
	int a = (int)(MJDT->Days + MJDT->FracDay + 2400001);
	int b = a + 1537;
	int c = (int)((b - 122.1) / 365.25);
	int d = (int)(365.25*c);
	int e = (int)((b - d) / 30.6001);

	CT->Day = b - d - (int)(30.6001*e);
	CT->Month = e - 1 - 12 * (int)(e / 14);
	CT->Year = c - 4715 - (int)((7 + CT->Month) / 10);

	CT->Hour = (int)(MJDT->FracDay * 24);
	CT->Minute = (int)((MJDT->FracDay * 24 - CT->Hour) * 60);
	CT->Second = ((MJDT->FracDay * 24 - CT->Hour) * 60 - CT->Minute) * 60.0;
	return true;
}

/* Convert GPST to MJD ---------------------------------------------------------
* convert GPS Time to MJD
* args   : const GPSTIME * GT       I   to be converted
*          MJDTIME * MJDT           O   target TIME
* return : bool				        O   success or failure
*-----------------------------------------------------------------------------*/
bool GPST2MJD(const gtime_t *GT, mjdtime_t *MJDT){
	if (GT->Week == 0 && abs(GT->SecOfWeek) < EPSILON) {
		return false;
	}
	MJDT->Days = 44244 + GT->Week * 7 + (int)(GT->SecOfWeek / 86400.0);
	MJDT->FracDay = 44244 + GT->Week * 7 + (GT->SecOfWeek / 86400.0) - MJDT->Days;
	return true;
}

/* Convert MJD to GPST ---------------------------------------------------------
* convert MJD to GPS Time
* args   : const MJDTIME * MJDT     I   to be converted
*          GPSTIME * GT             O   target TIME
* return : bool				        O   success or failure
*-----------------------------------------------------------------------------*/
bool MJD2GPST(const mjdtime_t *MJDT, gtime_t *GT){
	if (MJDT->Days == 0 && abs(MJDT->FracDay) < EPSILON){
		return false;
	}
	GT->Week = (int)((MJDT->Days + MJDT->FracDay - 44244) / 7.0);
	GT->SecOfWeek = (MJDT->Days + MJDT->FracDay - 44244 - GT->Week * 7) * 86400;
	return true;
}

/* Convert Common Time to GPST -------------------------------------------------
* convert Common Time to GPS Time 
* args   : const COMMONTIME * CT    I   to be converted
*          GPSTIME * GT             O   target TIME
* return : bool				        O   success or failure
*-----------------------------------------------------------------------------*/
bool CT2GPST(const utime_t *CT, gtime_t *GT){
	if (CT->Month == 0 || CT->Day == 0){
		return false;
	}
	mjdtime_t *MJD = new mjdtime_t;
	CT2MJD(CT, MJD);
	MJD2GPST(MJD, GT);
	delete MJD;
	return true;
}

/* Convert GPST to Common Time -------------------------------------------------
* convert GPS Time to Common Time
* args   : const GPSTIME * GT    I   to be converted
*          COMMONTIME * CT       O   target TIME
* return : bool				     O   success or failure
*-----------------------------------------------------------------------------*/
bool GPST2CT(const gtime_t *GT, utime_t *CT){
	if (GT->Week == 0 && abs(GT->SecOfWeek) < EPSILON){
		return false;
	}
	mjdtime_t *MJD = new mjdtime_t;
	GPST2MJD(GT, MJD);
	MJD2CT(MJD, CT);
	delete MJD;
	return true;
}

/* get difference of two GPST time ---------------------------------------------
* convert degree to degree-minute-second
* args   : const GPSTIME * GT2    I   GPST: GT2 subtract
*          const GPSTIME * GT1    I   GPST: GT1 subtracted
* return : double				  O   DiffGPST(seconds):GT2-GT1
*-----------------------------------------------------------------------------*/
double diffTime(const gtime_t *GT2, const gtime_t *GT1){
	//RETURN=SECONDS(GT2-GT1)
	return (GT2->Week - GT1->Week) * 7 * 86400.0 + (GT2->SecOfWeek - GT1->SecOfWeek);
}

/* get Current Time String ------------------------------------------------------
* get Current Time
* args   : NONE
* return : string				  O   the Current Time formatted as"%Y-%m-%d-%H-%M-%S"
*-----------------------------------------------------------------------------*/
string getCurrentTimeString()
{
	auto now = std::chrono::system_clock::now();
	std::time_t currentTime = std::chrono::system_clock::to_time_t(now);
	std::tm* localTime = std::localtime(&currentTime);
	std::stringstream ss;
	ss << std::put_time(localTime, "%Y-%m-%d-%H-%M-%S");
	return ss.str();
}

/* convertToStdString Function --------------------------------------------------
 * Converts a C-style string (const char*) to a C++ std::string.
 * args
 *     input: const char* - The C-style string to be converted to std::string.
 * return
 *     std::string - The converted string in std::string format.
 * Notes
 *     If the input pointer is nullptr, the function returns an empty std::string.
 *-----------------------------------------------------------------------------*/
string convertToStdString(const char* input) {
	if (input == nullptr) {
		return std::string(); 
	} else {
		return std::string(input);
	}
}

/* convert degree to deg-min-sec -----------------------------------------------
* convert degree to degree-minute-second
* args   : const double deg       I   degree
*          double *dms            O   degree-minute-second {deg,min,sec}
*          int    ndec            I   number of decimals of second
* return : none
*-----------------------------------------------------------------------------*/
void deg2dms(const double deg, double *dms, int ndec){
	double sign = deg < 0.0 ? -1.0 : 1.0, a = fabs(deg);
	double unit = pow(0.1, ndec);
	dms[0] = floor(a); a = (a - dms[0])*60.0;
	dms[1] = floor(a); a = (a - dms[1])*60.0;
	dms[2] = floor(a / unit + 0.5)*unit;
	if (dms[2] >= 60.0){
		dms[2] = 0.0;
		dms[1] += 1.0;
		if (dms[1] >= 60.0){
			dms[1] = 0.0;
			dms[0] += 1.0;
		}
	}
	dms[0] *= sign;
}

/* convert deg-min-sec to degree -----------------------------------------------
* convert degree-minute-second to degree
* args   : const double *dms      I   degree-minute-second {deg,min,sec}
* return : double                 O   degree
*-----------------------------------------------------------------------------*/
double dms2deg(const double *dms){
	double sign = dms[0] < 0.0 ? -1.0 : 1.0;
	return sign * (fabs(dms[0]) + dms[1] / 60.0 + dms[2] / 3600.0);
}

/* geodetic position to local coordinate transfromation matrix -----------------
* compute geodetic position to local coordinate transfromation matrix
* args   : const double *blhPos     I   geodetic position {lat,lon} (rad)
*          double *e2nMat           O   ecef to local coord transformation matrix (3x3)
* return : none
* notes  : matirix stored by row-major order
*-----------------------------------------------------------------------------*/
void blh2enuMat(const double *refblhPos, double *e2nMat){
	double sinb = sin(refblhPos[0]), cosb = cos(refblhPos[0]), sinl = sin(refblhPos[1]), cosl = cos(refblhPos[1]);

	e2nMat[0] = -sinl;        e2nMat[1] = cosl;         e2nMat[2] = 0.0;
	e2nMat[3] = -sinb * cosl; e2nMat[4] = -sinb * sinl; e2nMat[5] = cosb;
	e2nMat[6] = cosb * cosl;  e2nMat[7] = cosb * sinl;  e2nMat[8] = sinb;
}

/* transform ecef to geodetic postion ------------------------------------------
* transform ecef position to geodetic position
* args   : const double *xyzPos    I   ecef position {x,y,z} (m)
*          double *blhPos          O   geodetic position {lat,lon,h} (rad,m)
* return : none
* notes  : WGS84, ellipsoidal height
*-----------------------------------------------------------------------------*/
void xyz2blh(const double *xyzPos, double *blhPos){
	double e2 = F_WGS84 * (2.0 - F_WGS84), z, zk, n = R_WGS84, sinb;
	double r2 = xyzPos[0] * xyzPos[0] + xyzPos[1] * xyzPos[1];
	for (z = xyzPos[2], zk = 0.0; fabs(z - zk) >= 1E-5;){
		zk = z;
		sinb = z / sqrt(r2 + z * z);
		n = R_WGS84 / sqrt(1.0 - e2 * sinb*sinb);
		z = xyzPos[2] + n * e2*sinb;
	}// latitude is -90 to 90 degrees, we use atan();
	 // longitude is -180 to 180 degrees, we use atan2();
	 // additionally, it is necessary to distinguish the North and South Poles here.
	blhPos[0] = r2 > 1E-12 ? atan(z / sqrt(r2)) : (xyzPos[2] > 0.0 ? PI / 2.0 : -PI / 2.0);
	blhPos[1] = r2 > 1E-12 ? atan2(xyzPos[1], xyzPos[0]) : 0.0;
	blhPos[2] = sqrt(r2 + z * z) - n;
}

/* transform geodetic to ecef position -----------------------------------------
* transform geodetic position to ecef position
* args   : const double *blhPos    I   geodetic position {lat,lon,h} (rad,m)
*          double *xyzPos          O   ecef position {x,y,z} (m)
* return : none
* notes  : WGS84, ellipsoidal height
*-----------------------------------------------------------------------------*/
void blh2xyz(const double *blhPos, double *xyzPos){
	double sinb = sin(blhPos[0]), cosb = cos(blhPos[0]), sinl = sin(blhPos[1]), cosl = cos(blhPos[1]);
	double e2 = F_WGS84 * (2.0 - F_WGS84), n = R_WGS84 / sqrt(1.0 - e2 * sinb*sinb);

	xyzPos[0] = (n + blhPos[2])*cosb*cosl;
	xyzPos[1] = (n + blhPos[2])*cosb*sinl;
	xyzPos[2] = (n*(1.0 - e2) + blhPos[2])*sinb;
}

/* transfrom ecef position to local coordinate ---------------------------------
* compute ecef position to local coordinate transfromation matrix
* args   : const double *xyzPos        I   ecef position {x,y,z} (m)
*          const double *refxyzPos     I   reference ecef position {x0,y0,z0} (m)
*          double       *enuPos        O   local coordinate position {e,n,u} (m)
* return : none
*-----------------------------------------------------------------------------*/
void xyz2enu(const double *xyzPos, const double *refxyzPos, double *enuPos){
	double dxyzPos[3], refblhPos[3], e2nmat[9];

	xyz2blh(refxyzPos, refblhPos);
	matSub(xyzPos, refxyzPos, dxyzPos, 3, 1);
	blh2enuMat(refblhPos, e2nmat);
	matMul(e2nmat, 3, 3, dxyzPos, 3, 1, enuPos);
}

/* ecef to local coordinate transfromation matrix ------------------------------
 * compute ecef to local coordinate transfromation matrix
 * args   : double *pos      I   geodetic position {lat,lon} (rad)
 *          double *E        O   ecef to local coord transformation matrix (3x3)
 * return : none
 * notes  : matirix stored by column-major order (fortran convention)
 *-----------------------------------------------------------------------------*/
void xyz2enu(const double *pos, double *E)
{
	double sinp = sin(pos[0]), cosp = cos(pos[0]), sinl = sin(pos[1]), cosl = cos(pos[1]);

	E[0] = -sinl;			E[3] = cosl;			E[6] = 0.0;
	E[1] = -sinp * cosl;	E[4] = -sinp * sinl;	E[7] = cosp;
	E[2] = cosp * cosl;		E[5] = cosp * sinl;		E[8] = sinp;
}

/* transform local vector to ecef coordinate -----------------------------------
 * transform local tangental coordinate vector to ecef
 * args   : double *pos      I   geodetic position {lat,lon} (rad)
 *          double *e        I   vector in local tangental coordinate {e,n,u}
 *          double *r        O   vector in ecef coordinate {x,y,z}
 * return : none
 *-----------------------------------------------------------------------------*/
void enu2xyz(const double *pos, const double *e, double *r) {
	double E[9];

	xyz2enu(pos, E);
	matmul("TN", 3, 1, 3, 1.0, E, e, 0.0, r);
}

/* transform covariance to local tangental coordinate --------------------------
* transform ecef covariance to local tangental coordinate
* args   : double *pos      I   geodetic position {lat,lon} (rad)
*          double *P        I   covariance in ecef coordinate
*          double *Q        O   covariance in local tangental coordinate
* return : none
*-----------------------------------------------------------------------------*/
void covenu(const double *pos, const double *P, double *Q)
{
	double E[9], EP[9];

	xyz2enu(pos, E);
	matmul("NN", 3, 3, 3, 1.0, E, P, 0.0, EP);
	matmul("NT", 3, 3, 3, 1.0, EP, E, 0.0, Q);
}

/* transform local enu coordinate covariance to xyz-ecef -----------------------
* transform local enu covariance to xyz-ecef coordinate
* args   : double *pos      I   geodetic position {lat,lon} (rad)
*          double *Q        I   covariance in local enu coordinate
*          double *P        O   covariance in xyz-ecef coordinate
* return : none
*-----------------------------------------------------------------------------*/
void covxyz(const double *pos, const double *Q, double *P)
{
	double E[9], EQ[9];

	xyz2enu(pos, E);
	matmul("TN", 3, 3, 3, 1.0, E, Q, 0.0, EQ);
	matmul("NN", 3, 3, 3, 1.0, EQ, E, 0.0, P);
}

/* Compute Satellite Elevation angle and Azimuth angle -------------------------
* compute satellite elevation angle and ezimuth angle based on sat Pos and base Pos
* args   : const double *Xr   I   base Pos: ecef position {x,y,z} (m)
*          const double *Xs   I   sat Pos:  ecef position {x,y,z} (m)
*		   double *Elev       O   satellite elevation angle relative to the base station
*          double *Azim       O   satellite azimuth angle relative to the base station
* return : none
* notes  : WGS84, ellipsoidal height
*-----------------------------------------------------------------------------*/
void compSatElAz(const double *Xr, const double *Xs, double *Elev, double *Azim){
	double XrblhPos[3] = { 0,0,0 }, dxyzMat[3] = { 0,0,0 }, denuMat[3] = { 0,0,0 };
	double Xr2enuMat[9] = { 0,0,0,0,0,0,0,0,0 };
	xyz2blh(Xr, XrblhPos);
	blh2enuMat(XrblhPos, Xr2enuMat);
	matSub(Xs, Xr, dxyzMat, 3, 1);
	matMul(Xr2enuMat, 3, 3, dxyzMat, 3, 1, denuMat);

	double r2 = denuMat[1] * denuMat[1] + denuMat[0] * denuMat[0];
	*Elev = r2 > 1E-12 ? atan(denuMat[2] / sqrt(r2)) : (denuMat[2] > 0.0 ?  PI / 2.0 : -PI / 2.0);
	*Azim = atan2(denuMat[0], denuMat[1]);
}

/* Compute the Position error in the local coordinate -------------------------
* compute the position error in the local coordinate: enu, based on the known precise base station Pos
* args   : const double *X0      I   precise base Pos: ecef position {x,y,z} (m)
*          const double *Xr      I   computed base Pos: ecef position {x,y,z} (m)
*		   double *denuErrorMat  O   the pos Error of base station: enu position {dE, dN, dU} (m)
* return : none
* notes  : WGS84, ellipsoidal height
*-----------------------------------------------------------------------------*/
void denuPos(const double *X0, const double *Xr, double *denu){
	double X0blhPos[3] = { 0 }, X02enuMat[9] = { 0 }, dxyzMat[3] = { 0 };

	xyz2blh(X0, X0blhPos);
	blh2enuMat(X0blhPos, X02enuMat);
	matSub(Xr, X0, dxyzMat, 3, 1);
	matMul(X02enuMat, 3, 3, dxyzMat, 3, 1, denu);
}

/* geometric distance ----------------------------------------------------------
* compute geometric distance and receiver-to-satellite unit vector
* args   : double *rs       I   satellilte position (ecef at transmission) (m)
*          double *rr       I   receiver position (ecef at reception) (m)
* return : geometric distance (m) (0>:error/no satellite position)
* notes  : distance includes NO sagnac effect correction
*-----------------------------------------------------------------------------*/
double geodist(const double *rs, const double *rr) {
	double e[3] = { 0.0,0.0,0.0 };

	if (norm(rs, 3) < R_WGS84) return -1.0;
	for (int i = 0; i < 3; i++) { e[i] = rs[i] - rr[i]; }

	return norm(e, 3);
}

/* get sat Clock bias correction */
double svClkCorr(const double dt, const double *params) {
	return params[0] + params[1]*dt + params[2]*SQR(dt);
}

/* get sat Clock drift correction */
double svClkdCorr(const double dt, const double *params) {
	return params[1] + 2 * params[2] * dt;
}

/* get earth rotation correction matrix */
double *earthRotCorr(const double *vec3, const double time, const double omga)
{
	double rot[9];
	rot[0] =  cos(omga*time); rot[1] = sin(omga*time); rot[2] = 0;
	rot[3] = -sin(omga*time); rot[4] = cos(omga*time); rot[5] = 0;
	rot[6] = 0;				  rot[7] = 0;			   rot[8] = 1;
	double *ivec = new double[3];
	matMul(rot, 3, 3, vec3, 3, 1, ivec);
	return ivec;
}

/* get Hopfield model correction */
double Hopfield(const double H, const double Elev)
{
	if (H<-10000.0||H>8848.86) return 0.0;
	double E = Elev * R2D;
	double RH = cfg.hopfdRH0 * exp(-0.0006396*(H - cfg.hopfdH0));
	double p = cfg.hopfdp0 * pow(1 - 0.0000226*(H - cfg.hopfdH0), 5.225);
	double T = cfg.hopfdT0 - 0.0065*(H - cfg.hopfdH0);
	double e = RH * exp(-37.2465 + 0.213166*T - 0.000256908*SQR(T));
	double hw = 11000.0;
	double hd = 40136.0 + 148.72*(cfg.hopfdT0 - 273.16);
	double Kw = 155.2*(1e-7) * 4810.0 / (SQR(T))*e*(hw - H);
	double Kd = 155.2*(1e-7)*p / T * (hd - H);

	return Kd / sin(sqrt(SQR(E) + 6.25)*D2R) + Kw / sin(sqrt(SQR(E) + 2.25)*D2R);
}

/* smoother --------------------------------------------------------------------
* combine forward and backward filters by fixed-interval smoother as follows:
*
*   xs=Qs*(Qf^-1*xf+Qb^-1*xb), Qs=(Qf^-1+Qb^-1)^-1)
*
* args   : double *xf       I   forward solutions (n x 1)
* args   : double *Qf       I   forward solutions covariance matrix (n x n)
*          double *xb       I   backward solutions (n x 1)
*          double *Qb       I   backward solutions covariance matrix (n x n)
*          int    n         I   number of solutions
*          double *xs       O   smoothed solutions (n x 1)
*          double *Qs       O   smoothed solutions covariance matrix (n x n)
* return : status (0:ok,0>:error)
* notes  : see reference [4] 5.2
*          matirix stored by column-major order (fortran convention)
*-----------------------------------------------------------------------------*/
int smoother(const double *xf, const double *Qf, const double *xb, const double *Qb, int n, double *xs, double *Qs)
{
	double *invQf = mat(n, n), *invQb = mat(n, n), *xx = mat(n, 1);
	int i, info = -1;

	matcpy(invQf, Qf, n, n);
	matcpy(invQb, Qb, n, n);
	if (!matinv(invQf, n) && !matinv(invQb, n)) {
		for (i = 0; i < n*n; i++) Qs[i] = invQf[i] + invQb[i];
		if (!(info = matinv(Qs, n))) {
			matmul("NN", n, 1, n, 1.0, invQf, xf, 0.0, xx);
			matmul("NN", n, 1, n, 1.0, invQb, xb, 1.0, xx);
			matmul("NN", n, 1, n, 1.0, Qs, xx, 0.0, xs);
		}
	}
	free(invQf); free(invQb); free(xx);
	return info;
}

int filterspp(const double *Xk_1, const double *Pk_1, double *Xkk_1, double *Pkk_1)
{
	const double dt = 1.0;
	double T[36], T_[36], Q[36], tmp1[36], tmp2[36];

	/* state transfer matrix */
	eye(T, 6);
	T[23] = dt;
	T[29] = dt;
	matTran(T, 6, 6, T_);
	
	/* state noise matrix */
	zero(Q, 6, 6);
	Q[0]  = SQR(cfg.RcvHoriBias)*dt;
	Q[7]  = SQR(cfg.RcvHoriBias)*dt;
	Q[14] = SQR(cfg.RcvHoriBias)*dt;
	Q[21] = SQR(cfg.RcvClkBias)*dt + 1.0 / 3 * SQR(cfg.RcvClkDft)*dt*dt*dt;
	Q[22] = 1.0 / 3 * SQR(cfg.RcvClkDft)*dt*dt*dt;
	Q[23] = 1.0 / 2 * SQR(cfg.RcvClkDft)*dt*dt;
	Q[27] = 1.0 / 3 * SQR(cfg.RcvClkDft)*dt*dt*dt;
	Q[28] = SQR(cfg.RcvClkDft)*dt + 1.0 / 3 * SQR(cfg.RcvClkDft)*dt*dt*dt;
	Q[29] = 1.0 / 2 * SQR(cfg.RcvClkDft)*dt*dt;
	Q[33] = 1.0 / 2 * SQR(cfg.RcvClkDft)*dt*dt;
	Q[34] = 1.0 / 2 * SQR(cfg.RcvClkDft)*dt*dt;
	Q[35] = SQR(cfg.RcvClkDft)*dt;

	/* ahead prediction */
	matMul(T, 6, 6, Xk_1, 6, 1, Xkk_1);
	matMul(T, 6, 6, Pk_1, 6, 6, tmp1);
	matMul(tmp1, 6, 6, T_, 6, 6, tmp2);
	matAdd(tmp2, Q, Pkk_1, 6, 6);

	return 0;
}

int filterspp(const double *Pkk_1, const double *Xkk_1, double *Xk, double *Pk, epoch_t *obs, const int *locs, int num)
{
	const int MAXN = 40; int idx, i;
	double R[MAXN*MAXN], _R[MAXN*MAXN], H[MAXN*6], V[MAXN], F, d0, dX[3], K[6*MAXN], H_[6*MAXN], KV[6], I[36], KH[36], T[36], K_[MAXN*6];
	double  tmp1[6*MAXN], tmp2[MAXN*6], tmp3[MAXN*MAXN], tmp4[36], tmp5[36], tmp6[6*MAXN], tmp7[36];
	
	/* measurement noise matrix */
	eye(R, num); 
	matMul_s(SQR(cfg.CodeNoise),R, R, num*num);
	matcpy(_R, R, num*num);

	/* design matrix & predicted residual */
	for (i = 0; i < num; i++) {
		idx = locs[i];
		matSub(Xkk_1, obs->SatPVT[idx].SatPos, dX, 3, 1);
		d0 = norm(dX, 3);

		H[i * 6 + 0] = dX[0] / d0;
		H[i * 6 + 1] = dX[1] / d0;
		H[i * 6 + 2] = dX[2] / d0;
		if (obs->SatPVT[idx].Sys == GPS) {
			H[i * 6 + 3] = 1.0;
			H[i * 6 + 4] = 0.0;
			F = d0 + Xkk_1[3] - CLIGHT * obs->SatPVT[idx].SatClkOft + obs->SatPVT[idx].TropCorr;
			V[i] = getPIF(obs, idx, GPS) - F;
		} else {
			H[i * 6 + 3] = 0.0;
			H[i * 6 + 4] = 1.0;
			F = d0 + Xkk_1[4] - CLIGHT * obs->SatPVT[idx].SatClkOft + obs->SatPVT[idx].TropCorr;
			V[i] = getPIF(obs, idx, BDS) - F;
		}
		H[i * 6 + 5] = 0.0;
	}

	/* gain matrix */
	matTran(H, num, 6, H_);
	matMul(Pkk_1, 6, 6, H_, 6, num, tmp1);
	matMul(H, num, 6, Pkk_1, 6, 6, tmp2);
	matMul(tmp2, num, 6, H_, 6, num, tmp3);
	matAdd(tmp3, R, R, num, num);
	matinv(R, num);
	matMul(tmp1, 6, num, R, num, num, K);

	/* Xk */
	matMul(K, 6, num, V, num, 1, KV);
	matAdd(Xkk_1, KV, Xk, 6, 1);

	/* Dxk */
	eye(I, 6);
	matMul(K, 6, num, H, num, 6, KH);
	matSub(I, KH, I, 6, 6);
	matTran(I, 6, 6, T);
	matMul(I, 6, 6, Pkk_1, 6, 6, tmp4);
	matMul(tmp4, 6, 6, T, 6, 6, tmp5);
	matTran(K, 6, num, K_);
	matMul(K, 6, num, _R, num, num, tmp6);
	matMul(tmp6, 6, num, K_, num, 6, tmp7);
	matAdd(tmp5, tmp7, Pk, 6, 6);

	return 0;
}

/* Get the PIF of 'obs.SatObs[idx]' according to sys (TGD Correction included)------------------ */
double getPIF(epoch_t *obs, const int idx, const int sys)
{
	if (sys == BDS) {
		double p1, p3;
		p1 = obs->SatObs[idx].P[0]; p3 = obs->SatObs[idx].P[1];
		return (p3 - p1 * SQR(FC13R) + CLIGHT * SQR(FC13R)*obs->SatPVT[idx].Tgd1) / (1 - SQR(FC13R));
	} else if (sys == GPS) {
		return obs->SatPVT[idx].PIF;
	} else {
		return 0.0;
	}
}

/* Tropospheric delay correction --------------------------------------------------------------- */
void tropCorr(epoch_t *obs, const double *Xr, double *BLHr, const int *loc, const int num)
{
	int i; double El, Az, TropCorr;

	xyz2blh(Xr, BLHr);
	for (i = 0; i < num; i++) {
		compSatElAz(Xr, obs->SatPVT[loc[i]].SatPos, &El, &Az);
		TropCorr = Hopfield(BLHr[2], El);
		obs->SatPVT[loc[i]].Azimuth = Az;
		obs->SatPVT[loc[i]].Elevation = El;
		obs->SatPVT[loc[i]].TropCorr = TropCorr;
		if (El < cfg.elevaThreshold*D2R) obs->SatPVT[loc[i]].Valid = false;
	}
}

int rtkSyncf(FILE *fRov, FILE *fBas, raw_t *raw)
{	//-2:EOF, 0:NOT SYNCED, 1:SYNCED		
	int idxBas = 1, idxRov = 1;	//0:OBS, -2:EOF
	double dt = 10.0;

	idxRov = inputOEM7f(fRov, raw->RovObs, raw->ephgps, raw->ephbds);

	if (!idxRov) {	
		dt = diffTime(&raw->RovObs.Time, &raw->BasObs.Time);
		if (fabs(dt) < 1.0) return 1; //try to sync in <1s

		while (dt > 0 && idxBas != -2 && fabs(dt) >= 1.0) { //try to sync in 1s
			//Rover is after Base & Base is NOT EOF & NOT SYNCED
			idxBas = inputOEM7f(fBas, raw->BasObs, raw->ephgps, raw->ephbds);
			dt = diffTime(&raw->RovObs.Time, &raw->BasObs.Time);
		}
		
		if (fabs(dt) <= THRES_DT_FILE) {
			return 1;
		} else if (dt < 0 && idxBas != -2) {
			return 0;
		} else {
			return -2; //Base EOF & NOT SYNCED
		}
		
	} else {
		return -2; //Rover EOF->END
	}
}

int rtkSync(callback_t *p)
{
	//-2:EOF, 0:NOT SYNCED, 1:SYNCED		
	int idxBas = 1, idxRov = 1;	//0:OBS, -2:EOF
	double dt = 10.0;

	idxRov = inputOEM7(p->BuffRov, p->lenDRov, &p->raw.RovObs, p->raw.ephgps, p->raw.ephbds);// 0=obs, -2=rovBuff EOF

	if (!idxRov) {
		dt = diffTime(&p->raw.RovObs.Time, &p->raw.BasObs.Time);
		if (fabs(dt) < 1.0) return 1; //try to sync in <1s

		while (dt > 0 && idxBas != -2 && fabs(dt) >= 1.0) {	//try to sync in 1s
			//Rover is after Base & Base is NOT EOF & NOT SYNCED
			idxBas = inputOEM7(p->BuffBas, p->lenDBas, &p->raw.BasObs, p->raw.ephgps, p->raw.ephbds);
			dt = diffTime(&p->raw.RovObs.Time, &p->raw.BasObs.Time);
		}

		if (fabs(dt) <= THRES_DT_REALTIME) { //whether it's in 1s or not, if in 30s, RTK sync successfully!
			return 1;
		} else if (dt < 0 && idxBas != -2) {
			return 0;
		} else {
			return -2; //Base buff EOF & NOT SYNCED
		}

	} else {
		return -2; //Rover buff EOF->END
	}
}

/* SOCKET compute one OBSEPOCH in buff---------------------------------------------------------- */
void sppMain(epoch_t *obs, gpseph_t *ephgps, bdseph_t *ephbds, pos_t *pos) {
	satposs(ephgps, ephbds, obs, false);
	inOrder(obs);
	markOutlier(obs);
	sppspv(obs, ephgps, ephbds, pos);
}

void rtkMain(raw_t *raw, pos_t *base, pos_t *rover) {
	SDObs(raw);
	markCycleSlip(&raw->SDObs);
	setRefSat(&raw->BasObs, &raw->RovObs, &raw->SDObs, &raw->DDObs);
	rtkFloat(raw, base, rover);
	rtkFixed(raw, base, rover);
}

void SDObs(raw_t *raw)
{
	raw->SDObs.reset();

	int i, j, prn, n=0; double dp1, dp2, dl1, dl2; sys_t sys = UNKS;
	sdobs_t *p = &raw->SDObs; sdsat_t *pp= raw->SDObs.SDSatObs;

	for (i = 0; i < raw->RovObs.SatNum; i++) {
		prn = raw->RovObs.SatObs[i].Prn;
		sys = raw->RovObs.SatObs[i].Sys;
		for (j = 0; j < raw->BasObs.SatNum; j++) {
			if ((raw->BasObs.SatObs[j].Prn == prn) && (raw->BasObs.SatObs[j].Sys == sys)) {
				dp1 = getSDObs(raw->RovObs.SatObs[i], raw->BasObs.SatObs[j], 0, 0);
				dp2 = getSDObs(raw->RovObs.SatObs[i], raw->BasObs.SatObs[j], 1, 0);
				dl1 = getSDObs(raw->RovObs.SatObs[i], raw->BasObs.SatObs[j], 0, 1);
				dl2 = getSDObs(raw->RovObs.SatObs[i], raw->BasObs.SatObs[j], 1, 1);
				pp[n].sys = sys;							pp[n].prn = prn;
				pp[n].nRov = i;								pp[n].nBas = j;
				pp[n].dP[0] = dp1;							pp[n].dP[1] = dp2;
				pp[n].dL[0] = dl1;							pp[n].dL[1] = dl2;
				pp[n].ValidP[0] = (fabs(dp1) > EPSILON);	pp[n].ValidP[1] = (fabs(dp2) > EPSILON);
				pp[n].ValidL[0] = (fabs(dl1) > EPSILON);	pp[n].ValidL[1] = (fabs(dl2) > EPSILON);
				pp[n].halfc [0] = (!raw->RovObs.SatObs[i].parity[0])||(!raw->BasObs.SatObs[j].parity[0]);
				pp[n].halfc [1] = (!raw->RovObs.SatObs[i].parity[1])||(!raw->BasObs.SatObs[j].parity[1]);
				n++;				
				break;
			}
			else;
		}
	}
	p->Time = raw->RovObs.Time;	//shallow copy SDobs' GPST
	p->SatNum = n;				//nums of SD obs
}

void setRefSat(const epoch_t *Base, const epoch_t *Rover, sdobs_t *SDObs, ddobs_t *DDObs)
{
	DDObs->reset();

	int idx, nfreq, prn, sys=-1, tmpPrn[2][2]={-1,-1,-1,-1}, tmpIdx[2][2] = {-1,-1,-1,-1}; //[sys][freq]
	double cn0MaxFreq[2][2]={0.0,0.0,0.0,0.0}, elMaxFreq[2][2]={0.0,0.0,0.0,0.0},elMaxSys[2]={0.0,0.0}, elThres=40*D2R;
	sdsat_t *psat=SDObs->SDSatObs;

	for (idx = 0; idx < SDObs->SatNum; idx++) {
		if (!(psat[idx].ValidP[0]&&psat[idx].ValidP[1]&&psat[idx].ValidL[0]&&psat[idx].ValidL[1])
			|| !Rover->SatPVT[psat[idx].nRov].Valid || !Base->SatPVT[psat[idx].nBas].Valid
			|| (psat[idx].halfc[0]) || (psat[idx].halfc[1])) {
			continue; // 1.healthy pseudorange/carrier phase; 
		}			  // 2.healthy ephmeris, valid sat-pos
					  // 3.halfc does not exist
		if (cfg.fullfreqs && !psat[idx].flfreqchk()) continue;

		sys = (int)psat[idx].sys-1;
		prn = psat[idx].prn;

		if (Rover->SatPVT[psat[idx].nRov].Elevation > elMaxSys[sys]) {  // select ref-sats for each sys, 2 in total
			elMaxSys[sys] = Rover->SatPVT[psat[idx].nRov].Elevation;	// for each sys, select one with highest el
			DDObs->RefPrn[sys] = prn;
			DDObs->RefIdx[sys] = idx;
		}

		for (nfreq = 0; nfreq < 2; nfreq++)	{	// select ref-sats for each sys-freq, 4 in total
			if (Rover->SatPVT[psat[idx].nRov].Elevation > elThres) {					 // if el > 40 deg
				if (Rover->SatObs[psat[idx].nRov].cn0[nfreq] > cn0MaxFreq[sys][nfreq]) { // then select one with highest C/N0
					cn0MaxFreq[sys][nfreq] = Rover->SatObs[psat[idx].nRov].cn0[nfreq];
					tmpPrn[sys][nfreq] = prn;
					tmpIdx[sys][nfreq] = idx;
					if (Rover->SatPVT[psat[idx].nRov].Elevation > elMaxFreq[sys][nfreq]) {
						elMaxFreq[sys][nfreq] = Rover->SatPVT[psat[idx].nRov].Elevation;
					}
				}
			} else if (Rover->SatPVT[psat[idx].nRov].Elevation > elMaxFreq[sys][nfreq]) { 
				elMaxFreq[sys][nfreq] = Rover->SatPVT[psat[idx].nRov].Elevation; // if el <= 40 deg, select one with highest Elevation angle
				tmpPrn[sys][nfreq] = prn;
				tmpIdx[sys][nfreq] = idx;	
			} else;
		}
	}

	if (sys==-1) {
		return;	// For the initial epoch, no reference satellites are available
	} else {
		if (!cfg.fullfreqs) {
			for (int s = 0; s < 2; s++) {
				for (int f = 0; f < 2; f++) {
					DDObs->FreqRefPrn[s][f] = tmpPrn[s][f];
					DDObs->FreqRefIdx[s][f] = tmpIdx[s][f];
				}
			}
		} else { // only use full-freqs
			DDObs->FreqRefPrn[0][0] = DDObs->FreqRefPrn[0][1] = DDObs->RefPrn[0];
			DDObs->FreqRefPrn[1][0] = DDObs->FreqRefPrn[1][1] = DDObs->RefPrn[1];
			DDObs->FreqRefIdx[0][0] = DDObs->FreqRefIdx[0][1] = DDObs->RefIdx[0];
			DDObs->FreqRefIdx[1][0] = DDObs->FreqRefIdx[1][1] = DDObs->RefIdx[1];
		}
	}
	tracef("%d %f, PRN: GPS=%d,\tBDS=%d\n", Rover->Time.Week, Rover->Time.SecOfWeek, DDObs->RefPrn[0], DDObs->RefPrn[1]);
}

int rtkFloat(raw_t *raw, pos_t *posBas, pos_t *posRov)
{	
	////////////////////////////
	//// part1: preparation ////
	////////////////////////////
	int i, j, k, idx, sys, num, numGps, numBds, *SDIdx, nFreq[2][2], vfreq, badFreq[50][2], lsqDo=1;
	double ambDD[MAXCHANNUM][2], wMax, wtmp, wmean, wstd, *BasDist, *RovDist;
	sdsat_t *sd_raw = raw->SDObs.SDSatObs;	//[sys*nsat][freq]

	memcpy(&ekfParam.Time, &raw->RovObs.Time, sizeof(gtime_t));

	SDIdx = new int[raw->SDObs.SatNum];
	BasDist = new double[raw->SDObs.SatNum];
	RovDist = new double[raw->SDObs.SatNum]; // Indexed with 'invalid' and 'Valid', corresponding to 'SDSATOBS'

RECOMPUTE:
	sys = 0, num = 0, numGps = 0, numBds = 0, vfreq = 4, wmean = 1.0, wtmp = wMax = wstd = 0.0;
	memset(nFreq, 0, 4 * sizeof(int));
	memset(badFreq, 0, 100 * sizeof(int));
	memset(SDIdx, -1, raw->SDObs.SatNum * sizeof(int));
	memset(ekfParam.Index, -1, sizeof(int) * 2 * MAXFREQNUM);

	// (1) Compute baseline-satellite distances, initialize ambiguities, and count the number of SD obs of
	for (i = 0, idx = 0; i < raw->SDObs.SatNum; i++) 
	{	// full-freq(valid) pseudorange+phase data for each system.
		ambDD[i][0] = ambDD[i][1] = BasDist[i] = RovDist[i] = -1.0;

		if ((!raw->BasObs.SatPVT[sd_raw[i].nBas].Valid) || (!raw->RovObs.SatPVT[sd_raw[i].nRov].Valid) ||
			(!(raw->SDObs.checkFreq(i,0)||raw->SDObs.checkFreq(i,1))) ) {
			continue;
		}

		if (cfg.fullfreqs && !sd_raw[i].flfreqchk()) continue;

		BasDist[i] = geodist(raw->BasObs.SatPVT[sd_raw[i].nBas].SatPos, raw->BasObs.BestPos.XYZ);
		SDIdx[idx++] = i; // After the loop ends, idx holds the number of valid single-difference satellites
		sys = (int)sd_raw[i].sys - 1;
		
		for (j = 0; j < 2; j++) {
			if (raw->SDObs.checkFreq(i, j)) {
				ambDD[i][j] = -(sd_raw[i].dP[j] - sd_raw[raw->DDObs.FreqRefIdx[sys][j]].dP[j]) + (sd_raw[i].dL[j] - sd_raw[raw->DDObs.FreqRefIdx[sys][j]].dL[j]);
				ambDD[i][j] /= (j == 0) ? ((sys == 0) ? WL1_GPS : WL1_BDS) : ((sys == 0) ? WL2_GPS : WL3_BDS);
				nFreq[sys][j]++;
				num++;
			}
		}
	}

	// (2) adjust DD num (excluding ref sats)
	for (sys = 0; sys < 2; sys++) {
		for (j = 0; j < 2; j++)	{	// donot use it when there is only 0/1 sat for a single system frequency, or cannnot select a ref-sat for this sys-frequency
			if (nFreq[sys][j]<2 || raw->DDObs.FreqRefPrn[sys][j]==-1) {
				num -= nFreq[sys][j];
				nFreq[sys][j] = 0;
				vfreq--;
			}
		}
	}
	num -= vfreq;
	nFreq[0][0] -= (nFreq[0][0] == 0) ? 0 : 1;
	nFreq[0][1] -= (nFreq[0][1] == 0) ? 0 : 1;
	nFreq[1][0] -= (nFreq[1][0] == 0) ? 0 : 1;
	nFreq[1][1] -= (nFreq[1][1] == 0) ? 0 : 1;
	numGps = nFreq[0][0] + nFreq[0][1];
	numBds = nFreq[1][0] + nFreq[1][1];

	// (3) check nums 
	if (num>=0&&num<4) {
		tracef("[WARNINGS]: nums of usable sats < 4 AT %d %f\n", raw->RovObs.Time.Week, raw->RovObs.Time.SecOfWeek);
		if(ekfParam.IsInit) memcpy(posRov->Pos, ekfParam.X_, 3*sizeof(double)); // do not use SPP results
		delete[]SDIdx; delete[]BasDist; delete[]RovDist;
		return -1;
	} else if (num<6&&num>=4) {  // -2: insufficient double-difference obs equations
		tracef("[WARNINGS]: nums of usable 4 <= sats < 6 AT %d %f\n", raw->RovObs.Time.Week, raw->RovObs.Time.SecOfWeek);
		if (!cfg.rtkFilter) {
			delete[]SDIdx; delete[]BasDist; delete[]RovDist;
			return -2;
		} else {
			lsqDo = 0;
		}
	} else;

	// (4) renew 'DD' and 'ekfParam' data
	raw->DDObs.nFreq = ekfParam.nFreqs = num;
	raw->DDObs.nFreqSat[0][0] = nFreq[0][0];
	raw->DDObs.nFreqSat[0][1] = nFreq[0][1];
	raw->DDObs.nFreqSat[1][0] = nFreq[1][0];
	raw->DDObs.nFreqSat[1][1] = nFreq[1][1];

	int sdIdx, row, col, colGps, colBds, cnt, sta, end, ndd, iters = 0, *type, bn=0;
	double m[3], *B, *P, *w, *Q, *X;

	B = new double[num*2*(3+num)];	 P = new double[num*2*num*2];
	Q = new double[(3+num)*(3+num)]; w = new double[num*2*1];
	X = new double[3+num];			 type = new int[num]; // type=0->freq1, 1->freq2

	// Initialize: X
	matcpy(X, posRov->Pos, 3);
	for (i = 0, cnt = 0; cnt < num; i++) {
		sdIdx = SDIdx[i];
		sys = (int)sd_raw[sdIdx].sys - 1;

		for (j = 0; j < 2; j++) {
			if ((nFreq[sys][j]==0)||(!raw->SDObs.checkFreq(sdIdx,j))||(raw->DDObs.FreqRefIdx[sys][j]==sdIdx))
				continue; // Skip reference satellites; Skip a Freq Band whose sats cannot form a DD obs.

			// save valid SD sat-freqs for ekf (excluding reference sat-freqs)
			ekfParam.Index[cnt][0] = sdIdx; // if all are -1: check() == false
			ekfParam.Index[cnt][1] = j;

			type[cnt] = j;
			X[3+cnt] = ambDD[sdIdx][j];
			cnt++;
		}
	}
	// Initialize: lsqParam (copy addresses of {B, P, w, Q, m})
	lsqParam.setParam(B, P, w, Q, m, num * 2, 3 + num, X);

	////////////////////
	//// part2: lsq ////
	////////////////////
	// LSQ main: renew rovpos、Amb(NDD) by iterations. 
	// [Attention]: be careful to construct mat{B,W,P} when numgps/numbds==0,1
	if (lsqDo) {
	do {
		/* Initialization */
		sdIdx = 0, row = 0, col = 3, colGps=3, colBds=3+numGps, sta = 0, end = 0, ndd = 0;
		zero(B, num*2, 3+num); zero(P, num*2, num*2); zero(w, num*2, 1);
		for (i=0; i<raw->SDObs.SatNum; i++) RovDist[i] = geodist(raw->RovObs.SatPVT[sd_raw[i].nRov].SatPos, X);

		/* Construct Matrix B, P, w */
		for (i=0, cnt=0; cnt<num; i++) {

			for (j=0; j<2; j++) {

				sdIdx = SDIdx[i];
				sys = (int)sd_raw[sdIdx].sys - 1;
				if ((nFreq[sys][j]==0)||(!raw->SDObs.checkFreq(sdIdx,j))||(raw->DDObs.FreqRefIdx[sys][j]==sdIdx))
					continue; // Skip reference satellites; Skip a Freq Band whose sats cannot form a DD obs.

				if (sys==0) {
					col = colGps;	ndd = nFreq[0][j];	// ndd: nums of current satellite freq band;
					sta = 0;		end = numGps;		// col: a param that assists the construction of 'B';
				} else {								// sta/end: params that assist the construction of 'P'
					col = colBds;	ndd = nFreq[1][j];
					sta = numGps;	end = num;
				}

				// Pseudorange
				/* Matrix B */
				for (k=0; k<3; k++) { //k=0,1,2 -> SatPos[0],[1],[2], x,y,z
					B[row*(3+num)+k] = (X[k] - raw->RovObs.SatPVT[sd_raw[sdIdx].nRov].SatPos[k]) / RovDist[sdIdx] -
						(X[k] - raw->RovObs.SatPVT[sd_raw[raw->DDObs.FreqRefIdx[sys][j]].nRov].SatPos[k]) / RovDist[raw->DDObs.FreqRefIdx[sys][j]];
				}
				/* Matrix P */
				for (k=sta; k<end; k++)	{

					if (type[cnt]==type[k])
						P[row*num*2+k*2] = -1.0 / (ndd + 1);

					if(k==cnt)
						P[row*num*2+k*2] *= -ndd;
				}
				/* Matrix w */
				w[row] = sd_raw[sdIdx].dP[j] - sd_raw[raw->DDObs.FreqRefIdx[sys][j]].dP[j] - (RovDist[sdIdx] - RovDist[raw->DDObs.FreqRefIdx[sys][j]] - BasDist[sdIdx] + BasDist[raw->DDObs.FreqRefIdx[sys][j]]);
				row++;

				// Carrier Phase
				/* Matrix B */
				for (k=0; k<3; k++) { //k=0,1,2 -> SatPos[0],[1],[2], x,y,z
					B[row*(3+num)+k] = (X[k] - raw->RovObs.SatPVT[sd_raw[sdIdx].nRov].SatPos[k]) / RovDist[sdIdx] -
						(X[k] - raw->RovObs.SatPVT[sd_raw[raw->DDObs.FreqRefIdx[sys][j]].nRov].SatPos[k]) / RovDist[raw->DDObs.FreqRefIdx[sys][j]];
				}
				B[row*(3+num)+(col++)] = (j == 0) ? ((sys == 0) ? WL1_GPS : WL1_BDS) : ((sys == 0) ? WL2_GPS : WL3_BDS);
				/* Matrix P */
				for (k=sta; k<end; k++)	{

					if (type[cnt]==type[k])
						P[row*num*2+k*2+1] = -1000.0 / (ndd + 1);

					if(k==cnt)
						P[row*num*2+k*2+1] *= -ndd;
				}
				/* Matrix w */
				w[row] = sd_raw[sdIdx].dL[j] - sd_raw[raw->DDObs.FreqRefIdx[sys][j]].dL[j] - (RovDist[sdIdx] - RovDist[raw->DDObs.FreqRefIdx[sys][j]] - BasDist[sdIdx] + BasDist[raw->DDObs.FreqRefIdx[sys][j]]);
				w[row] -= (j==0) ? ((sys==0)?WL1_GPS*X[3+cnt]:WL1_BDS*X[3+cnt]) : ((sys==0)?WL2_GPS*X[3+cnt]:WL3_BDS*X[3+cnt]);
				if (sys == 0) colGps++; else colBds++;
				row++;

				/* Detect outlier/cycleslip: check bad freq from 'w' */
				wtmp = MAX(fabs(w[row-1]),fabs(w[row-2]));
				wstd = SQRT(SQR(wstd)*cnt/(cnt + 1) + SQR(wtmp-wmean)/(cnt + 1));
				wmean = (wmean*cnt + wtmp) / (cnt + 1); // >=0

				if(!cfg.fullfreqs) {
					if (iters == 0) {
						if (wtmp >= THRES_RTK_DDW) {
							badFreq[bn][0] = j;
							badFreq[bn][1] = sdIdx;
							bn++;
						} else if (((wtmp/(MIN(1.0,wMax)))>3.0&&wtmp>2.5||(wtmp>MIN(wmean+3*wstd,1.5))) && !sd_raw[sdIdx].flfreqchk()) {
							badFreq[bn][0] = j;
							badFreq[bn][1] = sdIdx;
							bn++;
						} else {
							wMax = MAX(wtmp, wMax);
						}
					} else { // post
						if (wtmp>=0.9&&(num-bn)>8&&!sd_raw[sdIdx].flfreqchk()) { // only single freq
							badFreq[bn][0] = j;
							badFreq[bn][1] = sdIdx;
							bn++;
						} else if (wtmp>=1.8&&(num-bn)>10) { // full freqs
							badFreq[bn][0] = j;
							badFreq[bn][1] = sdIdx;
							bn++;
						}
						else;
					}
				} else { // for full freqs
					/*if (iters == 0) {
						if (wtmp >= THRES_RTK_DDW && (num-bn)>18) {
							badFreq[bn][0] = j;
							badFreq[bn][1] = sdIdx;
							bn++;
						}
					} */
				}

				/* renew params */
				cnt++;
			}
		}

		if (bn > 0) { 
			for (k = 0; k < bn; k++)
				sd_raw[badFreq[k][1]].ValidL[badFreq[k][0]] = sd_raw[badFreq[k][1]].ValidP[badFreq[k][0]]= 0;
			delete[] B;	delete[] P;	delete[] w; delete[] Q; delete[] X; delete[] type;
			goto RECOMPUTE; 
		};

	} while (++iters<MAXITERS && lsqParam.lsq()>MAX_DPOS_RTK);
	}
	
	if (!cfg.rtkFilter || !ekfParam.IsInit) {
		/////////////////////////////////
		//// part3: LSQ save results ////
		/////////////////////////////////
		matcpy(posRov->Pos, X, 3);
		matSub(X, raw->BasObs.BestPos.XYZ, raw->DDObs.dPos, 3, 1);
		raw->DDObs.Bdist = BasDist;
		raw->DDObs.Rdist = RovDist;
		raw->DDObs.SDlist = SDIdx;
		raw->DDObs.type = type;
		raw->DDObs.bFixed = 0; //0=Float
		raw->DDObs.Q = Q;
		raw->DDObs.X = X;
		posRov->IsSuccess = true;
		matcpy(posRov->Pos, X, 3);
		matSub(X, raw->BasObs.BestPos.XYZ, raw->DDObs.dPos, 3, 1);

		ekfParam.p = ekfParam.q = num+3;
		ekfParam.initP();
		memcpy(&ekfParam.preSD, &raw->SDObs, sizeof(sdobs_t));
		memcpy(&ekfParam.preDD, &raw->DDObs, sizeof(ddobs_t));
		memcpy(ekfParam.X_, X, (3+num)*sizeof(double));
		memcpy(ekfParam.Index_, ekfParam.Index, sizeof(int)*2*MAXFREQNUM);
		memcpy(ekfParam.Xlsq, X, 3*sizeof(double));

		delete[] B;	delete[] P;	delete[] w;
		lsqParam.reset();
		return 0;

	} else {
		////////////////////
		//// part4: EKF ////
		////////////////////
		delete[] w; delete[] Q; delete[] type; delete[] SDIdx; delete[] RovDist;
		lsqParam.reset(); // do not delete[] *P & *B & *BasDist	
		raw->DDObs.Bdist = BasDist;

		int p, n, q, m, info; double *_X, *_P, *_Q, *_R, *_H, *_T, *_C, *_v;

		p = 3 + ekfParam.preDD.nFreq;
		q = p;
		n = 3 + raw->DDObs.nFreq;
		m = 2 * raw->DDObs.nFreq;
		_R = P;
		_H = B;
		_X = new double[p];
		_P = new double[p*p];
		_Q = new double[q*q];
		_T = new double[n*p];
		_C = new double[n*q];
		_v = new double[m];
		
		memcpy(_X, ekfParam.X_, sizeof(double)*p);
		memcpy(_P, ekfParam.P_, sizeof(double)*p*p);
		memcpy(ekfParam.Xlsq, X, 3*sizeof(double));
		memcpy(ekfParam.AmbFloat, X+3, sizeof(double)*(n-3));
		if (cfg.kinematic) memcpy(_X, posRov->Pos, sizeof(double)*3);
		
		// main EKF()
		ekfParam.setParam(_X, _P, _Q, _C, _R, _T, _H, _v, p, q, n, m);
		info=ekfParam.ekf(raw);
	
		matcpy(posRov->Pos, ekfParam.X, 3);
		matSub(ekfParam.X, raw->BasObs.BestPos.XYZ, raw->DDObs.dPos, 3, 1);
		raw->DDObs.bFixed = 0;
		if (info!=-1) posRov->IsSuccess = true;
		delete[] X; delete[] BasDist;
		return info;
	}
}

int rtkFixed(raw_t *raw, pos_t *posBas, pos_t *posRov)
{
	if (raw->DDObs.bFixed == -1) return -1;

	if (!cfg.rtkFilter||!ekfParam.IsInit) {
		int i, j, k, sys = 0, sdIdx, row, cnt, sta, end, ndd, num, numGps, numBds;
		double *Qn, *B, *P, *w, *Q, *X, m[3];
		sdsat_t *sd = raw->SDObs.SDSatObs;	ddobs_t *dd = &raw->DDObs;
	
		Qn = new double[SQR(dd->nFreq)];

		if (matBlock(dd->Q, (3+dd->nFreq), (3+dd->nFreq), 3, 3, dd->nFreq, dd->nFreq, Qn)) {
			if (lambda(dd->nFreq, 2, dd->X+3, Qn, dd->FixedAmb, dd->ResAmb)!=0) {
				tracef("[ERROR]: LAMBDA ERROR AT %d %f\n", raw->RovObs.Time.Week, raw->RovObs.Time.SecOfWeek);
				delete[] Qn; delete[] dd->Q; delete[] dd->X; delete[] dd->SDlist; delete[] dd->Bdist; delete[] dd->Rdist; delete[] dd->type;
				return -2;
			} else {
				Q = dd->Q; X = dd->X; num = dd->nFreq; numGps=dd->nFreqSat[0][0]+dd->nFreqSat[0][1]; numBds=dd->nFreqSat[1][0]+dd->nFreqSat[1][1];
				B = new double[num*3]; P = new double[num*num]; w = new double[num];		
			
				lsqParam.setParam(B, P, w, Q, m, num, 3, X);

				/* Initialization */
				sdIdx = 0, row = 0, sta = 0, end = 0, ndd = 0;
				zero(B, num, 3); zero(P, num, num); zero(w, num, 1);
				for (i = 0; i < raw->SDObs.SatNum; i++) dd->Rdist[i] = geodist(raw->RovObs.SatPVT[sd[i].nRov].SatPos, X);

				/* Construct Matrix B, P, w */
				for (i = 0, cnt = 0; cnt < num; i++) {
					for (j = 0; j < 2; j++) {

						sdIdx = dd->SDlist[i];
						sys = (int)sd[sdIdx].sys - 1;
						if ((dd->nFreqSat[sys][j] == 0) || (!raw->SDObs.checkFreq(sdIdx, j)) || (dd->FreqRefIdx[sys][j] == sdIdx))
							continue; // Skip reference satellites; Skip a Freq Band whose sats cannot form a DD obs.

						if (sys == 0) {
							ndd = dd->nFreqSat[0][j];	// ndd: nums of current satellite freq band;
							sta = 0; end = numGps;		// col: a param that assists the construction of 'B';
						} else {							// sta/end: params that assist the construction of 'P'
							ndd = dd->nFreqSat[1][j];
							sta = numGps; end = num;
						}

						// Carrier Phase
						/* Matrix B */
						for (k = 0; k < 3; k++) { //k=0,1,2 -> SatPos[0],[1],[2], x,y,z
							B[row*3 + k] = (X[k] - raw->RovObs.SatPVT[sd[sdIdx].nRov].SatPos[k]) / dd->Rdist[sdIdx] -
								(X[k] - raw->RovObs.SatPVT[sd[dd->FreqRefIdx[sys][j]].nRov].SatPos[k]) / dd->Rdist[dd->FreqRefIdx[sys][j]];
						}
						/* Matrix P */
						for (k = sta; k < end; k++) {

							if (dd->type[cnt] == dd->type[k])
								P[row*num + k] = -1000.0 / (ndd + 1);

							if (k == cnt)
								P[row*num + k] *= -ndd;
						}
						/* Matrix w */
						w[row] = sd[sdIdx].dL[j] - sd[dd->FreqRefIdx[sys][j]].dL[j] -
							(dd->Rdist[sdIdx] - dd->Rdist[dd->FreqRefIdx[sys][j]] - dd->Bdist[sdIdx] + dd->Bdist[dd->FreqRefIdx[sys][j]]);
						w[row] -= (j==0) ? ((sys==0)?WL1_GPS*dd->FixedAmb[cnt]:WL1_BDS*dd->FixedAmb[cnt]) : ((sys==0)?WL2_GPS*dd->FixedAmb[cnt]:WL3_BDS*dd->FixedAmb[cnt]);
						row++;
						/* renew params */
						cnt++;
					}
				}

				lsqParam.lsq();
				
				dd->Ratio = dd->ResAmb[1] / dd->ResAmb[0];
				if (dd->Ratio >= cfg.ratioThreshold) 
				{
					ekfParam.IsInit = true;

					matcpy(posRov->Pos, X, 3);
					matSub(X, raw->BasObs.BestPos.XYZ, dd->dPos, 3, 1);
					dd->FixRMS[0] = (float)lsqParam.mse;
					dd->bFixed = 1; // 1=Fixed
					memcpy(&ekfParam.preSD, &raw->SDObs, sizeof(sdobs_t));
					memcpy(&ekfParam.preDD, &raw->DDObs, sizeof(ddobs_t));
					memcpy(ekfParam.X_, X, ekfParam.n*sizeof(double));
					memcpy(ekfParam.P_, ekfParam.P, ekfParam.n*ekfParam.n*sizeof(double));

				} else {
					tracef("[WARNINGS]: RTK FLOAT AT %d %f, RATIO=%f\n", raw->RovObs.Time.Week, raw->RovObs.Time.SecOfWeek, dd->Ratio);
				}
				delete[] B;	delete[] P;	delete[] w; delete[] Qn; delete[] dd->Q; delete[] dd->X; 
				delete[] dd->type; delete[] dd->SDlist; delete[] dd->Bdist; delete[] dd->Rdist;
				lsqParam.reset();
			}
		} else {
			delete[] Qn; delete[] dd->Q; delete[] dd->X; delete[] dd->SDlist; delete[] dd->Bdist; delete[] dd->Rdist; delete[] dd->type;
			return -1;
		}

	} else { // EKF
		int i; double *Qn; ddobs_t *dd = &raw->DDObs; ekf_t *pp = &ekfParam;

		Qn = new double[SQR(pp->nFreqs)];

		if (matBlock(pp->P, pp->n, pp->n, 3, 3, pp->nFreqs, pp->nFreqs, Qn)) {
			if (lambda(pp->nFreqs, 2, pp->X+3, Qn, dd->FixedAmb, dd->ResAmb) != 0) {
				tracef("[ERROR]: LAMBDA ERROR AT %d %f\n", raw->RovObs.Time.Week, raw->RovObs.Time.SecOfWeek);
				delete[] Qn; delete[] pp->X; delete[] pp->P; delete[] pp->Q; delete[] pp->R; delete[] pp->H; delete[] pp->T; delete[] pp->C; delete[] pp->v;
				return -2;
			} else {
				dd->Ratio = MIN(999.9,dd->ResAmb[1] / dd->ResAmb[0]);
				if (dd->Ratio >= cfg.ratioThreshold) {
					// measurement update
					matResize(pp->H, pp->m*pp->n, (pp->n-3)*pp->n);		zero(pp->H, (pp->n-3), pp->n);
					matResize(pp->R, pp->m*pp->m, (pp->n-3)*(pp->n-3));	zero(pp->R, (pp->n-3), (pp->n-3));
					matResize(pp->v, pp->m, (pp->n-3));					zero(pp->v, (pp->n-3), 1);
				
					for (i = 0; i < (pp->n-3); i++) pp->H[i*pp->n + i + 3] = 1.0;
					for (i = 0; i < (pp->n-3); i++) pp->R[i*(pp->n-3) + i] = 1e-10;
					for (i = 0; i < (pp->n-3); i++) pp->v[i] = dd->FixedAmb[i] - pp->X[i + 3];

					filter(pp->X, pp->P, pp->H, pp->v, pp->R, pp->n, (pp->n-3));

					matcpy(posRov->Pos, pp->X, 3);
					matSub(pp->X, raw->BasObs.BestPos.XYZ, dd->dPos, 3, 1);
					dd->bFixed = 1; // 1=Fixed

					// for the next kalman epoch
					memcpy(pp->X_, pp->X, pp->n*sizeof(double));
					memcpy(pp->P_, pp->P, sizeof(double)*pp->n*pp->n);
					memcpy(pp->Index_, pp->Index, sizeof(int)*2*MAXFREQNUM);
					memcpy(&pp->preSD, &raw->SDObs, sizeof(sdobs_t));
					memcpy(&pp->preDD, &raw->DDObs, sizeof(ddobs_t));

				} else {
					tracef("[WARNINGS]: RTK FLOAT AT %d %f, RATIO=%f\n", raw->RovObs.Time.Week, raw->RovObs.Time.SecOfWeek, dd->Ratio);
					//if (cfg.kinematic) ekfParam.IsInit = false; // filter reStart after FLOAT Epoch! 
					matcpy(posRov->Pos, pp->X, 3);
					matSub(pp->X, raw->BasObs.BestPos.XYZ, dd->dPos, 3, 1);

					//// for the next kalman epoch
					//memcpy(pp->X_, pp->X, pp->n * sizeof(double));
					//memcpy(pp->P_, pp->P, sizeof(double)*pp->n*pp->n);
					//memcpy(pp->Index_, pp->Index, sizeof(int) * 2 * MAXFREQNUM);
					//memcpy(&pp->preSD, &raw->SDObs, sizeof(sdobs_t));
					//memcpy(&pp->preDD, &raw->DDObs, sizeof(ddobs_t));
				}

				delete[] Qn;delete[] pp->Q;delete[] pp->R;delete[] pp->H;delete[] pp->T;delete[] pp->C;delete[] pp->v;delete[] pp->P;delete[] pp->X;
				pp->reset();
			}

		} else { // dd->bFixed==0
			tracef("[WARNINGS]: RTK FLOAT AT %d %f\n", raw->RovObs.Time.Week, raw->RovObs.Time.SecOfWeek);
			delete[] Qn; delete[] pp->Q; delete[] pp->R; delete[] pp->H; delete[] pp->T; delete[] pp->C; delete[] pp->v; delete[] pp->P; delete[] pp->X;
			return -1;	
		}
	}

	return 0;
}

double getSDObs(const satobs_t& sati, const satobs_t& satj, int k, short flag)
{	
	if (k > 1) return 0.0;

	double mi, mj;
	
	if (!flag) {  //flag==0->P, flag==1->L; k: the carrier frequency index(0<=k<=1).
		mi = (sati.validP[k] && sati.validL[k]) ? sati.P[k] : 0.0;
		mj = (satj.validP[k] && satj.validL[k]) ? satj.P[k] : 0.0; // consistency
	} else {
		mi = (sati.validP[k] && sati.validL[k]) ? sati.L[k] : 0.0;
		mj = (satj.validP[k] && satj.validL[k]) ? satj.L[k] : 0.0; // consistency
	}

	return mi == 0.0 || mj == 0.0 ? 0.0 : mi - mj;
}

void checkConsistency(satobs_t *pre, satobs_t *cur, int n_pre, int n_cur, double dt)
{
	int i, j, k; double Cp, Cl, Cd, Dm, wl;

	if (dt >= 2.0 && cfg.kinematic) return;

	for (i = 0; i < MAXSYSNUM; i++) {
		if (cur[i].Sys == UNKS)	continue;

		for (k = 0; k < n_pre; k++) {
			if (pre[k].Sys==cur[i].Sys&&pre[k].Prn==cur[i].Prn) {
				for (j = 0; j < 2; j++) {
					if (!pre[k].validL[j] || !cur[i].validL[j] || !pre[k].validP[j] || !cur[i].validP[j] ||
						fabs(pre[k].D[j]) < EPSILON || fabs(cur[i].D[j]) < EPSILON)	continue;

					wl = (cur[i].Sys == GPS) ? ((j == 0) ? WL1_GPS : WL2_GPS) : ((j == 0) ? WL1_BDS : WL3_BDS);
					Dm = -(cur[i].D[j] + pre[k].D[j]) / 2.0;
					Cd = wl * fabs(cur[i].D[j] - pre[k].D[j]);
					Cp = fabs(cur[i].P[j] - pre[k].P[j] + wl * Dm*dt);
					Cl = fabs((cur[i].L[j] - pre[k].L[j]) + wl * Dm*dt);

					cur[i].validL[j] = cur[i].validP[j] = (Cd<20.0 && Cp<8.0 && Cl<0.5);
				}
			}
		}
	}
}

/* Mark the outlier of obs.SatObs[...] --------------------------------------------------------- */
void markOutlier(epoch_t *obs)
{
	int i; mwgf_t lastEpoch[MAXCHANNUM];

	for (i = 0; i < MAXCHANNUM; i++) {
		MWGFdpcy(&(obs->COBObs[i]), &(lastEpoch[i]));
		obs->COBObs[i].reset();
		if (initMWGF(obs, i) < 0)	obs->SatObs[i].Valid = false;//The dual-frequency pseudorange or phase observation value
	}   // is missing/incomplete, or the carrier phase observation value at a certain frequency point has a cycle slip.

	detectOutlier(obs, lastEpoch);
}

/* Initialize the obs.ComObs[...] -------------------------------------------------------------- */
int initMWGF(epoch_t *obs, const int idx)
{
	int i; double f[2], p[2], l[2];

	obs->COBObs[idx].Prn = obs->SatObs[idx].Prn;
	obs->COBObs[idx].Sys = obs->SatObs[idx].Sys;
	obs->COBObs[idx].n = 1;

	if (obs->COBObs[idx].Sys == GPS) {
		f[0] = FG1_GPS; f[1] = FG2_GPS;
	} else if (obs->COBObs[idx].Sys == BDS) {
		f[0] = FG1_BDS; f[1] = FG3_BDS;
	} else 
		return -1;

	l[0] = obs->SatObs[idx].L[0];
	l[1] = obs->SatObs[idx].L[1];
	p[0] = obs->SatObs[idx].P[0];
	p[1] = obs->SatObs[idx].P[1];

	for (i = 0; i < 2; i++)	{
		if (obs->SatObs[idx].validP[i])	obs->SatObs[idx].validP[i] = (fabs(p[i]) > EPSILON);
		if (obs->SatObs[idx].validL[i])	obs->SatObs[idx].validL[i] = (fabs(l[i]) > EPSILON && !(obs->SatObs[idx].lli[i] == LLI_SLIP));
	}
	
	if (!(obs->SatObs[idx].validP[0] && obs->SatObs[idx].validP[1] && 
		  obs->SatObs[idx].validL[0] && obs->SatObs[idx].validL[1]) ||
		(obs->SatObs[idx].lli[0] == LLI_HALFC) || (obs->SatObs[idx].lli[1] == LLI_HALFC)) 
		return -1;   // include parity==0

	obs->COBObs[idx].PIF = (SQR(f[0])*p[0] - SQR(f[1])*p[1]) / (SQR(f[0]) - SQR(f[1]));
	obs->COBObs[idx].LIF = (SQR(f[0])*l[0] - SQR(f[1])*l[1]) / (SQR(f[0]) - SQR(f[1]));
	obs->COBObs[idx].LMW = ((f[0]*l[0]-f[1]*l[1])/(f[0]-f[1])-(f[0]*p[0]+f[1]*p[1])/(f[0]+f[1]));
	obs->COBObs[idx].LGF = l[0] - l[1];
	obs->COBObs[idx].PGF = p[0] - p[1];
	obs->SatPVT[idx].PIF = obs->COBObs[idx].PIF;
	obs->SatPVT[idx].LIF = obs->COBObs[idx].LIF;

	return 0;
}

/* Detect the outlier of obs.SatPVT[...] using dMW, GF ----------------------------------------- */
void detectOutlier(epoch_t *obs, const mwgf_t *lastEpoch)
{
	bool isValid; int i, j; double dLGF, dLMW;

	for (i = 0; i < obs->SatNum; i++)
	{
		//SatPVT[i]: Unsuccessful solution, ephemeris unavailable, detection skips
		if (!obs->SatPVT[i].Sys) {
			tracef("%d %f: There is no satellite PVT result for the current epoch, and the ephemeris is unavailable,sys=%d,prn=%d\n",
					obs->Time.Week, obs->Time.SecOfWeek, obs->SatObs[i].Sys, obs->SatObs[i].Prn);
			continue;
		}
		if (!obs->SatObs[i].Valid) {//Incomplete dual-frequency observation of the current epoch, detection skips
			tracef("%d %f: The dual-frequency observations of the current epoch are incomplete,sys=%d,prn=%d\n",
					obs->Time.Week, obs->Time.SecOfWeek, obs->SatPVT[i].Sys, obs->SatPVT[i].prn);
			continue;
		}
		bool hasfound = false;
		for (j = 0; j < obs->SatNum; j++)
		{
			//the current epoch: Complete dual-frequency observations of SatObs[i], and has found the corresponding satellite 
			if ((obs->COBObs[i].Prn == lastEpoch[j].Prn) && (obs->COBObs[i].Sys == lastEpoch[j].Sys))
			{
				if (fabs(lastEpoch[j].LMW) < EPSILON)
				{	//Incomplete dual-frequency observations from the previous epoch, complete in the current epoch.
					//Poor observing conditions are believed to render them unusable.
					obs->COBObs[i].n = 1;
					obs->SatObs[i].Valid = false;
					obs->SatObs[i].validL[0] = false;
					obs->SatObs[i].validL[1] = false;
					obs->SatObs[i].validP[0] = false;
					obs->SatObs[i].validP[1] = false;
					tracef("%d %f: The dual-frequency observation values in the previous epoch are incomplete and the current epoch is complete,sys=%d,prn=%d\n",
							obs->Time.Week, obs->Time.SecOfWeek, obs->SatPVT[i].Sys, obs->SatPVT[i].prn);
					hasfound = true; break;//break
				}
				//Dual-frequency observations from both the previous and current epochs are complete
				//Cycle slips detection is conducted
				dLGF = obs->COBObs[i].LGF - lastEpoch[j].LGF;
				dLMW = obs->COBObs[i].LMW - lastEpoch[j].LMW;
				isValid = (dLGF <= THRES_dLGF) && (dLMW <= THRES_dLMW);
				obs->SatObs[i].Valid = isValid;
				if (isValid) { // No outlier
					obs->COBObs[i].n = lastEpoch[j].n + 1;
					obs->COBObs[i].LMW = (obs->COBObs[i].LMW + lastEpoch[j].LMW*lastEpoch[j].n) / (lastEpoch[j].n + 1);
				} else { // Outliers are detected(Cycle Slip)
					obs->COBObs[i].n = 1;
					obs->SatObs[i].validL[0] = false;
					obs->SatObs[i].validL[1] = false;
					obs->SatObs[i].validP[0] = false;
					obs->SatObs[i].validP[1] = false;
					tracef("%d %f: The satellite has a cycle slip in the current epoch,sys=%d,prn=%d\n",
						obs->Time.Week, obs->Time.SecOfWeek, obs->SatPVT[i].Sys, obs->SatPVT[i].prn);
				}
				hasfound = true; break;
			}
		}
		//Complete dual-frequency observations in the current epoch
		//However, the corresponding satellite from the previous epoch could not be identified
		//Initial tracking does not confirm the absence of outliers/cycle slips; marked as unusable.
		if (!hasfound) {
			obs->COBObs[i].n = 1;
			tracef("%d %f: The satellite was observed for the first time in the current epoch,sys=%d,prn=%d\n",
					obs->Time.Week, obs->Time.SecOfWeek, obs->SatPVT[i].Sys, obs->SatPVT[i].prn);
			obs->SatObs[i].Valid = false;
			obs->SatObs[i].validL[0] = false;
			obs->SatObs[i].validL[1] = false;
			obs->SatObs[i].validP[0] = false;
			obs->SatObs[i].validP[1] = false;
		}
	}
}

void markCycleSlip(sdobs_t *SDobs)
{
	mwgf_t lastEpoch[MAXCHANNUM];

	for (int i = 0; i < MAXCHANNUM; i++) {
		MWGFdpcy(&(SDobs->SDCOBObs[i]), &(lastEpoch[i]));
		SDobs->SDCOBObs[i].reset();
		initMWGF(SDobs, i);
	}
	detectCycleSlip(SDobs, lastEpoch);
}

int initMWGF(sdobs_t *SDobs, const int idx)
{
	double f1, f2, p1, p2, l1, l2;
	sdsat_t *pob = &SDobs->SDSatObs[idx];
	mwgf_t  *pcb = &SDobs->SDCOBObs[idx];

	pcb->Prn = pob->prn;
	pcb->Sys = pob->sys;
	pcb->n = 1;

	if (pcb->Sys == GPS) {
		f1 = FG1_GPS; f2 = FG2_GPS;
	} else if (pcb->Sys == BDS) {
		f1 = FG1_BDS; f2 = FG3_BDS;
	} else 
		return -1;

	l1 = pob->dL[0]; l2 = pob->dL[1];
	p1 = pob->dP[0]; p2 = pob->dP[1];

	if (!(pob->ValidP[0]&&pob->ValidP[1]&&pob->ValidL[0]&&pob->ValidL[1])) return 0;

	pcb->LMW = ((f1 * l1 - f2 * l2) / (f1 - f2) - (f1 * p1 + f2 * p2) / (f1 + f2));
	pcb->LGF = l1 - l2;
	pcb->PGF = p1 - p2;

	return 1;
}

void detectCycleSlip(sdobs_t *SDobs, mwgf_t *lastEpoch)
{
	bool isValid; int i; double dLGF, dLMW, stdMW;
	sdsat_t *pio = NULL; mwgf_t *pic = NULL, *pjc = NULL;

	for (i = 0; i < SDobs->SatNum; i++)
	{
		pio = &(SDobs->SDSatObs[i]);
		pic = &(SDobs->SDCOBObs[i]);

		if (!(pio->ValidL[0]&&pio->ValidL[1]&&pio->ValidP[0]&&pio->ValidP[1])) {
			tracef("%d %f: The dual-frequency observations of the current epoch are either incomplete or invalid,sys=%d,prn=%d\n",
				SDobs->Time.Week, SDobs->Time.SecOfWeek, SDobs->SDSatObs[i].sys, SDobs->SDSatObs[i].prn);
			continue;
		}
		bool hasfound = false;
		for (int j = 0; j < SDobs->SatNum; j++)
		{
			pjc = &(lastEpoch[j]);
			//the current epoch: Complete dual-frequency observations of SatObs[i], and has found the corresponding satellite 
			if ((pic->Prn == pjc->Prn) && (pic->Sys == pjc->Sys))
			{
				if (fabs(pjc->LMW) < EPSILON)
				{	//Incomplete dual-frequency observations from the previous epoch, complete in the current epoch.
					//Poor observing conditions are believed to render them unusable.
					pic->n = 1;
					pio->ValidL[0] = false;
					pio->ValidL[1] = false;
					tracef("%d %f: The dual-frequency observation values in the previous epoch are incomplete and the current epoch is complete,sys=%d,prn=%d\n",
							SDobs->Time.Week, SDobs->Time.SecOfWeek, pio->sys, pio->prn);
					hasfound = true; break;
				}

				//Dual-frequency observations from both the previous and current epochs are complete
				//Cycle slips detection is conducted
				dLGF = pic->LGF - pjc->LGF;
				dLMW = pic->LMW - pjc->LMW;
				stdMW = SQRT(SQR(pjc->stdLMW)*pjc->n/(pjc->n+1)+SQR(dLMW)/(pjc->n+1));
				
				if (!cfg.kinematic)
					isValid = (dLGF <= 0.02) && MAX(3*stdMW, 1.0);
				else
					isValid = (dLGF <= 0.02) && (dLMW <= MIN(MAX(3*stdMW,0.15),0.8)); //MAX????? or MIN?????

				if (isValid) { // No outlier
					pic->n = pjc->n + 1;
					pic->LMW = (pic->LMW + pjc->LMW*pjc->n) / (pjc->n + 1);
					pic->stdLMW = stdMW;
				}
				else { // Cycle Slip are detected
					pic->n = 1;
					pio->ValidL[0] = false;
					pio->ValidL[1] = false;
					tracef("%d %f: The satellite has a cycle slip in the current epoch,sys=%d,prn=%d\n",
							SDobs->Time.Week, SDobs->Time.SecOfWeek, pio->sys, pio->prn);
				}
				hasfound = true; break;
			}
		}
		//Complete dual-frequency observations in the current epoch
		//However, the corresponding satellite from the previous epoch could not be identified
		//Initial tracking does not confirm the absence of outliers/cycle slips; marked as unusable.
		if (!hasfound) {
			pic->n = 1;
			tracef("%d %f: The satellite was observed for the first time in the current epoch,sys=%d,prn=%d\n",
					SDobs->Time.Week, SDobs->Time.SecOfWeek, pio->sys, pio->prn);
			pio->ValidL[0] = false;
			pio->ValidL[1] = false;
		}
	}
}

/* deepCopy 'MWGF' type ------------------------------------------------------------------------ */
void MWGFdpcy(const mwgf_t *source, mwgf_t *dst)
{
	dst->Sys = source->Sys;
	dst->Prn = source->Prn;
	dst->n   = source->n;
	dst->PIF = source->PIF;
	dst->LIF = source->LIF;
	dst->PGF = source->PGF;
	dst->LGF = source->LGF;
	dst->LMW = source->LMW;
}

/* make the obsdata/satPVTdata in order/accordance --------------------------------------------- */
void inOrder(epoch_t *obs)
{
	int idx = 0, i;

	satobs_t ordsatobs[MAXSYSNUM];
	satres_t ordsatpvt[MAXSYSNUM];
	
	for (i = 0; i < MAXSYSNUM; i++) {
		if (obs->SatObs[i].Sys) {
			memcpy(&ordsatobs[idx], &(obs->SatObs[i]), sizeof(satobs_t));
			memcpy(&ordsatpvt[idx], &(obs->SatPVT[i]), sizeof(satres_t));
			idx++;
		}
	}//shallow copy
	memcpy(obs->SatObs, ordsatobs, sizeof(satobs_t)*MAXSYSNUM);
	memcpy(obs->SatPVT, ordsatpvt, sizeof(satres_t)*MAXSYSNUM);
}

/* broadcast ephemeris to satellite position and clock bias -------------------------------------
* compute satellite position and clock bias with broadcast ephemeris (gps,bds)
* args   : GPSEPHREC *ephgps     I     GPS broadcast ephemeris
*          BDSEPHREC *ephbds     I     Beidou broadcast ephemeris
*          EPOCHOBS  *obs        I/O   obs data of current epoch
* return : none
* notes  : satellite clock includes relativity correction, and code bias (tgd or bgd)
*----------------------------------------------------------------------------------------------- */
void satposs(gpseph_t *ephgps, bdseph_t *ephbds, epoch_t *obs, const bool isSPP)
{
	int i;	double dt = 0.0; // i=prn-1

	for (i = 0; i < MAXGPSNUM; i++) {
		if (gpsposs(ephgps, obs, i, isSPP, dt) == -1) {
			tracef("GPS sat: prn=%d PVT computed unsuccessfully AT %d %f\n", i + 1, obs->Time.Week, obs->Time.SecOfWeek);
		}
	}
	for (i = 0; i < MAXBDSNUM; i++) {
		if (bdsposs(ephbds, obs, i, isSPP, dt) == -1) {
			tracef("BDS sat: prn=%d PVT computed unsuccessfully AT %d %f\n", i + 1, obs->Time.Week, obs->Time.SecOfWeek);
		}
	}
}

/* compute gps satellite pos and vel, prn_1=prn-1------------------------------------------------------------- */
int gpsposs(gpseph_t *ephgps, epoch_t *obs, const int prn_1, const bool isSPP, const double dt)
{
	int n, idx;
	double tk, n0, M, E, Ek, sinE, cosE, v, u, r, i, O, sin2u, cos2u, x, y, sinO, cosO, sini, cosi, mu, omge, ClkParams[3];

	idx = searchSat(obs, prn_1 + 1, GPS);
	mu = GM_WGS84; omge = Omega_WGS;
	tk = diffTime(&(obs->Time), &(ephgps[prn_1].TOE));
	n0 = sqrt(mu) / pow(ephgps[prn_1].SqrtA, 3);
	ClkParams[0] = ephgps[prn_1].ClkBias;
	ClkParams[1] = ephgps[prn_1].ClkDrift;
	ClkParams[2] = ephgps[prn_1].ClkDriftRate;

	if (ephgps[prn_1].Sys && (!ephgps[prn_1].SVHealth) && (!(fabs(tk) > THRES_tk)) && (idx != -1))
	{	//ephemeris expired/didn't exist, or sat unhealty
		if (!isSPP) {
			obs->SatPVT[idx].SatClkOft = svClkCorr(diffTime(&(obs->Time), &(ephgps[prn_1].TOC)), ClkParams);
			obs->SatPVT[idx].SatClkSft = svClkdCorr(diffTime(&(obs->Time), &(ephgps[prn_1].TOC)), ClkParams);
			tk -= obs->SatPVT[idx].SatClkOft;
		} else {
			tk -= dt;
		}
		M = ephgps[prn_1].M0 + (n0 + ephgps[prn_1].DeltaN)*tk;
		for (n = 0, E = M, Ek = 0.0; fabs(E - Ek) > RTOL_KEPLER&&n < MAX_ITER_KEPLER; n++) {
			Ek = E; E -= (E - ephgps[prn_1].e*sin(E) - M) / (1.0 - ephgps[prn_1].e*cos(E));
		}
		if (n >= MAX_ITER_KEPLER) {
			tracef("[ERROR]: eph2pos: kepler iteration overflow GPS_prn=%d AT %d %f\n", ephgps[prn_1].PRN, obs->Time.Week, obs->Time.SecOfWeek);
			obs->SatPVT[idx].Valid = false;
			return -1;
		}
		sinE = sin(E); cosE = cos(E);

		v = atan2(sqrt(1.0 - SQR(ephgps[prn_1].e))*sinE, cosE - ephgps[prn_1].e);
		u = v + ephgps[prn_1].omega;
		r = SQR(ephgps[prn_1].SqrtA)*(1.0 - ephgps[prn_1].e*cosE);
		i = ephgps[prn_1].i0 + ephgps[prn_1].iDot*tk;
		sin2u = sin(2.0*u); cos2u = cos(2.0*u);
		u += ephgps[prn_1].Cus*sin2u + ephgps[prn_1].Cuc*cos2u;
		r += ephgps[prn_1].Crs*sin2u + ephgps[prn_1].Crc*cos2u;
		i += ephgps[prn_1].Cis*sin2u + ephgps[prn_1].Cic*cos2u;
		x = r * cos(u); y = r * sin(u); cosi = cos(i); sini = sin(i);
		O = ephgps[prn_1].OMEGA + (ephgps[prn_1].OMEGADot - omge)*tk - omge * ephgps[prn_1].TOE.SecOfWeek;
		sinO = sin(O); cosO = cos(O);

		/* compute sat pos */
		double xk[3];
		xk[0] = x * cosO - y * cosi*sinO;
		xk[1] = x * sinO + y * cosi*cosO;
		xk[2] = y * sini;

		/* compute sat vel */
		double E_dot, u_dot, f_dot, r_dot, I_dot, O_dot, xk_dot, yk_dot, dXk[3];
		E_dot = (n0 + ephgps[prn_1].DeltaN) / (1 - ephgps[prn_1].e*cosE);
		f_dot = E_dot * sqrt(1 - SQR(ephgps[prn_1].e)) / (1 - ephgps[prn_1].e*cosE);
		u_dot = f_dot + 2 * f_dot*(ephgps[prn_1].Cus*cos2u - ephgps[prn_1].Cuc*sin2u);
		r_dot = SQR(ephgps[prn_1].SqrtA)*ephgps[prn_1].e*sinE*E_dot + 2 * f_dot*(ephgps[prn_1].Crs*cos2u - ephgps[prn_1].Crc*sin2u);
		I_dot = ephgps[prn_1].iDot + 2 * f_dot*(ephgps[prn_1].Cis*cos2u - ephgps[prn_1].Cic*sin2u);
		O_dot = (ephgps[prn_1].OMEGADot - omge);
		xk_dot = r_dot * cos(u) - r * u_dot*sin(u);
		yk_dot = r_dot * sin(u) + r * u_dot*cos(u);

		dXk[0] = -xk[1] * O_dot - (yk_dot*cosi - xk[2] * I_dot)*sin(O) + xk_dot * cos(O);
		dXk[1] = xk[0] * O_dot + (yk_dot*cosi - xk[2] * I_dot)*cos(O) + xk_dot * sin(O);
		dXk[2] = yk_dot * sini + y * I_dot*cosi;

		obs->SatPVT[idx].Sys = GPS;
		obs->SatPVT[idx].prn = ephgps[prn_1].PRN;
		obs->SatPVT[idx].Tgd1 = ephgps[prn_1].TGD;
		matcpy(obs->SatPVT[idx].SatPos, xk, 3);
		matcpy(obs->SatPVT[idx].SatVel, dXk, 3);
		if (!isSPP) obs->SatPVT[idx].Valid = true;

	} else {
		if (!isSPP) obs->SatPVT[idx].Valid = false;
		return -1;
	}
	return 0;
}

/* compute bds satellite pos and vel, prn_1=prn-1------------------------------------------------------------- */
int bdsposs(bdseph_t *ephbds, epoch_t *obs, const int prn_1, const bool isSPP, const double dt)
{
	double tk, n0, M, E, Ek, sinE, cosE, v, u, r, i, O, sin2u, cos2u, x, y, sinO, cosO, sini, cosi, mu, omge;
	double xg, yg, zg, sino, coso, ClkParams[3];
	int n, idx;

	idx = searchSat(obs, prn_1 + 1, BDS);
	mu = GM_CGCS; omge = Omega_CGCS;
	tk = diffTime(&(obs->Time), &(ephbds[prn_1].TOE));
	n0 = sqrt(mu) / pow(ephbds[prn_1].SqrtA, 3);
	gtime_t BDT(ephbds[prn_1].TOE.Week - GPST_BDT_WEEKS, ephbds[prn_1].TOE.SecOfWeek - GPST_BDT);
	ClkParams[0] = ephbds[prn_1].ClkBias;
	ClkParams[1] = ephbds[prn_1].ClkDrift;
	ClkParams[2] = ephbds[prn_1].ClkDriftRate;

	if (ephbds[prn_1].Sys && (!ephbds[prn_1].SVHealth) && (!(fabs(tk) > THRES_tk)) && (idx != -1))
	{	//ephemeris expired/didn't exist, or sat unhealty
		if (!isSPP) {
			obs->SatPVT[idx].SatClkOft = svClkCorr(diffTime(&(obs->Time), &(ephbds[prn_1].TOC)), ClkParams);
			obs->SatPVT[idx].SatClkSft = svClkdCorr(diffTime(&(obs->Time), &(ephbds[prn_1].TOC)), ClkParams);
			tk -= obs->SatPVT[idx].SatClkOft;
		} else {
			tk -= dt;
		}
		M = ephbds[prn_1].M0 + (n0 + ephbds[prn_1].DeltaN)*tk;
		for (n = 0, E = M, Ek = 0.0; fabs(E - Ek) > RTOL_KEPLER&&n < MAX_ITER_KEPLER; n++) {
			Ek = E; E -= (E - ephbds[prn_1].e*sin(E) - M) / (1.0 - ephbds[prn_1].e*cos(E));
		}
		if (n >= MAX_ITER_KEPLER) {
			tracef("[ERROR]: eph2pos: kepler iteration overflow BDS_prn=%d AT %d %f\n", ephbds[prn_1].PRN, obs->Time.Week, obs->Time.SecOfWeek);
			if (!isSPP) obs->SatPVT[idx].Valid = false;
			return -1;  //iteration failed
		}
		sinE = sin(E); cosE = cos(E);

		v = atan2(sqrt(1.0 - SQR(ephbds[prn_1].e))*sinE, cosE - ephbds[prn_1].e);
		u = v + ephbds[prn_1].omega;
		r = SQR(ephbds[prn_1].SqrtA)*(1.0 - ephbds[prn_1].e*cosE);
		i = ephbds[prn_1].i0 + ephbds[prn_1].iDot*tk;
		sin2u = sin(2.0*u); cos2u = cos(2.0*u);
		u += ephbds[prn_1].Cus*sin2u + ephbds[prn_1].Cuc*cos2u;
		r += ephbds[prn_1].Crs*sin2u + ephbds[prn_1].Crc*cos2u;
		i += ephbds[prn_1].Cis*sin2u + ephbds[prn_1].Cic*cos2u;
		x = r * cos(u); y = r * sin(u); cosi = cos(i); sini = sin(i);

		/* compute sat pos */
		double xk[3], dXk[3];
		if (ephbds[prn_1].PRN <= 5 || ephbds[prn_1].PRN >= 59) {/* beidou geo satellite */
			O = ephbds[prn_1].OMEGA + ephbds[prn_1].OMEGADot*tk - omge * BDT.SecOfWeek;
			sinO = sin(O); cosO = cos(O);
			xg = x * cosO - y * cosi*sinO;
			yg = x * sinO + y * cosi*cosO;
			zg = y * sini;
			sino = sin(omge*tk); coso = cos(omge*tk);
			xk[0] = xg * coso + yg * sino*COS_5 + zg * sino*SIN_5;
			xk[1] = -xg * sino + yg * coso*COS_5 + zg * coso*SIN_5;
			xk[2] = -yg * SIN_5 + zg * COS_5;

			/* compute sat vel */
			double E_dot, u_dot, f_dot, r_dot, I_dot, O_dot, xk_dot, yk_dot;
			E_dot = (n0 + ephbds[prn_1].DeltaN) / (1 - ephbds[prn_1].e*cosE);
			f_dot = E_dot * sqrt(1 - SQR(ephbds[prn_1].e)) / (1 - ephbds[prn_1].e*cosE);
			u_dot = f_dot + 2 * f_dot*(ephbds[prn_1].Cus*cos2u - ephbds[prn_1].Cuc*sin2u);
			r_dot = SQR(ephbds[prn_1].SqrtA)*ephbds[prn_1].e*sinE*E_dot + 2 * f_dot*(ephbds[prn_1].Crs*cos2u - ephbds[prn_1].Crc*sin2u);
			I_dot = ephbds[prn_1].iDot + 2 * f_dot*(ephbds[prn_1].Cis*cos2u - ephbds[prn_1].Cic*sin2u);
			O_dot = (ephbds[prn_1].OMEGADot);
			xk_dot = r_dot * cos(u) - r * u_dot*sin(u);
			yk_dot = r_dot * sin(u) + r * u_dot*cos(u);

			double dXg, dYg, dZg, dXk1, dYk1, dZk1, dXk2, dYk2, dZk2;
			dXg = -yg * O_dot - (yk_dot*cosi - zg * I_dot)*sin(O) + xk_dot * cos(O);
			dYg = xg * O_dot + (yk_dot*cosi - zg * I_dot)*cos(O) + xk_dot * sin(O);
			dZg = yk_dot * sini + y * I_dot*cosi;
			//part1
			dXk1 = dXg * coso + dYg * sino*COS_5 + dZg * sino*SIN_5;
			dYk1 = -dXg * sino + dYg * coso*COS_5 + dZg * coso*SIN_5;
			dZk1 = -dYg * SIN_5 + dZg * COS_5;
			//part2
			dXk2 = omge * (-sin(omge*tk)*xg + yg * COS_5*cos(omge*tk) + zg * SIN_5*cos(omge*tk));
			dYk2 = omge * (-cos(omge*tk)*xg - yg * COS_5*sin(omge*tk) - zg * SIN_5*sin(omge*tk));
			dZk2 = 0.0;
			//add
			dXk[0] = dXk1 + dXk2;	dXk[1] = dYk1 + dYk2;	dXk[2] = dZk1 + dZk2;
		} else {
			O = ephbds[prn_1].OMEGA + (ephbds[prn_1].OMEGADot - omge)*tk - omge * BDT.SecOfWeek;
			sinO = sin(O); cosO = cos(O);
			xk[0] = x * cosO - y * cosi*sinO;
			xk[1] = x * sinO + y * cosi*cosO;
			xk[2] = y * sini;

			/* compute sat vel */
			double E_dot, u_dot, f_dot, r_dot, I_dot, O_dot, xk_dot, yk_dot;
			E_dot = (n0 + ephbds[prn_1].DeltaN) / (1 - ephbds[prn_1].e*cosE);
			f_dot = E_dot * sqrt(1 - SQR(ephbds[prn_1].e)) / (1 - ephbds[prn_1].e*cosE);
			u_dot = f_dot + 2 * f_dot*(ephbds[prn_1].Cus*cos2u - ephbds[prn_1].Cuc*sin2u);
			r_dot = SQR(ephbds[prn_1].SqrtA)*ephbds[prn_1].e*sinE*E_dot + 2 * f_dot*(ephbds[prn_1].Crs*cos2u - ephbds[prn_1].Crc*sin2u);
			I_dot = ephbds[prn_1].iDot + 2 * f_dot*(ephbds[prn_1].Cis*cos2u - ephbds[prn_1].Cic*sin2u);
			O_dot = (ephbds[prn_1].OMEGADot - omge);
			xk_dot = r_dot * cos(u) - r * u_dot*sin(u);
			yk_dot = r_dot * sin(u) + r * u_dot*cos(u);

			dXk[0] = -xk[1] * O_dot - (yk_dot*cosi - xk[2] * I_dot)*sin(O) + xk_dot * cos(O);
			dXk[1] = xk[0] * O_dot + (yk_dot*cosi - xk[2] * I_dot)*cos(O) + xk_dot * sin(O);
			dXk[2] = yk_dot * sini + y * I_dot*cosi;
		}
		obs->SatPVT[idx].Sys = BDS;
		obs->SatPVT[idx].prn = ephbds[prn_1].PRN;
		obs->SatPVT[idx].Tgd1 = ephbds[prn_1].TGD1;
		obs->SatPVT[idx].Tgd2 = ephbds[prn_1].TGD2;
		matcpy(obs->SatPVT[idx].SatPos, xk, 3);
		matcpy(obs->SatPVT[idx].SatVel, dXk, 3);
		if (!isSPP) obs->SatPVT[idx].Valid = true;

	} else {
		if (!isSPP) obs->SatPVT[idx].Valid = false;
		return -1;
	}
	return 0;
}

/* find the idx in obs.Satobs[idx] according to prn and sys ------------------------------------ */
int searchSat(epoch_t *obs, const short prn, const int sys)
{
	for (int i = 0; i < MAXSYSNUM; i++)
		if (obs->SatObs[i].Prn == prn && obs->SatObs[i].Sys == sys)
			return i;

	return -1;
}

/* compute signal transmission time, Sat Clk/dClk, Sat POS/VEL with earth rotation correction -- */
int psatposs(epoch_t *obs, gpseph_t *ephgps, bdseph_t *ephbds, const double *Xr)
{
	int prn, i, sys = UNKS;	double dt, omge;

	for (i = 0; i < obs->SatNum; i++) {

		/* compute the Signal Transmission Time */
		if (!obs->SatPVT[i].Valid || !obs->SatObs[i].Valid) continue;
		prn = obs->SatPVT[i].prn;
		sys = obs->SatPVT[i].Sys;
		dt = geodist(obs->SatPVT[i].SatPos, Xr) / CLIGHT;

		/* reCompute the Sat Clk/dClk */
		switch (sys) {
			case GPS:	pgpsposs(obs, ephgps, prn - 1, dt); break;
			case BDS:	pbdsposs(obs, ephbds, prn - 1, dt); break;
			default: {
				tracef("[WARNINGS]: OEM7 UNKNOWN SYSTEM: sys=%d AT %d %f\n", sys, obs->Time.Week, obs->Time.SecOfWeek);
			}
		}

		/* reCompute the Sat POS/VEL without earth rotation correction */
		if (sys == GPS) {
			gpsposs(ephgps, obs, prn - 1, true, dt);
		}
		else if (sys == BDS) {
			bdsposs(ephbds, obs, prn - 1, true, dt);
		}
		else;

		/* earth rotation correction */
		omge = (sys == GPS) ? Omega_WGS : Omega_CGCS;
		double *pos = earthRotCorr(obs->SatPVT[i].SatPos, dt, omge);
		double *vel = earthRotCorr(obs->SatPVT[i].SatVel, dt, omge);
		matcpy(obs->SatPVT[i].SatPos, pos, 3);
		matcpy(obs->SatPVT[i].SatVel, vel, 3);
		delete[] pos; delete[] vel;
	}
	return 0;
}

/* compute GPS Sat Clk/dClk with signal transmission time -------------------------------------- */
int pgpsposs(epoch_t *obs, gpseph_t *ephgps, const int idx, const double dt)
{
	int n, ord_idx;
	double M, n0, tk, E, Ek, sinE, cosE, mu = GM_WGS84, ClkParams[3], dts, ddts, E_dot;

	tk = diffTime(&(obs->Time), &(ephgps[idx].TOE)) - dt;
	n0 = sqrt(mu) / pow(ephgps[idx].SqrtA, 3);
	M = ephgps[idx].M0 + (n0 + ephgps[idx].DeltaN)*tk;
	for (n = 0, E = M, Ek = 0.0; fabs(E - Ek) > RTOL_KEPLER&&n < MAX_ITER_KEPLER; n++) {
		/* Newton's iteration method */
		Ek = E; E -= (E - ephgps[idx].e*sin(E) - M) / (1.0 - ephgps[idx].e*cos(E));
	}
	if (n >= MAX_ITER_KEPLER) {
		tracef("[ERROR]: eph2pos: kepler iteration overflow GPS_prn=%d AT %d %f\n", ephgps[idx].PRN, obs->Time.Week, obs->Time.SecOfWeek);
		return -1; //iteration failed
	}
	sinE = sin(E); cosE = cos(E);

	ClkParams[0] = ephgps[idx].ClkBias;
	ClkParams[1] = ephgps[idx].ClkDrift;
	ClkParams[2] = ephgps[idx].ClkDriftRate;
	/* sat clock oft and sft! */
	dts = svClkCorr(diffTime(&(obs->Time), &(ephgps[idx].TOC)) - dt, ClkParams);
	ddts = svClkdCorr(diffTime(&(obs->Time), &(ephgps[idx].TOC)) - dt, ClkParams);
	/* relativity correction */
	dts -= 2.0*sqrt(mu*SQR(ephgps[idx].SqrtA))*ephgps[idx].e*sinE / SQR(CLIGHT);
	E_dot = (n0 + ephgps[idx].DeltaN) / (1 - ephgps[idx].e*cosE);
	ddts -= 2.0*sqrt(mu)*ephgps[idx].SqrtA*ephgps[idx].e*cosE*E_dot / SQR(CLIGHT);

	ord_idx = searchSat(obs, idx + 1, GPS);
	obs->SatPVT[ord_idx].SatClkOft = dts;
	obs->SatPVT[ord_idx].SatClkSft = ddts;

	return 0;
}

/* compute BDS Sat Clk/dClk with signal transmission time -------------------------------------- */
int pbdsposs(epoch_t *obs, bdseph_t *ephbds, const int idx, const double dt)
{
	int n, ord_idx;
	double M, n0, tk, E, Ek, sinE, cosE, mu = GM_CGCS, ClkParams[3], dts, ddts, E_dot;

	tk = diffTime(&(obs->Time), &(ephbds[idx].TOE)) - dt;
	n0 = sqrt(mu) / pow(ephbds[idx].SqrtA, 3);
	M = ephbds[idx].M0 + (n0 + ephbds[idx].DeltaN)*tk;
	for (n = 0, E = M, Ek = 0.0; fabs(E - Ek) > RTOL_KEPLER&&n < MAX_ITER_KEPLER; n++) {
		/* Newton's iteration method */
		Ek = E; E -= (E - ephbds[idx].e*sin(E) - M) / (1.0 - ephbds[idx].e*cos(E));
	}
	if (n >= MAX_ITER_KEPLER) {
		tracef("[ERROR]: eph2pos: kepler iteration overflow BDS_prn=%d AT %d %f\n", ephbds[idx].PRN, obs->Time.Week, obs->Time.SecOfWeek);
		return -1; //iteration failed
	}
	sinE = sin(E); cosE = cos(E);

	ClkParams[0] = ephbds[idx].ClkBias;
	ClkParams[1] = ephbds[idx].ClkDrift;
	ClkParams[2] = ephbds[idx].ClkDriftRate;
	/* sat clock oft and sft! */
	dts = svClkCorr(diffTime(&(obs->Time), &(ephbds[idx].TOC)) - dt, ClkParams);
	ddts = svClkdCorr(diffTime(&(obs->Time), &(ephbds[idx].TOC)) - dt, ClkParams);
	/* relativity correction */
	dts -= 2.0*sqrt(mu*SQR(ephbds[idx].SqrtA))*ephbds[idx].e*sinE / SQR(CLIGHT);
	E_dot = (n0 + ephbds[idx].DeltaN) / (1 - ephbds[idx].e*cosE);
	ddts -= 2.0*sqrt(mu)*ephbds[idx].SqrtA*ephbds[idx].e*cosE*E_dot / SQR(CLIGHT);

	ord_idx = searchSat(obs, idx + 1, BDS);
	obs->SatPVT[ord_idx].SatClkOft = dts;
	obs->SatPVT[ord_idx].SatClkSft = ddts;
	return 0;
}

/* spp-spv main function: LS -------------------------------------------------------------------- */
int sppspv(epoch_t *obs, gpseph_t *ephgps, bdseph_t *ephbds, pos_t *pos)
{
	(*pos).reset();
	pos->Time.Week = obs->Time.Week;
	pos->Time.SecOfWeek = obs->Time.SecOfWeek;

	if (!cfg.sppFilter) {
		if (lsqspp(obs, ephgps, ephbds, pos) < 0) return -1;
		if (lsqspv(obs, pos) < 0) return -1;
	} else {
		if (!pos->ISFIRST) {
			if (ekfspp(obs, ephgps, ephbds, pos) < 0) return -1;
			if (lsqspv(obs, pos) < 0) return -1;
		} else { //first epoch
			if (lsqspp(obs, ephgps, ephbds, pos) < 0) return -1;
			if (lsqspv(obs, pos) < 0) return -1;
			eye(pos->P, 6);
			matMul_s(P0_XYZ, pos->P, pos->P, 36);
			pos->ISFIRST = false;
		}
	}
	pos->IsSuccess = true; // SPP

	return 1;
}

/* spp main function --------------------------------------------------------------------------- */
int lsqspp(epoch_t *obs, gpseph_t *ephgps, bdseph_t *ephbds, pos_t *pos)
{
	bool subed = false;	int i, nn = 5, num, numgps, numbds, n = 0, *loc;
	double Pdop = 999.9, BLHr[3], msePos = 999.9, dps = 1, Dxx[25], Qxx[25], v[40], Xr[3], rcvClkOft[2], prePos[3], dPos[3];

	loc = new int[obs->SatNum];
	//Pos[3] of the previous epoch 'pos' has not been cleared, and the first epoch is initialized to '0.0'
	for (i = 0; i < 3; i++) Xr[i] = pos->Pos[i];
	for (i = 0; i < 2; i++) rcvClkOft[i] = pos->RcvClkOft[i];
	for (i = 0; i < 25; i++) { Dxx[i] = 99.9; Qxx[i] = 999.9; }
	for (i = 0; i < 40; i++) v[i] = 0.0;

LOOP:
	while (n++<MAXITERS && dps>MAX_DPOS_SPP) {
		matcpy(prePos, Xr, 3);
		if (searchSats(obs, loc, num, numgps, numbds) < nn + 1) { delete[] loc; return -1; }
		if ((!(numgps > 0 && numbds > 0)) && (!subed)) { nn--; subed = true; }
		if (lsqspp_(obs, loc, Xr, rcvClkOft, Qxx, Dxx, nn, num, msePos, Pdop, v) < 0) { delete[] loc; return -1; }
		tropCorr(obs, Xr, BLHr, loc, num);
		psatposs(obs, ephgps, ephbds, Xr);

		matSub(prePos, Xr, dPos, 3, 1);
		dps = norm(dPos, 3);
	}

	if (MIN(fabs(maxElement(v,num,1)),fabs(minElement(v,num,1)))>THRES_ABS_V_MAX) {
		for (i = 0; i < num; i++)
			if (fabs(v[i]) > THRES_ABS_V_MAX)
				obs->SatObs[loc[i]].Valid = false;
		n = 0;
		dps = 1;
		memset(v, 0, sizeof(double) * 40);
		goto LOOP;
	}

	matcpy(pos->P, Dxx, nn*nn);
	matcpy(pos->Q, Qxx, nn*nn);
	matcpy(pos->RcvClkOft, rcvClkOft, 2);
	matcpy(pos->Pos, Xr, 3);
	pos->nn = nn;
	pos->AllSatNum = num;
	pos->GPSSatNum = numgps;
	pos->BDSSatNum = numbds;
	pos->PDOP = Pdop;
	pos->SigmaPos = msePos;

	delete[] loc;
	return 0;
}

/* spv main function --------------------------------------------------------------------------- */
int lsqspv(epoch_t *obs, pos_t *pos)
{
	int nn = 4, num, *loc;	double mseVel = 999.9, rcvClkSft = 0.0, Vel[3], Xr[3];

	loc = new int[obs->SatNum];
	matcpy(Xr, pos->Pos, 3);

	if (searchSats(obs, loc, num) < nn + 1) { delete[] loc; return -1; }
	if (lsqspv_(obs, loc, num, nn, Vel, Xr, mseVel, rcvClkSft) < 0) { delete[] loc; return -1; }

	pos->RcvClkSft = rcvClkSft;
	pos->SigmaVel = mseVel;
	matcpy(pos->Vel, Vel, 3);

	delete[] loc;
	return 0;
}

int ekfspp(epoch_t *obs, gpseph_t *ephgps, bdseph_t *ephbds, pos_t *pos)
{
	/* PART 1: backup Epoch k-1: Xk_1(rcvpos,rcvclock), Dxk_1 */
	double Xk_1[6] = { pos->Pos[0], pos->Pos[1], pos->Pos[2], pos->RcvClkOft[0], pos->RcvClkOft[1], pos->RcvClkSft };
	double Pk_1[36], v[40];
	matcpy(Pk_1, pos->P, 36);
	memset(v, 0, sizeof(double) * 40);

	/* PART 2: LS for accurate satellite pos, vel, satclock, PDOP */
	bool subed = false;
	int *loc = new int[obs->SatNum];
	int necessNum = 5, num, numgps, numbds, n = 0;
	double Pdop = 999.9, BLHr[3], sigmaPos = 999.9, dpos = 1, Dxx[36], Qxx[36];
	//Pos[3] of the previous epoch 'pos' has not been cleared, and the first epoch is initialized to '0.0'
	double Xr[3] = { pos->Pos[0],pos->Pos[1],pos->Pos[2] };
	double rcvClkoft[2] = { pos->RcvClkOft[0], pos->RcvClkOft[1] };
	for (int i = 0; i < 25; i++) { Dxx[i] = 99.9; Qxx[i] = 999.9; }

	double prePos[3], dPos[3];
LOOP:
	while (n++<MAXITERS && dpos>MAX_DPOS_SPP) {
		matcpy(prePos, Xr, 3);
		if (searchSats(obs, loc, num, numgps, numbds) < necessNum + 1) { delete[] loc; return -1; }
		if ((!(numgps > 0 && numbds > 0)) && (!subed)) { necessNum--; subed = true; }
		if (lsqspp_(obs, loc, Xr, rcvClkoft, Qxx, Dxx, necessNum, num, sigmaPos, Pdop, v) < 0) { delete[] loc; return -1; }
		tropCorr(obs, Xr, BLHr, loc, num);
		psatposs(obs, ephgps, ephbds, Xr);
		matSub(prePos, Xr, dPos, 3, 1);
		dpos = norm(dPos, 3);
	}

	if ((fabs(maxElement(v,num,1))>THRES_ABS_V_MAX)||(fabs(minElement(v, num, 1))>THRES_ABS_V_MAX))	{
		for (int i = 0; i < num; i++)
			if (fabs(v[i]) > THRES_ABS_V_MAX)
				obs->SatObs[loc[i]].Valid = false;
		n = 0;
		dpos = 1;
		memset(v, 0, sizeof(double) * 40);
		goto LOOP;
	}

	/* PART 3: EKF */
	double Xkk_1[6], Pkk_1[36], Xk[6], Pk[36];
	filterspp(Xk_1, Pk_1, Xkk_1, Pkk_1);
	filterspp(Pkk_1, Xkk_1, Xk, Pk, obs, loc, num);

	/* PART 4: save results */
	matcpy(pos->Pos, Xk, 3);
	matcpy(pos->RcvClkOft, Xk + 3, 2);
	matcpy(pos->P, Pk, 36);
	pos->RcvClkSft = Xk[5];
	pos->AllSatNum = num;
	pos->BDSSatNum = numbds;
	pos->GPSSatNum = numgps;
	pos->nn = 6;
	pos->SigmaPos = sqrt(Pk[0] + Pk[7] + Pk[14]);
	pos->PDOP = Pdop;

	delete[] loc;
	return 0;
}

/* find valid locs for spp --------------------------------------------------------------------- */
int searchSats(epoch_t *obs, int *locs, int &num, int &numgps, int &numbds)
{
	num = 0, numgps = 0, numbds = 0;
	for (int i = 0; i < obs->SatNum; i++) { //mark the valid idx
		if (obs->SatPVT[i].Valid && obs->SatObs[i].Valid) {
			locs[num++] = i;	
			if(obs->SatPVT[i].Sys==GPS) numgps++;
			if(obs->SatPVT[i].Sys==BDS) numbds++;
		}
	}
	return num;
}

/* find valid locs for spv --------------------------------------------------------------------- */
int searchSats(epoch_t *obs, int *locs, int &num)
{
	num = 0;
	for (int i = 0; i < obs->SatNum; i++) //mark the valid idx
		if (obs->SatPVT[i].Valid && obs->SatObs[i].Valid && (obs->SatObs[i].D[0])!=0) //only use L1C,B1I
			locs[num++] = i;	

	return num;
}

/* spp, Method: LS ----------------------------------------------------------------------------- */
int lsqspp_(epoch_t *obs, const int *locs, double *Xr, double *rcvClkOft, double *Qxx, double *Dxx,
	int colB, int rowB, double &Mse, double &Pdop, double *v)
{
	int i, j, idx, iters = 0; double dX[3], d0, m[3], *B, *w, *X, *P, *Q;

	B = new double[rowB*colB];
	w = new double[rowB*1];
	X = new double[colB*1];
	P = new double[rowB*rowB];
	Q = new double[colB*colB];

	lsqParam.setParam(B, P, w, Q, m, rowB, colB, X);
	lsqParam.v = new double[rowB];

	zero(X, colB, 1); 
	eye(P, rowB);
	for(i=0;i<3;i++) X[i]=Xr[i];
	for(i=0;i<colB-3;i++) X[i+3]=rcvClkOft[i];

	do {
		for (i=0; i<rowB; i++) {
			idx = locs[i];
			matSub(obs->SatPVT[idx].SatPos, X, dX, 3, 1);
			d0=norm(dX, 3);
			/* Matrix B,w */
			for (j=0; j<3; j++) B[i*colB+j] = (X[j]-obs->SatPVT[idx].SatPos[j])/d0;
			if (obs->SatPVT[idx].Sys == GPS) {
				B[i*colB+3] = 1;
				if (colB == 5) B[i*colB+4] = 0;
				w[i*1] = getPIF(obs, idx, GPS) - (d0+X[3]-CLIGHT*obs->SatPVT[idx].SatClkOft+obs->SatPVT[idx].TropCorr);
			}
			else if (obs->SatPVT[idx].Sys == BDS) {
				B[i*colB+3] = 0;
				B[i*colB+colB-1] = 1;
				w[i*1] = getPIF(obs, idx, BDS) - (d0+X[colB-1]-CLIGHT*obs->SatPVT[idx].SatClkOft+obs->SatPVT[idx].TropCorr);
			}
			else;
		}
	} while (++iters<MAXITERS && lsqParam.lsq()>THRES_LSMAXCORR);

	if (iters>=MAXITERS) { 
		delete[] B; delete[] w; delete[] X;	delete[] P; delete[] Q;
		delete[] lsqParam.v; lsqParam.reset();
		return -1; 
	}

	matcpy(Xr, X, 3);
	matcpy(rcvClkOft, X+3, colB-3);
	matcpy(Qxx, Q, colB*colB);
	matMul_s(SQR(lsqParam.mse), Q, Dxx, colB*colB);
	Mse = lsqParam.mse;
	Pdop = sqrt(SQR(Q[0])+SQR(Q[1*colB+1])+SQR(Q[2*colB+2]));
	matcpy(v, lsqParam.v, rowB);

	delete[] B; delete[] w; delete[] X;	delete[] P; delete[] Q;	delete[] lsqParam.v; 
	lsqParam.reset();
	return 0;
}

/* spv, Method: LS ----------------------------------------------------------------------------- */
int lsqspv_(epoch_t *obs, const int *locs, const int num, const int nn, double *vel, double *Xr,
	double &mseVel, double &rcvClksft)
{
	int i, j, idx; double m[3], Xs[3], dX[3], d, dd, wl, *B, *w, *X, *P, *Q;

	B = new double[num*nn];
	w = new double[num*1];
	X = new double[nn*1];
	P = new double[num*num];
	Q = new double[nn*nn];
	
	zero(X, nn, 1);

	/* Matrix P */
	eye(P, num);
	for (i = 0; i < num; i++) {
		idx = locs[i];
		wl = (obs->SatPVT[idx].Sys == GPS) ? WL1_GPS : WL1_BDS;
		for (j=0;j<3;j++) Xs[j]=obs->SatPVT[idx].SatPos[j];
		matSub(Xs, Xr, dX, 3, 1);
		d = norm(dX, 3);
		dd = 0.0;
		for(j=0;j<3;j++) dd+=(obs->SatPVT[idx].SatPos[j]-Xr[j])*obs->SatPVT[idx].SatVel[j];
		dd /= d;		
		/* Matrix B */
		for(j=0;j<3;j++) B[i*nn+j] = (-obs->SatPVT[idx].SatPos[j]+Xr[j])/d;
		B[i*nn+3] = 1.0;
		/* Matrix w */
		w[i*1] = obs->SatObs[idx].D[0]*wl - (dd-CLIGHT*obs->SatPVT[idx].SatClkSft);
	}

	lsqParam.setParam(B, P, w, Q, m, num, nn, X);
	lsqParam.lsq();

	mseVel = lsqParam.mse;
	for(j=0;j<3;j++) vel[j] = X[j];
	rcvClksft = X[3];

	delete[] B; delete[] w; delete[] X; delete[] P; delete[] Q;
	lsqParam.reset();
	return 0;
}

/* kalman filter ---------------------------------------------------------------
* extended kalman filter state measurement update as follows:
*
*   K=P*H*(H'*P*H+R)^-1, xk=x+K*v, Pk=(I-K*H)*P*(I-K*H)'+K*R*K'
*
* args   : double *x        I/O   states vector (n x 1):Xk_k1
*          double *P        I/O   covariance matrix of states (n x n):Pk_k1
*          double *H        I     design matrix (m x n)
*          double *v        I     innovation (measurement - model) (m x 1)
*          double *R        I     covariance matrix of measurement error (m x m)
*          int    n,m       I     number of states and measurements
* return : status (0:ok,<0:error)
* notes  : matrix stored by row-major order
*-----------------------------------------------------------------------------*/
int filter(double *x, double *P, const double *H, const double *v, const double *R, int n, int m)
{
	double *H_, *PH_, *HPH_, *K, *K_, *KR, *KH, *I, *I_, *IP, *IPI_, *KRK_, *dX;

	H_ = new double[n*m]; PH_ = new double[n*m]; HPH_ = new double[m*m]; K = new double[n*m]; KH = new double[n*n];
	I = new double[n*n]; I_ = new double[n*n]; IPI_ = new double[n*n];	KRK_ = new double[n*n];	dX = new double[n];
	K_ = new double[m*n]; KR = new double[n*m]; IP = new double[n*n];

	if (H_==nullptr||PH_==nullptr||HPH_==nullptr||K==nullptr||K_==nullptr||KR==nullptr||KH==nullptr
		||I==nullptr||I_==nullptr||IP==nullptr||IPI_==nullptr||KRK_==nullptr||dX==nullptr) {
		tracef("[ERROR]: Bad Alloc In Func'filter()' AT %d %f\n", ekfParam.Time.Week, ekfParam.Time.SecOfWeek);
		return -1;
	}
	/* xk */
	matTran(H, m, n, H_);
	matMul(P, n, n, H_, n, m, PH_);
	matMul(H, m, n, PH_, n, m, HPH_);
	matAdd(HPH_, R, HPH_, m, m);
	matinv(HPH_, m);
	matMul(PH_, n, m, HPH_, m, m, K);
	matMul(K, n, m, v, m, 1, dX);
	matAdd(x, dX, x, n, 1);
	/* Pk */
	eye(I, n);
	matMul(K, n, m, R, m, m, KR);
	matTran(K, n, m, K_);
	matMul(KR, n, m, K_, m, n, KRK_);
	matMul(K, n, m, H, m, n, KH);
	matSub(I, KH, I, n, n);
	matTran(I, n, n, I_);
	matMul(I, n, n, P, n, n, IP);
	matMul(IP, n, n, I_, n, n, IPI_); zero(P, n, n);
	matAdd(IPI_, KRK_, P, n, n);

	delete[]H_, delete[]PH_, delete[]HPH_, delete[]K, delete[]K_, delete[]KR;
	delete[]KH, delete[]I, delete[]I_, delete[]IP, delete[]IPI_, delete[]KRK_, delete[]dX;
	return 0;
}

/* kalman filter ---------------------------------------------------------------
* extended kalman filter state time prediction as follows:
*
*   xkk_1=T*xk_1, Pkk_1=T*Pk_1*T'+C*Q*C'
*
* args   : double *x      I/O   states vector: Xk_1(p x 1) -> Xkk_1(n x 1)
*          double *P      I/O   covariance matrix of states: Pk_1(p x p) -> Pk_1(n x n)
*          double *Q      I     process noise (q x q)
*          double *T      I     state transition matrix (n x p)
*          double *C      I     covariance matrix of measurement error (n x q)
*          int    p,q,n   I     number of states when 'time=k-1', dims of process noise, number of states when 'time=k<-k-1'
* return : status (0:ok,<0:error)
* notes  : matrix stored by row-major order
*-----------------------------------------------------------------------------*/
int filter(double *&x, double *&P, const double *Q, const double *T, const double *C, int p, int q, int n)
{
	double *TPT_, *TP, *T_, *CQC_, *CQ, *C_, *_x, *_P;

	TPT_ = new double[n*n]; TP = new double[n*p];	T_ = new double[p*n];	_x = new double[n];
	CQC_ = new double[n*n]; CQ = new double[n*q];	C_ = new double[q*n];	_P = new double[n*n];

	if (TPT_==nullptr||TP==nullptr||T_==nullptr||CQC_==nullptr||CQ==nullptr||C_==nullptr||_x==nullptr||_P==nullptr) {
		tracef("[ERROR]: filter: Memory Bad Alloc AT %d %f\n", ekfParam.Time.Week, ekfParam.Time.SecOfWeek);
		return -1;
	}
	/* xkk_1 */
	matMul(T, n, p, x, p, 1, _x);
	if (matResize(x, p, n) < 0) return -1;
	memcpy(x, _x, sizeof(double)*n);
	/* Pkk_1 */
	matTran(T, n, p, T_);
	matMul(T, n, p, P, p, p, TP);
	matMul(TP, n, p, T_, p, n, TPT_);
	matTran(C, n, q, C_);
	matMul(C, n, q, Q, q, q, CQ);
	matMul(CQ, n, q, C_, q, n, CQC_);
	matAdd(TPT_, CQC_, _P, n, n);
	if (matResize(P, p*p, n*n) < 0) return -1;
	memcpy(P, _P, sizeof(double)*n*n);

	delete[]TPT_; delete[]TP; delete[]T_; delete[]CQC_; delete[]CQ; delete[]C_; delete[]_x; delete[]_P;
	return 0;
}
