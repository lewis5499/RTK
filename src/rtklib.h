/*------------------------------------------------------------------------------
* rtklib.h : RTK software library
*
*          Copyright (C) 2024 by H.Z. Liu, All rights reserved.
*
* options : none
*
* references :  [1]"RTK_Structs.h" [2] rtklib ver.b34
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
*           2024 / 03 / 18	 1.7
*           2024 / 04 / 09	 1.8 new
*-----------------------------------------------------------------------------*/
#ifndef _RTKLIB_H_
#define _RTKLIB_H_

#ifdef WIN_DLL
#define EXPORT __declspec(dllexport) /* for Windows DLL */
#else
#define EXPORT
#endif

#pragma comment(lib,"WS2_32.lib")
#pragma warning(disable:4996)

#include <stdio.h>
#include <math.h>
#include <sys/stat.h>
#include <direct.h>
#include <io.h>
#include <WinSock.h>
#include <winnt.h>
#include <minwindef.h>
#include <iostream>
#include <string>

#ifndef _STDINT
typedef signed char        int8_t;
typedef short              int16_t;
typedef int                int32_t;
typedef long long          int64_t;
typedef unsigned char      uint8_t;
typedef unsigned short     uint16_t;
typedef unsigned int       uint32_t;
typedef unsigned long long uint64_t;
#else
#include <stdint.h>
#endif // !_STDINT

using namespace std;

/* constants ----------------------------------------------------------------------*/
/* mode, path */
#define MODE_FILE		0
#define MODE_SOCKET		1
#define PATH_CFGDIR		"..\\config\\rtkcfg.config"
#define PATH_LOGDIR		"..\\log"

/* Kalman Filter: P0 */
#define P0_XYZ			200.0		 /* m^2 */
#define P0_AMB			400.0		 /* cycle^2 */
#define STD_SPP			6.0			 /* m^2 */

/* OEM basic info */
#define OEM4SYNC1       0xAA	     /* oem7/6/4 message start sync code 1 */
#define OEM4SYNC2       0x44		 /* oem7/6/4 message start sync code 2 */
#define OEM4SYNC3       0x12		 /* oem7/6/4 message start sync code 3 */
#define OEM4SYNCLEN     3
#define OEM4HLEN        28			 /* oem7/6/4 message header length (bytes) */
#define CRC32LEN		4
#define POLYCRC32       0xEDB88320u  /* CRC32 polynomial */

/* message IDs */
#define ID_RANGECMP			 140     /* oem7/6/4 range compressed */
#define ID_RANGE			  43     /* oem7/6/4 range measurement */
#define ID_RAWEPHEM			  41     /* oem7/6/4 raw ephemeris */
#define ID_IONUTC			   8     /* oem7/6/4 iono and utc data */
#define ID_RAWWAASFRAME		 287     /* oem7/6/4 raw waas frame */
#define ID_RAWSBASFRAME		 973     /* oem7/6 raw sbas frame */
#define ID_GLOEPHEMERIS		 723     /* oem7/6/4 glonass ephemeris */
#define ID_GALEPHEMERIS		1122     /* oem7/6 decoded galileo ephemeris */
#define ID_GALIONO			1127     /* oem7/6 decoded galileo iono corrections */
#define ID_GALCLOCK			1121     /* oem7/6 galileo clock information */
#define ID_QZSSRAWEPHEM		1331     /* oem7/6 qzss raw ephemeris */
#define ID_QZSSRAWSUBFRAME  1330     /* oem7/6 qzss raw subframe */
#define ID_QZSSIONUTC		1347     /* oem7/6 qzss ion/utc parameters */
#define ID_BDSEPHEM			1696     /* oem7/6 decoded bds ephemeris */
#define ID_NAVICEPHEMERIS   2123     /* oem7 decoded navic ephemeris */
#define ID_GPSEPHEM			7		 /* oem7 decoded GPS L1 C/A ephemerides */
#define ID_BESTPOS			42		 /* oem7 decoded best position */

/* code IDs */
#define CODE_NONE   0                   /* obs code: none or unknown */
#define CODE_L1C    1                   /* obs code: L1C/A,G1C/A,E1C (GPS,GLO,GAL,QZS,SBS) */
#define CODE_L1P    2                   /* obs code: L1P,G1P,B1P (GPS,GLO,BDS) */
#define CODE_L1W    3                   /* obs code: L1 Z-track (GPS) */
#define CODE_L1Y    4                   /* obs code: L1Y        (GPS) */
#define CODE_L1M    5                   /* obs code: L1M        (GPS) */
#define CODE_L1N    6                   /* obs code: L1codeless,B1codeless (GPS,BDS) */
#define CODE_L1S    7                   /* obs code: L1C(D)     (GPS,QZS) */
#define CODE_L1L    8                   /* obs code: L1C(P)     (GPS,QZS) */
#define CODE_L1E    9                   /* (not used) */
#define CODE_L1A    10                  /* obs code: E1A,B1A    (GAL,BDS) */
#define CODE_L1B    11                  /* obs code: E1B        (GAL) */
#define CODE_L1X    12                  /* obs code: E1B+C,L1C(D+P),B1D+P (GAL,QZS,BDS) */
#define CODE_L1Z    13                  /* obs code: E1A+B+C,L1S (GAL,QZS) */
#define CODE_L2C    14                  /* obs code: L2C/A,G1C/A (GPS,GLO) */
#define CODE_L2D    15                  /* obs code: L2 L1C/A-(P2-P1) (GPS) */
#define CODE_L2S    16                  /* obs code: L2C(M)     (GPS,QZS) */
#define CODE_L2L    17                  /* obs code: L2C(L)     (GPS,QZS) */
#define CODE_L2X    18                  /* obs code: L2C(M+L),B1_2I+Q (GPS,QZS,BDS) */
#define CODE_L2P    19                  /* obs code: L2P,G2P    (GPS,GLO) */
#define CODE_L2W    20                  /* obs code: L2 Z-track (GPS) */
#define CODE_L2Y    21                  /* obs code: L2Y        (GPS) */
#define CODE_L2M    22                  /* obs code: L2M        (GPS) */
#define CODE_L2N    23                  /* obs code: L2codeless (GPS) */
#define CODE_L5I    24                  /* obs code: L5I,E5aI   (GPS,GAL,QZS,SBS) */
#define CODE_L5Q    25                  /* obs code: L5Q,E5aQ   (GPS,GAL,QZS,SBS) */
#define CODE_L5X    26                  /* obs code: L5I+Q,E5aI+Q,L5B+C,B2aD+P (GPS,GAL,QZS,IRN,SBS,BDS) */
#define CODE_L7I    27                  /* obs code: E5bI,B2bI  (GAL,BDS) */
#define CODE_L7Q    28                  /* obs code: E5bQ,B2bQ  (GAL,BDS) */
#define CODE_L7X    29                  /* obs code: E5bI+Q,B2bI+Q (GAL,BDS) */
#define CODE_L6A    30                  /* obs code: E6A,B3A    (GAL,BDS) */
#define CODE_L6B    31                  /* obs code: E6B        (GAL) */
#define CODE_L6C    32                  /* obs code: E6C        (GAL) */
#define CODE_L6X    33                  /* obs code: E6B+C,LEXS+L,B3I+Q (GAL,QZS,BDS) */
#define CODE_L6Z    34                  /* obs code: E6A+B+C,L6D+E (GAL,QZS) */
#define CODE_L6S    35                  /* obs code: L6S        (QZS) */
#define CODE_L6L    36                  /* obs code: L6L        (QZS) */
#define CODE_L8I    37                  /* obs code: E5abI      (GAL) */
#define CODE_L8Q    38                  /* obs code: E5abQ      (GAL) */
#define CODE_L8X    39                  /* obs code: E5abI+Q,B2abD+P (GAL,BDS) */
#define CODE_L2I    40                  /* obs code: B1_2I      (BDS) */
#define CODE_L2Q    41                  /* obs code: B1_2Q      (BDS) */
#define CODE_L6I    42                  /* obs code: B3I        (BDS) */
#define CODE_L6Q    43                  /* obs code: B3Q        (BDS) */
#define CODE_L3I    44                  /* obs code: G3I        (GLO) */
#define CODE_L3Q    45                  /* obs code: G3Q        (GLO) */
#define CODE_L3X    46                  /* obs code: G3I+Q      (GLO) */
#define CODE_L1I    47                  /* obs code: B1I        (BDS) (obsolute) */
#define CODE_L1Q    48                  /* obs code: B1Q        (BDS) (obsolute) */
#define CODE_L5A    49                  /* obs code: L5A SPS    (IRN) */
#define CODE_L5B    50                  /* obs code: L5B RS(D)  (IRN) */
#define CODE_L5C    51                  /* obs code: L5C RS(P)  (IRN) */
#define CODE_L9A    52                  /* obs code: SA SPS     (IRN) */
#define CODE_L9B    53                  /* obs code: SB RS(D)   (IRN) */
#define CODE_L9C    54                  /* obs code: SC RS(P)   (IRN) */
#define CODE_L9X    55                  /* obs code: SB+C       (IRN) */
#define CODE_L1D    56                  /* obs code: B1D        (BDS) */
#define CODE_L5D    57                  /* obs code: L5D(L5S),B2aD (QZS,BDS) */
#define CODE_L5P    58                  /* obs code: L5P(L5S),B2aP (QZS,BDS) */
#define CODE_L5Z    59                  /* obs code: L5D+P(L5S) (QZS) */
#define CODE_L6E    60                  /* obs code: L6E        (QZS) */
#define CODE_L7D    61                  /* obs code: B2bD       (BDS) */
#define CODE_L7P    62                  /* obs code: B2bP       (BDS) */
#define CODE_L7Z    63                  /* obs code: B2bD+P     (BDS) */
#define CODE_L8D    64                  /* obs code: B2abD      (BDS) */
#define CODE_L8P    65                  /* obs code: B2abP      (BDS) */
#define CODE_L4A    66                  /* obs code: G1aL1OCd   (GLO) */
#define CODE_L4B    67                  /* obs code: G1aL1OCd   (GLO) */
#define CODE_L4X    68                  /* obs code: G1al1OCd+p (GLO) */
#define MAXCODE     68                  /* max number of obs code */

#define SYS_NONE    0x00                /* navigation system: none */
#define SYS_GPS     0x01                /* navigation system: GPS */
#define SYS_SBS     0x02                /* navigation system: SBAS */
#define SYS_GLO     0x04                /* navigation system: GLONASS */
#define SYS_GAL     0x08                /* navigation system: Galileo */
#define SYS_QZS     0x10                /* navigation system: QZSS */
#define SYS_BDS     0x20                /* navigation system: BeiDou */
#define SYS_IRN     0x40                /* navigation system: IRNS */
#define SYS_LEO     0x80                /* navigation system: LEO */
#define SYS_ALL     0xFF                /* navigation system: all */

#define LLI_SLIP    0x01                /* LLI: cycle-slip */
#define LLI_HALFC   0x02                /* LLI: half-cycle not resovled */
#define LLI_BOCTRK  0x04                /* LLI: boc tracking of mboc signal */
#define LLI_HALFA   0x40                /* LLI: half-cycle added */
#define LLI_HALFS   0x80                /* LLI: half-cycle subtracted */

/* mathmatic functions */
#define SQR(x)		((x)*(x))
#define SQRT(x)     ((x)<=0.0||(x)!=(x)?0.0:sqrt(x))
#define MAX(x,y)    ((x)>(y)?(x):(y))
#define MIN(x,y)    ((x)<(y)?(x):(y))
#define ROUND(x)    (int)floor((x)+0.5)

/* MATH COMMON UINTS */
#define EPSILON		1e-10			         /* A very small threshold close to 0 */					
#define PI			3.1415926535898		     /* PI */
#define D2R			(PI/180.0)               /* Radians per degree */
#define R2D			(180.0/PI)               /* Degrees per radian */
#define CLIGHT		299792458.0              /* Speed of light  [m/s]; IAU 1976  */

/* solution params */
#define RTOL_KEPLER			1E-13				 /* relative tolerance for Kepler equation */
#define MAX_ITER_KEPLER		30				     /* max number of iteration of Kelpler */
#define SIN_5				-0.0871557427476582  /* sin(-5.0 deg) */
#define COS_5				0.9961946980917456	 /* cos(-5.0 deg) */
#define THRES_tk			7500.0
#define THRES_dLGF			0.05
#define THRES_dLMW			3.0
#define THRES_DT_FILE		15.0
#define THRES_DT_REALTIME	30.0
#define THRES_LSMAXCORR		1e-6
#define THRES_ABS_V_MAX		2.8
#define THRES_RTK_DDW		8.0
#define MAXITERS			10
#define MAX_DPOS_SPP		1e-4
#define MAX_DPOS_RTK		1e-8
#define GPST_BDT			14.0			     /* Difference between GPS time and Beidou time[s] */
#define GPST_BDT_WEEKS		1356
#define MAXCHANNUM			46
#define MAXFREQNUM			80
#define MAXGPSNUM			32
#define MAXBDSNUM			63
#define MAXSYSNUM			(MAXGPSNUM+MAXBDSNUM)
#define MAXPRNNUM			MAX(MAXGPSNUM,MAXBDSNUM)
#define MAXRAWLEN			40960			     /* max length of receiver raw message */
#define MAXMSGLEN			16384
#define BDSEPHEMLEN			196
#define GPSEPHEMLEN			224
#define MAXTIMEOUTS			5
#define NSATS_CHANGED		15

/* ECEF params */
#define R_WGS84		6378137.0               /* Radius Earth [m]; WGS-84  */
#define F_WGS84		(1.0/298.257223563)     /* Flattening; WGS-84   */
#define Omega_WGS	7.2921151467e-5         /*[rad/s], the earth rotation rate */
#define GM_WGS84	398600.5e+9             /* [m^3/s^2]; WGS-84 */

#define R_CGCS2000  6378137.0               /* Radius Earth [m]; CGCS2000  */
#define F_CGCS2000  (1.0/298.257222101)     /* Flattening; CGCS2000   */
#define Omega_CGCS  7.2921150e-5            /*[rad/s], the earth rotation rate */
#define GM_CGCS     398600.4418e+9          /* [m^3/s^2]; CGCS2000  */

/* some constants about GPS signal */
#define FG1_GPS    1575.42E6				/* L1 signal frequency */
#define FG2_GPS    1227.60E6				/* L2 signal frequency */
#define FG12R      (77/60.0)				/* FG1_Freq/FG2_Freq */
#define WL1_GPS	   (CLIGHT/FG1_GPS)			/* L1 signal wavelength */
#define WL2_GPS    (CLIGHT/FG2_GPS)			/* L2 signal wavelength */

/* some constants about BDS signal */
#define FG1_BDS	   1561.098E6				/* B1 signal frequency */
#define FG2_BDS    1207.140E6				/* B2 signal frequency */
#define FG3_BDS    1268.520E6				/* B2 signal frequency */
#define FC12R      (763/590.0)				/* B1 signal frequency/B2 signal frequency */
#define FC13R      (763/620.0)				/* B1 signal frequency/B3 signal frequency */
#define WL1_BDS    (CLIGHT/FG1_BDS)			/* B1 signal wavelength */
#define WL2_BDS    (CLIGHT/FG2_BDS)			/* B2 signal wavelength */
#define WL3_BDS    (CLIGHT/FG3_BDS)			/* B3 signal wavelength */

/* HEADER to be printed */
#define PROGRAM_NAME "[REAL-TIME RTK SOFTWARE]"
#define AUTHOR "lewis5499@whu.edu.cn"
#define VERSION "2.0"
#define COPYRIGHT "Copyright(C) 2024 by H.Z.Liu, All rights reserved."
#define HEADER \
"    #########################################################\n"\
"    ##                                                     ##\n"\
"    ##              " PROGRAM_NAME "               ##\n"\
"    ##                                                     ##\n"\
"    ##      Contact: " AUTHOR "                  ##\n"\
"    ##                                                     ##\n"\
"    ##      Version: " VERSION "                                   ##\n"\
"    ##                                                     ##\n"\
"    ##  " COPYRIGHT " ##\n"\
"    ##                                                     ##\n"\
"    #########################################################\n\n"

#define HEADER_POSFILE " Wk     SOW         ECEF-X/m       ECEF-Y/m       ECEF-Z/m    \
REF-ECEF-X/m    REF-ECEF-Y/m   REF-ECEF-Z/m   EAST/m  NORTH/m  UP/m       B/deg          \
L/deg        H/m      VX/m     VY/m     VZ/m    BaseLine_dX/m  BaseLine_dY/m  BaseLine_dZ/m  \
BaseLine_dE/m  BaseLine_dN/m  BaseLine_dU/m  PDOP    SigmaP   SigmaV  DD-GS  DD-BS  DD-n   Ratio  State  FixRMS  GS  BS   n"

#define HEADER_CONSOLE " Wk     SOW        ECEF-X/m       ECEF-Y/m      ECEF-Z/m     \
B/deg    L/deg     H/m      E/m      N/m      U/m      dX/m       dY/m       dZ/m    Ratio  State  PDOP   GS BS nFreq\n"


/* type definitions ---------------------------------------------------------------*/

enum sys_t { UNKS = 0, GPS, BDS, GLONASS, GALILEO, QZSS };

struct mjdtime_t {             
	int		Days;
	double	FracDay;

	mjdtime_t();
	mjdtime_t(int d, double f);
};

struct utime_t {   
	short	 Year;
	uint16_t Month;
	uint16_t Day;
	uint16_t Hour;
	uint16_t Minute;
	double   Second;

	utime_t();
	utime_t(short y, uint16_t mon, uint16_t d, uint16_t h, uint16_t min, double s);
};

struct gtime_t {              
	uint16_t Week;
	double   SecOfWeek;

	gtime_t();
	gtime_t(uint16_t w, double s);
	gtime_t(gtime_t *t);
};

struct dcode_t {
	int msgtype;
	uint16_t week, msglen, msgID;
	double tow;
};

struct track_t {
	int sys, code, track, sigtp;
	int plock, clock, parity, halfc;
};

struct gpseph_t {			
	uint16_t PRN;
	sys_t    Sys;
	gtime_t  TOC, TOE;
	short	 SVHealth;
	double	 ClkBias, ClkDrift, ClkDriftRate;
	double	 IODE1, IODC, IODE2;
	double   TGD;
	double	 SqrtA, e, M0, OMEGA, i0, omega, OMEGADot, iDot, DeltaN;
	double	 Crs, Cuc, Cus, Cic, Cis, Crc;
	double	 SVAccuracy;

	gpseph_t();
};

struct bdseph_t {
	uint16_t PRN;
	sys_t    Sys;
	gtime_t  TOC, TOE;
	short	 SVHealth;
	double	 ClkBias, ClkDrift, ClkDriftRate;
	double	 IODE, IODC;
	double   TGD1, TGD2;
	double   SqrtA, e, M0, OMEGA, i0, omega, OMEGADot, iDot, DeltaN;
	double	 Crs, Cuc, Cus, Cic, Cis, Crc;
	double	 SVAccuracy;

	bdseph_t();
};

struct satobs_t {
	sys_t    Sys;
	short    Prn;
	double   P[2], L[2], D[2];   // m
	double   cn0[2], LockTime[2];
	uint8_t  half[2];
	int		 parity[2], track[2], lli[2];
	bool     Valid, validP[2], validL[2];

	// valid=false: 1.当前历元双频P/L四个数据不全;
	//				2.或上个历元卫星存在,且当前历元初次跟踪到全的P/L四个数据,
	//                认为当前历元“初次”跟踪到全的双频数据，质量不能保证，认为不可用。
	//				3.或上个历元卫星不存在，且当前历元初次跟踪到全的P/L四个数据。
	//				4.经MW/GF组合探测，当前SatObs双频观测值存在粗差
	// valid=true: 上个历元与当前历元均有双频P/L四个数据,且经MW、GF组合探测无周跳。
	// validL[2]={true,true}: 该频点有载波相位观测值，且该观测值无lli异常标记。
	//			={false,...}: 1.MW/GF组合探测双频观测值存在周跳，2.或当前历元初次跟踪到全双频观测值
	// validP[2]={true,true}: 该频点有伪距观测值(注意: 后续应增加伪距站间单差的粗差探测函数)

	satobs_t();
};

struct mwgf_t {
	sys_t   Sys;
	short	Prn;
	double	LMW, stdLMW;
	double	PGF, LGF;
	double	PIF, LIF;
	int		n;

	mwgf_t();
	void reset();
};

struct satres_t {
	sys_t   Sys;
	short	prn;
	double	SatPos[3], SatVel[3];
	double	SatClkOft, SatClkSft;
	double	Elevation, Azimuth;
	double	TropCorr;
	double	Tgd1, Tgd2;
	double	PIF, LIF;
	bool	Valid;  //false=no ephemeris or ephemeris expired, or too small elevation angle
					//true=calculation successful
	satres_t();
};

struct refpos_t {
	gtime_t Time;
	double	XYZ[3];
	double	ENU[3];

	refpos_t();
};

struct epoch_t {
	gtime_t   Time;				  // Current obs time: GPST.
	short     nobs;			      // The total number of obs values from all obs satellites.
	short     SatNum;			  // obs sats nums(GPS+BDS)
	satobs_t  SatObs[MAXSYSNUM];  // GPS, BDS in order
	satres_t  SatPVT[MAXSYSNUM];  // Calculation results such as satellite positions, the array index is the same as 'SatObs'
	mwgf_t    COBObs[MAXSYSNUM];  // Combined observations for the current epoch, the array index is the same as 'SatObs'
	refpos_t  BestPos;
	gtime_t   tobs[MAXSYSNUM][2]; // observation data time
	double    LockT[MAXSYSNUM][2];// GPS+BDS, L1/L2 + B1/B3 
	bool	  getBestPos;
	uint8_t   type;			      // type=1(default)-> rover stat, type=0->base stat

	epoch_t();
	void reset();
};

struct pos_t {
	gtime_t Time;
	double	Pos[3];
	double	Vel[3];
	double	RcvClkOft[2];              /* 0=GPS clock offset; 1=BDS clock offset */
	double	RcvClkSft;
	double	PDOP, SigmaPos, SigmaVel;  /* Accuracy index */
	double	Q[6*6], P[6*6];
	short	nn;						   /* necessary obs nums */
	short	GPSSatNum, BDSSatNum;      /* Number of GPS/BDS satellites used for SPP */
	short	AllSatNum;                 /* Number of all satellites in the observation epoch */
	bool	IsSuccess;                 /* Record whether SPP is successful or not */
	bool	ISFIRST;

	pos_t();
	void reset();
};

struct sdsat_t {
	short   prn;
	sys_t   sys;
	short   ValidP[2], ValidL[2];	// 1=healthy; 0=cycle slip or outlier
	double  dP[2], dL[2];			// unit: meter
	short   nBas, nRov;			    // Storage of indices
	bool	halfc[2];				// true = has half-cyle, false = healthy

	sdsat_t();
	bool flfreqchk();
};

struct sdobs_t {
	gtime_t  Time;
	short    SatNum;
	sdsat_t  SDSatObs[MAXCHANNUM];
	mwgf_t   SDCOBObs[MAXCHANNUM];

	sdobs_t();

	void reset();
	bool checkFullFreq(int idx);	   // not include halfc
	bool checkFreq(int idx, int freq); // include halfc
};

struct ddobs_t
{
	int		RefPrn[2], RefIdx[2];				// the 'prn' and 'idx' of ref-sat in 'SD', 0=GPS; 1=BDS
	int		FreqRefPrn[2][2], FreqRefIdx[2][2]; // sys/freq: 4 ref-sats for GPS L1/L2, BDS B1/B3
	int		nSat, nSysSat[2];					// nums of dd-sats, nums of dd-sats of each sys, 0=GPS; 1=BDS
	int		nFreq, nFreqSat[2][2];				// nums of dd-sats' freqs, [sys][freq]
	double	FixedAmb[MAXCHANNUM*2*2];			// Contains dual-frequency optimal solution and suboptimal solution;
	double	ResAmb[2], Ratio, dPos[3];			// Ambiguity residuals in LAMBDA float solutions, ratio, vector of baseline
	float	FixRMS[2];							// Fixed solution positioning rms error
	short	bFixed;								// -1=SPP, 0=Float, 1=Fixed
	int		*type, *SDlist;						// rtkFloat params
	double	*X, *Q, *Rdist, *Bdist;				// rtkFloat params

	ddobs_t();
	void reset();
};

struct conf_t {
	short	dataMode, sppFilter, rtkFilter, kinematic;	// 0=file, 1=com; 0=LSQ,  1=EKF
	short	saveRawBin, saveRTKResToFile, fullfreqs;	// 0=false, 1=true
	char	ipBase[20], ipRover[20];					// COM IPs   of Base and Rover
	short	portBase, portRover;						// COM ports of Base and Rover
	float	elevaThreshold, ratioThreshold;				// Sats cut-off Elevation Angle, Ratio Threshold
	float	hopfdH0, hopfdT0, hopfdp0, hopfdRH0;		// Hopfield parameters
	double	basRefPos[3], rovRefPos[3];					// reference pos of Base/Rover station
	float	CodeNoise, CPNoise;							// pseudorange noise, carrier phase noise
	float	RcvHoriBias, RcvVertBias, AmbBias, RcvClkBias, RcvClkDft;	// Process noise
	char	pathBaseBin[256], pathRoverBin[256];		// Base/Rover binary files' paths
	char	pathRoverLog[256], pathRoverPos[256];		// Base/Rover log	 files' paths
	char	pathBaseBinRTK[256], pathRoverBinRTK[256];	// Base/Rover binary saved files' paths when RTK 
	char	pathRoverLogRTK[256], pathRoverPosRTK[256];	// Base/Rover log and pos result files' paths when RTK 
	short	enableTrace, enableLog;						// a trace-enable mark
	char	traceLogPath[256];							// trace: 'Log' file path
};

struct raw_t {
	epoch_t	 BasObs;
	epoch_t	 RovObs;
	sdobs_t	 SDObs;
	ddobs_t	 DDObs;
	gpseph_t ephgps[MAXGPSNUM];
	bdseph_t ephbds[MAXBDSNUM];

	raw_t();
};

struct lsq_t {
	double *X; int sizeX;
	double *B; int rowB, colB;
	double *P; int sizeP;
	double *w; int sizeW;
	double *Q; int sizeQ;
	double *m; int sizeM;
	double *v; int sizeV;
	double mse;

	lsq_t();
	void setParam(double *B, double *P, double *w, double *Q, double *m, int rowB, int colB, double *X);
	void reset();
	double lsq();
};

struct ekf_t {
	gtime_t Time;
	ddobs_t preDD;
	sdobs_t preSD;
	double  *X, *P, *Q, *C, *R, *T, *H, *v;
	int     p, q, n, m, nFreqs;
	double  X_[3+MAXFREQNUM], P_[SQR((3+MAXFREQNUM))], Xlsq[3];		// state backup
	int	    Index_[MAXFREQNUM][2], Index[MAXFREQNUM][2];	// valid SDObs idx&freq &backup
	int		AmbFixed[MAXFREQNUM];	// ambiguities that 'Fix and Hold', -1=not fixed or cycle-slip, 0=fixed
	double	AmbFloat[MAXFREQNUM];	// LSQ float ambiguities
	bool	IsInit;

	ekf_t();
	void setParam(double *x, double *P, double *Q, double *C, double *R, double *T, double *H, double *v, int p, int q, int n, int m);
	void reset();
	bool check();
	void initP();
	void calcC();
	void calcQ(raw_t *raw);
	void calcT(raw_t *raw);
	void calcV(raw_t *raw);
	void calcH(raw_t *raw);
	void calcR(raw_t *raw);
	void reInit();
	int ekf(raw_t *raw);
};

struct callback_t {
	raw_t		raw;
	pos_t		posBas, posRov;
	SOCKET		NetRov, NetBas;
	HANDLE		Timer, TimerQueue;
	int8_t	   *buffRov, *BuffRov, *buffBas, *BuffBas;
	FILE	   *rawBasfp, *rawRovfp, *logRovfp, *posRovfp, *cmpRovfp;
	const char *filelog, *filepos, *rawRov, *rawBas;
	char		filecmp[256];
	bool		rawRovOpen, rawBasOpen;
	int			lenRRov, lenDRov, idxRov;
	int			lenRBas, lenDBas, idxBas;
	int			RovTimeouts, BasTimeouts;
	int			sync;

	callback_t();
	~callback_t();
};


/* NovArel oem7 decoding functions ------------------------------------------------*/
#define U1(p) (*((uint8_t *)(p)))
#define I1(p) (*((int8_t  *)(p)))
static uint16_t U2(uint8_t *p) { uint16_t u; memcpy(&u, p, 2); return u; }
static uint32_t U4(uint8_t *p) { uint32_t u; memcpy(&u, p, 4); return u; }
static int32_t  I4(uint8_t *p) { int32_t  i; memcpy(&i, p, 4); return i; }
static float    R4(uint8_t *p) { float    r; memcpy(&r, p, 4); return r; }
static double   R8(uint8_t *p) { double   r; memcpy(&r, p, 8); return r; }
uint32_t rtkCRC32(const uint8_t *buff, const int len);
int sig2code(int sys, int sigtype);
int syncOEM7(uint8_t *buff, uint8_t data);
int decodeOEM7f(FILE* fp, uint8_t *buff, epoch_t *obs, gpseph_t *ephgps, bdseph_t *ephbds);
int decodeOEM7(int8_t *Buff, uint8_t *buff, epoch_t *obs, gpseph_t *ephgps, bdseph_t *ephbds, int pos, int &lenD);
int decodeMsg(uint8_t *buff, epoch_t *obs, gpseph_t *ephgps, bdseph_t *ephbds, dcode_t *dat);
int decodeTrackStat(uint32_t stat, track_t * track);
int decodeRangeb(uint8_t *buff, epoch_t *obs, dcode_t *dat);
int decodeGpsEphem(uint8_t *buff, gpseph_t *eph, dcode_t *dat);
int decodeBdsEphem(uint8_t *buff, bdseph_t *eph, dcode_t *dat);
int decodeBestpos(uint8_t *buff, epoch_t *obs);

/* time, string, and coordinate functions ------------------------------------------------------*/
bool CT2MJD(const utime_t *CT, mjdtime_t *MJDT);
bool MJD2CT(const mjdtime_t *MJDT, utime_t *CT);
bool GPST2MJD(const gtime_t *GT, mjdtime_t *MJDT);
bool MJD2GPST(const mjdtime_t *MJDT, gtime_t *GT);
bool CT2GPST(const utime_t *CT, gtime_t *GT);
bool GPST2CT(const gtime_t *GT, utime_t *CT);
double diffTime(const gtime_t *GT2, const gtime_t*GT1);
void deg2dms(const double deg, double *dms, int ndec);
double dms2deg(const double *dms);
void blh2enuMat(const double *blhPos, double *e2nMat);
void xyz2blh(const double *xyzPos, double *blhPos);
void blh2xyz(const double *blhPos, double *xyzPos);
void xyz2enu(const double *xyzPos, const double *refxyzPos, double *enuPos);
void xyz2enu(const double *blh, double *E);
void enu2xyz(const double *blh, const double *enu, double *xyz);
void covenu(const double *pos, const double *P, double *Q);
void covxyz(const double *pos, const double *Q, double *P);
void compSatElAz(const double *Xr, const double *Xs, double *Elev, double *Azim);
void denuPos(const double *X0, const double *Xr, double *dNEU);
double geodist(const double *rs, const double *rr);
string getCurrentTimeString();
string convertToStdString(const char* input);

/* outlier/cycle-slip detection ---------------------------------------------------*/
void checkConsistency(satobs_t *pre, satobs_t *cur, int nn, int n, double dt);
void markOutlier(epoch_t *obs);
int initMWGF(epoch_t *obs, const int idx);
void detectOutlier(epoch_t *obs, const mwgf_t *lastEpoch);
void markCycleSlip(sdobs_t *SDobs);
int initMWGF(sdobs_t *SDobs, const int idx);
void detectCycleSlip(sdobs_t *SDobs, mwgf_t *lastEpoch);
void MWGFdpcy(const mwgf_t *source, mwgf_t *dst);

/* satellites, obs, clocks, systems, models functions -----------------------------*/
double svClkCorr(const double dt, const double *params);
double svClkdCorr(const double dt, const double *params);
double *earthRotCorr(const double *vec3, const double time, const double omga);
double getPIF(epoch_t *obs, const int idx, const int sys);
double getSDObs(const satobs_t& sati, const satobs_t& satj, int k, short flag);
double Hopfield(const double H, const double Elev);
void tropCorr(epoch_t *obs, const double *Xr, double *BLHr, const int *locs, const int num);
void satposs(gpseph_t *ephgps, bdseph_t *ephbds, epoch_t *obs, const bool isSPP);
int gpsposs(gpseph_t *ephgps, epoch_t *obs, const int prn_1, const bool isSPP, const double dt);
int bdsposs(bdseph_t *ephbds, epoch_t *obs, const int prn_1, const bool isSPP, const double dt);
int searchSat(epoch_t *obs, const short prn, const int sys);
int psatposs(epoch_t *obs, gpseph_t *ephgps, bdseph_t *ephbds, const double *Xr);
int pgpsposs(epoch_t *obs, gpseph_t *ephgps, const int idx, const double dt);
int pbdsposs(epoch_t *obs, bdseph_t *ephbds, const int idx, const double dt);
int searchSats(epoch_t *obs, int *locs, int &num, int &numgps, int &numbds); //spp
int searchSats(epoch_t *obs, int *locs, int &num); //spv
void inOrder(epoch_t *obs);
void SDObs(raw_t *raw);
void setRefSat(const epoch_t *base, const epoch_t *rover, sdobs_t *SDObs, ddobs_t *DDObs);

/* spp, rtk main functions --------------------------------------------------------*/
void sppMain(epoch_t *obs, gpseph_t *ephgps, bdseph_t *ephbds, pos_t *sppPos);
void rtkMain(raw_t *raw, pos_t *base, pos_t *rover);
int sppspv(epoch_t *obs, gpseph_t *ephgps, bdseph_t *ephbds, pos_t *pos);
int rtkSyncf(FILE *fRov, FILE *fBas, raw_t *raw);
int rtkSync(callback_t *params);
int rtkFloat(raw_t *raw, pos_t *posBas, pos_t *posRov);
int rtkFixed(raw_t *raw, pos_t *posBas, pos_t *posRov);

/* Least Squares ------------------------------------------------------------------*/
int lsqspp_(epoch_t *obs, const int *locs, double *Xr, double *rcvClkOft, double *Qxx, double *Dxx, int colB, int rowB, double &Mse, double &Pdop, double *v);
int lsqspv_(epoch_t *obs, const int *locs, const int rowB, const int colB, double *vel, double *Xr, double &sigmaVel, double &rcvClksft);
int lsqspp(epoch_t *obs, gpseph_t *ephgps, bdseph_t *ephbds, pos_t *pos);
int lsqspv(epoch_t *obs, pos_t *pos);

/* Extended Kalman Filter ---------------------------------------------------------*/
int smoother(const double *xf, const double *Qf, const double *xb, const double *Qb, int n, double *xs, double *Qs);
int filterspp(const double *Xk_1, const double *Pxk_1, double *Xkk_1, double *Pkk_1);
int filterspp(const double *Pkk_1, const double *Xkk_1, double *Xk, double *Pxk, epoch_t *obs, const int *loc, int num);
int ekfspp(epoch_t *obs, gpseph_t *ephgps, bdseph_t *ephbds, pos_t *pos);
int filter(double *x, double *P, const double *H, const double *v, const double *R, int n, int m);
int filter(double *&x, double *&P, const double *Q, const double *T, const double *C, int p, int q, int n);

/* win32 socket functions ---------------------------------------------------------*/
int  socketMode();
int  paramset(callback_t *p);
void paramfree(callback_t *p);
void checkstat(int *stat, fd_set *set, callback_t *p); //[0]=base, [1]=rover
void rtksol(callback_t *p);
void cache(callback_t *p, char type);
void rejoin(callback_t *p, char type); //type==0: base, type==1: rover
bool openSocket(SOCKET& sock, const char IP[], const unsigned short Port);
void closeSocket(SOCKET& sock);
VOID CALLBACK TimerCallback(PVOID lpParam, BOOLEAN TimerOrWaitFired);

/* rtk software entry's functions -------------------------------------------------*/
void menu();
int loadConf(const char *cfgfilepath, conf_t& rtkCfg);
int fileMode(const char *fileRov, const char *filelog, const char *filepos, const char *fileBas);
int inputOEM7f(FILE *fp, epoch_t &obs, gpseph_t *ephgps, bdseph_t *ephbds);
int inputOEM7(int8_t *Buff, int &lenD, epoch_t *obs, gpseph_t* ephgps, bdseph_t* ephbds);
int outputOEM7f(FILE *fpLog, FILE *fpRes, pos_t *pos, raw_t *raw);
int outputOEM7(epoch_t *obs, pos_t *pos, raw_t* raw);
int outputPOSf(FILE *fpRes, pos_t *pos, raw_t *raw, char type);
void printSatLog(satres_t *SatPVT, FILE *fp_log);
void printPosLog(pos_t *pos, FILE *fp_log);
void printPos(FILE *fp, raw_t *raw, pos_t *pos);
int posCheck(epoch_t *obs, pos_t *pos, raw_t* raw);

#endif