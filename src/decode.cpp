/*------------------------------------------------------------------------------
* decode.cpp : RTK software decoding operations
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
#include "rtklib.h"
#include "trace.h"

extern ekf_t ekfParam;

#define U1(p) (*((uint8_t *)(p)))
#define I1(p) (*((int8_t  *)(p)))
static uint16_t U2(uint8_t *p) { uint16_t u; memcpy(&u, p, 2); return u; }
static uint32_t U4(uint8_t *p) { uint32_t u; memcpy(&u, p, 4); return u; }
static int32_t  I4(uint8_t *p) { int32_t  i; memcpy(&i, p, 4); return i; }
static float    R4(uint8_t *p) { float    r; memcpy(&r, p, 4); return r; }
static double   R8(uint8_t *p) { double   r; memcpy(&r, p, 8); return r; }

/* sync header ----------------------------------------------------------------------------------*/
int syncOEM7(uint8_t *buff, uint8_t data) {
	buff[0] = buff[1]; buff[1] = buff[2]; buff[2] = data;
	return buff[0] == OEM4SYNC1 && buff[1] == OEM4SYNC2 && buff[2] == OEM4SYNC3;
}

/* decode one "AA4412" in buff ------------------------------------------------------------------
* it should be confirmed that buff[0]='AA',buff[1]='44',buff[2]='12'
* ---------------------------------------------------------------------------------------------- */
int decodeOEM7f(FILE* fp, uint8_t *buff, epoch_t *obs, gpseph_t *ephgps, bdseph_t *ephbds) 
{
	dcode_t dat;
	
	fread(buff + 3, 25, 1, fp);// has read 'header', buff[0]=AA, buff have 28 elements
	dat.tow		= U4(buff + 16)*0.001;
	dat.week	= U2(buff + 14);
	dat.msglen	= U2(buff + 8);
	dat.msgID	= U2(buff + 4);
	dat.msgtype = (U1(buff + 6) >> 4) & 0x3; /* message type: 0=binary,1=ascii */

	if (dat.msglen > MAXMSGLEN) { //CRC32->msglen error!
		tracef("[WARNINGS]: OEM7 MESSAGE LENGTH ERROR OCCURS AT %d, %.3f\n", dat.week, dat.tow);
		return 2;	//'return 2'->'continue' in the outer loop
	}

	fread(buff + OEM4HLEN, dat.msglen + CRC32LEN, 1, fp);// read msg+CRC32, then 'buff' has OEM4HLEN+msglen+CRC32LEN elements

	return decodeMsg(buff, obs, ephgps, ephbds, &dat);
}

/* SOCKET decode one "AA4412" in buff----------------------------------------------------------- */
int decodeOEM7(int8_t *Buff, uint8_t *buff, epoch_t *obs, gpseph_t *ephgps, bdseph_t *ephbds, int pos, int &lenD) 
{
	dcode_t dat; int lenU; // lenU: length of used buff

	memcpy(buff+3, Buff+pos+1, 25*sizeof(int8_t));// has read 'header', onebuff[0]='AA', buff have 28 elements
	dat.tow		= U4(buff + 16)*0.001;
	dat.week	= U2(buff + 14);
	dat.msglen	= U2(buff + 8);
	dat.msgID	= U2(buff + 4);
	dat.msgtype = (U1(buff + 6) >> 4) & 0x3; /* message type: 0=binary,1=ascii */

	tracef("[OEM7 MSG] lenD=%d AT %d, %.3f WITH STAT TYPE: %d\n", lenD, dat.week, dat.tow, obs->type);

	if (dat.msglen > MAXMSGLEN) { //CRC32->msglen error!
		tracef("[WARNINGS]: MESSAGE LENGTH ERROR OCCURS AT %d, %.3f\n", dat.week, dat.tow);
		lenU = OEM4HLEN + pos - 2;
		if (lenU > lenD) return -2;
		lenD -= lenU;
		memmove(Buff, Buff + lenU, lenD);
		return 2;
	}

	lenU = OEM4HLEN + dat.msglen + CRC32LEN + pos - 2;
	if (lenU > lenD) return -2;
	
	memcpy(buff+28, Buff+pos+26, sizeof(int8_t)*(dat.msglen+CRC32LEN)); // read msg+CRC32, then 'buff' has OEM4HLEN+msglen+CRC32LEN elements
	lenD -= lenU;
	memmove(Buff, Buff + lenU, lenD);

	return decodeMsg(buff, obs, ephgps, ephbds, &dat);
}

/* SOCKET decode one Message in buff------------------------------------------------------------ */
int decodeMsg(uint8_t *buff, epoch_t *obs, gpseph_t *ephgps, bdseph_t *ephbds, dcode_t *dat) {
	/* check crc32 */
	if (rtkCRC32(buff, OEM4HLEN + dat->msglen) != U4(buff + OEM4HLEN + dat->msglen)) {
		logout("[LOG]: CRC32 FALIURE OCCURS AT %d, %.3f\n", dat->week, dat->tow);
		return 2;//'return 2'->'continue' in the outer loop

	} else if (dat->msgID == ID_RANGE && dat->msgtype == 0) {
		int idx, nn, n; 
		satobs_t pre[MAXSYSNUM]; 
		gtime_t pret(&obs->Time);

		nn = obs->SatNum;
		memcpy(pre, obs->SatObs, sizeof(satobs_t)*MAXSYSNUM);
		(*obs).reset();
		idx = decodeRangeb(buff, obs, dat);
		n = obs->SatNum;
		if (idx != -1) checkConsistency(pre, obs->SatObs, nn, n, diffTime(&obs->Time, &pret));
		return 0;

	} else if (dat->msgID == ID_GPSEPHEM && dat->msgtype == 0) {
		decodeGpsEphem(buff, ephgps, dat);
		return 1;

	} else if (dat->msgID == ID_BDSEPHEM && dat->msgtype == 0) {
		decodeBdsEphem(buff, ephbds, dat);
		return 1;

	} else if (dat->msgID == ID_BESTPOS && dat->msgtype == 0) {
		if ((obs->getBestPos==false&&obs->type==0) || obs->type==1) {
			decodeBestpos(buff, obs);
			obs->getBestPos = true;
		}
		return 1;

	} else {
		//printf("[WARNINGS]: UNMATCHED OEM7 MESSAGE!\n");
	};

	return 2;
}

/* check crc32 --------------------------------------------------------------------------------- */
uint32_t rtkCRC32(const uint8_t *buff, const int len){
	int i, j; uint32_t crc = 0;

	for (i = 0; i < len; i++) {
		crc ^= buff[i];
		for (j = 0; j < 8; j++) {
			if (crc & 1) 
				crc = (crc >> 1) ^ POLYCRC32; 
			else 
				crc >>= 1;
		}
	}
	return crc;
}

/* signal type to obs code --------------------------------------------------------------------- */
static int sig2code(int sys, int sigtype) {
	if (sys == SYS_GPS) {
		switch (sigtype) {
		case  0: return CODE_L1C; /* L1C/A */
		case  5: return CODE_L2P; /* L2P    (OEM7) */
		case  9: return CODE_L2W; /* L2P(Y),semi-codeless */
		case 14: return CODE_L5Q; /* L5Q    (OEM6) */
		case 16: return CODE_L1L; /* L1C(P) (OEM7) */
		case 17: return CODE_L2S; /* L2C(M) (OEM7) */
		}
	} else if (sys == SYS_GLO) {
		switch (sigtype) {
		case  0: return CODE_L1C; /* L1C/A */
		case  1: return CODE_L2C; /* L2C/A (OEM6) */
		case  5: return CODE_L2P; /* L2P */
		case  6: return CODE_L3Q; /* L3Q   (OEM7) */
		}
	} else if (sys == SYS_GAL) {
		switch (sigtype) {
		case  2: return CODE_L1C; /* E1C  (OEM6) */
		case  6: return CODE_L6B; /* E6B  (OEM7) */
		case  7: return CODE_L6C; /* E6C  (OEM7) */
		case 12: return CODE_L5Q; /* E5aQ (OEM6) */
		case 17: return CODE_L7Q; /* E5bQ (OEM6) */
		case 20: return CODE_L8Q; /* AltBOCQ (OEM6) */
		}
	} else if (sys == SYS_QZS) {
		switch (sigtype) {
		case  0: return CODE_L1C; /* L1C/A */
		case 14: return CODE_L5Q; /* L5Q    (OEM6) */
		case 16: return CODE_L1L; /* L1C(P) (OEM7) */
		case 17: return CODE_L2S; /* L2C(M) (OEM7) */
		case 27: return CODE_L6L; /* L6P    (OEM7) */
		}
	} else if (sys == SYS_BDS) {
		switch (sigtype) {
		case  0: return CODE_L2I; /* B1I with D1 (OEM6) */
		case  1: return CODE_L7I; /* B2I with D1 (OEM6) */
		case  2: return CODE_L6I; /* B3I with D1 (OEM7) */
		case  4: return CODE_L2I; /* B1I with D2 (OEM6) */
		case  5: return CODE_L7I; /* B2I with D2 (OEM6) */
		case  6: return CODE_L6I; /* B3I with D2 (OEM7) */
		case  7: return CODE_L1P; /* B1C(P) (OEM7) */
		case  9: return CODE_L5P; /* B2a(P) (OEM7) */
		case 11: return CODE_L7D; /* B2b(I) (OEM7,F/W 7.08) */
		}
	} else if (sys == SYS_IRN) {
		switch (sigtype) {
		case  0: return CODE_L5A; /* L5 (OEM7) */
		}
	} else if (sys == SYS_SBS) {
		switch (sigtype) {
		case  0: return CODE_L1C; /* L1C/A */
		case  6: return CODE_L5I; /* L5I (OEM6) */
		}
	}
	return 0;
}

/* decode receiver tracking status --------------------------------------------------------------
* decode receiver tracking status
* args   : uint32_t stat I  tracking status field
*          int    *sys   O      system (SYS_???)
*          int    *code  O      signal code (CODE_L??)
*          int    *track O      tracking state
*                         (OEM4/5)
*                         0=L1 idle                   8=L2 idle
*                         1=L1 sky search             9=L2 p-code align
*                         2=L1 wide freq pull-in     10=L2 search
*                         3=L1 narrow freq pull-in   11=L2 pll
*                         4=L1 pll                   12=L2 steering
*                         5=L1 reacq
*                         6=L1 steering
*                         7=L1 fll
*                         (OEM6/7)
*                         0=idle                      7=freq-lock loop
*                         2=wide freq band pull-in    9=channel alignment
*                         3=narrow freq band pull-in 10=code search
*                         4=phase lock loop          11=aided phase lock loop
*          int    *plock O      phase-lock flag   (0=not locked, 1=locked)
*          int    *clock O      code-lock flag    (0=not locked, 1=locked)
*          int    *parity O     parity known flag (0=not known,  1=known)
*          int    *halfc O      phase measurement (0=half-cycle not added,
*                                                  1=added)
* return : freq-index (-1: when exclude[GPS:L1C/L2W BDS:BII/B3I] )
* notes  : null
*----------------------------------------------------------------------------------------------- */
int decodeTrackStat(uint32_t stat, track_t *t)
{
	int satsys, obstp = -1;

	t->code    = CODE_NONE;
	t->track   = stat & 0x1F;
	t->plock   = (stat >> 10) & 1;
	t->parity  = (stat >> 11) & 1;
	t->clock   = (stat >> 12) & 1;
	satsys   = (stat >> 16) & 7;	//0:GPS, 4:BDS
	t->halfc   = (stat >> 28) & 1;
	t->sigtp = (stat >> 21) & 0x1F; //sigtype = GPS(0:L1C,9:L2W), BDS(0,4:L2I,2,6:L6I)

	switch (satsys) {
		case 0: t->sys = SYS_GPS; break;
		case 1: t->sys = SYS_GLO; return -1;
		case 2: t->sys = SYS_SBS; return -1;
		case 3: t->sys = SYS_GAL; return -1;  /* OEM6 */
		case 4: t->sys = SYS_BDS; break;	  /* OEM6 F/W 6.400 */
		case 5: t->sys = SYS_QZS; return -1;  /* OEM6 */
		case 6: t->sys = SYS_IRN; return -1;  /* OEM7 */
		default: {
			tracef("[WARNINGS]: OEM7 UNKNOWN SYSTEM: sys=%d AT %d %f\n", satsys, ekfParam.Time.Week, ekfParam.Time.SecOfWeek);
		}
	}
	t->code = sig2code(t->sys, t->sigtp);
	if (!(t->code)) {
		tracef("[WARNINGS]: OEM7 SIGNAL TYPE ERROR: sys=%d sigtype=%d AT %d %f\n", t->sys, t->sigtp, ekfParam.Time.Week, ekfParam.Time.SecOfWeek);
		return -1;
	}
	//satsys = 0:GPS, 4:BDS  //sigtype = GPS(0:L1C,9:L2W), BDS(0,4:L2I,2,6:L6I)
	if ((satsys==0&&(t->sigtp==0||t->sigtp==9))||(satsys==4&&(t->sigtp==0||t->sigtp==4||t->sigtp==2||t->sigtp==6)))
		obstp = 0;
	else
		return -1;

	return obstp;
}

/* decode binary range message------------------------------------------------------------------ */
int decodeRangeb(uint8_t *buff, epoch_t *obs, dcode_t *dat) {
	uint8_t *p = buff + OEM4HLEN;
	double psr, adr, dopp, snr, lockt, dt;
	int i, idx, nobs, prn, lli=0, satcnt=0;
	track_t t;

	nobs = U4(p);
	if (dat->msglen != 4 + nobs * 44) {
		tracef("[WARNINGS]: oem7 rangeb length error: len=%d nobs=%d AT %d %f\n", dat->msglen, nobs, obs->Time.Week, obs->Time.SecOfWeek);
		return -1;
	}

	obs->Time.Week = dat->week;
	obs->Time.SecOfWeek = dat->tow;

	for (i = 0, p += 4; i < nobs; i++, p += 44) {
		if ((idx = decodeTrackStat(U4(p + 40), &t)) < 0) continue;

		//sigtype = GPS(0:L1C,9:L2W), BDS(0,4:L2I,2,6:L6I)
		//P[loc] L[loc] D[loc] cn0[loc] LockTime[loc] half[loc]
		//The value of 'loc' can also be judged by 'code'
		//sys:SYS_GPS(0x01)(1) or SYS_BDS(0x20)(32)
		int loc = 0, obssys = 0, sat; double wl = 0.0;

		if (t.sys == SYS_GPS) {
			obssys = GPS;
			loc = t.sigtp == 0 ? 0 : 1;
			wl = (loc == 0) ? WL1_GPS : WL2_GPS;
		} else if (t.sys == SYS_BDS) {
			obssys = BDS;
			loc = (t.sigtp == 0 || t.sigtp == 4) ? 0 : 1;
			wl = (loc == 0) ? WL1_BDS : WL3_BDS;
		} else;
		prn  = U2(p);
		psr  = R8(p + 4);
		adr  =-R8(p + 16)*wl;//adr = -R8(p + 16);
		dopp =-R4(p + 28);
		snr  = R4(p + 32);//(uint16_t)(snr / SNR_UNIT + 0.5)
		lockt= R4(p + 36);
		sat = t.sys == SYS_GPS ? prn : prn + MAXGPSNUM;

		if (obs->tobs[sat-1][loc].Week != 0) {
			dt = diffTime(&obs->Time, &obs->tobs[sat-1][loc]);
			lli = lockt - obs->LockT[sat-1][loc] + 0.05 <= dt ? LLI_SLIP : 0;
		} else {
			lli = 0;
		}
		/// for debug ///
		if (lli==LLI_SLIP) {
			tracef("[WARNINGS]: LLI_SLIP OCCURS AT %d, %.3f, SYS_TYPE=%d, FREQ_TYPE=%d, PRN=%d\n",dat->week, dat->tow, obssys, loc, prn);
		}
		/// ///   /// ///
		if (!t.parity) lli |= LLI_HALFC;
		if (t.halfc  ) lli |= LLI_HALFA;
		if (!t.clock) { psr = 0.0; tracef("[WARNINGS]: psr-code lock: %d AT %d %f\n", t.clock, dat->week, dat->tow); }          /*  code unlock */
		if (!t.plock) { adr = dopp = 0.0;  tracef("[WARNINGS]: adr-phase lock: %d AT %d %f\n", t.plock, dat->week, dat->tow); } /* phase unlock */

		obs->tobs[sat-1][loc].Week		= obs->Time.Week;
		obs->tobs[sat-1][loc].SecOfWeek = obs->Time.SecOfWeek;
		obs->LockT[sat-1][loc]		= lockt;
		obs->SatObs[sat-1].Sys		= (sys_t)obssys;
		obs->SatObs[sat-1].Prn			= prn;
		obs->SatObs[sat-1].P[loc]		= psr;
		obs->SatObs[sat-1].L[loc]		= adr;
		obs->SatObs[sat-1].D[loc]		= dopp;
		obs->SatObs[sat-1].cn0[loc]		= snr;
		obs->SatObs[sat-1].half[loc]	= t.halfc;
		obs->SatObs[sat-1].LockTime[loc]= lockt;
		obs->SatObs[sat-1].parity[loc]	= t.parity;
		obs->SatObs[sat-1].track[loc]	= t.track;
		obs->SatObs[sat-1].lli[loc]		= lli;
		obs->SatObs[sat-1].Valid		= true;
		obs->SatObs[sat-1].validP[loc]	= true;
		obs->SatObs[sat-1].validL[loc]	= true;
	}

	for (i = 0; i < MAXSYSNUM; i++)
		if (((obs->SatObs) + i)->Sys)
			satcnt++;

	obs->nobs = nobs;
	obs->SatNum = satcnt;

	return 1;
}

/* decode binary gps ephemeris message---------------------------------------------------------- */
int decodeGpsEphem(uint8_t *buff, gpseph_t *eph, dcode_t *dat)
{
	uint8_t *p = buff + OEM4HLEN;
	if (dat->msglen != GPSEPHEMLEN) {
		tracef("[WARNINGS]: OEM7 GPS EPHEMRIS LENGTH ERROR: len=%d AT %d %f\n", dat->msglen, dat->week, dat->tow);
		return -1;
	}
	uint32_t prn = U4(p);		p += 4;
	double tow = R8(p);			p += 8;
	uint32_t health = U4(p);	p += 4;
	uint32_t IODE1 = U4(p);     p += 4;
	uint32_t IODE2 = U4(p);	    p += 4;
	uint32_t week = U4(p);		p += 4;
	uint32_t zweek = U4(p);		p += 4;
	double toe = R8(p);			p += 8;
	double sqrtA = sqrt(R8(p)); p += 8;
	double deln = R8(p);		p += 8;
	double M0 = R8(p);			p += 8;
	double e = R8(p);			p += 8;
	double omg = R8(p);			p += 8;
	double cuc = R8(p);			p += 8;
	double cus = R8(p);			p += 8;
	double crc = R8(p);			p += 8;
	double crs = R8(p);			p += 8;
	double cic = R8(p);			p += 8;
	double cis = R8(p);			p += 8;
	double i0 = R8(p);			p += 8;
	double idot = R8(p);		p += 8;
	double OMG0 = R8(p);		p += 8;
	double omgdot = R8(p);		p += 8;
	uint32_t iodc = U4(p);		p += 4;
	double toc = R8(p);			p += 8;
	double tgd = R8(p);			p += 8;
	double af0 = R8(p);			p += 8;
	double af1 = R8(p);			p += 8;
	double af2 = R8(p);			p += 8;
	/* omit AS */				p += 4;
	double N = R8(p);			p += 8;
	double ura = R8(p);			p += 8;

	eph[prn - 1].Cic = cic;
	eph[prn - 1].Cis = cis;
	eph[prn - 1].Crc = crc;
	eph[prn - 1].Crs = crs;
	eph[prn - 1].Cuc = cuc;
	eph[prn - 1].Cus = cus;
	eph[prn - 1].ClkBias = af0;
	eph[prn - 1].ClkDrift = af1;
	eph[prn - 1].ClkDriftRate = af2;
	eph[prn - 1].DeltaN = deln;
	eph[prn - 1].e = e;
	eph[prn - 1].i0 = i0;
	eph[prn - 1].iDot = idot;
	eph[prn - 1].IODC = iodc;
	eph[prn - 1].IODE1 = IODE1;
	eph[prn - 1].IODE2 = IODE2;
	eph[prn - 1].M0 = M0;
	eph[prn - 1].OMEGA = OMG0;
	eph[prn - 1].omega = omg;
	eph[prn - 1].OMEGADot = omgdot;
	eph[prn - 1].PRN = prn;
	eph[prn - 1].SqrtA = sqrtA;
	eph[prn - 1].SVAccuracy = ura;
	eph[prn - 1].SVHealth = health;
	eph[prn - 1].Sys = GPS;
	eph[prn - 1].TGD = tgd;
	eph[prn - 1].TOC.Week = week;
	eph[prn - 1].TOC.SecOfWeek = toc;
	eph[prn - 1].TOE.Week = week;
	eph[prn - 1].TOE.SecOfWeek = toe;

	return 1;
}

/* decode binary bds ephemeris message---------------------------------------------------------- */
int decodeBdsEphem(uint8_t *buff, bdseph_t *eph, dcode_t *dat)
{
	uint8_t *p = buff + OEM4HLEN;

	if (dat->msglen != BDSEPHEMLEN) {
		tracef("[WARNINGS]: OEM7 BDS EPHEMRIS LENGTH ERROR: len=%d AT %d %f\n", dat->msglen, dat->week, dat->tow);
		return -1;
	}
	int prn = U4(p);			 p += 4;
	uint32_t week = U4(p);		 p += 4;
	double ura = R8(p);			 p += 8;
	uint32_t health = U4(p)&1;   p += 4;  /* 0:good, 1:not good: broadcasting satellite */
	double tgd1 = R8(p);		 p += 8;  /* TGD1 for B1 (s) */
	double tgd2 = R8(p);		 p += 8;  /* TGD2 for B2 (s) */
	uint32_t AODC = U4(p);		 p += 4;  /* AODC */
	uint32_t toc = U4(p);		 p += 4;
	double a0 = R8(p);			 p += 8;
	double a1 = R8(p);			 p += 8;
	double a2 = R8(p);			 p += 8;
	uint32_t AODE = U4(p);		 p += 4;  /* AODE */
	uint32_t toe = U4(p);	  	 p += 4;
	double sqrtA = R8(p);		 p += 8;
	double e = R8(p);			 p += 8;
	double omg = R8(p);			 p += 8;
	double deln = R8(p);	     p += 8;
	double M0 = R8(p);		 	 p += 8;
	double OMG0 = R8(p);		 p += 8;
	double OMGd = R8(p);		 p += 8;
	double i0 = R8(p);			 p += 8;
	double IDOT = R8(p);		 p += 8;
	double cuc = R8(p);			 p += 8;
	double cus = R8(p);			 p += 8;
	double crc = R8(p);			 p += 8;
	double crs = R8(p);			 p += 8;
	double cic = R8(p);		     p += 8;
	double cis = R8(p);			 p += 8;

	// GPST-1356weeks-14sec=BDST
	eph[prn - 1].TOE.Week = week + GPST_BDT_WEEKS;
	eph[prn - 1].TOE.SecOfWeek = toe + GPST_BDT;
	eph[prn - 1].TOC.Week = week + GPST_BDT_WEEKS;
	eph[prn - 1].TOC.SecOfWeek = toc + GPST_BDT;
	eph[prn - 1].TGD1 = tgd1;
	eph[prn - 1].TGD2 = tgd2;
	eph[prn - 1].Sys = BDS;
	eph[prn - 1].SVHealth = health;
	eph[prn - 1].SVAccuracy = ura;
	eph[prn - 1].SqrtA = sqrtA;
	eph[prn - 1].PRN = prn;
	eph[prn - 1].OMEGADot = OMGd;
	eph[prn - 1].omega = omg;
	eph[prn - 1].OMEGA = OMG0;
	eph[prn - 1].M0 = M0;
	eph[prn - 1].IODE = AODE;
	eph[prn - 1].IODC = AODC;
	eph[prn - 1].iDot = IDOT;
	eph[prn - 1].i0 = i0;
	eph[prn - 1].e = e;
	eph[prn - 1].DeltaN = deln;
	eph[prn - 1].Cus = cus;
	eph[prn - 1].Cuc = cuc;
	eph[prn - 1].Crs = crs;
	eph[prn - 1].Crc = crc;
	eph[prn - 1].Cic = cic;
	eph[prn - 1].Cis = cis;
	eph[prn - 1].ClkBias = a0;
	eph[prn - 1].ClkDrift = a1;
	eph[prn - 1].ClkDriftRate = a2;

	return 1;
}

int decodeBestpos(uint8_t *buff, epoch_t *obs)
{
	uint8_t *p = buff + OEM4HLEN + 8;
	double BLH[3];

	BLH[0] = R8(p)*D2R;	p += 8;
	BLH[1] = R8(p)*D2R;	p += 8;
	BLH[2] = R8(p);		p += 8;
	BLH[2] += R4(p);

	blh2xyz(BLH, obs->BestPos.XYZ);
	tracef("BESTPOS: %.4f\t%.4f\t%.4f\t\t%.4f\t%.4f\t%.4f AT %d %f WITH station type: %d\n", obs->BestPos.XYZ[0], obs->BestPos.XYZ[1], obs->BestPos.XYZ[2],
		BLH[0]*R2D, BLH[1]*R2D, BLH[2], obs->Time.Week, obs->Time.SecOfWeek, obs->type);
	return 1;
}