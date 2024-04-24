/*------------------------------------------------------------------------------
* solution.h : RTK software type function definations
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
*			2024 / 03 / 18	 1.7
*			2024 / 04 / 09	 1.8 new
*-----------------------------------------------------------------------------*/
#include "rtklib.h"
#include "matrix.h"
#include "trace.h"

using namespace matrix;

callback_t::callback_t()
{
	Timer = TimerQueue = NULL;
	rawRovOpen = rawBasOpen = false;
	RovTimeouts = BasTimeouts = 0;
	lenRRov = lenDRov = idxRov = 0;
	lenRBas = lenDBas = idxBas = 0;
	sync = 0;
	
	buffRov = new int8_t[MAXRAWLEN];
	BuffRov = new int8_t[MAXRAWLEN * 3];
	buffBas = new int8_t[MAXRAWLEN];
	BuffBas = new int8_t[MAXRAWLEN * 3];

	filelog = { cfg.pathRoverLogRTK };
	filepos = { cfg.pathRoverPosRTK };
	rawRov = { cfg.pathRoverBinRTK };
	rawBas = { cfg.pathBaseBinRTK };
	strcpy(filecmp, filepos);
	strcat(filecmp, ".cmp");

	logRovfp = fopen(filelog, "w");
	posRovfp = fopen(filepos, "w");
	cmpRovfp = fopen(filecmp, "w");

	if (cfg.saveRawBin)	{
		if ((rawRovfp = fopen(rawRov, "wb")) == NULL) { 
			logout("[WARNINGS]: Failed to open binary file %s\n", rawRov);
		} else { 
			rawRovOpen = true; 
		}

		if ((rawBasfp = fopen(rawBas, "wb")) == NULL) {
			logout("[WARNINGS]: Failed to open binary file %s\n", rawBas);
		} else { 
			rawBasOpen = true; 
		}
	}
}

callback_t::~callback_t()
{
	if (rawRovOpen) fclose(rawRovfp);
	if (rawBasOpen) fclose(rawBasfp);
	fclose(logRovfp); fclose(posRovfp); fclose(cmpRovfp);
	delete[] buffRov; 	delete[] BuffRov;
	delete[] buffBas; 	delete[] BuffBas;
}

raw_t::raw_t()
{
	this->BasObs.type = 0;
}

ekf_t::ekf_t() 
{
	int i, j;
	IsInit = false;
	nFreqs = p = q = n = m = 0;
	T = H = v = C = X = P = Q = R = NULL;
	Xlsq[0] = Xlsq[1] = Xlsq[2] = 0.0;

	for (i = 0; i < MAXFREQNUM; i++) {
		Index_[i][0] = Index_[i][1] = Index[i][0] = Index[i][1] = -1;
		AmbFixed[i] = 0;
		AmbFloat[i] = 0.0;
	}
		
	for (i = 0; i < 3 + MAXFREQNUM; i++) {
		X_[i] = 0.0;
		for (j = 0; j < 3 + MAXFREQNUM; j++)
			P_[i*(3 + MAXFREQNUM) + j] = 0.0;
	}
}

void ekf_t::setParam(double *x, double *P, double *Q, double *C, double *R, double *T, double *H, double *v, int p, int q, int n, int m)
{
	this->X = x;
	this->P = P;
	this->Q = Q;
	this->C = C;
	this->R = R;
	this->T = T;
	this->H = H;
	this->v = v;
	this->p = p;
	this->q = q;
	this->n = n;
	this->m = m;
}

void ekf_t::reset()
{
	X = P = Q = C = R = T = H = v = NULL;
	p = q = n = m = 0;
	for (int i = 0; i < MAXFREQNUM; i++) {
		AmbFixed[i] = 0;
		AmbFloat[i] = 0.0;
	}
	memset(this->Index, -1, 2 * MAXFREQNUM * sizeof(int));
}

bool ekf_t::check()
{
	int i;  bool m = true;
	for (i = 0; i < this->nFreqs; i++) m = m&&(this->Index[i][0]==-1);
	return !m;
}

void ekf_t::initP()
{
	int i;
	for(i=0;i<SQR((3+MAXFREQNUM));i++) this->P_[i]=0.0;
	for(i=0;i<3;i++) this->P_[i*this->p+i]=P0_XYZ;	//m^2
	for(i=3;i<this->p;i++) this->P_[i*this->p+i]=P0_AMB; //cycle^2
}

void ekf_t::calcQ(raw_t *raw)
{
	int i, j; double Qp[9], Pp[9], var=0.0;
	double blh[3], enu[3] = { cfg.RcvHoriBias, cfg.RcvHoriBias, cfg.RcvVertBias };

	zero(Qp, 3, 3); zero(Pp, 3, 3); zero(Q, this->q, this->q);

	if (cfg.kinematic) for(i=0;i<3;i++) enu[i]=STD_SPP;
	
	xyz2blh(this->X_, blh);
	for(i=0;i<3;i++) Qp[i*3+i]=SQR(enu[i]);
	covxyz(blh, Qp, Pp);

	for(i=0;i<3;i++)
		for(j=0;j<3;j++)
			this->Q[i*this->q+j]=SQR(Pp[i*3+j]);	//m^2

	for(i=3;i<this->q;i++) this->Q[i*this->q+i] = SQR(cfg.AmbBias); //cycle^2
	matMul_s(diffTime(&this->Time, &this->preSD.Time), this->Q, this->Q, this->q*this->q);
}

void ekf_t::calcT(raw_t *raw)
{
	bool m, _m; int i, i_, _i, ii, ii_, _ii, ff, ff_, _ff;
	sdsat_t *cSD = raw->SDObs.SDSatObs;
	sdsat_t *pSD = this->preSD.SDSatObs;

	zero(T, n, p);

	for (i = 0; i < 3; i++)	T[i*this->p+i] = 1.0;

	for (i = 0; i < this->n-3; i++)
	{
		m = false;
		ii = this->Index[i][0]; // the valid SD index of current epoch 
		ff = this->Index[i][1];

		for (i_ = 0; i_ < this->p-3; i_++)
		{
			ii_ = this->Index_[i_][0]; // the valid SD index of previous epoch 
			ff_ = this->Index_[i_][1];

			if (ff==ff_ && cSD[ii].prn==pSD[ii_].prn && cSD[ii].sys==pSD[ii_].sys)
			{ 
				m = true; // the sat matches the sys/freq from the previous epoch

				if (raw->DDObs.FreqRefPrn[(int)cSD[ii].sys-1][ff]==this->preDD.FreqRefPrn[(int)pSD[ii_].sys-1][ff_])
				{ // the ref-sat of this sat remains unchanged
					T[(3+i)*this->p+3+i_] = 1.0;
					break;
				}
				else // the ref-sat of this sat changes
				{
					_m = false;
					// try to find whether the valid SD of this sat from previous epoch exists, if does, then get the pos
					for (_i = 0; _i < this->p-3; _i++)
					{
						_ii = this->Index_[_i][0];
						_ff = this->Index_[_i][1];

						if (_ff==ff && pSD[_ii].prn== raw->DDObs.FreqRefPrn[(int)cSD[ii].sys-1][ff] && pSD[_ii].sys==cSD[ii].sys)
						{
							_m = true; // find the sys/freq of this sat from the previous epoch
							T[(3+i)*this->p+3+i_] = 1.0;
							T[(3+i)*this->p+3+_i] =-1.0;
							break;
						}
					}

					if (!_m)
					{
						this->AmbFixed[i] = -1;
					}

					break;
				}
			}
			else if (this->preDD.FreqRefPrn[(int)pSD[ii_].sys-1][ff_]==cSD[ii].prn && pSD[ii_].prn==raw->DDObs.FreqRefPrn[(int)cSD[ii].sys-1][ff] && ff == ff_)
			{ // Nij->Nji: ref-sat exchanges between current and previous epoch(i->last ref-sat，j->current ref-sat)
				T[(3+i)*this->p+3+i_] = -1;
				break;
			}
			else;
		}

		if (!m) // the sat newly exists in current epoch
		{
			this->AmbFixed[i] = -1;
		}
	}
}

void ekf_t::calcC()
{
	memcpy(this->C, this->T, sizeof(double)*n*p);
}

void ekf_t::calcH(raw_t *raw)
{
	int i, j, k, sdIdx, sys, num, row, col, colGps, colBds, numGps;	double RovDist[MAXCHANNUM];	sdsat_t *sd = raw->SDObs.SDSatObs;

	zero(this->H, this->m, this->n);
	num = raw->DDObs.nFreq;
	numGps = raw->DDObs.nFreqSat[0][0] + raw->DDObs.nFreqSat[0][1];
	row = 0, col = 3, colGps = 3, colBds = 3 + numGps;

	for (i = 0; i < raw->SDObs.SatNum; i++) RovDist[i] = geodist(raw->RovObs.SatPVT[sd[i].nRov].SatPos, this->X);

	for (i = 0; i < num; i++)
	{
		sdIdx = this->Index[i][0];
		j = this->Index[i][1];
		sys = (int)raw->SDObs.SDSatObs[sdIdx].sys - 1;

		if (sys == 0) col = colGps;	else col = colBds;

		/* Matrix H */
		// Pseudorange
		for (k=0; k<3; k++) { //k=0,1,2 -> SatPos[0],[1],[2], x,y,z
			H[row*(3+num)+k] = (this->X[k] - (raw)->RovObs.SatPVT[sd[sdIdx].nRov].SatPos[k]) / RovDist[sdIdx] -
				(this->X[k] - (raw)->RovObs.SatPVT[sd[(raw)->DDObs.FreqRefIdx[sys][j]].nRov].SatPos[k]) / RovDist[(raw)->DDObs.FreqRefIdx[sys][j]];
		}
		row++;
		// Carrier Phase
		for (k=0; k<3; k++) { //k=0,1,2 -> SatPos[0],[1],[2], x,y,z
			H[row*(3+num)+k] = (this->X[k] - (raw)->RovObs.SatPVT[sd[sdIdx].nRov].SatPos[k]) / RovDist[sdIdx] -
				(this->X[k] - (raw)->RovObs.SatPVT[sd[(raw)->DDObs.FreqRefIdx[sys][j]].nRov].SatPos[k]) / RovDist[(raw)->DDObs.FreqRefIdx[sys][j]];
		}
		H[row*(3+num)+(col++)] = (j == 0) ? ((sys == 0) ? WL1_GPS : WL1_BDS) : ((sys == 0) ? WL2_GPS : WL3_BDS);
		row++;
		if (sys == 0) colGps++; else colBds++;
	}
}

void ekf_t::calcV(raw_t *raw)
{
	int i, j, idx, sys, row; double *basDist, *rovDist; sdsat_t *sd;

	sd = raw->SDObs.SDSatObs;
	basDist = raw->DDObs.Bdist;
	rovDist = new double[raw->SDObs.SatNum];

	for (i=0; i<raw->SDObs.SatNum; i++) rovDist[i] = geodist(raw->RovObs.SatPVT[sd[i].nRov].SatPos, this->X);

	for (i=0, row=0; i<this->nFreqs; i++)
	{
		idx = this->Index[i][0];
		sys = sd[idx].sys - 1;
		j = this->Index[i][1];

		this->v[row] = sd[idx].dP[j] - sd[raw->DDObs.FreqRefIdx[sys][j]].dP[j] - (rovDist[idx] - rovDist[raw->DDObs.FreqRefIdx[sys][j]] - basDist[idx] + basDist[raw->DDObs.FreqRefIdx[sys][j]]);
		row++;
		this->v[row] = sd[idx].dL[j] - sd[raw->DDObs.FreqRefIdx[sys][j]].dL[j] - (rovDist[idx] - rovDist[raw->DDObs.FreqRefIdx[sys][j]] - basDist[idx] + basDist[raw->DDObs.FreqRefIdx[sys][j]]);
		this->v[row] -= (j==0) ? ((sys==0)?WL1_GPS*X[3+i]:WL1_BDS*X[3+i]) : ((sys==0)?WL2_GPS*X[3+i]:WL3_BDS*X[3+i]);
		row++;
	}

	delete[] rovDist;
}

void ekf_t::calcR(raw_t *raw)
{
	int i, k, idx, sys, sta, end, numGps, num, row;

	zero(this->R, this->m, this->m);
	row = 0;
	num = raw->DDObs.nFreq;
	numGps = raw->DDObs.nFreqSat[0][0] + raw->DDObs.nFreqSat[0][1];

	for (i = 0; i < num; i++)
	{
		idx = this->Index[i][0];
		sys = (int)raw->SDObs.SDSatObs[idx].sys - 1;

		if (sys == 0) {
			sta = 0;		end = numGps;		// col: a param that assists the construction of 'B';
		} else {								// sta/end: params that assist the construction of 'P'
			sta = numGps;	end = num;
		}

		/* Matrix R */
		// Pseudorange
		for (k=sta; k<end; k++)	{

			if (this->Index[i][1]==this->Index[k][1])
				this->R[row*num*2+k*2] = 2.0*SQR(cfg.CodeNoise);

			if(k==i)
				this->R[row*num*2+k*2] *= 2;
		}
		row++;
		// Carrier Phase
		for (k=sta; k<end; k++)	{

			if (this->Index[i][1]==this->Index[k][1])
				this->R[row*num*2+k*2+1] = 2.0*SQR(cfg.CPNoise);

			if(k==i)
				this->R[row*num*2+k*2+1] *= 2;
		}
		row++;
	}
}

void ekf_t::reInit()
{
	for (int i = 3; i < this->n; i++)
	{
		if (this->AmbFixed[i-3]==-1)
		{
			this->P[i*this->n+i]=100.0;
			this->X[i]=AmbFloat[i-3];
		}
	}
}

int ekf_t::ekf(raw_t *raw)
{
	if (!this->check()) return -1;

	this->calcQ(raw);
	this->calcT(raw);
	this->calcC();

	if (filter(this->X, this->P, this->Q, this->T, this->C, this->p, this->q, this->n) < 0)
		return -1;
	else {
		this->reInit();
		this->calcH(raw);
		this->calcV(raw);
		this->calcR(raw);
		return filter(this->X, this->P, this->H, this->v, this->R, this->n, this->m);
	}
}

lsq_t::lsq_t() 
{
	B = P = w = Q = m = X = v = NULL;
	rowB = colB = sizeP = sizeW = sizeQ = sizeM = sizeX = sizeV = 0;
	mse = 0.0;
}

void lsq_t::setParam(double *B, double *P, double *w, double *Q, double *m, int rowB, int colB, double *X)
{
	this->B = B;
	this->P = P;
	this->w = w;
	this->Q = Q;
	this->m = m;
	this->X = X;
	this->mse = mse;
	this->rowB = rowB;
	this->colB = colB;
	this->sizeM = 3;
	this->sizeP = this->rowB;
	this->sizeW = this->rowB;
	this->sizeV = this->rowB;
	this->sizeQ = this->colB;
	this->sizeX = this->colB;
}

void lsq_t::reset()
{
	this->B = NULL;
	this->P = NULL;
	this->w = NULL;
	this->Q = NULL;
	this->m = NULL;
	this->X = NULL;
	this->v = NULL;
	this->mse = 0.0;
	this->rowB = 0;
	this->colB = 0;
	this->sizeM = 3;
	this->sizeP = 0;
	this->sizeW = 0;
	this->sizeQ = 0;
	this->sizeX = 0;
	this->sizeV = 0;
}

double lsq_t::lsq()
{
	lsq_t *p = this;

	double res = 0.0;
	double *BT   = new double [p->colB*p->rowB];
	double *BTP  = new double [p->colB*p->sizeP];
	double *BTPB = new double [p->colB*p->colB];
	double *BTPw = new double [p->colB*1];
	double *x	 = new double [p->colB*1];
	double *v	 = new double [p->rowB*1];
	double *Bx	 = new double [p->rowB*1];
	double *vTP  = new double [1*p->sizeP];

	matTran(p->B, p->rowB, p->colB, BT);
	matMul(BT, p->colB, p->rowB, p->P, p->sizeP, p->sizeP, BTP);
	matMul(BTP, p->colB, p->sizeP, p->B, p->rowB, p->colB, BTPB);
	matMul(BTP, p->colB, p->sizeP, p->w, p->rowB, 1, BTPw);
	matinv(BTPB, p->colB);
	matMul(BTPB, p->colB, p->colB, BTPw, p->colB, 1, x);
	matMul(p->B, p->rowB, p->colB, x, p->colB, 1, Bx);
	matSub(Bx, p->w, v, p->rowB, 1);
	matMul(v, 1, p->rowB, p->P, p->sizeP, p->sizeP, vTP);
	matMul(vTP, 1, p->sizeP, v, p->rowB, 1, &p->mse);

	if(p->v!=NULL) matcpy(p->v, v, p->rowB);
	p->mse /= p->rowB - p->colB;
	p->mse = SQRT(p->mse);
	for(int i=0;i<3;i++) p->m[i] = p->mse*SQRT(BTPB[i*(p->sizeQ+1)]);
	matcpy(p->Q, BTPB, SQR(p->colB));
	matAdd(p->X, x, p->X, p->sizeX, 1);
	res = norm(x, 3);

	delete[]BT; delete[]BTP; delete[]BTPB; delete[]BTPw; 
	delete[]x;	delete[]v;	 delete[]Bx;   delete[]vTP;
	return res;
}

ddobs_t::ddobs_t()
{
	int i;
	for (i = 0; i < 2; i++) {
		nSysSat[i] = 0;
		RefIdx[i] = RefPrn[i] = -1;
	}
	nSat = nFreq = 0;
	Ratio = 1.0;
	dPos[0] = dPos[1] = dPos[2] = 0.0;
	ResAmb[0] = ResAmb[1] = FixRMS[0] = FixRMS[1] = 0.0;
	bFixed = -1;

	memset(FreqRefPrn, -1, sizeof(int) * 4);
	memset(FreqRefIdx, -1, sizeof(int) * 4);
	memset(nFreqSat, 0, sizeof(int) * 4);

	for (i = 0; i < 4*MAXCHANNUM; i++) FixedAmb[i] = 0.0;

	X = Q = Rdist = Bdist = NULL;	// rtkFloat
	SDlist = type = NULL;
}

void ddobs_t::reset()
{
	int i;
	for (i = 0; i < 2; i++) {
		nSysSat[i] = 0;
		RefIdx[i] = RefPrn[i] = -1;
	}
	nSat = nFreq = 0;
	Ratio = 1.0;
	dPos[0] = dPos[1] = dPos[2] = 0.0;
	ResAmb[0] = ResAmb[1] = FixRMS[0] = FixRMS[1] = 0.0;
	bFixed = -1;

	memset(FreqRefPrn, -1, sizeof(int) * 4);
	memset(FreqRefIdx, -1, sizeof(int) * 4);
	memset(nFreqSat, 0, sizeof(int) * 4);

	for (i = 0; i < 4 * MAXCHANNUM; i++) FixedAmb[i] = 0.0;

	X = Q = Rdist = Bdist = NULL;	// rtkFloat
	SDlist = type = NULL;
}

sdobs_t::sdobs_t()
{
	SatNum = 0;
}
void sdobs_t::reset()
{
	SatNum = 0;
	memset(&Time, 0, sizeof(gtime_t));
	memset(SDSatObs, 0, sizeof(sdsat_t)*MAXCHANNUM);
}

bool sdobs_t::checkFullFreq(int idx)
{
	return (SDSatObs[idx].ValidL[0]) && (SDSatObs[idx].ValidL[1])
		&& (SDSatObs[idx].ValidP[0]) && (SDSatObs[idx].ValidP[1]);
}

bool sdobs_t::checkFreq(int idx, int freq)
{
	return (SDSatObs[idx].ValidL[freq] && SDSatObs[idx].ValidP[freq] && !SDSatObs[idx].halfc[freq]);
}

sdsat_t::sdsat_t()
{
	prn = nBas = nRov = 0;
	sys = UNKS;
	dP[0] = dL[0] = dP[1] = dL[1] = 0.0;
	ValidP[0] = ValidP[1] = 0;
	ValidL[0] = ValidL[1] = 0;
	halfc [0] = halfc [1] = 0;
}

bool sdsat_t::flfreqchk()
{
	return this->ValidL[0]&&this->ValidL[1]&&this->ValidP[0]&&this->ValidP[1]&&(!this->halfc[0])&&(!this->halfc[1]);
}

pos_t::pos_t()
{
	Pos[0] = Pos[1] = Pos[2] = 0.0;
	Vel[0] = Vel[1] = Vel[2] = 0.0;
	for (int i = 0; i < 36; i++) { P[i] = 99.9; Q[i] = 999.9; }
	RcvClkOft[0] = RcvClkOft[1] = RcvClkSft = 0.0;
	PDOP = SigmaPos = SigmaVel = 999.9;
	GPSSatNum = BDSSatNum = AllSatNum = 0;
	nn = 5;
	IsSuccess = false;
	ISFIRST = true;
	//if(USE_FILTER){ NecessaryNum = 6;}
}

refpos_t::refpos_t()
{
	XYZ[0] = cfg.basRefPos[0];
	XYZ[1] = cfg.basRefPos[1];
	XYZ[2] = cfg.basRefPos[2];
	memset(ENU, 0, sizeof(double) * 3);
}

void pos_t::reset() {
	//Pos[0] = Pos[1] = Pos[2] = 0.0;
	//Vel[0] = Vel[1] = Vel[2] = 0.0;
	//RcvClkOft[0] = RcvClkOft[1] = RcvClkSft = 0.0;
	PDOP = SigmaPos = SigmaVel = 999.9;
	GPSSatNum = BDSSatNum = AllSatNum = 0;
	nn = 5;
	IsSuccess = false;
	//if(USE_FILTER){ NecessaryNum = 6;}
}

epoch_t::epoch_t()
{
	type = 1;
	SatNum = nobs = 0;
	getBestPos = false;
	for (int i = 0; i < MAXSYSNUM; i++) {
		for (int j = 0; j < 2; j++) {
			LockT[i][j] = 0.0;
		}
	}
}
void epoch_t::reset()
{
	SatNum = nobs = 0;
	memset(&Time, 0, sizeof(gtime_t));
	memset(SatObs, 0, sizeof(satobs_t)*MAXSYSNUM);
	memset(SatPVT, 0, sizeof(satres_t)*MAXSYSNUM);
}

satres_t::satres_t()
{
	prn = 0; Sys = UNKS;
	SatPos[0] = SatPos[1] = SatPos[2] = 0.0;
	SatVel[0] = SatVel[1] = SatVel[2] = 0.0;
	Elevation = PI / 2.0;
	Azimuth = 0.0;
	SatClkOft = SatClkSft = 0.0;
	Tgd1 = Tgd2 = TropCorr = 0.0;
	PIF = LIF = 0.0;
	Valid = false;
}

mwgf_t::mwgf_t()
{
	Prn = n = 0;
	Sys = UNKS;
	LMW = stdLMW = PGF = LGF = PIF = LIF = 0.0;
}
void mwgf_t::reset() {
	Prn = n = 0;
	Sys = UNKS;
	LMW = stdLMW = PGF = LGF = PIF = LIF = 0.0;
}

satobs_t::satobs_t()
{
	Prn = 0;
	Sys = UNKS;
	parity[0] = parity[1] = 0;
	track[0] = track[1] = 0;
	lli[0] = lli[1] = 0;
	half[0] = half[1] = 0;
	LockTime[0] = LockTime[1] = 0.0;
	cn0[0] = cn0[1] = 0.0;
	P[0] = P[1] = L[0] = L[1] = D[0] = D[1] = 0.0;
	Valid = false;
	validP[0] = validP[1] = false;
	validL[0] = validL[1] = false;
}

bdseph_t::bdseph_t()
{
	PRN = SVHealth = 0;
	Sys = UNKS;
	ClkBias = ClkDrift = ClkDriftRate = IODE = IODC = TGD1 = TGD2 = 0.0;
	SqrtA = e = M0 = OMEGA = i0 = omega = OMEGADot = iDot = DeltaN = 0.0;
	Crs = Cuc = Cus = Cic = Cis = Crc = SVAccuracy = 0.0;
}

gpseph_t::gpseph_t()
{
	PRN = SVHealth = 0;
	Sys = UNKS;
	ClkBias = ClkDrift = ClkDriftRate = IODE1 = IODE2 = IODC = TGD = 0.0;
	SqrtA = e = M0 = OMEGA = i0 = omega = OMEGADot = iDot = DeltaN = 0.0;
	Crs = Cuc = Cus = Cic = Cis = Crc = SVAccuracy = 0.0;
}

gtime_t::gtime_t()
{
	Week = 0;
	SecOfWeek = 0.0;
}
gtime_t::gtime_t(uint16_t w, double s)
{
	Week = w;
	SecOfWeek = s;
}

gtime_t::gtime_t(gtime_t *t)
{
	this->Week = t->Week;
	this->SecOfWeek = t->SecOfWeek;
}

utime_t::utime_t()
{
	Year = 0;
	Month = 0;
	Day = 0;
	Hour = 0;
	Minute = 0;
	Second = 0.0;
}
utime_t::utime_t(short y, uint16_t mon, uint16_t d, uint16_t h, uint16_t min, double s)
{
	Year = y;
	Month = mon;
	Day = d;
	Hour = h;
	Minute = min;
	Second = s;
}

mjdtime_t::mjdtime_t()
{
	Days = 0;
	FracDay = 0.0;
}
mjdtime_t::mjdtime_t(int d, double f)
{
	Days = d;
	FracDay = f;
}