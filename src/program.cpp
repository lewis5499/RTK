/*------------------------------------------------------------------------------
* program.cpp : RTK software basic function entries
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
#include "confl.h"
#include "rtklib.h"
#include "trace.h"

FILE *traceLogfile = NULL;
conf_t cfg;
char rest[3] = { 0,1,2 }; // 0=Lat/Lon/Height, 1=X/Y/Z-ECEF, 2=E/N/U-Baseline
extern ekf_t ekfParam;

/* start MENU ---------------------------------------------------------------------------- */
void menu()
{
	if (loadConf(PATH_CFGDIR, cfg) < 0) return;

	TRACE_INIT;

	switch (cfg.dataMode) {
		case MODE_FILE: fileMode(cfg.pathRoverBin, cfg.pathRoverLog, cfg.pathRoverPos, cfg.pathBaseBin); break;
		case MODE_SOCKET: socketMode(); break;
		default: break;
	}

	TRACE_FREE;

	system("pause");
}

/* load configurations ------------------------------------------------------------------- */
int loadConf(const char *cfgfilepath, conf_t& rtkcfg)
{
	config_t *myconf = confRead(cfgfilepath);

	if (!myconf) return -1;

	rtkcfg.dataMode			= confGetInt16(myconf, "dataMode");
	rtkcfg.rtkFilter		= confGetInt16(myconf, "rtkFilter");
	rtkcfg.sppFilter		= confGetInt16(myconf, "sppFilter");
	rtkcfg.saveRawBin		= confGetInt16(myconf, "saveRawBin");
	rtkcfg.saveRTKResToFile = confGetInt16(myconf, "saveRTKResToFile");
	rtkcfg.kinematic		= confGetInt16(myconf, "kinematic");
	rtkcfg.fullfreqs		= confGetInt16(myconf, "fullfreqs");
	rtkcfg.enableTrace      = confGetInt16(myconf, "enableTrace");
	rtkcfg.enableLog		= confGetInt16(myconf, "enableLog");
	rtkcfg.ratioThreshold	= confGetFloat(myconf, "ratioThreshold");
	rtkcfg.elevaThreshold	= confGetFloat(myconf, "elevaThreshold");
	rtkcfg.hopfdH0			= confGetFloat(myconf, "hopfdH0");
	rtkcfg.hopfdT0			= confGetFloat(myconf, "hopfdT0");
	rtkcfg.hopfdp0			= confGetFloat(myconf, "hopfdp0");
	rtkcfg.hopfdRH0			= confGetFloat(myconf, "hopfdRH0");
	rtkcfg.CodeNoise		= confGetFloat(myconf, "CodeNoise");
	rtkcfg.CPNoise			= confGetFloat(myconf, "CPNoise");
	rtkcfg.RcvHoriBias	    = confGetFloat(myconf, "RcvHorizonalBias");
	rtkcfg.RcvVertBias      = confGetFloat(myconf, "RcvVerticalBias");
	rtkcfg.RcvClkBias		= confGetFloat(myconf, "ReceiverClkBias");
	rtkcfg.RcvClkDft		= confGetFloat(myconf, "ReceiverClkDrift");
	rtkcfg.AmbBias			= confGetFloat(myconf, "AmbiguityBias");
	rtkcfg.basRefPos[0]		= confGetDouble(myconf, "basRefPosX");
	rtkcfg.basRefPos[1]		= confGetDouble(myconf, "basRefPosY");
	rtkcfg.basRefPos[2]		= confGetDouble(myconf, "basRefPosZ");
	rtkcfg.rovRefPos[0]		= confGetDouble(myconf, "rovRefPosX");
	rtkcfg.rovRefPos[1]		= confGetDouble(myconf, "rovRefPosY");
	rtkcfg.rovRefPos[2]		= confGetDouble(myconf, "rovRefPosZ");
	strcpy(rtkcfg.pathRoverLog,    confGet(myconf, "pathRoverLogf"));
	strcpy(rtkcfg.pathRoverPos,    confGet(myconf, "pathRoverPosf"));
	strcpy(rtkcfg.pathBaseBinRTK,  confGet(myconf, "pathBaseBinRTK"));
	strcpy(rtkcfg.pathRoverBinRTK, confGet(myconf, "pathRoverBinRTK"));
	strcpy(rtkcfg.pathRoverLogRTK, confGet(myconf, "pathRoverLogRTK"));
	strcpy(rtkcfg.pathRoverPosRTK, confGet(myconf, "pathRoverPosRTK"));
	strcpy(rtkcfg.traceLogPath,    confGet(myconf, "traceLogPath"));

	if (rtkcfg.dataMode == 0) {
		strcpy(rtkcfg.pathBaseBin, confGet(myconf, "pathBaseBinf"));
		strcpy(rtkcfg.pathRoverBin, confGet(myconf, "pathRoverBinf"));
	} else {
		strcpy(rtkcfg.ipBase, confGet(myconf, "ipBase"));
		strcpy(rtkcfg.ipRover, confGet(myconf, "ipRover"));
		rtkcfg.portBase = confGetInt16(myconf, "portBase");
		rtkcfg.portRover = confGetInt16(myconf, "portRover");
	}

	confFree(myconf);

	return 0;
}

/* start file program ---------------------------------------------------------------------*/
int fileMode(const char *fileRov, const char *fileLog, const char *filePos, const char *fileBas)
{
	if (access(PATH_LOGDIR, 0)==-1) { 
		if (mkdir(PATH_LOGDIR)) { 
			logout("[WARNINGS]: UNABLE TO CREATE DIRECTORY.\n"); 
		}
	}
	
	int sync = 0; char poscmp[256];
	FILE *fpRov, *fpBas, *fpLog, *fpPos, *fpCmp;
	raw_t raw; pos_t posBas, posRov; 

	strcpy(poscmp, filePos);
	strcat(poscmp, ".cmp");
	fpBas = fopen(fileBas, "rb");	//base
	fpRov = fopen(fileRov, "rb");	//rover
	fpLog = fopen(fileLog, "w");	//rover
	fpPos = fopen(filePos, "w");	//rover
	fpCmp = fopen(poscmp, "w");		//rover

	if (!fpBas||!fpRov) {
		logout("[WARNINGS]: CANNOT OPEN OEM7 FILE!\n");
		return -1; 
	}
	if (!fpLog||!fpPos||!fpCmp) {
		logout("[WARNINGS]: CANNOT CREATE ROVER's log/pos/pos.cmp FILE!\n");
		return -1; 
	}

	fprintf(fpPos, "%s\n", HEADER_POSFILE);
	printf("%s", HEADER);
	printf("[INFO]: RTK BASE  STATION: %s\n", fileBas);
	printf("[INFO]: RTK ROVER STATION: %s\n", fileRov);

	do {
		sync = rtkSyncf(fpRov, fpBas, &raw);
		/// for debug ///
		if (sync == 0) {
			logout("[LOG]: RTK NOT SYNCED AT %d, %.3f\n", raw.RovObs.Time.Week, raw.RovObs.Time.SecOfWeek);
		}
		/// ///   /// ///
		if (sync > 0) { //-2:EOF, 0: NOT SYNCED, 1 SYNCED		
			//step1: Base&Rover: SatPVT, SPP, SPV
			sppMain(&raw.BasObs, raw.ephgps, raw.ephbds, &posBas);
			sppMain(&raw.RovObs, raw.ephgps, raw.ephbds, &posRov);

			//step2: RTK algorithm
			rtkMain(&raw, &posBas, &posRov);

			//step3: Rover: RTK result outputs
			outputPOSf(fpCmp, &posRov, &raw, rest[0]);
			outputOEM7f(fpLog, fpPos, &posRov, &raw);// Output SATpos/SPPpos to log/final(pos) file.
			//outputOEM7(&raw.RovObs, &posRov, &raw);
		}
	} while (sync != -2);//-2:EOF

	fclose(fpBas); fclose(fpRov); fclose(fpLog); fclose(fpPos); fclose(fpCmp);
	return 0;
}

/* open file, start decode and return observations ----------------------------------------*/
int inputOEM7f(FILE *fp, epoch_t &obs, gpseph_t *ephgps, bdseph_t *ephbds)
{
	uint8_t	buff[MAXMSGLEN];
	int data, idx;

	while (1) {
		memset(buff, 0, sizeof(uint8_t)*MAXMSGLEN);
		/* synchronize frame */
		for (int i = 0;; i++) {
			if ((data = fgetc(fp)) == EOF) {
				printf("%s: EOF\n", (!obs.type) ? "[BASE]" : "[ROVER]");
				return -2; //EOF
			}
			if (syncOEM7(buff, (uint8_t)data)) break;
		}
		idx = decodeOEM7f(fp, buff, &obs, ephgps, ephbds);
		if (!idx) return idx;
	}
}

/* input socket Buff data, start to decode ------------------------------------------------*/
int inputOEM7(int8_t *Buff, int &lenD, epoch_t *obs, gpseph_t *ephgps, bdseph_t *ephbds)
{
	uint8_t msgBuff[MAXMSGLEN];
	int data, i, idx; bool sync;
	
	while (1) {
		sync = false;	idx = 2;
		/* synchronize frame */
		for (i = 0; i < lenD; i++) {
			data = I1(Buff + i);
			if (syncOEM7(msgBuff, (uint8_t)data)) { 
				sync = true; 
				break; //msgBuff[0]~[2]:'AA4412', (Buff+i-2)begins with 'AA4412'
			}
		}
		if (!sync) break;

		idx = decodeOEM7(Buff, msgBuff, obs, ephgps, ephbds, i, lenD); //0=obs, >0=continue, -2=buff EOF
		memset(msgBuff, 0, sizeof(uint8_t)*MAXMSGLEN);

		if (idx < 0) break; //'Buff': insufficient length! [-2=buff EOF]
		if (!idx) return idx;  
	}

	return -2;
}

/* print spp/spv and satellite's pvt results to file --------------------------------------*/
int outputOEM7f(FILE *fpLog, FILE *fpPos, pos_t *pos, raw_t *raw)
{
	if (!pos->IsSuccess) return -1;

	int i = 0;

	printPosLog(pos, fpLog);
	while (i++ < raw->RovObs.SatNum) 
		if (raw->RovObs.SatPVT[i-1].Valid) printSatLog(&(raw->RovObs.SatPVT[i-1]), fpLog);
	printPos(fpPos, raw, pos);

	return 0;
}

int outputPOSf(FILE *fp, pos_t *pos, raw_t *raw, char type) // match 'rtkplot.exe'
{	
	//if (!pos->IsSuccess) return -1;
	if (posCheck(&raw->RovObs, pos, raw) < 0) return -1;

	int8_t Q; double BLH[3], dENU[3];

	Q = (int8_t)raw->DDObs.bFixed; //Q=1:fix,2:float,4:dgps,5:single
	Q = (Q==-1)?5:((Q==0)?2:1);
	xyz2blh(pos->Pos, BLH);
	xyz2enu(pos->Pos, raw->BasObs.BestPos.XYZ, dENU);
	
	fprintf(fp, "%4d %.3f ", pos->Time.Week, pos->Time.SecOfWeek);
	if (type==rest[0]) {  // 0=Lat/Lon/Height, 1=X/Y/Z-ECEF, 2=E/N/U-Baseline
		fprintf(fp, "%20.9f %20.9f %15.5f ", BLH[0] * R2D, BLH[1] * R2D, BLH[2]);
	} else if (type==rest[1]) {
		fprintf(fp, "%20.9f %20.9f %20.9f ", pos->Pos[0], pos->Pos[1], pos->Pos[2]);
	} else if (type==rest[2]) {
		fprintf(fp, "%15.5f %15.5f %15.5f ", dENU[0], dENU[1], dENU[2]);
	}
	else;
	fprintf(fp, "%3d %5d ", Q, raw->DDObs.nFreq);
	fprintf(fp, "%3f %3f %3f %3f %3f %3f %3f ", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0); // std: no value
	fprintf(fp, "%7.1f\n", raw->DDObs.Ratio);

	return 0;
}

/* output the SOCKET spp/spv/rtk results ---------------------------------------------------*/
int outputOEM7(epoch_t *obs, pos_t *pos, raw_t* raw)
{
	if (!pos->IsSuccess) return -1;

	double BLH[3], ENU[3];

	xyz2blh(pos->Pos, BLH);
	xyz2enu(pos->Pos, obs->BestPos.XYZ, ENU);
	printf("%4d %.3f %13.3f %13.3f %13.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %10.3f %10.3f %10.3f %7.1f %4d %7.3f %3d %3d %3d\n",
			pos->Time.Week, pos->Time.SecOfWeek, pos->Pos[0], pos->Pos[1], pos->Pos[2],
			BLH[0] * R2D, BLH[1] * R2D, BLH[2], ENU[0], ENU[1], ENU[2], raw->DDObs.dPos[0], raw->DDObs.dPos[1], raw->DDObs.dPos[2],
			(raw->DDObs.Ratio)>1000.0?999.9:raw->DDObs.Ratio, raw->DDObs.bFixed, pos->PDOP, raw->DDObs.nFreqSat[0][0]+raw->DDObs.nFreqSat[0][1],
			raw->DDObs.nFreqSat[1][0]+raw->DDObs.nFreqSat[1][1], raw->DDObs.nFreq);
	return 0;
}

/* output the FILE SatPVT results ---------------------------------------------------------*/
void printSatLog(satres_t *sat, FILE* fp)
{
	string PRN = "";

	PRN += (sat->Sys == GPS) ? "G" : "C";
	PRN += (sat->prn < 10) ? ("0" + to_string(sat->prn)) : to_string(sat->prn);

	fprintf(fp, "%s X=%13.3f Y=%13.3f Z=%13.3f Clk=%13.6e Vx=%13.4f Vy=%13.4f Vz=%13.4f Clkd=%13.6e PIF=%13.4f Trop=%7.3f E=%7.3fdeg\n",
			PRN.c_str(), sat->SatPos[0], sat->SatPos[1], sat->SatPos[2], sat->SatClkOft, 
			sat->SatVel[0], sat->SatVel[1], sat->SatVel[2], sat->SatClkSft, sat->PIF, sat->TropCorr, (sat->Elevation)*R2D);
}

/* output the POS results to LOG FILE -----------------------------------------------------*/
void printPosLog(pos_t *pos, FILE *fp)
{
	double blh[3];

	xyz2blh(pos->Pos, blh);
	fprintf(fp, "> %d %8.3f Sats:%2d GPSSats:%2d BDSSats:%2d \nRTK X:%13.3f Y:%13.3f Z:%13.3f B:%7.3f L:%7.3f H:%7.3f gpsClk:%7.3f bdsClk:%7.3f PDOP:%7.3f SigmaPos:%7.3f\n",
			pos->Time.Week, pos->Time.SecOfWeek, pos->AllSatNum, pos->GPSSatNum, pos->BDSSatNum, pos->Pos[0], pos->Pos[1], pos->Pos[2], blh[0] * R2D, blh[1] * R2D, blh[2],
			pos->RcvClkOft[0], pos->RcvClkOft[1], pos->PDOP, pos->SigmaPos);
}

/* output the RTK pos/vel to POS FILE -----------------------------------------------------*/
void printPos(FILE *fp, raw_t *raw, pos_t *pos)
{
	int i = 0; double BLH[3], ENU[3], dENU[3]; string str = "";
	
	xyz2blh(pos->Pos, BLH);
	xyz2enu(pos->Pos, raw->RovObs.BestPos.XYZ, ENU);
	xyz2enu(pos->Pos, raw->BasObs.BestPos.XYZ, dENU); // baseline
	while (i++ < MAXCHANNUM) {
		if (raw->RovObs.SatPVT[i-1].Valid && raw->RovObs.SatObs[i-1].Valid) {
			if (raw->RovObs.SatPVT[i - 1].Sys == GPS) {
				str += "G";
				str += (raw->RovObs.SatPVT[i - 1].prn < 10) ? "0" + to_string(raw->RovObs.SatPVT[i - 1].prn) : to_string(raw->RovObs.SatPVT[i - 1].prn);
			}
			else if (raw->RovObs.SatPVT[i - 1].Sys == BDS) {
				str += "C";
				str += (raw->RovObs.SatPVT[i - 1].prn < 10) ? "0" + to_string(raw->RovObs.SatPVT[i - 1].prn) : to_string(raw->RovObs.SatPVT[i - 1].prn);
			}
			else;
		}
	}
	fprintf(fp, "%4d %.3f %14.4f %14.4f %14.4f %14.4f %14.4f %14.4f %7.3f %7.3f %7.3f %14.8f %14.8f %8.3f %8.3f %8.3f %8.3f %14.5f %14.5f %14.5f %14.5f %14.5f %14.5f %8.3f %8.3f %8.3f  %3d    %3d    %3d  %8.3f  %3d  %7.3f %3d %3d %3d %s\n",
			pos->Time.Week, pos->Time.SecOfWeek, pos->Pos[0], pos->Pos[1], pos->Pos[2], raw->RovObs.BestPos.XYZ[0], raw->RovObs.BestPos.XYZ[1], raw->RovObs.BestPos.XYZ[2],	ENU[0], ENU[1], ENU[2], BLH[0] * R2D, BLH[1] * R2D, BLH[2], 
			pos->Vel[0], pos->Vel[1], pos->Vel[2], raw->DDObs.dPos[0], raw->DDObs.dPos[1], raw->DDObs.dPos[2], dENU[0], dENU[1], dENU[2], pos->PDOP, pos->SigmaPos, pos->SigmaVel, raw->DDObs.nFreqSat[0][0]+raw->DDObs.nFreqSat[0][1],
			raw->DDObs.nFreqSat[1][0]+raw->DDObs.nFreqSat[1][1], raw->DDObs.nFreq, (raw->DDObs.Ratio)>1000.0?999.999:raw->DDObs.Ratio, raw->DDObs.bFixed, raw->DDObs.FixRMS[0], pos->GPSSatNum, pos->BDSSatNum, pos->AllSatNum, str.c_str());
}

int posCheck(epoch_t *obs, pos_t *pos, raw_t *raw)
{
	if (!pos->IsSuccess) { // SPP/RTK failed
		if (cfg.kinematic) return -1;
		else {  // static
			if (cfg.rtkFilter && ekfParam.IsInit) {
				raw->DDObs.bFixed = 0; // set float
				memcpy(pos->Pos, ekfParam.X_, 3 * sizeof(double));
				memcpy(&pos->Time, &ekfParam.Time, sizeof(gtime_t));
				raw->DDObs.Ratio = ekfParam.preDD.Ratio;
			} else if (cfg.rtkFilter && !ekfParam.IsInit) {
				return -1;
			} else; // LSQ:none
		}
	} else if (pos->IsSuccess && raw->DDObs.bFixed == -1 && !cfg.kinematic) { // static & SPP successful, RTK failed
		if (cfg.rtkFilter  && ekfParam.IsInit) { // EKF
			raw->DDObs.bFixed = 0; // set float
			memcpy(pos->Pos, ekfParam.X_, 3 * sizeof(double));
			memcpy(&pos->Time, &ekfParam.Time, sizeof(gtime_t));
			raw->DDObs.Ratio = ekfParam.preDD.Ratio;
		} else if (cfg.rtkFilter && !ekfParam.IsInit) {} //none
		else; // LSQ:none
	}
	else;

	if (raw->DDObs.bFixed == -1 && !cfg.kinematic) return -1; // not output SPP res when in static mode

	return 0;
}
