/*------------------------------------------------------------------------------
* socket.cpp : RTK software socket operations
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

extern char rest[];
static enum TYPE_t { BASE, ROVER } TYPE;


/* Open Socket -------------------------------------------------------------- */
static bool openSocket(SOCKET& sock, const char IP[], const unsigned short Port)
{
	WSADATA wsaData;
	SOCKADDR_IN addrSrv;

	if (!WSAStartup(MAKEWORD(1, 1), &wsaData))
	{
		if ((sock = socket(AF_INET, SOCK_STREAM, 0)) != INVALID_SOCKET)
		{
			addrSrv.sin_addr.S_un.S_addr = inet_addr(IP);
			addrSrv.sin_family = AF_INET;
			addrSrv.sin_port = htons(Port);
			return (connect(sock, (SOCKADDR*)&addrSrv, sizeof(SOCKADDR)) == 0);
		}
	}
	return false;
}

/* Close Socket ------------------------------------------------------------- */
static void closeSocket(SOCKET& sock)
{
	closesocket(sock);
	WSACleanup();
}

static void checkstat(int *stat, fd_set *set, callback_t *p) //[0]=base, [1]=rover
{
	timeval timeout;

	FD_ZERO(&set[BASE]);
	FD_SET(p->NetBas, &set[BASE]);
	FD_ZERO(&set[ROVER]);
	FD_SET(p->NetRov, &set[ROVER]);

	timeout.tv_sec = 0;      // Unit: microseconds
	timeout.tv_usec = 50000; // Adjust the timeout as needed (unit: milliseconds)

	stat[BASE]  = select((int)(p->NetBas) + 1, &set[BASE],  NULL, NULL, &timeout);
	stat[ROVER] = select((int)(p->NetRov) + 1, &set[ROVER], NULL, NULL, &timeout);
}

static void rtksol(callback_t *p)
{
	do {
		p->sync = rtkSync(p);

		if (p->sync == 0) { // 0: NOT SYNCED
			logout("[LOG]: RTK NOT SYNCED AT %d, %.3f\n", p->raw.RovObs.Time.Week, p->raw.RovObs.Time.SecOfWeek);
		}

		if (p->sync > 0) {  // 1 SYNCED		
			//step1: Base&Rover: SatPVT, SPP, SPV
			sppMain(&p->raw.BasObs, p->raw.ephgps, p->raw.ephbds, &p->posBas);
			sppMain(&p->raw.RovObs, p->raw.ephgps, p->raw.ephbds, &p->posRov);

			//step2: RTK algorithm
			rtkMain(&p->raw, &p->posBas, &p->posRov);

			//step3: Rover: RTK result outputs
			outputPOSf(p->cmpRovfp, &p->posRov, &p->raw, rest[0]);
			outputOEM7(&p->raw.RovObs, &p->posRov, &p->raw);
			if (cfg.saveRTKResToFile) outputOEM7f(p->logRovfp, p->posRovfp, &p->posRov, &p->raw);
		}

	} while (p->sync != -2); //-2: buff EOF
}

static void cache(callback_t *p, char type)
{
	switch (type)
	{
	case BASE: {
		p->BasTimeouts = 0;	// Reset the consecutive timeout counter
		memcpy(p->BuffBas + p->lenDBas, p->buffBas, p->lenRBas);
		p->lenDBas += p->lenRBas;

		if (cfg.saveRawBin&&p->rawBasOpen) // save binary data
			fwrite(p->buffBas, sizeof(int8_t), p->lenRBas, p->rawBasfp);

		break;
	}

	case ROVER: {
		p->RovTimeouts = 0;
		memcpy(p->BuffRov + p->lenDRov, p->buffRov, p->lenRRov);
		p->lenDRov += p->lenRRov;

		if (cfg.saveRawBin&&p->rawRovOpen)
			fwrite(p->buffRov, sizeof(int8_t), p->lenRRov, p->rawRovfp);

		break;
	}

	default: break;
	}
}

static void rejoin(callback_t* p, char type)
{
	switch (type)
	{
	case BASE: {
		p->BasTimeouts++;	// Increment the consecutive timeout counter

		if (p->BasTimeouts >= MAXTIMEOUTS) { // If consecutive timeouts reach a threshold, close and reopen the socket
			logout("[LOG]: Closing and reopening the Base socket after consecutive timeouts AT %d, %.3f\n", p->raw.RovObs.Time.Week, p->raw.RovObs.Time.SecOfWeek);
			closeSocket(p->NetBas);

			if (!openSocket(p->NetBas, cfg.ipBase, cfg.portBase)) // Attempt to reopen the socket
				logout("[LOG]: Failed to reopen the Base socket AT %d, %.3f\n", p->raw.RovObs.Time.Week, p->raw.RovObs.Time.SecOfWeek);

			p->BasTimeouts = 0; // Reset the Base's consecutive timeout counter
		}
		break;
	}

	case ROVER: {
		p->RovTimeouts++;

		if (p->RovTimeouts >= MAXTIMEOUTS) {
			logout("[LOG]: Closing and reopening the Rover socket after consecutive timeouts AT %d, %.3f\n", p->raw.RovObs.Time.Week, p->raw.RovObs.Time.SecOfWeek);
			closeSocket(p->NetRov);

			if (!openSocket(p->NetRov, cfg.ipRover, cfg.portRover))
				logout("[LOG]: Failed to reopen the Rover socket AT %d, %.3f\n", p->raw.RovObs.Time.Week, p->raw.RovObs.Time.SecOfWeek);

			p->RovTimeouts = 0;
		}
		break;
	}

	default: break;
	}
}

/* RTK mode: Socket --------------------------------------------------------- */
int socketMode() 
{	
	callback_t params;

	if (paramset(&params) < 0) return -1;
	
	/* Create a Timer */
	if (!CreateTimerQueueTimer(
		&params.Timer,      // Output: Timer handle
		params.TimerQueue,  // Timer queue handle
		TimerCallback,		// Timer callback function
		(PVOID)&params,     // User data (can pass on required data)
		100,				// Initial delay (milliseconds)
		1000,               // Interval (milliseconds)
		0                   // Flag (0 indicates that the timer is deleted 
							// immediately after execution of the timer task)
							// another option: WT_EXECUTEDEFAULT
	))  {
		logout("[WARNINGS]: Failed to create timer!\n");
		return -1;
	} else;

	/* Exit: Wait for the user to press Enter */
	getchar(); 

	/* Clean up system resources  */
	paramfree(&params);

	return 0;
}

int paramset(callback_t *params)
{
	params->TimerQueue = CreateTimerQueue();

	if (NULL == params->TimerQueue) {
		logout("[WARNINGS]: Failed to create timer queue!\n");
		return -1;
	}

	if (!openSocket(params->NetRov, cfg.ipRover, cfg.portRover)) {
		logout("[WARNINGS]: Rover ip & port was not opened.\n");
		DeleteTimerQueue(params->TimerQueue);
		return -1;
	}

	if (!openSocket(params->NetBas, cfg.ipBase, cfg.portBase)) {
		logout("[WARNINGS]: Base ip & port was not opened.\n");
		DeleteTimerQueue(params->TimerQueue);
		return -1;
	}

	// When compiling and running the .exe file within the project, the output will
	// be directed to the 'log' folder located in the root directory of the project.
	if (cfg.saveRTKResToFile)
		if (access(PATH_LOGDIR, 0) == -1)
			if (mkdir(PATH_LOGDIR))
				logout("[WARNINGS]: Unable to create directory.\n");

	fprintf(params->posRovfp, "%s\n", HEADER_POSFILE);

	/* Initialize the console output */
	system("cls");
	printf("%s%s", HEADER, HEADER_CONSOLE);

	return 0;
}

void paramfree(callback_t *params)
{
	DeleteTimerQueueTimer(params->TimerQueue, params->Timer, NULL);
	DeleteTimerQueue(params->TimerQueue);
	closeSocket(params->NetRov);
	closeSocket(params->NetBas);
}

/* Timer callback function -------------------------------------------------- */
VOID CALLBACK TimerCallback(PVOID lpParam, BOOLEAN TimerOrWaitFired)
{
	int stat[2]; fd_set readset[2]; // [0]=base, [1]=rover
	callback_t *p = (callback_t*)lpParam;

	checkstat(stat, readset, p);

	if (stat[ROVER] && FD_ISSET(p->NetRov, &readset[ROVER])) {	// Rover Socket is ready for reading

		if ((p->lenRRov = recv(p->NetRov, (char*)p->buffRov, MAXRAWLEN, 0)) > 0) {
			
			cache(p, ROVER);

			if (stat[BASE] && FD_ISSET(p->NetBas, &readset[BASE])) {	// Base Socket is ready for reading

				if ((p->lenRBas = recv(p->NetBas, (char*)p->buffBas, MAXRAWLEN, 0)) > 0) {
					cache(p, BASE);
					rtksol(p); // MAIN
					memset(p->buffBas, 0, sizeof(int8_t)*MAXRAWLEN);
				} else {
					logout("[LOG]: Base Socket read none AT %d, %.3f\n", p->raw.RovObs.Time.Week, p->raw.RovObs.Time.SecOfWeek);
					rtksol(p); // MAIN, WITH BASE NONE MSG
					rejoin(p, BASE);
					return; // Skip socket read and allow the timer to continue
				}

			} else if (!stat[BASE]) {
				logout("[LOG]: Base Socket read timeout AT %d, %.3f\n", p->raw.RovObs.Time.Week, p->raw.RovObs.Time.SecOfWeek);
				rtksol(p); // MAIN, WITH BASE TIMEOUT
				rejoin(p, BASE);
				return;

			} else {
				logout("[LOG]: Base Socket select error AT %d, %.3f\n", p->raw.RovObs.Time.Week, p->raw.RovObs.Time.SecOfWeek);
				rtksol(p); // MAIN, WITH BASE ERROR
				rejoin(p, BASE);
				return;
			}
			memset(p->buffRov, 0, sizeof(int8_t)*MAXRAWLEN);
			tracef("[ROVER RECV]: lenR:%d\t lenD: %d\t\n", p->lenRRov, p->lenDRov);

		} else {
			logout("[LOG]: Rover Socket read none AT %d, %.3f\n", p->raw.RovObs.Time.Week, p->raw.RovObs.Time.SecOfWeek);
			rejoin(p, ROVER);
			return;
		}

	} else if (!stat[ROVER]) {
		logout("[LOG]: Rover Socket read timeout AT %d, %.3f\n", p->raw.RovObs.Time.Week, p->raw.RovObs.Time.SecOfWeek);
		rejoin(p, ROVER);
		return;

	} else {
		logout("[LOG]: Rover Socket select error AT %d, %.3f\n", p->raw.RovObs.Time.Week, p->raw.RovObs.Time.SecOfWeek);
		rejoin(p, ROVER);
		return;
	}
}