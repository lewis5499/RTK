/*  C programming for trace/log-related functions and macros.
*
*
*	Provides functionality for logging debug information to a file or standard output.
*
*   Author: Liu Hengzhen
*   Date: April 22nd, 2024
*/

#ifndef _C_TRACE_H_
#define _C_TRACE_H_

#include <stdio.h>
#include "rtklib.h"

extern FILE *traceLogfile;
extern conf_t cfg;

#define ENABLE_LOG		1
#define ENABLE_TRACE	1

/* log operations */
#if ENABLE_LOG

#define logerr(fmt, ...) \
    do { \
        if (cfg.enableLog) { \
            fprintf(stderr, fmt, ##__VA_ARGS__); \
        } \
    } while (0)

#define logout(fmt, ...)\
    do { \
        if (cfg.enableLog) { \
            fprintf(stdout, fmt, ##__VA_ARGS__); \
        } \
    } while (0)

#else
#define logerr(fmt, ...)	do{}while(0)
#define logout(fmt, ...)	do{}while(0)
#endif

/* trace operations */
#if ENABLE_TRACE

#define trace(fmt, ...) printf("FILE PATH= %s, LINES= %d, MSG= " fmt, __FILE__, __LINE__, ##__VA_ARGS__)

#define tracef(fmt, ...) \
    do { \
        if (cfg.enableTrace && traceLogfile) { \
            fprintf(traceLogfile, "FILE PATH= %s, LINES= %d, MSG= " fmt, __FILE__, __LINE__, ##__VA_ARGS__); \
            fflush(traceLogfile); \
        } \
    } while (0)

#define TRACE_INIT \
    do { \
        traceLogfile = fopen(cfg.traceLogPath, "w"); \
        if (!traceLogfile) { \
            fprintf(stderr, "Error opening trace log file.\n"); \
            break; \
        } \
    } while (0)

#define TRACE_FREE \
    do { \
        if (traceLogfile) { \
            fclose(traceLogfile); \
            traceLogfile = NULL; \
        } \
    } while (0)

#else  // empty
#define trace(fmt, ...)		do{}while(0)
#define tracef(fmt, ...)	do{}while(0)
#define TRACE_INIT			do{}while(0)
#define TRACE_FREE			do{}while(0)
#endif



#endif
