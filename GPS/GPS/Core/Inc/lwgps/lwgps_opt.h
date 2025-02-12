/*
 * lwgps_opt.h
 *
 *  Created on: Aug 31, 2022
 *      Author: CHHOUN OUDOM
 */
#ifndef LWGPS_HDR_OPT_H
#define LWGPS_HDR_OPT_H


#ifndef LWGPS_IGNORE_USER_OPTS
#include "lwgps_opts.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif


#ifndef LWGPS_CFG_DOUBLE
#define LWGPS_CFG_DOUBLE                    1
#endif


#ifndef LWGPS_CFG_STATUS
#define LWGPS_CFG_STATUS                    0
#endif


#ifndef LWGPS_CFG_STATEMENT_GPGGA
#define LWGPS_CFG_STATEMENT_GPGGA           1
#endif


#ifndef LWGPS_CFG_STATEMENT_GPGSA
#define LWGPS_CFG_STATEMENT_GPGSA           1
#endif


#ifndef LWGPS_CFG_STATEMENT_GPRMC
#define LWGPS_CFG_STATEMENT_GPRMC           1
#endif


#ifndef LWGPS_CFG_STATEMENT_GPGSV
#define LWGPS_CFG_STATEMENT_GPGSV           1
#endif


#ifndef LWGPS_CFG_STATEMENT_GPGSV_SAT_DET
#define LWGPS_CFG_STATEMENT_GPGSV_SAT_DET   0
#endif


#ifndef LWGPS_CFG_STATEMENT_PUBX
#define LWGPS_CFG_STATEMENT_PUBX            0
#endif


#ifndef LWGPS_CFG_STATEMENT_PUBX_TIME
#define LWGPS_CFG_STATEMENT_PUBX_TIME       0
#endif


#ifndef LWGPS_CFG_CRC
#define LWGPS_CFG_CRC                       1
#endif


#if LWGPS_CFG_STATEMENT_PUBX_TIME && !LWGPS_CFG_STATEMENT_PUBX
#error LWGPS_CFG_STATEMENT_PUBX must be enabled when enabling LWGPS_CFG_STATEMENT_PUBX_TIME
#endif


#ifdef __cplusplus
}
#endif

#endif
