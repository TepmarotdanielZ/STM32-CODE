/*
 * lwgps.h
 *
 *  Created on: Aug 31, 2022
 *      Author: CHHOUN OUDOM
 */
#ifndef LWGPS_HDR_H
#define LWGPS_HDR_H

#include <stdint.h>
#include <stddef.h>
#include <limits.h>
#include "lwgps/lwgps_opt.h"

#ifdef __cplusplus
extern "C" {
#endif

#if LWGPS_CFG_DOUBLE || __DOXYGEN__
typedef double lwgps_float_t;
#else
typedef float lwgps_float_t;
#endif

typedef struct {
    uint8_t num;
    uint8_t elevation;
    uint16_t azimuth;
    uint8_t snr;
} lwgps_sat_t;


typedef enum {
    STAT_UNKNOWN    = 0,
    STAT_GGA        = 1,
    STAT_GSA        = 2,
    STAT_GSV        = 3,
    STAT_RMC        = 4,
    STAT_UBX        = 5,
    STAT_UBX_TIME   = 6,
    STAT_CHECKSUM_FAIL = UINT8_MAX
} lwgps_statement_t;


typedef struct {
#if LWGPS_CFG_STATEMENT_GPGGA || __DOXYGEN__
    lwgps_float_t latitude;
    lwgps_float_t longitude;
    lwgps_float_t altitude;
    lwgps_float_t geo_sep;
    uint8_t sats_in_use;
    uint8_t fix;
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
#endif

#if LWGPS_CFG_STATEMENT_GPGSA || __DOXYGEN__
    lwgps_float_t dop_h;
    lwgps_float_t dop_v;
    lwgps_float_t dop_p;
    uint8_t fix_mode;
    uint8_t satellites_ids[12];
#endif

#if LWGPS_CFG_STATEMENT_GPGSV || __DOXYGEN__

    uint8_t sats_in_view;
#if LWGPS_CFG_STATEMENT_GPGSV_SAT_DET || __DOXYGEN__
    lwgps_sat_t sats_in_view_desc[12];
#endif
#endif

#if LWGPS_CFG_STATEMENT_GPRMC || __DOXYGEN__

    uint8_t is_valid;
    lwgps_float_t speed;
    lwgps_float_t course;
    lwgps_float_t variation;
    uint8_t date;
    uint8_t month;
    uint8_t year;
#endif

#if LWGPS_CFG_STATEMENT_PUBX_TIME || __DOXYGEN__
#if !LWGPS_CFG_STATEMENT_GPGGA && !__DOXYGEN__

    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
#endif
#if !LWGPS_CFG_STATEMENT_GPRMC && !__DOXYGEN__

    uint8_t date;
    uint8_t month;
    uint8_t year;
#endif
    lwgps_float_t utc_tow;
    uint16_t utc_wk;
    uint8_t leap_sec;
    uint32_t clk_bias;
    lwgps_float_t clk_drift;
    uint32_t tp_gran;
#endif

#if !__DOXYGEN__
    struct {
        lwgps_statement_t stat;
        char term_str[13];
        uint8_t term_pos;
        uint8_t term_num;

        uint8_t star;

#if LWGPS_CFG_CRC
        uint8_t crc_calc;
#endif

        union {
            uint8_t dummy;
#if LWGPS_CFG_STATEMENT_GPGGA
            struct {
                lwgps_float_t latitude;
                lwgps_float_t longitude;
                lwgps_float_t altitude;
                lwgps_float_t geo_sep;
                uint8_t sats_in_use;
                uint8_t fix;
                uint8_t hours;
                uint8_t minutes;
                uint8_t seconds;
            } gga;
#endif
#if LWGPS_CFG_STATEMENT_GPGSA
            struct {
                lwgps_float_t dop_h;
                lwgps_float_t dop_v;
                lwgps_float_t dop_p;
                uint8_t fix_mode;
                uint8_t satellites_ids[12];
            } gsa;
#endif
#if LWGPS_CFG_STATEMENT_GPGSV
            struct {
                uint8_t sats_in_view;
                uint8_t stat_num;
            } gsv;
#endif
#if LWGPS_CFG_STATEMENT_GPRMC
            struct {
                uint8_t is_valid;
                uint8_t date;
                uint8_t month;
                uint8_t year;
                lwgps_float_t speed;
                lwgps_float_t course;
                lwgps_float_t variation;
            } rmc;
#endif
#if LWGPS_CFG_STATEMENT_PUBX_TIME
            struct {
                uint8_t hours;
                uint8_t minutes;
                uint8_t seconds;
                uint8_t date;
                uint8_t month;
                uint8_t year;
                lwgps_float_t utc_tow;
                uint16_t utc_wk;
                uint8_t leap_sec;
                uint32_t clk_bias;
                lwgps_float_t clk_drift;
                uint32_t tp_gran;
            } time;
#endif
        } data;
    } p;
#endif
} lwgps_t;


typedef enum {

    lwgps_speed_kps,
    lwgps_speed_kph,
    lwgps_speed_mps,
    lwgps_speed_mpm,


    lwgps_speed_mips,
    lwgps_speed_mph,
    lwgps_speed_fps,
    lwgps_speed_fpm,


    lwgps_speed_mpk,
    lwgps_speed_spk,
    lwgps_speed_sp100m,
    lwgps_speed_mipm,
    lwgps_speed_spm,
    lwgps_speed_sp100y,


    lwgps_speed_smph,
} lwgps_speed_t;


typedef void (*lwgps_process_fn)(lwgps_statement_t res);


#if LWGPS_CFG_STATEMENT_GPRMC || __DOXYGEN__
#define lwgps_is_valid(_gh)         ((_gh)->is_valid)
#else
#define lwgps_is_valid(_gh)         (0)
#endif

uint8_t         lwgps_init(lwgps_t* gh);
#if LWGPS_CFG_STATUS || __DOXYGEN__
uint8_t         lwgps_process(lwgps_t* gh, const void* data, size_t len, lwgps_process_fn evt_fn);
#else
uint8_t         lwgps_process(lwgps_t* gh, const void* data, size_t len);
#endif
uint8_t         lwgps_distance_bearing(lwgps_float_t las, lwgps_float_t los, lwgps_float_t lae, lwgps_float_t loe, lwgps_float_t* d, lwgps_float_t* b);
lwgps_float_t   lwgps_to_speed(lwgps_float_t sik, lwgps_speed_t ts);



#ifdef __cplusplus
}
#endif

#endif
