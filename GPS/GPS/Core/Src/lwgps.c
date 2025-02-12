/*
 * lwgps.c
 *
 *  Created on: Aug 31, 2022
 *      Author: CHHOUN OUDOM
 */
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "lwgps/lwgps.h"

#define FLT(x)              ((lwgps_float_t)(x))
#define D2R(x)              FLT(FLT(x) * FLT(0.01745329251994))
#define R2D(x)              FLT(FLT(x) * FLT(57.29577951308232))
#define EARTH_RADIUS        FLT(6371.0)

#if LWGPS_CFG_CRC
#define CRC_ADD(_gh, ch)    (_gh)->p.crc_calc ^= (uint8_t)(ch)
#else
#define CRC_ADD(_gh, ch)
#endif
#define TERM_ADD(_gh, ch)   do {    \
        if ((_gh)->p.term_pos < (sizeof((_gh)->p.term_str) - 1)) {  \
            (_gh)->p.term_str[(_gh)->p.term_pos] = (ch);\
            (_gh)->p.term_str[++(_gh)->p.term_pos] = 0; \
        }                               \
    } while (0)
#define TERM_NEXT(_gh)      do { (_gh)->p.term_str[((_gh)->p.term_pos = 0)] = 0; ++(_gh)->p.term_num; } while (0)

#define CIN(x)              ((x) >= '0' && (x) <= '9')
#define CIHN(x)             (((x) >= '0' && (x) <= '9') || ((x) >= 'a' && (x) <= 'f') || ((x) >= 'A' && (x) <= 'F'))
#define CTN(x)              ((x) - '0')
#define CHTN(x)             (((x) >= '0' && (x) <= '9') ? ((x) - '0') : (((x) >= 'a' && (x) <= 'z') ? ((x) - 'a' + 10) : (((x) >= 'A' && (x) <= 'Z') ? ((x) - 'A' + 10) : 0)))


static int32_t
prv_parse_number(lwgps_t* gh, const char* t) {
    int32_t res = 0;
    uint8_t minus;

    if (t == NULL) {
        t = gh->p.term_str;
    }
    for (; t != NULL && *t == ' '; ++t) {}

    minus = (*t == '-' ? (++t, 1) : 0);
    for (; t != NULL && CIN(*t); ++t) {
        res = 10 * res + CTN(*t);
    }
    return minus ? -res : res;
}


static lwgps_float_t
prv_parse_float_number(lwgps_t* gh, const char* t) {
    lwgps_float_t res;

    if (t == NULL) {
        t = gh->p.term_str;
    }
    for (; t != NULL && *t == ' '; ++t) {}

#if LWGPS_CFG_DOUBLE
    res = strtod(t, NULL);
#else
    res = strtof(t, NULL);
#endif

    return FLT(res);
}


static lwgps_float_t
prv_parse_lat_long(lwgps_t* gh) {
    lwgps_float_t ll, deg, min;

    ll = prv_parse_float_number(gh, NULL);
    deg = FLT((int)((int)ll / 100));
    min = ll - (deg * FLT(100));
    ll = deg + (min / FLT(60.0));

    return ll;
}


static uint8_t
prv_parse_term(lwgps_t* gh) {
    if (gh->p.term_num == 0) {
        if (0) {
#if LWGPS_CFG_STATEMENT_GPGGA
        } else if (!strncmp(gh->p.term_str, "$GPGGA", 6) || !strncmp(gh->p.term_str, "$GNGGA", 6)) {
            gh->p.stat = STAT_GGA;
#endif
#if LWGPS_CFG_STATEMENT_GPGSA
        } else if (!strncmp(gh->p.term_str, "$GPGSA", 6) || !strncmp(gh->p.term_str, "$GNGSA", 6)) {
            gh->p.stat = STAT_GSA;
#endif
#if LWGPS_CFG_STATEMENT_GPGSV
        } else if (!strncmp(gh->p.term_str, "$GPGSV", 6) || !strncmp(gh->p.term_str, "$GNGSV", 6)) {
            gh->p.stat = STAT_GSV;
#endif
#if LWGPS_CFG_STATEMENT_GPRMC
        } else if (!strncmp(gh->p.term_str, "$GPRMC", 6) || !strncmp(gh->p.term_str, "$GNRMC", 6)) {
            gh->p.stat = STAT_RMC;
#endif
#if LWGPS_CFG_STATEMENT_PUBX
        } else if (!strncmp(gh->p.term_str, "$PUBX", 5)) {
            gh->p.stat = STAT_UBX;
#endif
        } else {
            gh->p.stat = STAT_UNKNOWN;
        }
        return 1;
    }


    if (gh->p.stat == STAT_UNKNOWN) {
#if LWGPS_CFG_STATEMENT_GPGGA
    } else if (gh->p.stat == STAT_GGA) {
        switch (gh->p.term_num) {
            case 1:
                gh->p.data.gga.hours = 10 * CTN(gh->p.term_str[0]) + CTN(gh->p.term_str[1]);
                gh->p.data.gga.minutes = 10 * CTN(gh->p.term_str[2]) + CTN(gh->p.term_str[3]);
                gh->p.data.gga.seconds = 10 * CTN(gh->p.term_str[4]) + CTN(gh->p.term_str[5]);
                break;
            case 2:
                gh->p.data.gga.latitude = prv_parse_lat_long(gh);
                break;
            case 3:
                if (gh->p.term_str[0] == 'S' || gh->p.term_str[0] == 's') {
                    gh->p.data.gga.latitude = -gh->p.data.gga.latitude;
                }
                break;
            case 4:
                gh->p.data.gga.longitude = prv_parse_lat_long(gh);
                break;
            case 5:
                if (gh->p.term_str[0] == 'W' || gh->p.term_str[0] == 'w') {
                    gh->p.data.gga.longitude = -gh->p.data.gga.longitude;
                }
                break;
            case 6:
                gh->p.data.gga.fix = (uint8_t)prv_parse_number(gh, NULL);
                break;
            case 7:
                gh->p.data.gga.sats_in_use = (uint8_t)prv_parse_number(gh, NULL);
                break;
            case 9:
                gh->p.data.gga.altitude = prv_parse_float_number(gh, NULL);
                break;
            case 11:
                gh->p.data.gga.geo_sep = prv_parse_float_number(gh, NULL);
                break;
            default:
                break;
        }
#endif
#if LWGPS_CFG_STATEMENT_GPGSA
    } else if (gh->p.stat == STAT_GSA) {
        switch (gh->p.term_num) {
            case 2:
                gh->p.data.gsa.fix_mode = (uint8_t)prv_parse_number(gh, NULL);
                break;
            case 15:
                gh->p.data.gsa.dop_p = prv_parse_float_number(gh, NULL);
                break;
            case 16:
                gh->p.data.gsa.dop_h = prv_parse_float_number(gh, NULL);
                break;
            case 17:
                gh->p.data.gsa.dop_v = prv_parse_float_number(gh, NULL);
                break;
            default:

                if (gh->p.term_num >= 3 && gh->p.term_num <= 14) {
                    gh->p.data.gsa.satellites_ids[gh->p.term_num - 3] = (uint8_t)prv_parse_number(gh, NULL);
                }
                break;
        }
#endif
#if LWGPS_CFG_STATEMENT_GPGSV
    } else if (gh->p.stat == STAT_GSV) {
        switch (gh->p.term_num) {
            case 2:
                gh->p.data.gsv.stat_num = (uint8_t)prv_parse_number(gh, NULL);
                break;
            case 3:
                gh->p.data.gsv.sats_in_view = (uint8_t)prv_parse_number(gh, NULL);
                break;
            default:
#if LWGPS_CFG_STATEMENT_GPGSV_SAT_DET
                if (gh->p.term_num >= 4 && gh->p.term_num <= 19) {
                    uint8_t index, term_num = gh->p.term_num - 4;
                    uint16_t value;

                    index = ((gh->p.data.gsv.stat_num - 1) << 0x02) + (term_num >> 2);
                    if (index < sizeof(gh->sats_in_view_desc) / sizeof(gh->sats_in_view_desc[0])) {
                        value = (uint16_t)prv_parse_number(gh, NULL);
                        switch (term_num & 0x03) {
                            case 0:
                                gh->sats_in_view_desc[index].num = value;
                                break;
                            case 1:
                                gh->sats_in_view_desc[index].elevation = value;
                                break;
                            case 2:
                                gh->sats_in_view_desc[index].azimuth = value;
                                break;
                            case 3:
                                gh->sats_in_view_desc[index].snr = value;
                                break;
                            default:
                                break;
                        }
                    }
                }
#endif
                break;
        }
#endif
#if LWGPS_CFG_STATEMENT_GPRMC
    } else if (gh->p.stat == STAT_RMC) {
        switch (gh->p.term_num) {
            case 2:
                gh->p.data.rmc.is_valid = (gh->p.term_str[0] == 'A');
                break;
            case 7:
                gh->p.data.rmc.speed = prv_parse_float_number(gh, NULL);
                break;
            case 8:
                gh->p.data.rmc.course = prv_parse_float_number(gh, NULL);
                break;
            case 9:
                gh->p.data.rmc.date = (uint8_t)(10 * CTN(gh->p.term_str[0]) + CTN(gh->p.term_str[1]));
                gh->p.data.rmc.month = (uint8_t)(10 * CTN(gh->p.term_str[2]) + CTN(gh->p.term_str[3]));
                gh->p.data.rmc.year = (uint8_t)(10 * CTN(gh->p.term_str[4]) + CTN(gh->p.term_str[5]));
                break;
            case 10:
                gh->p.data.rmc.variation = prv_parse_float_number(gh, NULL);
                break;
            case 11:
                if (gh->p.term_str[0] == 'W' || gh->p.term_str[0] == 'w') {
                    gh->p.data.rmc.variation = -gh->p.data.rmc.variation;
                }
                break;
            default:
                break;
        }
#endif
#if LWGPS_CFG_STATEMENT_PUBX
    } else if (gh->p.stat == STAT_UBX) {
        if (gh->p.term_str[0] == '0' && gh->p.term_str[1] == '4') {
            gh->p.stat = STAT_UBX_TIME;
        }
#if LWGPS_CFG_STATEMENT_PUBX_TIME
    } else if (gh->p.stat == STAT_UBX_TIME) {
        switch (gh->p.term_num) {
            case 2:
                gh->p.data.time.hours = 10 * CTN(gh->p.term_str[0]) + CTN(gh->p.term_str[1]);
                gh->p.data.time.minutes = 10 * CTN(gh->p.term_str[2]) + CTN(gh->p.term_str[3]);
                gh->p.data.time.seconds = 10 * CTN(gh->p.term_str[4]) + CTN(gh->p.term_str[5]);
                break;
            case 3:
                gh->p.data.time.date = 10 * CTN(gh->p.term_str[0]) + CTN(gh->p.term_str[1]);
                gh->p.data.time.month = 10 * CTN(gh->p.term_str[2]) + CTN(gh->p.term_str[3]);
                gh->p.data.time.year = 10 * CTN(gh->p.term_str[4]) + CTN(gh->p.term_str[5]);
                break;
            case 4:
                gh->p.data.time.utc_tow = prv_parse_float_number(gh, NULL);
                break;
            case 5:
                gh->p.data.time.utc_wk = prv_parse_number(gh, NULL);
                break;
            case 6:

                if (gh->p.term_str[2] == 'D' || gh->p.term_str[2] == '\0') {
                    gh->p.data.time.leap_sec = 10 * CTN(gh->p.term_str[0])
                                               + CTN(gh->p.term_str[1]);
                } else {
                    gh->p.data.time.leap_sec = 100 * CTN(gh->p.term_str[0])
                                               + 10 * CTN(gh->p.term_str[1])
                                               + CTN(gh->p.term_str[2]);
                }
                break;
            case 7:
                gh->p.data.time.clk_bias = prv_parse_number(gh, NULL);
                break;
            case 8:
                gh->p.data.time.clk_drift = prv_parse_float_number(gh, NULL);
                break;
            case 9:
                gh->p.data.time.tp_gran = prv_parse_number(gh, NULL);
                break;
            default:
                break;
        }
#endif
#endif
    }
    return 1;
}

#if LWGPS_CFG_CRC

static uint8_t
prv_check_crc(lwgps_t* gh) {
    uint8_t crc;
    crc = (uint8_t)((CHTN(gh->p.term_str[0]) & 0x0F) << 0x04) | (CHTN(gh->p.term_str[1]) & 0x0F);
    return gh->p.crc_calc == crc;
}
#else
#define prv_check_crc(_gh)              (1)
#endif /* LWGPS_CFG_CRC */


static uint8_t
prv_copy_from_tmp_memory(lwgps_t* gh) {
    if (0) {
#if LWGPS_CFG_STATEMENT_GPGGA
    } else if (gh->p.stat == STAT_GGA) {
        gh->latitude = gh->p.data.gga.latitude;
        gh->longitude = gh->p.data.gga.longitude;
        gh->altitude = gh->p.data.gga.altitude;
        gh->geo_sep = gh->p.data.gga.geo_sep;
        gh->sats_in_use = gh->p.data.gga.sats_in_use;
        gh->fix = gh->p.data.gga.fix;
        gh->hours = gh->p.data.gga.hours;
        gh->minutes = gh->p.data.gga.minutes;
        gh->seconds = gh->p.data.gga.seconds;
#endif
#if LWGPS_CFG_STATEMENT_GPGSA
    } else if (gh->p.stat == STAT_GSA) {
        gh->dop_h = gh->p.data.gsa.dop_h;
        gh->dop_p = gh->p.data.gsa.dop_p;
        gh->dop_v = gh->p.data.gsa.dop_v;
        gh->fix_mode = gh->p.data.gsa.fix_mode;
        memcpy(gh->satellites_ids, gh->p.data.gsa.satellites_ids, sizeof(gh->satellites_ids));
#endif
#if LWGPS_CFG_STATEMENT_GPGSV
    } else if (gh->p.stat == STAT_GSV) {
        gh->sats_in_view = gh->p.data.gsv.sats_in_view;
#endif
#if LWGPS_CFG_STATEMENT_GPRMC
    } else if (gh->p.stat == STAT_RMC) {
        gh->course = gh->p.data.rmc.course;
        gh->is_valid = gh->p.data.rmc.is_valid;
        gh->speed = gh->p.data.rmc.speed;
        gh->variation = gh->p.data.rmc.variation;
        gh->date = gh->p.data.rmc.date;
        gh->month = gh->p.data.rmc.month;
        gh->year = gh->p.data.rmc.year;
#endif
#if LWGPS_CFG_STATEMENT_PUBX_TIME
    } else if (gh->p.stat == STAT_UBX_TIME) {
        gh->hours = gh->p.data.time.hours;
        gh->minutes = gh->p.data.time.minutes;
        gh->seconds = gh->p.data.time.seconds;
        gh->date = gh->p.data.time.date;
        gh->month = gh->p.data.time.month;
        gh->year = gh->p.data.time.year;
        gh->utc_tow = gh->p.data.time.utc_tow;
        gh->utc_wk = gh->p.data.time.utc_wk;
        gh->leap_sec = gh->p.data.time.leap_sec;
        gh->clk_bias = gh->p.data.time.clk_bias;
        gh->clk_drift = gh->p.data.time.clk_drift;
        gh->tp_gran = gh->p.data.time.tp_gran;
#endif
    }
    return 1;
}


uint8_t
lwgps_init(lwgps_t* gh) {
    memset(gh, 0x00, sizeof(*gh));
    return 1;
}


uint8_t
#if LWGPS_CFG_STATUS || __DOXYGEN__
lwgps_process(lwgps_t* gh, const void* data, size_t len, lwgps_process_fn evt_fn) {
#else
lwgps_process(lwgps_t* gh, const void* data, size_t len) {
#endif
    const uint8_t* d = data;

    for (; len > 0; ++d, --len) {
        if (*d == '$') {
            memset(&gh->p, 0x00, sizeof(gh->p));
            TERM_ADD(gh, *d);
        } else if (*d == ',') {
            prv_parse_term(gh);
            CRC_ADD(gh, *d);
            TERM_NEXT(gh);
        } else if (*d == '*') {
            prv_parse_term(gh);
            gh->p.star = 1;
            TERM_NEXT(gh);
        } else if (*d == '\r') {
            if (prv_check_crc(gh)) {

                prv_copy_from_tmp_memory(gh);
#if LWGPS_CFG_STATUS
                if (evt_fn != NULL) {
                    evt_fn(gh->p.stat);
                }
            } else if (evt_fn != NULL) {
                evt_fn(STAT_CHECKSUM_FAIL);
#endif
            }
        } else {
            if (!gh->p.star) {
                CRC_ADD(gh, *d);
            }
            TERM_ADD(gh, *d);
        }
    }
    return 1;
}


uint8_t
lwgps_distance_bearing(lwgps_float_t las, lwgps_float_t los, lwgps_float_t lae, lwgps_float_t loe, lwgps_float_t* d, lwgps_float_t* b) {
    lwgps_float_t df, dfi, a;

    if (d == NULL && b == NULL) {
        return 0;
    }


    df = D2R(lae - las);
    dfi = D2R(loe - los);
    las = D2R(las);
    los = D2R(los);
    lae = D2R(lae);
    loe = D2R(loe);


    if (d != NULL) {

#if LWGPS_CFG_DOUBLE
        a = FLT(sin(df * 0.5) * sin(df * 0.5) + sin(dfi * 0.5) * sin(dfi * 0.5) * cos(las) * cos(lae));
        *d = FLT(EARTH_RADIUS * 2.0 * atan2(sqrt(a), sqrt(1.0 - a)) * 1000.0);
#else
        a = FLT(sinf(df * 0.5f) * sinf(df * 0.5f) + sinf(dfi * 0.5f) * sinf(dfi * 0.5f) * cosf(las) * cosf(lae));
        *d = FLT(EARTH_RADIUS * 2.0f * atan2f(sqrtf(a), sqrtf(1.0f - a)) * 1000.0f);
#endif
    }

    if (b != NULL) {
#if LWGPS_CFG_DOUBLE
        df = FLT(sin(loe - los) * cos(lae));
        dfi = FLT(cos(las) * sin(lae) - sin(las) * cos(lae) * cos(loe - los));

        *b = R2D(atan2(df, dfi));
#else
        df = FLT(sinf(loe - los) * cosf(lae));
        dfi = FLT(cosf(las) * sinf(lae) - sinf(las) * cosf(lae) * cosf(loe - los));

        *b = R2D(atan2f(df, dfi));
#endif
        if (*b < 0) {
            *b += FLT(360);
        }
    }
    return 1;
}


lwgps_float_t
lwgps_to_speed(lwgps_float_t sik, lwgps_speed_t ts) {
    switch (ts) {
        case lwgps_speed_kps:
            return FLT(sik * FLT(2000));
        case lwgps_speed_kph:
            return FLT(sik * FLT(1852));
        case lwgps_speed_mps:
            return FLT(sik * FLT(5144));
        case lwgps_speed_mpm:
            return FLT(sik * FLT(30.87));

        case lwgps_speed_mips:
            return FLT(sik * FLT(0.0003197));
        case lwgps_speed_mph:
            return FLT(sik * FLT(1.151));
        case lwgps_speed_fps:
            return FLT(sik * FLT(1.688));
        case lwgps_speed_fpm:
            return FLT(sik * FLT(101.3));

        case lwgps_speed_mpk:
            return FLT(sik * FLT(32.4));
        case lwgps_speed_spk:
            return FLT(sik * FLT(1944.0));
        case lwgps_speed_sp100m:
            return FLT(sik * FLT(194.4));
        case lwgps_speed_mipm:
            return FLT(sik * FLT(52.14));
        case lwgps_speed_spm:
            return FLT(sik * FLT(3128.0));
        case lwgps_speed_sp100y:
            return FLT(sik * FLT(177.7));

        case lwgps_speed_smph:
            return FLT(sik * FLT(10.0));
        default:
            return 0;
    }
}
