/******************************************************************************
 * @gps.h
 * @brief Interface for reading and parsing data from the GPS.
 * @version 0.0.1
 *  **************************************************************************/
#ifndef GPS_H_
#define GPS_H_

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include "serial_io_uart.h"

extern bool DEBUG;

/******************************************************************************
 * 								DEFS
******************************************************************************/

#define GPS_BAUD_RATE 9600
#define MAX_NMEA_LEN 82
#define GPS_LOC_INFO_TIME_BUF_SIZE 18
#define RECV_TIMEOUT_MS 3000
#define EARTH_RADIUS_METERS 6378100
#define EARTH_RADIUS_KMS 6373
#define MATH_PI 3.14159265358979323846

#define __packed __attribute__((__packed__))

/* Parsing defs */
#define GGA_PREFIX "$GPGGA" // GGA - most info
#define RMC_PREFIX "$GPRMC" // has date
#define GSA_PREFIX "$GPGSA"
#define PREFIX_LEN 6
#define DELIMITER ','
#define END_OF_TOKEN '\0'
#define GGA_FIELDS_NUM 14
#define RMC_FIELDS_NUM 11
#define GSA_FIELDS_NUM 17
#define GGA_MIN_REQUIRED_FIELDS 7 // time .. #satellites
#define RMC_DATE_FIELD 8
#define RMC_TIME_FIELD 0

#define FLOAT_RMV_FACTOR 10000000
#define ALT_FACTOR 100
#define HDOP_FACTOR 5
#define LAT_DEG_DIGITS 2
#define LONGIT_DEG_DIGITS 3

#define DATE_FORMAT "%c%c:%c%c:%c%c %c%c.%c%c.%c%c"
#define GPS_PAYLOAD_FORMAT "gps,name=NetanelFayoumi_SapirElyovitch,ICCID=%s latitude=%f,longitude=%f,altitude=%d,hdop=%u,valid_fix=%u,num_sats=%u %s000000000"
#define GPS_PAYLOAD_SPEED_FORMAT "gps,name=NetanelFayoumi_SapirElyovitch,ICCID=%s,highspeed=%s latitude=%f,longitude=%f,altitude=%d,hdop=%u,valid_fix=%u,num_sats=%u %s000000000"


typedef __packed struct _GPS_LOCATION_INFO {
    int32_t latitude;
    int32_t longitude;
    int32_t altitude;
    uint8_t hdop;
    uint8_t valid_fix : 1;
    uint8_t reserved1 : 3;
    uint8_t num_sats : 4;
    char fixtime[GPS_LOC_INFO_TIME_BUF_SIZE]; // hh:mm:ss DD.MM.YY\0
} GPS_LOCATION_INFO;



/**
 * @brief Initiate GPS connection.
 */
void GPSInit();


/**
 * Gets line from GPS.
 * @param buf - output buffer to put the line into.
 * @param maxlen - max length of a line.
 * @return number of bytes received.
 */
uint32_t GPSGetReadRaw(char *buf, unsigned int maxlen);


/**
 * Updates location with information from GPSGetReadRaw.
 * @param location - the struct to be filled.
 * @return 0 if successful, -1 otherwise.
 */
bool GPSGetFixInformation(GPS_LOCATION_INFO *location);


/*
 * Disable GPS connection.
 */
void GPSDisable();

/**
 * Converting GPS fixtime to Unixtime
 */
void GPSConvertFixtimeToUnixTime(GPS_LOCATION_INFO * gps_data, char * unix_time);


/**
 * @brief prepare a predefined GPS payload based on inputs.
 * @param gps_data GPS LOCATION as received from GPS module
 * @param gps_payload where to store result
 * @return length of payload
 */
int GPSGetPayload(GPS_LOCATION_INFO * gps_data, char * iccid, char * unix_time, char * gps_payload, int maximal_payload_size);


/**
 * @brief prepare a predefined GPS payload with highspeed tag based on inputs.
 * @param gps_data GPS LOCATION as received from GPS module
 * @param gps_payload where to store result
 * @return length of payload
 */
int GPSGetSPEEDPayload(GPS_LOCATION_INFO * gps_data, char * iccid, char * unix_time, bool highspeed, char * gps_payload, int maximal_payload_size);

/**
 * get speed of movement between to GPS location
 * @return speed in km/h
 */
double GPSGetSpeedOfLocations(GPS_LOCATION_INFO * old_loc, GPS_LOCATION_INFO * new_loc);

#endif /* GPS_H_ */
