/******************************************************************************
 * @gps.c
 * @brief Interface for reading and parsing data from the GPS.
 * @version 0.0.1
 *  **************************************************************************/
#include <stdlib.h>
#include <time.h>
#include "gps.h"

/******************************************************************************
 * 							GLOBAL VARIABLES
******************************************************************************/
static bool GPS_INITIALIZED = false;

/******************************************************************************
 * @brief Initiate GPS connection.
 *****************************************************************************/
void GPSInit(char * port) {
	printf("Initializing GPS...");
    GPS_INITIALIZED = SerialInitGPS(port, GPS_BAUD_RATE);
    if (!GPS_INITIALIZED) {
    	printf("Initialization FAILED.\n");
        exit(EXIT_FAILURE);
    }
    printf("Initializing successfully.\n");
}

/******************************************************************************
 * Gets line from GPS.
 * @param buf - output buffer to put the line into.
 * @param maxlen - max length of a line.
 * @return number of bytes received.
 *****************************************************************************/
uint32_t GPSGetReadRaw(char *buf, unsigned int maxlen) {
	memset(buf, '\0', maxlen);
    if (GPS_INITIALIZED) {
        return SerialRecvGPS((unsigned char*) buf, maxlen, RECV_TIMEOUT_MS);
    }
    else {
        return 0;
    }
}

/******************************************************************************
 * Gets a GPS line and splits it into tokens.
 * @param num_of_fields - number of fields in the type of line being split.
 * @param prefix_length - the length of the line prefix.
 * @param buf - the buffer.
 * @param tokens_array - the output token array.
 * @return true if successful, false otherwise.
 *****************************************************************************/
bool splitLineToFields(int num_of_fields, int prefix_length, char
buf[MAX_NMEA_LEN], char**
tokens_array){

    char* p = &buf[prefix_length]; // start line after prefix
    int i=0;
    char* last_token = NULL;

    while (i < num_of_fields) {
        if (*p == DELIMITER)
        {
            *p = END_OF_TOKEN;
            if (last_token != NULL){
                tokens_array[i++] = last_token;
            }
            // new field
            p++;
            if (*p == DELIMITER)
            {
                // empty field
                tokens_array[i++] = NULL;
                last_token = NULL;
            } else {
                // non-empty field
                last_token = p;
            }
        } else {
            // same field
            if (i == num_of_fields-1) // last field
            {
                tokens_array[i++] = last_token;
            }
            p++;
        }
    }
    return true;
}


/******************************************************************************
 * Gets array (split_line) with tokens that are RMC's fields,
 * updates location accordingly.
 * @param split_line - RMC line split into tokens.
 * @param location - location struct to be filled.
 * @return true if successful, false otherwise.
 *****************************************************************************/
bool parseRMC(char** split_line, GPS_LOCATION_INFO *location){
    // only fills date and time in the following order:
    // hhmmssDDMMYY
    if (split_line[RMC_TIME_FIELD] == NULL ||
        split_line[RMC_DATE_FIELD] == NULL){
        return false;
    }

    sprintf(location->fixtime,
    		DATE_FORMAT,
			split_line[RMC_TIME_FIELD][0],
			split_line[RMC_TIME_FIELD][1],
			split_line[RMC_TIME_FIELD][2],
			split_line[RMC_TIME_FIELD][3],
			split_line[RMC_TIME_FIELD][4],
			split_line[RMC_TIME_FIELD][5],
			split_line[RMC_DATE_FIELD][0],
			split_line[RMC_DATE_FIELD][1],
			split_line[RMC_DATE_FIELD][2],
			split_line[RMC_DATE_FIELD][3],
			split_line[RMC_DATE_FIELD][4],
			split_line[RMC_DATE_FIELD][5]);
    return true;
}


/******************************************************************************
 * Gets array (split_line) with tokens that are GGA's fields,
 * updates location accordingly.
 * @param split_line - GGA line split into tokens.
 * @param location - location struct to be filled.
 * @return true if successful, false otherwise.
 *****************************************************************************/
bool parseGGA(char** split_line, GPS_LOCATION_INFO *location){

    // check for empty required fields
    for (int j=0; j<GGA_MIN_REQUIRED_FIELDS; j++){
        if (split_line[j] == NULL){
            return false;
        }
    }

    // counter for the fields in split_line
    int i = 0;

    /* time */ //HHMMSS (UTC)
    location->fixtime[0] = split_line[i][0];
    location->fixtime[1] = split_line[i][1];
    location->fixtime[2] = ':';
    location->fixtime[3] = split_line[i][2];
    location->fixtime[4] = split_line[i][3];
    location->fixtime[5] = ':';
    location->fixtime[6] = split_line[i][4];
    location->fixtime[7] = split_line[i][5];

    i++;

    /* latitude */
    int32_t degrees = (split_line[i][0] - '0') * 10 + (split_line[i][1] - '0');
    split_line[i] += LAT_DEG_DIGITS;
    double minutes = atof(split_line[i]) / 60;
    minutes += degrees;
    degrees = minutes * FLOAT_RMV_FACTOR;
    i++;

    /* +N/S- */
    if (*split_line[i] == 'S')
    {
        // negate result for south
        degrees = 0 - degrees;
    }
    location->latitude = degrees;
    i++;

    /* Longitude */
    degrees = (split_line[i][0] - '0') * 100 + (split_line[i][1] - '0') * 10 +
                      (split_line[i][2] - '0');
    split_line[i] += LONGIT_DEG_DIGITS;
    minutes = (double) atof(split_line[i]) / 60;
    minutes += degrees;
    degrees = minutes * FLOAT_RMV_FACTOR;
    i++;

    /* +E/W- */
    if (*split_line[i] == 'W')
    {
        // negate result for west
        degrees = 0 - degrees;
    }
    location->longitude = degrees;
    i++;

    /* fix quality */
    location->valid_fix = atoi(split_line[i]);
    i++;

    /* num of satellites */
    location->num_sats = atoi(split_line[i]);
    i++;

    /* hDOP */
    location->hdop = atoi(split_line[i]) * HDOP_FACTOR;
    i++;

    /* Altitude, meters, above sea level */
    if (split_line[i] != NULL)
    {
        location->altitude = atoi(split_line[i]);
        //location->altitude = atoi(split_line[i]) * ALT_FACTOR;
    }
    i++;

    // M of altitude
    i++;
    // Height of geoid
    i++;
    // time in seconds since last DGPS update
    i++;
    // DGPS station ID num
    i++;
    // checksum data, begins with *
    i++;

    return true;
}

/******************************************************************************
 *  Gets a buffer and a GPS_LOCATION_INFO struct and fills it with data
 *  from the buffer.
 *  @param buf - buffer for the GPS.
 *  @param location - the GPS info struct.
 *  @return true if successful, false otherwise.
 *****************************************************************************/
bool parseRawData(char* buf, GPS_LOCATION_INFO* location) {
    int result = 0;
    int tokens_array_size = GGA_FIELDS_NUM;
    char* tokens_array[tokens_array_size];
    /* GGA Line */
    if (strncmp(buf, GGA_PREFIX, PREFIX_LEN) == 0) {
        result = splitLineToFields(GGA_FIELDS_NUM, PREFIX_LEN, buf, tokens_array);
        if (result == true) {
            result = parseGGA(tokens_array, location);
        }
        return result;
	/* RMC Line */
    } else if (strncmp(buf, RMC_PREFIX, PREFIX_LEN) == 0) {
        result = splitLineToFields(RMC_FIELDS_NUM, PREFIX_LEN, buf, tokens_array);
        if (result == true) {
            result = parseRMC(tokens_array, location);
        }
        return result;
    } else {
        // unimportant line
        return false;
    }
}


/******************************************************************************
 * Updates location with information from GPSGetReadRaw.
 * @param location - the struct to be filled.
 * @return true if successful, false otherwise.
 *****************************************************************************/
bool GPSGetFixInformation(GPS_LOCATION_INFO *location){
    char buf[MAX_NMEA_LEN] = "";
    uint32_t bytes_read;

    bool result = false;
    while (!result) {
        // call GPSGetReadRaw
        bytes_read = GPSGetReadRaw(buf, MAX_NMEA_LEN);
        if (bytes_read > 0) {
            result = parseRawData(buf, location);
        }
    }

    return result;
}


/******************************************************************************
 * @brief Disable GPS connection.
 *****************************************************************************/
void GPSDisable() {
    if (GPS_INITIALIZED) {
    	SerialDisableGPS();
        GPS_INITIALIZED = false;
    }
}


/**
 * @brief this method will convert gps time to unix time in seconds.
 * @param gps_data - struct of GPS_LOCATION_INFO to extract time from.
 * @param unix_time - pointer to buffer where to store result.
 */
void GPSConvertFixtimeToUnixTime(GPS_LOCATION_INFO * gps_data, char * unix_time) {
	if (strlen(gps_data->fixtime) == 0) {
		memset(unix_time,'\0', GPS_LOC_INFO_TIME_BUF_SIZE);
	} else {
		struct tm current_time;
		char temp[3];
		memset(temp, '\0', 3);
		strncpy(temp, gps_data->fixtime, 2);
		current_time.tm_hour = atoi(temp);      //hours since midnight – [0, 23]
		strncpy(temp, &gps_data->fixtime[3], 2);
		current_time.tm_min = atoi(temp);    //minutes after the hour – [0, 59]
		strncpy(temp, &gps_data->fixtime[6], 2);
		current_time.tm_sec = atoi(temp);    //seconds after the minute – [0, 60]
		strncpy(temp, &gps_data->fixtime[9], 2);
		current_time.tm_mday = atoi(temp);       //day of the month – [1, 31]
		strncpy(temp, &gps_data->fixtime[12], 2);
		current_time.tm_mon = atoi(temp) - 1;  //months since January – [0, 11]
		strncpy(temp, &gps_data->fixtime[15], 2);
		current_time.tm_year = atoi(temp) + 100;//years since 1900

		time_t unix_time_stamp = mktime(&current_time);
		sprintf(unix_time, "%d", unix_time_stamp);
	}
}


/**
 * @brief prepare a predefined GPS payload based on inputs.
 * @param gps_data GPS LOCATION as received from GPS module
 * @param gps_payload where to store result
 * @return length of payload
 */
int GPSGetPayload(GPS_LOCATION_INFO * gps_data, char * iccid, char * unix_time, char * gps_payload, int maximal_payload_size) {
	memset(gps_payload, '\0', maximal_payload_size);

    //latitude= 31.7498445,longitude= 35.1838178,altitude=10,hdop=2,valid_fix=1,num_sats=4 1557086230000000000
    float lat_deg = (gps_data->latitude) / 10000000.0;
    float long_deg = (gps_data->longitude) / 10000000.0;

    //FORMAT "--data-binary gps,name=NetanelFayoumi_SapirElyovitch,ICCID=%s latitude=%f,longitude=%f,altitude=%ld,hdop=%u,valid_fix=%u,num_sats=%u %s"
	int	payload_len = sprintf(gps_payload, GPS_PAYLOAD_FORMAT,
								  iccid, lat_deg, long_deg, gps_data->altitude,
								  gps_data->hdop, gps_data->valid_fix, gps_data->num_sats, unix_time);
    return payload_len;
}


/**
 * @brief prepare a predefined GPS payload with highspeed tag based on inputs.
 * @param gps_data GPS LOCATION as received from GPS module
 * @param gps_payload where to store result
 * @return length of payload
 */
int GPSGetSPEEDPayload(GPS_LOCATION_INFO * gps_data, char * iccid, char * unix_time, bool highspeed, char * gps_payload, int maximal_payload_size) {
	memset(gps_payload, '\0', maximal_payload_size);

    float lat_deg = (gps_data->latitude) / 10000000.0;
    float long_deg = (gps_data->longitude) / 10000000.0;

	char * highspeed_val = (highspeed? "true" : "false");
    //GPS_PAYLOAD_SPEED_FORMAT "gps,name=NetanelFayoumi_SapirElyovitch,ICCID=%s,highspeed=%s latitude=%f,longitude=%f,altitude=%d,hdop=%u,valid_fix=%u,num_sats=%u %s000000000"
	int payload_len = sprintf(gps_payload, GPS_PAYLOAD_SPEED_FORMAT,
						  iccid, highspeed_val, lat_deg, long_deg, gps_data->altitude,
						  gps_data->hdop, gps_data->valid_fix, gps_data->num_sats, unix_time);
    return payload_len;
}


/**
 * get speed of movement between to GPS location
 * @return speed in Km/h
 */
double GPSGetSpeedOfLocations(GPS_LOCATION_INFO * old_loc, GPS_LOCATION_INFO * new_loc) {
	double old_lat_deg = (old_loc->latitude) / 10000000.0;
	double old_long_deg = (old_loc->longitude) / 10000000.0;
	double new_lat_deg = (new_loc->latitude) / 10000000.0;
	double new_long_deg = (new_loc->longitude) / 10000000.0;

	double diff_long = (MATH_PI/180) * (new_long_deg - old_long_deg);
	double diff_lat  = (MATH_PI/180) * (new_lat_deg - old_lat_deg);
	double a = pow(sin(diff_lat / 2), 2) + (cos(old_lat_deg) * cos(new_lat_deg) * pow(sin(diff_long / 2),2));
	double c = 2 * atan2(sqrt(a), sqrt(1-a));
	double distance = c * EARTH_RADIUS_KMS;

	char old_unix_time[GPS_LOC_INFO_TIME_BUF_SIZE];
	char new_unix_time[GPS_LOC_INFO_TIME_BUF_SIZE];

	GPSConvertFixtimeToUnixTime(old_loc, old_unix_time);
	GPSConvertFixtimeToUnixTime(new_loc, new_unix_time);

	int time_diff_sec = atoi(new_unix_time) - atoi(old_unix_time);
	double speed_kmps = distance / time_diff_sec;
	double speed_kph = speed_kmps * 3600.0;
	return speed_kph;
}
