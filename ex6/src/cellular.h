/**************************************************************************//**
 * @cellular.h
 * @brief
 * @version 0.0.1
 *  ***************************************************************************/
#ifndef IOT_CELLULAR_H
#define IOT_CELLULAR_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

extern bool DEBUG;

/****************************************************************************
 * 								DEFS
*****************************************************************************/
#define MAX_OPERATOR_NAME_SIZE 20
#define MAX_OPERATOR_TECH_SIZE 4

typedef struct __OPERATOR_INFO {
    char operatorName[MAX_OPERATOR_NAME_SIZE];
    int  operatorCode;
    char accessTechnology[MAX_OPERATOR_TECH_SIZE];
    int csq;
} OPERATOR_INFO;

#define MODEM_BAUD_RATE 115200
#define HTTP_POST_srvProfileId 6
#define WAIT_BETWEEN_CMDS_MS 100
#define ICCID_BUFFER_SIZE 23
#define CELL_PAYLOAD_FORMAT "cellular,name=NetanelFayoumi_SapirElyovitch,ICCID=%s %s %s000000000"
#define CELL_PAYLOAD_NO_TIME_FORMAT "cellular,name=NetanelFayoumi_SapirElyovitch,ICCID=%s %s "
//#define SPEED_PAYLOAD_NO_TIME_FORMAT "networkAnalyzer,name=NetanelFayoumi_SapirElyovitch,ICCID=%s %s "
#define PARTNER_MCC_MNC 42501
#define CELLCOM_MCC_MNC 42502
#define PELEPHONE_MCC_MNC 42503
#define MAX_CANDIDATE_CELL_OPS 3
#define iNA_MAX_CANDIDATE_CELL_OPS 6

/**************************************************************************//**
 * 							GLOBAL VARIABLES
*****************************************************************************/
enum MODE{REG_AUTOMATICALLY, SPECIFIC_OP, DEREGISTER};
enum ERROR_MODE {DISABLED, NUMERIC, VERBOSE};


/**
 * Initialize whatever is needed to start working with the cellular modem (e.g. the serial port).
 * @param port
 */
void CellularInit(char *port);


/**
 * Deallocate / close whatever resources CellularInit() allocated.
 */
void CellularDisable();


/**
 * Checks if the modem is responding to AT commands.
 * @return Return true if it does, returns false otherwise.
 */
bool CellularCheckModem(void);


/**
 * @param status
 * @return Returns false if the modem did not respond or responded with an error.
 * Returns true if the command was successful and the registration status was obtained
 * from the modem. In that case, the status parameter will be populated with the numeric
 * value of the <regStatus> field of the Ã¢â‚¬Å“+CREGÃ¢â‚¬ï¿½ AT command.

 */
bool CellularGetRegistrationStatus(int *status);


/**
 * @param csq
 * @return Returns false if the modem did not respond or responded with an error
 * (note, CSQ=99 is also an error!) Returns true if the command was successful and
 * the signal quality was obtained from the modem. In that case, the csq parameter
 * will be populated with the numeric value between -113dBm and -51dBm.

 */
bool CellularGetSignalQuality(int *csq);


/**
 * Forces the modem to register/deregister with a network.
 * If mode=0, sets the modem to automatically register with an operator
 * (ignores the operatorName parameter).
 * If mode=1, forces the modem to work with a specific operator, given in operatorName.
 * If mode=2, deregisters from the network (ignores the operatorName parameter).
 * See the AT+COPS=<mode>, command for more details.
 * @param mode
 * @param operatorName
 * @param act - access technology 0:2G 2:3G [in case mode = 1]
 * @return Returns true if the command was successful, returns false otherwise.
 */
bool CellularSetOperator(int mode, char *operatorNamem, int act);


/**
 * Forces the modem to search for available operators (see Ã¢â‚¬Å“+COPS=?Ã¢â‚¬ï¿½ command).
 * @param opList - a pointer to the first item of an array of type CELLULAR_OP_INFO, which is
 * allocated by the caller of this function.
 * @param maxops - The array contains a total of maxops items.
 * @param numOpsFound - numOpsFound is allocated by the caller and will contain the number
 * of operators found and populated into the opList.
 * @return Returns false if an error occurred or no operators found.
 * Returns true and populates opList and opsFound if the command succeeded.
 */
bool CellularGetOperators(OPERATOR_INFO *opList, int maxops, int *numOpsFound);


bool CellularGetSpecificOperators(OPERATOR_INFO *opList, int maxops, int *numOpsFound);


/**
 * Initialize an internet connection profile (AT^SICS) with inactTO=inact_time_sec and conType= GPRS0
 * and apn="postm2m.lu".
 * @param inact_time_sec
 * @return
 */
bool CellularSetupInternetConnectionProfile(int inact_time_sec);


/**
 * Send an HTTP POST request. Opens and closes the socket.
 * @param URL
 * @param payload
 * @param payload_len
 * @param response
 * @param response_max_len
 * @return The return value indicates the number of read bytes in response.
 *         If there is any kind of error, return -1.
 */
int CellularSendHTTPPOSTRequest(char *URL, char *payload, int payload_len, char *response, int response_max_len);


/**
 * Returns additional information on the last error occurred during CellularSendHTTPPOSTRequest.
 * The response includes urcInfoId, then comma , then urcInfoText, e.g. "200,Socket-Error:3".
 * @param errmsg
 * @param errmsg_max_len
 * @return
 */
int CellularGetLastError(char *errmsg, int errmsg_max_len);


/**
 * Will send AT+CCID cmd and parse the response to return the sim ICCID
 * @param iccid buffer to storce ICCID
 * @return ICCID length
 */
int CellularGetICCID(char * iccid);


/**
 *
 * @param opList OPERATOR_INFO array of successfully registerd operators.
 * @param num_of_ops num of operators in array
 * @param unix_time unix time provided from gps
 * @param cell_payload buffer to store payload into.
 * @return length of payload copied to buffer.
 */
int CellularGetPayload(OPERATOR_INFO *opList, int candidate_op_idx, char * iccid, char * unix_time, char * cell_payload, int maximal_payload_size);


/**
 * This method will return the radio Access Technology for the given operator info struct
 * @param operator - OPERATOR_INFO struct of given operator.
 * @return act of the operator. 0 for 2G(GSM), 2 for 3G(UTRAN). will return -1 in case of error.
 */
int CellularGetAcT(OPERATOR_INFO * operator);


/**
 * This method will close Internet service of given srvProfileId
 * @return True if succeeded to close the service, false otherwise.
 */
bool inetServiceClose();


/**
 * @brief Delays number of msTick Systicks (typically 1 ms)
 * @param dlyTicks Number of ticks to delay
 */
void CellularDelay(uint32_t dlyTicks);


#endif //IOT_CELLULAR_H
