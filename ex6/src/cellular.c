#include "cellular.h"
#include "serial_io_usart.h"


/****************************************************************************
 * 								DECLARATIONS
*****************************************************************************/
void sendATcommand(unsigned char* command, unsigned int command_size);
bool waitForOK();
bool waitForATresponse(unsigned char ** token_array, unsigned char * expected_response, unsigned int response_size, int max_responses, unsigned int timeout_ms);
int splitBufferToResponses(unsigned char * buffer, unsigned char ** tokens_array, int max_tokens);
int splitCopsResponseToOpsTokens(unsigned char * cops_response, OPERATOR_INFO *opList, int max_ops, bool specific_operators);
bool splitOpTokensToOPINFO(unsigned char * op_token, OPERATOR_INFO *opInfo, bool specific_operators);
int getSISURCs(unsigned char ** token_array, int max_urcs, unsigned int timeout_ms);

/*****************************************************************************
 * 								DEFS
*****************************************************************************/
//#define MAX_INCOMING_BUF_SIZE 1000//todo
#define MAX_INCOMING_BUF_SIZE 1600
#define MAX_OP_TOKEN_SIZE 50
#define MAX_AT_CMD_LEN 100
#define GENERAL_RECV_TIMEOUT_MS 10000
#define GENERAL_RECV_DLY_TIMEOUT_MS 15000
#define SIS_RECV_TIMEOUT_MS 30000
#define GET_OPS_TIMEOUT_MS 120000
#define MAX_conProfileId 5


#define SISS_CMD_HTTP_GET 0
#define SISS_CMD_HTTP_POST 1
#define SISS_CMD_HTTP_HEAD 2

#define RESPONSE_TOKENS_SIZE 10

#define SAMPLE_PACKET "iNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKET"

/*****************************************************************************
 * 							GLOBAL VARIABLES
*****************************************************************************/
static bool CELLULAR_INITIALIZED = false;
unsigned char command_to_send_buffer[MAX_INCOMING_BUF_SIZE] = "";
unsigned char speed_packet[] = SAMPLE_PACKET;

unsigned char AT_CMD_SUFFIX[] = "\r\n";
// AT_COMMANDS
unsigned char AT_CMD_ECHO_OFF[] = "ATE0\r\n";
unsigned char AT_CMD_AT[] = "AT\r\n";
unsigned char AT_CMD_COPS_TEST[] = "AT+COPS=?\r\n";
const unsigned char AT_CMD_COPS_WRITE_PREFIX[] = "AT+COPS=";
unsigned char AT_CMD_CREG_READ[] = "AT+CREG?\r\n";
unsigned char AT_CMD_CSQ[] = "AT+CSQ\r\n";
unsigned char AT_CMD_SICS_WRITE_PRFX[] = "AT^SICS=";
unsigned char AT_CMD_SISS_WRITE_PRFX[] = "AT^SISS=";
unsigned char AT_CMD_SISO_WRITE_PRFX[] = "AT^SISO=";
unsigned char AT_CMD_SISR_WRITE_PRFX[] = "AT^SISR=";
unsigned char AT_CMD_SISW_WRITE_PRFX[] = "AT^SISW=";
unsigned char AT_CMD_SISC_WRITE_PRFX[] = "AT^SISC=";
unsigned char AT_CMD_SISE_WRITE_PRFX[] = "AT^SISE=";
unsigned char AT_CMD_CCID_READ[] = "AT+CCID?";
unsigned char AT_CMD_SISX_WRITE_PRFX[] = "AT^SISX=";
unsigned char AT_CMD_SHUTDOWN[] = "AT^SMSO\r\n";

// AT RESPONDS
unsigned char AT_RES_OK[] = "OK";
unsigned char AT_RES_ERROR[] = "ERROR";
unsigned char AT_RES_CME[] = "+CME";
unsigned char AT_RES_SYSSTART[] = "^SYSSTART";
unsigned char AT_RES_PBREADY[] = "+PBREADY";
unsigned char AT_URC_SHUTDOWN[] = "^SHUTDOWN";

// Internet connection profile identifier. 0..5
// The <conProfileId> identifies all parameters of a connection profile,
// and, when a service profile is created with AT^SISS the <conProfileId>
// needs to be set as "conId" value of the AT^SISS parameter <srvParmTag>.
int conProfileId = -1;


/**
 * Initialize whatever is needed to start working with the cellular modem (e.g. the serial port).
 * @param port
 */
void CellularInit(char *port){
    printf("Initializing Cellular modem..\n");

    if (!CELLULAR_INITIALIZED) {
        CELLULAR_INITIALIZED = SerialInitCellular(port, MODEM_BAUD_RATE);
        if (!CELLULAR_INITIALIZED) {
            printf("Initialization FAILED.\n");
            exit(EXIT_FAILURE);
        }

        // check modem responded with ^+PBREADY
        unsigned char * token_array[5] = {};
        while(!waitForATresponse(token_array, AT_RES_PBREADY, sizeof(AT_RES_PBREADY) - 1, 5, GENERAL_RECV_TIMEOUT_MS));

        bool echo_off = false;
        printf("Setting echo off...");
        while (!echo_off) {
            // set modem echo off
            sendATcommand(AT_CMD_ECHO_OFF, sizeof(AT_CMD_ECHO_OFF) - 1);

            // verify echo off
            echo_off = waitForOK();
        }

        printf("successfully.\n");
        printf("Cellular modem initialized successfully.\n");
    }
}


/**
 * Deallocate / close whatever resources CellularInit() allocated.
 */
void CellularDisable(){
    if (CELLULAR_INITIALIZED) {
        // shut down modem
        sendATcommand(AT_CMD_SHUTDOWN, sizeof(AT_CMD_SHUTDOWN) - 1);

        // Disable serial connection
        SerialDisableCellular();
        CELLULAR_INITIALIZED = false;
    }
}


/**
 * Checks if the modem is responding to AT commands.
 * @return Return true if it does, returns false otherwise.
 */
bool CellularCheckModem(void){
    printf("Checks that the modem is responding...");
    if (CELLULAR_INITIALIZED) {
        // send "hello" (AT\r\n)
        sendATcommand(AT_CMD_AT, sizeof(AT_CMD_AT) - 1);

        // verify modem response
        if (waitForOK()) {
            printf("modem is responsive.\n");
            return true;
        } else {
            printf("modem is NOT responsive!\n");
            return false;
        }
    } else {
        printf("modem wasn't initialized!\n");
        return false;
    }
}


/**
 * @param status
 * @return Returns false if the modem did not respond or responded with an error.
 * Returns true if the command was successful and the registration status was obtained
 * from the modem. In that case, the status parameter will be populated with the numeric
 * value of the <regStatus> field of the "+CREG" AT command.

 */
bool CellularGetRegistrationStatus(int *status){
    // AT+CREG?
    // response: +CREG: <Mode>, <regStatus>[, <netLac>, <netCellId>[, <AcT>]] followed by OK
    sendATcommand(AT_CMD_CREG_READ, sizeof(AT_CMD_CREG_READ) - 1);

	unsigned char * token_array[5] = {};
	if (waitForATresponse(token_array, AT_RES_OK, sizeof(AT_RES_OK) - 1, 5, GENERAL_RECV_TIMEOUT_MS)) {
		// "+CREG: <Mode>,<regStatus>"
		char * token;
		token = strtok(token_array[0], ",");
		token = strtok(NULL, ",");
		*status = atoi(token);
		return true;
	}
    return false;
}


/**
 * @param csq
 * @return Returns false if the modem did not respond or responded with an error
 * (note, CSQ=99 is also an error!)
 * Returns true if the command was successful and the signal quality was obtained from the modem.
 * In that case, the csq parameter will be populated with the numeric value between -113dBm and
 * -51dBm.

 */
bool CellularGetSignalQuality(int *csq) {
    if (!CELLULAR_INITIALIZED) {
        return false;
    }
    // AT+CSQ
    // response : +CSQ <rssi>,<ber> followed by OK
    // rssi: 0,1,2-30,31,99, ber: 0-7,99unknown

    // send AT+CSQ
    sendATcommand(AT_CMD_CSQ, sizeof(AT_CMD_CSQ) - 1);
	unsigned char * token_array[5] = {};
	if (waitForATresponse(token_array, AT_RES_OK, sizeof(AT_RES_OK) - 1, 5, GENERAL_RECV_TIMEOUT_MS)) {
		char * token;
		token = strtok(&(token_array[0])[5], ",");
		if (strcmp(token, "99") != 0) {
			// -113 + 2* rssi
			int rssi = atoi(token);
			*csq = -113 + (2 * rssi);
			return true;
		}
	}

    return false;


}


/**
 * Forces the modem to register/deregister with a network.
 * If mode=0, sets the modem to automatically register with an operator
 * (ignores the operatorName parameter).
 * If mode=1, forces the modem to work with a specific operator, given in operatorName.
 * If mode=2, deregisters from the network (ignores the operatorName parameter).
 * See the "+COPS=<mode>,..." command for more details.
 * @param mode
 * @param operatorName
 * @return Returns true if the command was successful, returns false otherwise.
 */
bool CellularSetOperator(int mode, char *operatorName, int act){
    // 1: Manual operator selection.
    // Write command requires <opName> in numeric format, i.e. <format> shall be 2.
    // 2: Manually deregister from network and remain unregistered until <mode>=0 or
    // 1 or 4 is selected. Please keep in mind that the AT+COPS write command is SIM PIN protected.
    // When SIM PIN is disabled or after SIM PIN authentication has completed and
    // "+PBREADY" URC has shown up the power up default <mode>=2 automatically
    // changes to <mode>=0, causing the ME to select a network.
    memset(command_to_send_buffer, '\0',MAX_INCOMING_BUF_SIZE);
    if (mode == REG_AUTOMATICALLY) {
    	int cmd_size = sprintf(command_to_send_buffer, "%s%d%s", AT_CMD_COPS_WRITE_PREFIX, mode, AT_CMD_SUFFIX);
    	unsigned char * token_array[5] = {};
    	return waitForATresponse(token_array, AT_RES_OK, sizeof(AT_RES_OK) - 1, 5, GET_OPS_TIMEOUT_MS);

    } else if (mode == DEREGISTER) {
        int cmd_size = sprintf(command_to_send_buffer, "%s%d%s", AT_CMD_COPS_WRITE_PREFIX, mode, AT_CMD_SUFFIX);

        // send command
        sendATcommand(command_to_send_buffer, cmd_size);

        // wait for OK
        return waitForOK();

    } else if (mode == SPECIFIC_OP) {
        int cmd_size = sprintf(command_to_send_buffer, "%s%d,0,%s,%d%s", AT_CMD_COPS_WRITE_PREFIX, mode, operatorName, act, AT_CMD_SUFFIX);
        sendATcommand(command_to_send_buffer, cmd_size);

        return waitForOK();

    } else {
        return false;
    }
}


/**
 * Forces the modem to search for available operators (see "+COPS=?" command).
 * @param opList - a pointer to the first item of an array of type CELLULAR_OP_INFO, which is
 * allocated by the caller of this function.
 * @param maxops - The array contains a total of maxops items.
 * @param numOpsFound - numOpsFound is allocated by the caller and will contain the number
 * of operators found and populated into the opList.
 * @return Returns false if an error occurred or no operators found.
 * Returns true and populates opList and opsFound if the command succeeded.
 */
bool CellularGetOperators(OPERATOR_INFO *opList, int maxops, int *numOpsFound){
    // send AT+COPS=?
    sendATcommand(AT_CMD_COPS_TEST, sizeof(AT_CMD_COPS_TEST) - 1);

    unsigned char * token_array[10] = {};
    if (waitForATresponse(token_array, AT_RES_OK, sizeof(AT_RES_OK) - 1, 10, GET_OPS_TIMEOUT_MS)) {

        char operators[MAX_INCOMING_BUF_SIZE];
        memset(operators, '\0', MAX_INCOMING_BUF_SIZE);

        // this remove "+COPS: "
        strncpy(operators, &(token_array[0])[7], strlen(token_array[0]) - 7);

        int num_of_found_ops = splitCopsResponseToOpsTokens(operators, opList, maxops, false);
        // fill results
        if (num_of_found_ops != 0) {
            *numOpsFound = num_of_found_ops;
            return true;
        }
    }
    return false;
}


bool CellularGetSpecificOperators(OPERATOR_INFO *opList, int maxops, int *numOpsFound){
    // send AT+COPS=?
    sendATcommand(AT_CMD_COPS_TEST, sizeof(AT_CMD_COPS_TEST) - 1);

    unsigned char * token_array[10] = {};
    if (waitForATresponse(token_array, AT_RES_OK, sizeof(AT_RES_OK) - 1, 10, GET_OPS_TIMEOUT_MS)) {

        char operators[MAX_INCOMING_BUF_SIZE];
        memset(operators, '\0', MAX_INCOMING_BUF_SIZE);

        // this remove "+COPS: "
        strncpy(operators, &(token_array[0])[7], strlen(token_array[0]) - 7);

        int num_of_found_ops = splitCopsResponseToOpsTokens(operators, opList, maxops, true);
        // fill results
        if (num_of_found_ops != 0) {
            *numOpsFound = num_of_found_ops;
            return true;
        }
    }
    return false;
}


/**
 * This method will return
 * @param command - command to send
 * @param command_size - length of command
 * @return true if send the command serial. otherwise false.
 */
void sendATcommand(unsigned char* command, unsigned int command_size) {
	if (DEBUG) { printf("\n%s\n", command); }
	if (CELLULAR_INITIALIZED){
		while(!SerialSendCellular(command, command_size));
		CellularDelay(WAIT_BETWEEN_CMDS_MS);
	}
}

/**
 * This method will wait for OK as response
 * @return true iff OK was the last token received
 */
bool waitForOK() {
    unsigned char incoming_buffer[MAX_INCOMING_BUF_SIZE] = "";
    unsigned char * token_array[10] = {};
    unsigned int bytes_received = 0;
    int num_of_tokens = 0;

	memset(incoming_buffer, '\0', MAX_INCOMING_BUF_SIZE);
	bytes_received = SerialRecvCellular(incoming_buffer, MAX_INCOMING_BUF_SIZE, GENERAL_RECV_TIMEOUT_MS);
	if (DEBUG) { printf("\n%s\n", incoming_buffer); }

	if (bytes_received > 0) {
		num_of_tokens = splitBufferToResponses(incoming_buffer, token_array, 10);

		if (num_of_tokens > 0) {
			return (memcmp(token_array[num_of_tokens-1], AT_RES_OK, sizeof(AT_RES_OK) - 1) == 0);
		}
	}

	return false;
}


/**
 * This method will wait for specific response
 * @return true iff the given response was the last token received
 */
bool waitForATresponse(unsigned char ** token_array, unsigned char * expected_response,
        unsigned int response_size, int max_responses, unsigned int timeout_ms) {

	unsigned char incoming_buffer[MAX_INCOMING_BUF_SIZE] = "";
	unsigned int bytes_received = 0;
	int num_of_tokens = 0;

	memset(incoming_buffer, '\0', MAX_INCOMING_BUF_SIZE);
	bytes_received = SerialRecvCellular(incoming_buffer, MAX_INCOMING_BUF_SIZE, timeout_ms);
	if (DEBUG) { printf("\n%s\n", incoming_buffer); }
	if (bytes_received == 0) {
		token_array[0] = NULL;
		if (DEBUG) { printf("bytes = 0\n"); }
		return false;
	}

	num_of_tokens = splitBufferToResponses(incoming_buffer, token_array, max_responses);

	if (num_of_tokens > 0) {
		if (memcmp(token_array[num_of_tokens-1], expected_response, response_size) == 0) {
			if (DEBUG) { printf("token_array = %s\n", token_array[num_of_tokens - 1]); }
			return true;
		} else if ((memcmp(token_array[num_of_tokens-1], AT_RES_ERROR, 5) == 0) ||
				   (memcmp(token_array[num_of_tokens-1], AT_RES_CME, 4) == 0)) {
			if (DEBUG) { printf("token_array = %s\n", token_array[num_of_tokens - 1]); }
			return false;
		}
	}

	if (DEBUG) { printf("num of tokens %d\n", num_of_tokens); }
	token_array[0] = NULL;
	return false;
}


int getSISURCs(unsigned char ** token_array, int max_urcs, unsigned int timeout_ms) {
    unsigned char incoming_buffer[MAX_INCOMING_BUF_SIZE] = "";

    int num_of_tokens = 0;

	memset(incoming_buffer, '\0', MAX_INCOMING_BUF_SIZE);
	unsigned int bytes_received = SerialRecvCellular(incoming_buffer, MAX_INCOMING_BUF_SIZE, timeout_ms);

	if (bytes_received != 0) {
		num_of_tokens = splitBufferToResponses(incoming_buffer, token_array, max_urcs);
	}

    return num_of_tokens;
}


/**
 *
 * @param sis_result the suffix of "^SIS: " urc
 * @param urcCause pointer to save urcCause
 * @param urcInfoId pointer to save urcInfoId
 * @return false if found error, otherwise true.
 */
bool parseSISresponse(char * sis_result, int * urcCause, int * urcInfoId) {
    // ^SIS: <srvProfileId>, <urcCause>[, [<urcInfoId>][, <urcInfoText>]]
    char * temp_token = strtok(sis_result, ",");   //<srvProfileId>,
    temp_token = strtok(NULL, ",");         //<urcCause>
    *urcCause = atoi(temp_token);
    temp_token = strtok(NULL, ",");         //<urcInfoId>
    *urcInfoId = atoi(temp_token);

    if (*urcCause == 0) {
    	if (*urcInfoId == 200) {
    		temp_token = strtok(NULL, ",");         //<urcInfoText>>
    		return (strcmp(temp_token, "HTTP-CODE: 204") == 0);

    	} else if ((1 <= *urcInfoId) && (*urcInfoId <= 2000)) {
    		return false;
    	}
    }
    return true;
}

/**
 * parse "^SISR: " urc or "^SISW: " urc suffixes
 * @param sisrw_result "^SISR: " urc or "^SISW: " urc suffixes
 * @return urcCauseID
 */
int parseSISRWresponse(char * sisrw_result) {
    //^SISR: <srvProfileId>, <urcCauseId>
    char * urcCauseId = "";
    strcpy(urcCauseId, &sisrw_result[2]);
    return atoi(urcCauseId);
}


/**
 * this method parse SIS URCs
 * @param token_array tokens of SIS responses
 * @param received_urcs num of tokens.
 * @return false if SIS URC reflecting error, true otherwise
 */
bool parseSISURCs(unsigned char ** token_array, int received_urcs, char * urc_read_buffer) {
//    const char urc_delimiter[] = ": ";
    for (int urc_idx = 0; urc_idx < received_urcs; urc_idx++) {
        char urc_prefix[100] = "";
        char urc_result[100] = "";

        if (strcmp(token_array[urc_idx], "ERROR") == 0) {
            return false;
        } else if (strcmp(token_array[urc_idx], "OK") == 0) {
			continue;
        } else if (strncmp(token_array[urc_idx], "^SIS: ", 6) == 0) {
        	strncpy(urc_prefix, token_array[urc_idx], 4);
        	strncpy(urc_result, &(token_array[urc_idx])[6], 100);
        	strcpy(urc_read_buffer, &urc_result[2]);

        	int urcCause, urcInfoId;
			if (parseSISresponse(urc_result, &urcCause, &urcInfoId)) {
				return true;
			}
        }
    }
    return true;
}


/**
 * This method will setup an Http service profile
 * @param URL dest url to post/get requests to/from
 * @param payload the request payload
 * @param payload_len
 * @return true iff setup suceeded
 */
bool inetServiceSetupProfile(char *URL, char *payload, int payload_len) {
    // AT^SISS=<srvProfileId>, <srvParmTag>, <srvParmValue>

    // AT^SISS=6,"SrvType","Http"
    int cmd_size = sprintf(command_to_send_buffer, "%s%d,\"SrvType\",\"Http\"%s",
                           AT_CMD_SISS_WRITE_PRFX, HTTP_SRV_PROFILE_ID, AT_CMD_SUFFIX);
    // send command and check response OK/ERROR
    sendATcommand(command_to_send_buffer, cmd_size);
    if (!waitForOK()) { return false; }

    // AT^SISS=6,"conId","<conProfileId>"
    cmd_size = sprintf(command_to_send_buffer, "%s%d,\"conId\",\"%d\"%s",
                       AT_CMD_SISS_WRITE_PRFX, HTTP_SRV_PROFILE_ID, conProfileId, AT_CMD_SUFFIX);
    // send command and check response OK/ERROR
    sendATcommand(command_to_send_buffer, cmd_size);
    if (!waitForOK()) { return false; }

    // AT^SISS=6,"address","<url>"
    cmd_size = sprintf(command_to_send_buffer, "%s%d,\"address\",\"%s\"%s",
                       AT_CMD_SISS_WRITE_PRFX, HTTP_SRV_PROFILE_ID, URL, AT_CMD_SUFFIX);
    // send command and check response OK/ERROR
    sendATcommand(command_to_send_buffer, cmd_size);
    if (!waitForOK()) { return false; }


    // AT^SISS=6,"cmd","1"
    cmd_size = sprintf(command_to_send_buffer, "%s%d,\"cmd\",\"%d\"%s",
                       AT_CMD_SISS_WRITE_PRFX, HTTP_SRV_PROFILE_ID, SISS_CMD_HTTP_POST, AT_CMD_SUFFIX);
    // send command and check response OK/ERROR
    sendATcommand(command_to_send_buffer, cmd_size);
    if (!waitForOK()) { return false; }


    // AT^SISS=6,"hcContLen","0"
    // If "hcContLen" = 0 then the data given in the "hcContent" string will be posted
    // without AT^SISW required.
    cmd_size = sprintf(command_to_send_buffer, "%s%d,\"hcContLen\",\"%d\"%s",
                       AT_CMD_SISS_WRITE_PRFX, HTTP_SRV_PROFILE_ID, 0, AT_CMD_SUFFIX);
    // send command and check response OK/ERROR
    sendATcommand(command_to_send_buffer, cmd_size);
    if (!waitForOK()) { return false; }


    //AT^SISS=6,"hcContent","HelloWorld!"
    cmd_size = sprintf(command_to_send_buffer, "%s%d,\"hcContent\",\"%s\"%s",
                       AT_CMD_SISS_WRITE_PRFX, HTTP_SRV_PROFILE_ID, payload, AT_CMD_SUFFIX);
    // send command and check response OK/ERROR
   sendATcommand(command_to_send_buffer, cmd_size);
   return waitForOK();
}

/**
 * This method will open Internet service of given srvProfileId
 * @param serviceID
 * @return True if succeeded to open the service, false otherwise.
 */
bool serviceProfileOpen(int serviceID) {
    //AT^SISO=<serviceID>
    int cmd_size = sprintf(command_to_send_buffer, "%s%d%s", AT_CMD_SISO_WRITE_PRFX, serviceID, AT_CMD_SUFFIX);
    sendATcommand(command_to_send_buffer, cmd_size);
    bool result = waitForOK();
    return result;
}


/**
 * This method will close Internet service of given srvProfileId
 * @param serviceID
 * @return True if succeeded to close the service, false otherwise.
 */
bool serviceProfileClose(int serviceID) {
    //AT^SISC=<serviceID>
    int cmd_size = sprintf(command_to_send_buffer, "%s%d%s", AT_CMD_SISC_WRITE_PRFX, serviceID, AT_CMD_SUFFIX);
    sendATcommand(command_to_send_buffer, cmd_size);

    bool result = waitForOK();
	return result;
}


int splitBufferToResponses(unsigned char * buffer, unsigned char ** tokens_array, int max_tokens) {
    unsigned char temp_buffer[MAX_INCOMING_BUF_SIZE];
    memcpy(temp_buffer, buffer, MAX_INCOMING_BUF_SIZE);

    const char delimiter[] = "\r\n";
    char * token;
    int i=0;
    token = strtok(temp_buffer, delimiter);
    while (token != NULL && i < max_tokens) {
        tokens_array[i++] = (unsigned char *)token;
        token = strtok(NULL, delimiter);
    }
    return i;
}


int splitCopsResponseToOpsTokens(unsigned char *cops_response, OPERATOR_INFO *opList, int max_ops, bool specific_operators) {
    // [list of supported (<opStatus>, long alphanumeric <opName>, short alphanumeric <opName>,
    //numeric <opName>, <AcT>)s ]
    const char op_start_delimiter[] = "(";
    char * operator_token;
    int op_index = 0;
    operator_token = strtok(cops_response, op_start_delimiter);

    unsigned char * operators_tokens[max_ops];

    for (int i=0; i < max_ops; i++) {
        operators_tokens[i] = (unsigned char *) malloc(MAX_OP_TOKEN_SIZE);
    }

    while (operator_token != NULL) {
        if (op_index >= max_ops) {
            // found max operators
            break;
        }

        // set char[] for helper method
        strcpy(operators_tokens[op_index], operator_token);
        operator_token = strtok(NULL, op_start_delimiter);
        op_index++;
    }

    int parsed_ops_index = 0;
    for (; op_index > 0; op_index--) {
        if (splitOpTokensToOPINFO(operators_tokens[parsed_ops_index], &opList[parsed_ops_index], specific_operators)) {
            parsed_ops_index++;
        }

    }

    for (int i=0; i < max_ops; i++) {
		free(operators_tokens[i]);
	}
    return parsed_ops_index;
}


bool splitOpTokensToOPINFO(unsigned char * op_token, OPERATOR_INFO *opInfo, bool specific_operators) {
    const char op_field_delimiter[] = ",";
    int field_index = 0;
    unsigned char * field_token = strtok(op_token, op_field_delimiter);

    while (field_token != NULL) {
        if (field_index == 1) {
            // long alphanumeric <opName>
            memset(opInfo->operatorName, '\0', 20);
            strcpy(opInfo->operatorName, field_token);

        } else if (field_index == 3) {
            //numeric <opName>
            opInfo->operatorCode = atoi(&field_token[1]);
            if (specific_operators &&((opInfo->operatorCode < PARTNER_MCC_MNC) || (opInfo->operatorCode > PELEPHONE_MCC_MNC))) {
            	opInfo->operatorCode = 0;
            	return false;
            }

        } else if (field_index == 4) {
            memset(opInfo->accessTechnology, '\0', 4);
            if (field_token[0] == '0') {
                strcpy(opInfo->accessTechnology, "2G");
            } else if (field_token[0] == '2') {
                strcpy(opInfo->accessTechnology, "3G");
            } else {
                return false;
            }
        }

        field_token = (unsigned char *) strtok(NULL, op_field_delimiter);
        field_index++;
    }
    opInfo->csq = 99;
    return true;
}


/**
 * Initialize an internet connection profile (AT^SICS) with inactTO=inact_time_sec and conType= GPRS0
 * and apn="postm2m.lu".
 * @param inact_time_sec
 * @return true if inet connection profile set successfully, false otherwise
 */
bool CellularSetupInternetConnectionProfile(int inact_time_sec) {
	printf("Setup Internet connection..");
    int conProfileId_cand;

    for (conProfileId_cand = 0; conProfileId_cand <= MAX_conProfileId; conProfileId_cand++) {
        memset(command_to_send_buffer, '\0',MAX_INCOMING_BUF_SIZE);

        // AT^SICS=0,conType,GPRS0
        int cmd_size = sprintf(command_to_send_buffer, "%s%d,conType,GPRS0%s", AT_CMD_SICS_WRITE_PRFX, conProfileId_cand, AT_CMD_SUFFIX);
        sendATcommand(command_to_send_buffer, cmd_size);

        if (!waitForOK()) {
            continue;
        }

        memset(command_to_send_buffer, '\0', MAX_INCOMING_BUF_SIZE);

        // AT^SICS=0,"inactTO", "20"
        cmd_size = sprintf(command_to_send_buffer, "%s%d,\"inactTO\", \"%d\"%s",
                           AT_CMD_SICS_WRITE_PRFX, conProfileId_cand, inact_time_sec, AT_CMD_SUFFIX);
        sendATcommand(command_to_send_buffer, cmd_size);

        if (!waitForOK()) {
            continue;
        }

        memset(command_to_send_buffer, '\0', MAX_INCOMING_BUF_SIZE);

        // AT^SICS=0,apn,"postm2m.lu"
        cmd_size = sprintf(command_to_send_buffer, "%s%d,apn,\"postm2m.lu\"%s",
                           AT_CMD_SICS_WRITE_PRFX, conProfileId_cand, AT_CMD_SUFFIX);
        sendATcommand(command_to_send_buffer, cmd_size);

        if (waitForOK()) {
            conProfileId = conProfileId_cand;
            printf("success\n");
            return true;
        }
    }
    printf("failed.\n");
    return false;
}


/**
 * Send an HTTP POST request. Opens and closes the socket.
 * @param URL is the complete address of the page we are posting to, e.g. “https://helloworld.com/mystuf/thispagewillreceivemypost?andadditional=stuff”
 * @param payload payload is everything that is sent as HTTP POST content.
 * @param payload_len payload_len is the length of the payload parameter.
 * @param response response is a buffer that holds the content of the HTTP response.
 * @param response_max_len response_max_len is the response buffer length.
 * @return The return value indicates the number of read bytes in response.
 *         If there is any kind of error, return -1.
 */
int CellularSendHTTPPOSTRequest(char *URL, char *payload, int payload_len, char *response, int response_max_len) {
	memset(response, '\0', response_max_len);

	printf("Sending HTTP POST request..");
    // sanity check: conProfileId exists
    if (conProfileId == -1) { return -1;}

    if (!inetServiceSetupProfile(URL, payload, payload_len)) {
        return -1;
    }

    if (!serviceProfileOpen(HTTP_SRV_PROFILE_ID)) {
        return -1;
    }

    // need to handle ^SIS URCs
    // ^SIS: 6,0,2200,"Http 51.143.141.28:8086"
    // ^SIS: 6,0,200,"HTTP-CODE: 204"
    char * urc_read_buff = "";
	unsigned char * tokens_array[5] = {};
	int received_urcs = getSISURCs(tokens_array, 5, SIS_RECV_TIMEOUT_MS);
	if (!parseSISURCs(tokens_array, received_urcs, urc_read_buff)) {
		return -1;
	}

    if (!serviceProfileClose(HTTP_SRV_PROFILE_ID)) {
        return -1;
    }

    // all went fine
    return true;
}


/**
 * Returns additional information on the last error occurred during CellularSendHTTPPOSTRequest.
 * The response includes urcInfoId, then comma (‘,’), then urcInfoText, e.g. “200,Socket-Error:3”.
 * @param errmsg
 * @param errmsg_max_len
 * @return
 */
int CellularGetLastError(char *errmsg, int errmsg_max_len) {

    // AT^SISE=<srvProfileId>
    int cmd_size = sprintf(command_to_send_buffer, "%s%d%s",
                           AT_CMD_SISE_WRITE_PRFX, HTTP_SRV_PROFILE_ID, AT_CMD_SUFFIX);
    // send command and check response OK/ERROR
    sendATcommand(command_to_send_buffer, cmd_size);

    char * urc_read_buff = "";
    unsigned char * tokens_array[10] = {};
    // response:
    // ^SISE: <srvProfileId>, <infoID>[, <info>]
    int received_urcs = getSISURCs(tokens_array, 10, GENERAL_RECV_DLY_TIMEOUT_MS);
    if (!parseSISURCs(tokens_array, received_urcs, urc_read_buff)) {
        return -1;
    }

    // all went fine
    strncpy(errmsg, urc_read_buff, errmsg_max_len);

    if (errmsg_max_len < strlen(urc_read_buff)) {
        return errmsg_max_len;
    } else {
        return strlen(urc_read_buff);
    }
}


/**
 * Will send AT+CCID cmd and parse the response to return the sim ICCID
 * @param iccid buffer to store ICCID MIN LENGTH of ICCID_BUFFER_SIZE
 * @return ICCID length
 */
int CellularGetICCID(char * iccid) {

	memset(iccid, '\0', ICCID_BUFFER_SIZE);
    //AT+CCID?
    int cmd_size = sprintf(command_to_send_buffer, "%s%s", AT_CMD_CCID_READ, AT_CMD_SUFFIX);
    unsigned char * tokens_array[5] = {};
    do {
    	sendATcommand(command_to_send_buffer, cmd_size);
    } while (!waitForATresponse(tokens_array, AT_RES_OK, sizeof(AT_RES_OK) - 1, 5, GENERAL_RECV_TIMEOUT_MS));



	// +CCID: <ICCID> or OK or ERROR
	const char urc_delimiter[] = ": ";
	for (int urc_idx = 0; urc_idx < 5; urc_idx++) {
		if (strlen(tokens_array[urc_idx]) == 0) {
			continue;
		}
		if (strcmp(tokens_array[urc_idx], "ERROR") == 0) {
			return 0;
		} else if (*tokens_array[urc_idx] == '+') {
			char * temp_token = "";
			temp_token = strtok(tokens_array[urc_idx], urc_delimiter);
			temp_token = strtok(NULL, urc_delimiter);
			int token_str_len = strlen(temp_token);
			strncpy(iccid, temp_token, token_str_len + 1);
			return token_str_len;
		}
	}
	return 0;
}


/**
 *
 * @param opList OPERATOR_INFO array of successfully registerd operators.
 * @param num_of_ops num of operators in array
 * @param unix_time unix time provided from gps
 * @param cell_payload buffer to store payload into.
 * @return length of payload copied to buffer.
 */
int CellularGetPayload(OPERATOR_INFO *opList, int candidate_op_idx, char * iccid, char * unix_time, char * cell_payload, int maximal_payload_size) {
	memset(cell_payload, '\0', maximal_payload_size);
    char operators_payload[300] = "";
    memset(operators_payload, '\0', 300);

    for (int op_idx = 1; op_idx <= MAX_CANDIDATE_CELL_OPS; op_idx++) {
    	if (opList[candidate_op_idx].operatorCode == 0) {
    		continue;
    	}
    	if (op_idx != 1) {
    		strcat(operators_payload, ",");
    	}

    	int cur_op_csq = opList[candidate_op_idx].csq;

    	if (cur_op_csq < 0) {
    		cur_op_csq = -1 * cur_op_csq;
    	}

        char current_op[50] = "";
        memset(current_op, '\0', 50);
        sprintf(current_op, "opt%dname=%d,opt%dcsq=%d,opt%dtech=%c",
                        op_idx, opList[candidate_op_idx].operatorCode,
						op_idx, cur_op_csq,
						op_idx, opList[candidate_op_idx].accessTechnology[0]);

        strcat(operators_payload, current_op);
        candidate_op_idx = (candidate_op_idx + 1) % MAX_CANDIDATE_CELL_OPS;
    }

    int payload_len = 0;
    if (strlen(unix_time) > 0) {
    //8935201641400948300 opt1name= 42501,opt1csq=17,opt2name=42502,opt2csq=5,opt3name=42503,opt3csq=28 1557086230000000000
    	payload_len = sprintf(cell_payload, CELL_PAYLOAD_FORMAT, iccid, operators_payload, unix_time);
    } else {
    	payload_len = sprintf(cell_payload, CELL_PAYLOAD_NO_TIME_FORMAT, iccid, operators_payload);
    }
    return payload_len;
}


/**
 * This method will return the radio Access Technology for the given operator info struct
 * @param operator - OPERATOR_INFO struct of given operator.
 * @return act of the operator. 0 for 2G(GSM), 2 for 3G(UTRAN). will return -1 in case of error.
 */
int CellularGetAcT(OPERATOR_INFO * operator) {
	if (strcmp(operator->accessTechnology, "2G") == 0) {
		return 0;

	} else if (strcmp(operator->accessTechnology, "3G") == 0) {
		return 2;

	} else {
		return -1;
	}
}


/**
 * This method will make a ping to given address and will return the mean rtt it took
 * @param ip_address the host to ping to
 * @param mean_rtt int pointer to save the result
 * @return true iff ping succeeded and mean_rtt stored, false otherwise
 */
bool CellularPing(char * ip_address, int * mean_rtt) {
    // AT^SISX=<service>, <conProfileId>, <address>[, <request>[, <timelimit>]]
    // <service> = "Ping"
    int num_packets = 30; // 1-30
    int max_responses = num_packets + 5;
    unsigned int packet_timeout_ms = 1000;
    int cmd_size = sprintf(command_to_send_buffer, "%sPing, %d, %s, %d, %d", AT_CMD_SISX_WRITE_PRFX, conProfileId, ip_address, num_packets, packet_timeout_ms);
    sendATcommand(command_to_send_buffer, cmd_size);

    unsigned char * tokens_array[40] = {};
    if (!waitForATresponse(tokens_array, AT_RES_OK, sizeof(AT_RES_OK) - 1, 40, GENERAL_RECV_TIMEOUT_MS+(num_packets*packet_timeout_ms))) {
        // error
        return false;
    }
    // response OK
    for (int idx = 39; idx >= 0; idx--) {
        if (strncmp(tokens_array[idx], "^SISX: \"Ping\", 3", strlen("^SISX: \"Ping\", 3")) == 0) {
            // ^SISX:"Ping", 3, <conProfileId>, <minRTT>, <maxRTT>, <meanRTT>
            char * mean_rtt_res = (tokens_array[idx], ',');
            *mean_rtt = atoi(mean_rtt_res);
            return true;
        }
    }
    return false;

}


/**
 * This method will setup an TCP Socket service profile to pre defined
 * @param remote_addr the ip address and port of destination
 * @return true iff setup succeeded
 */
bool socketServiceSetupProfile(char * remote_addr) {
    // AT^SISS=<srvProfileId>, <srvParmTag>, <srvParmValue>

    // AT^SISS=9,"SrvType","Socket"
    int cmd_size = sprintf(command_to_send_buffer, "%s%d,\"SrvType\",\"Socket\"%s",
                           AT_CMD_SISS_WRITE_PRFX, SOCKET_SRV_PROFILE_ID, AT_CMD_SUFFIX);
    // send command and check response OK/ERROR
    sendATcommand(command_to_send_buffer, cmd_size);
    if (!waitForOK()) { return false; }

    // AT^SISS=9,"conId","<conProfileId>"
    cmd_size = sprintf(command_to_send_buffer, "%s%d,\"conId\",\"%d\"%s",
                       AT_CMD_SISS_WRITE_PRFX, SOCKET_SRV_PROFILE_ID, conProfileId, AT_CMD_SUFFIX);
    // send command and check response OK/ERROR
    sendATcommand(command_to_send_buffer, cmd_size);
    if (!waitForOK()) { return false; }

    // AT^SISS=9,"address","socktcp://95.179.243.207:54321"
    cmd_size = sprintf(command_to_send_buffer, "%s%d,\"address\",\"socktcp://%s\"%s",
                       AT_CMD_SISS_WRITE_PRFX, SOCKET_SRV_PROFILE_ID, remote_addr, AT_CMD_SUFFIX);
    // send command and check response OK/ERROR
    sendATcommand(command_to_send_buffer, cmd_size);
    return waitForOK();
}


/**
 * This method will check for ^SISW: URC
 * @return <urcCauseId>
 */
int handleSISWURC() {
    unsigned char * tokens_array[1] = {};
    int received_urcs = getSISURCs(tokens_array, 1, SIS_RECV_TIMEOUT_MS);

    if (received_urcs != 1) {
        return -1;
    } else if (strcmp(tokens_array[0], "^SISW: 9,1") == 0) {
        return 1;

    } else if (strcmp(tokens_array[0], "^SISW: 9,2") == 0) {
        return 2;

    } else {
        return -1;
    }
}


/**
 * This method will handle the response comes after AT^SISW command
 * @return <cnfWriteLength>
 */
int handleSISWresponse() {
    unsigned char * tokens_array[1] = {};
    int received_urcs = getSISURCs(tokens_array, 1, SIS_RECV_TIMEOUT_MS);

    // ^SISW: <srvProfileId>, <cnfWriteLength>, <unackData>
    if (received_urcs != 1) {
        return -1;

    } else {
        char * last_delim = strrchr(&(tokens_array[0][9]), ',');
        *last_delim = '\0';
        return atoi(&(tokens_array[0][9]));
    }
}


/**
 * This method will check for ^SISR: URC
 * @return <urcCauseId>
 */
int handleSISRURC() {
    unsigned char * tokens_array[1] = {};
    int received_urcs = getSISURCs(tokens_array, 1, SIS_RECV_TIMEOUT_MS);

    if (received_urcs != 1) {
        return -1;
    } else if (strcmp(tokens_array[0], "^SISR: 9,1") == 0) {
        return 1;

    } else if (strcmp(tokens_array[0], "^SISR: 9,2") == 0) {
        return 2;

    } else {
        return -1;
    }
}


/**
 * This method will handle the response comes after AT^SISR command
 * @return <cnfReadLength>
 */
int handleSISRresponse() {
    unsigned char * tokens_array[5] = {};
    int received_urcs = getSISURCs(tokens_array, 3, SIS_RECV_TIMEOUT_MS);

    // ^SISR: <srvProfileId>, <cnfReadLength>
    if (received_urcs != 3) {
        return -1;

    } else {
        if (!(strcmp(tokens_array[2], "OK") == 0)) {
            return -1;
        }
        return atoi(&(tokens_array[0][9]));
    }
}


/**
 * This method will send #ANALYZER_PACKET_SIZE Bytes over the TCP socket.
 */
void sendSpeedPacket() {
    int cnfWriteLength;
    int sent = 0;
    while (sent < ANALYZER_PACKET_SIZE) {
        // AT^SISW=<srvProfileId>, <reqWriteLength>
        int cmd_size = sprintf(command_to_send_buffer, "%s%d,%d\"%s",
                               AT_CMD_SISW_WRITE_PRFX, SOCKET_SRV_PROFILE_ID, ANALYZER_PACKET_SIZE-sent, AT_CMD_SUFFIX);
        // send command
        sendATcommand(command_to_send_buffer, cmd_size);
        // ^SISW: <srvProfileId>, <cnfWriteLength>, <unackData>
        cnfWriteLength = handleSISWresponse();
        if (cnfWriteLength <= 0) {
            continue;
        }

        //Number of data bytes as specified by <cnfWriteLength>.
        while(!SerialSendCellular(&speed_packet[sent], cnfWriteLength));
        while(!SerialSendCellular(AT_CMD_SUFFIX, 2));

        //OK or ERROR
        if (waitForOK()) {
            sent = sent + cnfWriteLength;
            while(handleSISWURC() == -1);
        }
    }
}


/**
 * This method consume the pre defined server special upload ack for the entire upload test.
 */
void waitForULAck() {
    // AT^SISR=<srvProfileId>, <reqReadLength>//todo check reqReadLength
    int cmd_size = sprintf(command_to_send_buffer, "%s%d,%d\"%s",
                           AT_CMD_SISR_WRITE_PRFX, SOCKET_SRV_PROFILE_ID, ANALYZER_PACKET_SIZE, AT_CMD_SUFFIX);
    // send command
    sendATcommand(command_to_send_buffer, cmd_size);
    //^SISR: <srvProfileId>, <cnfReadLength>
    //Number of data bytes as specified by <cnfWriteLength>.
    //OK or ERROR
    handleSISRresponse();
}


/**
 * This method will receive #ANALYZER_PACKET_SIZE Bytes over the TCP socket.
 */
void receiveSpeedPacket() {
    int cnfWriteLength;
    int recv = 0;
    while (recv < ANALYZER_PACKET_SIZE) {
        // AT^SISR=<srvProfileId>, <reqReadLength>
        int cmd_size = sprintf(command_to_send_buffer, "%s%d,%d\"%s",
                               AT_CMD_SISR_WRITE_PRFX, SOCKET_SRV_PROFILE_ID, ANALYZER_PACKET_SIZE-recv, AT_CMD_SUFFIX);
        // send command
        sendATcommand(command_to_send_buffer, cmd_size);
        //^SISR: <srvProfileId>, <cnfReadLength>
        //Number of data bytes as specified by <cnfWriteLength>.
        //OK or ERROR
        cnfWriteLength = handleSISRresponse();
        if (cnfWriteLength <= 0) {
            continue;
        }
        recv = recv + cnfWriteLength;
        while(handleSISRURC() == -1);
    }
}
/**
 * @brief Delays number of msTick Systicks (typically 1 ms)
 * @param dlyTicks Number of ticks to delay
 */
void CellularDelay(uint32_t dlyTicks) {
  uint32_t curTicks;

  curTicks = msTicks;
  while ((msTicks - curTicks) < dlyTicks) ;
}
