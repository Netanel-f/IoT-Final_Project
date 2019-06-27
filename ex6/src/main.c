#include "gps.h"
#include "cellular.h"

#include <stdio.h>
#include "em_device.h"
#include "em_cmu.h"
#include "em_chip.h"
#include "em_emu.h"
#include "bsp.h"

#include "em_gpio.h"
#include "em_leuart.h"

#include "em_acmp.h"
#include "capsense.h"
#include "display.h"
#include "textdisplay.h"
#include "retargettextdisplay.h"


/****************************************************************************
 * 								DECLARATIONS
*****************************************************************************/
void Delay(uint32_t dlyTicks);
int getPreTestData();
void testSpeed(long double * ul_Bps, long double * dl_Bps);
void transmitResult(int payload_len);
int getOperatorPayload(OPERATOR_INFO * current_op, float ul, float dl, int latency );
//void infoOnDemand(GPS_LOCATION_INFO* last_location);
//bool speedLimitInternet();
//void speedLimitInterval(GPS_LOCATION_INFO* last_location, GPS_LOCATION_INFO* old_location);

/****************************************************************************
 * 								DEFS
*****************************************************************************/
#define MAX_IL_CELL_OPS 10
#define MIN_GPS_MSGS_TO_FIXTIME 3

#define SPEEDTEST_SERVER_ADDR "95.179.243.207:54321"
#define SPEEDTEST_PING_ADDR "95.179.243.207"
#define TRANSMIT_URL "http://51.143.141.28:8086/write?db=mydb"
#define ONE_MINUTE_IN_MS 60000
#define FIVE_SECS_IN_MS 5000
#define THREE_MINS_IN_MS 180000
//#define MIN_SPEED_LIM 0
//#define MAX_SPEED_LIM 30
#define UNIX_TIME_BUF_SIZE 35
#define TRANS_RES_BUF_SIZE 100
#define MAX_PAYLOAD_SIZE 255
#define ANALYZER_FORMAT "networkAnalyzer,ICCID=%s latitude=%f,longitude=%f,altitude=%d,opt_code=%d,opt_ul=%f,opt_act=%s,opt_dl=%f,opt_latency=%d "

enum PROCEDURE_TO_RUN{WELCOME_SCREEN, INIT_TEST, TEST_IN_PROGRESS, TEST_PAUSED};

/**************************************************************************//**
 * 							GLOBAL VARIABLES
*****************************************************************************/
#ifdef _WIN64
static char* GPS_PORT = "COM4";
static char* MODEM_PORT = "COM5";
#elif _WIN32
static char* GPS_PORT = "COM4";
static char* MODEM_PORT = "COM5";
#else
static char* GPS_PORT = "3";
static char* MODEM_PORT = "0";
#endif

bool DEBUG = false;
volatile uint32_t msTicks = 0; /* counts 1ms timeTicks */

enum PROCEDURE_TO_RUN CURRENT_OPERATION = WELCOME_SCREEN;
//todo
GPS_LOCATION_INFO* current_location;
OPERATOR_INFO * found_operators;
int num_of_operators_found;

//char unix_time[UNIX_TIME_BUF_SIZE];
char iccid[ICCID_BUFFER_SIZE];
char payload_buffer[MAX_PAYLOAD_SIZE];
char transmit_response[TRANS_RES_BUF_SIZE];

//static int speed_limit = MIN_SPEED_LIM;
//static bool high_speed_flag = false;
//static bool transmit_speed_event = false;
//static uint32_t last_location_Ticks = 0;
//static uint32_t old_location_Ticks = 0;
//static bool part2_registered = false;

/***************************************************************************//**
 * @brief SysTick_Handler
 * Interrupt Service Routine for system tick counter
 ******************************************************************************/
void SysTick_Handler(void)
{
  msTicks++;       /* increment counter necessary in Delay()*/
}


/*******************************************************************************
 * @brief Delays number of msTick Systicks (typically 1 ms)
 * @param dlyTicks Number of ticks to delay
 ******************************************************************************/
void Delay(uint32_t dlyTicks)
{
  uint32_t curTicks;
  curTicks = msTicks;
  while ((msTicks - curTicks) < dlyTicks) ;
}


/*******************************************************************************
 * @brief  This method will init all necessary device's settings.
 ******************************************************************************/
void initDeviceSettings() {
	/* Enable clock for GPIO purposes */
	CMU_ClockEnable(cmuClock_GPIO, true);
	/* Enable clock for USART module */
	CMU_ClockEnable(cmuClock_USART2, true);
	/* Enable clock for HF peripherals */
	CMU_ClockEnable(cmuClock_HFPER, true);


	/* Initialize the display module. */
	DISPLAY_Init();
	RETARGET_TextDisplayInit();

	/* setting stdout to flush prints immediately */
	setbuf(stdout, NULL);


	/* Configure PB0 as input and enable interrupt  */
	GPIO_PinModeSet(BSP_GPIO_PB0_PORT, BSP_GPIO_PB0_PIN, gpioModeInputPull, 1);
	GPIO_IntConfig(BSP_GPIO_PB0_PORT, BSP_GPIO_PB0_PIN, false, true, true);

	/* Configure PB1 as input and enable interrupt */
	GPIO_PinModeSet(BSP_GPIO_PB1_PORT, BSP_GPIO_PB1_PIN, gpioModeInputPull, 1);
	GPIO_IntConfig(BSP_GPIO_PB1_PORT, BSP_GPIO_PB1_PIN, false, true, true);

	/* Buttons interrupts config */
	NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
	NVIC_EnableIRQ(GPIO_EVEN_IRQn);
	NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
	NVIC_EnableIRQ(GPIO_ODD_IRQn);


	/* Setup SysTick Timer for 1 msec interrupts  */
	if (SysTick_Config(CMU_ClockFreqGet(cmuClock_CORE) / 1000)) {
		while (1) ;
	}

	/* Start capacitive sense buttons */
	CAPSENSE_Init();
}

/*******************************************************************************
 * @brief  Main function
 ******************************************************************************/
int main(void)
{
	/* Chip errata */
	CHIP_Init();

	/* Init DCDC regulator with kit specific parameters */
	EMU_DCDCInit_TypeDef dcdcInit = EMU_DCDCINIT_DEFAULT;
	EMU_DCDCInit(&dcdcInit);

	/* Init other device settings */
	initDeviceSettings();

	/* General declarations and memory allocations for main use. */
	current_location = malloc(sizeof(GPS_LOCATION_INFO));
	if (current_location == NULL) {
	  printf("Memory Allocation Error");
	  return 1;
	}
	found_operators = malloc(iNA_MAX_CANDIDATE_CELL_OPS * sizeof(OPERATOR_INFO));
	if (found_operators == NULL) {
	  printf("Memory Allocation Error");
	  return 1;
	}

	/* Init GPS & Cellular modules */
	printf("\n MCU initialized\n Turn on GPS & Modem\n");
	Delay(1000);
	GPSInit(GPS_PORT);
	CellularInit(MODEM_PORT);

	/* Main loop */
	while(true) {
		if (CURRENT_OPERATION == WELCOME_SCREEN) {
			CURRENT_OPERATION = INIT_TEST;
			printf("\f Welcome to\n   iNetworkAnalyzer\n");
			printf(" BTN1:\n  Analyze Cell Network\n");

		} else if (CURRENT_OPERATION == INIT_TEST) {
			/* Get GPS + available operators  */
			num_of_operators_found = getPreTestData();
			if (num_of_operators_found > 0) {
				CURRENT_OPERATION = TEST_IN_PROGRESS;
			} else {
				CURRENT_OPERATION = WELCOME_SCREEN;
				Delay(1000);
			}

		} else if (CURRENT_OPERATION == TEST_IN_PROGRESS) {

//		} else if (CURRENT_OPERATION == GPS_CELL_ON_DEMAND) {
//			CURRENT_OPERATION = HELLO_SCREEN;
//			part2_registered = false;
//			infoOnDemand(last_location);
//
//		} else if (CURRENT_OPERATION == SPEED_LIMIT_INIT) {
//			printf("Saving current location GPS data\n");
//			/* setting operation variables */
//			speed_limit = MIN_SPEED_LIM;
//			high_speed_flag = false;
//			transmit_speed_event = false;
//
//			speedLimitInternet();
//
//			// update current location
//			memset(last_location->fixtime, '\0', GPS_LOC_INFO_TIME_BUF_SIZE);
//			int times_received_gps_info = 0;
//			while (times_received_gps_info < MIN_GPS_MSGS_TO_FIXTIME) {
//				bool result = GPSGetFixInformation(last_location);
//				if (result && last_location->valid_fix != 0){
//					times_received_gps_info++;
//				}
//			}
////			memcpy(old_location, last_location, sizeof(GPS_LOCATION_INFO));
//
//			last_location_Ticks = msTicks;
//			old_location_Ticks = last_location_Ticks;
//			printf("\f\nCurrent speed limit:\n   %2d Km/h", speed_limit);
//
//			CURRENT_OPERATION = SPEED_LIMIT;
//
//		} else if (CURRENT_OPERATION == SPEED_LIMIT) {

//			if (msTicks - last_location_Ticks > FIVE_SECS_IN_MS) {
//				speedLimitInterval(last_location, old_location);
//			}

//			Delay(100);
//			CAPSENSE_Sense();
//
//			if (CAPSENSE_getPressed(BUTTON1_CHANNEL)
//				&& !CAPSENSE_getPressed(BUTTON0_CHANNEL)) {
//				if (speed_limit < MAX_SPEED_LIM){
//					speed_limit += 5;
//				}
//			  printf("\r   %2d", speed_limit);
//			} else if (CAPSENSE_getPressed(BUTTON0_CHANNEL)
//					   && !CAPSENSE_getPressed(BUTTON1_CHANNEL)) {
//			  if (speed_limit > MIN_SPEED_LIM) {
//				  speed_limit -= 5;
//			  }
//			  printf("\r   %2d", speed_limit);
//			}
		}
	}

	printf("\nDisabling Cellular and exiting..\n");
	CellularDisable();
	GPSDisable();

	free(current_location);
	free(found_operators);
	exit(0);
}

int getPreTestData() {
	/* clear current_location struct */
	memset(current_location->fixtime, '\0', GPS_LOC_INFO_TIME_BUF_SIZE);
	/* Get GPS data */
	while (current_location->valid_fix == 0) {
		GPSGetFixInformation(current_location);
	}

	/* clear candidate_operators struct */
	for (int i=0; i < iNA_MAX_CANDIDATE_CELL_OPS; i++) {
		memset(found_operators[i].operatorName, '\0', MAX_OPERATOR_NAME_SIZE);
		memset(found_operators[i].accessTechnology, '\0', MAX_OPERATOR_TECH_SIZE);
		found_operators[i].operatorCode = 0;
		found_operators[i].csq = -114;
	}

	/* Get all available operators */
	/* Makes sure modem is  responding to AT commands. */
	while (!CellularCheckModem());

	/* Setting modem to unregister from current operator and remain unregistered. */
	while (!CellularSetOperator(DEREGISTER, NULL, 0));


	// store data about found operators and registered ones.
	OPERATOR_INFO operators_info[MAX_IL_CELL_OPS];

	int num_operators_found = -1;

	/* Finds all available cellular operators. */
	printf("Finding all available cellular operators...");
	if (CellularGetSpecificOperators(found_operators, iNA_MAX_CANDIDATE_CELL_OPS, &num_operators_found)) {
		printf(" %d operators found.\n", num_operators_found);
	} else if (num_operators_found == 0) {
		printf(" Couldn't find any operators\n");
		Delay(1000);
	} else {
		printf(" Error occurred\n");
		Delay(1000);
	}
	return num_operators_found;
}


void testOperators() {
	/* Tries to register with Pelephone,Cellcom & Partner. */
	for (int op_index = 0; op_index < num_of_operators_found; op_index++) {

		/* Get operator Access Technology */
		int cur_op_act = CellularGetAcT(&found_operators[op_index]);
		if (cur_op_act == -1) {
			continue;
		}

		/* unregister from current operator */
		while (!CellularSetOperator(DEREGISTER, NULL, 0));

		/* register to specific operator */
		printf("Trying to register with %s...", found_operators[op_index].operatorName);
		if (CellularSetOperator(SPECIFIC_OP, found_operators[op_index].operatorName, cur_op_act)) {

			// If registered successfully, it prints the signal quality.
			int registration_status = 0;

			// verify registration to operator and check status
			if (CellularGetRegistrationStatus(&registration_status) &&
					(registration_status == 1 || registration_status == 5)) {
				printf("registered successfully\n");
				// registration_status == 1: Registered to home network
				// registration_status == 5: Registered,
				// roaming ME is registered at a foreign network (national or international network)

				int signal_quality = 0;
				if (CellularGetSignalQuality(&signal_quality)) {
					printf("Current signal quality: %ddBm\n", signal_quality);
					found_operators[op_index].csq = signal_quality;

                    long double ul_Bps, dl_Bps, latency;
                    int mean_rtt;
                    testSpeed(&ul_Bps, &dl_Bps);
                    CellularPing(SPEEDTEST_PING_ADDR, &mean_rtt);
                    latency = mean_rtt / 2.0;
					int payload_len = getOperatorPayload(found_operators[op_index], ul_Bps, dl_Bps, latency);
					transmitResult(payload_len);

				} else {
					printf("Couldn't get signal quality.\n");
				}
			}
		}
		printf("Modem registration failed\n");
	}


//	// Connects to an available operator (if available, if no, tries again after one minute)
//	for (int op_index = 0; op_index < iNA_MAX_CANDIDATE_CELL_OPS; op_index++) {
//
//		/* Checking if candidate operator (cellcom/partner/pelephone) was found. */
//		if (found_operators[op_index].operatorCode == 0) {
//			continue;
//		}
//
//		/* unregister from current operator */
//		printf("Unregister from current operator\n");
//		while (!CellularSetOperator(DEREGISTER, NULL, 0));
//
//		/* register to operator */
//		printf("Trying to register with %s...\n", candidate_operators[op_index].operatorName);
//		int cur_op_act = CellularGetAcT(&candidate_operators[op_index]);
//
//		if (CellularSetOperator(SPECIFIC_OP, candidate_operators[op_index].operatorName, cur_op_act)) {
//
//			// If registered successfully, it prints the signal quality.
//			int registration_status = 0;
//
//			/* Trying to register to operator. (if available, if no, tries again after one minute)  */
//			/* verify registration to operator and check status */
//			if (!(CellularGetRegistrationStatus(&registration_status) &&
//				  (registration_status == 1 || registration_status == 5))) {
//
//				printf("Trying again in one minute.\n");
//				Delay(ONE_MINUTE_IN_MS);
//
//				if (!(CellularGetRegistrationStatus(&registration_status) &&
//					  (registration_status == 1 || registration_status == 5))) {
//					printf("Failed to register to %s\n", candidate_operators[op_index].operatorName);
//					continue;
//				}
//			}
//
//			/* we registered to operator, setup Internet connection */
//			if (!CellularSetupInternetConnectionProfile(20)) {
//				printf("Failed to setup Internet connection");
//				continue;
//			}
//
//			/* get ICCID */
//			CellularGetICCID(iccid);
//
//			/* clear Unix time and transmit response buffers */
//			memset(unix_time, '\0', UNIX_TIME_BUF_SIZE);
//
//			if (got_gps_data) {
//				printf("Preparing to send GPS payload...");
//				// get Unix time
//				GPSConvertFixtimeToUnixTime(last_location, unix_time);
//
//				// transmit GPS data over HTTP
//				int gps_payload_len = GPSGetPayload(last_location, iccid, unix_time, payload_buffer, MAX_PAYLOAD_SIZE);
//
//				if (CellularSendHTTPPOSTRequest(TRANSMIT_URL, payload_buffer, gps_payload_len, transmit_response, TRANS_RES_BUF_SIZE) == -1) {
//					printf("Failed\n");
//					printf("Trying another operator.\n");
//					continue;
//				}
//				printf("Sent\n");
//
//				// check the service is closed
//				while (!inetServiceClose(HTTP_POST_srvProfileId));
//			}
//
//			/* Transmit cell data over HTTP */
//			printf("Preparing to send Cellular payload...");
//			int cell_payload_len = CellularGetPayload(candidate_operators, op_index, iccid, unix_time, payload_buffer, MAX_PAYLOAD_SIZE);
//
//			if (CellularSendHTTPPOSTRequest(TRANSMIT_URL, payload_buffer, cell_payload_len, transmit_response, TRANS_RES_BUF_SIZE) == -1) {
//				printf("Failed\n");
//				printf("Trying another operator.\n");
//				continue;
//			}
//			printf("Sent\n");
//			part2_registered = true;
//			return;
//		}
//	}
//	part2_registered = false;
//	printf("Couldn't send using any main operator.\n");
//	Delay(3000);
}

void testSpeed(long double * ul_Bps, long double * dl_Bps) {
    int scnt = 0;
    int rcnt = 0;
    uint32_t ul_start, ul_end, dl_start, dl_end;

    // send 1000 packets of 1500 bytes. and wait for server response.

    /* we registered to operator, setup Internet connection */
    if (!CellularSetupInternetConnectionProfile(20)) {
        printf("Failed to setup Internet connection");
        return;
    }
    /* Setup TCP socket service profile */
    if (!socketServiceSetupProfile(SPEEDTEST_SERVER_ADDR)) {
        printf("Failed to setup TCP Socket service");
        return;
    }
    /* Open socket service profile */
    if (!serviceProfileOpen(SOCKET_SRV_PROFILE_ID)) {
        printf("Failed to open Socket service");
        return;
    }

    // handle ^SISW: 9,1
    if (handleSISWURC() != 1) {
        printf("handle ^SISW failed");
        return;
    }

    ul_start = msTicks;
    while (scnt < ANALYZER_TOTAL_PACKETS) {
        sendSpeedPacket();
        scnt++;
    }
    // handle ^SISR: 9,1 - wait for ack packet
    int temp_result = handleSISRURC();
    ul_end = msTicks;

    if (temp_result != 1) {
        printf("handle ^SISR failed");
        return;
    }
    waitForULAck();

    dl_start = msTicks;
    while (rcnt < ANALYZER_TOTAL_PACKETS) {
        receiveSpeedPacket();
        rcnt++;
    }
    dl_end = msTicks;

    *ul_Bps = ANALYZER_TOTAL_PACKETS * ANALYZER_PACKET_SIZE / ((ul_end - ul_start)*1000000);
    *dl_Bps = ANALYZER_TOTAL_PACKETS * ANALYZER_PACKET_SIZE / ((dl_end - dl_start)*1000000);

    /* Close socket service profile */
    if (!serviceProfileClose(SOCKET_SRV_PROFILE_ID)) {
        printf("Failed to close Socket service");
        return;
    }
}

int getOperatorPayload(OPERATOR_INFO * current_op, float ul, float dl, int latency ) {
	memset(payload_buffer, '\0', MAX_PAYLOAD_SIZE);

	/* get ICCID */
	CellularGetICCID(iccid);

	//latitude= 31.7498445,longitude= 35.1838178,altitude=10,hdop=2,valid_fix=1,num_sats=4 1557086230000000000
	float lat_deg = (current_location->latitude) / 10000000.0;
	float long_deg = (current_location->longitude) / 10000000.0;

	//FORMAT "networkAnalyzer,ICCID=%s latitude=%f,longitude=%f,altitude=%d,opt_code=%d,opt_act=%s,opt_ul=%f,opt_dl=%f,opt_latency=%d "
	int	payload_len = sprintf(payload_buffer, ANALYZER_FORMAT,
								  iccid, lat_deg, long_deg, current_location->altitude, current_op->operatorCode, current_op->accessTechnology, ul, dl, latency);
	return payload_len;
}

void transmitResult(int payload_len) {
	// transmit GPS data over HTTP
    if (CellularSendHTTPPOSTRequest(TRANSMIT_URL, payload_buffer, payload_len, transmit_response, TRANS_RES_BUF_SIZE) == -1) {
        printf("Failed\n");
	}

    // check the service is closed
	while (!serviceProfileClose(HTTP_SRV_PROFILE_ID));

	return;
}

/***************************************************************************//**
 * @brief Unified GPIO Interrupt handler (pushbuttons)
 *        PB0 Prints first name.
 *        PB1 Prints last name.
 *****************************************************************************/
void GPIO_Unified_IRQ(void) {
	/* Get and clear all pending GPIO interrupts */
	uint32_t interruptMask = GPIO_IntGet();
	GPIO_IntClear(interruptMask);

	/* Act on interrupts */
	if (interruptMask & (1 << BSP_GPIO_PB0_PIN)) {
	  /* Clear screen */
	  printf("\f");
	  /* set flag */
//	  CURRENT_OPERATION = GPS_CELL_ON_DEMAND;
	}

	if (interruptMask & (1 << BSP_GPIO_PB1_PIN)) {
		/* check if already running speed limit interval */
//		if (CURRENT_OPERATION != SPEED_LIMIT) {
//			/* Clear screen */
//			printf("\f");
//			/* set flag */
//			CURRENT_OPERATION = SPEED_LIMIT_INIT;
//		}
	}
}

/***************************************************************************//**
 * @brief GPIO Interrupt handler for even pins
 ******************************************************************************/
void GPIO_EVEN_IRQHandler(void) {
	GPIO_Unified_IRQ();
}

/***************************************************************************//**
 * @brief GPIO Interrupt handler for odd pins
 ******************************************************************************/
void GPIO_ODD_IRQHandler(void) {
	GPIO_Unified_IRQ();
}
