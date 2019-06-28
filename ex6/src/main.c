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
void testOperators();
void testSpeed(long double * ul_Bps, long double * dl_Bps);
void transmit_results(OPERATOR_INFO * current_op, float ul, float dl, int latency );
void printTestResults(OPERATOR_INFO * operatorInfo, long double ul_bps, long double dl_bps, long double latency);

/****************************************************************************
 * 								DEFS
*****************************************************************************/
#define MAX_IL_CELL_OPS 10
#define MIN_GPS_MSGS_TO_FIXTIME 3

#define SPEEDTEST_SERVER_ADDR "95.179.243.207:54321"
#define SPEEDTEST_PING_ADDR "95.179.243.207"
#define TRANSMIT_URL "http://51.143.141.28:8086/write?db=mydb"
#define TRANS_RES_BUF_SIZE 100
#define MAX_PAYLOAD_SIZE 255
#define ANALYZER_FORMAT "networkAnalyzer,ICCID=%s latitude=%f,longitude=%f,altitude=%d,opt_code=%d,opt_act=%s,opt_ul=%f,opt_dl=%f,opt_latency=%d "
//todo
#define BYTES_TO_BITS 8
#define KILOBIT_IN_BITS 1000
#define MEGABIT_IN_BITS 1000000
#define Mbps_STR "Mbps"
#define Kbps_STR "Kbps"
#define bps_STR "bps"

enum PROCEDURE_TO_RUN{WELCOME_SCREEN, INIT_TEST, TEST_IN_PROGRESS, EXIT_TEST};

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
char speed_results[MAX_PAYLOAD_SIZE];//todo check size
//todo set profiles only once.

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
	while(CURRENT_OPERATION != EXIT_TEST) {
		if (CURRENT_OPERATION == WELCOME_SCREEN) {
			CURRENT_OPERATION = INIT_TEST;
			printf("\f Welcome to\n   iNetworkAnalyzer\n");
			printf(" BTN1:\n  Analyze Cell Network\n");
			printf(" BTN0:\n  Exit iNA\n");

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
            testOperators();
            Delay(3000);
            CURRENT_OPERATION = WELCOME_SCREEN;
		}
	}

	printf("\nDisabling Cellular and exiting..\n");
	CellularDisable();
	GPSDisable();

	free(current_location);
	free(found_operators);
	exit(0);
}

/**
 * This method will get the current location & will check the available operators.
 * @return number of networks of the specific 3 Major operators (Partner, Cellcom, Pelephone)
 */
int getPreTestData() {
	/* clear current_location struct */
    memset(current_location->fixtime, '\0', GPS_LOC_INFO_TIME_BUF_SIZE);
    /* clear speed results buffer */
	memset(speed_results, '\0', MAX_PAYLOAD_SIZE);//todo check size

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


/**
 * This method will try to register each of the available networks of the supported operators and
 * set an internet connection and to test Latency, upload & download bit rates.
 * The result for each networks will be transmitted to influxDB server + stored in local buffer,
 * for the purporse of printing and the end of the run.
 */
void testOperators() {
	/* Tries to register with Pelephone, Cellcom & Partner. */
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

                    long double ul_bps, dl_bps, latency;
                    int mean_rtt;
                    /* measure internet performances */
                    testSpeed(&ul_bps, &dl_bps);
                    CellularPing(SPEEDTEST_PING_ADDR, &mean_rtt);
                    latency = mean_rtt / 2.0;

                    /* transmit result to remote DB */
					transmit_results(&found_operators[op_index], ul_bps, dl_bps, latency);

					/* print result to screen */
                    printTestResults(&found_operators[op_index], ul_bps, dl_bps, latency);



				} else {
					printf("Couldn't get signal quality.\n");
				}
			}
		}
		printf("Modem registration failed\n");
	}
}


/**
 * This method will test speed of current operator by setting up socket connection
 * @param ul_bps long double pointer to save upload bitrate to
 * @param dl_bps long double pointer to save download bitrate to
 */
void testSpeed(long double * ul_bps, long double * dl_bps) {
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

    *ul_bps = BYTES_TO_BITS * (ANALYZER_TOTAL_PACKETS * ANALYZER_PACKET_SIZE / ((ul_end - ul_start)*1000000));
    *dl_bps = BYTES_TO_BITS * (ANALYZER_TOTAL_PACKETS * ANALYZER_PACKET_SIZE / ((dl_end - dl_start)*1000000));

    /* Close socket service profile */
    if (!serviceProfileClose(SOCKET_SRV_PROFILE_ID)) {
        printf("Failed to close Socket service");
        return;
    }
}


/**
 * This method will create the operator results payload and transmit it DB
 * @param current_op tested operator
 * @param ul upload bitrate bps
 * @param dl download bitrate bps
 * @param latency ms
 */
void transmit_results(OPERATOR_INFO * current_op, float ul, float dl, int latency) {
	memset(payload_buffer, '\0', MAX_PAYLOAD_SIZE);
	/* get ICCID */
	CellularGetICCID(iccid);

	//latitude= 31.7498445,longitude= 35.1838178,altitude=10,hdop=2,valid_fix=1,num_sats=4 1557086230000000000
	float lat_deg = (current_location->latitude) / 10000000.0;
	float long_deg = (current_location->longitude) / 10000000.0;

	//FORMAT "networkAnalyzer,ICCID=%s latitude=%f,longitude=%f,altitude=%d,opt_code=%d,opt_act=%s,opt_ul=%f,opt_dl=%f,opt_latency=%d "
	int	payload_len = sprintf(payload_buffer, ANALYZER_FORMAT,
								  iccid, lat_deg, long_deg, current_location->altitude, current_op->operatorCode, current_op->accessTechnology, ul, dl, latency);
    // transmit GPS data over HTTP
    if (CellularSendHTTPPOSTRequest(TRANSMIT_URL, payload_buffer, payload_len, transmit_response, TRANS_RES_BUF_SIZE) == -1) {
        printf("Failed\n");
    }

    // check the service is closed
    while (!serviceProfileClose(HTTP_SRV_PROFILE_ID));//todo check if needed
}


/**
 * This method will print speed test results to screen with bit rates units
 * @param operatorInfo the tested operator
 * @param ul_bps upload bitrate bits per seconds
 * @param dl_bps download bitrate bits per seconds
 * @param latency ping latency ms
 */
void printTestResults(OPERATOR_INFO * operatorInfo, long double ul_bps, long double dl_bps, long double latency) {
    char * ul_unit;
    char * dl_unit;
    if (ul_bps >= MEGABIT_IN_BITS) {
        ul_bps = ul_bps / MEGABIT_IN_BITS;
        ul_unit = Mbps_STR;
    } else if (ul_bps >= KILOBIT_IN_BITS) {
        ul_bps = ul_bps / KILOBIT_IN_BITS;
        ul_unit = Kbps_STR;
    } else {
        ul_unit = bps_STR;
    }

    if (dl_bps >= MEGABIT_IN_BITS) {
        dl_bps = dl_bps / MEGABIT_IN_BITS;
        dl_unit = Mbps_STR;
    } else if (dl_bps >= KILOBIT_IN_BITS) {
        dl_bps = dl_bps / KILOBIT_IN_BITS;
        dl_unit = Kbps_STR;
    } else {
        dl_unit = bps_STR;
    }

    /* add result to local buffer */
    // Operator-xG:
    //  Latency: ddd ms
    //  UL: ddd Xbps
    //  DL: ddd Xbps
    printf(speed_results, " %s - %s\n Latency: %.1f ms\n DL: %.2f %s\n UL: %.2f %s\n",
           operatorInfo->operatorName, operatorInfo->accessTechnology,
           latency, dl_bps, dl_unit, ul_bps, ul_unit);

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
	    /* exit the app possible only on welcome screen */
	    if (CURRENT_OPERATION == WELCOME_SCREEN) {
            /* Clear screen */
            printf("\f\n\n\n   Goodbye!");
            CURRENT_OPERATION = EXIT_TEST;
	    }
	}

	if (interruptMask & (1 << BSP_GPIO_PB1_PIN)) {
		/* run test is possible only on welcome screen */
		if (CURRENT_OPERATION == WELCOME_SCREEN) {
			/* Clear screen */
			printf("\f");
			/* set flag */
			CURRENT_OPERATION = INIT_TEST;
		}
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
