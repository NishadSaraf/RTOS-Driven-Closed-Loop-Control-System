/* Project3 - ECE 544 Project #3 Application template
 *
 *
 * Modified by: Nishad Saraf, Chaitanya Deshpande
 * Date:		6th march 2017
 * Revision:	4.1
 * Description:
 * ------------	
 * This program implements feedback control of DC motor using PID Controller using
 * Xilkernal operating System. Four threads are used in this program 1st control thread
 * for controlling switches and reading rotary count 2rd thread motor speed thread used for reading
 * speed 3rd thread PID thread used for control logic 4th thread motor command thread is used for
 * setting duty cycle of the wave.
 */

// Includes
#include "xmk.h"
#include "os_config.h"
#include "config/config_param.h"
#include "sys/ksched.h"
#include "sys/init.h"
#include <pthread.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <semaphore.h>
#include <sys/intr.h>
#include <sys/timer.h>
#include <sys/types.h>
#include <stdio.h>
#include <errno.h>
#include "xintc.h"
#include "xparameters.h"
#include "platform_config.h"
#include "platform.h"
#include "stdbool.h"
#include "xgpio.h"
#include "xwdttb.h"
#include "xtmrctr.h"
#include "xstatus.h"
#include "nexys4IO.h"
#include "pmodENC.h"
#include "PMODHB3IP.h"
#include "PmodOLEDrgb.h"


// Declarations
#define BTN_GPIO_DEVICEID		XPAR_AXI_GPIO_1_DEVICE_ID
//#define SW_GPIO_DEVICEID		XPAR_AXI_GPIO_SW_DEVICE_ID
//#define LED_GPIO_DEVICEID		XPAR_AXI_GPIO_LED_DEVICE_ID
#define INTC_DEVICEID			XPAR_MICROBLAZE_0_AXI_INTC_DEVICE_ID
#define WDT_DEVICEID			XPAR_WDTTB_0_DEVICE_ID
#define TMRCTR0_DEVICEID		XPAR_AXI_TIMER_0_DEVICE_ID
#define TMRCTR1_DEVICEID		XPAR_AXI_TIMER_1_DEVICE_ID

#define TMRCTR0_INTR_NUM		XPAR_INTC_0_TMRCTR_0_VEC_ID
#define TMRCTR1_INTR_NUM		XPAR_INTC_0_TMRCTR_1_VEC_ID
#define BTN_GPIO_INTR_NUM		XPAR_MICROBLAZE_0_AXI_INTC_AXI_GPIO_1_IP2INTC_IRPT_INTR
#define WDT_INTR_NUM			XPAR_MICROBLAZE_0_AXI_INTC_AXI_TIMEBASE_WDT_0_WDT_INTERRUPT_INTR

// Definitions for peripheral NEXYS4IO
#define NX4IO_DEVICE_ID		XPAR_NEXYS4IO_0_DEVICE_ID
#define NX4IO_BASEADDR		XPAR_NEXYS4IO_0_S00_AXI_BASEADDR
#define NX4IO_HIGHADDR		XPAR_NEXYS4IO_0_S00_AXI_HIGHADDR

// Definitions for peripheral PMODENC
#define PMODENC_DEVICE_ID		XPAR_PMODENC_0_DEVICE_ID
#define PMODENC_BASEADDR		XPAR_PMODENC_0_S00_AXI_BASEADDR
#define PMODENC_HIGHADDR		XPAR_PMODENC_0_S00_AXI_HIGHADDR




// Definitions for peripheral PMODHB3
#define PMODHB3_DEVICE_ID		XPAR_PMODHB3IP_0_DEVICE_ID
#define PMODHB3_BASEADDR		XPAR_PMODHB3IP_0_S00_AXI_BASEADDR
#define PMODHB3_HIGHADDR		XPAR_PMODHB3IP_0_S00_AXI_HIGHADDR

// macro functions
#define MIN(a, b)  				( ((a) <= (b)) ? (a) : (b) )
#define MAX(a, b)  				( ((a) >= (b)) ? (a) : (b) )

// Peripheral instances
XGpio BTNInst, SWInst, LEDInst;
XTmrCtr TMRCTR1Inst;
XIntc 	IntrptCtlrInst;
XWdtTb WDTInst;
PmodENC 	pmodENC_inst;
PmodOLEDrgb	pmodOLEDrgb_inst;
//global variables
volatile u32 	hall_val_high=0;
int  RotaryIncr;
bool RotaryNoNeg;
int RotaryCnt;
u16 measured_speed = 0;
volatile u16 median[16];
int j,k;
u32 temp;
volatile uint8_t kp = 0,ki=0,kd=0;
u16 sw_state; 		//switch state
int sys_flag = 1;	//system running flag
int force_crash_flag = 0; //force crash flag
int motorSpeed_flag, motorCmd_flag, control_flag =0, pid_flag=0;
int sw_flag, button_flag, led_flag =0;
volatile u16	gpio_sw;
volatile u8		gpio_pb;
volatile u16	gpio_led;
int msqid;
int duty_flag = 0;
volatile u8 scaled_rpm = 0;
volatile int16_t error = 0;
volatile int16_t pError = 0;
volatile int16_t iError = 0;
volatile int16_t dError = 0;
volatile int16_t oldError = 0;
volatile int32_t actual_speed = 0;
u32 set_rpm = 0;
int z = 0;
u16 switches;
int Value;



// Message passing variables
typedef struct					// LED message definition
{
	//int msg_src;
	int msg_value;
} t_message, *t_LED_messageptr;

const int msg_key = 1;		// message key for LED message queue
//struct msqid_ds	led_msgstats;	// statistics from message queue
t_message msg_ctr, msg_pb, led_msgstats, msg_speed, speed_rcvd,msg_cmd;

// Synchronization variables
sem_t btn_press_sema;			// semaphore between clock tick ISR and the clock main thread
volatile u16 btn_state;			// button state - shared between button handler and button thread

// Function declarations
void* master_thread(void *arg);
void* button_thread(void *arg);
void* motorSpeed_thread(void *arg);
void* motorCommand_thread(void *arg);
void* control_thread(void *arg);
void* pid_thread(void *arg);
void  button_handler(void);
void  wdt_handler(void);
void usleep1(u32 usecs);
XStatus init_peripherals(void);

static const u32	DELAY_1US_CONSTANT	= 15;	// constant for 1 microsecond delay

void usleep1(u32 usec)
{
	volatile u32 i, j;

	for (i = 0; i < usec; i++)
	{
		for (j = 0; j < DELAY_1US_CONSTANT; j++);
	}
	return;
}
/****************************************************************************************/
/****************************************** main program ********************************/
/****************************************************************************************/
int main()
{
	XStatus sts;


	// initialize the platform and the peripherals
	init_platform();
	sts = init_peripherals();
	if (sts != XST_SUCCESS)
	{

		xil_printf("FATAL ERROR: Could not initialize the peripherals\n\r");
		xil_printf("Please power cycle or reset the system\n\r");
		return -1;
	}
	RotaryIncr = 1;
	RotaryNoNeg = true;
	pmodENC_init(&pmodENC_inst, RotaryIncr, RotaryNoNeg);
	pmodENC_clear_count(&pmodENC_inst);



	// check if WDT expired and caused the reset - if so, don't start
	if (XWdtTb_IsWdtExpired(&WDTInst))
	{
		do
		{
			xil_printf("**************************\n\r");
			xil_printf("* Watchdog timer expired  *\n");
			xil_printf("* check if force crash ON *\n");
			xil_printf("* if ON, OFF force crash  *\n");
			xil_printf("* Press CPU reset button  *\n");
			xil_printf("**************************\n\n\r");
		}while(sys_flag = 0);
	}

	// Initialize xilkernel
	xilkernel_init();

	// Create the master thread
	xmk_add_static_thread(master_thread, 0);

	// Start the kernel
	xilkernel_start();

	// Should never be reached
	cleanup_platform();

	return 0;
}

/****************************************************************************************/
/************************************** master thread ***********************************/
/****************************************************************************************/
// The master thread
void* master_thread(void *arg)
{
	unsigned time;
	pthread_t motorSpeed;
	pthread_t motorCommand;
	pthread_t control;
	pthread_t pid;
	pthread_attr_t attr;
	struct sched_param spar;

	int ret;
	pmodENC_init(&pmodENC_inst, RotaryIncr, RotaryNoNeg);
	pmodENC_clear_count(&pmodENC_inst);


	xil_printf("----------------------------------------------------------------------------\r\n");
	xil_printf("ECE 544 Project 3 Starter Application \r\n");
	xil_printf("----------------------------------------------------------------------------\r\n");
	xil_printf("This Xilkernel based application reads the buttons and switches on the FPGA \r\n"
			"development board and displays them on the LEDs.  Even though the application is\r\n"
			"simple it uses several of the synchronization and interprocess communication\r\n"
			"capabilities offered in the Xilkernel\r\n\r\n"
			"To demonstrate, press any of the buttons and/or flip switches on the board.\r\n"
			"The current state of the buttons and switches should be displayed on the LEDs\r\n");
	xil_printf("----------------------------------------------------------------------------\r\n\r\n\r\n");;

	xil_printf("MASTER: Master Thread Starting\r\n");

	// set the priority of all but the master thread to 1.  The master thread runs at priority 0
	// because it is responsible for tickling the WDT.
	pthread_attr_init (&attr);
	spar.sched_priority = 1;
	pthread_attr_setschedparam(&attr, &spar);
	//control
	ret = pthread_create (&control, &attr, (void*) control_thread, NULL);
	if (ret != 0)
	{
		xil_printf("ERROR (%d) IN MASTER THREAD: could not launch %s\r\n", ret, "Control thread");
		xil_printf("FATAL ERROR: Master Thread Terminating\r\n");
		return (void*) -3;
	}
	//speed
	ret = pthread_create (&motorSpeed, &attr, (void*) motorSpeed_thread, NULL);
	if (ret != 0)
	{
		xil_printf("ERROR (%d) IN MASTER THREAD: could not launch %s\r\n", ret, "Motor Speed thread");
		xil_printf("FATAL ERROR: Master Thread Terminating\r\n");
		return (void*) -3;
	}
	//motor command
	ret = pthread_create (&motorCommand, &attr, (void*) motorCommand_thread, NULL);
	if (ret != 0)
	{
		xil_printf("ERROR (%d) IN MASTER THREAD: could not launch %s\r\n", ret, "Motor Command thread");
		xil_printf("FATAL ERROR: Master Thread Terminating\r\n");
		return (void*) -3;
	}
	//pid command
	ret = pthread_create (&pid, &attr, (void*) pid_thread, NULL);
	if (ret != 0)
	{
		xil_printf("ERROR (%d) IN MASTER THREAD: could not launch %s\r\n", ret, "PID thread");
		xil_printf("FATAL ERROR: Master Thread Terminating\r\n");
		return (void*) -3;
	}


	// initialize the button press semaphore
	ret = sem_init (&btn_press_sema, 0, 0);
	if (ret != 0)
	{
		xil_printf("ERROR (%d) IN MASTER THREAD: could not initialize %s\r\n", errno, "button press semaphore");
		xil_printf("FATAL ERROR: Master Thread Terminating\r\n");
		return (void*) -3;
	}
	else
	{
		xil_printf ("MASTER: Button press semaphore has been initialized\n\r");
	}

	// Register the interrupt handlers
	ret = register_int_handler(WDT_INTR_NUM, (void*) wdt_handler, NULL);
	if (ret != XST_SUCCESS)
	{
		return (void*) -4;
	}
	ret = register_int_handler(BTN_GPIO_INTR_NUM, (void*) button_handler, NULL);
	if (ret != XST_SUCCESS)
	{
		return (void*) -4;
	}

	// Enable interrupts and start the WDT...we're off to the races
	enable_interrupt(BTN_GPIO_INTR_NUM);
	enable_interrupt(WDT_INTR_NUM);
	xil_printf("MASTER: Interrupts have been enabled\r\n");

	XWdtTb_Start(&WDTInst);
	xil_printf("MASTER: Watchdog timer has been started\r\n");

	// master thread main loop
	while(1)
	{
		if(motorSpeed & motorCmd_flag & control_flag & pid_flag)
			sys_flag = 1;
		else sys_flag = 0;

		if(force_crash_flag == 1)
			sys_flag = 0;

		wdt_handler();


		motorCmd_flag =0;
		control_flag=0;
		pid_flag=0;
		motorSpeed=0;
		//delay_msecs(100);
		time =  sleep (40);
	}


	return NULL;
}





/****************************************************************************************/
/***************************************** button thread ********************************/
/****************************************************************************************/
// The button thread
void* button_thread(void *arg)
{
	xil_printf("%d",gpio_sw);
	//	xil_printf("**************************\n\r");



	gpio_sw = XGpio_DiscreteRead(&BTNInst, 1);
	xil_printf("%d",gpio_sw);

	int lock_sem;
	int update_led;

	msqid = msgget (1, IPC_CREAT);
	//periodically tell the WDT that BUTTON thread is being invoked
	button_flag = 1;

	//lock the semaphore
	lock_sem = sem_wait (&btn_press_sema);

	//check if semaphore has been locked
	if(lock_sem != 0)
	{
		/*xil_printf("**************************\n\r");
		xil_printf("* Error locking semaphore*\n\r");
		xil_printf("**************************\n\n\r");*/
	}

	//store the new state of buttons to be sent to the message queue
	msg_pb.msg_value = btn_state;

	xil_printf("**************************\n\r");
	xil_printf("*button state = %d*\n\r",btn_state);
	xil_printf("**************************\n\n\r");

	//send update data to LED thread using a message queue
	update_led = msgsnd (msqid, &msg_pb, 1, 0);

	//check if the message was successfully sent to the message queue
	//display appropriate messages on the console
	if(update_led != 0)
	{
		/*xil_printf("*******************************************\n\r");
		xil_printf("* Error sending message from switch thread*\n\r");
		xil_printf("*******************************************\n\n\r");*/
	}
	xil_printf("************************************************\n\r");
	xil_printf("* switch thread sent new message to led thread *\n\r");
	xil_printf("************************************************\n\n\r");
	return NULL;
}
void* control_thread(void *arg)
{
	int update_thread;
	int msqid;
int i;
	while(1)
	{
		control_flag = 1;

		msqid = msgget (msg_key, IPC_CREAT);
		switches = NX4IO_getSwitches();
		if(switches == 0){
			pmodENC_init(&pmodENC_inst, 1, false);
		}else if(switches == 1){
			pmodENC_init(&pmodENC_inst, 5, false);
		}else if(switches == 2 || switches == 3){
			pmodENC_init(&pmodENC_inst, 10, false);
		}
		pmodENC_read_count(&pmodENC_inst, &RotaryCnt);

		msg_ctr.msg_value = RotaryCnt;

		update_thread = msgsnd (msqid, &msg_ctr, 2, 0);
		if (NX4IO_isPressed(BTNC))
		{
			duty_flag = 1;
			kp=0;
			ki=0;
			kd=0;
			pmodENC_clear_count(&pmodENC_inst);


			HB3_IP_setDutyCycle(0);


		}

		if (NX4IO_isPressed(BTNU))	// clear the rotary count
		{
			//pmodENC_clear_count(&pmodENC_inst);
			//dispSetFlg = false;			// clear the row of the display which shows the value of Enc in decimal
			if(switches==4||switches==5||switches== 6 ||switches==7)
			{
				for(i=0;i<950;i++);
				Value=(Value==100)?(100):(Value+1);
				kp = (kp==255)?(255):(kp+1);

			}
			else if(switches==8||switches==10||switches==9||switches==11)
			{
				Value=(Value==100)?(100):(Value+1);
				ki= (ki==255)?(255):(ki+1);

			}
			else if(switches==12||switches==14||switches==13||switches==15)
			{
				Value=(Value==100)?(100):(Value+1);
				kd= (kd==255)?(255):(kd+1);
			}
		}
		else if(NX4IO_isPressed(BTND))
		{
			if(switches==4||switches==5||switches== 6 ||switches==7)
			{
				for(i=0;i<950;i++);
				Value=(Value==0)?(0):(Value-1);
				kp = (kp==0)?(0):(kp-1);
			}
			else if(switches==8||switches==10||switches==9||switches==11)
			{
				Value=(Value==0)?(0):(Value-1);
				ki = (ki==0)?(0):(ki-1);
			}
			else if(switches==12||switches==14||switches==13||switches==15)
			{
				Value=(Value==0)?(0):(Value-1);
				kd = (kd==0)?(0):(kd-1);
			}
		}
		u32 tU;
		if(switches==4||switches==5||switches== 6 ||switches==7)
		{
			tU=kp;
		}
		else if(switches==8||switches==10||switches==9||switches==11)
		{
			tU=ki;
		}
		else if(switches==12||switches==14||switches==13||switches==15)
		{
			tU=kd;
		}
		if(switches == 0x8000){
			force_crash_flag = 1;
		}
		NX4IO_setLEDs(switches);

		NX4IO_SSEG_setDigit(SSEGHI, DIGIT7,(int)  (tU/10)%10);
		NX4IO_SSEG_setDigit(SSEGHI, DIGIT6,(int)  tU%10);

		//check if the message was successfully sent to the message queue
		//display appropriate messages on the console
		if(update_thread != 0)
		{
			/*xil_printf("*******************************************\n\r");
			xil_printf("* Error sending message from Control thread*\n\r");
			xil_printf("*******************************************\n\n\r");*/
		}
		//		xil_printf("* Control thread sent new message to PID thread RC = %d *\n\r",RotaryCnt );
	}
	return NULL;

}
/****************************************************************************************/
/**************************************** Motor Speed thread *********************************/
/****************************************************************************************/
// The motor speed thread
void* motorSpeed_thread(void *arg)
{
	int update_thread;

	while(1)
	{

		motorSpeed_flag = 1;
		msqid = msgget (msg_key, IPC_CREAT);

		hall_val_high = HB3_IP_High_Count();

		actual_speed = hall_val_high * 0.32;



		scaled_rpm = hall_val_high * 0.0085; //Scaled RPM value
		msg_speed.msg_value = scaled_rpm;

		//send the new set of states to the LED thread
		update_thread = msgsnd (msqid, &msg_speed, 2, 0);
		NX4IO_SSEG_setDigit(SSEGHI, DIGIT4,(int) (actual_speed/10000)%10);
		NX4IO_SSEG_setDigit(SSEGLO, DIGIT3,(int) (actual_speed/1000)%10);
		NX4IO_SSEG_setDigit(SSEGLO, DIGIT2,(int) (actual_speed/100) %10 ) ;
		NX4IO_SSEG_setDigit(SSEGLO, DIGIT1,(int) (actual_speed%100)/10);
		NX4IO_SSEG_setDigit(SSEGLO, DIGIT0,(int)  actual_speed%10);
		//check if the message was successfully sent to the message queue
		//display appropriate messages on the console
		if(update_thread != 0)
		{
			/*xil_printf("*******************************************\n\r");
			xil_printf("* Error sending message from Motor Speed thread*\n\r");
			xil_printf("*******************************************\n\n\r");*/
		}

		//		xil_printf("scaled_rpm %d*\n\r", scaled_rpm);
		//		xil_printf("*hall_val_high %d*\n\r", hall_val_high);

	}
	return NULL;

}




/****************************************************************************************/
/****************************************** PID thread **********************************/
/****************************************************************************************/
// The pid thread
void* pid_thread(void *arg)
{
	int msg_len, msg_len2;

	u16 send,rpm,incr, send_dc;
	u8 rc;
	int t;
	int msqid1, msqid2, msqid;
	while(1)
	{
		pid_flag = 1;

		msqid = msgget (msg_key, IPC_CREAT);
		//msqid = msgget (msg_key, IPC_CREAT);

		//wait for a new message
		msg_len = msgrcv (msqid1, &msg_ctr, 2, 0,0);
		msg_len = msgrcv (msqid2, &msg_speed, 2, 0,0);



		//check if message has been recieved
		//If not display error message on console
		if( (msg_len < 0) )
		{
			/*xil_printf("****************************************\n\r");
			xil_printf("* Error receiving message by PID thread*\n\r");
			xil_printf("****************************************\n\n\r");*/
		}

		rc = msg_ctr.msg_value;

		rpm = msg_speed.msg_value;


		error = (rc - scaled_rpm);
		pError = error;
		iError = oldError + error;
		dError = oldError - error;
		incr = (((kp*error)>>6) + ((ki*iError)>>6) + ((kd*dError)>>6)) ;

		send_dc = incr + rc + 10;
		//update the LEDs via gpio_led
		msg_cmd.msg_value = send_dc;
		t = msgsnd (msqid, &msg_cmd, 2, 0);
		//		set_rpm = rc * 47;


		//xil_printf("$%d %d;",RotaryCnt, send_dc);

		if(t != 0)
		{
			/*xil_printf("*******************************************\n\r");
			xil_printf("* Error sending message from PID thread*\n\r");
			xil_printf("*******************************************\n\n\r");*/
		}
		xil_printf("incr = %d\n\r", incr);
		xil_printf("Sent speed-Duty = %d\n\r", send_dc);
		/*xil_printf("****************************************\n\r");
		xil_printf("Sent speed-Duty = %d\n\r", send_dc);
		xil_printf("****************************************\n\r");
		xil_printf("****************************************\n\r");*/
		oldError = error;
	}
	return NULL;

}

/****************************************************************************************/
/****************************************** Command thread **********************************/
/****************************************************************************************/
// The command thread
void* motorCommand_thread(void *arg)
{
	int msqid;
	int msg_len;
	u16 send, speed;
	while(1)
	{

		motorCmd_flag = 1;
		msqid = msgget (msg_key, IPC_CREAT);

		//periodically tell the WDT that LED thread is being invoked
		//wait for a new message
		msg_len = msgrcv (msqid, &msg_cmd, 2, 0,0);
		/*xil_printf("*******************************************\n\r");
		xil_printf("*recieved msg for speed-DC = %d*\r\n",msg_cmd.msg_value);
		xil_printf("*******************************************\n\n\r");
		xil_printf("****************************************\n\r");*/
		//check if message has been recieved
		//If not display error message on console
		if(msg_len < 0)
		{
			/*xil_printf("****************************************\n\r");
			xil_printf("* Error receiving message by command thread*\n\r");
			xil_printf("****************************************\n\n\r");*/
		}

		speed = msg_cmd.msg_value ;
		/*xil_printf("****************************************\n\r");
		xil_printf("* message recieved by comand thread of speed-DC %d *\n\r",speed );
		xil_printf("****************************************\n\r");*/
		HB3_IP_setDutyCycle(speed);
		/*xil_printf("****************************************\n\r");
		xil_printf("****************************************\n\r");
		xil_printf("***SPEED SET****\n\r");
		xil_printf("****************************************\n\r");
		xil_printf("****************************************\n\r");*/
	}
	return NULL;

}





/****************************************************************************************/
/******************************* initialization function ********************************/
/****************************************************************************************/
// init_peripherals() - Initializes the peripherals
XStatus init_peripherals(void)
{
	XStatus status;
	/*// initialize the switch GPIO instance*/
	/*status = XGpio_Initialize(&SWInst, SW_GPIO_DEVICEID);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}*/
	// all ports configured as input ports

	// initialize the button GPIO instance
	status = XGpio_Initialize(&BTNInst, BTN_GPIO_DEVICEID);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}
	//all ports configured as input ports

	// initialize the LED GPIO instance
	/*status = XGpio_Initialize(&LEDInst, LED_GPIO_DEVICEID);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}*/
	//all ports configured as outputs
	//XGpio_SetDataDirection(&LEDInst, 1, 0x00);

	// initialize the watchdog timer
	status = XWdtTb_Initialize(&WDTInst,WDT_DEVICEID);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	// initialize the interrupt controller
	status = XIntc_Initialize(&IntrptCtlrInst, INTC_DEVICEID);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	// start the interrupt controller such that interrupts are enabled for
	// all devices that cause interrupts.
	status = XIntc_Start(&IntrptCtlrInst, XIN_REAL_MODE);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}
	status = XIntc_Start(&IntrptCtlrInst, XIN_REAL_MODE);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}
	// initialize the PMod544IO driver and the PmodENC and PmodCLP
	status = pmodENC_initialize(&pmodENC_inst, PMODENC_BASEADDR);
	if (status == XST_FAILURE)
	{
		exit(1);
	}
	// initialize the Nexys4 driver and (some of)the devices
	status = (uint32_t) NX4IO_initialize(NX4IO_BASEADDR);
	if (status == XST_FAILURE)
	{
		exit(1);
	}
	// initialize the PModHB3
	status = HB3_initialize(PMODHB3_BASEADDR);
	if (status == XST_FAILURE)
	{

		exit(1);
	}

	return XST_SUCCESS;

	return XST_SUCCESS;
}



/****************************************************************************************/
/******************************* button interrupt handler *******************************/
/****************************************************************************************/
// button press interrupt handler
void button_handler(void)
{
	int unlock_sem;

	//read the switch state
	gpio_pb = XGpio_DiscreteRead(&BTNInst, 1);

	//store the switch state in a varible thatcan be accessed by the button thread
	btn_state = gpio_pb;

	//unlock the semaphore
	unlock_sem = sem_post (&btn_press_sema);
	if(unlock_sem != 0)
	{
		/*xil_printf("****************************\n\r");
		xil_printf("* Error unlocking semaphore*\n");
		xil_printf("****************************\n\n\r");*/
	}
	acknowledge_interrupt(BTN_GPIO_INTR_NUM);
}



/****************************************************************************************/
/************************* watchdog timer interrupt handler *****************************/
/****************************************************************************************/
// WDT interrupt handler
void wdt_handler(void)
{
	//if system running flag is set, restart the watch dog timer
	if((sys_flag == 1||sys_flag ==0) && force_crash_flag == 0)
		XWdtTb_RestartWdt(&WDTInst);
	else
		xil_printf("Force Crash Successful!!!\n");
	acknowledge_interrupt(WDT_INTR_NUM);
}

