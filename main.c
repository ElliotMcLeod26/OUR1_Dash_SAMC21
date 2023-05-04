#include <atmel_start.h>
#include <stdio.h>
#include <string.h>
#include "DisplayNumbersAndText.h"
#include "SPI_io_descriptor.h"
#include "USER_DEF.h"

// liGHT comes on initially, wont turn off if receiving messages - if no message, if the bits not good
//NOTE: SPI baud rate changed to 1Mbaud in config file but not atmel start in the (sercom config file), check this boi

// Variables Global

// For the screen
float current = 0;
int dcl = 1;
float motor_voltage1 = 0;
float motor_voltage2 = 0;
float pack_voltage = 100;
int battery_charge = 100;
int battery_temp = 0;
int motor_temp1 = 0;
int motor_temp2 = 0;
int vcu_status[8];
int pack_dcl = 0;
int positive_slew_rate = 0;
int negative_slew_rate = 0;
int torque_vectoring_active = 0;

uint8_t AMS_active = 0; // start the AMS light NOT ready // state 0. 
// @@ NOTE LIGHT STARTS ON, in state 0. This is because for the first three seconds, 
// the light should be red and then only remain red if there are no messages OR if the bit logic is incorrect
uint8_t AMS_state = 1; // Record and send the state to the light (the actual state!) // should start on
uint8_t RTD_switch_state = 0;
uint8_t TS_button_state = 0;

bool TS_button_timer_started = false; //flags for controlling ts button state
bool TS_button_wait_for_release = false;

bool charge_safety;
bool discharge_state;
bool precharge_enabled;
bool logging_started;

static int max_timeout_ms = 10000; // can go max 500ms with no message`s

// Dealing with the timer
static unsigned int display_ms_counter = 0;
static unsigned int can_tx_ms_counter = 0; // This counts for each timer click to know when to send the can message 
static unsigned int msg_timeout_counter = 0; // to count if the message times-out
static unsigned int ts_button_counter = 0; // to count the 1 second button hold time

#define CAN_RX_BUFFER_LENGTH 64

#define DISCHARGE_STATE_BIT 0 // THIS IS BIT 0 -- MOVE BY 7
#define CHARGE_SAFETY_BIT 2 // THIS IS BIT 2 -- MOVE by 5


//Define Buffer - Needs to be volatile, but have to write volatile memcpy function

struct can_message CAN_buffer[CAN_RX_BUFFER_LENGTH];
volatile uint8_t CAN_buffer_head = 0;
volatile uint8_t CAN_buffer_tail = 0;
volatile uint8_t CAN_buffer_len = 0;

static struct can_message* can_buffer_push(void){
	struct can_message* ret = &CAN_buffer[CAN_buffer_head];
	
	if(CAN_buffer_head + 1 < CAN_RX_BUFFER_LENGTH) CAN_buffer_head++;
	else CAN_buffer_head = 0;
	
	if(CAN_buffer_len < CAN_RX_BUFFER_LENGTH) CAN_buffer_len++;
	
	return ret;
}

static struct can_message* can_buffer_pop(void){
	struct can_message* ret = &CAN_buffer[CAN_buffer_tail];
	
	if(CAN_buffer_tail + 1 < CAN_RX_BUFFER_LENGTH) CAN_buffer_tail++;
	else CAN_buffer_tail = 0;
	
	if(CAN_buffer_len > 0) CAN_buffer_len--;
	else return NULL;
	
	return ret;
}

// Timer
static struct timer_task DISPLAY_TIMER_task; // timer to display screen

static struct timer_task TIMER_AMS_task_prelight_three; // task to unlatch after the first three seconds

static struct timer_task send_can_timer_task; //task to time the sending of can messages

static struct timer_task MSG_TIMEOUT_TIMER_task; // count for max timeout from battery message

static struct timer_task TS_BUTTON_TIMER_task;

// for tt1
static void DISPLAY_TIMER_cb(const struct timer_task *const timer_task)
{
	display_ms_counter++; // toggle the counter for the display, this is instead of delays	
}

static void MSG_TIMEOUT_timer_cb(const struct timer_task *const timer_task)
{
	msg_timeout_counter++; // count the delay between messages	
}

static void TIMER_AMS_task_prelight_three_cb(const struct timer_task *const timer_task)
{
	// Here, we need to keep the light on for the three seconds before it becomes active
	// For the first three seconds the light will remain on anyway as it is set to start on. 
	// So just enable the messages and timeout AFTER these three seconds pass 
	// Note this is a one shot timer, so the code here will get executed at the end of the three seconds
	
	AMS_active = 1; // Now the AMS is active and ready to act the way it should based on the messages received and timing constraints. 
	AMS_state = 0;
	gpio_set_pin_level(AMS_LIGHT,AMS_state); // This happens at the end of the call, turn it off.
	msg_timeout_counter = 0; // now reset the counter;
}

static void TS_BUTTON_TIMER_cb(const struct timer_task *const timer_task){
	ts_button_counter++;
}


///////////////////////////////////// SEND CAN MESSAGE /////////////////////////////////////////////////////////
static void send_can_message()
{
	struct can_message msg_send1;

	uint8_t tx_data_1[8] = {RTD_switch_state,AMS_state,0,0,0,0,0,0};

	msg_send1.id = 0x469;
	msg_send1.type = CAN_TYPE_DATA;
	for(int i = 0; i < 8; i++)
		msg_send1.data[i] = tx_data_1[i];
	msg_send1.len = 8;
	msg_send1.fmt = CAN_FMT_STDID;
	
	struct can_message msg_send2;

	uint8_t tx_data_2[8] = {TS_button_state,0,0,0,0,0,0,0};

	msg_send2.id = 0x500;
	msg_send2.type = CAN_TYPE_DATA;
	for(int i = 0; i < 8; i++)
		msg_send2.data[i] = tx_data_2[i];
	msg_send2.len = 8;
	msg_send2.fmt = CAN_FMT_STDID;
	
	can_async_enable(&CAN_0);
	can_async_write(&CAN_0, &msg_send1);
	can_async_write(&CAN_0, &msg_send2);
}
///////////////////////////////////// SEND CAN MESSAGE /////////////////////////////////////////////////////////

static void send_can_timer_cb(const struct timer_task *const timer_task)
{
	can_tx_ms_counter++;
}

// CANS CALLBACK

static void CAN_0_tx_callback(struct can_async_descriptor *const descr) // just to not make it angry
{
	(void)descr;
}

// can handler
static void CAN_0_rx_callback(struct can_async_descriptor *const descr) // This is an interrupt, we will parse the messages
{
	struct can_message* msg = can_buffer_push();
	
	can_async_read(descr, msg);
	return;
}

void handle_can(void){
	struct can_message* msg = can_buffer_pop();
	
	if(msg == NULL) return;
	
	switch (msg->id)
	{
		// 0xnumbers means numbers in base 16
		
		case 0x008:
		if(msg->data[0] == 1){
			precharge_enabled = true;
		}
		else{
			precharge_enabled = false;
		}
		
		break;
		
		case 0x009:// AMS LIGHT READ RELAY STATE
		// Re toggle the message timer counter to 0 every time a message is received
		msg_timeout_counter = 0;  // @@ separate later into two different timeouts
		
		// It is 2 bytes, bit 0 is the discharge relay and bit 2 is the charge safety, if both 0, turn the light on.
		// Perform logic directly as this is important
		
		//checks least significant bits
		//charge_safety = ((msg.data[0]>>(7-CHARGE_SAFETY_BIT))&1);
		//discharge_state = ((msg.data[0]>>(7-DISCHARGE_STATE_BIT))&1);
		
		// message is 00 41 on non fault, and duirng fault it is B0 49
		
		//checks most significant bits
		charge_safety = ((msg->data[1]>>(CHARGE_SAFETY_BIT))&1); // shifts by 2 such that xxxx x?xx --> xxxx x?  this & 1 = ?
		discharge_state = ((msg->data[1]>>(DISCHARGE_STATE_BIT))&1); // shifts by 0 such that xxxx xxx? --> xxxx xxxx this & 1 = ?
		
		// Now apply this logic only if the light is on and the state is non-active (as this is latching)
		
		if (AMS_active == 1 && AMS_state == 0) // If the light is now active (responding to the messages) AND it is currently off (meaning that the car is safe)
		{
			if (charge_safety == 0 && discharge_state == 0) // If both are 0, meaning that neither one is high then the EMSDC loop will be OPEN so LED illuminated
			{
				AMS_state = 1;
				gpio_set_pin_level(AMS_LIGHT,AMS_state); // This cannot be turned off until the LV and HV is power cycled
			}
			
		}
		
		break;
		
		case 0x00A: // Temperature from battery in byte 1, in byte 3 is the charge
		
		// Re toggle the message timer counter to 0 every time a message is received
		//msg_timeout_counter = 0;
		
		battery_temp = msg->data[1];
		battery_charge = msg->data[3];
		
		break;
		
		case 0x101:
		logging_started = true;
		
		break;
		
		case 0x6B0: //message with instantaneous pack current and voltage
		pack_voltage = (msg->data[2]*256+msg->data[3])/10.0f;
		current = (msg->data[0]*256+msg->data[1])/10.0f;
		
		break;
		
		case 0x6B1: //message with discharge current limit
		
		dcl = msg->data[0]*256 + msg->data[1];
		
		break;
		
		case 0x7A1: //message with motor data 1
		
		motor_temp1 = msg->data[0]/2;
		motor_voltage1 = (msg->data[4]*256+msg->data[5])/10.0f;
		
		break;
		
		case 0x7A2: //message with motor data 2
		
		motor_temp2 = msg->data[0]/2;
		motor_voltage2 = (msg->data[4]*256+msg->data[5])/10.0f;
		
		break;
		
		case 0x7A4: //status message
		
		for(int i = 0; i < 8; i++)	vcu_status[i] = msg->data[i];
		
		break;
		
		case 0x7A5: //startup parameters message
		
		positive_slew_rate = msg->data[0]*256 + msg->data[1];
		negative_slew_rate = msg->data[2]*256 + msg->data[3];
		pack_dcl = msg->data[4]*256 + msg->data[5];
		torque_vectoring_active = msg->data[6];
		
		break;
		
		default:
		
		break;
	}
	
	return;
}

// code to turn can screen into display log
void create_log()
{
	// This was for the can display
	////for(uint8_t j = 0; j < CAN_RX_BUFFER_LENGTH; j++)
	//////uint8_t j = 0;
	////{
	////char temp_str[256];	//Placing 64 bytes represented as hex => 64*3 minimum chars needed
	////
	////char* temp_ptr = temp_str;
	////for(uint8_t i = 0; i < 16; i++)
	////{
	////temp_ptr += snprintf(temp_ptr, 4, "%02x ",CAN_rx_buffer[j][i]);
	////}
	////display_text(50, 25*j, 16, temp_str);
	////}
	////char temp_str[15];
	////snprintf(temp_str,15,"%u",framenum%100);
	////display_text(50, 250, 16, temp_str);

	//display_numberRight((int)(WIDTH-(0.03*WIDTH)),(int)((HEIGHT*0.3333)+(0.03*HEIGHT)),1,bob);
}

void display_handler()
{
	// Everything must happen within the Start and End frame placeholders
	startFrame();
	
	// Screen Display
	
	// AMMETER/VOLTMETER
	if(precharge_enabled){
		display_gauge((int)((motor_voltage1 < motor_voltage2 ? motor_voltage1 : motor_voltage2)/pack_voltage*100)); // voltmeter GAUGE
		display_text((int)(gauge_x0-(gauge_radius*0.4)),10,22,"VOLTMETER"); // TITLE
		display_text((int)(gauge_x0-(gauge_radius*0.4)),(int)(HEIGHT-(0.05*HEIGHT)),20,"% max voltage"); // UNIT
	}
	else{
		display_gauge((int)(current/dcl*100)); // ammeter GAUGE
		display_text((int)(gauge_x0-(gauge_radius*0.4)),10,22,"AMMETER"); // TITLE
		display_text((int)(gauge_x0-(gauge_radius*0.4)),(int)(HEIGHT-(0.05*HEIGHT)),20,"% max current"); // UNIT	
	}
	
	// SEPARATOR LINE (SECTION, SPEEDOMETER)
	display_line(VERT_X,0, VERT_X, HEIGHT, 3);
	
	// SERPARATOR LINES, RIGHT HAND SIDE SECTIONS
	display_line(VERT_X,(int)(HEIGHT*(0.3333)),WIDTH,(int)(HEIGHT*(0.3333)),3);
	display_line(VERT_X,(int)(HEIGHT*(0.6667)),WIDTH,(int)(HEIGHT*(0.6667)),3);
	
	// BATTERY CHARGE - read it from the buffer - third byte
	display_progress((battery_charge)/2);
	display_text((int)(VERT_X+(0.05*WIDTH)),10,22,"BATTERY CHARGE"); // TITLE
	
	// reset the buffer (later as i need to do the second part of the message) // buffer is already reset in the can
	
	// BATTERY TEMP
	display_text((int)(VERT_X+(0.05*WIDTH)),(int)((HEIGHT*0.3333)+(0.03*HEIGHT)),22,"BATTERY"); // TITLE
	display_text((int)(VERT_X+(0.05*WIDTH)),(int)((HEIGHT*0.3333)+(0.13*HEIGHT)),22,"TEMP"); // TITLE
	display_text((int)(VERT_X+(0.05*WIDTH)),(int)((HEIGHT*0.3333)+(0.23*HEIGHT)),22,"(degC)"); // TITLE
	// TO SET A FONT HIGHER THAN 31:
	FT8_cmd_romfont(1,33);
	//display_numberRight((int)(WIDTH-(0.03*WIDTH)),(int)((HEIGHT*0.3333)+(0.03*HEIGHT)),1,30); // The last argument is the value
	
	display_numberRight((int)(WIDTH-(0.03*WIDTH)),(int)((HEIGHT*0.3333)+(0.03*HEIGHT)),1,battery_temp); // The last argument is the value
	//display_numberRight((int)(WIDTH-(0.03*WIDTH)),(int)((HEIGHT*0.3333)+(0.03*HEIGHT)),1,discharge_state); // The last argument is the value
	
	//MOTOR TEMP
	display_text((int)(VERT_X+(0.05*WIDTH)),(int)((HEIGHT*0.6667)+(0.03*HEIGHT)),22,"MOTOR"); // TITLE
	display_text((int)(VERT_X+(0.05*WIDTH)),(int)((HEIGHT*0.6667)+(0.13*HEIGHT)),22,"TEMP"); // TITLE
	display_text((int)(VERT_X+(0.05*WIDTH)),(int)((HEIGHT*0.6667)+(0.23*HEIGHT)),22,"(degC)"); // TITLE
	// TO SET A FONT HIGHER THAN 31:
	FT8_cmd_romfont(1,33);
	display_numberRight((int)(WIDTH-(0.03*WIDTH)),(int)((HEIGHT*0.6667)+(0.03*HEIGHT)),1,motor_temp1 > motor_temp2 ? motor_temp1 : motor_temp2);
	
	//LOGGING STATUS
	display_text(0,0,20,"Logging:");
	if(logging_started)	display_textColor(WIDTH/11,0,20,"Active",0,128,0);
	else				display_textColor(WIDTH/11,0,20,"Inactive",200,0,0);
	
	endFrame();
}

void vcu_status_log_display(){
	startFrame();
	
	display_text(0,0,22,"EMSDC latched, power cycle to reset");
	
	const char* names[] = {"inv_err","prc_err","timeout","emsdc","ins_err","unused","unused","comms"};
	
	for(int i = 0; i < 8; i++){
		display_text(WIDTH*i/12,HEIGHT/3,20,names[i]);
		display_number(WIDTH*i/12,HEIGHT/2,22,vcu_status[i]);
	}
	
	endFrame();
}

void ignition_display(){
	startFrame();
	
	char positive_slew_rate_msgvar[30];
	sprintf(positive_slew_rate_msgvar,"Positive slew rate: %d",positive_slew_rate);
	const char *positive_slew_rate_msg = positive_slew_rate_msgvar;
	
	char negative_slew_rate_msgvar[30];
	sprintf(negative_slew_rate_msgvar,"Negative slew rate: %d",negative_slew_rate);
	const char *negative_slew_rate_msg = negative_slew_rate_msgvar;
	
	char max_power_msgvar[30];
	sprintf(max_power_msgvar,"Max current: %d A",pack_dcl);
	const char *max_power_msg = max_power_msgvar;
	
	display_text(WIDTH/4,1*HEIGHT/6,29,positive_slew_rate_msg);
	display_text(WIDTH/4,2*HEIGHT/6,29,negative_slew_rate_msg);
	display_text(WIDTH/4,3*HEIGHT/6,29,max_power_msg);
	if(torque_vectoring_active){
		display_textColor(WIDTH/4,4*HEIGHT/6,29,"Torque Vectoring Active",0,200,0);	
	}
	else{
		display_text(WIDTH/4,4*HEIGHT/6,29,"Torque Vectoring Inactive");
	}
	display_text(WIDTH/4,5*HEIGHT/6,30,"GOOD LUCK");
	
	endFrame();
	//display_logo(); doesn't work :(
}


int main(void)
{
	/* Initializers */
	atmel_start_init();
	gpio_set_pin_level(AMS_LIGHT,AMS_state); // Set the AMS light ON
	create_spi_descriptor();
	FT8_init();
	
	/*for(int i = 0; i < CAN_RX_BUFFER_LENGTH; i++){
		uint8_t data[8];
		CAN_buffer[i].data = data;
	}*/
	
	struct can_filter  filter;
	
	// can receive
	can_async_register_callback(&CAN_0, CAN_ASYNC_TX_CB, (FUNC_PTR)CAN_0_tx_callback); // do not anger the god
	can_async_register_callback(&CAN_0, CAN_ASYNC_RX_CB, (FUNC_PTR)CAN_0_rx_callback);
	
	filter.id = 0;
	filter.mask = 0;
	can_async_set_filter(&CAN_0,0,CAN_FMT_STDID, &filter);
	
	// timer set up
	
	DISPLAY_TIMER_task.interval = 1; // 1 ms interrupt
	DISPLAY_TIMER_task.cb       = DISPLAY_TIMER_cb;
	DISPLAY_TIMER_task.mode     = TIMER_TASK_REPEAT;
	timer_add_task(&TIMER_0, &DISPLAY_TIMER_task);
	
	MSG_TIMEOUT_TIMER_task.interval = 1; // 1 ms interrupt
	MSG_TIMEOUT_TIMER_task.cb       = MSG_TIMEOUT_timer_cb;
	MSG_TIMEOUT_TIMER_task.mode     = TIMER_TASK_REPEAT;
	timer_add_task(&TIMER_0, &MSG_TIMEOUT_TIMER_task);
	
	send_can_timer_task.interval = 1; // 1 ms interrupt
	send_can_timer_task.cb       = send_can_timer_cb;
	send_can_timer_task.mode     = TIMER_TASK_REPEAT;
	timer_add_task(&TIMER_0, &send_can_timer_task);
	
	TIMER_AMS_task_prelight_three.interval = 3000; // Interrupt after three seconds
	TIMER_AMS_task_prelight_three.cb       = TIMER_AMS_task_prelight_three_cb;
	TIMER_AMS_task_prelight_three.mode     = TIMER_TASK_ONE_SHOT;
	timer_add_task(&TIMER_0, &TIMER_AMS_task_prelight_three);
	
	TS_BUTTON_TIMER_task.interval = 1;
	TS_BUTTON_TIMER_task.cb = TS_BUTTON_TIMER_cb;
	TS_BUTTON_TIMER_task.mode = TIMER_TASK_REPEAT;
	timer_add_task(&TIMER_0, &TS_BUTTON_TIMER_task);
	
	timer_start(&TIMER_0);
	
	/* Functionality Loop */
	
	while (1) {
		
		// Check if the message length has gone over 
		if ((msg_timeout_counter > max_timeout_ms) && AMS_state == 0 && AMS_active == 1) // there have not been new messages received in a while and the light is off
		{
			AMS_state = 1; // light should be on
			gpio_set_pin_level(AMS_LIGHT,AMS_state);
		}
		
		if (display_ms_counter > 100) // Display every 100 ms
		{
			int i;
			for(i = 0; i < 5; i++){
				if(vcu_status[i] != 0) break;
			}
			
			if(i < 5) vcu_status_log_display();
			else if(TS_button_timer_started && precharge_enabled) ignition_display();
			else display_handler();
			
			
			display_ms_counter = 0;
		}
		
		
		if (gpio_get_pin_level(RTD_SWITCH)== true) // The car is in ready to drive mode!!
		{
			// toggle the state
			RTD_switch_state = 1;
			
			// The switch state is sent by the main loop using a timer activated by a counter every 100ms. 
		}
		else
		{
			RTD_switch_state = 0;
		}
		
		if (gpio_get_pin_level(TS_BUTTON)== true && !TS_button_wait_for_release)
		{
			if(!TS_button_timer_started){
				TS_button_timer_started = true;
				ts_button_counter = 0;
			}
			else if(ts_button_counter > 2500){
				TS_button_state = (TS_button_state+1)%2; //the ! flag for integers
				send_can_message(); //additional can message so that the car starts immediately
				TS_button_timer_started = false;
				TS_button_wait_for_release = true;
			}
			
		}
		else if(gpio_get_pin_level(TS_BUTTON) == false){
			TS_button_timer_started = false;
			TS_button_wait_for_release = false;
		}
		
		
		if (can_tx_ms_counter > 100) // send every 100ms
		{
			send_can_message();
			can_tx_ms_counter = 0;
		}
		
		handle_can();
		
	}
}
