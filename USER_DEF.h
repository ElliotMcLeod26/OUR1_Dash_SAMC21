/*
 * IncFile1.h
 *
 * Created: 30/09/2022 22:29:13
 *  Author: Codrutza Dragu
 *
 *
 * File outlining all of the user-changeable variables
 *
 */ 


#ifndef INCFILE1_H_
#define INCFILE1_H_

/**** COLOR DEFINITIONS ****/

/** Text and Number and Background (General) **/

//setting constants for background color 
#define backgroundColorR  127
#define backgroundColorG  127
#define backgroundColorB  127

//setting constants for default draw color
#define defDrawColorR  255
#define defDrawColorG  255
#define defDrawColorB  255

/** Gauge **/

// predefined colours for the gauge (not too bright!!)
#define GREENG 0x047004
#define YELLOWG 0xe3a70e
#define REDG 0xb51616
#define GRAY 0xC0C0C0

/**** SCREEN SIZE DEFINIIONS ****/

// In the Config File: Need type long, so define the screen size as follows:

/*** WHEN CHANGING SCREEN ***/
/*

For the 4.2" screen the values should be:
#define VSIZE 272L //Tvd Number of visible lines (in lines) - display height
#define HSIZE 480L // Thd Length of visible part of line (in PCLKs) - display width

For the 5" screen the valuse should be
#define VSIZE 480L //Tvd Number of visible lines (in lines) - display height
#define HSIZE 800L // Thd Length of visible part of line (in PCLKs) - display width

Everything in the code is coded wrt to WIDTH and HEIGH vars which are the second set of variables below:
*/

// Only change the following as per above:
#define VSIZE 272L //Tvd Number of visible lines (in lines) - display height
#define HSIZE 480L // Thd Length of visible part of line (in PCLKs) - display width

// Do not change this, it is only to cast the height and width specified above for use in the rest of the code, which is written with respect to the screen size proportions.
#define HEIGHT (int)FT8_VSIZE
#define WIDTH (int)FT8_HSIZE

/**** GAUGE VALUES DEFINITIONS ****/
#define gauge_radius (int)(0.35*HEIGHT)
#define gauge_range 100
#define gauge_major_increm 10
#define gauge_minor_increm 5
#define gauge_big_number 25
#define gauge_x0 (int)(0.3*WIDTH)
#define gauge_y0 (int)(0.57*HEIGHT)

/**** VERTICAL SECTION LINE DEFINITIONS ****/
#define VERT_X_PROPORTION 0.6
#define VERT_X (int)(VERT_X_PROPORTION*WIDTH)

/**** BATTERY PERCENTAGE DISPLAY ****/
#define battery_range 100





#endif /* INCFILE1_H_ */