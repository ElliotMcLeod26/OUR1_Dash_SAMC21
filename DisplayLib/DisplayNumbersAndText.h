
#include "FT8_commands.h"
#include "FT8.h"  //default values for draw and background color can be found in FT8.h
#include "FT8_config.h"
#include "USER_DEF.h"



// write this function at the beginning of every frame - clears previous frame and handles default values
void startFrame(){
  FT8_cmd_dl(CMD_DLSTART); /* start the display list */
  FT8_cmd_dl(DL_CLEAR_RGB | COLOR_RGB(backgroundColorR,backgroundColorG,backgroundColorB)); /* set the default clear the background color */
  FT8_cmd_dl(DL_CLEAR | CLR_COL | CLR_STN | CLR_TAG); /* clear the screen - this and the previous prevent artifacts between lists, Attributes are the color, stencil and tag buffers */
}

// write this function at the end of every frame - handles end of frame data prottocols to force screen update and flush buffer
void endFrame(){
  FT8_cmd_dl(DL_DISPLAY); /* instruct the graphics processor to show the list */
  FT8_cmd_dl(CMD_SWAP); /* make this list active */
  FT8_cmd_execute();
}


void display_number(int x, int y, int font, int number){
  //displays number at given coordinates and font(size)
  //(x,y) set the TOP LEFT point of the number
  //color is the default draw color
  FT8_cmd_dl(DL_COLOR_RGB | COLOR_RGB(defDrawColorR,defDrawColorG,defDrawColorB));
  FT8_cmd_number(x, y, font, 0, number);
}

void display_numberColor(int x, int y, int font, int number, byte R, byte G, byte B){
  //displays number at given coordinates and font(size)
  //(x,y) set the TOP LEFT point of the number
  //color is set by the input arguments
  FT8_cmd_dl(DL_COLOR_RGB | COLOR_RGB(R,G,B));
  FT8_cmd_number(x, y, font, 0, number);
}

void display_numberRight(int x, int y, int font, int number){
  //displays number at given coordinates and font(size)
  //(x,y) set the TOP RIGHT point of the number
  //color is the default draw color
  FT8_cmd_dl(DL_COLOR_RGB | COLOR_RGB(defDrawColorR,defDrawColorG,defDrawColorB));
  FT8_cmd_number(x, y, font, FT8_OPT_RIGHTX, number);
}

void display_numberColorRight(int x, int y, int font, int number, byte R, byte G, byte B){
  //displays number at given coordinates and font(size)
  //(x,y) set the TOP RIGHT point of the number
  //color is set by the input arguments
  FT8_cmd_dl(DL_COLOR_RGB | COLOR_RGB(R,G,B));
  FT8_cmd_number(x, y, font, FT8_OPT_RIGHTX, number);
}



void display_text(int x, int y, int font, const char* text){
  //displays text at given coordinates and font(size)
  //(x,y) set the TOP LEFT point of the text
  //color is the default draw color
  FT8_cmd_dl(DL_COLOR_RGB | COLOR_RGB(defDrawColorR,defDrawColorG,defDrawColorB));
  FT8_cmd_text(x, y, font, 0, text);
}

void display_textColor(int x, int y, int font, const char* text, byte R, byte G, byte B){
  //displays text at given coordinates and font(size)
  //(x,y) set the TOP LEFT point of the text
  //color is set by the input arguments
  FT8_cmd_dl(DL_COLOR_RGB | COLOR_RGB(R,G,B));
  FT8_cmd_text(x, y, font, 0, text);
}

void display_textRight(int x, int y, int font, const char* text){
  //displays text at given coordinates and font(size)
  //(x,y) set the TOP RIGHT point of the text
  //color is the default draw color
  FT8_cmd_dl(DL_COLOR_RGB | COLOR_RGB(defDrawColorR,defDrawColorG,defDrawColorB));
  FT8_cmd_text(x, y, font, FT8_OPT_RIGHTX, text);
}

void display_textColorRight(int x, int y, int font, const char* text, byte R, byte G, byte B){
  //displays text at given coordinates and font(size)
  //(x,y) set the TOP RIGHT point of the text
  //color is set by the input arguments
  FT8_cmd_dl(DL_COLOR_RGB | COLOR_RGB(R,G,B));
  FT8_cmd_text(x, y, font, FT8_OPT_RIGHTX, text);
}

void display_numbers_around_gauge(int x0, int y0, int radius, int font)
{
	// gauge is set up such that the bottom major divisions are at 45 degrees from the vertical center line
	// therefore, the other angles are placed accordingly depending on the number of major divisions + 1 (as it starts from 0) (default is 6, hence it will be 7)
	// consider the gauge being put inside a square and use the width and height accordingly to determine the proportions
	
	// gauge code is set to go from 0-60, therefore there will be 45 degree angle between each major sector. Feel free to change this if you wish
	
	// 0
	FT8_cmd_dl(DL_COLOR_RGB | WHITE);
	FT8_cmd_number((int)(x0-(radius*0.8)), (int)(y0+(radius*0.8)), font, FT8_OPT_RIGHTX, 0);
	
	//commands for 0-60 gauge
	//FT8_cmd_number((int)(x0-(radius*1.1)), (int)(y0-(0.1*radius)), font, FT8_OPT_RIGHTX, 10);
	
	//FT8_cmd_number((int)(x0-(radius*0.8)), (int)(y0-(radius*0.9)), font, FT8_OPT_RIGHTX, 20);
	
	//FT8_cmd_number((int)(x0-(radius*0.05)), (int)(y0-(1.3*radius)), font, 0, 30);
	
	//FT8_cmd_number((int)(x0+(radius*0.8)), (int)(y0-(radius*0.9)), font, 0, 40);
	
	//FT8_cmd_number((int)(x0+(radius*1.1)), (int)(y0-(0.1*radius)), font, 0, 50);
	
	//FT8_cmd_number((int)(x0+(radius*0.8)), (int)(y0+(radius*0.8)), font, 0, 60);

	//commands for 0-100 gauge
	FT8_cmd_number((int)(x0-(radius*1.05)), (int)(y0-(0.33*radius)), font, FT8_OPT_RIGHTX, 20);
	
	FT8_cmd_number((int)(x0-(radius*0.4)), (int)(y0-(radius*1.12)), font, FT8_OPT_RIGHTX, 40);
	
	FT8_cmd_number((int)(x0+(radius*0.4)), (int)(y0-(1.12*radius)), font, 0, 60);
	
	FT8_cmd_number((int)(x0+(radius*1.05)), (int)(y0-(radius*0.33)), font, 0, 80);
	
	FT8_cmd_number((int)(x0+(radius*0.8)), (int)(y0+(radius*0.8)), font, 0, 100);
}

void display_gauge_number_large(int x0, int y0, int radius, int font, int val)
{
	// Set up the rectangle in which the large number will be displayed
	FT8_cmd_dl(DL_COLOR_RGB | WHITE);
	FT8_cmd_rect((int)(x0-(0.375*radius)), (int)(y0+(1.05*radius)), (int)(x0+(0.375*radius)),(int)(y0+(0.625*radius)), 1);
	
	// Set up the actual number display
	FT8_cmd_dl(DL_COLOR_RGB | BLUE);
	FT8_cmd_number((int)(x0+(0.2*radius)), (int)(y0+(0.625*radius)), font, FT8_OPT_RIGHTX, val);
	
}

void display_gauge(int val)
{
	FT8_cmd_dl(DL_COLOR_RGB | WHITE);
	
	// Check the speed and match COLOUR
	if (val <= (int)(gauge_range*0.5))
	{
		FT8_cmd_bgcolor(DL_COLOR_RGB | GREENG);
	} 
	else if (val <= (int)(gauge_range*0.75))
	{
		FT8_cmd_bgcolor(DL_COLOR_RGB | YELLOWG);
	}
	else
	{
		FT8_cmd_bgcolor(DL_COLOR_RGB | REDG);
	}

	FT8_cmd_gauge(gauge_x0, gauge_y0, gauge_radius, 0, gauge_major_increm, gauge_minor_increm, val, gauge_range); // major = 6 (6 increments of 60 is sections of 10) and minor is increments of 5 (so sections of 2)
	
	display_numbers_around_gauge(gauge_x0, gauge_y0, gauge_radius, gauge_big_number-2);
	display_gauge_number_large(gauge_x0, gauge_y0, gauge_radius, gauge_big_number, val);
}

void display_line(int x0, int y0, int x1, int y1, int w0)
{
	FT8_cmd_dl(DL_COLOR_RGB | WHITE);
	FT8_cmd_line(x0,y0,x1,y1,w0);
}


void display_progress(int val)
{
	
	// Check the battery value and match the colour
	if (val <= (int)(battery_range*0.3333))
	{
		FT8_cmd_dl(DL_COLOR_RGB | REDG);
	}
	else if (val <= (int)(battery_range*0.6667))
	{
		FT8_cmd_dl(DL_COLOR_RGB | YELLOWG);
	}
	else
	{
		FT8_cmd_dl(DL_COLOR_RGB | GREENG);
	}
	
	
	FT8_cmd_bgcolor(DL_COLOR_RGB | GRAY);

	FT8_cmd_progress(VERT_X+(int)(0.1*WIDTH), (int)(HEIGHT*0.13), (int)(((1-VERT_X_PROPORTION-0.1)*0.7)*WIDTH), (int)(HEIGHT*0.1667), 0, val, battery_range); //range 100
	
	display_numberColorRight(10+VERT_X+(int)(0.1*WIDTH)+(int)(((1-VERT_X_PROPORTION-0.1)*0.7)*WIDTH)/2,(int)(HEIGHT*0.13)+8,29,val,255,255,255);
	display_textColor(10+VERT_X+(int)(0.1*WIDTH)+(int)(((1-VERT_X_PROPORTION-0.1)*0.7)*WIDTH)/2,(int)(HEIGHT*0.13)+8,29,"%",255,255,255);
}


void display_text_number(int x, int y, int font, const char* text, int number){
  //displays text next to the number (in this order)
  //(x,y) set the top of the line between the text and the number => BE CAREFULL with alignment
  //color is the default draw color
  display_textRight(x, y, font, text);
  display_number(x, y, font, number);
}

void display_text_numberColor(int x, int y, int font, const char* text, int number, byte R, byte G, byte B){
  //displays text next to the number (in this order)
  //(x,y) set the top of the line between the text and the number => BE CAREFULL with alignment
  //color is set by the input arguments
  display_textColorRight(x, y, font, text, R, G, B);
  display_numberColor(x, y, font, number, R, G, B);
}

void display_logo(){
	const uint8_t data[] = {0,0,0,0,0,255,255,255,255,255,0,0,0,0};
	FT8_cmd_loadimage(0,0,data,sizeof(data));
}