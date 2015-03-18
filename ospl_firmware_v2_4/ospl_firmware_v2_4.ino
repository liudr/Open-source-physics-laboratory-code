/** \file
 *  \brief     This is the firmware of Dr. Liu's Open Source Physics Laboratory Data Acquisition System with 16x2 display and a rotary encoder. This version supports PCB version up to V 2.4.X
 *  \details   This software requires phi_interfaces and phi_prompt libraries for user interaction. It also needs sdfat library for SD card, RTCLib library for real time clock, and I2CDev library for the sensor board.
 * This sample code only uses simple functions from phi_prompt library. For more control over the aspects of the library, read the phi_prompt documentation and try other functions.
 *  \author    Dr. John Liu
 *  \version   2.4.6
 *  \date      03/18/2015
 *  \pre       Compatible with Arduino IDE 1.0.6, and 1.6.0, with OSPL modifications.
 *  \warning   PLEASE DO NOT REMOVE THIS COMMENT WHEN REDISTRIBUTING! No warranty!
 *  \copyright GNU GPL V3.0
 *  \par Contact
 * Obtain the documentation or find details of the phi_interfaces, phi_prompt TUI library, and hardware or contact Dr. Liu at:
 *
 * <a href="http://liudr.wordpress.com/phi_interfaces/">http://liudr.wordpress.com/phi_interfaces/</a>
 *
 * <a href="http://liudr.wordpress.com/phi-panel/">http://liudr.wordpress.com/phi-panel/</a>
 *
 * <a href="http://liudr.wordpress.com/phi_prompt/">http://liudr.wordpress.com/phi_prompt/</a>
 *
 * <a href="http://liudr.wordpress.com/phi-2-shield/">http://liudr.wordpress.com/phi-2-shield/</a>
 *
 *  \par Updates
 * 03/18/2015: Created github repository open-source-physics-laboratory-code to host firmware and sample code
 * 03/09/2015: Added adjusting delay parameters and credits.
 * 03/05/2015: Changed to .TXT file. Added adjusting clock.
 * 03/04/2015: Displaying clock has been added under I2C.
 * 03/03/2015: Added data logging to the routine. Sending to serial port is replaced by saving to sd card if sd card exists (SD_card_present=1).
 * 03/02/2015: Added code to support SD card and RTC. Need to clean up the code and test it.
 * 02/13/2015: Updated a few string values.
 * 02/12/2015: Made main code and phi_prompt library compatible with Arduino IDE 1.6.0. Also removed older definitions from firmware version 2.4.1.
 * 02/10/2015: Adding hardware pin assignment for PCB version 2.4.5
 * 11/06/2013: Modified regulated_voltage to 5.00 for AC adapter or battery operation.
 * 10/07/2013: Added Vernier conductivity probe 3 settings.
 * 07/12/2013: Adjusted the behavior for turning the knob.
 * 07/11/2013: Added ADXL345 BMP085 and HMC5883L support. Also moved menus and formatting texts to PROGMEM.
 * 01/11/2013: Added PASCO sonic ranger support to sonic ranger.
 * 01/10/2013: Added OSPL sonic ranger code.
 * 01/08/2013: Added Vernier sonic ranger code and analog/digital sensor selection screen. Added mockup photogate screen.
 * 01/03/2013: Modified auto_ID_pullup value to 10Kohm to correspond to what I soldered on the V 2.0 prototype.
 * 01/02/2013: Modified to use on OSPL V2.0 hardware.
 * 09/26/2012: Added an AUTO_ID resistor sensing routine to find out these resistor values. Fixed problem on channel 1 auto_id reading. Student didn't solder the IC socket well enough to the PCB so A3 is not connected to channel 1.
 * 08/19/2012: Changed the direct temp probe a b and unit from 100, 0, F to 55.55 -17.7 C.
 * After some testing with internal pullup and two external pulldown I found out the internal pullup to be around 32K ohm and the voltage drop across the gate (enabling the internal pullup) to be 0.35V.
 * I measured regulated voltage to be 5.05V both with USB and Vin.
 * So effectively the voltage divider with internal pullup and one external pulldown resistor is 4.7V. After these numbers were entered into the formula, the temperature reading of the TMP-BTA is around 0.5C apart from LabQuest between 25C and 50C.
 * More accurate measurement could make the reading more accurate but at the moment the problem is solved.
 * 08/18/2012: Added Vernier gas pressure sensor (GPS-BTA) to the list of sensors. This sensor has a different a value than the Pressure sensor (PS-DIN).
 * Added Veriner temperature sensor (TMP-BTA) and modified the read_channel to read voltage divider and convert to temperature with Steinhart-Hart equation.
 * I realized that one needs a pullup resistor when using the TMP-BTA but when a 10K ohm external pullup resistor is used with analog sensors, the reading are off, far off for DT-DIN (should be 0.75V but saw 4.4V)
 * I later desoldered the external pullup and measured the internal pullup to be around 38.9K ohm. I programmed the select_sensors() to engage internal pullup for TMP-BTA and disengage it for other sensors.
 * The result is not very accurate. When calibrated for 20 C the reading will be off about more than 1 C at 40 C. I will investigate it with external pullup to see if the internal pullup/its enabling gate is the source of the problem.
 * 02/29/2012: Tested the system to work perfectly with numbers and PC upload.
 * 02/28/2012: Fixed floating point printing problem with format_fp() function.
 * 02/24/2012: Tested on dual force sensor and student force sensor. The floating point printing is not correct when printing negative numbers and when it gets close to X.0
 * 02/20/2012: To be tested on prototype.
 * 02/19/2012: Added select_sensors(), list_sensors(), display_sensors(), read_sensor() and format_output().
 * 02/18/2012: The code was tested to work on phi-2 shield with 20x4 display.
 * 02/15/2012: The code was tested to work on phi-lab.
*/

#include <EEPROM.h>
#include <avr/pgmspace.h>
#include <LiquidCrystal.h>      ///< Include the liquid crystal library
#include <phi_interfaces.h>     ///< Include the phi_interfaces input devices library
#include <phi_prompt.h>         ///< Include the phi_prompt user interface library
#include <Wire.h>
#include <I2Cdev.h>
#include <ADXL345.h>
#include <BMP085.h>
#include <HMC5883L.h>
#include "../Wire/utility/twi.h"
#include <SdFat.h>
#include <RTClib.h>
#include "gates.h"
#include "defs.h"

int useless;
// Please don't modify this section. Find the end of this section and you may modify content beyond the end.
#define OSPL_V2_4

#define total_channels 3        ///< There are a total of 3 analog sensor channels on OSPL. If you connected the jumpers inside the unit to wire I2C to the last channel, you will only have 2 left.
#define UI_delay_def 200        ///< Delay when running the prototype with LCD output
#define serial_delay_def 10     ///< Delay when running the prototype with serial output
#define NO_VENDORs 3            ///< No vendors
#define regulated_voltage 5.000 ///< This is measured at V+ when powered by external source, such as battery and AC adapter. This voltage is accurate up to mV for battery and AC adapter. Powering the OSPL with USB is a bad thing. The voltage is around 5.05V and is fluctuating too much.
#define effective_voltage_with_internal_pullup 4.7  ///< This is the effective voltage of the voltage divider formed between the internal pullup and an external pulldown resistor. The gate that enables internal pullup drops 0.35V across.
#define analog_external_pullup 1e4       ///< This is the external pullup resistor on every sensor analog channel. It is not present in PCB version 2.0
#define auto_ID_pullup 1e4      ///< This is the external pullup resistor on every autoID analog channel
#define internal_pullup 3.22e4  ///< Actually measured internal pullup are 39.1 38.8 and 38.7 with my low-quality meter for channels 0, 1, and 2 (analog channels 0,2,4).
#define analog_sensors 0        ///< Selection choice of analog sensors at the beginning.

#define digital_sensors 1       ///< Selection choice of digital sensors at the beginning.
#define sonic_ranger 0          ///< Selection choice for sonic ranger.
#define OSPL_sonic_ranger 1     ///< Selection choice for OSPL sonic ranger.
#define photogate 3             ///< Selection choice for photogate
#define photogate_speed_mode 1 ///< Both photogates are measuring speed independently
#define photogate_acceleration_mode 2 ///< Both photogates are measuring acceleration independently
#define photogate_acceleration_2_gate_mode 3 ///< Both photogates are measuring speed independently and report one acceleration as change of speed over change in time
#define photogate_delay_mode 4 ///< Both photogates are measuring time independently and report time between triggering the two gates
#define photogate_angular_speed_mode 5 ///< Both photogates are measuring angular speed independently
#define photogate_angular_acceleration_mode 6 ///< Both photogates are measuring angular acceleration independently
#define Pasco_photogate 0
#define Pasco_sonic_ranger 1

#define i2c_sensors 2           ///< Selection choice of I2C sensors at the beginning.
#define ADXL345_sensor 0
#define ADXL_2G_factor (0.0039)

#define BMP083_sensor 1
#define HMC5883L_1090_factor (1/1.090) // 1090Lsb/G=1.09Lsb/mG in 1090 gain mode.
#define HMC5883L_sensor 2
#define DS1307_sensor 3

#define settings 3              ///< Selection choice of settings at the beginning.
#define UI_delay_setting 0      ///< Selection of UI_delay in settings
#define Serial_delay_setting 1  ///< Selection of serial_delay in settings
#define RTC_setting 2           ///< Selection of real-time clock in settings
#define credits 3               ///< Selection of show credits in settings

#define two_pi 6.2831853
#define spoke_number 10

#ifdef OSPL_V2_4                ///< The following are for OSPL V2.4 hardware

#define lcd_rows 2              ///< Specify the height of the LCD.
#define lcd_columns 16          ///< Specify the width of the LCD.

#define total_buttons 1         ///< The total number of push buttons in a buttons group object. This is needed to instantiate a phi_button_groups object
#define encoder_detents 18      ///< How many detents per rotation
#define ch_b 27                 ///< I/O pin for rotary encoder channel b
#define ch_a 26                 ///< I/O pin for rotary encoder channel a
#define btn_b 24                ///< I/O pin for a button

#define LCD_RS 4                ///< Arduino pin connected to LCD RS pin
#define LCD_EN 5                ///< Arduino pin connected to LCD EN pin
#define LCD_D4 6                ///< Arduino pin connected to LCD D4 pin
#define LCD_D5 7                ///< Arduino pin connected to LCD D5 pin
#define LCD_D6 9                ///< Arduino pin connected to LCD D6 pin
#define LCD_D7 8                ///< Arduino pin connected to LCD D7 pin
#define LCD_BL 31              ///< Arduino pin connected to LCD back light LED

#define speaker 25              ///< Arduinopin connected to the speaker that plays simple tones.

#define SD_card_chip_select 10

byte pins_buttons[]={btn_b};       ///< The digital pins connected to the buttons are included.
char mapping_buttons[]={'B'};      ///< This is a list of names for each button.
byte pins_encoder[]={ch_a,ch_b};   ///< The digital pins connected to rotary encoder channels.
char mapping_encoder[]={'U','D'};  ///< This is a list of names for each button.

// The following lines instantiate several keypads.
phi_button_groups* my_btns= new phi_button_groups(mapping_buttons, pins_buttons, total_buttons);    ///< This instantiates a button group to sense the buttons.
phi_rotary_encoders * rotary_keypad= new phi_rotary_encoders(mapping_encoder,pins_encoder[0],pins_encoder[1],encoder_detents);    ///< This is the rotary encoder up and down.

// The following adds all available keypads as inputs for phi_prompt library
multiple_button_input * keypads[]={my_btns,rotary_keypad,0};   ///< Two keypads are added, the actual three buttons, and a serial keypad. Note that the array needs to be zero terminated.
#endif

// The following sets up function keys for phi_prompt library
char up_keys[]={"U"};           ///< All keys that act as the up key are listed here. Must be terminated with a zero. You can terminate with zero by using double quotation.
char down_keys[]={"D"};         ///< All keys that act as the down key are listed here. Must be terminated with a zero. You can terminate with zero by using double quotation.
char left_keys[]={"L"};         ///< All keys that act as the left key are listed here. Must be terminated with a zero. You can terminate with zero by using double quotation.
char right_keys[]={"R"};        ///< All keys that act as the right key are listed here. Must be terminated with a zero. You can terminate with zero by using double quotation.
char enter_keys[]={"B"};        ///< All keys that act as the enter key are listed here. Must be terminated with a zero. You can terminate with zero by using double quotation.
char escape_keys[]={"A"};       ///< All keys that act as the escape key are listed here. Must be terminated with a zero. You can terminate with zero by using double quotation.
char * function_keys[]={up_keys,down_keys,left_keys,right_keys,enter_keys,escape_keys}; ///< All function key names are gathered here fhr phi_prompt.

LiquidCrystal lcd(LCD_RS,LCD_EN,LCD_D4,LCD_D5,LCD_D6,LCD_D7);   ///< Create the lcd object

// The following are variables for the channels
byte channel_sensor_vendors[]={NO_VENDORs,NO_VENDORs,NO_VENDORs,NO_VENDORs};
byte channel_sensor_types[]={NO_VENDORs,NO_VENDORs,NO_VENDORs,NO_VENDORs};
byte channel_pins[]={A0,A2,A4,A0,A1};   ///< These are analog pins for channels 0-2 and digital pins for channel 3

#define photogate_pin0 channel_pins[3]
#define photogate_pin1 channel_pins[4]
#define sonic_ranger_init channel_pins[3]
#define sonic_ranger_echo channel_pins[4]
// End of the Please don't modify section
gates * gate_sensors[2];
byte in_polarity=0;


// The following are lists of vendor names and sensor names of each vendor
#define Vernier 0
#define Pasco 1

#define TMP_BTA 1 // This is where that item is located in the vernier_sensor_names list so it needs to be changed if the list is updated and its order is changed.
#define AUTO_ID 7 // This is where that item is located in the vernier_sensor_names list so it needs to be changed if the list is updated and its order is changed.
#define TMP_BTA_K0 1.02119e-3
#define TMP_BTA_K1 2.22468e-4
#define TMP_BTA_K2 1.33342e-7

char formatting_buffer[48];
char buffer[256]; ///< For buffering menus
char float_buffer[4][8]; ///< For storing floating point number for printing

const char PROGMEM PROGMEM_msg0[]="Open Source";                ///< This message is stored in PROGMEM to save SRAM.
const char PROGMEM PROGMEM_msg1[]="Physics Lab V2.4";                ///< This message is stored in PROGMEM to save SRAM.
const char PROGMEM PROGMEM_msg2[]="Select function";              ///< This message is stored in PROGMEM to save SRAM.
const char PROGMEM serial_uploading_msg[]="Serial uploading";    ///< This message is stored in PROGMEM to save SRAM.
const char PROGMEM sd_card_logging_msg[]="Logging to SD";
const char PROGMEM sensor_type_menu[]="Select function:\nAnalog Sensor\nDigital Sensor\nI2C Sensor\nSettings\n";
const char PROGMEM vendors_menu[]="Vendor:\nVernier\nPasco\nOSPL\nNo sensor\n";
const char PROGMEM vernier_analog_sensors_menu[]="Vernier:\nDirect Temp\nTemperature\nForce +-10N\nForce +-50N\nPressure PS-DIN\nGas Pressure\nVoltage\nAUTO_ID\nCond. 200uS/cm\nCond. 2KuS/cm\nCond. 20KuS/cm\nAccelerometer\n";
const char PROGMEM digital_sensors_menu[]="Digital sensor:\nVernier Ranger\nOSPL Ranger\nPhotogate\n";
const char PROGMEM i2c_sensors_menu[]="I2C sensor:\nADXL345 Accele\nBMP083 Baromet\nHMC5883L maget\n"; // L3G4200D gyros\n
const char PROGMEM photogate_modes_menu[]="Mode:\nSpeed\nAcceleration\nAcc. 2 gate\nDelay\nAng. Speed\nAng. Acc.\n";
const char PROGMEM no_sensors_menu[]="a\nNo sensor\n";
const char PROGMEM pasco_sensors_menu[]="Pasco:\nPhotogate\nSonic ranger\n";
const char PROGMEM ospl_sensors_menu[]="OSPL:\nNo sensor\n";
const char PROGMEM settings_menu[]="Settings:\nUI delay\nSer./SD delay\nSet clock\nShow credits\n";
const char PROGMEM credits_text[]="Open source physics laboratory data acquisition system\nV2.4 Mar. 2015\nDr. Liu\nSaint Cloud State University, MN USA\nPCB CC-BY-SA\nFirmware: GNU GPL V3";

const char* sensor_names[]={vernier_analog_sensors_menu,pasco_sensors_menu,ospl_sensors_menu,no_sensors_menu};

// The following are sensor output formatting strings for each sensor from each vendor. The formatting string has to follow a strict rule of "%Xd.%0Yd" and may carry units. The expected output must be exactly 8 characters to ensure the previous output is erased by a new one.
const char PROGMEM vernier_sensor_output_format0[]="%3d.%01d C "; ///< Direct temperature probe DCT-DIN
const char PROGMEM vernier_sensor_output_format1[]="%3d.%01d C "; ///< Temperature probe TMP-BTA
const char PROGMEM vernier_sensor_output_format2[]="%3d.%02dN ";  ///< Dual range force sensor DFS-DIN or DFS-BTA 10N range
const char PROGMEM vernier_sensor_output_format3[]="%3d.%01dN ";  ///< Dual range force sensor DFS-DIN or DFS-BTA 50N range
const char PROGMEM vernier_sensor_output_format4[]="%1d.%03dATM"; ///< Pressure sensor PS-DIN
const char PROGMEM vernier_sensor_output_format5[]="%1d.%03dATM"; ///< Gas Pressure sensor GPS-BTA
const char PROGMEM vernier_sensor_output_format6[]="%1d.%03d V "; ///< Voltage probe
const char PROGMEM vernier_sensor_output_format7[]="%2d.%01d KO "; ///< AUTO_ID resistor probe
const char PROGMEM vernier_sensor_output_format8[]="%3d.%01duS"; ///< Conductivity CON-BTA at 200uS
const char PROGMEM vernier_sensor_output_format9[] ="%1d.%03dmS"; ///< Conductivity CON-BTA at 2mS
const char PROGMEM vernier_sensor_output_format10[]="%2d.%02dmS"; ///< Conductivity CON-BTA at 20mS
const char PROGMEM vernier_sensor_output_format11[]="%2d.%03dg "; ///< Accelerometer 3D-BTA, any axis
const char * vernier_sensor_output_formats[]={vernier_sensor_output_format0,vernier_sensor_output_format1,vernier_sensor_output_format2,vernier_sensor_output_format3,vernier_sensor_output_format4,vernier_sensor_output_format5,vernier_sensor_output_format6,vernier_sensor_output_format7,vernier_sensor_output_format8,vernier_sensor_output_format9,vernier_sensor_output_format10,vernier_sensor_output_format11};
const char ** (sensor_LCD_output_formats[])={vernier_sensor_output_formats,0,0,0};

const char PROGMEM vernier_sensor_serial_output_format0[]="%3d.%01d";
const char PROGMEM vernier_sensor_serial_output_format1[]="%3d.%01d";
const char PROGMEM vernier_sensor_serial_output_format2[]="%3d.%02d";
const char PROGMEM vernier_sensor_serial_output_format3[]="%3d.%01d";
const char PROGMEM vernier_sensor_serial_output_format4[]="%1d.%03d";
const char PROGMEM vernier_sensor_serial_output_format5[]="%1d.%03d";
const char PROGMEM vernier_sensor_serial_output_format6[]="%1d.%03d";
const char PROGMEM vernier_sensor_serial_output_format7[]="%2d.%01d"; ///< AUTO_ID resistor probe
const char PROGMEM vernier_sensor_serial_output_format8[]="%3d.%01d uS/cm"; ///< Conductivity CON-BTA at 200uS
const char PROGMEM vernier_sensor_serial_output_format9[]="%1d.%03dmS"; ///< Conductivity CON-BTA at 2mS
const char PROGMEM vernier_sensor_serial_output_format10[]="%2d.%02dmS"; ///< Conductivity CON-BTA at 20mS
const char PROGMEM vernier_sensor_serial_output_format11[]="%2d.%03dg"; ///< Accelerometer 3D-BTA, any axis
const char * vernier_sensor_serial_output_formats[]={vernier_sensor_serial_output_format0,vernier_sensor_serial_output_format1,vernier_sensor_serial_output_format2,vernier_sensor_serial_output_format3,vernier_sensor_serial_output_format4,vernier_sensor_serial_output_format5,vernier_sensor_serial_output_format6,vernier_sensor_serial_output_format7,vernier_sensor_serial_output_format8,vernier_sensor_serial_output_format9,vernier_sensor_serial_output_format10,vernier_sensor_serial_output_format11};
const char ** (sensor_serial_output_formats[])={vernier_sensor_serial_output_formats,0,0,0};

const char PROGMEM ADXL_formatting0[]="%ld x:%sg y:%sg z:%sg T:%sg\n\r";
const char PROGMEM ADXL_formatting1[]="X:%sgY:%sg";
const char PROGMEM ADXL_formatting2[]="Z:%sgT:%sg";

const char PROGMEM BMP085_formatting0[]="%ld P:%sPa Alt:%dm T:%sDegC\n\r";
const char PROGMEM BMP085_formatting1[]="%sPa %s""\xDF""C";
const char PROGMEM BMP085_formatting2[]="%4dm  Altitude";

const char PROGMEM HMC5883L_formatting0[]="%ld x:%4dmG y:%4dmG z:%4dmG Heading:%3dDeg\n\r";
const char PROGMEM HMC5883L_formatting1[]="X:%4dmGY:%4dmG";
const char PROGMEM HMC5883L_formatting2[]="Z:%4dmG ~%3d""\xDF";

const char PROGMEM err_msg_00[]="Error: ";
const char PROGMEM err_msg_01[]="SD Card failed\n";
const char PROGMEM err_msg_02[]="Failed to create file\n";


const char PROGMEM sd_msg_00[]="Type any character to start\n";
const char PROGMEM sd_msg_01[]="Starting SD card\n";
const char PROGMEM sd_msg_02[]="SD card started.\n";
const char PROGMEM sd_msg_03[]="File:s1_Data.csv\n";
const char PROGMEM sd_msg_04[]="RTC failed\n";


const char PROGMEM month_00[]="JAN";
const char PROGMEM month_01[]="FEB";
const char PROGMEM month_02[]="MAR";
const char PROGMEM month_03[]="APR";
const char PROGMEM month_04[]="MAY";
const char PROGMEM month_05[]="JUN";
const char PROGMEM month_06[]="JUL";
const char PROGMEM month_07[]="AUG";
const char PROGMEM month_08[]="SEP";
const char PROGMEM month_09[]="OCT";
const char PROGMEM month_10[]="NOV";
const char PROGMEM month_11[]="DEC";
const char PROGMEM * const month_items[]= {month_00,month_01,month_02,month_03,month_04,month_05,month_06,month_07,month_08,month_09,month_10,month_11};

const char PROGMEM dow_00[]="SUN";
const char PROGMEM dow_01[]="MON";
const char PROGMEM dow_02[]="TUE";
const char PROGMEM dow_03[]="WED";
const char PROGMEM dow_04[]="THU";
const char PROGMEM dow_05[]="FRI";
const char PROGMEM dow_06[]="SAT";
const char PROGMEM * const dow_items[]= {dow_00,dow_01,dow_02,dow_03,dow_04,dow_05,dow_06};

const char PROGMEM RTC_formatting0[]="%4d/%02d/%02d\n%02d:%02d:%02d"; // 2015/03/03\n22:32:17

// The following are conversion factors of each vendor. The order of the factors is the same as the order of sensors in the sensor name list. Result=aX+b, where 0<=X<5V. You may directly use k1 for a and k0 for b out of Vernier's sensor manual.
float vernier_sensor_factor_a[]={   55.55,    0.0,  -4.9,    -21.0, 2.203,  0.5103,    1.0,  0.0,  65.7,  0.960,  9.0,  3.104};
float vernier_sensor_factor_b[]={   -17.7,    0.0,  12.25,   53.0,  0.0,    0.0,       0.0,  0.0,  0.0,   0.0,    0.0,  -7.403};

float pasco_sensor_factor_a[]={};
float pasco_sensor_factor_b[]={};

float liudr_sensor_factor_a[]={};
float liudr_sensor_factor_b[]={};

float * sensor_factor_a[]={vernier_sensor_factor_a,pasco_sensor_factor_a,liudr_sensor_factor_a};
float * sensor_factor_b[]={vernier_sensor_factor_b,pasco_sensor_factor_b,liudr_sensor_factor_b};

// The following are operating parameters
boolean pause=false;
boolean clear=true;
boolean serial_link=false;
long loop_delay=200;

RTC_DS1307 RTC; // define the Real Time Clock object
DateTime now;
SdFat sd;
SdFile logfile, cachefile, indexfile, debugfile; 
Stream* Debug=&Serial; 
char current_file_name[16]="20150302.CSV"; // Current file name may change at the end of a day to first make files smaller, second add date and time to the file, which has no file creation data with the current SD library.
byte SD_card_present=0;

unsigned int UI_delay=UI_delay_def;
unsigned int serial_delay=serial_delay_def;
/// Only runs once. Put initializations here.
void setup()
{
  setup_channels();
  Serial.begin(115200);                 ///< Start the serial port for data upload and serial keypad.
  Serial1.begin(9600);                 ///< Start the serial port for data upload and serial keypad.
  lcd.begin(lcd_columns, lcd_rows);     ///< Initialize the lcd object.
  init_phi_prompt(&lcd,keypads,function_keys, lcd_columns, lcd_rows, '~');  ///< Supply the liquid crystal object, input keypads, and function key names. Also supply the column and row of the lcd, and indicator as '>'. You can also use '\x7e', which is a right arrow.
  lcd.clear();                          ///< Clears the lcd.
  simple_select_list_scroll_bar(false);
  simple_select_list_auto_scroll(true);
  center_text_P(PROGMEM_msg0);          ///< Displays a short message in PROGMEM centered with the display size
  lcd.setCursor(0,1);
  center_text_P(PROGMEM_msg1);          ///< Displays a short message in PROGMEM centered with the display size
  while (!wait_on_escape(2000)){}       ///< This delay can be dismissed by any key press, unlike delay().
  init_sd_card();
  Wire.begin();
  if (!RTC.begin()) 
  {
    logfile.println(F("RTC failed"));
    //msg_serial(sd_msg_04);//lcdPanel.println("RTC failed");
    while(1){;}
  }
}

/// Runs repeatedly. Put code that you want run repeatedly here.
void loop()
{
  byte ret;
  strcpy_P(buffer,sensor_type_menu);
  ret=simple_select_list(buffer);  ///< Analog or digital sensors
  switch (ret)
  {
    case analog_sensors:
    handle_analog_sensors();
    break;
    
    case digital_sensors:
    handle_digital_sensors();
    break;
    
    case i2c_sensors:
    handle_i2c_sensors();
    break;
    
    case settings:
    handle_settings();
    break;
    
    default:
    break;
  }
}

/*
// The following are sensor codes
#define Vernier_sensors 0       ///< Vernier sensor types
#define Pasco_sensors 1         ///< Pasco sensor types
#define Liudr_sensors 2         ///< Liudr sensor types

#define Vernier_DT_DIN 0        ///< Vernier direct temperature DIN plug. 0.01V/F no offset
#define Vernier_DFS_DIN 1       ///< Vernier dual range force sensor DIN plug. 10N and 50N ranges.
#define Veriner_PS_DIN 2        ///< Vernier pressure sensor DIN plug.
#define Vernier_VS_DIN 3        ///< Vernier voltage sensor DIN plug. Direct connection to voltage source.
#define Vernier_FS_DIN 4        ///< Vernier single range force sensor DIN plug.

#define Pasco_PG_QS 0           ///< Pasco photogate 1/4 inch stereo plug on channel 4 by default.
#define Pasco_SR_QS 1           ///< Pasco sonic ranger 2 1/4 inch stereo plug on channel 4 by default.
*/

/// Formats in_num into floating point string and store in buffer for output
int format_fp(float in_num,int bd, int ad, char * buffer)
{
    int num=fabs(in_num*pow(10,ad))+0.5,num2=(num-num/int(pow(10,ad)+0.5)*int(pow(10,ad)+0.5));
    char formatting[]="% d.%0 d";
    formatting[1]=bd+'0';
    formatting[6]=ad+'0';
    
    sprintf(buffer, formatting, (num/int(pow(10,ad)+0.5)),num2);
    if ((in_num<0)&&(num2!=0))
    {
       int i=0;
       while(buffer[i]==' '){i++;}
       if (i==0) return -1; else buffer[i-1]='-'; // Increase digits before decimal. Current value is not enough.
    }
    return 0;
}
     
/// Formats in_num into floating point string and store in buffer for output. Takes formatting strings instead of digits before and after decimal.
int format_fp(float in_num, char * formatting, char * buffer)
{
    int ad,bd;
    bd=formatting[1]-'0'; // Number of digits before decimal
    ad=formatting[6]-'0'; // Number of digits after decimal
    int num=fabs(in_num*pow(10,ad))+0.5,num2=(num-num/int(pow(10,ad)+0.5)*int(pow(10,ad)+0.5));
    
    sprintf(buffer, formatting, (num/int(pow(10,ad)+0.5)),num2);
    if ((in_num<0)&&(num2!=0))
    {
       int i=0;
       while(buffer[i]==' '){i++;}
       if (i==0) return -1; else buffer[i-1]='-'; // Increase digits before decimal. Current value is not enough.
    }
    return 0;
}

/// This function extracts rounded decimals into a positive integer. Eg. extract_rounded_decimals(-1.5678,3) returns 568. (-1.5678,2) returns 57. The returned number can be used in sprintf to form the actual floating point number.
int extract_rounded_decimal(float in_num,double digits)
{
    int num=fabs(in_num*pow(10,digits))+0.5;
    return (num-num/int(pow(10,digits)+0.5)*int(pow(10,digits)+0.5));
}

/// This function extracts rounded integers into a signed integer. Eg. extract_rounded_integer(-1.5678,3) returns -1. (-1.9678,2) returns -2. The returned number can be used in sprintf to form the actual floating point number.
int extract_rounded_integer(float in_num,double digits)
{
    int num=fabs(in_num*pow(10,digits))+0.5;
    return (((in_num>1e-9)?1:-1)*(num/int(pow(10,digits)+0.5)));
}

void measure_freq() // Measure mode depends on sys_stat. Enter to clear display. Up to unpause. Down to pause. Esc to escape to menu.
{
  long tavg, t[4]; // Average time at which the speed is measured.
  float ang_spd;
  byte temp1,temp2, results=0; //results is 1 if some results are out and demands output.
  int a,b;
  char msg[10];
  int list_mode=2;
  temp2=pause?100:0;
  if(!pause) // If the program is in pause mode, don't do any update.
  {
    for (byte i=0;i<2;i++) // Sense all gates.
    {
      switch(gate_sensors[i]->run()) // Depending on what the gate reports, switch to just one case: ready_to_trigger - just triggered, then default. No other status was processed.
      {
        case ready_to_triggered:
        switch (gate_sensors[i]->get_counts()) // Depending on how many times the gate is triggered, switch to two cases: 1 count - first trigger for speed so just store time in t[i] for gate_i and continue. 2 counts - second trigger for speed so proceed to calculation speed and ready output. Acceleration mode (future) can use 3 counts.
        {
          case 1: //This is first data of velocity or acceleration
          t[i]=gate_sensors[i]->t_triggered;
          a=t[i]/1000000;
          b=t[i]/1000-a*1000;  
          a%=100;
          sprintf(msg,"%2d.%03d",a,b); // There's a problem with handeling long integers.
          gate_sensors[i]->clear_counts();
          results=1;
          if (!results) break;// Break out and ignore generating output if there isn't any to generate.
          lcd.setCursor(1+((i)>>1)*7,(i&B1));
          lcd.print(msg);
          break;
          
          case 2: //This is second data of velocity or acceleration
          gate_sensors[i]->clear_counts();
          ang_spd=two_pi/(float)spoke_number*1e6/((float)(gate_sensors[i]->t_triggered-t[i]));
          a=int(ang_spd);
          b=(ang_spd-a)*100;  
          sprintf(msg,"%3d.%02d",a,b); // There's a problem with handeling long integers.
          results=1;
          if (!results) break;// Break out and ignore generating output if there isn't any to generate.
          lcd.setCursor(1+((i)>>1)*7,(i&B1));
          lcd.print(msg);
          break;
          
          default:
          gate_sensors[i]->set_counts(1);
          t[i]=gate_sensors[i]->t_triggered;
          break;
        }  
        break;
        
        default:
        break;
      }
    }
  }
}

void setup_channels()
{
  for (byte i=A0;i<=A7;i++)              ///< Set all analog pins to input and disable internal pullup resistors.
  {
    pinMode(i,INPUT);
    digitalWrite(i,LOW);
  }
}  
