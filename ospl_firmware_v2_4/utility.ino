// Returns true if user decides to exit.
boolean LCD_interaction(byte ret)
{
  switch (ret) /// The following is the switch-case for key press processing.
  {
    case NO_KEY: /// If no key is pressed, render the list only when needed, such as when the highlighted item is long and needs scrolling.
    break;
    
    case phi_prompt_up: /// Up is pressed. Pause/unpause the display to read numbers.
    if (!pause)
    {
      if (serial_link)
      {
        serial_link=false;
        loop_delay=UI_delay;
      }
      else
      {
        pause=true;
      }
    }

    //pause=!pause;
    break;
    
    case phi_prompt_down: /// Down is pressed. Toggle to serial output.

    if (!serial_link)
    {
      if (pause)
      {
        pause=false;
        loop_delay=UI_delay;
      }
      else
      {
        serial_link=true;
        lcd.clear();
        if (SD_card_present)
        {
          center_text_P(sd_card_logging_msg);
        }
        else
        {
          center_text_P(serial_uploading_msg);
        }
        loop_delay=serial_delay;
      }
    }
    break;
    
    case phi_prompt_enter: /// Enter is pressed. The function returns 1.
    pause=false;
    serial_link=false;
    return true;
    break;
  }
  return false;
}  

/**
 * \details This generates a name of the file with the current date.
 * \param buf. It needs to be 12 bytes long.
 */
char * generate_file_name(char * buf, int y, int m, int d)
{
  sprintf(buf,"%4d%02d%02d.TXT",y,m,d);
  return buf;
}


/**
 * \details This function starts the SD card. It hangs the program and prints error message if sd card can't start.
 * Use this in setup and after un-pause.
 */
void init_sd_card()
{
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(SD_card_chip_select, OUTPUT);
  delay(50);
  
  Debug->print(millis());
  Debug->println(F("Initializing SD card"));
  // see if the card is present and can be initialized:
  if (!sd.begin(SD_card_chip_select, SPI_HALF_SPEED))
  {
    SD_card_present=0;
    PGM_RAM(sd_msg_01,buffer);
    simple_formatted_msg(buffer);
    wait_on_escape(2000);
  }
  else
  {
    SD_card_present=1;
    SdFile::dateTimeCallback(file_date_time); // Attach callback function to provide date time to files.
    Debug->print(millis());
    Debug->println(F("SD card initialized"));
    PGM_RAM(sd_msg_02,buffer);
    simple_formatted_msg(buffer);
    wait_on_escape(2000);
  }
}

/**
 * \details This function writes a row of headers to line one when the file is created.
 * Use this before writing to log file. It opens the correct file. After logging, use stop_logging() to close the file.
 */

byte start_logging()
{
  now=RTC.now();
//  generate_file_name(current_file_name,now.day(), now.hour(),now.minute()/30); // For testing purpose to quickly create more names.
  generate_file_name(current_file_name,now.year(), now.month(), now.day());
/*
  Debug->println(now.year());
  Debug->println(now.month());
  Debug->println(now.day());
  Debug->println(now.hour());
  Debug->println(now.minute());
*/
  Debug->println(current_file_name);
  if (sd.exists(current_file_name))
  {
    if (!logfile.open(current_file_name, O_WRITE | O_CREAT | O_AT_END))
    {
      //error(err_msg_02); //Serial.println("Can't create file."); 
      while(1);
    }
  }
  else
  {
    if (!logfile.open(current_file_name, O_WRITE | O_CREAT | O_AT_END))
    {
      //error(err_msg_02); //Serial.println("Can't create file."); 
      while(1);
    }
  }

  logfile.print(now.year());
  logfile.write('/');
  logfile.print(now.month());
  logfile.write('/');
  logfile.print(now.day());
  logfile.write(' ');
  logfile.print(now.hour());
  logfile.write(':');
  logfile.print(now.minute());
  logfile.write(':');
  logfile.println(now.second());
  //logfile.print(buffer);

  //logfile.seekEnd();

//  rawfile = SD.open("S1_Raw.TXT", FILE_WRITE);
//  delay(500);
//  debugfile = SD.open("S1_Debug.TXT", FILE_WRITE);
}

byte stop_logging()
{
  logfile.sync();
  logfile.close();
  delay(500);
}

//This function returns the date and time for SD card file access and modify time. One needs to call in setup() to register this callback function: SdFile::dateTimeCallback(file_date_time);
void file_date_time(uint16_t* date, uint16_t* time) 
{
  DateTime now = RTC.now();
  *date=FAT_DATE(now.year(),now.month(),now.day());
  *time=FAT_TIME(now.hour(),now.minute(),now.second());
}

void error(const char* PROGMEM str)
{
  msg_serial(str);//Serial.println(str);
  while(1);
}

/**
 * \details This is a quick and easy way to display a string in the PROGMEM to the lcdPanel port/Serial LCD.
 * \param msg_line This is the name of the char string stored in PROGMEM.
 */
void msg_serial(const char* PROGMEM msg_line)
{
  byte i=0,ch;
  lcd.clear();
  while (ch=pgm_read_byte_near(msg_line+i))
  {
    i++;
    Debug->write(ch);
    lcd.write(ch);
  }
}

/**
 * \details This copys a string in the PROGMEM to the RAM buffer.
 * \param PGM_msg This is the name of the char string stored in PROGMEM.
 * \param RAM_buffer This is the name of the char string buffer in RAM.
 */

void PGM_RAM(const char* PROGMEM PGM_msg, char* RAM_buffer)
{
  byte i=0,ch;
  while (ch=pgm_read_byte_near(PGM_msg+i))
  {
    RAM_buffer[i]=ch;
    i++;
  }
  RAM_buffer[i]='\0';
}


void Set_clock() //Set the clock
{
  int user_input;
  phi_prompt_struct myIntegerInput, myListInput; // This structure stores the main menu.

  //render_RTC(0);
  now=RTC.now();

  user_input=now.year(); // Current value
  myIntegerInput.ptr.i_buffer=&user_input; // Pass the address of the buffer
  myIntegerInput.low.i=2000; // Lower limit
  myIntegerInput.high.i=2099; // Upper limit
  myIntegerInput.step.i=1; // Step size
  myIntegerInput.col=6; // Display prompt at column 7
  myIntegerInput.row=1; // Display prompt at row 1
  myIntegerInput.width=4; // The number occupies 2 characters space
  myIntegerInput.option=0; // Option 0, space pad right, option 1, zero pad left, option 2, space pad left.
  lcd.clear();
  center_text("Year"); // Prompt user for input
  if (input_integer(&myIntegerInput)!=-1) now.set_year(user_input); // If the user didn't press escape (return -1) then update the ultimate storage with the value in the buffer.
  else return;

  myListInput.ptr.list=(char**)&month_items; // Assign the list to the pointer
  myListInput.low.i=now.month()-1; // Default item highlighted on the list
  myListInput.high.i=11; // Last item of the list is size of the list - 1.
  myListInput.width=3; // Length in characters of the longest list item.
  myListInput.col=0; // Display prompt at column 0
  myListInput.row=1; // Display prompt at row 1
  myListInput.option=1; 
  myListInput.step.c_arr[0]=lcd_rows-1; // rows to auto fit entire screen
  myListInput.step.c_arr[1]=lcd_columns/4; // multi col list
  lcd.clear();
  center_text("Month"); // Prompt user for input
  if (select_list(&myListInput)!=-1)  now.set_month(myListInput.low.i+1); // select_list stores user choice in myListInput.low. If the user didn't press escape (return -1) then update the user choice with the value in myListInput.low.
  else return;
  
  user_input=now.day(); // Current value
  myIntegerInput.low.i=1; // Lower limit
  myIntegerInput.high.i=31; // Upper limit
  lcd.clear();
  center_text("Date"); // Prompt user for input
  if (input_integer(&myIntegerInput)!=-1) now.set_day(user_input); // If the user didn't press escape (return -1) then update the ultimate storage with the value in the buffer.
  else return;
  
  user_input=now.hour(); // Current value
  myIntegerInput.ptr.i_buffer=&user_input; // Pass the address of the buffer
  myIntegerInput.low.i=0; // Lower limit
  myIntegerInput.high.i=23; // Upper limit
  myIntegerInput.step.i=1; // Step size
  myIntegerInput.col=7; // Display prompt at column 7
  myIntegerInput.row=1; // Display prompt at row 1
  myIntegerInput.width=2; // The number occupies 2 characters space
  myIntegerInput.option=1; // Option 0, space pad right, option 1, zero pad left, option 2, space pad left.
  lcd.clear();
  center_text("Hour"); // Prompt user for input
  if (input_integer(&myIntegerInput)!=-1) now.set_hour(user_input); // If the user didn't press escape (return -1) then update the ultimate storage with the value in the buffer.
  else return;

  user_input=now.minute(); // Current value
  myIntegerInput.low.i=0; // Lower limit
  myIntegerInput.high.i=59; // Upper limit
  lcd.clear();
  center_text("Minute"); // Prompt user for input
  if (input_integer(&myIntegerInput)!=-1) now.set_minute(user_input); // If the user didn't press escape (return -1) then update the ultimate storage with the value in the buffer.
  else return;
  
  user_input=now.second(); // Current value
  lcd.clear();
  center_text("Second"); // Prompt user for input
  if (input_integer(&myIntegerInput)!=-1) now.set_second(user_input); // If the user didn't press escape (return -1) then update the ultimate storage with the value in the buffer.
  else return;
  
  /*
  myListInput.ptr.list=(char**)&dow_items; // Assign the list to the pointer
  myListInput.low.i=now.dayOfWeek()-1; // Default item highlighted on the list
  myListInput.high.i=6; // Last item of the list is size of the list - 1.
  myListInput.width=3; // Length in characters of the longest list item.
  myListInput.col=0; // Display prompt at column 0
  myListInput.row=1; // Display prompt at row 1
  myListInput.option=1; 
  myListInput.step.c_arr[0]=lcd_rows-1; // rows to auto fit entire screen
  myListInput.step.c_arr[1]=lcd_columns/4; // multi col list
  lcd.clear();
  center_text("Day of the week"); // Prompt user for input
  if (select_list(&myListInput)!=-1) now.set_dayOfWeek(myListInput.low.i+1); // select_list stores user choice in myListInput.low. If the user didn't press escape (return -1) then update the user choice with the value in myListInput.low.
  else return;
  */
  RTC.adjust(now);
}

//This is not being used at the moment
void render_RTC(int temp)
{
  char msg[17];
  int user_input;
  phi_prompt_struct myListInput; // This structure stores the main menu.

  now=RTC.now();

  switch(temp)
  {
    case 0:
    myListInput.ptr.list=(char**)&month_items; // Assign the list to the pointer
    myListInput.low.i=now.month()-1; // Default item highlighted on the list
    myListInput.high.i=11; // Last item of the list is size of the list - 1.
    myListInput.width=3; // Length in characters of the longest list item.
    myListInput.col=lcd_columns/2-8; // Display prompt at column 0
    myListInput.row=lcd_rows/2-1; // Display prompt at row 1
    myListInput.option=0; 
    myListInput.step.c_arr[0]=1; // rows to auto fit entire screen
    myListInput.step.c_arr[1]=1; // one col list
    render_list(&myListInput);
    sprintf(msg,"/%02d/%4d/",now.day(),now.year());
    lcd.print(msg);
    
    myListInput.ptr.list=(char**)&dow_items; // Assign the list to the pointer
    myListInput.low.i=now.dayOfWeek()-1; // Default item highlighted on the list
    myListInput.high.i=6; // Last item of the list is size of the list - 1.
    myListInput.col=lcd_columns/2-8+12; // Display prompt at column 0
    render_list(&myListInput);

    lcd.setCursor(lcd_columns/2-4,lcd_rows/2);
    sprintf(msg,"%2d:%02d:%02d",now.hour(),now.minute(),now.second());
    lcd.print(msg);
    break;
    
    case 1:
    /*
    sprintf(msg,"%02d%02d",now.minute(),now.second());
    render_big_msg(msg,lcd_columns/2-8,0);
    if ((lcd_rows==4)&&(lcd_columns==20))
    {
      strcpy_P(msg,(char*)pgm_read_word(dow_items+now.hour()-1));
      sprintf(msg+3,"%02d",now.month());
      render_big_msg(msg,0,2);
    }
    if (now.second()%2)
    {
      lcd.setCursor(lcd_columns/2-1,0);
      lcd.write('.');
      lcd.setCursor(lcd_columns/2-1,1);
      lcd.write('.');
    }
    else
    {
      lcd.setCursor(lcd_columns/2-1,0);
      lcd.write(' ');
      lcd.setCursor(lcd_columns/2-1,1);
      lcd.write(' ');
    }
    */
    break;
    default:
    break;
  }
}


