void handle_settings()
{
  byte ret;
  strcpy_P(buffer,settings_menu);
  ret=simple_select_list(buffer);
  switch (ret)
  {
    case UI_delay_setting:
    lcd.clear();
    UI_delay_setting_menu();
    break;
    
    case Serial_delay_setting:
    lcd.clear();
    Serial_delay_setting_menu();
    break;
    
    case RTC_setting:
    lcd.clear();
    acquire_DS1307_sensor();
    break;
    
    case credits:
    lcd.clear();
    show_credits();
    break;
    
    default:
    break;
  }
}

void UI_delay_setting_menu()
{
  int user_input;
  phi_prompt_struct myIntegerInput; // This structure stores the main menu.

  user_input=UI_delay; // Current value
  myIntegerInput.ptr.i_buffer=&user_input; // Pass the address of the buffer
  myIntegerInput.low.i=100; // Lower limit
  myIntegerInput.high.i=20000; // Upper limit
  myIntegerInput.step.i=100; // Step size
  myIntegerInput.col=4; // Display prompt at column 4
  myIntegerInput.row=1; // Display prompt at row 1
  myIntegerInput.width=5; // The number occupies 5 characters space
  myIntegerInput.option=1; // Option 0, space pad right, option 1, zero pad left, option 2, space pad left.
  lcd.clear();
  center_text("UI_delay"); // Prompt user for input
  if (input_integer(&myIntegerInput)!=-1) UI_delay=user_input; // If the user didn't press escape (return -1) then update the ultimate storage with the value in the buffer.
  else return;
}

void Serial_delay_setting_menu()
{
  int user_input;
  phi_prompt_struct myIntegerInput; // This structure stores the main menu.

  user_input=serial_delay; // Current value
  myIntegerInput.ptr.i_buffer=&user_input; // Pass the address of the buffer
  myIntegerInput.low.i=10; // Lower limit
  myIntegerInput.high.i=30000; // Upper limit
  myIntegerInput.step.i=10; // Step size
  myIntegerInput.col=4; // Display prompt at column 4
  myIntegerInput.row=1; // Display prompt at row 1
  myIntegerInput.width=5; // The number occupies 5 characters space
  myIntegerInput.option=1; // Option 0, space pad right, option 1, zero pad left, option 2, space pad left.
  lcd.clear();
  if (SD_card_present)
  {
    center_text("SD card delay"); // Prompt user for input
  }
  else
  {
    center_text("Serial delay"); // Prompt user for input
  }
  if (input_integer(&myIntegerInput)!=-1) serial_delay=user_input; // If the user didn't press escape (return -1) then update the ultimate storage with the value in the buffer.
  else return;
}

void show_credits()
{
  PGM_RAM(credits_text,buffer);
  simple_text_area(buffer);
}

