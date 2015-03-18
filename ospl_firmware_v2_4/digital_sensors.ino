/// This handles all digital sensors.
void handle_digital_sensors()
{
  byte ret;
  strcpy_P(buffer,digital_sensors_menu);
  ret=simple_select_list(buffer);
  switch (ret)
  {
    case sonic_ranger:
    lcd.clear();
    acquire_sonic_ranger();
    break;
    
    case OSPL_sonic_ranger:
    lcd.clear();
    acquire_ospl_sonic_ranger();
    break;
    
    case photogate:
    acquire_photogate();
    break;
  }
}

void acquire_ospl_sonic_ranger()
{
  byte ret;
  unsigned long micros_now,micros_delay;
  unsigned int distance_mm;

  Serial.println(F("Starting..."));
  pinMode(sonic_ranger_init,OUTPUT);
  pinMode(sonic_ranger_echo,INPUT);
  if (SD_card_present)
  {
    start_logging();
  }
  while(1)
  {
    micros_now=micros();
    digitalWrite(sonic_ranger_init,LOW);
    delayMicroseconds(2);
    digitalWrite(sonic_ranger_init,HIGH);
    delayMicroseconds(15);
    digitalWrite(sonic_ranger_init,LOW);
    delayMicroseconds(20);
    micros_delay=pulseIn(sonic_ranger_echo,HIGH,35000L);
    distance_mm=micros_delay*343/2000; // This is round trip time so 1/2/1000=1/2000

    ret=wait_on_escape(((micros()-micros_now)/1000)>50?0:(50-(micros()-micros_now)/1000));
    
    if (serial_link)
    {
        upload_sonic_ranger(distance_mm);
    }
    else if (!pause)
    {
      //Serial.println(distance_mm);
      update_sonic_ranger_display(distance_mm);
    }

    if(LCD_interaction(ret)) // User pressed escape.
    {
      if (SD_card_present)
      {
        stop_logging();
      }
      setup_channels();
      return;
    }  
  }
}

void acquire_sonic_ranger()
{
  byte ret;
  unsigned long micros_now;
  unsigned int distance_mm;

  while(1)
  {
    micros_now=micros();
    digitalWrite(sonic_ranger_init,HIGH);
    delayMicroseconds(200); // Didn't work. Have to turn init to low after receiving data.
//    digitalWrite(sonic_ranger_init,LOW);
    while(!digitalRead(sonic_ranger_echo)){;}
    distance_mm=(micros()-micros_now)*343/2000; // This is round trip time so 1/2/1000=1/2000
    delayMicroseconds(200);
    digitalWrite(sonic_ranger_init,LOW);
    update_sonic_ranger_display(distance_mm);
    ret=wait_on_escape(((micros()-micros_now)/1000)>50?0:(50-(micros()-micros_now)/1000));
//    ret=wait_on_escape(50-(micros()-micros_now)/1000);
    
    switch (ret)
    {
      case 0:
      break;
      
      case phi_prompt_up: // Pause
      while(!wait_on_escape(50)){;}
      break;
      
      case phi_prompt_down: //Serial print
      break;
      case phi_prompt_enter:
      return;
      break;
    }
  }
}

void update_sonic_ranger_display(unsigned int distance_mm)
{
  sprintf(buffer,"%4dmm          ",distance_mm);
  lcd.setCursor(0,0);
  lcd.print(buffer);
  format_fp(distance_mm/25.4, 3, 1, buffer);
  lcd.setCursor(0,1);
  lcd.print(buffer);
  lcd.print("in");
}

void upload_sonic_ranger(unsigned int distance_mm)
{
  sprintf(buffer,"%lu\t%4d\tmm",millis(), distance_mm);  
  if (SD_card_present)
  {
    logfile.println(buffer); ///< Print the formatted output to serial port.
  }
  else
  {
    Serial1.println(buffer); ///< Print the formatted output to serial port.
  }
}

void acquire_photogate()
{
  byte ret;
  strcpy_P(buffer,photogate_modes_menu);
  ret=simple_select_list(buffer);
  switch (ret)
  {
    case photogate_speed_mode: ///< Both photogates are measuring speed independently
    break;
    
    case photogate_acceleration_mode: ///< Both photogates are measuring acceleration independently
    break;
    
    case photogate_acceleration_2_gate_mode: ///< Both photogates are measuring speed independently and report one acceleration as change of speed over change in time
    break;
    
    case photogate_delay_mode: ///< Both photogates are measuring time independently and report time between triggering the two gates
    break;
    
    case photogate_angular_speed_mode: ///< Both photogates are measuring angular speed independently
    break;
    
    case photogate_angular_acceleration_mode: ///< Both photogates are measuring angular acceleration independently
    break;
  }
}


