/// This handles all analog sensors.
void handle_analog_sensors()
{
  byte ret;
  select_analog_sensors();
  list_analog_sensors();
  wait_on_escape(2000);                 ///< This delay can be dismissed by any key press, unlike delay().
  lcd.clear();                          ///< Clears the lcd.
  loop_delay=UI_delay;  
  if (SD_card_present)
  {
    start_logging();
  }
  while (1)
  {
    if (serial_link)
    {
        upload_analog_sensors();
    }
    else if (!pause) display_analog_sensors();

    ret=wait_on_escape(loop_delay);                  ///< This delay can be dismissed by any key press, unlike delay().
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
  if (SD_card_present)
  {
    stop_logging();
  }
}

/// Asks the user to set up what channel has what sensors.  
void select_analog_sensors()
{
    for(byte i=0;i<total_channels;i++)
    {
        lcd.clear();                          ///< Clears the lcd.
        center_text_P(PROGMEM_msg2);          ///< Displays a short message in PROGMEM centered with the display size
        lcd.setCursor(0,1);
        lcd.print("Channel:");
        lcd.print(i);
        wait_on_escape(2000);                 ///< This delay can be dismissed by any key press, unlike delay().
        lcd.clear();                          ///< Clears the lcd.

        strcpy_P(buffer,vendors_menu);
        channel_sensor_vendors[i]=simple_select_list(buffer);     ///< Use a select list as a more elegant menu. You select it with up/down/enter keys so this works with as few as 2 buttons. Narrow items such as "JAN" "FEB" are automatically displayed in multiple columns. Return value is 0-15. Up to 16 items are allowed.
        if (channel_sensor_vendors[i]!=NO_VENDORs)
        {
          strcpy_P(buffer,sensor_names[channel_sensor_vendors[i]]);
          channel_sensor_types[i]=simple_select_list(buffer);   ///< Only ask for sensor type if the vendor is not NO_SENSORs.
        }
        else channel_sensor_types[i]=NO_VENDORs;  ///< If vendor is NO_VENDORs, also set sensor type to NO_SENSORs.
        if ((channel_sensor_vendors[i]==Vernier)&&(channel_sensor_types[i]==TMP_BTA)) digitalWrite(14+channel_pins[i],HIGH); ///< If the sensor is a thermistor, engage internal pullup resistor to form voltage divider.
        else digitalWrite(14+channel_pins[i],LOW); ///< Having internal pullup resistor will affect sensor readings largely, such as the TMP-DIN or DFS-DIN. Disengage pullup if these sensors are connected.
    }
}

///List what sensors are connected to which channel
void list_analog_sensors()
{
    //char buffer[21];
    for(byte i=0;i<total_channels;i++)
    {
        lcd.clear();                          ///< Clears the lcd.
        lcd.print("Chn:");
        lcd.print(i);
        lcd.print(' ');
        strcpy_P(buffer,vendors_menu);
        get_simple_list_item(buffer,buffer,channel_sensor_vendors[i]);   ///< Extract an item from a simple list.
        lcd.print(buffer);
        if (channel_sensor_vendors[i]!=NO_VENDORs)
        {
          lcd.setCursor(0,1);
          strcpy_P(buffer,sensor_names[channel_sensor_vendors[i]]);
          get_simple_list_item(buffer,buffer,channel_sensor_types[i]); ///< If vendor is not NO_SENSORs, display sensor type.
          lcd.print(buffer);
        }
        wait_on_escape(2000); ///< This delay can be dismissed by any key press, unlike delay().
    }
    //Serial.println(F("Listed sensors"));
}

/// Read all analog sensor outputs and process them for serial link upload.
void upload_analog_sensors()
{
    //char buffer[lcd_rows*lcd_columns+1];
    float readings[total_channels];
    for(byte i=0;i<total_channels;i++)
    {
        if (channel_sensor_vendors[i]!=NO_VENDORs) readings[i]=read_analog_channel(channel_pins[i],channel_sensor_vendors[i],channel_sensor_types[i]); ///< If there is an attached sensor, read it according to what sensor it is and get an appropriate output.
    }
    format_serial_output(buffer,readings,channel_sensor_vendors, channel_sensor_types); ///< Form an output string in buffer with appropriate output formats, such as digits and units.
    
    if (SD_card_present)
    {
      logfile.println(buffer); ///< Print the formatted output to serial port.
    }
    else
    {
      //Serial1.print(millis());
      //Serial1.print("\t");
      Serial1.println(buffer); ///< Print the formatted output to serial port.
    }
}
/// Display sensor outputs on LCD
void display_analog_sensors()
{
    //char buffer[lcd_rows*lcd_columns+1];
    float readings[total_channels];
    if (channel_sensor_vendors[3]==NO_VENDORs) // Regular analog sensors
    {
        for(byte i=0;i<total_channels;i++)
        {
            if (channel_sensor_vendors[i]!=NO_VENDORs) readings[i]=read_analog_channel(channel_pins[i],channel_sensor_vendors[i],channel_sensor_types[i]); ///< If there is an attached sensor, read it according to what sensor it is and get an appropriate output.
        }
        //Serial.println(F("Sensors read"));
        format_LCD_output(buffer,readings,channel_sensor_vendors, channel_sensor_types); ///< Form an output string in buffer with appropriate output formats, such as digits and units.
        //Serial.println(F("Output formatted"));
        simple_formatted_msg(buffer); ///< Display the formatted output on LCD.
        
    }
    else if (channel_sensor_vendors[3]==Pasco&&(channel_sensor_types[3]==Pasco_photogate)) // Photogate
    {
        gates gate0=gates(photogate_pin0);
        gate_sensors[0]=&gate0;
        while(1) {measure_freq();}
    }
}

/// Read an analog channel and return the converted reading.
float read_analog_channel(byte pin, byte ven, byte sen)
{
    if ((ven==Vernier)&&(sen==TMP_BTA)) //This is a 20k ohm thermistor. Need to calculate for resistance and use Steinhart_Hart equation.
    {
      analogRead(pin); //Read once just in case the reading is bad.
      int a=analogRead(pin);
      float rth=internal_pullup/(1024*effective_voltage_with_internal_pullup/regulated_voltage-a)*a; // The gate that enables internal pullup consumes voltage so effective voltage is what drops across the entire voltage divider.
      return (1/(TMP_BTA_K0+TMP_BTA_K1*log(rth)+TMP_BTA_K2*pow(log(rth),3))-273.15);
    }
    else if ((ven==Vernier)&&(sen==AUTO_ID)) //This is a routine to identify the auto_ID resistor.
    {
      analogRead(pin+1); //Read once just in case the reading is bad.
      int a=analogRead(pin+1); // pin+1 is the auto_id pin
      float rth=auto_ID_pullup/(1024-a)*a;
      rth=rth/1e3;
      if (rth>1000) rth=0; // This prevents huge readings from disconnected sensors.
      return (rth);
    }
    else
    {
      analogRead(pin); //Read once just in case the reading is bad.
      return (analogRead(pin)*regulated_voltage/1024*(sensor_factor_a[ven])[sen]+(sensor_factor_b[ven])[sen]); //This is a regular analog sensor. Just use aX+b.
    }
}

// millis tick is included.
void format_serial_output(char buffer[], float readings[], byte ven[], byte typ[])
{
    unsigned long temp=millis();
    sprintf(buffer,"%lu\t",temp);
    for (byte i=0;i<total_channels;i++)
    {
        if (ven[i]!=NO_VENDORs)
        {
            strcpy_P(formatting_buffer,(sensor_serial_output_formats[ven[i]])[typ[i]]);
            format_fp(readings[i], formatting_buffer, buffer+strlen(buffer));
            if (i!=total_channels-1) sprintf(buffer+strlen(buffer),"\t"); ///< Add tab
        }
    }
}  

void format_LCD_output(char buffer[], float readings[], byte ven[], byte typ[])
{
    byte j=0;
    for (byte i=0;i<total_channels;i++)
    {
        if (ven[i]!=NO_VENDORs)
        {
            strcpy_P(formatting_buffer,(sensor_LCD_output_formats[ven[i]])[typ[i]]);
            //Serial.println(formatting_buffer);
            format_fp(readings[i], formatting_buffer, buffer+j);
            //sprintf(buffer+j, (const char*) ((sensor_output_formats[ven[i]])[typ[i]]), int(readings[i]), int(0.5+abs((readings[i]-int(readings[i]))*pow(10,((sensor_output_formats[ven[i]])[typ[i]])[6]-'0'))));
        }
        else sprintf(buffer+j,"%s","        ");
        j=strlen(buffer);
    }
}  


