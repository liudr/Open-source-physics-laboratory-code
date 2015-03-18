/// This handles all I2C sensors.
void handle_i2c_sensors()
{
  byte ret;
  strcpy_P(buffer,i2c_sensors_menu);
  ret=simple_select_list(buffer);
  switch (ret)
  {
    case ADXL345_sensor:
    lcd.clear();
    acquire_ADXL345_sensor();
    break;
    
    case BMP083_sensor:
    lcd.clear();
    acquire_BMP083_sensor();
    break;
    
    case HMC5883L_sensor:
    acquire_HMC5883L_sensor();
    break;

/* This has been moved to settings
    case DS1307_sensor:
    acquire_DS1307_sensor();
    break;
*/
    default:
    break;
  }
}

void acquire_ADXL345_sensor()
{
  byte ret;
  ADXL345 accel;
  //int8_t xo,yo,zo;

  Wire.begin();
  setup_ADXL345_2G(&accel);
  loop_delay=UI_delay;  
  if (SD_card_present)
  {
    start_logging();
  }
  while(1)
  {
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

    if (!pause) measure_ADXL345_2G(&accel,serial_link);
  }
}

void acquire_BMP083_sensor()
{
  byte ret;
  BMP085 barometer;
  
  Wire.begin();
  setup_BMP085(&barometer);
  loop_delay=UI_delay;  
  if (SD_card_present)
  {
    start_logging();
  }
  while(1)
  {
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
    if (!pause) measure_BMP085(&barometer,serial_link);
  }
}

void setup_BMP085(BMP085 * barometer)
{
  barometer->initialize();
}

void measure_BMP085(BMP085 * barometer,boolean remote)
{
  float temperature;
  float pressure;
  float altitude;
  
  int32_t lastMicros;
  // request temperature
  barometer->setControl(BMP085_MODE_TEMPERATURE);
  
  // wait appropriate time for conversion (4.5ms delay)
  lastMicros = micros();
  while (micros() - lastMicros < barometer->getMeasureDelayMicroseconds());

  // read calibrated temperature value in degrees Celsius
  temperature = barometer->getTemperatureC();

  // request pressure (3x oversampling mode, high detail, 23.5ms delay)
  barometer->setControl(BMP085_MODE_PRESSURE_3);
  while (micros() - lastMicros < barometer->getMeasureDelayMicroseconds());

  // read calibrated pressure value in Pascals (Pa)
  pressure = barometer->getPressure();

  // calculate absolute altitude in meters based on known pressure
  // (may pass a second "sea level pressure" parameter here,
  // otherwise uses the standard value of 101325 Pa)
  altitude = barometer->getAltitude(pressure);

  dtostrf(pressure,6,0,float_buffer[0]);
  dtostrf(temperature,4,1,float_buffer[1]);
  //dtostrf(azg,5,2,float_buffer[2]);
  switch (remote)
  {
    case true:
    strcpy_P(formatting_buffer,BMP085_formatting0);
    sprintf(buffer,formatting_buffer,millis(),float_buffer[0],int(altitude), float_buffer[1]);
    if (SD_card_present)
    {
      logfile.println(buffer);
    }
    else
    {
      Serial.println(buffer);
    }
    break;
    
    case false:
    strcpy_P(formatting_buffer,BMP085_formatting1);
    sprintf(buffer,formatting_buffer, float_buffer[0],float_buffer[1]);
    lcd.setCursor(0,0);
    lcd.print(buffer);
    strcpy_P(formatting_buffer,BMP085_formatting2);
    sprintf(buffer,formatting_buffer, int(altitude));
    lcd.setCursor(0,1);
    lcd.print(buffer);
    break;
  }  
}

void acquire_HMC5883L_sensor()
{
  byte ret;
  HMC5883L mag;
  
  Wire.begin();
  setup_HMC5883L(&mag);
  loop_delay=UI_delay;  
  if (SD_card_present)
  {
    start_logging();
  }
  while(1)
  {
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
    if (!pause) measure_HMC5883L(&mag,serial_link);
  }
}

void setup_HMC5883L(HMC5883L * mag)
{
  mag->initialize();
}

void measure_HMC5883L(HMC5883L * mag,boolean remote)
{
  int16_t x, y, z;
  float mx,my,mz,heading;
  mag->getHeading(&x, &y, &z);
  mx=x*HMC5883L_1090_factor;
  my=y*HMC5883L_1090_factor;
  mz=z*HMC5883L_1090_factor;
// To calculate heading in degrees. 0 degree indicates North
  heading = atan2(y, x);
  if(heading < 0) heading += 2 * M_PI;
  heading=heading * 180/M_PI;

  switch (remote)
  {
    case true:
    strcpy_P(formatting_buffer,HMC5883L_formatting0);
    sprintf(buffer,formatting_buffer,millis(), int(mx),int(my),int(mz),int(heading));
    if (SD_card_present)
    {
      logfile.println(buffer);
    }
    else
    {
      Serial.println(buffer);
    }
    break;
    
    case false:
    strcpy_P(formatting_buffer,HMC5883L_formatting1);
    sprintf(buffer,formatting_buffer, int(mx),int(my));
    lcd.setCursor(0,0);
    lcd.print(buffer);
    strcpy_P(formatting_buffer,HMC5883L_formatting2);
    sprintf(buffer,formatting_buffer, int(mz),int(heading));
    lcd.setCursor(0,1);
    lcd.print(buffer);
    break;
  }  
}

void adjust_time()
{
  
}

void print_RTC()
{
  PGM_RAM(RTC_formatting0,formatting_buffer);
  sprintf(buffer, formatting_buffer,now.year(), now.month(), now.day(),now.hour(), now.minute(), now.second());
  simple_formatted_msg(buffer);
}

void acquire_DS1307_sensor()
{
  byte ret;
  while(1)
  {
    if (serial_link)
    {
        Set_clock();//adjust_time();
        serial_link=false;
        loop_delay=UI_delay;
        lcd.clear();
        print_RTC();
    }
    else if (!pause)
    {
      now=RTC.now();
      print_RTC();
    }
    ret=wait_on_escape(999);
    if(LCD_interaction(ret)) // User pressed escape.
    {
      setup_channels();
      return;
    }
  }  
}

void setup_ADXL345_2G(ADXL345 * accel)
{
  //Serial.println("Initializing I2C devices...");
  accel->initialize();
  //Serial.print("Range is:");
  //Serial.println((int)(accel->getRange()));
  accel->setRange(ADXL345_RANGE_2G);
  //accel->getOffset(&xo,&yo,&zo);
  //sprintf(buffer, "Offsets: %d,%d,%d\n",(int)xo,(int)yo,(int)zo);
  accel->setOffset((int8_t)(256-2),(int8_t)(0),(int8_t)(8));
  //Serial.print(buffer);
  // verify connection
  //Serial.println("Testing device connections...");
  //Serial.println(accel->testConnection() ? "ADXL345 connection successful" : "ADXL345 connection failed");
}

void measure_ADXL345_2G(ADXL345 * accel, boolean remote)
{
  char buffer[64];
  char float_buffer[4][8];
  int16_t x, y, z;
  float axg,ayg,azg;
  // read raw accel measurements from device
  accel->getAcceleration(&x, &y, &z);
  axg=x*ADXL_2G_factor;
  ayg=y*ADXL_2G_factor;
  azg=z*ADXL_2G_factor;
  // display tab-separated accel x/y/z values
  dtostrf(axg,5,2,float_buffer[0]);
  dtostrf(ayg,5,2,float_buffer[1]);
  dtostrf(azg,5,2,float_buffer[2]);
  dtostrf(sqrt(pow(axg,2)+pow(ayg,2)+pow(azg,2)),5,2,float_buffer[3]);
  switch (remote)
  {
    case true:
    strcpy_P(formatting_buffer,ADXL_formatting0);
    sprintf(buffer,formatting_buffer,millis(),float_buffer[0],float_buffer[1],float_buffer[2],float_buffer[3]);
    if (SD_card_present)
    {
      logfile.println(buffer);
    }
    else
    {
      Serial.println(buffer);
    }
    break;
    
    case false:
    strcpy_P(formatting_buffer,ADXL_formatting1);
    sprintf(buffer,formatting_buffer, float_buffer[0],float_buffer[1]);
    lcd.setCursor(0,0);
    lcd.print(buffer);
    strcpy_P(formatting_buffer,ADXL_formatting2);
    sprintf(buffer,formatting_buffer, float_buffer[2],float_buffer[3]);
    lcd.setCursor(0,1);
    lcd.print(buffer);
    break;
  }  
}
