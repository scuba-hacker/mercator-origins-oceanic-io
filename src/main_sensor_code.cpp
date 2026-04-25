#ifdef BUILD_INCLUDE_MAIN_SENSOR_CODE

void initI2C()
{
  const int gpioSDA = 6;
  const int gpioSCL = 7;

  DEV_I2C->setPins(gpioSDA, gpioSCL);
}

void initHumiditySensor()
{
  ahtStatus = ahtSensor.begin(DEV_I2C);
}

#ifdef COMPILE_TOF

void initToFSensor()
{

  VL53L4CX_Error error = VL53L4CX_ERROR_NONE;

  if (I2CDeviceAvailable(VL53L4CX_DEFAULT_DEVICE_ADDRESS >> 1, &DEV_I2C)) 
  {
    DEV_I2C->begin();
    sensor_vl53l4cx_sat.setI2cDevice(DEV_I2C);
    sensor_vl53l4cx_sat.setXShutPin(XSHUT_PIN);

    // Configure VL53L4CX satellite component.
    sensor_vl53l4cx_sat.begin();
    
    // Switch off VL53L4CX satellite component.
    sensor_vl53l4cx_sat.VL53L4CX_Off();

    //Initialize VL53L4CX satellite component.
    error = sensor_vl53l4cx_sat.InitSensor(VL53L4CX_DEFAULT_DEVICE_ADDRESS);

    if (error != VL53L4CX_ERROR_NONE)
    {
      USB_SERIAL_PRINT("Error Initializing Sensor: ");
      USB_SERIAL_PRINTLN(error);
    }
    else
    {
      error = sensor_vl53l4cx_sat.VL53L4CX_SetDistanceMode(VL53L4CX_DISTANCEMODE_LONG);

      if (error != VL53L4CX_ERROR_NONE)
      {
        USB_SERIAL_PRINT("Error Initializing Distance Mode: ");
        USB_SERIAL_PRINTLN(error);
      }
      else
      {
        // Start Measurements
        sensor_vl53l4cx_sat.VL53L4CX_StartMeasurement();

        tofStatus = true;
      }
    }
  }
}

/* VL53L4CX_RangeStatusCode --------------------------------------------------*/
String VL53L4CX_RangeStatusCode(uint8_t status)
{
  switch (status)
  {
    case VL53L4CX_RANGESTATUS_RANGE_VALID:
      return "VL53L4CX_RANGESTATUS_RANGE_VALID";
    case VL53L4CX_RANGESTATUS_SIGMA_FAIL:
      return "VL53L4CX_RANGESTATUS_SIGMA_FAIL";
    case VL53L4CX_RANGESTATUS_RANGE_VALID_MIN_RANGE_CLIPPED:
      return "VL53L4CX_RANGESTATUS_RANGE_VALID_MIN_RANGE_CLIPPED";
    case VL53L4CX_RANGESTATUS_OUTOFBOUNDS_FAIL:
      return "VL53L4CX_RANGESTATUS_OUTOFBOUNDS_FAIL";
    case VL53L4CX_RANGESTATUS_HARDWARE_FAIL:
      return "VL53L4CX_RANGESTATUS_HARDWARE_FAIL";
    case VL53L4CX_RANGESTATUS_RANGE_VALID_NO_WRAP_CHECK_FAIL:
      return "VL53L4CX_RANGESTATUS_RANGE_VALID_NO_WRAP_CHECK_FAIL";
    case VL53L4CX_RANGESTATUS_WRAP_TARGET_FAIL:
      return "VL53L4CX_RANGESTATUS_WRAP_TARGET_FAIL";
    case VL53L4CX_RANGESTATUS_PROCESSING_FAIL:
      return "VL53L4CX_RANGESTATUS_PROCESSING_FAIL";
    case VL53L4CX_RANGESTATUS_XTALK_SIGNAL_FAIL:
      return "VL53L4CX_RANGESTATUS_XTALK_SIGNAL_FAIL";
    case VL53L4CX_RANGESTATUS_SYNCRONISATION_INT:
      return "VL53L4CX_RANGESTATUS_SYNCRONISATION_INT";
    case VL53L4CX_RANGESTATUS_RANGE_VALID_MERGED_PULSE:
      return "VL53L4CX_RANGESTATUS_RANGE_VALID_MERGED_PULSE";
    case VL53L4CX_RANGESTATUS_TARGET_PRESENT_LACK_OF_SIGNAL:
      return "VL53L4CX_RANGESTATUS_TARGET_PRESENT_LACK_OF_SIGNAL";
    case VL53L4CX_RANGESTATUS_MIN_RANGE_FAIL:
      return "VL53L4CX_RANGESTATUS_MIN_RANGE_FAIL";
    case VL53L4CX_RANGESTATUS_RANGE_INVALID:
      return "VL53L4CX_RANGESTATUS_RANGE_INVALID";
    case VL53L4CX_RANGESTATUS_NONE:
      return "VL53L4CX_RANGESTATUS_NONE";
    default:
      return ("UNKNOWN STATUS: " + String(status));
  }
}

void acquireLidarDistanceReading()
{
  if (!tofStatus)
  {
    mapScreen->setLidarDistance(-1.0);
    return;
  }

  float maxDistance = 0.0;

  VL53L4CX_Error status = sensor_vl53l4cx_sat.VL53L4CX_GetMeasurementDataReady(&newLidarDataReady);

  USB_SERIAL_PRINTF("GetMeasurementDataReady Status: %s\n",VL53L4CX_RangeStatusCode(status).c_str());

  if (!newLidarDataReady)
  {
    USB_SERIAL_PRINTF("Data Not Ready, returning\n");
    return;
  }
  else
  {
    USB_SERIAL_PRINTF("Data Ready, extract objects\n");
  }

  VL53L4CX_MultiRangingData_t MultiRangingData;
  VL53L4CX_MultiRangingData_t *pMultiRangingData = &MultiRangingData;

  if (!status)
  {
    status = sensor_vl53l4cx_sat.VL53L4CX_GetMultiRangingData(pMultiRangingData);

    USB_SERIAL_PRINTF("GetMultiRangingData Status: %s\n",VL53L4CX_RangeStatusCode(status).c_str());

    int no_of_object_found = pMultiRangingData->NumberOfObjectsFound;

    USB_SERIAL_PRINTF("VL53L4CX Satellite: Count=%d, #Objs=%1d \n", pMultiRangingData->StreamCount, no_of_object_found);
      
    maxDistance = -0.5;
    for (int j = 0; j < no_of_object_found; j++) 
    {
      if (pMultiRangingData->RangeData[j].RangeMilliMeter > maxDistance)
        maxDistance = ((float)pMultiRangingData->RangeData[j].RangeMilliMeter)/1000.0;

      if (j != 0) 
        USB_SERIAL_PRINT("\r\n                               ");

      USB_SERIAL_PRINT("status=");
      USB_SERIAL_PRINT(pMultiRangingData->RangeData[j].RangeStatus);
      USB_SERIAL_PRINT(", D=");
      USB_SERIAL_PRINT(pMultiRangingData->RangeData[j].RangeMilliMeter);
      USB_SERIAL_PRINT("mm");
      USB_SERIAL_PRINT(", Signal=");
      USB_SERIAL_PRINT((float)pMultiRangingData->RangeData[j].SignalRateRtnMegaCps / 65536.0);
      USB_SERIAL_PRINT(" Mcps, Ambient=");
      USB_SERIAL_PRINT((float)pMultiRangingData->RangeData[j].AmbientRateRtnMegaCps / 65536.0);
      USB_SERIAL_PRINT(" Mcps, ");
      USB_SERIAL_PRINT(VL53L4CX_RangeStatusCode(pMultiRangingData->RangeData[j].RangeStatus));
    }
  }

  if (status == 0)
  {
    status = sensor_vl53l4cx_sat.VL53L4CX_ClearInterruptAndStartMeasurement();
    USB_SERIAL_PRINTF("VL53L4CX_ClearInterruptAndStartMeasurement Status: %s\n",VL53L4CX_RangeStatusCode(status).c_str());
  }

  mapScreen->setLidarDistance(maxDistance);

  newLidarDataReady = 0;
}

#endif

void acquireHumidityAndTemperatureReadings()
{
    float relative_humidity = -1.0;
    float temperature = -1.0;

    if (ahtStatus)
    {
      sensors_event_t humidity, temp;
      ahtSensor.getEvent(&humidity, &temp);
      relative_humidity = humidity.relative_humidity;
      temperature = temp.temperature;
    }

    mapScreen->setHumidityTemp(relative_humidity,temperature);
}

/* I2CDeviceAvailable --------------------------------------------------------*/
bool I2CDeviceAvailable(uint8_t address, TwoWire **wire)
{
  byte error = 1;
  bool available = false;

  // Check if device is available at the expected address
  Wire.begin();
  Wire.beginTransmission(address);
  error = Wire.endTransmission();

  if (error == 0) {
    *wire = &Wire;
    available = true;

    Serial.print("  I2c Device Found at Address 0x");
    Serial.print(address, HEX); Serial.println(" on Wire");
  }
  Wire.end();

  return available;
}

void readSensors()
{
  if (millis() > nextReadHumiditySensorTime)
  {
    nextReadHumiditySensorTime = millis() + timeBetweenHumidityReads;
    acquireHumidityAndTemperatureReadings();
  }
}

#endif
