#ifdef BUILD_INCLUDE_ESP_NOW_MESSAGE_CODE

bool isMakoMessage(char m)
{
  return (m == 'X' || m == 'c');
}

void processReceivedESPNowMessages()
{  
  // do not process queued messages if ota is active, mapscreen is deleted and espnow will be shutdown anyway.
  if (msgsESPNowReceivedQueue && !otaActive)
  {
    if (xQueueReceive(msgsESPNowReceivedQueue,&(rxQueueESPNowItemBuffer),(TickType_t)0))
    {
      char messageType = rxQueueESPNowItemBuffer[0];
      if (isMakoMessage(messageType) && !isPairedWithMako) // only pair with Mako once first message received from Mako.
      {
        pairWithMako();
      }

      switch (messageType)
      {
        case 'X':   // location, heading and current Target info.
        {
          // format: targetCode[7],lat,long,heading,targetText
          const int targetCodeOffset = 1;
          const int latitudeOffset = 8;
          const int longitudeOffset = 16;
          const int headingOffset = 24;
          const int depthOffset = 32;
          const int courseOffset = 36;
          const int targetHeadingOffset = 40;
          const int targetDistanceOffset = 44;
          const int x_message_flags_offset = 48;
          const int currentTargetOffset = 52;
                    
          char targetCode[7];

          double old_latitude = latitude;
          double old_longitude = longitude;
          double old_heading = heading;

          strncpy(targetCode,rxQueueESPNowItemBuffer + targetCodeOffset,sizeof(targetCode));
          targetCode[sizeof(targetCode) - 1] = '\0';  // Guarantee null termination
          memcpy(&latitude,  rxQueueESPNowItemBuffer + latitudeOffset,  sizeof(double));
          memcpy(&longitude, rxQueueESPNowItemBuffer + longitudeOffset, sizeof(double));
          memcpy(&heading,   rxQueueESPNowItemBuffer + headingOffset, sizeof(double));
          memcpy(&depth,   rxQueueESPNowItemBuffer + depthOffset, sizeof(float));
          memcpy(&course,   rxQueueESPNowItemBuffer + courseOffset, sizeof(float));
          memcpy(&targetHeadingFromMako,   rxQueueESPNowItemBuffer + targetHeadingOffset, sizeof(float));
          memcpy(&targetDistanceFromMako,   rxQueueESPNowItemBuffer + targetDistanceOffset, sizeof(float));
          memcpy(&x_message_flags,   rxQueueESPNowItemBuffer + x_message_flags_offset, sizeof(uint32_t));

          locationHasFix = x_message_flags & X_MESSAGE_FIX_FLAG;

          if (locationHasFix)
            fixMessagesReceived++;
          else
            noFixMessagesReceived++;

          if (!fixMessagesReceived)
          {
            USB_SERIAL_PRINTLN("Received Location X Message: First fix not received.\n");

            awaitingFirstFixScreen(noFixMessagesReceived);
            return;
          }
          else
          {
            if (fixMessagesReceived == 1)
              compositeSprite->setTextWrap(false,false);
          }

          if (strcmp(rxQueueESPNowItemBuffer+currentTargetOffset,currentTarget) != 0)
          {
            strncpy(previousTarget,currentTarget,sizeof(previousTarget));
            strncpy(currentTarget,rxQueueESPNowItemBuffer+currentTargetOffset,sizeof(currentTarget));
            refreshTargetShown = true;
          }

          // override latitude and longitude - this is the original GPS location
          //._lat = 51.066017, ._long = 1.270883},
          // MBJMBJ - override lat long to dover wreck site
          
//          latitude = 51.066016;
  //        longitude = 1.270880;

          // 20 metres south and 20 metres east of the central point
    //      latitude = 51.065836224;
      //    longitude = 1.271165315;

          mapScreen->setLocationLatLong(latitude, longitude);
          mapScreen->setDepth(depth);
          mapScreen->setCourse(course);
          mapScreen->setHeading(heading);
          mapScreen->setTargetDirectionFromMako(targetHeadingFromMako,targetDistanceFromMako);

          USB_SERIAL_PRINTF("targetCode: %s\n",targetCode);
          USB_SERIAL_PRINTF("latitude: %f longitude: %f\n",latitude, longitude);
          USB_SERIAL_PRINTF("heading: %f course: %f depth: %f\n",heading, course, depth);
          USB_SERIAL_PRINTF("currentTarget: %s\n",currentTarget);
          USB_SERIAL_PRINTF("targetHeadingFromMako: %f targetDistanceFromMako: %f\n",targetHeadingFromMako, targetDistanceFromMako);

          mapScreen->setTargetWaypointByLabel(targetCode);

          if (testPreCannedLatLong)
          {
            latitude = old_latitude;
            longitude = old_longitude+0.00001;
            heading = static_cast<int>((old_heading + 5)) % 360;
          }

          mapScreen->drawDiverOnBestFeaturesMapAtCurrentZoom(latitude, longitude, heading);
        }

        case 'c':   // current target
        {
          if (strcmp(rxQueueESPNowItemBuffer+1,currentTarget) != 0)
          {
            strncpy(previousTarget,currentTarget,sizeof(previousTarget));
            strncpy(currentTarget,rxQueueESPNowItemBuffer+1,sizeof(currentTarget));
            refreshTargetShown = true;
          }
          break;
        }

        case 'B':   // record bread crumb trail on/off from Mako
        {
          mapScreen->setBreadCrumbTrailRecord(rxQueueESPNowItemBuffer[1] == 'Y');
          break;
        }

        case 'P':   // record bread crumb trail on/off from Mako
        {
          char* pinMessage = rxQueueESPNowItemBuffer+1;

          char* next = nullptr;

          double lat = strtod(pinMessage,&next);
          double lng = strtod(next,&next);
          double head = strtod(next,&next);
          double dep = strtod(next,&next);

          mapScreen->placePin(lat, lng, head, dep);
          break;
        }

        case 'D': // Receive current brightness from Mako
        {
          currentBrightnessReceived = *(rxQueueESPNowItemBuffer + 1)+((*(rxQueueESPNowItemBuffer + 2)) << 8);

          if (currentBrightnessSet != dimBrightness)
          {
            if (currentBrightnessReceived < nightBrightnessThreshold)
            {
              setScreenBrightness(nightBrightness);
            }
            else if (currentBrightnessReceived > dayBrightnessThreshold)
            {
              setScreenBrightness(dayBrightness);
            }
          }
        }

        default:
        {
          break;
        }
      }
    }
    if (!isPairedWithMako && millis() > nextBattUpdateTime)
    {
      nextBattUpdateTime += battUpdateCadence;
      waitingForMakoESPNowScreen();
    }
  }
}

void publishToMakoBreadCrumbRecord(const bool record)
{
  if (isPairedWithMako && ESPNow_mako_peer.channel == ESPNOW_CHANNEL)
  {
    snprintf(mako_espnow_buffer,sizeof(mako_espnow_buffer),"B%c",(record ? 'Y' : 'N'));
    USB_SERIAL_PRINTLN("Sending ESP B msg to Mako...");
    USB_SERIAL_PRINTLN(mako_espnow_buffer);

    ESPNowSendResult = esp_now_send(ESPNow_mako_peer.peer_addr, reinterpret_cast<uint8_t*>(mako_espnow_buffer), strlen(mako_espnow_buffer)+1);
  }
}

void publishToMakoTestMessage(const char* testMessage)
{
  if (isPairedWithMako && ESPNow_mako_peer.channel == ESPNOW_CHANNEL)
  {
    snprintf(mako_espnow_buffer,sizeof(mako_espnow_buffer),"T%s",testMessage);
    USB_SERIAL_PRINTLN("Sending ESP T msg to Mako...");
    USB_SERIAL_PRINTLN(mako_espnow_buffer);

    ESPNowSendResult = esp_now_send(ESPNow_mako_peer.peer_addr, reinterpret_cast<uint8_t*>(mako_espnow_buffer), strlen(mako_espnow_buffer)+1);
  }
}

void publishToMakoReedActivation(const bool topReed, const uint32_t ms)
{
  if (isPairedWithMako && ESPNow_mako_peer.channel == ESPNOW_CHANNEL)
  {
    snprintf(mako_espnow_buffer,sizeof(mako_espnow_buffer),"R%c%lu       ",(topReed ? 'T' : 'B'),ms);
    USB_SERIAL_PRINTLN("Sending ESP R msg to Mako...");
    USB_SERIAL_PRINTLN(mako_espnow_buffer);
    ESPNowSendResult = esp_now_send(ESPNow_mako_peer.peer_addr, reinterpret_cast<uint8_t*>(mako_espnow_buffer), strlen(mako_espnow_buffer)+1);
  }
  else
  {
    USB_SERIAL_PRINTLN("ESPNow inactive - not sending ESP R msg to Mako...");
  }
}


#endif
