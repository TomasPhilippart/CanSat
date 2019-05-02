// rf95_client.pde
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messageing client
// with the RH_RF95 class. RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example rf95_server
// Tested with Anarduino MiniWirelessLoRa, Rocket Scream Mini Ultra Pro with the RFM95W 

#include <SPI.h>
#include <RH_RF95.h>

#define RF95_FREQ     433.55


// Singleton instance of the radio driver
RH_RF95 rf95;


void setup() 
{
  Serial.begin(115200);
  if (!rf95.init())
    Serial.println("init failed");


    Serial.println("Listening at "); Serial.println(RF95_FREQ);
    rf95.setFrequency(RF95_FREQ);
}

void loop()
{
  Serial.println("Sending to CanSat");
  // Send a message to rf95_server
  uint8_t data[] = "Hello World!";
  rf95.send(data, sizeof(data));

  rf95.waitPacketSent();
  // Now wait for a reply
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  if (rf95.waitAvailableTimeout(3000))
  { 
    // Should be a reply message for us now   
    if (rf95.recv(buf, &len))
   {
      Serial.println((char*)buf);
//      Serial.print("RSSI: ");
//      Serial.println(rf95.lastRssi(), DEC);    
    }
    else
    {
      Serial.println("recv failed");
    }
  }
  else
  {
    Serial.println("No reply, is rf95_server running?");
  }
  delay(400);
}
