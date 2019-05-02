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


    Serial.print("Listening at "); Serial.print(RF95_FREQ); Serial.println(" MHz");
    rf95.setFrequency(RF95_FREQ);
}

void loop()
{
  // Waiting for a reply
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  if (rf95.waitAvailableTimeout(3000)) { 
    // Should be a reply message for us now   
    if (rf95.recv(buf, &len)) {
      Serial.println((char*)buf); 
    }
  }
  
  delay(400);
}
