
/*  PROJETO CANSAT - Escola Secundária José Gomes Ferreira (AEB)

    BMP280 - Pressure, Temperature and Altitude
    RFM96 LoRa - RF Transceiver
    SD Card - Storing data
    Pixy 2 Camera - Water and land detection
    Buzzer - CanSat retrieval system
    Cosmic Watch - Muon Detector
*/

// ___________________________ PARAMETERS TO ADJUST ______________________________

#define PRESSAO      1027.7  // Current barometric pressure at sea level
#define BAUD         115200  // console speed
#define DATA_INTERVAL   250  // 250 ms interval between muon count
#define SEND_INTERVAL  2000  // 2 s interval for sending interval
#define SIGNAL_THRESHOLD 50  // Min muon threshold to trigger on
#define RF95_FREQ    433.55  // RF frequency
#define TXPOWER          14  // RF entre 5-23 dbm
#define SIGMAP            3  // signature bitmap for Pixy 2 = signature 1 (bit 1) + signature 2 (bit 2) = 0000 0011 = 3 em decimal
#define WATER_SIG         1  // signature for water
#define LAND_SIG          2  // signature for land

// activate or deactivate sensors/functions
#define ENABLE_BMP           // BMP sensor
#define ENABLE_RF            // RF
//#define ENABLE_SD            // SD card
#define ENABLE_PIXY          // Pixy2 camera
#define ENABLE_CSW           // Cosmic Watch muon detector
#define ENABLE_BUZZER        // Buzzer
#define DEBUG                // prints to the console

// PINS
#define SD_CS       4   // Arduino D4 (SPI)
#define BMP_CS      6   // Arduino D6 (SPI)
#define RFM_CS     10   // Arduino D10 (SPI SS)
#define RFM_RST     9   // Arduino D9 (SPI)
#define RFM_INT     2   // Arduino D2 (SPI)
#define BUZZ_PIN    8 

// BIBLIOTECAS
#include <Wire.h>
#include <SPI.h>
#ifdef ENABLE_SD
  #include <SdFat.h>
  SdFat sd;
  SdFile file;
#endif
#ifdef ENABLE_BMP
  #include <Adafruit_BMP280.h>
  Adafruit_BMP280 bmp(BMP_CS); // hardware SPI
#endif
#ifdef ENABLE_RF
  #include <RH_RF95.h>
  RH_RF95 rf95(RFM_CS, RFM_INT); // SPI
#endif
#ifdef ENABLE_PIXY
  #include <Pixy2I2C.h>
  Pixy2I2C pixy;
#endif

// structure to save data sent by RF
struct Measurements {
  float altitude;         // meter
  double pressure;        // hPa
  double temperature;     // celsius
  uint8_t water;          // number of water blocks (Pixy)
  uint8_t land;           // number of land blocks (Pixy)
  uint8_t muons;          // number of muons
};

// GLOBAL VARIABLES
uint32_t sequence = 0;    // sequence number of RF messages
uint16_t elapsed = 0;     // elapsed time since last message
char filename[12] = "data"; // SD filename
Measurements dados;       // measurement data

//------------------------------------ SD CARD ----------------------------------
void setup_SD() {
#ifdef ENABLE_SD
  // create random filename
  uint8_t num = analogRead(A1); // random value (0 to 255)
  itoa(num, filename+4, 10);
  strcat(filename, ".csv");
  
  // initialize SD card 
  if (!sd.begin(SD_CS)) sd.initErrorHalt();
  if (sd.exists(filename)) sd.remove(filename);
#endif
}
//------------------------------------ PIXY 2 ----------------------------------
void setup_PIXY() {
#ifdef ENABLE_PIXY
  // Initialize Pixy2 camera
  if (pixy.init()) {
    #ifdef DEBUG
      Serial.println(F("PIXY2 OK"));
    #endif
  } else {
    #ifdef DEBUG
      Serial.println(F("PIXY2 failed"));
    #endif
  }
#endif
}
//------------------------------------ RF -------------------------------------
void setup_RF() {
#ifdef ENABLE_RF
  // Initialize the RFM96 Module
  if (!rf95.init()) {
    #ifdef DEBUG
      Serial.println(F("RFM96 failed"));
    #endif
  }
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
  rf95.setFrequency(RF95_FREQ);
  
  // you can set transmitter powers from 5 to 23 dBm:
  //rf95.setTxPower(TXPOWER, false); // CONFIRMAR A POTENCIA A USAR

  // Send header to test connection
  const uint8_t header[] = "time,alt,press,temp,water,land,muons";
  rf95.send(header, sizeof(header));
  rf95.waitPacketSent();
  #ifdef DEBUG
    Serial.println(F("RFM96 OK"));
  #endif
#endif
}
//------------------------------------ BMP ------------------------------------
void setup_BMP() {
#ifdef ENABLE_BMP
  // Initialize BMP280 sensor
  if (bmp.begin()) {
    #ifdef DEBUG
      Serial.println(F("BMP280 OK"));
    #endif
  } else {
    #ifdef DEBUG
      Serial.println(F("BMP280 failed"));
    #endif
  }
#endif
}
//--------------------------------- BUZZER ------------------------------------
void setup_BUZZER() {
#ifdef ENABLE_BUZZER
  pinMode(BUZZ_PIN, OUTPUT); //sets Buzzer Pin (8) as an output
#endif
}

// ================================ SETUP =====================================

void setup() {
  #ifdef DEBUG
    #ifndef ESP8266
      while (!Serial) yield();   // pause until serial console opens
    #endif
    Serial.begin(BAUD); //initializes serial communication at baudrate defined earlier
    Serial.println(F("REBOOT"));
  #endif

  // initialize measurements to zero
  memset(&dados, 0, sizeof(Measurements));

  // disable all SPI devices before initializing
  pinMode(BMP_CS, OUTPUT);
  pinMode(RFM_CS, OUTPUT);
  pinMode(SD_CS, OUTPUT);
  digitalWrite(BMP_CS, HIGH);
  digitalWrite(RFM_CS, HIGH);
  digitalWrite(SD_CS, HIGH);
  delay(10);
  SPI.begin();
  
  setup_BUZZER();
  setup_BMP();
  setup_SD();
  setup_PIXY();
  setup_RF();
  // no setup required for cosmic watch
}

//----------------------------------------------- BMP FUNCTIONS ----------------------------------------------
void read_BMP_data(Measurements* data) { // measures temperature and pressure
#ifdef ENABLE_BMP
  //reading update
  double T, P, A;
  T = bmp.readTemperature();
  P = bmp.readPressure() / 100;   // convert Pa to hPa
  A = bmp.readAltitude(PRESSAO);  // current sea level pressure in Santa Maria
  //writes data on serial monitor
  #ifdef DEBUG
    //Serial.println(P,2);  // units = hPa
    //Serial.println(T,2);  // units = ºC
    //Serial.println(A,2);  // units = m
  #endif
  data->temperature = T;
  data->pressure = P;
  data->altitude = A;
#endif
}
//----------------------------------------------- PIXY FUNCTIONS ----------------------------------------------
void read_PIXY_data(Measurements* data) { 
#ifdef ENABLE_PIXY
  pixy.ccc.getBlocks(false, SIGMAP); // no waiting, water & land signatures
  for (int i = 0; i < pixy.ccc.numBlocks; i++) {
    switch (pixy.ccc.blocks[i].m_signature) {
      case WATER_SIG:
        data->water++;
        break;
      case LAND_SIG:
        data->land++;
        break;
    }
  }
  #ifdef DEBUG
    // prints number of blocks detected
    //Serial.println(pixy.ccc.numBlocks);
  #endif
#endif
}
//----------------------------------------------- COSMIC WATCH FUNCTIONS ----------------------------------------------
void read_CSW_data(Measurements* data) { // counts muons
#ifdef ENABLE_CSW
  int adc = analogRead(A0); 
  if (adc > SIGNAL_THRESHOLD){
    data->muons++;
  }
  #ifdef DEBUG
    // prints muon pulse signal amplitude
    Serial.println(adc);
  #endif
#endif
}
//--------------------------------- RF FUNCTIONS --------------------------
void send_and_save_measurements(Measurements* data) {
  // int sequence, float altitude, double pressure, double temperature, 
  // int water, int land, int muons
  char buf[100];
  char val[10] = ",";
  
  itoa(sequence++, buf, 10);
  dtostrf(data->altitude, 0, 2, val+1); strcat(buf, val);
  dtostrf(data->pressure, 0, 2, val+1); strcat(buf, val);
  dtostrf(data->temperature, 0, 2, val+1); strcat(buf, val);
  itoa(data->water, val+1, 10), strcat(buf, val);
  itoa(data->land, val+1, 10), strcat(buf, val);
  itoa(data->muons, val+1, 10), strcat(buf, val);

  #ifdef ENABLE_RF
    rf95.send(buf, strlen(buf));
    rf95.waitPacketSent();
  #endif
  #ifdef ENABLE_SD
    if (file.open(filename, O_RDWR | O_CREAT | O_AT_END)) {
      file.println(buf);
      file.close();
    }
  #endif
  //escreve no monitor série o mesmo pacote que envia por RF
  #ifdef DEBUG
    Serial.println(buf);
  #endif
}
//------------------------------ BUZZER FUNCTIONS --------------------------
void buzzer_on(Measurements* data) {
#ifdef ENABLE_BUZZER
  if (data->altitude < 200) { // below 200 meters
    tone(BUZZ_PIN, 500, 100); // 500 Hz is the frequency (La = 440 Hz), 100ms duration
  }
#endif
}

//====================================== MAIN LOOP =========================================

void loop() {
  
  // MUONS every 250 ms
  read_CSW_data(&dados);
  elapsed += DATA_INTERVAL;

  if (elapsed >= SEND_INTERVAL) { // every 2 s
    elapsed = 0;
    
    // PRESSURE, TEMPERATURE AND ALTITUDE
    read_BMP_data(&dados);
    
    // BLOCKS
    read_PIXY_data(&dados);

    // DATA TRANSMISSION (RF)
    send_and_save_measurements(&dados);

    // cleans previous data
    memset(&dados, 0, sizeof(Measurements));

    // buzzer on below 200 meter altitude
    buzzer_on(&dados);
  }

  delay(DATA_INTERVAL);
}
