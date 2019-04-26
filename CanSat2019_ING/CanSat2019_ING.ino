
/*  PROJETO CANSAT - Escola Secundária José Gomes Ferreira (AEB)

    BMP280 - Pressure, Temperature and Altitude
    RFM96 LoRa - RF Transceiver
    SD Card - Storing data
    Pixy 2 Camera - Water and land detection
    Buzzer - CanSat retrieval system
    Cosmic Watch - Muon Detector
*/

// ___________________________ PARAMETERS TO ADJUST ______________________________

#define PRESSAO      1027.7  // Current barometric pressure
#define BAUD         115200  // console speed
#define DATA_INTERVAL   250  // 250 ms interval between muon count
#define SEND_INTERVAL  2000  // 2 s interval for sending interval
#define NMEA_SIZE       100  // 100 chars max
#define SIGNAL_THRESHOLD 50  // Min muon threshold to trigger on
#define TXPOWER          14  // RF entre 5-23 dbm
#define SIGMAP            3  // signature bitmap for Pixy 2 = signature 1 (bit 1) + signature 2 (bit 2) = 0000 0011 = 3 em decimal
#define WATER_SIG         1  // signature for water
#define LAND_SIG          2  // signature for land

// activate or deactivate sensors/functions
#define ENABLE_BMP           // BMP
#define ENABLE_RF            // RF
//#define ENABLE_SD            // SD card
//#define ENABLE_PIXY          // Pixy2
//#define ENABLE_CSW           // Cosmic Watch
#define DEBUG                // prints to the console

// BIBLIOTECAS
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>
#ifdef ENABLE_SD
  #include <SD.h>
#endif
#ifdef ENABLE_GPS
  #include <Adafruit_GPS.h>
  #include <SoftwareSerial.h>
#endif
#ifdef ENABLE_PIXY
  #include <Pixy2I2C.h>
  Pixy2I2C pixy;
#endif
#ifdef ENABLE_BMP
  #include <Adafruit_BMP280.h>
#endif
#ifdef ENABLE_RF
  #include <RH_RF95.h>
#endif

// RF Parameters
#define RF95_FREQ 433.55

// PINS
#define RxPin       7   // Software Serial port
#define TxPin       8   // Software Serial port

#define SD_CS       4   // Arduino D4 (SPI)

#define BUZZ_PIN    8 

#define BMP_CS      6   // Arduino D6 (SPI)

#define RFM95_CS    10  // Arduino D10 (SPI SS)
#define RFM95_RST   9   // Arduino D9 (SPI)
#define RFM95_INT   2   // Arduino D2 (SPI)

// SENSORES
#ifdef ENABLE_BMP
  //Adafruit_BMP280 bmp; //I2C
  Adafruit_BMP280 bmp(BMP_CS); // hardware SPI
#endif
#ifdef ENABLE_RF
  RH_RF95 rf95(RFM95_CS, RFM95_INT);
#endif
#ifdef ENABLE_GPS
  SoftwareSerial mySerial(RxPin, TxPin);
  Adafruit_GPS GPS(&mySerial);
#endif

// Other Variables
uint32_t timer = millis();
const float knots_2_mps = 0.5144444;
#define FILENAME   "DATA.txt"

// Calibration fit data for 10k,10k,249,10pf; 20nF,100k,100k, 0,0,57.6k,  1 point
const long double cal[] = {-9.085681659276021e-27, 4.6790804314609205e-23, -1.0317125207013292e-19,
  1.2741066484319192e-16, -9.684460759517656e-14, 4.6937937442284284e-11, -1.4553498837275352e-08,
   2.8216624998078298e-06, -0.000323032620672037, 0.019538631135788468, -0.3774384056850066, 12.324891083404246};

// structure to save data sent by RF
struct Measurements {
  char timestamp[13];       // HH:MM:SS.sss   -- incluir +1 char='/0' para terminar a string
  float latitude;           // decimal degrees
  float longitude;          // decimal degrees
  float altitude;           // meter
  float velocity;           // m/s
  int satellites;           // number
  double pressure;          // hPa
  double temperature;       // celsius
  uint8_t waterblocks;      // number of water blocks (Pixy)
  uint8_t landblocks;       // number of land blocks (Pixy)
  uint8_t muoncount;        // number of muons
  float voltage;            // amplitude of photon voltage pulse (http://cosmicwatch.lns.mit.edu/detector#how)
};

//------------------------------------ SD CARD ----------------------------------
void setup_SD() {
#ifdef ENABLE_SD
  //Initializes SD Card and checks if there are existing files
  pinMode(SD_CS, OUTPUT);
  SD.begin(SD_CS);
  if (SD.exists(FILENAME)) SD.remove(FILENAME);
  #ifdef DEBUG
     Serial.println(F("SD OK"));
  #endif
#endif
}
//------------------------------------ PIXY 2 ----------------------------------
void setup_PIXY() {
#ifdef ENABLE_PIXY
  //Initializes Pixy2
  if (pixy.init()) {
    #ifdef DEBUG
      Serial.println("PIXY2 OK");
    #endif
  } else {
    #ifdef DEBUG
      Serial.println("PIXY2 failed");
    #endif
  }
#endif
}

//------------------------------------ RF -------------------------------------
void setup_RF() {
#ifdef ENABLE_RF
  //Initializes the RFM96 Module (transceiver)
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  if (rf95.init()) {
    #ifdef DEBUG
      Serial.println("RFM96 OK");
    #endif
    } else {
    #ifdef DEBUG
      Serial.println("RFM96 failed");
    #endif
  }
  rf95.setFrequency(RF95_FREQ);
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(TXPOWER, false); // CONFIRMAR A POTENCIA A USAR

  // Send header to test connection
  const uint8_t header[] = "time,lat,long,alt,vel,sat,press,temp,water,land,muons,voltage";
  rf95.send(header, sizeof(header));
  rf95.waitPacketSent();
  #ifdef DEBUG
    Serial.println(F("RFM96 OK2"));
  #endif
#endif
}
//------------------------------------ BMP ------------------------------------
void setup_BMP() {
#ifdef ENABLE_BMP
  //Initializes BMP280
  if (bmp.begin()) {
    #ifdef DEBUG
      Serial.println("BMP280 OK");
    #endif
  } else {
    #ifdef DEBUG
      Serial.println("BMP280 failed");
    #endif
  }
#endif
}

// ================================ SETUP =====================================

void setup() {
  #ifdef DEBUG
    #ifndef ESP8266
      while (!Serial);   // pause until serial console opens
    #endif
    Serial.begin(BAUD); //initializes serial communication at baudrate defined earlier
    Serial.println(F("REBOOT"));
  #endif

  #ifdef BUZZER
    pinMode(BUZZ_PIN, OUTPUT); //sets Buzzer Pin (8) as an output
  #endif
  setup_SD();
  setup_BMP();
  setup_RF();
  //setup_GPS();
  setup_PIXY();
  // no setup required for cosmic watch
}


//----------------------------------------------- BMP FUNCTIONS ----------------------------------------------
void read_BMP_data(Measurements* data) { // measures temperature and pressure
#ifdef ENABLE_BMP
  //reading update
  double T, P, A;
  T = bmp.readTemperature();
  P = bmp.readPressure();
  A = bmp.readAltitude(P);
  //writes data on serial monitor
  #ifdef DEBUG
    Serial.print("Press: "); Serial.println(P,2);  // units = hPa
    Serial.print("Temp: "); Serial.println(T,2);   // units = ºC
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
        data->waterblocks++;
        break;
      case LAND_SIG:
        data->landblocks++;
        break;
    }
  }
  #ifdef DEBUG
    // prints number of blocks detected
    Serial.print(F("Blocks: ")); Serial.println(pixy.ccc.numBlocks);
  #endif
#endif
}
//----------------------------------------------- COSMIC WATCH FUNCTIONS ----------------------------------------------
void read_CSW_data(Measurements* data) { // counts muons
#ifdef ENABLE_CSW
  int adc = analogRead(A0); 
  data->voltage = 0;
  if (adc > SIGNAL_THRESHOLD){
    for (int i = 0; i < (sizeof(cal)/sizeof(float)); i++) {
      data->voltage += cal[i] * pow(adc,(sizeof(cal)/sizeof(float)-i-1));
  }
  data->muoncount++;
  }
  #ifdef DEBUG
    // prints number of muons detected
    Serial.print(F("Muons: ")); Serial.println(data->muoncount);
  #endif
#endif
}
//--------------------------------- RF FUNCTIONS --------------------------
void send_and_save_measurements(Measurements* data) {
  // String timestamp, float latitude, float longitude, float altitude, float velocity, int satellites,
  // double pressure, double temperature, int numblocks, int muoncount, float voltage
  char buf[500];
  char val[10] = ",";
  strcpy(buf, data->timestamp);
  dtostrf(data->latitude, 0, 4, val+1); strcat(buf, val);
  dtostrf(data->longitude, 0, 4, val+1); strcat(buf, val);
  dtostrf(data->altitude, 0, 2, val+1); strcat(buf, val);
  dtostrf(data->altitude, 0, 2, val+1); strcat(buf, val);
  dtostrf(data->velocity, 0, 2, val+1); strcat(buf, val);
  itoa(data->satellites, val+1, 10), strcat(buf, val);
  dtostrf(data->pressure, 0, 2, val+1); strcat(buf, val);
  dtostrf(data->temperature, 0, 2, val+1); strcat(buf, val);
  itoa(data->waterblocks, val+1, 10), strcat(buf, val);
  itoa(data->landblocks, val+1, 10), strcat(buf, val);
  itoa(data->muoncount, val+1, 10), strcat(buf, val);
  dtostrf(data->voltage, 0, 2, val+1); strcat(buf, val);

  #ifdef ENABLE_RF
    rf95.send(buf, strlen(buf));
    rf95.waitPacketSent();
  #endif
  #ifdef ENABLE_SD
    File mySensorData = SD.open(FILENAME, FILE_WRITE);
    mySensorData.println(buf);
    mySensorData.close();
  #endif
  //escreve no monitor série o mesmo pacote que envia por RF
  #ifdef DEBUG
    Serial.println(buf);
  #endif
}

//====================================== MAIN LOOP =========================================

Measurements dados;
bool first = true;
uint16_t elapsed = 0;



#ifdef BUZZER
  tone(buzzer, 1000);
#endif

void loop() {

  if (first) {
    memset(&dados, 0, sizeof(Measurements));
    first = false;
  }

  // MUONS                           every 250 ms
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
  }

  delay(DATA_INTERVAL);
}
