
/*  PROJETO CANSAT - Escola Secundária José Gomes Ferreira (AEB)
 *  Tomás Philippart 2019
 
    BMP280 - Pressão, Temperatura, Altitude (calculada)
    Adafruit Ultimate GPS - Longitude, Latitude, Altitude
    RFM96 LoRa - Emissor de dados RF
  Cartão SD - Guardar dados
  Pixy 2 Camera - Detetor de agua
  Cosmic Watch - Detector de muões
*/
// ___________________________ PARAMETROS PARA AJUSTAR ______________________________

#define PRESSAO      1027.7  // Pressão atmosférica atual
#define BAUD         115200  // console speed
#define DATA_INTERVAL   250  // 1 segundo para contar muons
#define SEND_INTERVAL  2000  // 2 segundos para outras medidas e enviar
#define NMEA_SIZE       100  // 100 chars max
#define SIGNAL_THRESHOLD 50  // Min muon threshold to trigger on 
#define TXPOWER          23  // entre 5-23 dbm

// para habilitar ou desabilitar funcionalidade para teste
#define ENABLE_BMP           // BPM
//#define ENABLE_GPS         // GPS
//#define ENABLE_RF          // RF
#define ENABLE_SD            // SD card
#define ENABLE_PIXY          // Pixy2
//#define ENABLE_CSW         // Cosmic Watch
#define DEBUG                // imprime valores na consola 

// LIVARIAS
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
  #include <Pixy2SPI_SS.h>
  Pixy2SPI_SS pixy;
#endif
#ifdef ENABLE_BMP
  #include <Adafruit_BMP280.h>
#endif
#ifdef ENABLE_RF 
  #include <RH_RF95.h>
#endif

// parâmetros do link RF
#define RF95_FREQ 433.55

// PINS
#define RxPin       7   // Software Serial port
#define TxPin       8   // Software Serial port

#define BMP_CS      6   // Arduino D6 (SPI)
#define SD_CS       4   // Arduino D4 (SPI)
#define PIXY_CS     5   // Arduino D5 - senao funcionar trocar com porta 10

#define RFM95_CS    10  // Arduino D10 (porta SPI SS)
#define RFM95_RST   9   // Arduino D9 (SPI)
#define RFM95_INT   2   // Arduino D2 (SPI)

// SENSORES
#ifdef ENABLE_BMP
  Adafruit_BMP280 bmp(BMP_CS); // Hardware SPI
#endif
#ifdef ENABLE_RF
  RH_RF95 rf95(RFM95_CS, RFM95_INT); 
#endif
#ifdef ENABLE_GPS
  SoftwareSerial mySerial(RxPin, TxPin);
  Adafruit_GPS GPS(&mySerial);
#endif

// OUTRAS VARIAVEIS
uint32_t timer = millis();
const float knots_2_mps = 0.5144444;
#define FILENAME   "DATA.txt"

// Calibration fit data for 10k,10k,249,10pf; 20nF,100k,100k, 0,0,57.6k,  1 point
const long double cal[] = {-9.085681659276021e-27, 4.6790804314609205e-23, -1.0317125207013292e-19,
  1.2741066484319192e-16, -9.684460759517656e-14, 4.6937937442284284e-11, -1.4553498837275352e-08,
   2.8216624998078298e-06, -0.000323032620672037, 0.019538631135788468, -0.3774384056850066, 12.324891083404246};

// estrutura para guardar os dados a enviar por RF
struct Measurements {
  char timestamp[13];       // HH:MM:SS.sss   -- incluir +1 char='/0' para terminar a string
  float latitude;           // decimal degrees
  float longitude;          // decimal degrees  
  float altitude;           // meter
  float velocity;           // m/s
  int satellites;           // number
  double pressure;          // hPa
  double temperature;       // celsius
  uint8_t numblocks;        // number of blocks (Pixy)
  uint8_t muoncount;        // number of muons
  float voltage;            // amplitude of photon voltage pulse (http://cosmicwatch.lns.mit.edu/detector#how)
};

//------------------------------------ SD CARD ----------------------------------
void setup_SD() {
#ifdef ENABLE_SD
  //Inicializa cartão SD e verifica se já existem ficheiros no cartão
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
  //Inicializa a Pixy2
  if (pixy.init(PIXY_CS)) {
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
//------------------------------------ GPS -----------------------------------
void setup_GPS() {  
#ifdef ENABLE_GPS
  //Inicializa o GPS
  GPS.begin(9600);                              //Turn on GPS at 9600 baud
  GPS.sendCommand("$PGCMD,33,0*6D");            //Turn off antenna update nuisance data
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); //Request RMC and GGA Sentences only
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);    //Set update rate to 1 Hz
  delay(1000);
  mySerial.println(PMTK_Q_RELEASE);             // Ask for firmware version ???
  #ifdef DEBUG
    Serial.println(F("GPS OK"));
  #endif
#endif
}
//------------------------------------ RF -------------------------------------
void setup_RF() {
#ifdef ENABLE_RF
  //inicializa o módulo RFM96 (emissor)
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
  
  // enviar o header para testar a ligação
  const uint8_t header[] = "time,lat,long,alt,vel,sat,press,temp,blocks,count,voltage";
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
  //Inicializa BMP280
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
    Serial.begin(BAUD); //inicializa comunicação série
    Serial.println(F("REBOOT"));
  #endif

  setup_SD();
  setup_RF();
  setup_BMP();
  setup_GPS();
  setup_PIXY();
  // no setup required for cosmic watch
}

//-------------------------- GPS FUNCTIONS --------------------------
void read_GPS_data(Measurements* data) { 
  /* mede o instante e a localização atual do cansat (tempo, lat, long, alt) 
   * exemplo: 3:40:51.0, -32.9408, 151.7184, 2300 
   */
#ifdef ENABLE_GPS
  char c = GPS.read();
  #ifdef DEBUG
    if (c) Serial.write(c);
  #endif
  if (GPS.newNMEAreceived()) {      // if a sentence is received, we can check the checksum, parse it...
    if (!GPS.parse(GPS.lastNMEA())) // this sets the newNMEAreceived() flag to false
      return;                       // we can fail to parse a sentence in which case we should just wait for another
  }
  if (timer > millis())  timer = millis(); // if millis() or timer wraps around, we'll just reset it
  if (millis() - timer > SEND_INTERVAL) {  // approximately every 2 seconds or so, print out the current stats
    timer = millis();                      // reset the timer
    if (GPS.fix == 1) {                    // guardar dados se tivermos um fix (contacto com satélites)
      #ifdef DEBUG
        Serial.print(F("Lat: ")); Serial.println(GPS.latitudeDegrees); 
        Serial.print(F("Lon: ")); Serial.println(GPS.longitudeDegrees);
      #endif
      sprintf(data->timestamp, "%d:%d:%d.%d", GPS.hour, GPS.minute, GPS.seconds, GPS.milliseconds);
      data->latitude = GPS.latitudeDegrees;
      data->longitude = GPS.longitudeDegrees;
      data->altitude = GPS.altitude; 
      data->velocity = GPS.speed * knots_2_mps;
      data->satellites = GPS.satellites;
    }
  }
#endif
}
//----------------------------------------------- BMP FUNCTIONS ----------------------------------------------
void read_BMP_data(Measurements* data) { // mede a temperatura e a pressão
#ifdef ENABLE_BMP
  //atualização da leitura de temperatura e pressão
  double T, P, A;
  T = bmp.readTemperature();
  P = bmp.readPressure();
  A = bmp.readAltitude(PRESSAO);
  //escreve dados no monitor série de Pressão e Temperatura
  #ifdef DEBUG
    Serial.print(F("Press: ")); Serial.println(P,2);  // units = hPa
    Serial.print(F("Temp: ")); Serial.println(T,2);   // units = ºC
  #endif
  data->temperature = T;
  data->pressure = P;
  data->altitude = A;
#endif
}
//----------------------------------------------- PIXY FUNCTIONS ----------------------------------------------
void read_PIXY_data(Measurements* data) { // mede a temperatura e a pressão
#ifdef ENABLE_PIXY
  pixy.ccc.getBlocks();
  data->numblocks = pixy.ccc.numBlocks; // DEVEMOS CONTAR CADA COR SEPARADAMENTE ???
  #ifdef DEBUG
    // prints number of blocks are detected 
    Serial.print(F("Blocks: ")); Serial.println(pixy.ccc.numBlocks);     
  #endif
#endif
}
//----------------------------------------------- COSMIC WATCH FUNCTIONS ----------------------------------------------
void read_CSW_data(Measurements* data) { // conta os muons
#ifdef ENABLE_CSW
  int adc = analogRead(A0); // verificar pin
  data->voltage = 0
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
  itoa(data->numblocks, val+1, 10), strcat(buf, val);
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
memset(&dados, 0, sizeof(Measurements));
uint16_t elapsed = 0;

void loop() {
  
  // MUONS                           cada 1 segundo
  read_CSW_data(&dados);
  elapsed += DATA_INTERVAL;

  if (elapsed >= SEND_INTERVAL) { // cada 2 segundos
    elapsed = 0;
  
    // GPS LATITUDE, LONGITUDE & ALTITUDE
    read_GPS_data(&dados);
  
    // PRESSÃO, TEMPERATURA, ALTITUDE (calc)
    read_BMP_data(&dados);
  
    // BLOCKS
    read_PIXY_data(&dados);
    
    // ENVIO DE DADOS (RF)
    send_and_save_measurements(&dados);
  
    // limpar os dados anteriores
    memset(&dados, 0, sizeof(Measurements));
  }
  
  delay(DATA_INTERVAL);
}
