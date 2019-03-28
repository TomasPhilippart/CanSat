
/*  PROJETO CANSAT - Escola Secundária José Gomes Ferreira (AEB)
 *  Tomás Philippart 2019
 
    BMP280 - Pressão, Temperatura, Altitude (calculada)
    Adafruit Ultimate GPS - Longitude, Latitude, Altitude medida
    Guarda tudo num cartão SD
    RFM69W LoRa - Emissor de dados RF
*/
// ___________________________ PARAMETROS PARA AJUSTAR ______________________________

#define PRESSÃO      1027.7 // Pressão atmosférica atual
#define BAUD         115200  // console speed
#define INTERVALO      2000  // 2 segundos entre medidas + emissões de dados
//#define NMEA_SIZE       100  // 100 chars max
#define DEBUG                // imprime valores na consola 
//#define ENABLE_BMP           // ligar o BMP
//#define ENABLE_GPS           // 
//#define ENABLE_RF            // para habilitar ou desabilitar funcionalidade para teste
//#define ENABLE_SD            // 
#define ENABLE_PIXY

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
  #include <Pixy2.h>
#endif
#ifdef ENABLE_BMP
  #include <Adafruit_BMP280.h>
#endif
#ifdef ENABLE_RF 
  #include <RH_RF95.h>
#endif

//parâmetros do link RF
#define RF95_FREQ 433.0

// PINS
#define chipSelect  10 
#define RxPin       1   //inicializa o Software Serial port
#define TxPin       0   //inicializa o Software Serial port
#define RFM95_CS    10  // VERIFICAR
#define RFM95_RST   9   // VERIFICAR
#define RFM95_INT   2   // VERIFICAR

// SENSORES
#ifdef ENABLE_BMP
  Adafruit_BMP280 bmp; //I2C
#endif
#ifdef ENABLE_RF
  RH_RF95 rf95(RFM95_CS, RFM95_INT);
#endif
#ifdef ENABLE_GPS
  SoftwareSerial mySerial(RxPin, TxPin);
  Adafruit_GPS GPS(&mySerial);
#endif

// outras
uint32_t timer = millis();
const float knots_2_mps = 0.5144444;
#define FILENAME   "DATA.txt"

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
  
};

//------------------------------------ SD CARD ----------------------------------
void setup_SD() {
#ifdef ENABLE_SD
  //Inicializa cartão SD e verifica se já existem ficheiros no cartão
  pinMode(10, OUTPUT);
  SD.begin(chipSelect); //inicializar o cartão SD no pino definido anteriormente
  if (SD.exists(FILENAME)) SD.remove(FILENAME);
  #ifdef DEBUG
     Serial.println(F("SD OK"));
  #endif
#endif
}
//------------------------------------ PIXY 2 ----------------------------------
void setup_PIXY() {
#ifdef ENABLE_PIXY
  Pixy2 pixy;
  //Inicializa a Pixy2
  if (pixy.init()) {
    #ifdef DEBUG
      Serial.println("PIXY2 OK");
    #endif 
    } else {
    #ifdef DEBUG
      Serial.println("PIXY2 failed");
    #endif
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
  if (r95.init()) {
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
  rf95.setTxPower(23, false);
  
  // enviar o header para testar a ligação
  const uint8_t header[] = "time,lat,long,alt,vel,sat,press,temp,acc_x,acc_y,acc_z,impact,lum";
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
  if (millis() - timer > 2000) {           // approximately every 2 seconds or so, print out the current stats
    timer = millis();                      // reset the timer
    if (GPS.fix == 1) {                    //So guardar dados se tivermos um fix (contacto com satélites)
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
  double T, P;
  T = bmp.readTemperature()
  P = bmp.readPressure()
  A = bmp.readAltitude(PRESSÃO);
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
  int i;
  pixy.ccc.getBlocks();

  if (pixy.ccc.numBlocks) {
      #ifdef DEBUG
        Serial.print(F("Blocks: ")); Serial.println(pixy.ccc.numBlocks);  // prints how many blocks are detected      
      #endif

      for (i=0; i<pixy.ccc.numBlocks; i++= {
        Serial.print("  block  ");
        Serial.print(i);
        Serial.print(": ");
        pixy.ccc.blocks[i].print();
      }
  }

#endif
}
//--------------------------------- RF FUNCTIONS --------------------------
void send_and_save_measurements(Measurements* data) {
  // String timestamp, float latitude, float longitude, float altitude, float velocity, int satellites, 
  // double pressure, double temperature, float acceleration_x, float acceleration_y, float acceleration_z, int impact, int luminosidade
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
  
  #ifdef ENABLE_RF
    RF.send(GATEWAYID, buf, strlen(buf));
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
void loop() {
  Measurements dados;

  // limpar os dados anteriores
  memset(&dados, 0, sizeof(Measurements));
  
  // GPS LATITUDE, LONGITUDE & ALTITUDE
  read_GPS_data(&dados);
  
  // PRESSÃO, TEMPERATURA, ALTITUDE (calc)
  read_BMP_data(&dados);
    
  // ENVIO DE DADOS (RF)
  send_and_save_measurements(&dados);
  
  delay(INTERVALO); // CUIDADO: deve ser ajustado para permitir o envio dos dados por RF
}
