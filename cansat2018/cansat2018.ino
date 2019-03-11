/*  PROJETO CANSAT - Escola Secundária José Gomes Ferreira (AEB)
 *  Tomás Philippart 2018
 
    BMP180 - Pressão, Temperatura, Altitude (calculada)
    LIS3DH - Aceleração X, Y, Z
    FSR - Sensor de impacto
    Adafruit GPS - Longitude, Latitude, Altitude medida
    Guarda tudo num cartão SD
    RFM69 - Emissor de dados RF
*/
// ___________________________ PARAMETROS PARA AJUSTAR ______________________________

#define BAUD         115200  // console speed
#define INTERVALO      1000  // 5 segundos entre medidas + emissões de dados
#define NMEA_SIZE       100  // 100 chars max
//#define DEBUG                // imprime valores na consola (4%)
#define ENABLE_BMP           // ligar o BMP e LIS ao mesmo tempo ! (7%)
#define ENABLE_LIS           // ligar o BMP e LIS ao mesmo tempo ! (9%)
#define ENABLE_FSR           // sem impacto no PROGMEM
//#define ENABLE_GPS           // 30% do codigo
#define ENABLE_RF            // para habilitar ou desabilitar funcionalidade para teste
#define ENABLE_LDR           // valores nao mudam
//#define ENABLE_SD            // 30% do codigo

// LIVARIAS
#include <qbcan.h>
#include <Wire.h>
//#include <SPI.h>
#ifdef ENABLE_LIS
  #include <Adafruit_LIS3DH.h>
  #include <Adafruit_Sensor.h>
#endif
#ifdef ENABLE_SD
  #include <SD.h>
#endif
#ifdef ENABLE_GPS
  #include <Adafruit_GPS.h>
  #include <SoftwareSerial.h>
#endif

//parâmetros do link RF
#define NODEID        2               //nó do emissor 1..255
#define NETWORKID     100             //o mesmo valor entre 0..255 em todos os nós da rede (define o canal, i.e, a frequência)
#define GATEWAYID     1               //nó do recetor 
#define ENCRYPTKEY    "TeamAlpha2018" //chave de encriptação de 16 caracteres
#define FREQUENCY     RF69_433MHZ     //define a frequência base

// PINS
#define chipSelect   5  // pin do cartão SD (CS)
#define fsrPin      6   // o FSR e o 10k estão ligados ao pin 6 (sensor de impacto)
#define ldrPin      4   // o LDR e o 10k estão ligados ao pin 4
#define RxPin       0   //inicializa o Software Serial port
#define TxPin       1   //inicializa o Software Serial port

// SENSORES
#ifdef ENABLE_LIS
  Adafruit_LIS3DH lis = Adafruit_LIS3DH(); 
#endif
#ifdef ENABLE_BMP
  BMP180 bmp;
#endif
#ifdef ENABLE_RF
  RFM69 RF;
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
  float acceleration_x;     // m/s2
  float acceleration_y;     // m/s2
  float acceleration_z;     // m/s2
  int impact;               // newton
  int luminosidade;         // raw reading
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
  //inicializa o módulo RFM69 (emissor)
  RF.initialize(FREQUENCY, NODEID, NETWORKID);
  RF.setHighPower();      // usar a capacidadade mais alta
  RF.encrypt(ENCRYPTKEY);
  const char header[] = "time,lat,long,alt,vel,sat,press,temp,acc_x,acc_y,acc_z,impact,lum";
  RF.send(GATEWAYID, header, strlen(header));
  #ifdef DEBUG
    Serial.println(F("RFM69 OK"));
  #endif
#endif
}
//------------------------------------ BMP ------------------------------------
void setup_BMP() {
#ifdef ENABLE_BMP
  //Inicializa BMP180
  if (bmp.begin()) {
    #ifdef DEBUG
      Serial.println("BMP180 OK");
    #endif 
    } else {
    #ifdef DEBUG
      Serial.println("BMP180 failed");
    #endif
  }
#endif
}
//------------------------------------ LIS ------------------------------------
void setup_LIS() {
#ifdef ENABLE_LIS
  //Inicializa o sensor LIS3DH e se inicialização teve sucesso
  if (! lis.begin(0x18)) {   // I2C 0x18 ou 0x19
    #ifdef DEBUG
      Serial.println(F("LIS3DH failed"));
    #endif
  } else {
    lis.setRange(LIS3DH_RANGE_4_G);   // 2, 4, 8 or 16 G!
    #ifdef DEBUG
      Serial.println(F("LIS3DH OK"));
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
  setup_LIS();
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
  bmp.getData(T,P);
  //escreve dados no monitor série de Pressão e Temperatura
  #ifdef DEBUG
    Serial.print(F("Press: ")); Serial.println(P,2);  // units = hPa
    Serial.print(F("Temp: ")); Serial.println(T,2);   // units = ºC
  #endif
  data->temperature = T;
  data->pressure = P;
#endif
}
// ------------------------------------ LIS FUNCTIONS ------------------------------
void read_LIS_data(Measurements* data) {
#ifdef ENABLE_LIS
  //obtém x,y,z como eventos normalizados
  sensors_event_t event;
  lis.getEvent(&event);
  //escreve dados no monitor série de aceleração
  #ifdef DEBUG
    Serial.print(F("X: ")); Serial.println(event.acceleration.x);  // units = m/s2
    Serial.print(F("Y: ")); Serial.println(event.acceleration.y);  // units = m/s2
    Serial.print(F("Z: ")); Serial.println(event.acceleration.z);  // units = m/s2
  #endif
  data->acceleration_x = event.acceleration.x;
  data->acceleration_y = event.acceleration.y;
  data->acceleration_z = event.acceleration.z;
#endif
}
//-------------------------------- FSR FUNCTIONS ---------------------------
void read_FSR_data(Measurements* data) {
#ifdef ENABLE_FSR
  int fsrReading = analogRead(fsrPin); // A leitura (analógica) do FSR
  //escreve dados no monitor série de impacto
  #ifdef DEBUG
    Serial.print(F("Impacto: ")); Serial.println(fsrReading);
  #endif
  data->impact = fsrReading;
#endif
}

//-------------------------------- LDR FUNCTIONS ---------------------------
void read_LDR_data(Measurements* data) {
#ifdef ENABLE_LDR
  int ldrReading = analogRead(ldrPin); // A leitura (analógica) do FSR
  //escreve dados no monitor série de impacto
  #ifdef DEBUG
    Serial.print(F("Lum: ")); Serial.println(ldrReading);
  #endif
  data->luminosidade = ldrReading;
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
  dtostrf(data->velocity, 0, 2, val+1); strcat(buf, val);
  itoa(data->satellites, val+1, 10), strcat(buf, val);
  dtostrf(data->pressure, 0, 2, val+1); strcat(buf, val);
  dtostrf(data->temperature, 0, 2, val+1); strcat(buf, val);
  dtostrf(data->acceleration_x, 0, 3, val+1); strcat(buf, val);
  dtostrf(data->acceleration_y, 0, 3, val+1); strcat(buf, val);
  dtostrf(data->acceleration_z,  0,3, val+1); strcat(buf, val);
  itoa(data->impact, val+1, 10); strcat(buf, val);
  itoa(data->luminosidade, val+1, 10); strcat(buf, val);
  
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
  
  // ACCELERAÇÃO
  read_LIS_data(&dados);
  
  // FORÇA DE IMPACTO
  read_FSR_data(&dados);

  // LUMINOSIDADE
  read_LDR_data(&dados);
    
  // ENVIO DE DADOS (RF)
  send_and_save_measurements(&dados);
  
  delay(INTERVALO); // CUIDADO: deve ser ajustado para permitir o envio dos dados por RF
}
