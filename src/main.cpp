//bibliotecas -----------------------------------------------------------------------------------------------------------
#include <lmic.h>       //lmic
#include <hal/hal.h>    //lmic
#include <SPI.h>        //lmic
#include <TinyGPS++.h>  //gps
#include "axp20x.h"     //axp
#include "SD.h"         // SD
#include "FS.h"         // SD
#include <string>
//bibliotecas -----------------------------------------------------------------------------------------------------------

// TBEAM-1
//configs ---------------------------------------------------------------------------------------------------------------
//chaves de autenticação OTAA
#define APPEUI_KEY 0x46, 0x58, 0x54, 0x37, 0x46, 0x63, 0x34, 0x35                                                  //lsb -> ID aplicação
#define DEVEUI_KEY 0x85, 0x74, 0x06, 0xD0, 0x7E, 0xD5, 0xB3, 0x70                                                  //lsb -> ID dispositivo
#define APPKEY_KEY 0x5E, 0x24, 0x72, 0x71, 0xDE, 0x23, 0xF1, 0x8E, 0xC7, 0xE6, 0x79, 0x3F, 0x94, 0x92, 0x83, 0xAD  //msb -> chave de aplicação AES, unica de cada dispositivo


// TBEAM-2
//configs ---------------------------------------------------------------------------------------------------------------
//chaves de autenticação OTAA
//#define APPEUI_KEY 0x23, 0x31, 0x21, 0x24, 0x42, 0x43, 0x31, 0x12                                                //lsb
//#define DEVEUI_KEY 0x84, 0x74, 0x06, 0xD0, 0x7E, 0xD5, 0xB3, 0x70                                                 //lsb
//#define APPKEY_KEY 0x03, 0x79, 0x20, 0xBA, 0xEC, 0xA0, 0xBA, 0x33, 0xB9, 0x97, 0xE6, 0x18, 0xCB, 0x9C, 0x2F, 0xC9  //msb


/*// TBEAM-4
//configs ---------------------------------------------------------------------------------------------------------------
//chaves de autenticação OTAA
#define APPEUI_KEY 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00                                                  //lsb
#define DEVEUI_KEY 0xFE, 0x1D, 0x06, 0xD0, 0x7E, 0xD5, 0xB3, 0x70                                                  //lsb
#define APPKEY_KEY 0x1D, 0x3D, 0x39, 0x82, 0x9E, 0x77, 0xCA, 0x01, 0xF6, 0xA8, 0x84, 0x0E, 0xE5, 0x43, 0xB8, 0x6C  //msb
*/

//constante de intervalo de envio
#define INTERVALO_ENVIO 30  //em segundos

// corresponde à banda 2 do AU915 na TTN
#define SUBBAND 1

//mapa de pinos
#define NSS_PIN 18
#define RST_PIN 23
#define DIO0_PIN 26
#define DIO1_PIN 33
#define DIO2_PIN 32

//mapa de pinos SD
#define SD_MISO 35
#define SD_MOSI 13
#define SD_SCK 14
#define SD_CS 25

//velocidade serial
#define SERIAL_BAUND 9600

//informação do pacote
#define USE_SPEED
//#define USE_ALTITUDE
//configs ---------------------------------------------------------------------------------------------------------------

#define MIN_DISTANCE 20.0

//SD --------------------------------------------------------------------------------------------------------------------
SPIClass mySPI = SPIClass(HSPI);  //SPI virtual

void writeFile(fs::FS &fs, const char *path, const char *message) {
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if (file.print(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

void appendFile(fs::FS &fs, const char *path, const char *message) {
  digitalWrite(SD_CS, HIGH);
  digitalWrite(18, LOW);
  Serial.printf("Appending to file: %s\n", path);
  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if (file.print(message)) {
    Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
  digitalWrite(18, HIGH);
  digitalWrite(SD_CS, LOW);
}

void setupSD() {
  mySPI.begin(SD_SCK, SD_MISO, SD_MOSI);
  if (!SD.begin(SD_CS, mySPI, 10000000)) {
    Serial.println("Erro na leitura do arquivo não existe um cartão SD ou o módulo está conectado incorretamente...");
    return;
  }

  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("Nenhum cartao SD encontrado");
    return;
  }

  Serial.println("Inicializando cartao SD...");
  //if (!SD.begin(SD_CS, spi1))
  if (!SD.begin(SD_CS)) {
    Serial.println("ERRO - SD nao inicializado!");
    return;
  }
  File file = SD.open("/data.csv");
  if (!file) {
    Serial.println("SD: arquivo data.csv nao existe");
    Serial.println("SD: Criando arquivo...");
    writeFile(SD, "/data.csv", "FCNT; LONGITUDE; LATITUDE; VELOCIDADE/ALTITUDE; HDOP; \r\n");
  } else {
    Serial.println("SD: arquivo ja existe");
    appendFile(SD, "/data.csv", "FCNT; LONGITUDE; LATITUDE; VELOCIDADE/ALTITUDE; HDOP; \r\n");
  }

  file.close();
}
//SD --------------------------------------------------------------------------------------------------------------------

//axp -------------------------------------------------------------------------------------------------------------------
AXP20X_Class axp;

void configureAXP() {
  Wire.begin(21, 22);  // configurado a comunicação com o axp
  if (!axp.begin(Wire, AXP192_SLAVE_ADDRESS)) {
    Serial.println("AXP192 Begin PASS");
  } else {
    Serial.println("AXP192 Begin FAIL");
  }
  axp.setPowerOutPut(AXP192_LDO2, AXP202_ON);
  axp.setPowerOutPut(AXP192_LDO3, AXP202_ON);
  axp.setPowerOutPut(AXP192_DCDC2, AXP202_ON);
  axp.setPowerOutPut(AXP192_EXTEN, AXP202_ON);
  axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON);
}
//axp -------------------------------------------------------------------------------------------------------------------

//gps -------------------------------------------------------------------------------------------------------------------
TinyGPSPlus gps;
HardwareSerial GPS(1);

void configureGPS() {
  GPS.begin(9600, SERIAL_8N1, 34, 12);
}

void loopGPS() {
  while (GPS.available())
    gps.encode(GPS.read());
}
//gps -------------------------------------------------------------------------------------------------------------------

bool conectado = false;
//packet ----------------------------------------------------------------------------------------------------------------

//Contrução de pacote dos dados para anexação no SD
void buildPacket_toSD() {
  String leitura;
  leitura.concat(String(LMIC.seqnoUp));
  leitura.concat(";");
  leitura.concat(String(gps.location.lat(), 8));
  leitura.concat(";");
  leitura.concat(String(gps.location.lng(), 8));
  leitura.concat(";");
  leitura.concat(String(gps.speed.kmph()));
  leitura.concat(";");
  leitura.concat(String(gps.hdop.value()));
  leitura.concat("; \r\n");
  Serial.println(leitura);
  appendFile(SD, "/data.csv", leitura.c_str());
}


//Construção de pacote com os dados do gps para envio
void buildPacket(uint8_t packet[9]) {
  uint32_t LatitudeBinary = ((gps.location.lat() + 90) / 180.0) * 16777215;
  packet[0] = (LatitudeBinary >> 16) & 0xFF;
  packet[1] = (LatitudeBinary >> 8) & 0xFF;
  packet[2] = LatitudeBinary & 0xFF;

  uint32_t LongitudeBinary = ((gps.location.lng() + 180) / 360.0) * 16777215;
  packet[3] = (LongitudeBinary >> 16) & 0xFF;
  packet[4] = (LongitudeBinary >> 8) & 0xFF;
  packet[5] = LongitudeBinary & 0xFF;

#ifdef USE_SPEED
  uint16_t Speed = gps.speed.kmph();
  packet[6] = (Speed >> 8) & 0xFF;
  packet[7] = Speed & 0xFF;
#endif

#ifdef USE_ALTITUDE
  uint16_t Altitude = gps.altitude.meters() * 10;
  packet[6] = (Altitude >> 8) & 0xFF;
  packet[7] = Altitude & 0xFF;
#endif

  uint8_t Hdops = gps.hdop.value() / 10;
  packet[8] = Hdops & 0xFF;

  if (conectado) {
    buildPacket_toSD();
  }
}

void printPacket() {
  Serial.print("Latitude: ");
  Serial.println(gps.location.lat());
  Serial.print("Longitude: ");
  Serial.println(gps.location.lng());
  Serial.print("Altitude: ");
  Serial.println(gps.altitude.meters());
  Serial.print("Hdop: ");
  Serial.println(gps.hdop.value());
}
//packet ----------------------------------------------------------------------------------------------------------------

//LMIC ------------------------------------------------------------------------------------------------------------------
//chaves OTAA LMIC
static const u1_t PROGMEM APPEUI[8] = { APPEUI_KEY };  //lsb format
void os_getArtEui(u1_t *buf) {
  memcpy_P(buf, APPEUI, 8);
}
static const u1_t PROGMEM DEVEUI[8] = { DEVEUI_KEY };  // lsb format
void os_getDevEui(u1_t *buf) {
  memcpy_P(buf, DEVEUI, 8);
}
static const u1_t PROGMEM APPKEY[16] = { APPKEY_KEY };  //msb format
void os_getDevKey(u1_t *buf) {
  memcpy_P(buf, APPKEY, 16);
}

//sendjob LMIC
static osjob_t sendjob;

//pinmap TBEAM LMIC - settings.h
const lmic_pinmap lmic_pins = {
  .nss = NSS_PIN,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = RST_PIN,
  .dio = { DIO0_PIN, DIO1_PIN, DIO2_PIN },
};

const unsigned TX_INTERVAL = INTERVALO_ENVIO;

//payload
uint8_t payload[9];

//função de envio LMIC
void do_send(osjob_t *j);

//LMIC - eventos de comunicação
//os únicos eventos estanciados são EV_JOINING e EV_TXCOMPLETE
void onEvent(ev_t ev) {
  switch (ev) {
    case EV_JOINED:
      Serial.print(os_getTime());
      Serial.print(": ");
      Serial.println(F("EV_JOINED"));
      // Disable link check validation (automatically enabled
      // during join, but not supported by TTN at this time).
      //? Se já deu Join uma vez, desativa a verificação para as próximas comunicações
      LMIC_setLinkCheckMode(0);
      conectado = true;
      break;
    case EV_TXCOMPLETE:
      Serial.print(os_getTime());
      Serial.print(": ");
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ack"));
      if (LMIC.dataLen) {
        Serial.println(F("Received "));
        Serial.println(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
      }
      //!Agenda as proximas transmissões em TX_INTERVAL segundos
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
      break;
    default:
      Serial.print(os_getTime());
      Serial.print(": ");
      Serial.print(F("Unknown event: "));
      Serial.println((unsigned)ev);
      break;
  }
}

//função de envio LMIC
void do_send(osjob_t *j) {
  //verifica se já está ocorrendo uma tranmissão
  if (LMIC.opmode & OP_TXRXPEND)
    Serial.println("OP_TXRXPEND, not sending");
  else {
    buildPacket(payload);  //*contrução do pacote para envio - packet.h
    printPacket();
    LMIC_setTxData2(1, payload, sizeof(payload), 0);  //função de envio LMIC
    Serial.println("Packet queued");
    Serial.print("Sending packet on frequency: ");
    Serial.println(LMIC.freq);
  }
}
//LMIC ------------------------------------------------------------------------------------------------------------------


void setup() {

  Serial.begin(SERIAL_BAUND);

  delay(3000);

  Serial.println("Starting");

  //iniciando módulos
  configureAXP();  //axp.h
  Serial.println("Axp OK");
  delay(300);
  configureGPS();
  Serial.println("GPS OK");
  delay(300);
  setupSD();
  Serial.println("SD OK");
  delay(300);

  //iniciando lmic
  os_init();
  Serial.println("LMIC OK");
  delay(1000);
  LMIC_reset();  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_selectSubBand(SUBBAND);
  delay(1000);
  do_send(&sendjob);  //Start
}

void loop() {
  loopGPS();  //leitura GPS
  os_runloop_once();
}