//bibliotecas -----------------------------------------------------------------------------------------------------------
#include <lmic.h>       //lmic
#include <hal/hal.h>    //lmic
#include <SPI.h>        //lmic
#include <TinyGPS++.h>  //gps
#include "axp20x.h"     //axp
#include "SD.h"         // SD
#include "FS.h"         // SD
#include <string>
#include <Arduino.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
//bibliotecas -----------------------------------------------------------------------------------------------------------

// TBEAM-1
//configs ---------------------------------------------------------------------------------------------------------------
//chaves de autenticação OTAA
#define APPEUI_KEY 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00                                                  //lsb -> ID aplicação
#define DEVEUI_KEY 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00                                                  //lsb -> ID dispositivo
#define APPKEY_KEY 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00  //msb -> chave de aplicação AES, unica de cada dispositivo

/* // TBEAM-2
//configs ---------------------------------------------------------------------------------------------------------------
//chaves de autenticação OTAA
#define APPEUI_KEY 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00                                                  //lsb
#define DEVEUI_KEY 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00                                                  //lsb
#define APPKEY_KEY 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00  //msb
*/

/*// TBEAM-4
//configs ---------------------------------------------------------------------------------------------------------------
//chaves de autenticação OTAA
#define APPEUI_KEY 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00                                                  //lsb
#define DEVEUI_KEY 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00                                                  //lsb
#define APPKEY_KEY 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00  //msb
*/

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  1800       /* Time ESP32 will go to sleep (in seconds) */

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

//função de conversão de modulo
#define MODULO(x) ((x)>=0?(x):-(x)) 

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
    writeFile(SD, "/data.csv", "FCNT; LONGITUDE; LATITUDE; VELOCIDADE/ALTITUDE; HDOP; DATA; HORA \r\n");
  } else {
    Serial.println("SD: arquivo ja existe");
    appendFile(SD, "/data.csv", "FCNT; LONGITUDE; LATITUDE; VELOCIDADE/ALTITUDE; HDOP; DATA; HORA \r\n");
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
  axp.setPowerOutPut(AXP192_LDO2, AXP202_ON); //Lora
  axp.setPowerOutPut(AXP192_LDO3, AXP202_ON);  //GPS
  axp.setPowerOutPut(AXP192_DCDC2, AXP202_ON); //OLED
  axp.setPowerOutPut(AXP192_EXTEN, AXP202_ON); //Lora
  axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON); //GPS
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
  leitura.concat(String(gps.date.value())); //adiciona data
  leitura.concat("; \r\n");
  leitura.concat(String(gps.time.value())); //adiciona hora
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

//deep sleep ------------------------------------------------------------------------------------------------------------
/* void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason){
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
} */
//deep sleep ------------------------------------------------------------------------------------------------------------

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
      break;
  }
}
/*
float prev_lattitude = 0.0; //variaveis para verificar se houve movimento
float prev_longitude = 0.0;
int sleeptime = 0; //variavel para verificar o tempo de hibernação
*/
//função de envio LMIC
void do_send(osjob_t *j) {
  //verifica se já está ocorrendo uma tranmissão
  

  if (LMIC.opmode & OP_TXRXPEND)
    Serial.println("OP_TXRXPEND, not sending");
  else {
    /* if(gps.location.lat() - prev_lattitude > 0.0001 || gps.location.lng() - prev_longitude > 0.0001){ // verifica se houve movimento
      sleeptime = 0;
    } else{
      sleeptime += INTERVALO_ENVIO; // a variavel sleeptime representa o tempo desde que o gps parou em segundos
    }
    

    while(gps.location.lat() - prev_lattitude > 0.0001 || gps.location.lng() - prev_longitude > 0.0001){ // verifica se houve movimento
      sleeptime += 1; // a variavel sleeptime representa o tempo desde que o gps parou em segundos
      
      Serial.print("sleeptime = ");
      Serial.println(sleeptime);

      if(sleeptime > 1800){ // caso o tempo ultrapasse 30 minutos (1800/60 = 30), o axp desliga e entra em modo de hibernação
        Serial.println("Desligando o AXP");
        axp.setPowerOutPut(AXP192_LDO2, AXP202_OFF);
        axp.setPowerOutPut(AXP192_LDO3, AXP202_OFF);
        axp.setPowerOutPut(AXP192_DCDC2, AXP202_OFF);
        axp.setPowerOutPut(AXP192_EXTEN, AXP202_OFF);
        axp.setPowerOutPut(AXP192_DCDC1, AXP202_OFF);

        Serial.println("Entrando em modo de hibernação...");

        Serial.flush(); //limpa o buffer de serial

        esp_deep_sleep_start();
      }

      delay(1000);
    }
    
    prev_lattitude = gps.location.lat(); // atualiza a posição do gps
    prev_longitude = gps.location.lng();

    if(gps.hdop.value() == 9999 || (MODULO(gps.location.lat()) < 0.1 && MODULO(gps.location.lat())< 0.1)){ //HDOP 9999 geralmente significa que o GPS não está conectado ou não tem uma fixação válida - MODULO(gps.location.lat()) < 0.1 && MODULO(gps.location.lat())< 0.1 verifica se a posição do gps é 0,0 (posição inválida)
      Serial.println("Posição inválida, não enviando pacote");
      return;
    }
    */
    
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
  LMIC_setAdrMode(0);  //ativa o modo de ajuste de taxa de dados
  LMIC_setDrTxpow(DR_SF7, 14);  //configura a taxa de dados e a potência de transmissão
  // Desativa todas as fontes de despertar
  /*
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_EXT0);
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_EXT1);
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TOUCHPAD);
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ULP);
  
  // Mantém a fonte de despertar do temporizador habilitada
  // (não é necessário desativar, já que é a única fonte de despertar desejada)

  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR); //configura o tempo de hibernação
  Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) + " Seconds"); //imprime o tempo de hibernação

  */
  
  Serial.begin(SERIAL_BAUND);

  delay(3000);

  print_wakeup_reason(); //imprime o motivo do despertar

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
