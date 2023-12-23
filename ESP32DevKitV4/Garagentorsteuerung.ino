//https://beelogger.de/sensoren/temperatursensor-ds18b20/ für Pinning und Anregung
#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include <EmonLib.h>                   // Auswertung der SCT013-Sensoren
#include <esp_task_wdt.h>

#define LED_ERROR 23
#define LED_MSG 22
#define LED_OK 19
#define REED1 27                       // Tor-Zu-Sensor
#define REED2 26                       // Zwischenstufe 1
#define REED3 32                       // Zwischenstufe 2
#define REED4 33                       // Tor-Auf-Sensor
#define RELAIS 18                      // Relais zur Torsteuerung
#define ONE_WIRE_BUS 25
#define TRIGGERTIME 800                // 800ms Triggerzeit für das Garagentor
#define SENSORINVERT 1                 // 0 = Magnetsensoren werden nicht invertiert; 1 = Sensoren werden invertiert
static byte debug = 1;
bool tempError = 0;                    // tempError = 1 => keine DS18B20-Sensoren gefunden.
static String lastError = "";

// Definition der Zugangsdaten WiFi
#define HOSTNAME "ESP32_Garagentorsteuerung"
const char* ssid = "YourSSID";
const char* password = "YourWiFiPassword";
WiFiClient myWiFiClient;

//Definition der Zugangsdaten MQTT
#define MQTT_SERVER "192.168.2.127"
#define MQTT_PORT 1883
#define MQTT_USER "YourMQTTBrokerAccount"
#define MQTT_PASSWORD "YourMQTTPassword"
#define MQTT_CLIENTID "ESP32_Garagentorsteuerung" //Name muss eineindeutig auf dem MQTT-Broker sein!
#define MQTT_KEEPALIVE 90
#define MQTT_SOCKETTIMEOUT 30
#define MQTT_SERIAL_PUBLISH_STATUS "SmartHome/Garage/ESP32_Garagentorsteuerung/status"
#define MQTT_SERIAL_RECEIVER_COMMAND "SmartHome/Garage/ESP32_Garagentorsteuerung/command"
#define MQTT_SERIAL_PUBLISH_DS18B20 "SmartHome/Garage/ESP32_Garagentorsteuerung/Temperatur/"
#define MQTT_SERIAL_PUBLISH_STATE "SmartHome/Garage/ESP32_Garagentorsteuerung/state/"
#define MQTT_SERIAL_PUBLISH_CONFIG "SmartHome/Garage/ESP32_Garagentorsteuerung/config/"
#define MQTT_SERIAL_PUBLISH_BASIS "SmartHome/Garage/ESP32_Garagentorsteuerung/"
String mqttTopic;
String mqttJson;
String mqttPayload;
DeviceAddress myDS18B20Address;
String Adresse;
unsigned long MQTTReconnect = 0;
PubSubClient mqttClient(myWiFiClient);

// Zeitsteuerung für Tor auf und Tor zu
float timeTorAuf = 25.0;            // Gesamtzeit zum Öffnen des Garagentors [s]
float timeTorZu = 21.0;             // Gesamtzeit zum Schließen des Garagentors [s]
float timeHysterese = 1.0;          // Zeithysterese bei einer Auf-Auf-, Zu-Zu- oder schnellen Auf-Zu-Schaltung [s]
int volatile zustand = 0;           // aktuell gewünschte Toröffnung [0 = zu; 100 = auf; x -> Prozent Toröffnung über]
bool volatile positionZu = 0;       // Reed-Sensor 1 ist aktiv und Tor ist geschlossen
bool volatile positionAuf = 0;      // Reed-Sensor 4 ist aktiv und Tor ist vollständig offen
bool volatile position_2 = 0;       // Reed-Sensor 2 [optional] als Zwischenpositionsindikator (reine MQTT-Meldung)
bool volatile position_3 = 0;       // Reed-Sensor 3 [optional] als Zwischenpositionsindikator (reine MQTT-Meldung)

//Zustände
bool volatile torAuf = 0;           // Status der StateMaschine Zustand "TorAuf" - im Endanschlag - nächster Trigger schließt das Tor
bool volatile torZu = 0;            // Status der StateMaschine Zustand "TorZu" - im Endanschlag - nächster Trigger öffnet das Tor
bool volatile torUnDef = 0;         // Status der StateMaschine Zustand "TorUndefiniert" = irgendwas zwischen "Auf" und "Zu" & Fahrtrichtung bei Trigger ungekannt

// Warnstufen der angeschlossenen DS18B20 - Sensoren
float warnHighLevel = 30.0;         //obere Temperatur-Warnstufe bei Überschreitung
float warnLowLevel_1 = 5.0;         //untere Temperaturschwelle
float warnLowLevel_2 = 0.0;         //minimale Temperaturschwelle
bool volatile alertHighLevel = 0;   // Alarmschwelle HighLevel überschritten
bool volatile alertLowLevel_1 = 0;  // Alarmschwelle LowLevel_1 unterschritten
bool volatile alertLowLevel_2 = 0;  // Alarmschwelle LowLevel_1 unterschritten
 
//Initialisiere OneWire und Thermosensor(en)
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature myDS18B20(&oneWire);
int DS18B20_Count = 0;   //Anzahl der angeschlossenen Tempsensoren [0...n]

//Mutexdefinitionen
static SemaphoreHandle_t mutexTemp;
static SemaphoreHandle_t mutexStatus;
static SemaphoreHandle_t mutexTor;
static SemaphoreHandle_t mutexSensor;

//TaskRefrechTime
#define MQTTStateRefresh 20000         // Alle 20.000 Ticks = 20sec

//TaskHandler zur Verwendung mit ESP watchdog
static TaskHandle_t hgetTempFromSensor;
static TaskHandle_t hMQTTstate;
static TaskHandle_t hstateMaschine;

//Queue-Definition für Stromüberwachung
#define QUEUEDEPTH 10                 // Tiefe der Queue - 10 Schaltvorgänge einer Phase für 5s
#define QUEUEMAXWAITTIME 3            // Wartezeit für das Senden in eine Queue - danach Error!
typedef struct {                      // Struktur der Queue-Daten des Zustandes
  TickType_t ticktime;                // gewünchte TickTime der Phasenprüfung
  int zustand;                        // geschalteter Zustand 0-100%
} s_queueData;
bool relaisTrigger = 0;               // Queue-Triggervariabler zur Steuerung des Relais und dessen Pausenzeiten
static QueueHandle_t torZustand;      // Queue-Handler für den gewünschten Torzustand (offen / geschlossen)
static QueueHandle_t qRelais;         // Queue-Handler für den Schaltzustand des Relais - Steuerung der Zeit für Bewegungen


//-------------------------------------
// Callback für MQTT
void mqttCallback(char* topic, byte* message, unsigned int length) {
  BaseType_t rc;
  String str;
  unsigned long mqttValue;
  String mqttMessage;
  String mqttTopicAC;
  byte tx_ac = 1;
  
  for (int i = 0; i < length; i++) {
    str += (char)message[i];
  }
  if (debug > 1) {
    Serial.print("Nachricht aus dem Topic: ");
    Serial.print(topic);
    Serial.print(". Nachricht: ");
    Serial.println(str);
  }
  //Test-Botschaften  
  mqttTopicAC = MQTT_SERIAL_PUBLISH_BASIS;
  mqttTopicAC += "ac";
  if (str.startsWith("Test")) {
    if (debug) Serial.println("Test -> Test OK");
    mqttClient.publish(mqttTopicAC.c_str(), "Test OK");
    tx_ac = 0;
  }

  //Mutex holen
  rc = xSemaphoreTake(mutexStatus, portMAX_DELAY);
  assert(rc == pdPASS);

  //debug-Modfikation  
  if ((tx_ac) && (str.startsWith("debug=0"))) {
    debug = 0;
    mqttClient.publish(mqttTopicAC.c_str(), "debug=0 umgesetzt");
    tx_ac = 0;
  }
  if ((tx_ac) && (str.startsWith("debug=1"))) {
    debug = 1;
    mqttClient.publish(mqttTopicAC.c_str(), "debug=1 umgesetzt");
    tx_ac = 0;
  }
  if ((tx_ac) && (str.startsWith("debug=2"))) {
    debug = 2;
    mqttClient.publish(mqttTopicAC.c_str(), "debug=2 umgesetzt");
    tx_ac = 0;
  }
  if ((tx_ac) && (str.startsWith("debug=3"))) {
    debug = 3;
    mqttClient.publish(mqttTopicAC.c_str(), "debug=3 umgesetzt");
    tx_ac = 0;
  }
  //ErrorLED aus
  if ((tx_ac) && (str.startsWith("ErrorLED aus"))) {
    mqttMessage = "ErrorLED ausgeschaltet";
    digitalWrite(LED_ERROR, LOW);
    if (debug > 2) Serial.println(mqttMessage);
    mqttClient.publish(mqttTopicAC.c_str(), mqttMessage.c_str());
    tx_ac = 0;
  }
  if ((tx_ac) && (str.startsWith("Tortrigger"))) {
    mqttMessage = "Tortrigger ausgeführt";
    bool qData = true;
    BaseType_t rc;
    rc = xQueueSendToBack(qRelais, &qData, portMAX_DELAY);
    if (debug > 2) Serial.println(mqttMessage);
    mqttClient.publish(mqttTopicAC.c_str(), mqttMessage.c_str());
    tx_ac = 0;
  }
  if ((tx_ac) && (str.startsWith("Tortrigger2"))) {
    mqttMessage = "Tortrigger ausgeführt";
    bool qData = true;
    BaseType_t rc;
    rc = xQueueSendToBack(qRelais, &qData, portMAX_DELAY);
    if (debug > 2) Serial.println(mqttMessage);
    mqttClient.publish(mqttTopicAC.c_str(), mqttMessage.c_str());
    tx_ac = 0;
  }
  if ((tx_ac) && (str.startsWith("state="))) {
    BaseType_t rc;
    s_queueData qData;
    str.remove(0,6);
    qData.zustand=str.toInt();
    qData.ticktime=xTaskGetTickCount();
    rc = xQueueSendToBack(torZustand, &qData, portMAX_DELAY);
    mqttMessage = "state=";
    mqttMessage += String(qData.zustand);
    mqttMessage += " umgesetzt";
    if (debug > 2) Serial.println(mqttMessage);
    mqttClient.publish(mqttTopicAC.c_str(), mqttMessage.c_str());
    tx_ac = 0;
  }
  if ((tx_ac) && (str.startsWith("restart"))) {
    mqttClient.publish(mqttTopicAC.c_str(), "reboot in einer Sekunde!");
    if (debug) Serial.println("für Restart: alles aus & restart in 1s!");
    vTaskDelay(1000);
    if (debug) Serial.println("führe Restart aus!");
    ESP.restart();
  }
  //Free Mutex
  rc = xSemaphoreGive(mutexStatus);
  assert(rc == pdPASS);
}

//-------------------------------------
//Subfunktionen für MQTT-Status-Task
// MQTT DS18B20 Status senden
void printDS18B20MQTT() {
  int i;
  for (i = 0; i < DS18B20_Count; i++) {
    //MQTT-Botschaften
    //JSON        
    myDS18B20.getAddress(myDS18B20Address,i);
    Adresse="";
    for (uint8_t j = 0; j < 8; j++)
    {
      Adresse += "0x";
      if (myDS18B20Address[j] < 0x10) Adresse += "0";
      Adresse += String(myDS18B20Address[j], HEX);
      if (j < 7) Adresse += ", ";
    }
    mqttTopic = MQTT_SERIAL_PUBLISH_DS18B20 + String(i) + "/JSON"; 
    mqttJson = "{\"ID\":\"" + String(i) + "\"";
    mqttJson += ",\"Temperatur\":\"" + String(myDS18B20.getTempCByIndex(i)) + "\"";
    mqttJson += ",\"Adresse\":\"(" + Adresse + ")\"}";
    if (debug > 2) Serial.println("MQTT_JSON: " + mqttJson);
    mqttClient.publish(mqttTopic.c_str(), mqttJson.c_str());
    //Temperatur
    mqttTopic = MQTT_SERIAL_PUBLISH_DS18B20 + String(i) + "/Temperatur";
    mqttPayload = String(myDS18B20.getTempCByIndex(i));
    mqttClient.publish(mqttTopic.c_str(), mqttPayload.c_str());
    if (debug > 2) Serial.print("MQTT ID: ");
    if (debug > 2) Serial.println(mqttPayload);
    //ID
    mqttTopic = MQTT_SERIAL_PUBLISH_DS18B20 + String(i) + "/ID";
    mqttPayload = String(i);
    mqttClient.publish(mqttTopic.c_str(), mqttPayload.c_str());
    if (debug > 2) Serial.print("MQTT Temperatur: ");
    if (debug > 2) Serial.println(mqttPayload);
    //Adresse
    mqttTopic = MQTT_SERIAL_PUBLISH_DS18B20 + String(i) + "/Adresse";
    mqttClient.publish(mqttTopic.c_str(), Adresse.c_str());
    if (debug > 2) Serial.print("MQTT Adresse: ");
    if (debug > 2) Serial.println(Adresse);
  }
}

// MQTT Status Betrieb senden
void printStateMQTT() {
  mqttTopic = MQTT_SERIAL_PUBLISH_STATE;
  mqttTopic += "JSON_0";
  mqttJson = "{\"lastError\":\"" + String(lastError) + "\"";
  mqttJson += ",\"WiFi_Signal_Strength\":\"" + String(WiFi.RSSI()) + "\"";
  mqttJson += ",\"Torzustand_Prozent\":\"" + String(zustand) + " %" + "\"";
  mqttJson += ",\"Zustand\":\"";
  if (torAuf == 1) mqttJson += "Tor offen\"}";
  if (torZu == 1) mqttJson += "Tor geschlossen\"}";
  if (torUnDef == 1) mqttJson += "Torzustand unklar\"}";
  if (debug > 2) Serial.println("MQTT_JSON: " + mqttJson);
  mqttClient.publish(mqttTopic.c_str(), mqttJson.c_str());
  mqttTopic = MQTT_SERIAL_PUBLISH_STATE;
  mqttTopic += "JSON_1";
  mqttJson = "{\"SensorPositionZu\":\"" + String(positionZu) + "\"";
  mqttJson += ",\"SensorPositionAuf\":\"" + String(positionAuf) + "\"";
  mqttJson += ",\"SensorPosition2\":\"" + String(position_2) + "\"";
  mqttJson += ",\"SensorPosition3\":\"" + String(position_3) + "\"}";
  if (debug > 2) Serial.println("MQTT_JSON: " + mqttJson);
  mqttClient.publish(mqttTopic.c_str(), mqttJson.c_str());
  mqttTopic = MQTT_SERIAL_PUBLISH_STATE;
  mqttTopic += "JSON_2";
  mqttJson = "{\"alertHighLevel\":\"" + String(alertHighLevel) + "\"";
  mqttJson += ",\"alertLowLevel_1\":\"" + String(alertLowLevel_1) + "\"";
  mqttJson += ",\"alertLowLevel_2\":\"" + String(alertLowLevel_2) + "\"";
  mqttJson += ",\"tempError\":\"" + String(tempError) + "\"}";
  if (debug > 2) Serial.println("MQTT_JSON: " + mqttJson);
  mqttClient.publish(mqttTopic.c_str(), mqttJson.c_str());
  //lastError
  mqttTopic = MQTT_SERIAL_PUBLISH_STATE;
  mqttTopic += "lastError";
  mqttPayload = lastError;
  mqttClient.publish(mqttTopic.c_str(), mqttPayload.c_str());
  if (debug > 2) Serial.print("MQTT phase3error: ");
  if (debug > 2) Serial.println(mqttPayload);
  //WiFi Signalstärke
  mqttTopic = MQTT_SERIAL_PUBLISH_STATE;
  mqttTopic += "WiFi_Signal_Strength";
  mqttPayload = WiFi.RSSI();
  mqttClient.publish(mqttTopic.c_str(), mqttPayload.c_str());
  if (debug > 2) Serial.print("WiFi Signalstärke: ");
  if (debug > 2) Serial.println(mqttPayload);
  //Tor Zustand in Prozent
  mqttTopic = MQTT_SERIAL_PUBLISH_STATE;
  mqttTopic += "Torzustand_Prozent";
  mqttPayload = String(zustand) + " %";
  mqttClient.publish(mqttTopic.c_str(), mqttPayload.c_str());
  if (debug > 2) Serial.print("Torzustand [%]: ");
  if (debug > 2) Serial.println(mqttPayload);
  //Torstatus
  mqttTopic = MQTT_SERIAL_PUBLISH_STATE;
  mqttTopic += "Zustand";
  if (torAuf == 1) mqttPayload = "Tor offen";
  if (torZu == 1) mqttPayload = "Tor geschlossen";
  if (torUnDef == 1) mqttPayload = "Torzustand unklar";
  mqttPayload = 
  mqttClient.publish(mqttTopic.c_str(), mqttPayload.c_str());
  if (debug > 2) Serial.print("Zustand: ");
  if (debug > 2) Serial.println(mqttPayload);
  //Sensor Status "Zu"
  mqttTopic = MQTT_SERIAL_PUBLISH_STATE;
  mqttTopic += "SensorPositionZu";
  mqttPayload = positionZu;
  mqttClient.publish(mqttTopic.c_str(), mqttPayload.c_str());
  if (debug > 2) Serial.print("Sensor Position_Zu: ");
  if (debug > 2) Serial.println(mqttPayload);
  //Sensor Status "Auf"
  mqttTopic = MQTT_SERIAL_PUBLISH_STATE;
  mqttTopic += "SensorPositionAuf";
  mqttPayload = positionAuf;
  mqttClient.publish(mqttTopic.c_str(), mqttPayload.c_str());
  if (debug > 2) Serial.print("Sensor Position_Auf: ");
  if (debug > 2) Serial.println(mqttPayload);
  //Sensor Status "2"
  mqttTopic = MQTT_SERIAL_PUBLISH_STATE;
  mqttTopic += "SensorPosition2";
  mqttPayload = position_2;
  mqttClient.publish(mqttTopic.c_str(), mqttPayload.c_str());
  if (debug > 2) Serial.print("Sensor Position_2: ");
  if (debug > 2) Serial.println(mqttPayload);
  //Sensor Status "3"
  mqttTopic = MQTT_SERIAL_PUBLISH_STATE;
  mqttTopic += "SensorPosition3";
  mqttPayload = position_3;
  mqttClient.publish(mqttTopic.c_str(), mqttPayload.c_str());
  if (debug > 2) Serial.print("Sensor Position_3: ");
  if (debug > 2) Serial.println(mqttPayload);
  //alertHighLevel
  mqttTopic = MQTT_SERIAL_PUBLISH_STATE;
  mqttTopic += "alertHighLevel";
  mqttPayload = alertHighLevel;
  mqttClient.publish(mqttTopic.c_str(), mqttPayload.c_str());
  if (debug > 2) Serial.print("alertHighLevel: ");
  if (debug > 2) Serial.println(mqttPayload);
  //alertLowLevel_1
  mqttTopic = MQTT_SERIAL_PUBLISH_STATE;
  mqttTopic += "alertLowLevel_1";
  mqttPayload = alertLowLevel_1;
  mqttClient.publish(mqttTopic.c_str(), mqttPayload.c_str());
  if (debug > 2) Serial.print("alertLowLevel_1: ");
  if (debug > 2) Serial.println(mqttPayload);
  //alertLowLevel_2
  mqttTopic = MQTT_SERIAL_PUBLISH_STATE;
  mqttTopic += "alertLowLevel_2";
  mqttPayload = alertLowLevel_2;
  mqttClient.publish(mqttTopic.c_str(), mqttPayload.c_str());
  if (debug > 2) Serial.print("alertLowLevel_2: ");
  if (debug > 2) Serial.println(mqttPayload);
  //tempError
  mqttTopic = MQTT_SERIAL_PUBLISH_STATE;
  mqttTopic += "tempError";
  mqttPayload = tempError;
  mqttClient.publish(mqttTopic.c_str(), mqttPayload.c_str());
  if (debug > 2) Serial.print("tempError: ");
  if (debug > 2) Serial.println(mqttPayload);
}
// MQTT Config und Parameter senden
void printConfigMQTT() {
  //Teil 1
  mqttTopic = MQTT_SERIAL_PUBLISH_CONFIG;
  mqttTopic += "JSON_0";
  mqttJson = "{\"timeTorAuf\":\"" + String(timeTorAuf) + "\"";
  mqttJson += ",\"timeTorZu\":\"" + String(timeTorZu) + "\"";
  mqttJson += ",\"timeHysterese\":\"" + String(timeHysterese) + "\"";
  mqttJson += ",\"warnHighLevel\":\"" + String(warnHighLevel) + "\"";
  mqttJson += ",\"warnLowLevel_1\":\"" + String(warnLowLevel_1) + "\"";
  mqttJson += ",\"warnLowLevel_2\":\"" + String(warnLowLevel_2) + "\"";
  mqttJson += ",\"debug\":\"" + String(debug) + "\"}";
  if (debug > 2) Serial.println("MQTT_JSON: " + mqttJson);
  mqttClient.publish(mqttTopic.c_str(), mqttJson.c_str());
  //timeTorAuf
  mqttTopic = MQTT_SERIAL_PUBLISH_CONFIG;
  mqttTopic += "timeTorAuf";
  mqttPayload = timeTorAuf;
  mqttClient.publish(mqttTopic.c_str(), mqttPayload.c_str());
  if (debug > 2) Serial.print("timeTorAuf: ");
  if (debug > 2) Serial.println(mqttPayload);
  //timeTorZu
  mqttTopic = MQTT_SERIAL_PUBLISH_CONFIG;
  mqttTopic += "timeTorZu";
  mqttPayload = timeTorZu;
  mqttClient.publish(mqttTopic.c_str(), mqttPayload.c_str());
  if (debug > 2) Serial.print("timeTorZu: ");
  if (debug > 2) Serial.println(mqttPayload);
  //timeHysterese
  mqttTopic = MQTT_SERIAL_PUBLISH_CONFIG;
  mqttTopic += "timeHysterese";
  mqttPayload = timeHysterese;
  mqttClient.publish(mqttTopic.c_str(), mqttPayload.c_str());
  if (debug > 2) Serial.print("timeHysterese: ");
  if (debug > 2) Serial.println(mqttPayload);
  //warnHighLevel
  mqttTopic = MQTT_SERIAL_PUBLISH_CONFIG;
  mqttTopic += "warnHighLevel";
  mqttPayload = warnHighLevel;
  mqttClient.publish(mqttTopic.c_str(), mqttPayload.c_str());
  if (debug > 2) Serial.print("warnHighLevel: ");
  if (debug > 2) Serial.println(mqttPayload);
  //warnLowLevel_1
  mqttTopic = MQTT_SERIAL_PUBLISH_CONFIG;
  mqttTopic += "warnLowLevel_1";
  mqttPayload = warnLowLevel_1;
  mqttClient.publish(mqttTopic.c_str(), mqttPayload.c_str());
  if (debug > 2) Serial.print("warnLowLevel_1: ");
  if (debug > 2) Serial.println(mqttPayload);
  //warnLowLevel_2
  mqttTopic = MQTT_SERIAL_PUBLISH_CONFIG;
  mqttTopic += "warnLowLevel_2";
  mqttPayload = warnLowLevel_2;
  mqttClient.publish(mqttTopic.c_str(), mqttPayload.c_str());
  if (debug > 2) Serial.print("warnLowLevel_2: ");
  if (debug > 2) Serial.println(mqttPayload);
  //debug
  mqttTopic = MQTT_SERIAL_PUBLISH_CONFIG;
  mqttTopic += "debug";
  mqttPayload = debug;
  mqttClient.publish(mqttTopic.c_str(), mqttPayload.c_str());
  if (debug > 2) Serial.print("debug: ");
  if (debug > 2) Serial.println(mqttPayload);
}
//LED-Blik-OK
void LEDblinkMsg(){
  digitalWrite(LED_MSG, HIGH);
  delay(150);
  digitalWrite(LED_MSG, LOW);
}
//-------------------------------------
//MQTT-Status-Task
static void MQTTstate (void *args){
  BaseType_t rc;
  esp_err_t er;
  TickType_t ticktime;

  //ticktime initialisieren
  ticktime = xTaskGetTickCount();

  for (;;){                        // Dauerschleife des Tasks
    // Watchdog zurücksetzen
    esp_task_wdt_reset();
    //Lesen der Temperaturen
    if (debug > 1) Serial.print("TickTime: ");
    if (debug > 1) Serial.print(ticktime);
    if (debug > 1) Serial.println(" | MQTT-Status-Task gestartet");
    if (mqttClient.connected()) {
      if (tempError == 0){
        rc = xSemaphoreTake(mutexTemp, portMAX_DELAY);
        assert(rc == pdPASS);
          printDS18B20MQTT();
        rc = xSemaphoreGive(mutexTemp);
        assert(rc == pdPASS);
      }
      rc = xSemaphoreTake(mutexStatus, portMAX_DELAY);
      assert(rc == pdPASS);
        printStateMQTT();
      rc = xSemaphoreGive(mutexStatus);
      assert(rc == pdPASS);
      printConfigMQTT();
    }
    // Task schlafen legen - restart MQTTStateRefresh ticks
    LEDblinkMsg();
    vTaskDelayUntil(&ticktime, MQTTStateRefresh);
  }
}

//-------------------------------------
//Subfunktionen für MQTTwatchdog-Task
// MQTT Verbindung herstellen (wird auch von setup verwendet!)
void mqttConnect () {
  int i = 0;
  Serial.print("Verbindungsaubfau zu MQTT Server ");
  Serial.print(MQTT_SERVER);
  Serial.print(" Port ");  
  Serial.print(MQTT_PORT);
  Serial.print(" wird aufgebaut ");  
  while (!mqttClient.connected()) {
    Serial.print(".");
	i++;
    if (mqttClient.connect(MQTT_CLIENTID, MQTT_USER, MQTT_PASSWORD, MQTT_SERIAL_PUBLISH_STATUS, 0, true, "false")) {
      mqttClient.publish(MQTT_SERIAL_PUBLISH_STATUS, "true", true);
      Serial.println("");
      Serial.print("MQTT verbunden!");
    } 
    else {
      if (++i > 20) {
        Serial.println("MQTT scheint nicht mehr erreichbar! Reboot!!");
        ESP.restart();
      }
      Serial.print("fehlgeschlagen rc=");
      Serial.print(mqttClient.state());
      Serial.println(" erneuter Versuch in 5 Sekunden.");
      delay(5000);      
    }    
  }
  mqttClient.subscribe(MQTT_SERIAL_RECEIVER_COMMAND);
}
// MQTT Verbindungsprüfung 
void checkMQTTconnetion() {
  BaseType_t rc;
  if (!mqttClient.connected()) {
    if (debug) Serial.println("MQTT Server Verbindung verloren...");
    if (debug) Serial.print("Disconnect Errorcode: ");
    if (debug) Serial.println(mqttClient.state());  
    //Vorbereitung errorcode MQTT (https://pubsubclient.knolleary.net/api#state)
    mqttTopic = MQTT_SERIAL_PUBLISH_BASIS + String("error");
    mqttPayload = String(String(++MQTTReconnect) + ". reconnect: ") + String("; MQTT disconnect rc=" + String(mqttClient.state()));
    //reconnect
    mqttConnect();
    //sende Fehlerstatus
    mqttClient.publish(mqttTopic.c_str(), mqttPayload.c_str());
  }
  mqttClient.loop();
}
//-------------------------------------
//MQTT-MQTTwatchdog-Task
static void MQTTwatchdog (void *args){
  BaseType_t rc;
  TickType_t ticktime;

  //ticktime initialisieren
  ticktime = xTaskGetTickCount();

  for (;;){                        // Dauerschleife des Tasks
    //Lesen der Temperaturen
    if (debug > 1) Serial.print("TickTime: ");
    if (debug > 1) Serial.print(ticktime);
    if (debug > 1) Serial.println(" | MQTTonlinePrüf-Task gestartet");
    checkMQTTconnetion();

    // Task schlafen legen - restart alle 2s = 2*1000 ticks = 2000 ticks
    // mit mqttClient.loop() wird auch der MQTTcallback ausgeführt!
    vTaskDelayUntil(&ticktime, 2000);
  }
}

//-------------------------------------
//Subfunktionen für den TempSensor-Task
// Temperatursensoren auslesen
void readDS18B20() {
  if (debug > 2) Serial.print("Anfrage der Temperatursensoren... ");
  myDS18B20.requestTemperatures();  //Anfrage zum Auslesen der Temperaturen
  if (debug > 2) Serial.println("fertig");
  for (int i = 0; i < DS18B20_Count; i++) {
    myDS18B20.getAddress(myDS18B20Address,i);
    Adresse="";
    for (uint8_t j = 0; j < 8; j++)
    {
      Adresse += "0x";
      if (myDS18B20Address[j] < 0x10) Adresse += "0";
      Adresse += String(myDS18B20Address[j], HEX);
      if (j < 7) Adresse += ", ";
    }
  }
}
//Debug-Ausgabe der Temp-Sensorwerte
void printDS18B20() {
  if (debug > 2) {
    for (int i = 0; i < DS18B20_Count; i++) {
      //print to Serial
      Serial.print("DS18B20[");
      Serial.print(i);
      Serial.print("]: ");
      Serial.print(myDS18B20.getTempCByIndex(i));
      Serial.print(" *C (");
      myDS18B20.getAddress(myDS18B20Address,i);
      Adresse="";
      for (uint8_t j = 0; j < 8; j++)
      {
        Adresse += "0x";
        if (myDS18B20Address[j] < 0x10) Adresse += "0";
        Adresse += String(myDS18B20Address[j], HEX);
        if (j < 7) Adresse += ", ";
      }
      Serial.println(Adresse + ")");
    }
  }
}
//prüfe auf Über- oder Unterschreitung der TempLimits
void tempCheck(){
  float temp = 0;
  alertHighLevel = 0;
  alertLowLevel_1 = 0;
  alertLowLevel_2 = 0;
  for (int i = 0; i < DS18B20_Count; i++) {
    temp = myDS18B20.getTempCByIndex(i);
    if (temp > warnHighLevel) alertHighLevel = 1;
    if (temp < warnLowLevel_1) alertLowLevel_1 = 1;
    if (temp < warnLowLevel_2) alertLowLevel_2 = 1;
  }
}
//-------------------------------------
//Task zur Ermittlung der Temperaturen
static void getTempFromSensor (void *args){
  BaseType_t rc;
  esp_err_t er;
  TickType_t ticktime;

  //ticktime initialisieren
  ticktime = xTaskGetTickCount();

  for (;;){                        // Dauerschleife des Tasks
    // Watchdog zurücksetzen
    esp_task_wdt_reset();
    //Lesen der Temperaturen
    if (debug > 1) Serial.print("TickTime: ");
    if (debug > 1) Serial.print(ticktime);
    if (debug > 1) Serial.println(" | TempSensor-Task liest DS18B20-Sensoren aus");
    rc = xSemaphoreTake(mutexTemp, portMAX_DELAY);
    assert(rc == pdPASS);
      readDS18B20();                // Sensoren auslesen und den Variablen zuordnen
      printDS18B20();               // DebugInfo auf Serial (thermale Infos)
      tempCheck();                  // Prüfung, ob Limitg gerissen wurden
    rc = xSemaphoreGive(mutexTemp);
    assert(rc == pdPASS);

    // Task schlafen legen - restart alle 20s = 20*1000 ticks = 20000 ticks
    vTaskDelayUntil(&ticktime, 20000);
  }
}

//-------------------------------------
//Subfunktionen für den Tortrigger-Task
void triggerDoor(){
  BaseType_t rc;
  if (debug) Serial.println("führe Tortrigger aus.");
  //einschalten
  rc = xSemaphoreTake(mutexStatus, portMAX_DELAY);
  assert(rc == pdPASS);
    digitalWrite(RELAIS, LOW);
  rc = xSemaphoreGive(mutexStatus);
  assert(rc == pdPASS);
  //Wartezeit
  delay(TRIGGERTIME);
  //ausschalten
  rc = xSemaphoreTake(mutexStatus, portMAX_DELAY);
  assert(rc == pdPASS);
    digitalWrite(RELAIS, HIGH);
  rc = xSemaphoreGive(mutexStatus);
  assert(rc == pdPASS);
}
//-------------------------------------
//Tor-Trigger-Task
static void doorMotion (void *args){
  BaseType_t rc;
  TickType_t ticktime;
  bool motionTrigger;

  //ticktime initialisieren
  ticktime = xTaskGetTickCount();
  
  for (;;){                        // Dauerschleife des Tasks
    //Daten aus der Queue abfragen
    rc = xQueueReceive(qRelais, &motionTrigger, portMAX_DELAY); 
    //Tor-Triigger wird ausgeführt
    triggerDoor();
    //weitere, anstehende Trigger werden verworfen
    rc = xQueueReset(qRelais);

    // Task schlafen legen - restart MQTTStateRefresh ticks
    LEDblinkMsg();
    vTaskDelayUntil(&ticktime, 500); // check alle 500ms
  }
}

//-------------------------------------
//Subfunktionen für den Zustand-Task
//auslesen der Zustandsvariablen
bool readSensor(int type){
  //entprellen über MC14490P als ausreichend angesehen!
  bool sensor;
  BaseType_t rc;
  //Auslesen der Sensoren
  rc = xSemaphoreTake(mutexSensor, portMAX_DELAY);
  assert(rc == pdPASS);
    if (type == 1){
      //Position "Zu"
      sensor = digitalRead(REED1);
    }
    if (type == 2){
      //Position "2"
      sensor = digitalRead(REED2);
    }
    if (type == 3){
      //Position "3"
      sensor = digitalRead(REED3);
    }
    if (type == 4){
      //Position "Auf"
      sensor = digitalRead(REED4);
    }
  rc = xSemaphoreGive(mutexSensor);
  assert(rc == pdPASS);
  if (SENSORINVERT == 1) sensor ^= 1;
  if (debug > 1) Serial.print("SensorID ");
  if (debug > 1) Serial.print(type);
  if (debug > 1) Serial.print(": Value = ");
  if (debug > 1) Serial.println(sensor);
  return sensor;
}
//setzen der Sensorvariablen
void stateValue(){
  BaseType_t rc;
  bool posZu;
  bool pos2;
  bool pos3;
  bool posAuf;
  //Werte auslesen
  posZu = readSensor(1);
  pos2 = readSensor(2);
  pos3 = readSensor(3);
  posAuf = readSensor(4);
  //Schreiben der Sensorwerte  
  rc = xSemaphoreTake(mutexStatus, portMAX_DELAY);
  assert(rc == pdPASS);
    positionZu = posZu;
    position_2 = pos2;
    position_3 = pos3;
    positionAuf = posAuf;
  rc = xSemaphoreGive(mutexStatus);
  assert(rc == pdPASS);
}
//-------------------------------------
//Task StateMaschine
static void stateMaschine(void *args){
  BaseType_t rc;
  esp_err_t er;
  TickType_t ticktime;
  bool posZu;
  bool pos2;
  bool pos3;
  bool posAuf;
  bool stateAuf;
  bool stateZu;
  bool stateUnDef;
  int aktZustand;

  //ticktime initialisieren
  ticktime = xTaskGetTickCount();

  for (;;){                        // Dauerschleife des Tasks
    // Watchdog zurücksetzen
    esp_task_wdt_reset();
    //Startzeit ausgeben
    if (debug > 1) Serial.print("TickTime: ");
    if (debug > 1) Serial.print(ticktime);
    if (debug > 1) Serial.println(" | StateMaschine-Task gestartet");
    //lese den Sensorstatus aus
    stateValue();
    rc = xSemaphoreTake(mutexStatus, portMAX_DELAY);
    assert(rc == pdPASS);
      posZu = positionZu;
      pos2 = position_2;
      pos3 = position_3;
      posAuf = positionAuf;
    rc = xSemaphoreGive(mutexStatus);
    assert(rc == pdPASS);
    //Zustände der Statemaschine bestimmen
    //Zustand undefiniert, falls keinen anderer Zustand zutrifft
    stateUnDef = 1;
    stateAuf = 0;
    stateZu = 0;
    if (posZu == 1){
      stateAuf = 0;
      stateZu = 1;
      stateUnDef = 0;
      aktZustand = 0;
    } 
    if (posAuf == 1){
      stateAuf = 1;
      stateZu = 0;
      stateUnDef = 0;
      aktZustand = 100;
    } 
    if ((posZu == 1) && (posAuf == 1)) {
      lastError="Zustandsfehler! Tor kann nicht Auf und Zu gleichzeitig sein - bitte Sensoren prüfen!";
      stateUnDef = 1;
      stateAuf = 0;
      stateZu = 0;
    }
    rc = xSemaphoreTake(mutexStatus, portMAX_DELAY);
    assert(rc == pdPASS);
      torUnDef = stateUnDef;
      torAuf = stateAuf;
      torZu = stateZu;
      if ((posZu == 1) || (posAuf == 1)) zustand = aktZustand;
    rc = xSemaphoreGive(mutexStatus);
    assert(rc == pdPASS);

    if (debug > 1) Serial.print("Tor-Zustand Undefiniert = ");
    if (debug > 1) Serial.println(stateUnDef);
    if (debug > 1) Serial.print("Tor-Zustand Auf = ");
    if (debug > 1) Serial.println(stateAuf);
    if (debug > 1) Serial.print("Tor-Zustand Zu = ");
    if (debug > 1) Serial.println(stateZu);

    // Task schlafen legen - restart alle 1s = 1*1000 ticks = 1000 ticks
    vTaskDelayUntil(&ticktime, 1000);
  }
}

//-------------------------------------
//Task Torhöhenschaltung
static void window(void *args){
  BaseType_t rc;
  TickType_t ticktime;
  bool posZu;
  bool pos2;
  bool pos3;
  bool posAuf;
  s_queueData qDataTor;
  bool qData = true;
  int aktZustand;
  bool running;
  int i;
  float waitTime;

  //ticktime initialisieren
  ticktime = xTaskGetTickCount();
  
  //Startzeit ausgeben
  if (debug > 1) Serial.print("TickTime: ");
  if (debug > 1) Serial.print(ticktime);
  if (debug > 1) Serial.println(" | Windows-Task gestartet");

  for (;;){                        // Dauerschleife des Tasks
    //warte bis Anfrage kommt
    rc = xQueueReceive(torZustand, &qDataTor, portMAX_DELAY); 
    assert(rc == pdPASS);
    if (debug > 1) Serial.print("von Queue empfangene TickTime: ");
    if (debug > 1) Serial.println(qDataTor.ticktime);
    if (debug > 1) Serial.print("von Queue empfangene Höhe: ");
    if (debug > 1) Serial.println(qDataTor.zustand);
    //Ausführung nur für den letzten Befehl
    if (uxQueueMessagesWaiting (torZustand) == 0) {
      if (debug > 1) Serial.println("letztes Queue entdeckt => starte den Behanghöhe");
      //Prüfe, ob Zustand undefiniert, oder Endanschlöge gegeben
      rc = xSemaphoreTake(mutexStatus, portMAX_DELAY);
      assert(rc == pdPASS);
        posZu = positionZu;
        pos2 = position_2;
        pos3 = position_3;
        posAuf = positionAuf;
        if (timeTorAuf > timeTorZu) {
          waitTime = timeTorAuf*1000;
        } else {
          waitTime = timeTorZu*1000;
        }
      rc = xSemaphoreGive(mutexStatus);
      assert(rc == pdPASS);
      if (debug > 1) Serial.print("PosZu ="); 
      if (debug > 1) Serial.println(posZu);
      if (debug > 1) Serial.print("Pos2 =");
      if (debug > 1) Serial.println(pos2);
      if (debug > 1) Serial.print("Pos3 =");
      if (debug > 1) Serial.println(pos3);
      if (debug > 1) Serial.print("PosAuf =");
      if (debug > 1) Serial.println(posAuf);
      if (debug > 1) Serial.print("waitTime =");
      if (debug > 1) Serial.println(waitTime);
      aktZustand = -1;       //Wert für unklaren Zustand
      if (posZu == 1) {
        aktZustand = 0;      //aktueller Zustand = zu => 0%
      }
      if (posAuf == 1) {
        aktZustand = 100;    //aktueller Zustand = zu => 0%
      }
      if ((posAuf == 1) && (posZu == 1)){
        aktZustand = -1;     //Auf und Zu unmöglich => undefiniert
      }
      if (debug > 1) Serial.print("T1 - aktZustand ="); 
      if (debug > 1) Serial.println(aktZustand);
      if (aktZustand != qDataTor.zustand){
        //Zustand ungleich, damit start nötig
        if (aktZustand < 0){
          if (debug) Serial.println("bringe das Tor in einen Endzustand"); 
          //undefinierter Zustand - zuerst an Endanschlag fahren
          rc = xQueueSendToBack(qRelais, &qData, portMAX_DELAY);
          //ticktime initialisieren
          ticktime = xTaskGetTickCount();
          running = true;
          i = 0;
          while (running){
            if (debug > 1) Serial.println("wann kommt der Endschalter?"); 
            rc = xSemaphoreTake(mutexStatus, portMAX_DELAY);
            assert(rc == pdPASS);
              posZu = positionZu;
              posAuf = positionAuf;
            rc = xSemaphoreGive(mutexStatus);
            assert(rc == pdPASS);
            //Stop bei erreichen einer der Endanschläge
            if (posZu == 1) {
              if (debug > 1) Serial.println("...Zu"); 
              aktZustand = 0;
              running = false;
            }
            if (posAuf == 1) {
              if (debug > 1) Serial.println("...Auf"); 
              aktZustand = 100;
              running = false;
            }
            if ((posZu == 1) && (posAuf == 1)){
              if (debug > 1) Serial.println("...Fehler: beide Endschalter gleichzeitig!"); 
              aktZustand = -1;
            }
            //Stop bei erreichen des TimeOuts
            if (i*500 > (int)(1.5*waitTime)) {
              if (debug > 1) Serial.println("...timeOut"); 
              aktZustand = -1;
              running = false;
            }
            ++i;
            ticktime = xTaskGetTickCount();
            rc = xTaskDelayUntil(&ticktime,500);
          }
        }
        if (debug > 1) Serial.print("T2 - aktZustand ="); 
        if (debug > 1) Serial.println(aktZustand);
        //Tor steht auf Endanschlag, oder ein TimeOut beim Anfahren
        //Anstarten der Tors nur bei Endanschlägen (nue bei einem!)
        rc = xSemaphoreTake(mutexStatus, portMAX_DELAY);
        assert(rc == pdPASS);
          posZu = positionZu;
          posAuf = positionAuf;
        rc = xSemaphoreGive(mutexStatus);
        assert(rc == pdPASS);

        if ((posZu == 1) || (posAuf == 1)) {
          //einer der Endanschläge erreicht => fahrt auf Höhe jetzt möglich.
          ticktime = xTaskGetTickCount();
          //starte das Tor
          rc = xQueueSendToBack(qRelais, &qData, portMAX_DELAY);
          if (posZu == 1) waitTime = timeTorAuf * 1000 * ((float)qDataTor.zustand / 100.0);
          if (posAuf == 1) waitTime = timeTorZu * 1000 * ((float)(100 - qDataTor.zustand) / 100.0);
          if (debug > 1) Serial.print("von Queue empfangene Höhe: ");
          if (debug > 1) Serial.println(qDataTor.zustand);
          if (debug > 1) Serial.print("Zeit für delay =");
          if (debug > 1) Serial.println(waitTime);
          rc = xTaskDelayUntil(&ticktime,(int)waitTime);
          //stoppe das Tor
          rc = xQueueSendToBack(qRelais, &qData, portMAX_DELAY);
          //Zielhöhe näherungsweise erreicht
          aktZustand = qDataTor.zustand;
        }
      }

      if (debug > 1) Serial.print("T3 - aktZustand ="); 
      if (debug > 1) Serial.println(aktZustand);
      rc = xSemaphoreTake(mutexStatus, portMAX_DELAY);
      assert(rc == pdPASS);
        zustand = aktZustand;
      rc = xSemaphoreGive(mutexStatus);
      assert(rc == pdPASS);

      if (debug) Serial.print("angefahrene Torhöhe = ");
      if (debug) Serial.print(zustand);
      if (debug) Serial.println(" %");
    }
  }
}

void setup() {
  //Watchdog starten
  esp_err_t er;
  er = esp_task_wdt_init(300,true);  //restart nach 5min = 300s Inaktivität einer der 4 überwachten Tasks 
  assert(er == ESP_OK); 
  // Initialisierung und Plausibilitaetschecks
  Serial.begin(115200);
  while (!Serial)
  Serial.println("Start Setup");
  // Initialisierung der LED-Outputs
  pinMode(LED_ERROR, OUTPUT);
  digitalWrite(LED_ERROR, LOW);
  pinMode(LED_MSG, OUTPUT);
  digitalWrite(LED_MSG, HIGH);
  pinMode(LED_OK, OUTPUT);
  digitalWrite(LED_OK, HIGH);
  // Initialisierung der REED-Inputs
  pinMode(REED1, INPUT);
  pinMode(REED2, INPUT);
  pinMode(REED3, INPUT);
  pinMode(REED4, INPUT);
  // Initialisierung des Relais-Output
  pinMode(RELAIS, OUTPUT);
  digitalWrite(RELAIS, HIGH);
  //WiFi-Setup
  int i = 0;
  Serial.print(ssid);
  WiFi.setHostname(HOSTNAME);
  WiFi.begin(ssid,password);
  while (WiFi.status() != WL_CONNECTED)
  {
    ++i;
    if (i > 240) {
      // Reboot nach 2min der Fehlversuche
      Serial.println("WLAN scheint nicht mehr erreichbar! Reboot!!");
      ESP.restart();
    }
    delay(500);
    Serial.print(".");    
  }
  Serial.println("");
  Serial.println("WiFi verbunden.");
  Serial.print("IP Adresse: ");
  Serial.print(WiFi.localIP());
  Serial.println("");
  //MQTT-Setup
  Serial.println("MQTT Server Initialisierung laeuft...");
  mqttClient.setServer(MQTT_SERVER,MQTT_PORT); 
  mqttClient.setCallback(mqttCallback);
  mqttClient.setKeepAlive(MQTT_KEEPALIVE);
  mqttClient.setSocketTimeout(MQTT_SOCKETTIMEOUT);
  mqttConnect();
  mqttTopic = MQTT_SERIAL_PUBLISH_BASIS + String("error");
  mqttPayload = String(String(MQTTReconnect) + ".: keine MQTT-Fehler seit Reboot!");
  mqttClient.publish(mqttTopic.c_str(), mqttPayload.c_str());
  Serial.println("");
  //DS18B20-Setup
  Serial.println("Auslesen der DS18B20-Sensoren...");
  myDS18B20.begin();
  Serial.print("Anzahl gefundener 1-Wire-Geraete:  ");
  Serial.println(myDS18B20.getDeviceCount());
  DS18B20_Count = myDS18B20.getDS18Count();
  Serial.print("Anzahl gefundener DS18B20-Geraete: ");
  Serial.println(DS18B20_Count);
  if (DS18B20_Count == 0) {
    Serial.println("... Anzahl DB18B20 = 0!");
    lastError = "keine DB18B20-Sensoren gefunden - Temperaturmessung wird ignoriert!";
    tempError = 1;
    mqttTopic = MQTT_SERIAL_PUBLISH_DS18B20; 
    mqttTopic += "0/JSON"; 
    mqttJson = "{\"ID\":\"0\"";
    mqttJson += ",\"Temperatur\":\"-999\"";
    mqttJson += ",\"Adresse\":\"(0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00)\"}";
    mqttClient.publish(mqttTopic.c_str(), mqttJson.c_str());
    //Temperatur
    mqttTopic = MQTT_SERIAL_PUBLISH_DS18B20;
    mqttTopic += "0/Temperatur";
    mqttPayload = "-999";
    mqttClient.publish(mqttTopic.c_str(), mqttPayload.c_str());
    //ID
    mqttTopic = MQTT_SERIAL_PUBLISH_DS18B20;
    mqttTopic += "0/ID";
    mqttPayload = "0";
    mqttClient.publish(mqttTopic.c_str(), mqttPayload.c_str());
    //Adresse
    mqttTopic = MQTT_SERIAL_PUBLISH_DS18B20;
    mqttTopic += "0/Adresse";
    mqttPayload = "0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00";
    mqttClient.publish(mqttTopic.c_str(), mqttPayload.c_str());
  }
  if (tempError == 0){
    Serial.print("Globale Aufloesung (Bit):        ");
    Serial.println(myDS18B20.getResolution());
  }
  //Mutex-Initialisierung
  mutexTemp = xSemaphoreCreateMutex();
  assert(mutexTemp);
  mutexStatus = xSemaphoreCreateMutex();
  assert(mutexStatus);
  mutexTor = xSemaphoreCreateMutex();
  assert(mutexTor);
  mutexSensor = xSemaphoreCreateMutex();
  assert(mutexTor);
  //Queue einrichten für Torzusand und Relaissteuerung
  torZustand = xQueueCreate(QUEUEDEPTH, sizeof(s_queueData));
  assert(torZustand);
  qRelais = xQueueCreate(QUEUEDEPTH, sizeof(bool));
  assert(qRelais);
  //Höhe des Garagentor auf unbekannt initialisieren
  zustand = -1;
  //Task zur Displayaktualisierung starten
  int app_cpu = xPortGetCoreID();
  BaseType_t rc;
  if (tempError == 0) {
    rc = xTaskCreatePinnedToCore(
      getTempFromSensor,         //Taskroutine
      "getTempSensorTask",       //Taskname
      2048,                      //StackSize
      nullptr,                   //Argumente / Parameter
      2,                         //Priorität
      &hgetTempFromSensor,       //handler
      app_cpu);                  //CPU_ID
    assert(rc == pdPASS);
    Serial.println("TempSensor-Task gestartet.");
    er = esp_task_wdt_status(hgetTempFromSensor);  // Check, ob der Task schon überwacht wird
    assert(er == ESP_ERR_NOT_FOUND);
    if (er == ESP_ERR_NOT_FOUND) {
      er = esp_task_wdt_add(hgetTempFromSensor);   // Task zur Überwachung hinzugefügt  
      assert(er == ESP_OK); 
    }
  }
  rc = xTaskCreatePinnedToCore(
    MQTTwatchdog,              //Taskroutine
    "MQTTwatchdog",            //Taskname
    2048,                      //StackSize
    nullptr,                   //Argumente / Parameter
    1,                         //Priorität
    nullptr,                   //handler
    app_cpu);                  //CPU_ID
  assert(rc == pdPASS);
  Serial.println("MQTT-Watchdog-Task gestartet.");
  rc = xTaskCreatePinnedToCore(
    MQTTstate,                 //Taskroutine
    "MQTTstate",               //Taskname
    2048,                      //StackSize
    nullptr,                   //Argumente / Parameter
    1,                         //Priorität
    &hMQTTstate,               //handler
    app_cpu);                  //CPU_ID
  assert(rc == pdPASS);
  er = esp_task_wdt_status(hMQTTstate);  // Check, ob der Task schon überwacht wird
  assert(er == ESP_ERR_NOT_FOUND);
  if (er == ESP_ERR_NOT_FOUND) {
    er = esp_task_wdt_add(hMQTTstate);   // Task zur Überwachung hinzugefügt  
    assert(er == ESP_OK); 
  }
  Serial.println("MQTT-State-Task gestartet.");
  rc = xTaskCreatePinnedToCore(
    doorMotion,                //Taskroutine
    "doorMotion",              //Taskname
    2048,                      //StackSize
    nullptr,                   //Argumente / Parameter
    2,                         //Priorität
    nullptr,                   //handler
    app_cpu);                  //CPU_ID
  assert(rc == pdPASS);
  Serial.println("doorMotion-Task gestartet.");
  rc = xTaskCreatePinnedToCore(
    stateMaschine,             //Taskroutine
    "stateMaschine",           //Taskname
    2048,                      //StackSize
    nullptr,                   //Argumente / Parameter
    3,                         //Priorität
    &hstateMaschine,           //handler
    app_cpu);                  //CPU_ID
  assert(rc == pdPASS);
  er = esp_task_wdt_status(hstateMaschine);  // Check, ob der Task schon überwacht wird
  assert(er == ESP_ERR_NOT_FOUND);
  if (er == ESP_ERR_NOT_FOUND) {
    er = esp_task_wdt_add(hstateMaschine);   // Task zur Überwachung hinzugefügt  
    assert(er == ESP_OK); 
  }
  Serial.println("stateMaschine-Task gestartet.");
  rc = xTaskCreatePinnedToCore(
    window,                    //Taskroutine
    "window",                  //Taskname
    2048,                      //StackSize
    nullptr,                   //Argumente / Parameter
    2,                         //Priorität
    nullptr,                   //handler
    app_cpu);                  //CPU_ID
  assert(rc == pdPASS);
  Serial.println("window-Task gestartet.");
  //OK-Blinker
  digitalWrite(LED_MSG, LOW);
  delay(500);
  digitalWrite(LED_MSG, HIGH);
  delay(250);
  digitalWrite(LED_MSG, LOW);
  Serial.println("Normalbetrieb gestartet...");
  //Startmeldung via MQTT
  String mqttTopicAC;
  mqttTopicAC = MQTT_SERIAL_PUBLISH_BASIS;
  mqttTopicAC += "ac";
  mqttClient.publish(mqttTopicAC.c_str(), "Start durchgeführt.");
}

void loop() {
  //loop wird als Task nicht gebraucht
  vTaskDelete(nullptr);
}
