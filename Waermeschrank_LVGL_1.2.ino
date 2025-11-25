/*Find (⇅ for history)

 * ============================================================================
 * Wärmeschrank-Steuerung mit LVGL GUI und MQTT
 * ============================================================================
 * 
 * Hardware:
 * - ESP32 DevKit
 * - TFT-Display 1.69" mit ST7789 v2 Treiber (SPI)
 * - Rotary Encoder mit Taster (HW-040)
 * - Temperatursensor DS18B20
 * - Relais für Heizungssteuerung
 * 
 * Funktionen:
 * - Temperaturgeregelte Heizungssteuerung mit Hysterese
 * - Zeitbasierte Laufzeitüberwachung
 * - MQTT-Integration für Remote-Steuerung und Monitoring
 * - WiFi-Konfiguration über Web-Interface
 * - Grafische Benutzeroberfläche mit LVGL
 * 
 * Bedienung:
 * - Kurzer Tastendruck: Wechsel zwischen Temperatur- und Zeiteinstellung
 * - Langer Tastendruck (>3s): Konfigurationsmodus starten
 * - Encoder drehen: Wert ändern (je nach Modus)
 * 
 * MQTT Topics:
 * - [base]/temp_ist        : Aktuelle Temperatur (wird gesendet)
 * - [base]/temp_soll       : Solltemperatur (wird gesendet und empfangen)
 * - [base]/relay           : Relaisstatus (wird gesendet)
 * - [base]/set_temp_soll   : Solltemperatur setzen (Empfang)
 * - [base]/set_laufzeit    : Laufzeit setzen in Minuten (Empfang)
 * - [base]/laufzeit_min    : Laufzeit Bestätigung (wird gesendet)
 * 
 * Autor: Karl Effinger
 *
 * Change log
 *
 * v1.2.0 - 2025-11-13 - Zeiteinstellung umgestellt, GUI überarbeitet
 * v1.1.0 - 2025-11-11 - Umstellung auf AiEsp32RotaryEncoder Bibliothek
 * v1.0.0 - 2025-11-06 - Finale Version mit Kommentaren
 * 
 * Bekannte Einschränkungen:
 * - NVS Flash hat begrenzte Schreibzyklen (~100.000)
 * - MQTT wird nach 10 fehlgeschlagenen Versuchen bis zum Neustart deaktiviert
 * 
 * Ideen für Verbesserungen:
 * - Mehrere Temperaturzonen
 * - PID-Regelung statt einfacher Hysterese
 * 
 * Kontakt: kaeff@gmx.de
 */

// Rotary bei Zeiteinstellung hat zu große Steps reagiert nicht 1:1

// ============================================================================
// Debugging, für Produktivsystem erste Zeile (#define DEBUG) auskommentieren
// ============================================================================
  #define DEBUG
  #ifdef DEBUG
    #define DBEGIN() do { Serial.begin(115200); delay(100); } while(0)
    #define DPRINT(...) Serial.print(__VA_ARGS__)
    #define DPRINTLN(...) Serial.println(__VA_ARGS__)
    #define DPRINTF(...) Serial.printf(__VA_ARGS__)
    #define DFLUSH() Serial.flush()
    #include <esp_system.h>
  #else
    #define DBEGIN()
    #define DPRINT(...)
    #define DPRINTLN(...)
    #define DPRINTF(...)
    #define DFLUSH()
  #endif

// ============================================================================
// Bibliotheken
// ============================================================================

  #include <lvgl.h>                    // v8.3.11 Grafik-Bibliothek für GUI [WICHTIG: läuft nicht mit v9.x!] https://github.com/lvgl
  #include <TFT_eSPI.h>                // v2.5.43 TFT-Display-Treiber. https://github.com/Bodmer/TFT_eSPI
  #include <OneWire.h>                 // v2.3.8  1-Wire Protokoll für DS18B20. https://github.com/PaulStoffregen/OneWire
  #include <DallasTemperature.h>       // v4.0.5  DS18B20 Temperatursensor. https://github.com/milesburton/Arduino-Temperature-Control-Library
  #include <AiEsp32RotaryEncoder.h>    // v1.7    Drehencoder mit Taster https://github.com/igorantolic/ai-esp32-rotary-encoder
  #include <WiFi.h>                    // v3.3.2  WiFi-Funktionalität. Integriert in ESP32-Library
  #include <WebServer.h>               // v3.3.2  Webserver für Konfiguration. Integriert in ESP32-Library
  #include <Preferences.h>             // v3.3.2  Persistente Speicherung (NVS). Integriert in ESP32-Library
  #include <PubSubClient.h>            // v2.8    MQTT-Client. https://github.com/knolleary/pubsubclient/releases/tag/v2.8



// ============================================================================
// Hardware-Konfiguration
// ============================================================================
  #define ONE_WIRE_BUS 26              // GPIO für DS18B20 Temperatursensor
  #define RELAY_PIN 27                 // GPIO für Relais-Ansteuerung
  #define ROTARY_ENCODER_A_PIN 34      // CLK Pin
  #define ROTARY_ENCODER_B_PIN 35      // DT Pin
  #define ROTARY_ENCODER_BUTTON_PIN 32 // SW Pin
  #define ROTARY_ENCODER_VCC_PIN 33    // VCC Pin (optional)
  
// ============================================================================
// Display-Konfiguration (TFT_eSPI)
// ============================================================================
// HINWEIS: Pin-Konfiguration muss in User_Setup.h der TFT_eSPI Bibliothek erfolgen!
// Dort ist aktuell hinterlegt: CS: 21 / RST: 18 / DC: 19 / SCLK: 17 / MOSI bzw. SDA: 5
  TFT_eSPI tft = TFT_eSPI();

// ============================================================================
// LVGL Display-Konstanten und Puffer
// ============================================================================
  #define LVGL_DISPLAY_WIDTH 280       // Display-Breite in Pixel
  #define LVGL_DISPLAY_HEIGHT 240      // Display-Höhe in Pixel
  #define LVGL_DISPLAY_BUF_ROWS 40     // Anzahl Zeilen im Puffer
  #define LVGL_DISPLAY_BUF_SIZE (LVGL_DISPLAY_WIDTH * LVGL_DISPLAY_BUF_ROWS)

  static lv_disp_draw_buf_t draw_buf;  // LVGL Zeichenpuffer-Struktur
  static lv_color_t buf1[LVGL_DISPLAY_BUF_SIZE]; // Farbpuffer für Display
  static lv_disp_drv_t disp_drv;       // LVGL Display-Treiber

// ============================================================================
// GUI-Objekte (LVGL Labels)
// ============================================================================
  lv_obj_t *label_soll;                // Label: Soll-Temperatur
  lv_obj_t *label_ist;                 // Label: Ist-Temperatur
  lv_obj_t *label_sollzeit;            // Label: Sollzeit (hh:mm)
  lv_obj_t *label_istzeit;             // Label: Verstrichene Zeit
  lv_obj_t *label_timer;               // Label: Verbleibende Zeit
  lv_obj_t *label_relay;               // Label: Relaisstatus
  lv_obj_t *label_status;              // Label: WiFi/MQTT Status
  lv_obj_t *blink_dot;                 // Kleiner blinkender Kreis (grün) im Sekundentakt

// -------------------- GUI: cache / flags für sofortige Updates --------------------
// Damit die unmittelbaren Updates nicht mit cached internalen Werten in update_gui
// kollidieren, speichern wir die zuletzt gezeigten Werte global.
static int gui_last_sollTemp = -32768;
static float gui_last_istTemp = -1000.0f;
static int gui_lastColorCode = -1;
static int gui_lastEncoderModeForSoll = -1;
static unsigned long gui_last_soll_minutes = ULONG_MAX;
static unsigned long gui_last_ist_minutes = ULONG_MAX;
static unsigned long gui_last_rest_minutes = ULONG_MAX;
static int gui_lastEncoderModeForTime = -1;
static bool gui_lastRelayOn = false;
static char gui_last_status[64] = "";

// ============================================================================
// DS18B20 Temperatursensor
// ============================================================================
  OneWire oneWire(ONE_WIRE_BUS);       // 1-Wire Bus initialisieren
  DallasTemperature sensors(&oneWire); // Temperatursensor-Bibliothek
  DeviceAddress sensorAddress;         // Adresse des ersten gefundenen Sensors

  // Temperatursensor - Timing und Status
  unsigned long lastRequest = 0;       // Zeitpunkt der letzten Messanforderung
  bool tempRequested = false;          // Flag: Messung wurde angefordert
  const unsigned long tempRequestInterval = 10*1000;  // Messintervall: 10 Sekunden
  const unsigned long tempConversionDelay = 250;      // Wartezeit für A/D-Wandlung des DS18B20: bei 10 Bit ca. 200ms

// ============================================================================
// Rotary Encoder (Drehgeber mit Taster)
// ============================================================================
  #define ROTARY_ENCODER_STEPS 2       // Schritte pro Detent
  AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(
    ROTARY_ENCODER_A_PIN, 
    ROTARY_ENCODER_B_PIN, 
    ROTARY_ENCODER_BUTTON_PIN, 
    ROTARY_ENCODER_VCC_PIN,
    ROTARY_ENCODER_STEPS
  );

  int encoderMode = 0;                 // 0=Temperatur, 1=Laufzeit
  bool configModeRequested = false;    // Flag: Config-Modus angefordert, nach >3 Sekunden Tastendruck (siehe LONG_PRESS_TIME)

  volatile bool buttonStateChanged = false;  // ISR-sicheres Flag
  volatile unsigned long lastButtonChange = 0;
  const unsigned long DEBOUNCE_DELAY = 50;   // 50ms Entprellung
  
  // Button-Timing für langen Tastendruck
  unsigned long buttonPressStartTime = 0;
  bool buttonWasPressed = false;
  const unsigned long LONG_PRESS_TIME = 3000; // 3 Sekunden

// ============================================================================
// Systemzustände und Parameter
// ============================================================================
  float istTemp = 0.0;                 // Aktuelle Temperatur in °C
  int sollTemp = 40;                   // Solltemperatur in °C (Standard: 40°C)
  int lastSavedSollTemp;               // Zuletzt gespeicherte Solltemperatur

  unsigned long startTime = 0;         // Startzeitpunkt des Timers (0=inaktiv)
  unsigned long laufzeit = 0 * 60 * 60 * 1000; // Laufzeit in ms (Standard: 0h - sonst geht nach einem Stromausfall die Kiste wieder an!)
  const unsigned long LAUFZEIT_STEP_MS = 60*1000UL; // 1 Minute pro Encoder-Step
  bool LaufzeitEnde = false;           // Flag: Laufzeit abgelaufen

  // Auto-Save Konfiguration
  unsigned long lastConfigSave = 0;    // Zeitpunkt der letzten Config-Speicherung
  const unsigned long configSaveInterval = 5 * 60 * 1000; // Speicher-Intervall: 5 Minuten

// ============================================================================
// Relais-Steuerung
// ============================================================================
  const float hysterese = 0.5;         // Hysterese in °C (±0.5°C)
  bool relayOn = false;                // Aktueller Relaisstatus

// ============================================================================
// WiFi und MQTT Konfiguration
// ============================================================================
  WiFiClient espClient;                // WiFi-Client für MQTT
  PubSubClient mqttClient(espClient);  // MQTT-Client
  Preferences preferences;             // NVS (Non-Volatile Storage) für Einstellungen

  // WiFi-Zugangsdaten (werden aus NVS geladen)
  String wifi_ssid = "";
  String wifi_password = "";
  // MQTT-Verbindungsdaten (werden aus NVS geladen)
  String mqtt_server = "";
  int mqtt_port = 1883;
  String mqtt_user = "";
  String mqtt_password = "";
  String mqtt_topic_base = "waermeschrank";  // Basis-Topic für alle MQTT-Nachrichten

  // MQTT Publish-Timing
    unsigned long lastMqttPublish = 0;   // Zeitpunkt des letzten MQTT-Publish
    const unsigned long mqttPublishInterval = 2 * 60 * 1000; // Publish-Intervall: 2 Minuten

  // MQTT Reconnect-Logik
    int mqttReconnectAttempts = 0;       // Anzahl fehlgeschlagener Verbindungsversuche
    const int maxMqttReconnectAttempts = 10; // Max. Versuche vor Deaktivierung
    unsigned long lastMqttReconnectAttempt = 0; // Zeitpunkt des letzten Versuchs
    const unsigned long mqttReconnectCooldown = 60 * 1000; // Pause zwischen Versuchen: 1 Minute
    bool mqttEnabled = true;             // Flag: MQTT aktiv (wird bei zu vielen Fehlern false)

  // MQTT Status-Anzeige
  bool mqttSending = false;            // Flag: MQTT-Daten werden gerade gesendet
  unsigned long mqttSendingStartTime = 0; // Startzeitpunkt der Sendeanzeige

// ============================================================================
// Webserver für Konfiguration
// ============================================================================
  WebServer server(80);                // Webserver auf Port 80

// ============================================================================
// Funktions-Deklarationen (Forward Declarations)
// ============================================================================
  void my_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_p);
  void create_gui();
  void update_gui();
  void startTemperatureRequestIfNeeded();
  void readTemperatureIfReady();
  void loadConfig();
  void saveConfig(bool mitMQTT);
  void enterConfigMode();
  void handleRoot();
  void handleSave();
  void connectWiFi();
  void reconnectMQTT();
  void publishMQTTData();
  void mqttCallback(char* topic, byte* payload, unsigned int length);
  void rotary_onButtonClick();
  void rotary_loop();
  void IRAM_ATTR readEncoderISR();

// ============================================================================
// Interrupt Service Routine für Rotary Encoder samt Button
// ============================================================================
/**
 * @brief ISR für Rotary Encoder - muss mit IRAM_ATTR markiert sein
 * Diese Funktion wird bei jeder Encoder-Drehung aufgerufen
 */
void IRAM_ATTR readEncoderISR() {
  rotaryEncoder.readEncoder_ISR();
}

void IRAM_ATTR buttonISR() {
  unsigned long now = millis();
  if (now - lastButtonChange > DEBOUNCE_DELAY) {
    buttonStateChanged = true;
    lastButtonChange = now;
  }
}


// ============================================================================
// SETUP - Wird einmal beim Start ausgeführt
// ============================================================================
void setup() {
  // --------------------------------------------------------------------------
  // Serielle Schnittstelle initialisieren (sofern DEBUG definiert ist)
  // --------------------------------------------------------------------------
  DBEGIN();
  #ifdef DEBUG
    esp_reset_reason_t reason = esp_reset_reason();
    DPRINT("Reset-Grund: ");
    switch (reason) {
      case ESP_RST_POWERON: DPRINTLN("Power-On Reset"); break;
      case ESP_RST_EXT:     DPRINTLN("Externer Reset"); break;
      case ESP_RST_SW:      DPRINTLN("Software Reset"); break;
      case ESP_RST_PANIC:   DPRINTLN("Panik (z. B. Absturz)"); break;
      case ESP_RST_INT_WDT: DPRINTLN("Interrupt Watchdog"); break;
      case ESP_RST_TASK_WDT:DPRINTLN("Task Watchdog"); break;
      case ESP_RST_BROWNOUT:DPRINTLN("Brownout (Spannungseinbruch)"); break;
      case ESP_RST_SDIO:    DPRINTLN("SDIO Reset"); break;
      default:              DPRINTLN("Unbekannt"); break;
    }
  #endif
  
  // --------------------------------------------------------------------------
  // Konfiguration aus NVS laden
  // --------------------------------------------------------------------------
  loadConfig();                      // WiFi, MQTT und Solltemperatur laden
  
  // --------------------------------------------------------------------------
  // TFT-Display initialisieren
  // --------------------------------------------------------------------------
  tft.begin();
  tft.setRotation(1);                // Querformat
  
  // --------------------------------------------------------------------------
  // LVGL (Grafik-Bibliothek) initialisieren
  // --------------------------------------------------------------------------
  lv_init();
  
  // Display-Puffer konfigurieren
  lv_disp_draw_buf_init(&draw_buf, buf1, NULL, LVGL_DISPLAY_BUF_SIZE);
  
  // Display-Treiber konfigurieren
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = LVGL_DISPLAY_WIDTH;
  disp_drv.ver_res = LVGL_DISPLAY_HEIGHT;
  disp_drv.flush_cb = my_flush_cb;   // Callback zum Zeichnen
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);   // Treiber registrieren
  
  DPRINTLN("LVGL v8 display driver registered");
  
  // --------------------------------------------------------------------------
  // GUI erstellen und initial anzeigen
  // --------------------------------------------------------------------------
  create_gui();
  lv_refr_now(NULL);                 // GUI sofort zeichnen
  delay(50);
  
  // --------------------------------------------------------------------------
  // DS18B20 Temperatursensor initialisieren
  // --------------------------------------------------------------------------
  sensors.begin();
  if (!sensors.getAddress(sensorAddress, 0)) {
    DPRINTLN("Kein DS18B20 gefunden!");
  } else {
    // Auflösung auf 10 Bit setzen (0.25°C, 188ms Wandlungszeit)
    // Bei Änderung ggf. auch tempConversionDelay entsprechend anpassen!
    sensors.setResolution(sensorAddress, 10);
  }
  
  // --------------------------------------------------------------------------
  // Rotary Encoder konfigurieren
  // --------------------------------------------------------------------------
  rotaryEncoder.begin();
  rotaryEncoder.setup(readEncoderISR);
  pinMode(ROTARY_ENCODER_BUTTON_PIN, INPUT); // habe einen 10k Pull-Up Widerstand am Button!
  attachInterrupt(digitalPinToInterrupt(ROTARY_ENCODER_BUTTON_PIN), buttonISR, CHANGE);
  
  // Grenzen für Temperatur-Modus setzen (20-60°C)
  rotaryEncoder.setBoundaries(20, 60, false); // minValue, maxValue, circleValues
  rotaryEncoder.setAcceleration(100); // Beschleunigung beim schnellen Drehen (optional)
  
  // Startwert setzen
  rotaryEncoder.setEncoderValue(sollTemp);  // Startwert setzen
  
  DPRINTLN("Rotary Encoder initialisiert");
  
  // --------------------------------------------------------------------------
  // Relais-Pin konfigurieren
  // --------------------------------------------------------------------------
  pinMode(RELAY_PIN, OUTPUT);
  relayOn = false;
  digitalWrite(RELAY_PIN, LOW);      // Relais initial ausgeschaltet
  
  // --------------------------------------------------------------------------
  // WiFi verbinden (falls konfiguriert)
  // --------------------------------------------------------------------------
  if (wifi_ssid.length() > 0) {
    connectWiFi();
    
    // MQTT konfigurieren und verbinden (falls WiFi erfolgreich)
    if (mqtt_server.length() > 0 && WiFi.status() == WL_CONNECTED) {
      mqttClient.setServer(mqtt_server.c_str(), mqtt_port);
      mqttClient.setCallback(mqttCallback); // Callback für eingehende Nachrichten
      delay(500);
      reconnectMQTT();
      
      // Wenn MQTT initial nicht erreichbar, trotzdem weitermachen
      if (!mqttClient.connected()) {
        DPRINTLN("MQTT initial nicht verfuegbar - Geraet arbeitet offline");
        if (label_status) {
          lv_label_set_text(label_status, "WiFi OK, MQTT offline");
        }
      }
    }
  } else {
    // Keine WiFi-Konfiguration vorhanden
    DPRINTLN("Keine WiFi-Konfiguration vorhanden");
    DPRINTLN("Lange Taste druecken fuer Config-Mode");
    if (label_status) {
      lv_label_set_text(label_status, "Keine WiFi Config");
    }
  }
  
  // --------------------------------------------------------------------------
  // Setup abgeschlossen - Bedienungshinweise ausgeben
  // --------------------------------------------------------------------------
  DPRINTLN("\n=================================");
  DPRINTLN("Setup abgeschlossen!");
  DPRINTLN("Kurzer Tastendruck: Mode wechseln");
  DPRINTLN("Langer Tastendruck (3s): Config");
  DPRINTLN("=================================\n");
}

// ============================================================================
// LOOP - Wird kontinuierlich ausgeführt
// ============================================================================
void loop() {
  // --------------------------------------------------------------------------
  // Config-Modus prüfen (>3s Tastendruck) und ggf. starten
  // --------------------------------------------------------------------------
  if (configModeRequested) {
    enterConfigMode();                // Config-Modus mit Web-Interface
    ESP.restart();                    // Nach Config-Änderungen Neustart
  }
  
  // --------------------------------------------------------------------------
  // LVGL Zeitbasis aktualisieren (alle 5ms)
  // --------------------------------------------------------------------------
  static unsigned long lv_last_tick = 0;
  unsigned long now = millis();
  if (lv_last_tick == 0) lv_last_tick = now;
  if (now - lv_last_tick >= 5) {
    lv_tick_inc(5);                   // LVGL mitteilen, dass 5ms vergangen sind
    lv_last_tick += 5;
  }
  
  lv_task_handler();                  // LVGL Tasks verarbeiten
  delay(5);                           // Kurze Pause für Task-Scheduler
  
  // --------------------------------------------------------------------------
  // DS18B20 Temperaturmessung (non-blocking)
  // --------------------------------------------------------------------------
  startTemperatureRequestIfNeeded(); // Neue Messung starten wenn fällig
  readTemperatureIfReady();          // Messergebnis abholen wenn fertig
  
  // --------------------------------------------------------------------------
  // Rotary Encoder auslesen (NEUE BIBLIOTHEK)
  // --------------------------------------------------------------------------
  rotary_loop();
  
  // Overflow-Schutz: Wenn millis() kleiner als startTime ist, neu initialisieren
  // passiert nur, wenn das Gerät über 48 Tage läuft...
  if (startTime > 0 && millis() < startTime) {
      DPRINTLN("millis() Overflow erkannt - Timer zurückgesetzt");
      startTime = millis();  // Timer neu starten
  }

  // --------------------------------------------------------------------------
  // Relais-Steuerung mit Hysterese und Laufzeitüberwachung
  // --------------------------------------------------------------------------
  // Prüfen ob Laufzeit abgelaufen ist
  if (!LaufzeitEnde && startTime > 0 && (millis() - startTime >= laufzeit)) {
    LaufzeitEnde = true;
    DPRINTLN("Laufzeit abgelaufen – Relais wird deaktiviert.");
  }
  // Prüfen ob Laufzeit wieder neu gestartet wurde (z.B. durch Änderung)
  else if (LaufzeitEnde && startTime > 0 && (millis() - startTime <= laufzeit)) {
    DPRINTLN("Laufzeit wieder gestartet und Relais reaktiviert.");
    LaufzeitEnde = false;
  }
  


// Laufzeit-Check: Ist überhaupt eine Laufzeit gesetzt?
  bool laufzeitAktiv = (laufzeit > 0);
  
  // Bestimmen ob Laufzeit das Heizen erlaubt
  bool laufzeitErlaubtHeizen = false;
  if (!laufzeitAktiv) {
    // Keine Laufzeit gesetzt (0 Minuten) -> Heizen nicht erlaubt
    laufzeitErlaubtHeizen = false;
  } else if (startTime == 0) {
    // Timer noch nicht gestartet, aber Laufzeit gesetzt -> Heizen erlaubt (Aufheizphase)
    laufzeitErlaubtHeizen = true;
  } else {
    // Timer läuft -> nur erlaubt wenn noch nicht abgelaufen
    laufzeitErlaubtHeizen = !LaufzeitEnde;
  }
  
  // Relais einschalten wenn: Temperatur zu niedrig UND Laufzeit erlaubt Heizen
  if (!relayOn && laufzeitErlaubtHeizen && istTemp != DEVICE_DISCONNECTED_C && istTemp <= (sollTemp - hysterese)) {
    relayOn = true;
    digitalWrite(RELAY_PIN, HIGH);
  }
  // Relais ausschalten wenn: Temperatur zu hoch (Solltemperatur erreicht)
  else if (relayOn && istTemp >= (sollTemp + hysterese)) {
    relayOn = false;
    digitalWrite(RELAY_PIN, LOW);
    // Timer starten wenn Solltemperatur das erste Mal erreicht wurde
    if (laufzeitAktiv && startTime == 0) {
      startTime = millis();
      DPRINTLN("Timer gestartet - Solltemperatur erreicht!");
    }
  }
  // Relais ausschalten wenn: Laufzeit abgelaufen oder keine Laufzeit gesetzt
  else if (relayOn && (!laufzeitErlaubtHeizen)) {
    relayOn = false;
    digitalWrite(RELAY_PIN, LOW);
  }

  // Relais ausschalten, wenn der Temp-Sensor keine Daten liefert
  if (relayOn && istTemp == DEVICE_DISCONNECTED_C) {
    relayOn = false;
    digitalWrite(RELAY_PIN, LOW);
    DPRINTLN("Relais wegen Sensor-Fehler ausgeschaltet!");
  }
  
  // --------------------------------------------------------------------------
  // MQTT Verbindung aufrechterhalten
  // --------------------------------------------------------------------------
  if (WiFi.status() == WL_CONNECTED && mqtt_server.length() > 0 && mqttEnabled) {
    if (!mqttClient.connected()) {
      // Nicht verbunden - Reconnect versuchen (mit Cooldown)
      if (millis() - lastMqttReconnectAttempt >= mqttReconnectCooldown) {
        reconnectMQTT();
      }
    } else {
      // Verbunden - Fehlerzähler zurücksetzen
      if (mqttReconnectAttempts > 0) {
        mqttReconnectAttempts = 0;
        DPRINTLN("MQTT Zaehler zurueckgesetzt");
      }
    }
    
    mqttClient.loop();                // MQTT Nachrichten verarbeiten
    
    // Periodisch Daten publishen (alle <mqttPublishInterval> Millisekunden)
    if (mqttClient.connected() && millis() - lastMqttPublish >= mqttPublishInterval) {
      publishMQTTData();
      lastMqttPublish = millis();
    }
  }
  
  // --------------------------------------------------------------------------
  // MQTT-Sendeanzeige zurücksetzen (nach 2 Sekunden)
  // --------------------------------------------------------------------------
  if (mqttSending && (millis() - mqttSendingStartTime > 2000)) {
    mqttSending = false;
  }
  
  // --------------------------------------------------------------------------
  // Auto-Save: Konfiguration periodisch speichern (alle 5 Minuten)
  // --------------------------------------------------------------------------
  if (millis() - lastConfigSave >= configSaveInterval) {
    saveConfig(false);                // Nur Solltemperatur speichern (nicht MQTT-Config)
    lastConfigSave = millis();
  }
  
  // --------------------------------------------------------------------------
  // GUI aktualisieren (alle 200ms)
  // --------------------------------------------------------------------------
  static unsigned long lastGuiUpdate = 0;
  if (millis() - lastGuiUpdate >= 200) {
    update_gui();
    lastGuiUpdate = millis();
  }
}

// ============================================================================
// Rotary Encoder Handler
// ============================================================================
/**
 * @brief Verarbeitet Encoder-Drehung und Button-Druck
 * Muss in loop() regelmäßig aufgerufen werden
 */
void rotary_loop() {
  static int lastRawEncoder = 0;
  static bool encoderInitialized = false;
  
  // Beim ersten Aufruf: Startwert setzen
  if (!encoderInitialized) {
    lastRawEncoder = rotaryEncoder.readEncoder();
    encoderInitialized = true;
  }
  
  // --------------------------------------------------------------------------
  // ENCODER-WERT AUSLESEN (bei jeder Änderung)
  // --------------------------------------------------------------------------
  if (rotaryEncoder.encoderChanged()) {
    int currentRaw = rotaryEncoder.readEncoder();
    
    // Im Zeit-Modus: Nach Modus-Wechsel ersten Wert ignorieren und synchronisieren
    static int lastEncoderMode = 0;
    if (encoderMode == 1 && lastEncoderMode == 0) {
      // Gerade von Temp-Modus zu Zeit-Modus gewechselt
      lastRawEncoder = currentRaw;  // Synchronisieren, KEIN Delta berechnen
      lastEncoderMode = 1;
      DPRINTF("Zeit-Modus: Encoder synchronisiert auf %d\n", currentRaw);
      return;  // Diesen Wert ignorieren, keine Änderung
    }
    lastEncoderMode = encoderMode;
    
    int delta = currentRaw - lastRawEncoder;
    lastRawEncoder = currentRaw;
    
    if (encoderMode == 0) {
      // ====== MODUS 0: TEMPERATUR ======
      // Direkte Übernahme (20-60°C, wird von Boundaries begrenzt)
      if (currentRaw != sollTemp) {
        sollTemp = currentRaw;
        DPRINTF("Neue SollTemp: %d (delta=%d)\n", sollTemp, delta);
        gui_force_update_encoder_visuals();
      }
    } 
    else {
      // ====== MODUS 1: LAUFZEIT ======
      // Delta-basiert: Jeder Encoder-Schritt = 1 Minute
      if (delta != 0) {
        unsigned long ist_ms;
        if (startTime == 0) {
          ist_ms = 0;
        } else if (LaufzeitEnde) {
          ist_ms = laufzeit;
        } else {
          ist_ms = millis() - startTime;
          if (ist_ms > laufzeit) ist_ms = laufzeit;
        }
        
        // Neue Laufzeit berechnen
        long change_ms = (long)delta * (long)LAUFZEIT_STEP_MS;
        long newLaufzeit = (long)laufzeit + change_ms;
        
        // Grenzen: 0 bis 5940 Minuten (99 Stunden)
        unsigned long max_ms = 5940UL * LAUFZEIT_STEP_MS;
        if (newLaufzeit < 0) newLaufzeit = 0;
        if ((unsigned long)newLaufzeit > max_ms) newLaufzeit = (long)max_ms;
        
        // Bei laufendem Timer: nicht unter Istzeit gehen
        if (startTime > 0 && (unsigned long)newLaufzeit < ist_ms) {
          newLaufzeit = (long)ist_ms;
          if (newLaufzeit == (long)laufzeit) {
            // Bereits am Minimum -> keine Änderung
            LaufzeitEnde = true;
            DPRINTLN("Bereits am Minimum (Istzeit erreicht)");
          } else {
            LaufzeitEnde = true;
            DPRINTF("Auf Istzeit geclamped: %lu min\n", ist_ms/60000UL);
          }
        } else {
          // Prüfen ob Laufzeit wieder über Istzeit liegt
          if (startTime > 0 && (unsigned long)newLaufzeit > ist_ms) {
            LaufzeitEnde = false;
          }
        }
        
        laufzeit = (unsigned long)newLaufzeit;
        DPRINTF("Laufzeit: %lu min (delta=%d min)\n", 
                laufzeit/60000UL, delta);
        
        gui_force_update_encoder_visuals();
      }
    }
  }
  
  // --------------------------------------------------------------------------
  // BUTTON-ERKENNUNG (ISR-getriggert)
  // --------------------------------------------------------------------------
  if (buttonStateChanged) {
    buttonStateChanged = false;  // Flag zurücksetzen
    
    bool isPressed = (digitalRead(ROTARY_ENCODER_BUTTON_PIN) == LOW);
    
    if (isPressed && !buttonWasPressed) {
      // Button wurde gedrückt
      buttonWasPressed = true;
      buttonPressStartTime = millis();
      DPRINTLN("Button DOWN");
    }
    else if (!isPressed && buttonWasPressed) {
      // Button wurde losgelassen
      unsigned long pressDuration = millis() - buttonPressStartTime;
      buttonWasPressed = false;
      
      DPRINTF("Button UP (Dauer: %lu ms)\n", pressDuration);
      
      // Plausibilitätsprüfung
      if (pressDuration > 60000UL) {
        DPRINTLN("Unrealistische Dauer -> ignoriert");
        return;
      }
      
      if (pressDuration > LONG_PRESS_TIME) {
        // Langer Druck -> Config-Mode
        configModeRequested = true;
        DPRINTLN("==> Config-Mode angefordert");
      } 
      else if (pressDuration > DEBOUNCE_DELAY) {
        // Kurzer Druck -> Mode wechseln
        rotary_onButtonClick();
      }
    }
  }
}

/**
 * @brief Handler für kurzen Button-Druck (Modus-Wechsel)
 */
void rotary_onButtonClick() {
  encoderMode = (encoderMode + 1) % 2;
  
  if (encoderMode == 0) {
    // Modus 0: Temperatur einstellen (20-60°C)
    rotaryEncoder.setBoundaries(20, 60, false);
    rotaryEncoder.setAcceleration(100);
    rotaryEncoder.setEncoderValue(sollTemp);
    DPRINTLN("Modus: Temperatur");
  } 
  else {
    // Modus 1: Laufzeit einstellen (0-5940 min = 0-99.9h)
    rotaryEncoder.setBoundaries(-10000, 10000, false);
    rotaryEncoder.setAcceleration(250);
    rotaryEncoder.setEncoderValue(0);
    // Merke den letzten Encoderwert für delta-basiertes Arbeiten
    DPRINTLN("Modus: Laufzeit");
  }
  // sofortige UI-Aktualisierung (Farbe + ggf. Zeiten)
  gui_force_update_encoder_visuals();  
}

// ============================================================================
// LVGL: Display Flush Callback
// ============================================================================
void my_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_p) {
  uint32_t w = (area->x2 - area->x1 + 1); // Breite des Bereichs
  uint32_t h = (area->y2 - area->y1 + 1); // Höhe des Bereichs
  
  tft.startWrite();                   // SPI-Transaktion beginnen
  tft.setAddrWindow(area->x1, area->y1, w, h); // Zeichenbereich setzen
  tft.pushColors((uint16_t *)color_p, w * h, true); // Pixel übertragen
  tft.endWrite();                     // SPI-Transaktion beenden
  lv_disp_flush_ready(drv);           // LVGL mitteilen: Zeichnen abgeschlossen
}

// ============================================================================
// GUI: Benutzeroberfläche erstellen
// ============================================================================
/**
 * @brief Erstellt alle GUI-Elemente (Labels) auf dem Display
 * 
 * Die GUI besteht aus 6 Labels:
 * - Ist-Temperatur
 * - Soll-Temperatur
 * - Verbleibende Zeit
 * - Verstrichene Zeit
 * - Relaisstatus
 * - WiFi/MQTT Status
 */
void create_gui() {
  lv_obj_t *scr = lv_scr_act();       // Aktiven Screen holen
  
  // Soll-Temperatur Label (links auf derselben Zeile)
  label_soll = lv_label_create(scr);
  lv_label_set_text(label_soll, "Soll: -- °C");
  lv_obj_set_pos(label_soll, 10, 20); // links
  lv_obj_set_style_text_font(label_soll, &lv_font_montserrat_18, 0);

  // Ist-Temperatur Label (rechts auf derselben Zeile wie Soll-Temperatur)
  label_ist = lv_label_create(scr);
  lv_label_set_text(label_ist, "Ist: --.- °C");
  lv_obj_set_pos(label_ist, 150, 20); // rechts
  lv_obj_set_style_text_font(label_ist, &lv_font_montserrat_18, 0);
  
  // Sollzeit (hh:mm)
  label_sollzeit = lv_label_create(scr);
  lv_label_set_text(label_sollzeit, "Soll: --:--");
  lv_obj_set_pos(label_sollzeit, 10, 50);
  lv_obj_set_style_text_font(label_sollzeit, &lv_font_montserrat_18, 0);

  // Verstrichene Zeit Label (Istzeit)
  label_istzeit = lv_label_create(scr);
  lv_label_set_text(label_istzeit, "Ist:  --:--");
  lv_obj_set_pos(label_istzeit, 150, 50);
  lv_obj_set_style_text_font(label_istzeit, &lv_font_montserrat_18, 0);

  // Verbleibende Zeit Label (Restzeit)
  label_timer = lv_label_create(scr);
  lv_label_set_text(label_timer, "Rest: --:--");
  lv_obj_set_pos(label_timer, 10, 80);
  lv_obj_set_style_text_font(label_timer, &lv_font_montserrat_16, 0);

  // Blinkender Würfel (klein, grün) rechts oben bei den Zeiten
  blink_dot = lv_obj_create(scr);
  lv_obj_set_size(blink_dot, 10, 10);
  lv_obj_set_style_radius(blink_dot, 2, 0);
  lv_obj_set_style_bg_color(blink_dot, lv_color_hex(0x00AA00), 0);
  lv_obj_set_pos(blink_dot, 120, 55);
  lv_obj_clear_flag(blink_dot, LV_OBJ_FLAG_HIDDEN); // zum Toggeln zwischen sichtbar und unsichtbar
  
  // Relaisstatus
  label_relay = lv_label_create(scr);
  lv_label_set_text(label_relay, "Relais: aus");
  lv_obj_set_pos(label_relay, 10, 150);
  lv_obj_set_style_text_font(label_relay, &lv_font_montserrat_16, 0);
  
  // WIFI- und MQTT-Status
  label_status = lv_label_create(scr);
  lv_label_set_text(label_status, "WiFi: ---");
  lv_obj_set_pos(label_status, 10, 180);
  lv_obj_set_style_text_font(label_status, &lv_font_montserrat_16, 0);
}

// ============================================================================
// GUI: Anzeige aktualisieren
// ============================================================================
/**
 * @brief Aktualisiert alle Werte und Farben in der GUI
 * 
 * Wird alle 200ms aufgerufen (siehe loop())
 * - Aktualisiert Temperatur, Zeit, Relaisstatus
 * - Markiert aktiven Encoder-Modus rot
 * - Invertiert Zeitanzeige bei Laufzeitende
 */
void update_gui() {
  char buf[80];
  unsigned long now = millis();

  // -------------------- Temperatur-Teil (weniger oft, z.B. 500ms) --------------------
  static unsigned long last_temp_update = 0;
  if (now - last_temp_update >= 500) {
    last_temp_update = now;

    // Soll-Temperatur (nur bei Änderung aktualisieren)
    if (sollTemp != gui_last_sollTemp) {
      snprintf(buf, sizeof(buf), "Soll: %d °C", sollTemp);
      lv_label_set_text(label_soll, buf);
      gui_last_sollTemp = sollTemp;
    }

    // Ist-Temperatur (nur bei Änderung aktualisieren)

    if (istTemp != gui_last_istTemp) {
      snprintf(buf, sizeof(buf), "Ist: %.1f °C", istTemp);
      lv_label_set_text(label_ist, buf);
      gui_last_istTemp = istTemp;
    }

    // Farbe für Ist-Temperatur bestimmen (nur setzen wenn sich Farbe ändert)
    int colorCode; // 0=grün,1=rot,2=blau,3=grau
    if (istTemp == DEVICE_DISCONNECTED_C) colorCode = 3;
    else if (istTemp > (sollTemp + 0.5f)) colorCode = 1;
    else if (istTemp < (sollTemp - 0.5f)) colorCode = 2;
    else colorCode = 0;

    if (colorCode != gui_lastColorCode) {
      switch (colorCode) {
        case 0: lv_obj_set_style_text_color(label_ist, lv_color_hex(0x00AA00), 0); break; // grün
        case 1: lv_obj_set_style_text_color(label_ist, lv_color_hex(0xFF0000), 0); break; // rot
        case 2: lv_obj_set_style_text_color(label_ist, lv_color_hex(0x0000FF), 0); break; // blau
        default: lv_obj_set_style_text_color(label_ist, lv_color_hex(0x888888), 0); break; // grau
      }
      gui_lastColorCode = colorCode;
    }

    // Soll-Temperatur editierbar? (rot wenn encoderMode==0)
    if (encoderMode != gui_lastEncoderModeForSoll) {
      if (encoderMode == 0) lv_obj_set_style_text_color(label_soll, lv_color_hex(0xFF0000), 0);
      else lv_obj_set_style_text_color(label_soll, lv_color_hex(0x000000), 0);
      gui_lastEncoderModeForSoll = encoderMode;
    }
  } // end temp update

  // -------------------- Zeit-Teil (nur jede Sekunde aktualisieren + Blink) --------------------
  static unsigned long last_time_update = 0;
  if (now - last_time_update >= 1000) {
    last_time_update = now;

    // Istzeit berechnen (ms)
    unsigned long ist_ms;
    if (startTime == 0) {
      ist_ms = 0;
    } else if (LaufzeitEnde) {
      ist_ms = laufzeit;
    } else {
      ist_ms = millis() - startTime;
    }
    if (ist_ms > laufzeit) ist_ms = laufzeit;

    // Minuten-Berechnungen
    unsigned long soll_minutes = laufzeit / 60000UL;
    unsigned long ist_minutes = ist_ms / 60000UL;
    unsigned long rest_minutes = (soll_minutes <= ist_minutes) ? 0 : (soll_minutes - ist_minutes);

    if (soll_minutes != gui_last_soll_minutes) {
      unsigned long hh = soll_minutes / 60;
      unsigned long mm = soll_minutes % 60;
      snprintf(buf, sizeof(buf), "Soll: %02lu:%02lu", hh, mm);
      lv_label_set_text(label_sollzeit, buf);
      gui_last_soll_minutes = soll_minutes;
    }

    if (ist_minutes != gui_last_ist_minutes) {
      unsigned long hh = ist_minutes / 60;
      unsigned long mm = ist_minutes % 60;
      snprintf(buf, sizeof(buf), "Ist: %02lu:%02lu", hh, mm);
      lv_label_set_text(label_istzeit, buf);
      gui_last_ist_minutes = ist_minutes;
    }

    if (rest_minutes != gui_last_rest_minutes) {
      unsigned long hh = rest_minutes / 60;
      unsigned long mm = rest_minutes % 60;
      snprintf(buf, sizeof(buf), "Rest: %02lu:%02lu", hh, mm);
      lv_label_set_text(label_timer, buf);
      gui_last_rest_minutes = rest_minutes;
    }

    // Blink-Indicator: toggle sichtbar/unsichtbar (performanter als style-Änderungen)
    static bool blink_state = true;
    blink_state = !blink_state;
    blink_state ? lv_obj_clear_flag(blink_dot, LV_OBJ_FLAG_HIDDEN) : lv_obj_add_flag(blink_dot, LV_OBJ_FLAG_HIDDEN);

    // Markiere Sollzeit/Rest rot wenn Laufzeit-Modus aktiv (nur wenn Änderung)
    if (encoderMode != gui_lastEncoderModeForTime) {
      if (encoderMode == 1) {
        lv_obj_set_style_text_color(label_sollzeit, lv_color_hex(0xFF0000), 0);
        lv_obj_set_style_text_color(label_timer, lv_color_hex(0xFF0000), 0);
      } else {
        lv_obj_set_style_text_color(label_sollzeit, lv_color_hex(0x000000), 0);
        lv_obj_set_style_text_color(label_timer, lv_color_hex(0x000000), 0);
      }
      gui_lastEncoderModeForTime = encoderMode;
    }

    // invertiere die Laufzeit bei Erreichen des Laufzeitendes
    if (LaufzeitEnde) {
      lv_obj_set_style_bg_color(label_timer, lv_color_hex(0xFF0000), 0);
      lv_obj_set_style_bg_opa(label_timer, LV_OPA_COVER, 0);
      lv_obj_set_style_text_color(label_timer, lv_color_hex(0xFFFFFF), 0);
    } else {
      lv_obj_set_style_bg_opa(label_timer, LV_OPA_TRANSP, 0);
      lv_obj_set_style_text_color(label_timer, lv_color_hex(0x000000), 0);
    }
  } // end time update

  // -------------------- Relais & Status (wenig änderungen, daher nur bei Änderung) --------------------
  if (relayOn != gui_lastRelayOn) {
    snprintf(buf, sizeof(buf), "Relais: %s", relayOn ? "an" : "aus");
    lv_label_set_text(label_relay, buf);
    gui_lastRelayOn = relayOn;
  }

  // Status-Label zusammensetzen, nur setzen wenn sich Text ändert
  const char *new_status;
  static char status_buf[64];
  if (mqttSending) {
    new_status = "sende MQTT-Paket";
  } else if (WiFi.status() == WL_CONNECTED) {
    if (!mqttEnabled) new_status = "WiFi OK, MQTT aus";
    else if (mqttClient.connected()) new_status = "WiFi+MQTT OK";
    else new_status = "WiFi OK, MQTT --";
  } else {
    new_status = "WiFi: ---";
  }
  if (strcmp(new_status, gui_last_status) != 0) {
    // set text and cache it
    strncpy(gui_last_status, new_status, sizeof(gui_last_status)-1);
    gui_last_status[sizeof(gui_last_status)-1] = '\0';
    lv_label_set_text(label_status, new_status);
  }
}  

void gui_force_update_encoder_visuals() {
  // Aktualisiert nur die UI-Elemente, die sofort sichtbar sein müssen:
  // - Soll-Temperatur Text & Farbe (bei encoderMode==0 rot)
  // - Sollzeit / Rest / Ist Minutenanzeige und deren Hervorhebung (bei encoderMode==1)
  // - Relais-Text
  char buf[80];

  // 1) Soll-Temperatur Text + Farbe
  snprintf(buf, sizeof(buf), "Soll: %d °C", sollTemp);
  lv_label_set_text(label_soll, buf);
  gui_last_sollTemp = sollTemp;

  if (encoderMode == 0) {
    lv_obj_set_style_text_color(label_soll, lv_color_hex(0xFF0000), 0); // editierbar -> rot
  } else {
    lv_obj_set_style_text_color(label_soll, lv_color_hex(0x000000), 0); // normal -> schwarz
  }
  gui_lastEncoderModeForSoll = encoderMode;

  // 2) Zeiten: berechne Ist (ms) und Minuten (wie in update_gui)
  unsigned long ist_ms;
  if (startTime == 0) {
    ist_ms = 0;
  } else if (LaufzeitEnde) {
    ist_ms = laufzeit;
  } else {
    ist_ms = millis() - startTime;
  }
  if (ist_ms > laufzeit) ist_ms = laufzeit;

  unsigned long soll_minutes = laufzeit / 60000UL;
  unsigned long ist_minutes = (ist_ms == 0) ? 0 : (ist_ms + 60000UL - 1) / 60000UL; // ceil
  unsigned long rest_minutes = (soll_minutes <= ist_minutes) ? 0 : (soll_minutes - ist_minutes);

  // Aktualisiere Sollzeit (sofort)
  if (soll_minutes != gui_last_soll_minutes) {
    unsigned long hh = soll_minutes / 60;
    unsigned long mm = soll_minutes % 60;
    snprintf(buf, sizeof(buf), "Soll: %02lu:%02lu", hh, mm);
    lv_label_set_text(label_sollzeit, buf);
    gui_last_soll_minutes = soll_minutes;
  }

  // Aktualisiere Istzeit (sofort)
  if (ist_minutes != gui_last_ist_minutes) {
    unsigned long hh = ist_minutes / 60;
    unsigned long mm = ist_minutes % 60;
    snprintf(buf, sizeof(buf), "Ist: %02lu:%02lu", hh, mm);
    lv_label_set_text(label_istzeit, buf);
    gui_last_ist_minutes = ist_minutes;
  }

  // Aktualisiere Restzeit (sofort)
  if (rest_minutes != gui_last_rest_minutes) {
    unsigned long hh = rest_minutes / 60;
    unsigned long mm = rest_minutes % 60;
    snprintf(buf, sizeof(buf), "Rest: %02lu:%02lu", hh, mm);
    lv_label_set_text(label_timer, buf);
    gui_last_rest_minutes = rest_minutes;
  }

  // 3) Hervorhebung für Zeiten bei Laufzeit-Modus
  if (encoderMode == 1) {
    lv_obj_set_style_text_color(label_sollzeit, lv_color_hex(0xFF0000), 0);
    lv_obj_set_style_text_color(label_timer, lv_color_hex(0xFF0000), 0);
  } else {
    lv_obj_set_style_text_color(label_sollzeit, lv_color_hex(0x000000), 0);
    lv_obj_set_style_text_color(label_timer, lv_color_hex(0x000000), 0);
  }

  // Bei Laufzeitende invertieren
  if (LaufzeitEnde) {
    lv_obj_set_style_bg_color(label_timer, lv_color_hex(0xFF0000), 0);
    lv_obj_set_style_bg_opa(label_timer, LV_OPA_COVER, 0);
    lv_obj_set_style_text_color(label_timer, lv_color_hex(0xFFFFFF), 0);
  } else {
    lv_obj_set_style_bg_opa(label_timer, LV_OPA_TRANSP, 0);
    lv_obj_set_style_text_color(label_timer, lv_color_hex(0x000000), 0);
  }

  gui_lastEncoderModeForTime = encoderMode;

  // 4) Relaisstatus (falls nötig)
  if (relayOn != gui_lastRelayOn) {
    snprintf(buf, sizeof(buf), "Relais: %s", relayOn ? "an" : "aus");
    lv_label_set_text(label_relay, buf);
    gui_lastRelayOn = relayOn;
  }

  // 5) Optional: erzwinge sofortiges Redraw (falls auf dem Gerät sichtbar verzögert)
  // lv_refr_now(NULL) ist relativ teuer, aber für einzelne User-Events akzeptabel.
  lv_refr_now(NULL);
}


// ============================================================================
// DS18B20: Temperaturmessung starten (non-blocking)
// ============================================================================
/**
 * @brief Startet eine neue Temperaturmessung wenn das Intervall abgelaufen ist
 * 
 * Die Messung erfolgt asynchron (non-blocking):
 * 1. Diese Funktion startet die Messung (requestTemperatures)
 * 2. Nach <tempConversionDelay> Millisekunden kann das Ergebnis mit readTemperatureIfReady() abgeholt werden
 * 
 * Intervall: <tempRequestInterval> Millisekunden
 */
void startTemperatureRequestIfNeeded() {
  unsigned long now = millis();
  
  // Prüfen ob Messintervall abgelaufen und keine Messung läuft
  if (!tempRequested && (now - lastRequest >= tempRequestInterval)) {
    sensors.setWaitForConversion(false); // Asynchroner Modus
    sensors.requestTemperatures();       // Messung starten
    //DPRINTLN("Temp angefragt");
    tempRequested = true;
    lastRequest = now;
  }
}

// ============================================================================
// DS18B20: Temperaturmessung auslesen (non-blocking)
// ============================================================================
/**
 * @brief Liest das Temperatur-Messergebnis aus wenn die Wandlung fertig ist
 * 
 * Wartet <tempConversionDelay> Millisekunden nach der Messanforderung (je nach A/D-Wandlungszeit der gewählten Genauigkeit)
 * Startet den Timer automatisch wenn Solltemperatur erreicht wurde
 */
void readTemperatureIfReady() {
  unsigned long now = millis();
  
  // Prüfen ob Messung läuft und Wandlungszeit abgelaufen
  if (tempRequested && (now - lastRequest >= tempConversionDelay)) {
    float t = sensors.getTempC(sensorAddress);  // Temperatur auslesen
    tempRequested = false;
    
    if (t != DEVICE_DISCONNECTED_C) {
      // Gültige Temperatur empfangen
      istTemp = t;
      
      // Timer automatisch starten wenn Solltemperatur erreicht
      if (startTime == 0 && istTemp >= sollTemp) {
        startTime = millis();
      }
    } else {
      // Sensor nicht erreichbar
      DPRINTLN("Sensor disconnected!");
    }
  }
}

// ============================================================================
// Konfiguration aus NVS laden
// ============================================================================
/**
 * @brief Lädt alle gespeicherten Einstellungen aus dem Flash-Speicher (NVS)
 * 
 * Geladene Werte:
 * - WiFi SSID und Passwort
 * - MQTT Server, Port, User, Passwort, Topic-Basis
 * - Solltemperatur (Standard: 40°C)
 * 
 * preferences.begin() muss bereits ausgeführt sein!
 */
void loadConfig() {
  preferences.begin("waermeschrank", false); // Namespace öffnen (read/write)
  // WiFi-Konfiguration laden
  wifi_ssid = preferences.getString("wifi_ssid", "");
  wifi_password = preferences.getString("wifi_pass", "");
  
  // MQTT-Konfiguration laden
  mqtt_server = preferences.getString("mqtt_server", "");
  mqtt_port = preferences.getInt("mqtt_port", 1883);
  mqtt_user = preferences.getString("mqtt_user", "");
  mqtt_password = preferences.getString("mqtt_pass", "");
  mqtt_topic_base = preferences.getString("mqtt_topic", "waermeschrank");
  
  // Solltemperatur laden (Standard: 40°C)
  sollTemp = preferences.getInt("soll_temp", 40);
  lastSavedSollTemp = sollTemp; // Initialen Wert merken für Änderungserkennung
  
  preferences.end();

  // Debug-Ausgabe
  DPRINTLN("Konfiguration geladen:");
  DPRINTLN("WiFi SSID: " + wifi_ssid);
  DPRINTLN("MQTT Server: " + mqtt_server);
  DPRINTF("Solltemperatur: %d °C\n", sollTemp);
}

// ============================================================================
// Konfiguration in NVS speichern
// ============================================================================
/**
 * @brief Speichert Einstellungen im Flash-Speicher (NVS)
 * 
 * @param mitMQTT true = Alle Werte speichern (WiFi + MQTT + Solltemperatur)
 *                false = Nur Solltemperatur speichern (wenn geändert)
 * 
 * Diese Funktion wird aufgerufen:
 * - Automatisch alle 5 Minuten (mitMQTT=false), aber nur falls Solltemperatur geändert
 * - Nach Config-Mode Änderungen (mitMQTT=true) - speichert alles
 * 
 * WICHTIG: NVS Flash hat begrenzte Schreibzyklen (~100.000)
 * Daher nur speichern wenn sich wirklich etwas geändert hat!
 */
void saveConfig(bool mitMQTT) {
  preferences.begin("waermeschrank", false);
  // --------------------------------------------------------------------------
  // WiFi/MQTT-Konfiguration speichern (nur wenn mitMQTT=true)
  // --------------------------------------------------------------------------
  if (mitMQTT) {
    preferences.putString("wifi_ssid", wifi_ssid);
    preferences.putString("wifi_pass", wifi_password);
    preferences.putString("mqtt_server", mqtt_server);
    preferences.putInt("mqtt_port", mqtt_port);
    preferences.putString("mqtt_user", mqtt_user);
    preferences.putString("mqtt_pass", mqtt_password);
    preferences.putString("mqtt_topic", mqtt_topic_base);
    DPRINTLN("MQTT-Konfiguration gespeichert");
  }
  // --------------------------------------------------------------------------
  // Solltemperatur speichern (nur wenn geändert - Flash-schonend!)
  // --------------------------------------------------------------------------
  if (sollTemp != lastSavedSollTemp) {
    preferences.putInt("soll_temp", sollTemp);
    lastSavedSollTemp = sollTemp; // Neuen Wert merken
    DPRINTF("SollTemp gespeichert: %d C\n", sollTemp);
  }
  preferences.end();
}

// ============================================================================
// Konfigurations-Modus (WiFi Access Point + Webserver)
// ============================================================================
/**
 * @brief Startet einen Access Point und Webserver für die Konfiguration
 * 
 * Funktion:
 * - Erstellt WiFi Access Point "Waermeschrank" (Passwort: config123)
 * - Startet Webserver unter http://192.168.4.1
 * - Zeigt Anleitung auf dem Display
 * - Kann per Tastendruck beendet werden
 * 
 * Nach dem Speichern startet das Gerät automatisch neu!
 */
void enterConfigMode() {
  DPRINTLN("Starte Config-Modus...");
  
  // --------------------------------------------------------------------------
  // Display vorbereiten (direkt via TFT_eSPI, nicht LVGL!)
  // Grund: LVGL kann bei WiFi-Operationen abstürzen
  // --------------------------------------------------------------------------
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  
  // Überschrift "zentriert"
  tft.setCursor(60, 10);
  tft.println("CONFIG MODE");
  
  // Zugangsdaten anzeigen
  tft.setCursor(10, 40);
  tft.println("SSID: Waermeschrank");
  tft.setCursor(10, 65);
  tft.println("Pass: config123");
  tft.setCursor(10, 90);
  tft.println("URL:  192.168.4.1");
  
  // Trennlinie
  tft.drawLine(0, 115, 280, 115, TFT_WHITE);
  
  // Hinweis zum Beenden (gelb hervorgehoben)
  tft.setCursor(10, 135);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.println("Taste -> Beenden");
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  
  // Status-Bereich
  tft.setCursor(10, 165);
  tft.setTextSize(1);
  tft.println("Starte Access Point...");
  delay(500);
  DPRINTLN("Starte Access Point...");
  
  // --------------------------------------------------------------------------
  // WiFi Access Point starten
  // --------------------------------------------------------------------------
  WiFi.mode(WIFI_AP);
  WiFi.setTxPower(WIFI_POWER_8_5dBm); // Reduzierte Sendeleistung für Stabilität - Benutzung erfolgt sowieso in unmittelbarer Nähe zum Gerät
  bool apStarted = WiFi.softAP("Waermeschrank", "config123");
  
  delay(500);
  
  if (apStarted) {
    // Access Point erfolgreich gestartet
    IPAddress IP = WiFi.softAPIP();
    tft.setCursor(10, 165);
    tft.print("AP gestartet: ");
    tft.println(IP);
    DPRINT("AP IP: ");
    DPRINTLN(IP);
  } else {
    // Fehler beim Starten
    tft.setCursor(10, 165);
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.println("AP FEHLER!");
    DPRINTLN("AP Start fehlgeschlagen!");
  }
  
  // --------------------------------------------------------------------------
  // Webserver konfigurieren und starten
  // --------------------------------------------------------------------------
  server.on("/save", HTTP_POST, handleSave); // Speichern-Route
  server.begin();
  tft.setCursor(10, 180);
  tft.println("Webserver aktiv");
  DPRINTLN("Webserver gestartet");
  DPRINTLN("Taste druecken zum Beenden");
  
  // Encoder für Config-Modus neu initialisieren (falls nötig)
  bool buttonPressed = false;
  
  // --------------------------------------------------------------------------
  // Haupt-Loop: Webserver bedienen und auf Tastendruck warten
  // --------------------------------------------------------------------------
  unsigned long lastUpdate = 0;
  unsigned long configStartTime = millis();
  int clientCount = 0;
  
  while (!buttonPressed) {
    server.handleClient();            // HTTP-Anfragen verarbeiten
    
    // Button-Status prüfen
    if (rotaryEncoder.isEncoderButtonDown()) {
      // Warte bis Button losgelassen wird
      while (rotaryEncoder.isEncoderButtonDown()) {
        delay(10);
      }
      buttonPressed = true;
      
      // Exit-Animation auf Display zeigen
      tft.fillScreen(TFT_GREEN);
      tft.setTextColor(TFT_BLACK, TFT_GREEN);
      tft.setTextSize(3);
      tft.setCursor(10, 100);
      tft.println("Beende");
      tft.setCursor(10, 130);
      tft.println("Config");
      
      DPRINTLN("Config-Mode durch Taste beendet");
      delay(1000);
      break; // Schleife verlassen
    }
    
    // --------------------------------------------------------------------------
    // Alle 5 Sekunden: Display-Status aktualisieren
    // --------------------------------------------------------------------------
    if (millis() - lastUpdate > 5000) {
      // Anzahl verbundener Clients anzeigen
      int connectedClients = WiFi.softAPgetStationNum();
      if (connectedClients != clientCount) {
        clientCount = connectedClients;
        char buf[50];
        snprintf(buf, sizeof(buf), "Clients: %d", clientCount);
        
        // Bereich löschen und neu zeichnen
        tft.fillRect(10, 220, 200, 10, TFT_BLACK);
        tft.setCursor(10, 205);
        tft.setTextSize(2);
        tft.println(buf);
      }
      
      lastUpdate = millis();
    }
    
    delay(10); // Kurze Pause
  }
  // Nach Verlassen der Schleife folgt ESP.restart() in loop()
}

// ============================================================================
// Webserver: Startseite mit Konfigurations-Formular
// ============================================================================
/**
 * @brief Erzeugt HTML-Seite mit Formular für WiFi/MQTT-Konfiguration
 * 
 * Die Seite enthält Eingabefelder für:
 * - WiFi: SSID und Passwort
 * - MQTT: Server, Port, User, Passwort, Topic-Basis
 * 
 * Beim Absenden wird POST-Request an /save gesendet
 */
void handleRoot() {
  // HTML-Header und Styles (mit F() Makro im Flash gespeichert = RAM-sparend)
  String html = F("<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1'><title>Config</title>");
  html += F("<style>body{font-family:Arial;margin:20px;background:#f0f0f0;}");
  html += F(".c{max-width:500px;margin:0 auto;background:#fff;padding:20px;border-radius:10px;}");
  html += F("h1{color:#333;border-bottom:2px solid #007bff;padding-bottom:10px;}");
  html += F("input{width:100%;padding:10px;margin:8px 0;box-sizing:border-box;border:1px solid #ddd;border-radius:4px;}");
  html += F("button{width:100%;padding:12px;margin-top:20px;background:#007bff;color:#fff;border:none;border-radius:4px;font-size:16px;}");
  html += F("label{display:block;margin-top:10px;color:#555;font-weight:bold;}</style>");
  html += F("</head><body><div class='c'><h1>Wärmeschrank Config</h1>");
  
  // Formular mit aktuellen Werten vorausgefüllt
  html += F("<form action='/save' method='POST'>");
  
  // WiFi-Bereich
  html += F("<h2>WiFi</h2>");
  html += F("<label>SSID:</label><input type='text' name='wifi_ssid' value='");
  html += wifi_ssid;
  html += F("' required><label>Passwort:</label><input type='password' name='wifi_pass' value='");
  html += wifi_password;
  html += F("'>");
  
  // MQTT-Bereich
  html += F("<h2>MQTT</h2><label>Server:</label><input type='text' name='mqtt_server' value='");
  html += mqtt_server;
  html += F("' placeholder='192.168.1.100'><label>Port:</label><input type='number' name='mqtt_port' value='");
  html += String(mqtt_port);
  html += F("'><label>User:</label><input type='text' name='mqtt_user' value='");
  html += mqtt_user;
  html += F("'><label>Pass:</label><input type='password' name='mqtt_pass' value='");
  html += mqtt_password;
  html += F("'><label>Topic:</label><input type='text' name='mqtt_topic' value='");
  html += mqtt_topic_base;
  html += F("'>");
  
  // Speichern-Button
  html += F("<button type='submit'>Speichern</button></form></div></body></html>");
  
  server.send(200, F("text/html"), html);
}

// ============================================================================
// Webserver: Formulardaten speichern und Neustart
// ============================================================================
/**
 * @brief Verarbeitet POST-Request mit neuen Konfigurationsdaten
 * 
 * Ablauf:
 * 1. Formulardaten aus HTTP-Request auslesen
 * 2. Werte in globale Variablen schreiben
 * 3. In NVS Flash speichern (saveConfig mit mitMQTT=true)
 * 4. Bestätigungs-HTML senden
 * 5. ESP32 neu starten (damit neue WiFi-Config aktiv wird)
 */
void handleSave() {
  // --------------------------------------------------------------------------
  // Display-Feedback während Speichervorgang
  // --------------------------------------------------------------------------
  tft.fillRect(0, 210, 280, 30, TFT_GREEN);
  tft.setTextColor(TFT_BLACK, TFT_GREEN);
  tft.setCursor(10, 215);
  tft.println("Speichere Daten...");
  
  // --------------------------------------------------------------------------
  // Formulardaten auslesen und in Variablen schreiben
  // --------------------------------------------------------------------------
  if (server.hasArg("wifi_ssid")) wifi_ssid = server.arg("wifi_ssid");
  if (server.hasArg("wifi_pass")) wifi_password = server.arg("wifi_pass");
  if (server.hasArg("mqtt_server")) mqtt_server = server.arg("mqtt_server");
  if (server.hasArg("mqtt_port")) mqtt_port = server.arg("mqtt_port").toInt();
  if (server.hasArg("mqtt_user")) mqtt_user = server.arg("mqtt_user");
  if (server.hasArg("mqtt_pass")) mqtt_password = server.arg("mqtt_pass");
  if (server.hasArg("mqtt_topic")) mqtt_topic_base = server.arg("mqtt_topic");
  
  // --------------------------------------------------------------------------
  // Alle Werte in NVS Flash speichern
  // --------------------------------------------------------------------------
  saveConfig(true); // mitMQTT=true -> speichert WiFi, MQTT und Solltemperatur
  
  // --------------------------------------------------------------------------
  // Display-Feedback: Erfolgsmeldung
  // --------------------------------------------------------------------------
  tft.fillRect(0, 180, 280, 30, TFT_BLUE);
  tft.setTextColor(TFT_WHITE, TFT_BLUE);
  tft.setCursor(10, 215);
  tft.println("Gespeichert! Neustart...");
  
  // --------------------------------------------------------------------------
  // Bestätigungs-HTML an Browser senden
  // --------------------------------------------------------------------------
  String html = F("<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1'>");
  html += F("<style>body{font-family:Arial;text-align:center;margin:50px;background:#f0f0f0;}");
  html += F(".s{background:#fff;padding:40px;border-radius:10px;max-width:400px;margin:0 auto;}");
  html += F("h1{color:#28a745;}p{color:#555;font-size:18px;}</style></head><body><div class='s'>");
  html += F("<h1>Gespeichert!</h1><p>Neustart...</p><p>Verbinden Sie sich wieder mit Ihrem WiFi.</p>");
  html += F("</div></body></html>");
  server.send(200, F("text/html"), html);
  
  DPRINTLN("Config gespeichert, starte neu...");
  DFLUSH(); // Sicherstellen dass Nachricht ausgegeben wird
  
  // --------------------------------------------------------------------------
  // 2 Sekunden warten (damit HTML-Seite geladen werden kann)
  // dann ESP32 neu starten
  // --------------------------------------------------------------------------
  delay(2000);
  ESP.restart();
}

// ============================================================================
// WiFi: Verbindung zum konfigurierten Netzwerk herstellen
// ============================================================================
/**
 * @brief Stellt Verbindung zum WiFi-Netzwerk her
 * 
 * - Versucht 20x im Abstand von 500ms zu verbinden (= 10 Sekunden max.)
 * - Zeigt Fortschritt auf dem Display
 * - Bei Erfolg: IP-Adresse anzeigen
 * - Bei Fehler: Fehlermeldung anzeigen
 * 
 * WICHTIG: Serielle Verbindung kann während WiFi-Operationen instabil werden!
 *          (z.B. weil WiFi höheren Strombedarf hat -> dickes USB-Kabel 
 *           und USB-Anschluss mit ausreichend Strom verwenden)
 */
void connectWiFi() {
  // Debug-Ausgabe vor WiFi-Start
  DPRINTLN("Verbinde mit WiFi: " + wifi_ssid);
  delay(100);
  
  // Display-Info während Verbindungsaufbau
  if (label_status) {
    lv_label_set_text(label_status, "WiFi verbindet...");
    lv_refr_now(NULL); // Sofort zeichnen
  }
  
  // --------------------------------------------------------------------------
  // WiFi im Station-Modus starten
  // --------------------------------------------------------------------------
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifi_ssid.c_str(), wifi_password.c_str());
  
  // --------------------------------------------------------------------------
  // Warten auf Verbindung (max. 20 Versuche = 10 Sekunden)
  // --------------------------------------------------------------------------
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    attempts++;
    
    // Display-Update alle 2 Versuche (= jede Sekunde)
    if (label_status && attempts % 2 == 0) {
      char buf[40];
      snprintf(buf, sizeof(buf), "WiFi... (%d/20)", attempts);
      lv_label_set_text(label_status, buf);
      lv_refr_now(NULL);
    }
  }
  
  // Kurze Pause nach WiFi-Operation (für Stabilität)
  delay(500);
  
  // --------------------------------------------------------------------------
  // Ergebnis auswerten und anzeigen
  // --------------------------------------------------------------------------
  if (WiFi.status() == WL_CONNECTED) {
    // Verbindung erfolgreich
    DPRINTLN("\n=== WiFi verbunden! ===");
    DPRINT("IP: ");
    DPRINTLN(WiFi.localIP());
    
    if (label_status) {
      lv_label_set_text(label_status, "WiFi OK");
      lv_refr_now(NULL);
    }
  } else {
    // Verbindung fehlgeschlagen
    DPRINTLN("\n=== WiFi FEHLER ===");
    
    if (label_status) {
      lv_label_set_text(label_status, "WiFi FEHLER!");
      lv_refr_now(NULL);
    }
  }
}

// ============================================================================
// MQTT: Verbindung zum Broker herstellen (mit Retry-Logik)
// ============================================================================
/**
 * @brief Versucht Verbindung zum MQTT-Broker herzustellen
 * 
 * Retry-Logik:
 * - Maximal 10 Verbindungsversuche
 * - Bei Erreichen des Limits wird MQTT dauerhaft deaktiviert (bis Neustart)
 * - Zwischen Versuchen wird 1 Minute gewartet (mqttReconnectCooldown)
 * 
 * Bei erfolgreicher Verbindung werden folgende Topics abonniert:
 * - [base]/set_temp_soll : Empfang neuer Solltemperatur
 * - [base]/set_laufzeit : Empfang neuer Laufzeit (in Minuten)
 */
void reconnectMQTT() {
  if (mqttClient.connected()) return; // Bereits verbunden
  
  // --------------------------------------------------------------------------
  // Prüfen ob maximale Anzahl Versuche erreicht
  // --------------------------------------------------------------------------
  if (mqttReconnectAttempts >= maxMqttReconnectAttempts) {
    if (mqttEnabled) {
      // MQTT dauerhaft deaktivieren (bis Neustart)
      DPRINTLN("\n=== MQTT DEAKTIVIERT ===");
      DPRINTLN("Zu viele Verbindungsfehler - Geraet arbeitet offline");
      mqttEnabled = false;
      
      if (label_status) {
        lv_label_set_text(label_status, "WiFi OK, MQTT aus");
        lv_refr_now(NULL);
      }
    }
    return;
  }
  
  // --------------------------------------------------------------------------
  // Verbindungsversuch vorbereiten
  // --------------------------------------------------------------------------
  lastMqttReconnectAttempt = millis();
  mqttReconnectAttempts++;
  
  DPRINTF("MQTT Verbindungsversuch %d/%d...\n", 
                mqttReconnectAttempts, maxMqttReconnectAttempts);
  
  // Status auf Display anzeigen
  if (label_status) {
    char buf[50];
    snprintf(buf, sizeof(buf), "MQTT try %d/%d", 
             mqttReconnectAttempts, maxMqttReconnectAttempts);
    lv_label_set_text(label_status, buf);
    lv_refr_now(NULL);
  }
  
  // --------------------------------------------------------------------------
  // MQTT Verbindung aufbauen
  // --------------------------------------------------------------------------
  String clientId = mqtt_topic_base; // Client-ID = Topic-Basis
  
  bool connected;
  if (mqtt_user.length() > 0) {
    // Mit Authentifizierung
    connected = mqttClient.connect(clientId.c_str(), 
                                   mqtt_user.c_str(), 
                                   mqtt_password.c_str());
  } else {
    // Ohne Authentifizierung
    connected = mqttClient.connect(clientId.c_str());
  }
  
  delay(100); // Kurze Pause nach Verbindungsversuch
  
  // --------------------------------------------------------------------------
  // Ergebnis auswerten
  // --------------------------------------------------------------------------
  if (connected) {
    // Verbindung erfolgreich
    DPRINTLN("=== MQTT verbunden! ===");
    mqttReconnectAttempts = 0; // Fehlerzähler zurücksetzen
    
    // Topics abonnieren für Fernsteuerung
    String topic_temp = mqtt_topic_base + "/set_temp_soll";
    String topic_zeit = mqtt_topic_base + "/set_laufzeit";
    
    bool sub1 = mqttClient.subscribe(topic_temp.c_str());
    bool sub2 = mqttClient.subscribe(topic_zeit.c_str());
    
    DPRINTF("Subscribe %s: %s\n", topic_temp.c_str(), sub1 ? "OK" : "FEHLER");
    DPRINTF("Subscribe %s: %s\n", topic_zeit.c_str(), sub2 ? "OK" : "FEHLER");
    
    if (label_status) {
      lv_label_set_text(label_status, "WiFi+MQTT OK");
      lv_refr_now(NULL);
    }
  } else {
    // Verbindung fehlgeschlagen
    DPRINTF("MQTT Fehler rc=%d (%d/%d Versuche)\n", 
                  mqttClient.state(), 
                  mqttReconnectAttempts, 
                  maxMqttReconnectAttempts);
    
    if (mqttReconnectAttempts >= maxMqttReconnectAttempts) {
      DPRINTLN("Maximale Versuche erreicht - MQTT wird deaktiviert");
    } else {
      DPRINTF("Naechster Versuch in %lu Sekunden\n", 
                    mqttReconnectCooldown / 1000);
    }
    
    if (label_status) {
      char buf[50];
      snprintf(buf, sizeof(buf), "MQTT Err:%d %d/%d", 
               mqttClient.state(), 
               mqttReconnectAttempts, 
               maxMqttReconnectAttempts);
      lv_label_set_text(label_status, buf);
      lv_refr_now(NULL);
    }
  }
}

// ============================================================================
// MQTT: Daten zum Broker senden
// ============================================================================
/**
 * @brief Sendet aktuelle Messwerte und Status zum MQTT-Broker
 * 
 * Gesendete Topics:
 * - [base]/relay : Relaisstatus ("ON" oder "OFF")
 * - [base]/temp_soll : Solltemperatur in °C
 * - [base]/temp_ist : Ist-Temperatur in °C (mit 1 Dezimalstelle)
 * 
 * Wird aufgerufen:
 * - Alle x Minuten automatisch (siehe mqttPublishInterval in loop())
 * 
 * Die Funktion aktiviert auch die MQTT-Sendeanzeige für 2 Sekunden
 */
void publishMQTTData() {
  if (!mqttClient.connected()) return; // Nur wenn verbunden
  bool success;			       // Zum Speichern der Rückmeldungen der .publish-Routinen
  // --------------------------------------------------------------------------
  // MQTT-Sendeanzeige aktivieren (wird nach 2s in loop() zurückgesetzt)
  // --------------------------------------------------------------------------
  mqttSending = true;
  mqttSendingStartTime = millis();
  
  char buf[100];
  
  // --------------------------------------------------------------------------
  // Relaisstatus publishen
  // --------------------------------------------------------------------------
  snprintf(buf, sizeof(buf), "%s", relayOn ? "ON" : "OFF");
  success = mqttClient.publish((mqtt_topic_base + "/relay").c_str(), buf);
  
  // --------------------------------------------------------------------------
  // Solltemperatur publishen
  // --------------------------------------------------------------------------
  snprintf(buf, sizeof(buf), "%d", sollTemp);
  success = mqttClient.publish((mqtt_topic_base + "/temp_soll").c_str(), buf);
  
  // --------------------------------------------------------------------------
  // Ist-Temperatur publishen (mit 1 Dezimalstelle)
  // --------------------------------------------------------------------------
  snprintf(buf, sizeof(buf), "%.1f", istTemp);
  success = mqttClient.publish((mqtt_topic_base + "/temp_ist").c_str(), buf);

  // --------------------------------------------------------------------------
  // Soll-Laufzeit publishen (in Millisekunden)
  // --------------------------------------------------------------------------
  snprintf(buf, sizeof(buf), "%lu", laufzeit);
  success = mqttClient.publish((mqtt_topic_base + "/sollLaufzeit").c_str(), buf);

  // --------------------------------------------------------------------------
  // Rest-Laufzeit publishen (in Millisekunden)
  // --------------------------------------------------------------------------
  unsigned long rest_ms;
  if (startTime == 0) {
    // Timer noch nicht gestartet - Gesamtzeit anzeigen
    rest_ms = laufzeit;
  } else {
    // Timer läuft - Restzeit berechnen
    unsigned long elapsed = millis() - startTime;
    rest_ms = (elapsed >= laufzeit) ? 0 : (laufzeit - elapsed);
  }
  snprintf(buf, sizeof(buf), "%lu", rest_ms);
  success = mqttClient.publish((mqtt_topic_base + "/restLaufzeit").c_str(), buf);
  //DPRINTF("MQTT: Relay %s, Soll-Temp %d, Ist-Temp %.1f, Solllaufzeit %lu, Rest %lu", relayOn ? "ON" : "OFF", sollTemp, istTemp, laufzeit, rest_ms);
}

// ============================================================================
// MQTT: Eingehende Nachrichten verarbeiten (Callback)
// ============================================================================
/**
 * @brief Wird aufgerufen wenn eine MQTT-Nachricht empfangen wurde
 * 
 * @param topic Topic der Nachricht (z.B. "waermeschrank/set_temp_soll")
 * @param payload Nutzdaten der Nachricht (Byte-Array)
 * @param length Länge der Nutzdaten
 * 
 * Verarbeitete Topics:
 * 
 * 1. [base]/set_temp_soll
 *    - Setzt neue Solltemperatur (Bereich: 20-60°C)
 *    - Synchronisiert Encoder (nur wenn im Temperatur-Modus)
 *    - Sendet Bestätigung zurück an [base]/temp_soll
 * 
 * 2. [base]/set_laufzeit
 *    - Setzt neue Laufzeit in Minuten (Bereich: 0-5940 min = 0-99h)
 *    - Synchronisiert Encoder (nur wenn im Laufzeit-Modus)
 *    - Sendet Bestätigung zurück an [base]/laufzeit_min
 * 
 * WICHTIG: Änderungen werden NICHT sofort gespeichert!
 * Speicherung erfolgt automatisch alle 5 Minuten (siehe Auto-Save in loop())
 */
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // --------------------------------------------------------------------------
  // Payload (Byte-Array) in String konvertieren
  // --------------------------------------------------------------------------
  String message = "";
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  String topicStr = String(topic);
  
  DPRINTF("MQTT empfangen: %s = %s\n", topic, message.c_str());
  
  // --------------------------------------------------------------------------
  // Topic 1: Solltemperatur setzen
  // --------------------------------------------------------------------------
  if (topicStr == mqtt_topic_base + "/set_temp_soll") {
    int newTemp = message.toInt();
    
    // Plausibilitätsprüfung: 20-60°C erlaubt
    if (newTemp >= 20 && newTemp <= 60) {
      sollTemp = newTemp;
      
      // Encoder synchronisieren (nur wenn im Temperatur-Modus)
      // Verhindert dass Encoder-Wert springt wenn User gerade Laufzeit einstellt
      if (encoderMode == 0) {
        rotaryEncoder.setEncoderValue(sollTemp);
      }
      
      DPRINTF("Solltemperatur via MQTT geaendert: %d C\n", sollTemp);
      
      // Bestätigung zurücksenden
      char buf[10];
      snprintf(buf, sizeof(buf), "%d", sollTemp);
      bool success = mqttClient.publish((mqtt_topic_base + "/temp_soll").c_str(), buf);
      DPRINTF("Temp Soll: %s -> %s\n", buf, success ? "OK" : "FEHLER");
      // HINWEIS: Speicherung erfolgt automatisch beim nächsten Auto-Save
      // (alle 5 Minuten in loop())
    } else {
      // Ungültiger Wert
      DPRINTF("FEHLER: Solltemperatur %d ausserhalb 20-60 C\n", newTemp);
    }
  }
  
  // --------------------------------------------------------------------------
  // Topic 2: Laufzeit setzen (in Minuten)
  // --------------------------------------------------------------------------
  else if (topicStr == mqtt_topic_base + "/set_laufzeit") {
    int newMinutes = message.toInt();
    
    // Plausibilitätsprüfung: 0-5940 Minuten (= 0-99 Stunden)
    if (newMinutes >= 0 && newMinutes <= 5940) {
      laufzeit = newMinutes * 60000UL; // Minuten in Millisekunden umrechnen
      
      // Encoder synchronisieren (nur wenn im Laufzeit-Modus)
      if (encoderMode == 1) {
        rotaryEncoder.setEncoderValue(laufzeit / LAUFZEIT_STEP_MS);
      }
      
      DPRINTF("Laufzeit via MQTT geaendert: %d Minuten (%lu ms)\n", 
                    newMinutes, laufzeit);
      
      // Bestätigung zurücksenden
      char buf[20];
      snprintf(buf, sizeof(buf), "%d", newMinutes);
      bool success = mqttClient.publish((mqtt_topic_base + "/laufzeit_min").c_str(), buf);
      DPRINTF("Laufzeit: %s -> %s\n", buf, success ? "OK" : "FEHLER");
      // HINWEIS: Laufzeit wird NICHT gespeichert (nur Solltemperatur wird persistiert)
    } else {
      // Ungültiger Wert
      DPRINTF("FEHLER: Laufzeit %d Minuten ausserhalb 0-5940 min (0-99h)\n", 
                    newMinutes);
    }
  }
  // Weitere Topics können hier hinzugefügt werden
}

// ============================================================================
// ENDE DES PROGRAMMS
// ============================================================================
