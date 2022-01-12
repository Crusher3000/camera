#include "esp_camera.h"                                           // Importation de l'index pour la caméra
#include <WiFi.h>                                                 // Bibliothèque permettant la gestion Wi-Fi de notre ESP32
#include <Wire.h>                                                 // Bibliothèque I²C pour l'OLED
#include <Adafruit_GFX.h>                                         // Bibliothèque incluant des fonctions graphique pour l'OLED
#include <Adafruit_SSD1306.h>                                     // Bibliothèque reprenant l'ensenmbles des fonctions d'utilisation de l'OLED
#include <PubSubClient.h>                                         // Bibliothèque fournissant des fonctions de messageries MQTT
#include <Adafruit_NeoPixel.h>        

#include <Adafruit_GrayOLED.h>                            // Bibliothèque pour la gestion de ruban LEDs adressables

//  OLED
#define SCREEN_WIDTH 128                                          // Largeur en pixel de l'OLED
#define SCREEN_HEIGHT 64                                          // Hauteur en pixel de l'OLED
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1); // Création de notre OLED SSD1306 pour communiquer avec l'I²C (SDA, SCL)

// Sélection du modèle de caméra
//#define CAMERA_MODEL_WROVER_KIT // Has PSRAM
//#define CAMERA_MODEL_ESP_EYE // Has PSRAM
//#define CAMERA_MODEL_M5STACK_PSRAM // Has PSRAM
//#define CAMERA_MODEL_M5STACK_V2_PSRAM // M5Camera version B Has PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE // Has PSRAM
//#define CAMERA_MODEL_M5STACK_ESP32CAM // No PSRAM
#define CAMERA_MODEL_AI_THINKER // Has PSRAM                      // Sélection du modèle de caméra utilisé dans notre cas
//#define CAMERA_MODEL_TTGO_T_JOURNAL // No PSRAM

#include "camera_pins.h"                                          // Importation des pins de l'ESP32-CAM en fonction du modèle choisi

// MOTEUR PAS À PAS
int motorPin1 = 13;                                               //
int motorPin2 = 12;                                               //
char sens1[2] = "0";                                              // Flag (set par défaut au lancement (après avec Node RED)) pour activer la rotation de la caméra dans un sens
char sens2[2] = "0";                                              // Flag (set par défaut au lancement (après avec Node RED)) pour activer la rotation de la caméra dans l'autre sens

// RESEAUX & COMMUNNICATION
const char* ssid = "AntoDB";                                      // SSID du réseau Wi-Fi auquel il faut se connecter
const char* password = "3049oQ7%";                                // Mot de passe du réseau Wi-Fi auquel il faut se connecter
const char* mqtt_server = "192.168.137.189";                      // Adresse IP du server MQTT auquel il faut se connecter
WiFiClient espClient;                                             // Création de "espClient" propre au client Wi-Fi
PubSubClient client(espClient);                                   // Création d'un système de messagerie MQTT du client

// RUBAN LEDS
#define LED_PIN 16
#define LED_COUNT 15
Adafruit_NeoPixel strip (LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
int red = 0;                                                          // Valeur de la couleur rouge du ruban led
int green = 0;                                                        // Valeur de la couleur verte du ruban led
int blue = 0;                                                         // Valeur de la couleur bleue du ruban led

void printToOLED(int x, int y,  char *message){                   // Fonction affichant un message dans l'OLED à une cetraine position
  display.setCursor(x, y);                                        // On place le cursor du message aux coordonner X Y avant de l'afficher
  display.setTextColor(WHITE,BLACK);                              // On superpose les textes si jamais en les affichants en blanc avec fond noir (pour "effacer" les données)
  display.print(message);                                         // On place ce message sur ce cursor
  display.display();                                              // On affiche le tout
}

void startCameraServer();                                         // Fonction de démarrage du web serveur pour la caméra

void callback(char* topic, byte* payload, unsigned int length) {  // Fonction de Callback quand un message MQTT arrive sur un topic (subscribe)
  Serial.print("Message arrived [");                              // Imprime un message dans la console
  Serial.print(topic);                                            // On affiche de quel topic il s'agit
  Serial.print("] ");                                             // Imprime un message dans la console
  for (int i = 0; i < length; i++) {                              // On boucle tous les caractères reçus sur le topic
    Serial.print((char)payload[i]);                               // On affiche un à un tous les caractères reçus
  }
  Serial.println();                                               // Imprime un retour à la ligne dans la console

  char buffer1[length+1];                                         // On crée une variable local de buffer
  if (String(topic) == "led_red") {
    for (int i = 0; i < length+1; i++) {
      buffer1[i] = (char)payload[i];
    }
    red = atoi(buffer1);
  }
  if (String(topic) == "led_green") {
    for (int i = 0; i < length+1; i++) {
      buffer1[i] = (char)payload[i];
    }
    green = atoi(buffer1);
  }
  if (String(topic) == "led_blue") {
    for (int i = 0; i < length+1; i++) {
      buffer1[i] = (char)payload[i];
    }
    blue = atoi(buffer1);
  }

  if (String(topic) == "moteur_cam1") {
    for (int i = 0; i < length+1; i++) {
      buffer1[i] = (char)payload[i];
    }
    sens1[0] = buffer1[0];
  }
  if (String(topic) == "moteur_cam2") {
    for (int i = 0; i < length+1; i++) {
      buffer1[i] = (char)payload[i];
    }
    sens2[0] = buffer1[0];
  }
}

void reconnect() {                                                // Fonction effectuant des tentative de reconnexion 
  while (!client.connected()) {                                   // Si le client se connecte
    Serial.print("Attempting MQTT connection...");                // On attent la conexion MQTT
    if (client.connect("ESP32-CAM")) {                            // Si le client se connecte en étant appelé "ESP32-CAM"
      Serial.println("connected");                                // On prévient qu'il est connecté
      
      client.subscribe("moteur_cam1");                            // On demande une lecture permanette asynchrone des données arrivant sur ce topic
      client.subscribe("moteur_cam2");                            // On demande une lecture permanette asynchrone des données arrivant sur ce topic
      client.subscribe("led_red");                                // On demande une lecture permanette asynchrone des données arrivant sur ce topic
      client.subscribe("led_green");                              // On demande une lecture permanette asynchrone des données arrivant sur ce topic
      client.subscribe("led_blue");                               // On demande une lecture permanette asynchrone des données arrivant sur ce topic
    } else {                                                      // Si la connexion rate
      for (int i = 0; i <= (LED_COUNT-1); i++) {
        strip.setPixelColor(i, 0, 0, 255);
      }
      strip.show();
      Serial.print("failed, rc=");                                // On affiche qu'il y a une erreur
      Serial.print(client.state());                               // On affiche le numéro de l'erreur (état)
      Serial.println(" try again in 5 seconds");                  // On affiche comme quoi on réessaye
      delay(5000);                                                // On attend 5 secondes avant de réessayer
    }
  }
}

void setup() {
  Serial.begin(115200);                                           // Début de communication série à 115200 bps
  Serial.setDebugOutput(true);
  Serial.println();                                               // Imprime un retour à la ligne dans la console

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  pinMode(14,INPUT_PULLUP); 
  pinMode(15,INPUT_PULLUP);
  Wire.begin(14,15);
  
  // OLED
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {                // Initialisation de l'OLED avec son adresse I²C (par défaut 0x3D pour un écran de taille 128x64)
    Serial.println(F("SSD1306 allocation failed"));               // Affichage d'une erreur dans la console si l'initialisation a échouée
    for(;;);                                                      // Arret du programme dans une boucle infinie
  }
  delay(2000);                                                    // Délais de 2 seconde
  display.clearDisplay();                                         // Nettoyage de tous les éléments graphics présents dans l'OLED
  display.drawRoundRect(0, 0, 90, 30, 3, WHITE);                  // Creation d'un rectangle (x1, y1, largeur, hauteur, rayon)
  display.drawRoundRect(94, 0, 33, 64, 3, WHITE);                 // Creation d'un rectangle (x1, y1, largeur, hauteur, rayon)
  display.drawRoundRect(0, 34, 90, 30, 3, WHITE);                 // Creation d'un rectangle (x1, y1, largeur, hauteur, rayon)
  display.setTextSize(1);                                         // Configuration de la taille de police
  display.setTextColor(WHITE, BLACK);                             // Configuration d'écriture de blanc sur noir dans l'OLED
  
  
  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if(psramFound()){
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1); // flip it back
    s->set_brightness(s, 1); // up the brightness just a bit
    s->set_saturation(s, -2); // lower the saturation
  }
  // drop down frame size for higher initial frame rate
  s->set_framesize(s, FRAMESIZE_QVGA);

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

  // RUBAN LEDS
  strip.begin();
  strip.show();
  strip.setBrightness(10);
  
  // COMMUNICATION WIFI
  WiFi.begin(ssid, password);                                     // Lancement de la connexion WiFi

  while (WiFi.status() != WL_CONNECTED) {                         // Tant que le microcontrôleur n'est pas connecté au WiFi
    for (int i = 0; i <= (LED_COUNT-1); i++) {
      strip.setPixelColor(i, 255, 0, 0);
    }
    strip.show();
    delay(500);                                                   // Délai de 500ms
    Serial.print(".");                                            // Imprimme un point dans la console
  }
  Serial.println("");                                             // Imprime un message dans la console
  Serial.println("WiFi connected");                               // Imprime un message dans la console

  // WEB SERVEUR CAMERA
  startCameraServer();                                            // Appel de la fonction de démarrage du web server pour la caméra

  Serial.print("Camera Ready! Use 'http://");                     // Imprime un message dans la console
  Serial.print(WiFi.localIP());                                   // Imprime l'ip dans la console
  Serial.println("' to connect");                                 // Imprime un message dans la console

  // COMMUNICATION
  client.setServer(mqtt_server, 1883);                            // On se connecte au serveur MQTT
  client.setCallback(callback);                                   // On synchronise aux messages entrant en MQTT

  // MOTEUR PAS A PAS
  pinMode(motorPin1, OUTPUT);                                     //
  pinMode(motorPin2, OUTPUT);                                     //
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  //sprintf(charsens1, "%i", sens1);
  printToOLED(5, 5, sens1);
  printToOLED(5, 17, sens2);

  if (sens1[0] == '1') {
    digitalWrite(motorPin1,LOW);
    digitalWrite(motorPin2,HIGH);
  }
  else if (sens2[0] == '1') {
    digitalWrite(motorPin2,LOW);
    digitalWrite(motorPin1,HIGH);
  }
  else {
    digitalWrite(motorPin1,LOW);
    digitalWrite(motorPin2,LOW);
  }

  for (int j = 0; j <= LED_COUNT; j++) {                                        // On boucle le nombre de LED(s) utilisées sur le ruban
    strip.setPixelColor(j, red, green, blue);                                   // On défini la couleur avec les valeurs définies
  }
  strip.show();                                                                 // On montre le changement (on actualise le ruban)*/
}