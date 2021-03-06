#include "esp_camera.h"                                           // Importation de l'index pour la caméra
#include <WiFi.h>                                                 // Bibliothèque permettant la gestion Wi-Fi de notre ESP32
#include <Wire.h>                                                 // Bibliothèque I²C pour l'OLED
#include <Adafruit_GFX.h>                                         // Bibliothèque incluant des fonctions graphique pour l'OLED
#include <Adafruit_SSD1306.h>                                     // Bibliothèque reprenant l'ensenmbles des fonctions d'utilisation de l'OLED
#include <PubSubClient.h>                                         // Bibliothèque fournissant des fonctions de messageries MQTT
#include <Adafruit_NeoPixel.h>                                    // Bibliothèque pour la gestion de ruban LEDs adressables
#include <Adafruit_GrayOLED.h>                                    // Bibliothèque pour la gestion de l'écran OLED

// OLED
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
int motorPin1 = 12;                                               // PIN 1A du driver L293D du moteur de rotation (0 ou 1)
int motorPin2 = 13;                                               // PIN 2A du driver L293D du moteur de rotation (0 ou 1)

char sens1[2] = "0";                                              // Flag (set par défaut au lancement (après avec Node RED)) pour activer la rotation de la caméra dans un sens
char sens2[2] = "0";                                              // Flag (set par défaut au lancement (après avec Node RED)) pour activer la rotation de la caméra dans l'autre sens

// RESEAUX & COMMUNNICATION
const char* ssid = "AntoDB";                                      // SSID du réseau Wi-Fi auquel il faut se connecter
const char* password = "3049oQ7%";                                // Mot de passe du réseau Wi-Fi auquel il faut se connecter
const char* mqtt_server = "192.168.137.189";                      // Adresse IP du server MQTT auquel il faut se connecter
WiFiClient espClient;                                             // Création de "espClient" propre au client Wi-Fi
PubSubClient client(espClient);                                   // Création d'un système de messagerie MQTT du client

// RUBAN LED
#define LED_PIN 16                                                // PIN de données du ruban LED WS2812B
#define LED_COUNT 16                                              // Nombre de LEDs utilisées
Adafruit_NeoPixel strip (LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
int red = 0;                                                      // Valeur de la couleur rouge du ruban led
int green = 0;                                                    // Valeur de la couleur verte du ruban led
int blue = 0;                                                     // Valeur de la couleur bleue du ruban led

char strRed[4];                                                   // Valeur textuelle de la couleur rouge du ruban led
char strGreen[4];                                                 // Valeur textuelle de la couleur verte du ruban led
char strBlue[4];                                                  // Valeur textuelle de la couleur bleue du ruban led

char str_battery[5];

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
  for (int i = 0; i < length+1; i++) {                            // On relis tous les caractères reçus sur le topic
    buffer1[i] = (char)payload[i];                                // On enregistre tous les caractères que l'on a besoin (uniquement)
  }

  if (String(topic) == "led_red") {                               // On vérifie si c'est le bon topic
    red = atoi(buffer1);                                          // Alors on le stocke dans la variable correspondante
    sprintf(strRed, "%3u   ", red);                               // Formatage de cette couleur en char
    printToOLED(82, 25, strRed);
  }
  if (String(topic) == "led_green") {                             // On vérifie si c'est le bon topic
    green = atoi(buffer1);                                        // Alors on le stocke dans la variable correspondante
    sprintf(strGreen, "%3u   ", green);                           // Formatage de cette couleur en char
    printToOLED(82, 37, strGreen);
  }
  if (String(topic) == "led_blue") {                              // On vérifie si c'est le bon topic
    blue = atoi(buffer1);                                         // Alors on le stocke dans la variable correspondante
    sprintf(strBlue, "%3u   ", blue);                             // Formatage de cette couleur en char
    printToOLED(82, 49, strBlue);
  }
  if (String(topic) == "moteur_cam1") {                           // On vérifie si c'est le bon topic
    sens1[0] = buffer1[0];                                        // Alors on le stocke dans la variable correspondante
  }
  if (String(topic) == "moteur_cam2") {                           // On vérifie si c'est le bon topic
    sens2[0] = buffer1[0];                                        // Alors on le stocke dans la variable correspondante
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
      for (int i = 0; i <= (LED_COUNT-1); i++) {                  // On boucle le nombre de LED(s) utilisées sur le ruban
        strip.setPixelColor(i, 0, 0, 255);                        // On défini la couleur avec une valeur fixe (ici en bleu)
      }
      strip.show();                                               // On montre le changement (on actualise le ruban)
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

  camera_config_t config;                                         // Reprend les données du fichier qui ont défini quel pin utiliser pour le modèle de caméra choisi
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
  display.drawRoundRect(0, 0, 127, 17, 3, WHITE);                 // Creation d'un rectangle (x1, y1, largeur, hauteur, rayon)
  display.drawRoundRect(64, 19, 63, 45, 3, WHITE);                // Creation d'un rectangle (x1, y1, largeur, hauteur, rayon)
  display.drawRoundRect(0, 19, 60, 30, 3, WHITE);                 // Creation d'un rectangle (x1, y1, largeur, hauteur, rayon)
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
  strip.begin();                                                  // Lancement de lu ruban LED
  strip.show();                                                   // On montre le changement (on actualise le ruban) (donc ici ça l'éteint s'il était par hasard encore allumé)
  strip.setBrightness(10);                                        // On défini la luminausité à 10/255
  
  // COMMUNICATION WIFI
  WiFi.begin(ssid, password);                                     // Lancement de la connexion WiFi

  while (WiFi.status() != WL_CONNECTED) {                         // Tant que le microcontrôleur n'est pas connecté au WiFi
    for (int i = 0; i <= (LED_COUNT-1); i++) {                    // On boucle le nombre de LED(s) utilisées sur le ruban
      strip.setPixelColor(i, 255, 0, 0);                          // On défini la couleur avec une valeur fixe (ici en rouge)
    }
    strip.show();                                                 // On montre le changement (on actualise le ruban)
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
  pinMode(motorPin1, OUTPUT);                                     // Défini que la pin 1A du driver L293D pour le moteur est une sortie
  pinMode(motorPin2, OUTPUT);                                     // Défini que la pin 2A du driver L293D pour le moteur est également une sortie
  printToOLED(5, 5, "16/01/2022");
  printToOLED(100, 5, "100%");
  printToOLED(5, 25, "Sens 1:");
  printToOLED(5, 37, "Sens 2:");
  printToOLED(70, 25, "R:");
  printToOLED(70, 37, "G:");
  printToOLED(70, 49, "B:");
  strRed[0] = '0';
  strGreen[0] = '0';
  strBlue[0] = '0';
}

void loop() {
  if (!client.connected()) {                                      // Si le client pour le MQTT en WiFi n'est pas connecté
    reconnect();                                                  // On appelle la fonction qui demande une reconnexion
  }
  client.loop();                                                  // Synchronisation du noeud (ESP32-CAM) au serveur MQTT

  if (millis()%500) {
    int battery = analogRead(0); // lire des valeur analogie sur la pin 0
    //int battery_map = map(battery, 2000,4500,0,100); // remis à l'échelle 
    //snprintf(str_battery,5,"%u",battery_map); // convertir le int en char* dans battery_map et le mettre dans str_battery
    sprintf(str_battery,"%u   ",battery);
    printToOLED(5, 52, "🎮💻");                                     // Affichage du sens 2 sur l'OLED
  }
  printToOLED(50, 25, sens1);                                     // Affichage du sens 1 sur l'OLED
  printToOLED(50, 37, sens2);                                     // Affichage du sens 2 sur l'OLED

  if (sens1[0] == '1') {                                          // Si le sens 1 est demandé
    digitalWrite(motorPin2,LOW);                                  // On désactive la pin demandant la rotation dans le sens 2 du moteur
    digitalWrite(motorPin1,HIGH);                                 // On active la pin demandant la rotation dans le sens 1 du moteur
  }
  else if (sens2[0] == '1') {                                     // Si le sens 2 est demandé
    digitalWrite(motorPin1,LOW);                                  // On désactive la pin demandant la rotation dans le sens 1 du moteur
    digitalWrite(motorPin2,HIGH);                                 // On active la pin demandant la rotation dans le sens 2 du moteur
  }
  else {
    digitalWrite(motorPin1,LOW);                                  // On désactive la pin demandant la rotation dans le sens 1 du moteur
    digitalWrite(motorPin2,LOW);                                  // On désactive la pin demandant la rotation dans le sens 2 du moteur
  }

  for (int j = 0; j <= LED_COUNT; j++) {                          // On boucle le nombre de LED(s) utilisées sur le ruban
    strip.setPixelColor(j, red, green, blue);                     // On défini la couleur avec les valeurs définies (via le callback MQTT correspondant)
  }
  strip.show();                                                   // On montre le changement (on actualise le ruban)
}
