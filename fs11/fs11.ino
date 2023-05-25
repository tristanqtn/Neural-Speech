#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128     // OLED display width, in pixels
#define SCREEN_HEIGHT 64     // OLED display height, in pixels
#define OLED_RESET 4         // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C  ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

int valeur = 0;  // Variable où on stock la valeur du potentiomètre
int choix = 0;

int entered = -1;  //choix selectionné

bool dans_choix = false;

const int PIN_POTENTIOMETRE = A1;
const int PIN_BOUTON_ENTRER = 2;
const int PIN_BOUTON_RETOUR = 3;

const char *options[5] = {
  "Acquisition classique",
  "Acquisition binaire",
  "Acquisition filtree",
  "Formation des frames",
  "MFCC"
};

//....MENU OLED....

void displaymenu(void) {

  valeur = analogRead(PIN_POTENTIOMETRE);  // on lit les données du pin A0

  choix = map(valeur, 0, 1023, 0, 5);

  if (digitalRead(PIN_BOUTON_ENTRER) == HIGH && dans_choix == false) {  //bouton enter pressé
    entered = choix;                                                    //récupère valeur choix
    dans_choix = true;
    Serial.println("bouton choix clique");
    Serial.println(choix);
  };
  if (digitalRead(PIN_BOUTON_RETOUR) == HIGH && dans_choix == true) {  //bouton retour pressé
    entered = -1;
    dans_choix = false;
    Serial.println("bouton retour clique");
    Serial.println(choix);
  };



  if (entered == -1) {  //si pas choix sélectionné afficher menu

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(F("MENU"));
    display.println("");

    for (int i = 0; i < 5; i++) {
      if (i == choix) {  //mettre en surbriance le choix
        display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
        display.println(options[i]);
      } else if (i != choix) {  //sinon fond noir
        display.setTextColor(SSD1306_WHITE);
        display.println(options[i]);
      }
    }


  } else if (entered == 0) {  //si choix=0
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("CHOIX 1");
  } else if (entered == 1) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("CHOIX 2");
  } else if (entered == 2) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("CHOIX 3");
  } else if (entered == 3) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("CHOIX 4");
  } else if (entered == 4 || entered == 5) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("CHOIX 5");
  }
  display.display();
}

void setup() {
  Serial.begin(9600);  // Initialisons la communication serial
  pinMode(A1, INPUT);  //déclare A0 comme sortie

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;  // Don't proceed, loop forever
  }
  display.clearDisplay();
  // Draw a single pixel in white
  display.drawPixel(10, 10, SSD1306_WHITE);
  display.display();
  delay(2000);  // Pause for 2 seconds
}

void loop() {
  displaymenu();
}