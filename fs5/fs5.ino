////////////////////////////////////////////////////////////////////
// LIBRARIES ///////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
#include <Arduino.h>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "arduinoMFCC.h"

////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////
// CONSTANT AND GLOBAL VARIABLES ///////////////////////////////////
////////////////////////////////////////////////////////////////////
//DISPLAY
#define SCREEN_WIDTH 128     // OLED display width, in pixels
#define SCREEN_HEIGHT 64     // OLED display height, in pixels
#define OLED_RESET -1        // (-1 partage le pin de reset Arduino)
#define SCREEN_ADDRESS 0x3C  //0x3C pourr 128x32
Adafruit_SSD1306 _display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//SAMPLING
const int samplingFrequency = 44000;

/*
FIR filter designed with
http://t-filter.appspot.com

sampling frequency: 44000 Hz

* 20 Hz - 16000 Hz
  gain = 1
  desired ripple = 3 dB
  actual ripple = 2.1129857238161254 dB

* 17000 Hz - 22000 Hz
  gain = 0
  desired attenuation = -10 dB
  actual attenuation = -10.97955300799253 dB

*/

#define FILTER_TAP_NUM 21

static double filter_taps_float[FILTER_TAP_NUM] = {
  -0.06072852230688862,
  -0.009743879979818206,
  0.03554309359439512,
  -0.06453910915450291,
  0.07912618274311577,
  -0.06366899994206501,
  0.012191671871374815,
  0.06710767199756262,
  -0.15331654700903166,
  0.21994704013789626,
  0.7550762235546863,
  0.21994704013789626,
  -0.15331654700903166,
  0.06710767199756262,
  0.012191671871374815,
  -0.06366899994206501,
  0.07912618274311577,
  -0.06453910915450291,
  0.03554309359439512,
  -0.009743879979818206,
  -0.06072852230688862
};


uint16_t filteredValue;

//ADC BUFFERS
int position = 0;
#define BUFFER_SIZE FILTER_TAP_NUM
volatile uint16_t adc_buffer[BUFFER_SIZE];  //doit etre sur 16bits
float filtered_buffer[BUFFER_SIZE];

bool enregistrement = false;

//PINS
// A0 - microphone
// D4 - debut enregistrement
// D5 - fin enregistrement
// D6 - led enregistrement
const int PIN_LED_ENREGISTREMENT = 6;
const int PIN_BOUTON_DEBUT = 4;
const int PIN_BOUTON_FIN = 10;

//FRAMES

uint16_t sample;  // Variable pour stocker chaque échantillon audio
// Déclarez le tableau de frames
int numFrames = 0;           // Nombre de frames extraites
int numSamplesInBuffer = 0;  // Nombre d'échantillons actuellement présents dans le tampon
int nbValeurRecouvrement = 10;

const int frame_size_num = 250;
const int frame_length = 256;

uint8_t frames[frame_size_num][frame_length];
int rang1 = 0;
int rang2;


// Définition des paramètres MFCC
const int num_channels = 13;  //coeff avant compression DCT
const int frame_size = 128;
const int hop_size = 102;
const int mfcc_size = 8;  //mfcc_size <= num_channels //coeffs finaux
const float sample_rate = 44000.;


arduinoMFCC* mymfcc = new arduinoMFCC(num_channels, frame_size, hop_size, mfcc_size, sample_rate);

////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////
// SETUP ///////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
void setup() {

  Serial.begin(115200);

  setup_ADC();
  setup_display();

  pinMode(PIN_BOUTON_DEBUT, INPUT);
  pinMode(PIN_BOUTON_FIN, INPUT);
  pinMode(PIN_LED_ENREGISTREMENT, OUTPUT);

  TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;

  for (int i = 0; i < BUFFER_SIZE; i++) {
    filtered_buffer[i] = 0;
  }

  // Déclaration de l'objet MFCC
  //mymfcc->setup();
}
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////
// SETUPS //////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
//function used to setup the OLED display
void setup_display() {
  if (!_display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;  // Don't proceed, loop forever
  }

  _display.clearDisplay();
  _display.setTextColor(WHITE);
  _display.setCursor(2, 2);
  _display.print(F("FS5"));
  _display.display();
}

//function used to setup the analogic / digital converter
void setup_ADC() {
  PMC->PMC_PCER1 |= PMC_PCER1_PID37;     // Active le périphérique ADC
  ADC->ADC_MR = ADC_MR_PRESCAL(255)      // Définit le diviseur de fréquence à 255
                | ADC_MR_STARTUP_SUT64   // Définit le temps de démarrage à 64 périodes d'ADC_CLK
                | ADC_MR_TRACKTIM(15)    // Définit le temps de suivi à 15 périodes d'ADC_CLK
                | ADC_MR_SETTLING_AST3;  // Définit le temps de stabilisation à 17 périodes d'ADC_CLK
  ADC->ADC_CHER = ADC_CHER_CH7;          // Active le canal 7 (A0)

  // Configure Timer Counter 0 Channel 0 (TC0) pour samplingFrequency
  PMC->PMC_PCER0 |= PMC_PCER0_PID27;  // Active le périphérique TC0
  TC0->TC_CHANNEL[0].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK4 | TC_CMR_CPCTRG;
  // Définit la source d'horloge à TCLK4 (MCK / 128, 84 MHz / 128 = 656.25 kHz)
  // Active le déclenchement de comparaison RC
  // Définit la valeur RC pour une fréquence samplingFrequency Hz
  TC0->TC_CHANNEL[0].TC_RC = 656250 / samplingFrequency - 1;
  // Active l'interruption de comparaison RC
  TC0->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;
  // Active l'interruption TC0_IRQn dans le NVIC
  NVIC_EnableIRQ(TC0_IRQn);

  // Configure le contrôleur DMA
  PMC->PMC_PCER1 |= PMC_PCER1_PID39;                  // Active le périphérique PDC
  ADC->ADC_PTCR = ADC_PTCR_RXTDIS | ADC_PTCR_TXTDIS;  // Désactive le transfert PDC
  ADC->ADC_RPR = (uint32_t)adc_buffer;                // Définit le pointeur de réception sur le tampon
  ADC->ADC_RCR = BUFFER_SIZE;                         // Définit le compteur de réception à la taille du tampon
  ADC->ADC_RNPR = (uint32_t)adc_buffer;               // Définit le prochain pointeur de réception sur le tampon
  ADC->ADC_RNCR = BUFFER_SIZE;                        // Définit le prochain compteur de réception à la taille du tampon
  ADC->ADC_PTCR = ADC_PTCR_RXTEN;                     // Active le transfert PDC
}
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////
// HANDLERS ////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
//function used to handle clock TC0 interrupt
void TC0_Handler() {
  // Lit le registre d'état pour effacer le drapeau d'interruption
  TC0->TC_CHANNEL[0].TC_SR;
  // Démarre une nouvelle conversion ADC
  ADC->ADC_CR = ADC_CR_START;
}
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////
// LOOP ////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
void loop() {


  if (digitalRead(PIN_BOUTON_DEBUT) == 1 && enregistrement == false) {
    Serial.println("debut enregistrement");
    digitalWrite(PIN_LED_ENREGISTREMENT, HIGH);
    enregistrement = true;
    rang1 = 0;
    rang2 = 0;

    for (int i = 0; i < frame_size_num; i++) {
      for (int j = 0; j < frame_length; j++) {
        frames[i][j] = 0;
      }
    }
  }

  if (digitalRead(PIN_BOUTON_FIN) == 1 && enregistrement == true) {
    Serial.println("fin enregistrement");
    digitalWrite(PIN_LED_ENREGISTREMENT, LOW);
    enregistrement = false;
    Serial.println("....DEBUT....");
    for (int i = 0; i < rang1 + 1; i++) {
      for (int j = 0; j < frame_length; j++) {
        Serial.print("  | ");
        Serial.print(frames[i][j]);
      }
      Serial.println("  | ");
    }
    Serial.println("....FIN....");
    Serial.println("....FRAMES....");
    Serial.println(rang1);

    _display.clearDisplay();
    _display.print("NB Frames ");
    _display.println(rang1);
    _display.println("Taille frames 256");
    _display.println("Recouvrement 102");
    _display.print("NB Coeffs/frame ");
    _display.println(mfcc_size);
    _display.display();
    delay(2000);

    for (int i = 0; i < rang1; i++) {
      for (int j = 0; j < frame_length; j++) {
        mymfcc->_frame[j] = frames[i][j];
      }
      mymfcc->computebust_dct();
      for (int j = 0; j < num_channels; j++) {
        Serial.print(mymfcc->_frame[j]);
        Serial.print(" | ");
      }
      Serial.println();

    }

  }

  // Vérifie si le transfert DMA est terminé
  if (ADC->ADC_ISR & ADC_ISR_ENDRX) {
    // Désactive le transfert PDC
    ADC->ADC_PTCR = ADC_PTCR_RXTDIS | ADC_PTCR_TXTDIS;
    filtersignalRIF2();

    if (enregistrement == true) {  // enregistrement en cours
      sample = filteredValue;      // Lire un nouvel échantillon audio+

      //Serial.println(sample);
      //décaler de moins que 8 bits

      frames[rang1][rang2] = sample / (1 << 4);
      rang2++;

      if (rang2 == 256) {



        for (int i = 0; i < 102; i++) {  //recouvrement pour 40% des 256 valeurs = 102

          frames[rang1 + 1][i] = frames[rang1][154 + i];  //permet de récupérer nos 102 dernières valeurs de la frame précedente
        }

        rang1++;
        rang2 = 102;
      }
    }
  }


  //Serial.println(filteredValue);
  // Réactive le transfert PDC
  ADC->ADC_PTCR = ADC_PTCR_RXTEN;
  // Réinitialise le pointeur de réception et le compteur
  ADC->ADC_RPR = (uint32_t)&adc_buffer[position];
  ADC->ADC_RCR = 1;

  // Réinitialise le prochain pointeur de réception et le compteur
  ADC->ADC_RNPR = (uint32_t)&adc_buffer[position];
  ADC->ADC_RNCR = 1;
}

////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////
// FILTRAGE ////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
void filtersignalRIF2() {
  filteredValue = 0;  // calcul de la nouvelle valeur filtrée
  //filtrage avec tampon circulaire
  for (int l = 0; l < FILTER_TAP_NUM; l++) {
    if ((position - l) < 0) {
      filteredValue += filter_taps_float[l] * adc_buffer[position - l + FILTER_TAP_NUM];
    } else {
      filteredValue += filter_taps_float[l] * adc_buffer[position - l];
    }
  }
  position = (position + 1) % FILTER_TAP_NUM;
}
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////