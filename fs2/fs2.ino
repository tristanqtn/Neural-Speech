////////////////////////////////////////////////////////////////////
// LIBRARIES ///////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
#include <Arduino.h>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
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

//ADC
#define MAX_ADC 4095
#define MIN_ADC 0

//SAMPLING
const int samplingFrequency = 44000;

//ADC BUFFERS
#define BUFFER_SIZE 128
volatile uint16_t adc_buffer[BUFFER_SIZE];

bool enregistrement = false;

//PINS
// A0 - microphone
// D4 - debut enregistrement
// D5 - fin enregistrement
// D6 - led enregistrement
const int PIN_LED_ENREGISTREMENT = 6;
const int PIN_BOUTON_DEBUT = 4;
const int PIN_BOUTON_FIN = 10;
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
  _display.println(F("FS2"));
  _display.println();
  _display.print("fe ");
  _display.print(samplingFrequency);
  _display.println(" Hz");
  _display.println();
  _display.print("Taille tampon");
  _display.println(BUFFER_SIZE);
  _display.display();
}

//function used to setup the analogic / digital converter
void setup_ADC() {
  PMC->PMC_PCER1 |= PMC_PCER1_PID37;     // Active le peripherique ADC
  ADC->ADC_MR = ADC_MR_PRESCAL(255)      // Definit le diviseur de frequence a 255
                | ADC_MR_STARTUP_SUT64   // Definit le temps de demarrage a 64 periodes d'ADC_CLK
                | ADC_MR_TRACKTIM(15)    // Definit le temps de suivi a 15 periodes d'ADC_CLK
                | ADC_MR_SETTLING_AST3;  // Definit le temps de stabilisation a 17 periodes d'ADC_CLK
  ADC->ADC_CHER = ADC_CHER_CH7;          // Active le canal 7 (A0)

  // Configure Timer Counter 0 Channel 0 (TC0) pour samplingFrequency
  PMC->PMC_PCER0 |= PMC_PCER0_PID27;  // Active le peripherique TC0
  TC0->TC_CHANNEL[0].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK4 | TC_CMR_CPCTRG;
  // Definit la source d'horloge a TCLK4 (MCK / 128, 84 MHz / 128 = 656.25 kHz)
  // Active le declenchement de comparaison RC
  // Definit la valeur RC pour une frequence samplingFrequency Hz
  TC0->TC_CHANNEL[0].TC_RC = 656250 / samplingFrequency - 1;
  // Active l'interruption de comparaison RC
  TC0->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;
  // Active l'interruption TC0_IRQn dans le NVIC
  NVIC_EnableIRQ(TC0_IRQn);

  // Configure le contrôleur DMA
  PMC->PMC_PCER1 |= PMC_PCER1_PID39;                  // Active le peripherique PDC
  ADC->ADC_PTCR = ADC_PTCR_RXTDIS | ADC_PTCR_TXTDIS;  // Desactive le transfert PDC
  ADC->ADC_RPR = (uint32_t)adc_buffer;                // Definit le pointeur de reception sur le tampon
  ADC->ADC_RCR = BUFFER_SIZE;                         // Definit le compteur de reception a la taille du tampon
  ADC->ADC_RNPR = (uint32_t)adc_buffer;               // Definit le prochain pointeur de reception sur le tampon
  ADC->ADC_RNCR = BUFFER_SIZE;                        // Definit le prochain compteur de reception a la taille du tampon
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
  // Lit le registre d'etat pour effacer le drapeau d'interruption
  TC0->TC_CHANNEL[0].TC_SR;
  // Demarre une nouvelle conversion ADC
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
    //Serial.println("debut enregistrement");
    digitalWrite(PIN_LED_ENREGISTREMENT, HIGH);
    enregistrement = true;
  }

  if (digitalRead(PIN_BOUTON_FIN) == 1 && enregistrement == true) {
    //Serial.println("fin enregistrement");
    digitalWrite(PIN_LED_ENREGISTREMENT, LOW);
    enregistrement = false;
  }

  // Verifie si le transfert DMA est termine
  if (ADC->ADC_ISR & ADC_ISR_ENDRX) {
    // Desactive le transfert PDC
    ADC->ADC_PTCR = ADC_PTCR_RXTDIS | ADC_PTCR_TXTDIS;
    // Afficher les valeurs ADC sur le moniteur serie et envoie les donnees au DAC
    if (enregistrement == true) {
      for (int i = 0; i < BUFFER_SIZE; i++) {
        Serial.write(adc_buffer[i]/(1<<8));
      }
    }
    // Reactive le transfert PDC
    ADC->ADC_PTCR = ADC_PTCR_RXTEN;
    // Reinitialise le pointeur de reception et le compteur
    ADC->ADC_RPR = (uint32_t)adc_buffer;
    ADC->ADC_RCR = BUFFER_SIZE;
    // Reinitialise le prochain pointeur de reception et le compteur
    ADC->ADC_RNPR = (uint32_t)adc_buffer;
    ADC->ADC_RNCR = BUFFER_SIZE;
  }
}
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////