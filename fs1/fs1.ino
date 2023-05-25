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
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////
// SETUP ///////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
void setup() {

  Serial.begin(4800);

  setup_ADC();
  setup_display();

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
  _display.print(F("FS1"));
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
  // Vérifie si le transfert DMA est terminé
  if (ADC->ADC_ISR & ADC_ISR_ENDRX) {
    // Désactive le transfert PDC
    ADC->ADC_PTCR = ADC_PTCR_RXTDIS | ADC_PTCR_TXTDIS;
    // Afficher les valeurs ADC sur le moniteur série et envoie les données au DAC
    for (int i = 0; i < BUFFER_SIZE; i++) {
      //display_buffer_information(i);
      //Serial.println((uint8_t)adc_buffer[i]);
    }
    // Réactive le transfert PDC
    ADC->ADC_PTCR = ADC_PTCR_RXTEN;
    // Réinitialise le pointeur de réception et le compteur
    ADC->ADC_RPR = (uint32_t)adc_buffer;
    ADC->ADC_RCR = BUFFER_SIZE;
    // Réinitialise le prochain pointeur de réception et le compteur
    ADC->ADC_RNPR = (uint32_t)adc_buffer;
    ADC->ADC_RNCR = BUFFER_SIZE;
  }
}
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////
// DISPLAYS ////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
//function used to display the adc_buffer
void display_buffer_information(int i) {
  Serial.println(adc_buffer[i]);
  if (i > 0) {
      _display.writeLine((int16_t)i - 1, (int16_t)SCREEN_HEIGHT - 1 - map(adc_buffer[i - 1], MIN_ADC, MAX_ADC, 0, 50), (int16_t)i, (int16_t)SCREEN_HEIGHT - 1 - map(adc_buffer[i], MIN_ADC, MAX_ADC, 0, 50), WHITE);
      _display.display();
  }
  if (i == BUFFER_SIZE - 1) {
    _display.clearDisplay();
    _display.setTextColor(WHITE);
    _display.setCursor(2, 2);
    _display.print(F("Signal Captured"));
    _display.display();
  }
}
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////