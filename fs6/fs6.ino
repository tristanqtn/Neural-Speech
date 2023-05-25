#include <Arduino.h>
// Constants
const int inputSize = 13;        // Number of input neurons
const int hiddenSize = 10;       // Number of hidden neurons
const int outputSize = 1;        // Number of output neurons
const int sequenceLength = 255;  // Length of the sequence to be predicted
const int ledPin = 13;           // Pin for the LED display
// Variables
float inputs[inputSize];                  // Input sequence
float outputs[outputSize];                // Output sequence
float hidden[hiddenSize];                 // Hidden state
float weightsIH[hiddenSize][inputSize];   // Input-hidden weights
float weightsHH[hiddenSize][hiddenSize];  // Hidden-hidden weights
float weightsHO[outputSize][hiddenSize];  // Hidden-output weights
float biasH[hiddenSize];                  // Hidden biases
float biasO[outputSize];                  // Output biases
float threshold = 0.01;                   // Prediction threshold
bool predict = false;                     // Flag for starting prediction sequence
//
void initializeWeights() {
  // Initialize input-hidden weights
  for (int i = 0; i < hiddenSize; i++) {
    for (int j = 0; j < inputSize; j++) {
      weightsIH[i][j] = random(-100, 100) / 100.0;
    }
  }
  // Initialize hidden-hidden weights
  for (int i = 0; i < hiddenSize; i++) {
    for (int j = 0; j < hiddenSize; j++) {
      weightsHH[i][j] = random(-100, 100) / 100.0;
    }
  }
  // Initialize hidden-output weights
  for (int i = 0; i < outputSize; i++) {
    for (int j = 0; j < hiddenSize; j++) {
      weightsHO[i][j] = random(-100, 100) / 100.0;
    }
  }
  // Initialize biases
  for (int i = 0; i < hiddenSize; i++) {
    biasH[i] = random(-100, 100) / 100.0;
  }
  for (int i = 0; i < outputSize; i++) {
    biasO[i] = random(-100, 100) / 100.0;
  }
}
///////////////////  forwardPass ///////////////////////////
void forwardPass(float* input, float* output, float* hidden) {
  // Compute hidden state
  for (int i = 0; i < hiddenSize; i++) {
    hidden[i] = 0;
    for (int j = 0; j < inputSize; j++) {
      hidden[i] += weightsIH[i][j] * input[j];
    }
    for (int j = 0; j < hiddenSize; j++) {
      hidden[i] += weightsHH[i][j] * hidden[j];
    }
    hidden[i] += biasH[i];
    hidden[i] = activation(hidden[i]);
  }
  // Compute output
  for (int i = 0; i < outputSize; i++) {
    output[i] = 0;
    for (int j = 0; j < hiddenSize; j++) {
      output[i] += weightsHO[i][j] * hidden[j];
    }
    output[i] += biasO[i];
    output[i] = activation(output[i]);
  }
}
///////////////////// Sigmoid activation function/////////////////////
float activation(float x) {
  // Sigmoid activation function
  return 1.0 / (1.0 + exp(-x));
}
///////////////////////////////////////////////////////////////////////
void readPoten() {
  // Read poten value and map to prediction threshold
  int potValue = 0.5;  // exemple
}
///////////////////////////////////////////////////////////////////////
void startPrediction() {
  // Initialize input sequence with random values
  for (int i = 0; i < sequenceLength; i++) {
    inputs[i] = random(0, 10) / 10.0;
  }
  ///////////////////////////////////////////////////////////////////////
  // Set flag to start prediction sequence
  predict = true;
}
//////////////////////////////////////////////////////////////////////////
void setup() {
  // Setup pins
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
  // Initialize weights and biases
  initializeWeights();
}

void loop() {
  // Read potentiometer value and adjust threshold
  readPoten();
  // Check if start button is pressed

  startPrediction();

  if (predict) {
    forwardPass(inputs, outputs, hidden);

    // Output predicted value on LED display
    if (outputs[0] > threshold) {
      digitalWrite(ledPin, HIGH);
    } else {
      digitalWrite(ledPin, LOW);
    }

    // Shift input sequence
    for (int i = sequenceLength - 1; i > 0; i--) {
      inputs[i] = inputs[i - 1];
    }
    inputs[0] = outputs[0];
  }
}