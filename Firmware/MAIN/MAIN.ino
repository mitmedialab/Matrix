/*
  ** E256 Firmware v1.1 **
  This file is part of the eTextile-matrix-sensor project - http://matrix.eTextile.org
  Copyright (c) 2014- Maurin Donneaud <maurin@etextile.org>
  This work is licensed under Creative Commons Attribution-ShareAlike 4.0 International license, see the LICENSE file for details.
*/

// FPS with CPU speed to 120 MHz (Overclock)
// Optimize Fastest with LTO (Link Time Optimizations)
//   523 FPS : ADC_INPUT
//    .. FPS : ADC_INPUT / BLOB_TRACKING
//    .. FPS : ADC_INPUT / BILINEAR_INTERPOLATION
//    39 FPS : ADC_INPUT / BILINEAR_INTERPOLATION / BLOB_TRACKING

#include <SPI.h>        // https://www.pjrc.com/teensy/td_libs_SPI.html
#include <ADC.h>        // https://github.com/pedvide/ADC
#include <MIDI.h>       // http://www.pjrc.com/teensy/td_midi.html

#include "config.h"
#include "interp.h"
#include "blob.h"

SPISettings settings(16000000, MSBFIRST, SPI_MODE0);

ADC *adc = new ADC();     // ADC object
ADC::Sync_result result;  // ADC_0 & ADC_1

// Array to store all parameters used to configure the two 8:1 analog multiplexeurs
// Eatch byte |ENA|A|B|C|ENA|A|B|C|
const byte setDualRows[RAW_ROWS] = {
  0x55, 0x77, 0x66, 0x44, 0x22, 0x11, 0x00, 0x33
};

uint8_t minValsArray[RAW_FRAME] = {0};            // 1D Array to store smallest values
uint8_t frameArray[RAW_FRAME] = {0};              // 1D Array to store ofseted analog input values

uint8_t bitmapArray[NEW_FRAME] = {0};             // 1D Array to store binary values (16*16) array containing (64*64) values
uint8_t interpFrameArray[NEW_FRAME] = {0};        // 1D Array to store bilinear interpolated values

xylr_t lifoArray[LIFO_MAX_NODES] = {0};           // 1D Array to store lifo nodes
blob_t blobArray[MAX_NODES] = {0};                // 1D Array to store blobs

uint8_t blobPacket[BLOB_PACKET_SIZE] = {0};       // 1D Array to store blobs in OSC format

image_t  inputFrame;         // Input frame values
interp_t interp;             //
image_t  interpolatedFrame;  //
image_t  bitmap;             // Used by Scanline Flood Fill algorithm / SFF
lifo_t   lifo_stack;         // Lifo free nodes stack
lifo_t   lifo;               // Lifo stack
llist_t  blobs_stack;        // Blobs free nodes linked list
llist_t  blobs;              // Intermediate blobs linked list
llist_t  outputBlobs;        // Output blobs linked list

uint8_t threshold = 20;
uint8_t calibration_cycles = 20;
boolean calibrate = true;

void setup() {

  //pinMode(LED_BUILTIN, OUTPUT);   // FIXME - BUILTIN_LED is used for SPI hardware
  //digitalWrite(LED_BUILTIN, LOW); // FIXME - BUILTIN_LED is used for SPI hardware

  //pinMode(BUTTON_PIN, INPUT_PULLUP);          // Set button pin as input and activate the input pullup resistor // FIXME - NO BUTTON_PIN ON the E256!
  //attachInterrupt(BUTTON_PIN, calib, RISING); // Attach interrrupt on button PIN // FIXME - NO BUTTON_PIN ON the E256

  SPI_SETUP();

  ADC_SETUP();

  INTERP_SETUP(
    &inputFrame,          // image_t*
    &frameArray[0],       // uint8_t*
    &interpolatedFrame,   // image_t*
    &interpFrameArray[0], // uint8_t*
    &interp               // interp_t*
  );

  BLOB_SETUP(
    &inputFrame,          // image_t*
    &bitmap,              // image_t*
    &bitmapArray[0],      // uint8_t*
    &lifo,                // lifo_t*
    &lifo_stack,          // lifo_t*
    &lifoArray[0],        // xylr_t*
    &blobs,               // list_t*
    &blobs_stack,         // list_t*
    &blobArray[0],        // blob_t*
    &outputBlobs          // list_t*
  );
  usbMIDI.begin();
}

//////////////////// LOOP
void loop() {

  if (calibrate) matrix_calibrate(&minValsArray[0]);
  matrix_scan(&frameArray[0]);
  usbMIDI_sendNotes(frameArray);
  matrix_interp(&interpolatedFrame, &inputFrame, &interp);
/*
  find_blobs(
    threshold,          // uint8_t
    &interpolatedFrame, // image_t (uint8_t array[NEW_FRAME] - 64*64 1D array)
    &bitmap,            // image_t (uint8_t array[NEW_FRAME] - 64*64 1D array)
    &lifo_stack,        // lifo_t
    &lifo,              // lifo_t
    &blobs_stack,       // list_t
    &blobs,             // list_t
    &outputBlobs        // list_t
  );

  // Send all blobs in MIDI format
  // usbMIDI.sendControlChange(control, value, channel);
  int pos = 0;
  for (blob_t* blob = ITERATOR_START_FROM_HEAD(&outputBlobs); blob != NULL; blob = ITERATOR_NEXT(blob)) {

    pos = blob->UID * 10;

    if (blob->alive == 1) {
      //usbMIDI.sendControlChange(1 + pos, blob->UID,      1); //
      usbMIDI.sendAfterTouchPoly(0 + pos, blob->alive,      1); //
      usbMIDI.sendAfterTouchPoly(1 + pos, blob->centroid.X, 1); //
      usbMIDI.sendAfterTouchPoly(2 + pos, blob->centroid.Y, 1); //
      usbMIDI.sendAfterTouchPoly(3 + pos, blob->box.W,      1); //
      usbMIDI.sendAfterTouchPoly(4 + pos, blob->box.H,      1); //
      usbMIDI.sendAfterTouchPoly(5 + pos, blob->box.D >> 1, 1); //
    }
    else {
      //usbMIDI.sendControlChange(1 + pos, 0, 1); //
      usbMIDI.sendAfterTouchPoly(0 + pos, 0, 1);   //
      usbMIDI.sendAfterTouchPoly(1 + pos, 0, 1);   //
      usbMIDI.sendAfterTouchPoly(2 + pos, 0, 1);   //
      usbMIDI.sendAfterTouchPoly(3 + pos, 0, 1);   //
      usbMIDI.sendAfterTouchPoly(4 + pos, 0, 1);   //
      usbMIDI.sendAfterTouchPoly(5 + pos, 0, 1);   //
    }
  }
*/
  while (usbMIDI.read()); // Read and discard any incoming MIDI messages

#ifdef DEBUG_ADC
  matrix_scan(&frameArray[0]);

  for (uint8_t col = 0; col < RAW_COLS; col++) {
    for (uint8_t row = 0; row < RAW_ROWS; row++) {
      uint8_t index = col * RAW_COLS + row;          // Compute 1D array index
      Serial.printf("\t%d", frameArray[index]);
    }
    Serial.println();
  }
  Serial.println();
  delay(50);
#endif /*__DEBUG_ADC__*/

#ifdef DEBUG_INTERP
  if (calibrate) matrix_calibrate(&minValsArray[0]);
  matrix_scan(&frameArray[0]);
  matrix_interp(&interpolatedFrame, &inputFrame, &interp);
  for (uint8_t col = 0; col < NEW_COLS; col++) {
    for (uint8_t row = 0; row < NEW_ROWS; row++) {
      uint16_t index = col * NEW_COLS + row;          // Compute 1D array index
      Serial.printf("-%d", interpFrameArray[index]);
    }
    Serial.println();
  }
  Serial.println();
  delay(50);
#endif /*__DEBUG_INTERP__*/

#ifdef DEBUG_BITMAP
  if (calibrate) matrix_calibrate(&minValsArray[0]);
  matrix_scan(&frameArray[0]);
  matrix_interp(&interpolatedFrame, &inputFrame, &interp);
  find_blobs(
    threshold,          // uint8_t
    &interpolatedFrame, // image_t (uint8_t array[NEW_FRAME] - 64*64 1D array)
    &bitmap,            // image_t (uint8_t array[NEW_FRAME] - 64*64 1D array)
    &lifo_stack,        // lifo_t
    &lifo,              // lifo_t
    &blobs_stack,       // list_t
    &blobs,             // list_t
    &outputBlobs        // list_t
  );

  for (uint8_t posY = 0; posY < NEW_COLS; posY++) {
    uint8_t* bmp_row = COMPUTE_BINARY_IMAGE_ROW_PTR (&bitmap, posY);
    for (uint8_t posX = 0; posX < NEW_ROWS; posX++) {
      Serial.print(IMAGE_GET_BINARY_PIXEL_FAST(bmp_row, posX));
    }
    Serial.println();
  }
  Serial.println();
  delay(50);
#endif /*__DEBUG_BITMAP__*/

#ifdef DEBUG_BLOBS_OSC
  if (calibrate) matrix_calibrate(&minValsArray[0]);
  matrix_scan(&frameArray[0]);
  matrix_interp(&interpolatedFrame, &inputFrame, &interp);
  find_blobs(
    threshold,          // uint8_t
    &interpolatedFrame, // image_t (uint8_t array[NEW_FRAME] - 64*64 1D array)
    &bitmap,            // image_t (uint8_t array[NEW_FRAME] - 64*64 1D array)
    &lifo_stack,        // lifo_t
    &lifo,              // lifo_t
    &blobs_stack,       // list_t
    &blobs,             // list_t
    &outputBlobs        // list_t
  );

  for (blob_t* blob = ITERATOR_START_FROM_HEAD(&outputBlobs); blob != NULL; blob = ITERATOR_NEXT(blob)) {
    Serial.print (blob->UID);        // uint8_t unique session ID
    Serial.print(" ");
    Serial.print (blob->alive);      // uint8_t
    Serial.print(" ");
    Serial.print (blob->centroid.X); // int8_t
    Serial.print(" ");
    Serial.print (blob->centroid.Y); // int8_t
    Serial.print(" ");
    Serial.print (blob->box.W);      // uint8_t
    Serial.print(" ");
    Serial.print (blob->box.H);      // uint8_t
    Serial.print(" ");
    Serial.print (blob->box.D);      // uint8_t
    Serial.println();
  }
#endif /*__DEBUG_BLOBS_OSC__*/
}

//////////////////////////////////////////////////// FONCTIONS

void SPI_SETUP(void) {

  //SPI.setSCK(E256_SS_PIN);        // D10 - Hardware SPI no need to specify it!
  SPI.setSCK(E256_SCK_PIN);         // D14 - using alternate pin (instead of D13)
  //SPI.setMOSI(E256_MOSI_PIN);     // D11 - Hardware SPI no need to specify it!
  pinMode(E256_SS_PIN, OUTPUT);
  pinMode(E256_SCK_PIN, OUTPUT);
  pinMode(E256_MOSI_PIN, OUTPUT);
  digitalWriteFast(E256_SS_PIN, LOW);    // Set latchPin LOW
  digitalWriteFast(E256_SS_PIN, HIGH);   // Set latchPin HIGH
  SPI.begin();                           // Start the SPI module
  SPI.beginTransaction(settings);        // (16000000, MSBFIRST, SPI_MODE0);
}

void ADC_SETUP(void) {

  pinMode(ADC0_PIN, INPUT);              // Teensy PIN A9
  pinMode(ADC1_PIN, INPUT);              // Teensy PIN A3

  adc->adc0->setAveraging(1);                                           // Set number of averages
  adc->adc0->setResolution(8);                                          // Set bits of resolution
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED); // Change the conversion speed
  //adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED);    // Change the conversion speed
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED);     // Change the sampling speed
  //adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED);        // Change the sampling speed
  //adc->adc0->enableCompare(1.0 / 3.3 * adc->adc0->getMaxValue(), 0);  // Measurement will be ready if value < 1.0V

  adc->adc1->setAveraging(1);                                           // Set number of averages
  adc->adc1->setResolution(8);                                          // Set bits of resolution
  adc->adc1->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED); // Change the conversion speed
  // adc->adc1->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED);   // Change the conversion speed
  adc->adc1->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED);     // Change the sampling speed
  //adc->adc1->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED);        // Change the sampling speed
  //adc->adc1->enableCompare(1.0 / 3.3 * adc->adc1->getMaxValue(), 0);  // Measurement will be ready if value < 1.0V
}

// Columns are digital OUTPUT PINS - We supply them one by one sequentially
// Rows are analog INPUT PINS - We sens them two by two
void matrix_scan(uint8_t* outputFrame) {

  uint32_t setCols = 0x10000;

  for (uint8_t col = 0; col < RAW_COLS; col++) {    // 0 to 15
    setCols = setCols >> 1;
    for (uint8_t row = 0; row < DUAL_ROWS; row++) { // 0 to 7

      digitalWriteFast(E256_SS_PIN, LOW);   // Set latchPin LOW
      SPI.transfer(setCols & 0xFF);         // Shift out the LSB byte to set up the OUTPUT shift register
      SPI.transfer((setCols >> 8) & 0xFF);  // Shift out the MSB byte to set up the OUTPUT shift register
      SPI.transfer(setDualRows[row]);       // Shift out one byte that setup the two 8:1 analog multiplexers
      digitalWriteFast(E256_SS_PIN, HIGH);  // Set latchPin HIGH
      //delayMicroseconds(5);               // TODO: see switching time of the 74HC4051BQ multiplexeur

      uint8_t rowIndexA = row * RAW_COLS + col; // Row IndexA computation
      uint8_t rowIndexB = rowIndexA + 128;      // Row IndexB computation (ROW_FRAME/2 == 128)

      result = adc->analogSynchronizedRead(ADC0_PIN, ADC1_PIN);

      int valA = result.result_adc0 - minValsArray[rowIndexA];
      valA > 0 ? outputFrame[rowIndexA] = (uint8_t)valA : outputFrame[rowIndexA] = 0;
      int valB = result.result_adc1 - minValsArray[rowIndexB];
      valB > 0 ? outputFrame[rowIndexB] = (uint8_t)valB : outputFrame[rowIndexB] = 0;
    }
  }
}

void matrix_calibration_set() {
  calibrate = true;
}

// Columns are digital OUTPUT PINS - We supply them one by one sequentially
// Rows are analog INPUT PINS - We sens them two by two
void matrix_calibrate(uint8_t* outputFrame) {

  for (uint8_t i = 0; i < calibration_cycles; i++) {

    uint32_t setCols = 0x10000;

    for (uint8_t col = 0; col < RAW_COLS; col++) {
      setCols = setCols >> 1;
      for (uint8_t row = 0; row < DUAL_ROWS; row++) {

        digitalWriteFast(E256_SS_PIN, LOW);   // Set latchPin LOW
        SPI.transfer(setCols & 0xFF);         // Shift out the LSB byte to set up the OUTPUT shift register
        SPI.transfer((setCols >> 8) & 0xFF);  // Shift out the MSB byte to set up the OUTPUT shift register
        SPI.transfer(setDualRows[row]);       // Shift out one byte that setup the two 8:1 analog multiplexers
        digitalWriteFast(E256_SS_PIN, HIGH);  // Set latchPin HIGH
        //delayMicroseconds(5);                 // TODO: See switching time of the 74HC4051BQ multiplexeur

        result = adc->analogSynchronizedRead(ADC0_PIN, ADC1_PIN);
        uint8_t ADC0_val = result.result_adc0;
        uint8_t ADC1_val = result.result_adc1;

        uint8_t rowIndexA = row * RAW_COLS + col; // Row IndexA computation
        uint8_t rowIndexB = rowIndexA + 128;      // Row IndexB computation (ROW_FRAME/2 == 128)

        if (ADC0_val > outputFrame[rowIndexA]) outputFrame[rowIndexA] = ADC0_val;
        if (ADC1_val > outputFrame[rowIndexB]) outputFrame[rowIndexB] = ADC1_val;
      }
    }
  }
  calibrate = false;
}

// Set the threshold
void matrix_threshold_set() {
  //threshold = msg.getInt(0) & 0xFF; // Get the first uint8_t of the int32_t
}

void usbMIDI_sendNotes(uint8_t* velocity) {
  static uint8_t prev_velocity[RAW_FRAME] = {0};
  const uint8_t threshold = 3;

  for (int note = 0; note < RAW_FRAME; note++) {
    const uint8_t channel = (note < 128) ? 0 : 1;

    if (velocity[note] < threshold) {
        velocity[note] = 0;

        // don't send off everytime:
        if (prev_velocity[note] != 0)
          usbMIDI.sendNoteOff(note, prev_velocity[note], channel);
    } else {
      usbMIDI.sendNoteOn(note, velocity[note], channel);
    }

    prev_velocity[note] = velocity[note];
  }
}

