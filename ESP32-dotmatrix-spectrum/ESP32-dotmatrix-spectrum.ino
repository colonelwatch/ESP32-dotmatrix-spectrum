// Copyright 2019 colonelwatch

#include <SPI.h>
// FFT using 8-bit integers
#include "fix_fft.h"
// Used for watchdog reset
#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"

// Pin mappings to HUB08 for DOIT ESP32 DEVKIT V1 (short version)
#define ENABLEPIN 5   // EN
#define LATCHPIN 19   // LT
// Data (R1 or G1) pin is hardware MOSI, or 23
// Clock (CLK) pin is hardware CLK, or 18
#define A_PIN 2       // A
#define B_PIN 4       // B
#define C_PIN 16      // C
#define D_PIN 17      // D
// Don't forget to wire ground!
// Microphone input is 39
// 3.5mm input is 36

// User-configurable settings
#define COLUMNS_COUNT 64              // Used to determine FFT size and modifies
                                      //  the flashDisplay function. Must be a
                                      //  power of 2.
#define ROWS_COUNT 16                 // Used to modify flashDisplay function.
                                      //  Likely shouldn't exceed 16 without a
                                      //  rewrite of flashDisplay (which I cannot
                                      //  test). Must be a power of 2.
#define CAP 90                        // Use to map post-processed FFT output to
                                      //  display (raise for longer bars, lower
                                      //  to shorter bars)
#define TIME_FACTOR 25                // Configures rise smoothing function (raise
                                      //  for smoother output, lower for dynamic
                                      //  output)
#define TIME_FACTOR2 25               // Configures fall smoothing function (raise
                                      //  for smoother output, lower for dynamic
                                      //  output)
#define SAMPLING_FREQUENCY 10000      // Frequency at which sampling interrupt will
                                      //  be called, actual output range will be
                                      //  limited to half because of Nyquist.
#define SENSITIVITY 1.65              // FFT output multiplier before post-processing.
#define DEBOUNCE 500                  // Debounce time in milliseconds for BOOT button

// Global constants
const float coeff = 1./TIME_FACTOR;                 // Coefficients for rise smoothing
const float anti_coeff = (TIME_FACTOR-1.)/TIME_FACTOR;
const float coeff2 = 1./TIME_FACTOR2;               // Coefficients for fall smoothing
const float anti_coeff2 = (TIME_FACTOR2-1.)/TIME_FACTOR2;
const int sample_period = 1000000/SAMPLING_FREQUENCY;
const int elem_count = COLUMNS_COUNT/8;
const int fft_size = COLUMNS_COUNT*2;
const int array_size = COLUMNS_COUNT*ROWS_COUNT/8;

// Global variables
volatile int analogBuffer[fft_size] = {0};         // Circular buffer for storing analogReads
volatile int analogBuffer_index = 0;          // Write index for analogBuffer, also used for reads
volatile bool analogBuffer_availible = true;  // Memory busy flag for analogBuffer
uint8_t displayBuffer[array_size] = {0};       // First buffer for storing LED matrix output
uint8_t doubleBuffer[array_size]  = {0};       // Second buffer for storing LED matrix output
uint8_t *readBuffer = doubleBuffer;     // Pointer that specifies which buffer is to be read from
uint8_t *writeBuffer = displayBuffer;   // Pointer that specifies which buffer is to be written to
bool displayBuffer_availible = true;    // Memory busy flag for display buffer
volatile int inputPin = 39;      // ADC pin, either 36(aux) or 39(microphone)
int fft_power;

// Function prototypes, explanations at bottom
void flashDisplay(uint8_t frame[array_size], int interval = 5);
void analogBuffer_store(int val);

// Core 0 thread
TaskHandle_t Task1;
void Task1code(void* pvParameters){
  int milliseconds = millis();
  while(true){
    // Debouncing code that checks if the BOOT button has been pressed and, if
    // so, changes the audio source between the microphone and the 3.5mm jack
    if(digitalRead(0) == LOW && millis() > milliseconds + DEBOUNCE){
      milliseconds = millis();
      if(inputPin == 36) inputPin = 39;
      else inputPin = 36;
    }
    
    // Double-buffered output with page flipping to display, only updating for new data
    // Currently somewhat slowerthan FFT process (1042 fps vs 1627.8 FFT's per second)
    // because of SPI frequency limitations on the display
    if(displayBuffer_availible){
      if(readBuffer == doubleBuffer){
        readBuffer = displayBuffer;
        writeBuffer = doubleBuffer;
      }
      else{
        readBuffer = doubleBuffer;
        writeBuffer = displayBuffer;
      }
      displayBuffer_availible = false;
    }
    flashDisplay(readBuffer, 5);
  }
}

// Core 1 Interrupt thread
hw_timer_t * timer = NULL;
void IRAM_ATTR onTimer(){
  // Basic function is to record new values for circular analogBuffer, but in cases where
  // the interrupt is triggered while the buffer is being read from, for the sake of
  // accuracy the values must be stored in a contingency buffer. Then, when the interrupt
  // is triggered and the reading is over, the values are transferred from the contingency
  // buffer to analogBuffer.
  static int contigBuffer[fft_size];
  static int contigBuffer_index = 0;
  if(!analogBuffer_availible){
    contigBuffer[contigBuffer_index] = analogRead(inputPin) - 2048;
    contigBuffer_index++;
  }
  else{
    for(int i = 0; i < contigBuffer_index; i++)
      analogBuffer_store(contigBuffer[i]);
    contigBuffer_index = 0;
    analogBuffer_store(analogRead(inputPin) - 2048);
  }
}

// Core 1 thread
void setup(){
  SPI.begin();
  SPI.setDataMode(SPI_MODE3);
  SPI.setFrequency(40000000);

  pinMode(LATCHPIN, OUTPUT);
  pinMode(ENABLEPIN, OUTPUT);
  pinMode(A_PIN, OUTPUT);
  pinMode(B_PIN, OUTPUT);
  pinMode(C_PIN, OUTPUT);
  pinMode(D_PIN, OUTPUT);
  
  pinMode(39, INPUT);
  pinMode(36, INPUT);
  pinMode(0, INPUT);

  fft_power = 0;
  while(1 << fft_power != fft_size) fft_power++;

  // Intializes interrupt at user-set frequency
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, sample_period, true); // 1000000 * 1 us = 1 s, use this conversion
  timerAlarmEnable(timer);

  // Intializes Core 0
  displayBuffer_availible = false;
  xTaskCreatePinnedToCore(
              Task1code,   /* Task function. */
              "Task1",     /* name of task. */
              10000,       /* Stack size of task */
              NULL,        /* parameter of the task */
              0,           /* priority of the task */
              &Task1,      /* Task handle to keep track of created task */
              0);          /* pin task to core 0 */ 
}

void loop(){
  int8_t vReal[fft_size];
  int8_t vImag[fft_size] = {0};
  int preprocess[fft_size];
  
  // Reads entire analog buffer for FFT calculations. This means a LOT of
  // redundant data but its necessary to because using new data every time
  // requires very long delays in total when recording at low frequencies.
  analogBuffer_availible = false;   // Closes off buffer to prevent corruption
  // Reads entire circular buffer, starting from analogBuffer_index
  for(int i = 0; i < fft_size; i++){
    preprocess[i] = analogBuffer[(i+analogBuffer_index)%fft_size] / 16;
  }
  analogBuffer_availible = true;    // Restores access to buffer


  // Finds the average, which represents the DC bias, and subtracts it. This
  // SHOULD produce usable 0th bin values (currenty 0.1uF capacitors in signal
  // path block low frequencies because they seemed to be really large anyway?)
  int sum = 0;
  for(int i = 0; i < fft_size; i++) sum += preprocess[i];
  int avg = sum / fft_size;
  for(int i = 0; i < fft_size; i++) vReal[i] = preprocess[i] - avg;

  fix_fft(vReal,vImag,fft_power,0); // Performs FFT calculations

  // Performs multiple operations: flattening, post-processing, and smoothing
  static float buff[COLUMNS_COUNT];
  uint8_t postprocess[COLUMNS_COUNT];
  for(int iCol = 0; iCol < COLUMNS_COUNT; iCol++){
    // Combining imaginary and real data into a unified array
    postprocess[iCol] = SENSITIVITY*sqrt(vReal[iCol]*vReal[iCol] + vImag[iCol]*vImag[iCol]);
    // Logarithmic scaling to create more visible output display height of 16
    if(postprocess[iCol] > 1) postprocess[iCol] = 40.*log((float)(postprocess[iCol]));
    else postprocess[iCol] = 0;   // Cuts off negative values before they are calculated
    //
    if(postprocess[iCol] > buff[iCol]){
      // Smoothing by factoring in past data
      postprocess[iCol] = buff[iCol] * anti_coeff + (float)postprocess[iCol] * coeff;
      buff[iCol] = postprocess[iCol];       // Storing new output as next frame's past data
    }
    else{
      postprocess[iCol] = buff[iCol] * anti_coeff2 + (float)postprocess[iCol] * coeff2;
      buff[iCol] = postprocess[iCol];
    }
  }

  // Translating output data into column heights, which is entered into the buffer
  displayBuffer_availible = false;  // Blocks access to display buffer
  for(int i = 0; i < array_size; i++) writeBuffer[i] = 0;
  for(int iCol = 0; iCol < COLUMNS_COUNT; iCol++){  
    int height = (postprocess[iCol]*ROWS_COUNT)/CAP;
    for(int iRow = 0; iRow < ROWS_COUNT; iRow++)
      if(height >= ROWS_COUNT-iRow || iRow == ROWS_COUNT-1) writeBuffer[iRow*elem_count + iCol/8] |= 1 << (7-iCol%8);
  }
  displayBuffer_availible = true;   // Restores access to display buffer

  // Credit goes to akshar001 for this watchdog reset three-liner
  // Github thread: https://github.com/espressif/arduino-esp32/issues/595
  TIMERG0.wdt_wprotect=TIMG_WDT_WKEY_VALUE;
  TIMERG0.wdt_feed=1;
  TIMERG0.wdt_wprotect=0;
}

// Function defintions

// Flashes 64*16 HUB08 display with an output specified by the passed array.
// Reads the array "row" by "row", left to right. (Each row is 64 bits, or
// 8 uint8_t elements long.) Should be run continuously, so that the higher
// the flash interval the brighter the display (max unmeasured, but exists),
// but can be asynchronously executed if needed. Bitbangs HUB12 protocol
// with hardware SPI and additonal pins.
void flashDisplay(uint8_t frame[array_size], int interval){
  for(uint8_t iRow = 0; iRow < ROWS_COUNT; iRow++){
    // Loads in a row, each element/column holding 8 pixels from left to right
    uint8_t column[elem_count];
    for(int iCol = 0; iCol < elem_count; iCol++) column[iCol] = frame[elem_count*iRow + iCol];

    // Outputting over HUB12
    digitalWrite(ENABLEPIN, HIGH);  // Turns display off (redundant?)
    
    // Sends data column by column, protocol emulated by SPI mode 3
    for(int iCol = 0; iCol < elem_count; iCol++) SPI.transfer(column[iCol]);
    
    digitalWrite(LATCHPIN, LOW);
    digitalWrite(LATCHPIN, HIGH);
    
    // Outputs row number in binary
    digitalWrite(A_PIN, iRow & 0b00000001);
    digitalWrite(B_PIN, (iRow & 0b00000010) >> 1);
    digitalWrite(C_PIN, (iRow & 0b00000100) >> 2);
    digitalWrite(D_PIN, (iRow & 0b00001000) >> 3);

    digitalWrite(ENABLEPIN, LOW);   // Turns display back on
    delayMicroseconds(interval);    // Allow the display to stay on
    digitalWrite(ENABLEPIN, HIGH);  // Turns display back off because display
                                    //  has a maximum interval the last row
                                    //  will be stuck on for otherwise.
  }
}
// Function note: If the maximum is known (or arbitratily set), then one can pwm
//  that interval to obtain variable brightness but constant execution time (if
//  needed).


// Stores the passed value into a 128-sized circular buffer called analogBuffer,
//  which is declared as a volatile int array. Uses global volatile int
//  analogBuffer_index as the write index and reads analogBuffer_availible as
//  memory flag.
void analogBuffer_store(int val){
  analogBuffer[analogBuffer_index] = val;

  // Increments index then uses modulo to limit range
  analogBuffer_index++;
  analogBuffer_index %= fft_size;
}
// Function note: Currently, analogBuffer_index is also used as the reference
// in order to read the entire array in chronological order.
