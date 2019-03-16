/*
 * Copyright (c) 2019 John P. Swensen <jpswensen@gmail.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of mosquitto nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <Wire.h>

#define DEBUG_PRINT

// Constants for i2c parameters and pin numbers
#define I2C_ADDR 8
#define I2C_RATE 100000L

#define PIN_CHANA 2
#define PIN_CHANB 3


// Variables to keep track of the channel state and the motor positions
volatile boolean error = 0;
volatile char prev_chA = 0;
volatile char prev_chB = 0;
volatile long counts = 0;

/**
 * This is the interrupt on change function that will handle finding the true 
 */
void enc_change_ISR (void) {
  char curr_chA = digitalRead(PIN_CHANA);
  char curr_chB = digitalRead(PIN_CHANB);

  // This is just fancy bit-wise math to implement the increment or decrement in a quick manner.
  // The alternative would be a long if..elseif..else statement checking the previous and current state
  // of the pins.
  counts += (curr_chA ^ prev_chB) - (prev_chA ^ curr_chB); 

  // This also checks whether two steps were made an sets an error flag. I the slow-ish motor I am testing
  // on, I haven't seen an error for missing steps except when I had a finicky solder joint.
  if((prev_chA ^ curr_chA) & (prev_chB ^ curr_chB))
  {
    error = true;
  }

  // Store the current readings as the previous pin state for the next pin change.s
  prev_chA = curr_chA;
  prev_chB = curr_chB;
}

/**
 * This union is used to easily convert from the int32 value of the encoder variable to
 * bytes for i2c transmission.
 * WARNING: This makes assumptions about the endianness of the two sides of the i2c transmission.
 */
union {
   int32_t value;
   byte arr[sizeof(int32_t)];
} int32_byte_converter;


/**
 * This is the interrupt service routine that handles requests over i2c.
 */
void countsRequestISR() {

#ifndef DEBUG_PRINT
  Serial.println("in i2c isr");
#endif

  // Copy the counts variable into the union variable and then send them out one by one.
  int32_byte_converter.value = counts;  
  Wire.write(&int32_byte_converter.arr[0], 1);
  Wire.write(&int32_byte_converter.arr[1], 1);
  Wire.write(&int32_byte_converter.arr[2], 1);
  Wire.write(&int32_byte_converter.arr[3], 1);
}

/**
 * Setup both the interrupt on change for detecting the encoder and the i2c for reporting the results
 * back to the i2c master.
 */
void setup() {
  
  // Setup the serial monitor
  Serial.begin(115200);

  // Set up up the pins as inputs and attach the interrupt for the timers
  pinMode (PIN_CHANA, INPUT);
  pinMode (PIN_CHANB, INPUT);

  // Read in initial state of the encoder pins
  prev_chA = digitalRead(PIN_CHANA);
  prev_chB = digitalRead(PIN_CHANB);

  // Start the interrupt on change to catch the encoder state changes
  attachInterrupt(digitalPinToInterrupt(PIN_CHANA), enc_change_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_CHANB), enc_change_ISR, CHANGE);

  // Setup this arduino as an i2c slave
  Wire.begin(I2C_ADDR);
  Wire.setClock(I2C_RATE);
  Wire.onRequest(countsRequestISR);
}


void loop() {
  // We really don't need to do anything in this loop, but will add some debug output 
  // to make sure things are working right.
#ifdef DEBUG_PRINT
  static int errorCount = 0;
  
  Serial.print("millis: ");
  Serial.print(millis());
  Serial.print("     counts: ");  
  Serial.print(counts);
  Serial.print("     error: ");
  Serial.println(error);

  if (error)
  {
    ++errorCount;
    Serial.print("Error in missing steps:");
    Serial.println(errorCount);
    error = 0;
  }
  
  delay(100);
#endif

}
