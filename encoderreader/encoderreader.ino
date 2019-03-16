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

// Constants for i2c parameters
#define I2C_ENCODER_ADDR 8
#define I2C_RATE 100000L

// Variables to keep track of the counts received over i2c
volatile long counts = 0;

/**
 * This union is used to easily convert from the int32 value of the encoder variable to
 * bytes for i2c transmission.
 * WARNING: This makes assumptions about the endianness of the two sides of the i2c transmission.
 */
union {
   int32_t value;
   byte arr[sizeof(int32_t)];
} int32_byte_converter;


// Set up the serial and i2c communications
void setup() {

  // Initialize serial port
  Serial.begin(115200);
  
  // Initialize i2c as master device
  Wire.begin();
  Wire.setClock(I2C_RATE);
}

/*
 * Main loop to request encoder position from Nano
 */
void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("requestFrom");

  // Read the encoder position from the Nano over i2c, and measure elapsed time
  long startTime = millis();
  Wire.requestFrom(I2C_ENCODER_ADDR, 4);    // request 4 bytes from slave device #8
  int count = 0;
  while (count < 4)
  {
    if (Wire.available()) { // slave may send less than requested
      char c = Wire.read(); // receive a byte as character
      int32_byte_converter.arr[count] = c;
    
      count++;
      //Serial.println("     byte received");
    }
  }
  long endTime = millis();

  counts = int32_byte_converter.value;
  Serial.print("Value: ");
  Serial.print(counts);
  Serial.print("    Elapsed: ");
  Serial.println(endTime-startTime);

  delay(1000);
}
