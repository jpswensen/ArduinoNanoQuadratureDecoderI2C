# ArduinoNanoQuadratureDecoderI2C
This implements a fast quadrature decoder using interrupt on change pins and can respond to i2c request to supply the current count. The purpose of this code is to offload the reading of quadrature decoding onto a separate board as it can be computationally intensive, especially when there are a lot of encoder counts per revolution of a fast rotating motor.

This effort was spawned out of the ME401 Mechatronics course I teach at Washington State University. We had been using the Digilent chipKit uc32 processors and using their 10 microsecond timer to ensure we weren't missing any encoder counts. This worked great, but then when having up to three servo motors running at the same time, which also rely on the chipKit's microsecond timer, their output was a little bit jittery and the servos would twich. This would allow us to offload the quadrature decoding onto another board (Arduino Nano and clones can be purchased for as little as $2.50 each) and free up pins and timer interrupt computation time for other things and stop the servo jittering.

# i2c Interface
As is typical for the Arduino devices, i2c communications is implemented on the A4 (SCL) and A5 (SDA) pins. The code currently has this i2c device set up with ID 8, but this could easily be changed. This code expects you to use the requestFrom() functionality of the Wire.h library from the master device in order to get

# Encoder connections
The Arduino Nano only has interrupt-on-change external interrupts on pins D2 and D3. You encoder must be attached to these two pins. The order in which these are attached will control whether the direction you turn your motor counts up or counts down, relative to the direction you motor is turned.

# Sketches included in this repository
There are two sketches included in this repository.

A) nanoencoder - This is the code that runs on the Arduino Nano, reads the encoder outputs, and listens for requests over i2c. The three important pieces that you may want to change are the I2C slave address (I2C_ADDR), the I2C data rate (I2C_RATE), and to comment the #define DEBUG_PRINT to turn off output.

B) encoderreader - This is the code that would run on a separate Arduino to retrieve the encoder information over the i2c bus by requesting the information as the i2c master device.


