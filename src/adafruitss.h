/*
 * Author: Stan Gifford <stan@gifford.id.au>
 * Copyright (c) 2015 Intel Corporation.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#pragma once

#include <mraa/i2c.h>

#define MAX_BUFFER_LENGTH 6

//namespace myupm {

 /**
  * @brief Adafruit PCA9685 based servo controller library
  * @defgroup adafruitss libupm-adafruitss
  * @ingroup adafruit i2c servos
  */

 /**
  * @library adafruitss
  * @sensor adafruitss
  * @comname Adafruit Servo Shield
  * @type servos
  * @man adafruit
  * @web http://www.adafruit.com/product/1411
  * @con i2c
  *
  * @brief C++ API for Adafruit Servo Shield
  *
  *	UPM library for the PCA9685 based Adafruit 16-channel servo shield. When 3
  * or more GWS servos attached results unpredictable. Adafruit do recommend a
  * capacitor be installed on the board which should alleviate the issue.
  * Sizing depends on servos and count.
  *
  * @snippet adafruitss.cxx Interesting
  */
  class adafruitss {
  public:
    /**
     * Creates a adafruitss object
     *
     * @param bus number of used i2c bus
     * @param i2c_address address of servo controller on i2c bus
     */
    adafruitss(int bus, int i2c_address);
    int update(void);
    /**
     * Sets the frequency for your servos
     *
     * @param freq the frequency at which the servos operate
     */
    void setPWMFreq(float freq);
    /**
     * Moves the one of the servos to the specified angle
     *
     * @param port port of the servo on the controller (servo number)
     * @param servo_type can be 0 = standard 1ms to 2ms, 1 = extended 0.6ms to 2.4ms, or 2 = extended 0.8ms to 2.2ms
     * @param degrees angle to set the servo to
     */
    void servo(uint8_t port, uint8_t servo_type, float degrees);
    void servo(uint8_t port, uint8_t servo_type, uint16_t degrees) { servo(port, servo_type, (float)degrees); }

  private:

    int pca9685_addr;
    mraa_i2c_context m_i2c;
    uint8_t m_rx_tx_buf[MAX_BUFFER_LENGTH];
    float _duration_1ms;
};

//}
