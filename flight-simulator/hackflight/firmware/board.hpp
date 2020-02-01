/*
   board.hpp : class header for board-specific routines

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#ifdef __arm__
extern "C" {
#endif

    class Board {

        public:

            // your implementation should support these methods

            // Core functionality
            static void     debug(char c);
            static void     delayMilliseconds(uint32_t msec);
            static uint32_t getMicros();
            static void     imuInit(uint16_t & acc1G, float & gyroScale);
            static void     imuRead(int16_t accADC[3], int16_t gyroADC[3]);
            static void     init(uint32_t & imuLooptimeUsec, uint32_t & calibratingGyroMsec);
            static void     ledSetState(uint8_t id, bool state);
            static uint16_t rcReadPWM(uint8_t chan);
            static bool     rcUseSerial(void);
            static uint16_t rcReadSerial(uint8_t chan);
            static bool     rcSerialReady(void); 
            static uint8_t  serialAvailableBytes(void);
            static uint8_t  serialReadByte(void);
            static void     serialWriteByte(uint8_t c);
            static void     writeMotor(uint8_t index, uint16_t value);

            // Helps with simulation
            static void     showArmedStatus(bool armed);
            static void     showAuxStatus(uint8_t status);

            // STM32
            static void     checkReboot(bool pendReboot);
            static void     reboot(void);

            // Baro
            static bool     baroInit(void);
            static void     baroUpdate(void);
            static int32_t  baroGetPressure(void);

            // Sonar
            static bool     sonarInit(uint8_t index);
            static void     sonarUpdate(uint8_t index);
            static uint16_t sonarGetDistance(uint8_t index);

            // Default constants
            static const uint32_t DEFAULT_IMU_LOOPTIME_USEC     = 3500;
            static const uint32_t DEFAULT_GYRO_CALIBRATION_MSEC = 3500;

    }; // class Board


#ifdef __arm__
} // extern "C"
#endif
