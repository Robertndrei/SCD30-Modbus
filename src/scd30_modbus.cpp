#include "scd30_modbus.h"

/*
 * @brief Calculate CRC-16 (Modbus)
 * @param buf Buffer to calculate
 * @param len Length of the buffer
 * @returns Two bytes with the crc
 */

unsigned char *SCD30_Modbus::calculateCRC(unsigned char *buf, uint16_t len) {
   static unsigned char crc[2];

   uint16_t crc_int = 0xFFFF;
   for (uint16_t pos = 0; pos < len; pos++){
      crc_int ^= (uint16_t)buf[pos];         // XOR byte into least sig. byte of 
                                             // crc

      for (uint16_t i = 8; i != 0; i--) {    // Loop over each bit
         if ((crc_int & 0x0001) != 0) {      // If the LSB is set
            crc_int >>= 1;                   // Shift right and XOR 0xA001
            crc_int ^= 0xA001;
         }
         else                                // Else LSB is not set
            crc_int >>= 1;                   // Just shift right
      }
   }

   crc[0] = crc_int & 0x00FF;
   crc[1] = (crc_int & 0xFF00) >> 8;

   return crc;
}

/*
 * @brief Check if received data crc is correct
 * @param buf Buffer to check
 * @param len Length of the buffer
 * @returns true: correct, false: incorrect
 */

bool SCD30_Modbus::checkCRC(unsigned char *buf, uint16_t len) {
   uint8_t target_length = len - 2;
   unsigned char target_buf[target_length];
   unsigned char *calculated_crc;

   for ( uint8_t i = 0; i < target_length; i++ ) {
      target_buf[i] = buf[i];
   }

   calculated_crc = calculateCRC(target_buf, target_length);

   if (calculated_crc[1] == buf[len - 1] && calculated_crc[0] == buf[len - 2]) {
      return true;
   } else {
      return false;
   }
}

/*
 * @brief Write data via serial to scd30
 * @param func Function code
 * @param cmd SCD30 command
 * @param data User data
 * @param addr SCD30 modbus address
 */

void SCD30_Modbus::scd30Write(uint8_t func, uint16_t cmd, uint16_t data, 
                              uint8_t addr = SCD30_MODBUSADDR_DEFAULT) {

   unsigned char buffer[6];
   unsigned char *crc;

   buffer[0] = addr;
   buffer[1] = func;
   buffer[2] = (cmd & 0xFF00) >> 8;
   buffer[3] = cmd & 0x00FF;
   buffer[4] = (data & 0xFF00) >> 8;
   buffer[5] = data & 0x00FF;

   crc = calculateCRC(buffer, 6);

   serial->write(buffer[0]);
   serial->write(buffer[1]);
   serial->write(buffer[2]);
   serial->write(buffer[3]);
   serial->write(buffer[4]);
   serial->write(buffer[5]);
   serial->write(crc[0]);
   serial->write(crc[1]);
}

/*
 * @brief Read data via serial from scd30
 * @param buf Buffer to store the data
 * @param bytes Number of bytes waiting to receive
 * @return True if the data was received correctrly and the crc is correct,
 * False if not
 */

bool SCD30_Modbus::scd30Read(unsigned char *buf, uint8_t bytes) {
   serial->readBytes(buf, bytes);

   if ( checkCRC(buf, bytes) ) {
      return true;
   } else {
      return false;
   }
}

/*
 * @brief Configure communication with scd30
 * @param serialPort Serial port to use, ie:
 * Arduino UNO -> Serial
 * Ardnino Nano Every -> Serial1
 * Arduino Mega -> Serial or Serial1 or Serial2 or Serial3
 * @param baudrate The baud rate speed for the transmission
 */

void SCD30_Modbus::begin(HardwareSerial *serialPort, int baudrate) {
   serial = serialPort;
   serial->begin(baudrate);
}

/*
 * @bried Restarts the sensor
 */

void SCD30_Modbus::reset(void) {
   uint8_t len = 8;
   unsigned char response[len];

   scd30Write(0x06, SCD30_CMD_SOFT_RESET, 0x0001);
   delay(30);
}

/*
 * @brief Get if the sensor have a data ready to read
 * @return status true: read, false: not ready
 */

bool SCD30_Modbus::dataReady(void) {
   uint8_t len = 7;
   unsigned char response[len];

   scd30Write(0x03, SCD30_CMD_GET_DATA_READY, 0x0001);

   if ( scd30Read(response, len) ) {
      if ( response[4] == 0x01 ) {
         return true;
      } else {
         return false;
      }
   } else {
      return false;
   }
}

/*
 * @brief Read a single measurement of CO2 concentration
 * @return True if the measurement was readed correctly
 */

bool SCD30_Modbus::read(void) {
   uint8_t len = 17;
   unsigned char response[len];

   scd30Write(0x03, SCD30_CMD_READ_MEASUREMENT, 0x0006);

   if ( scd30Read(response, len) ) {
      // CRCs are good, unpack floats
      uint32_t co2 = 0, temp = 0, hum = 0;

      co2 |= response[3];
      co2 <<= 8;
      co2 |= response[4];
      co2 <<= 8;
      co2 |= response[5];
      co2 <<= 8;
      co2 |= response[6];

      temp |= response[7];
      temp <<= 8;
      temp |= response[8];
      temp <<= 8;
      temp |= response[9];
      temp <<= 8;
      temp |= response[10];

      hum |= response[11];
      hum <<= 8;
      hum |= response[12];
      hum <<= 8;
      hum |= response[13];
      hum <<= 8;
      hum |= response[14];

      CO2_raw[0] = response[3];
      CO2_raw[1] = response[4];
      CO2_raw[2] = response[5];
      CO2_raw[3] = response[6];

      temperature_raw[0] = response[7];
      temperature_raw[1] = response[8];
      temperature_raw[2] = response[9];
      temperature_raw[3] = response[10];

      relative_humidity_raw[0] = response[11];
      relative_humidity_raw[1] = response[12];
      relative_humidity_raw[2] = response[13];
      relative_humidity_raw[3] = response[14];

      memcpy(&CO2, &co2, sizeof(CO2));
      memcpy(&temperature, &temp, sizeof(temperature));
      memcpy(&relative_humidity, &hum, sizeof(relative_humidity));

      return true;
   } else {
      return false;
   }
}

/*
 * @brief Sets the inverval for continuous measurement mode. Standard 
 * measurement interval is 2
 * @param interval Format uint16, interval in seconds. Available range:
 * [2 ... 1800] given in 2 bytes in the order MSB, LSB
 * @return If the command was sent correctly
 */

bool SCD30_Modbus::setMeasurementInterval(uint16_t interval) {
   uint8_t len = 8;
   unsigned char response[len];

   scd30Write(0x06, SCD30_CMD_SET_MEASUREMENT_INTERVAL, interval);

   delay(5);

   if ( scd30Read(response, len) ) {
      if ( response[4] == (interval & 0xFF00) >> 8 &&
           response[5] == (interval & 0x00FF) ) {
         return true;
      } else {
         return false;
      }
   } else {
      return false;
   }
}

/*
 * @brief Get the inverval for continuous measurement mode
 * @return The interval in seconds 
 */

uint16_t SCD30_Modbus::getMeasurementInterval(void) {
   uint8_t len = 7;
   unsigned char response[len];
   uint16_t data = 0;

   scd30Write(0x03, SCD30_CMD_SET_MEASUREMENT_INTERVAL, 0x0001);

   delay(5);

   if ( scd30Read(response, len) ) {
      data = response[3] << 8;
      data = data | response[4];
      return data;
   } else {
      return 0;
   }
}

/*
 * @brief Activate or Deactivate automatic self-calibration (ASC)
 * @param enable True to enable, False to disable
 * @return If the command was sent correctly
 */

bool SCD30_Modbus::selfCalibrationEnabled(bool enable) {
   uint8_t len = 8;
   unsigned char response[len];
   uint16_t data;

   if ( enable ) {
      data = 0x0001;
   } else {
      data = 0x0000;
   }

   scd30Write(0x06, SCD30_CMD_AUTOMATIC_SELF_CALIBRATION, data);

   delay(5);

   if ( scd30Read(response, len) ) {
      if ( response[4] == (data & 0xFF00) >> 8 &&
           response[5] == (data & 0x00FF) ) {
         return true;
      } else {
         return false;
      }
   } else {
      return false;
   }
}

/*
 * @brief Get status of automatic self-calibration (ASC)
 * @return The status
 */

bool SCD30_Modbus::selfCalibrationEnabled(void) {
   uint8_t len = 7;
   unsigned char response[len];
   uint16_t data = 0;

   scd30Write(0x03, SCD30_CMD_AUTOMATIC_SELF_CALIBRATION, 0x0001);

   delay(5);

   if ( scd30Read(response, len) ) {
      data = response[3] << 8;
      data = data | response[4];
      return data;
   } else {
      return 0;
   }
}

/*
 * @brief Triggers continuous measurement
 * @param pressure To compensate ambient pressure. Format uint16, available
 * range 0 & [700 ... 1400], pressure in [mBar]. 0 deactivates presure 
 * compensation
 * @return If the command was sent correctly
 */

bool SCD30_Modbus::startContinuousMeasurement(uint16_t pressure) {
   uint8_t len = 8;
   unsigned char response[len];

   scd30Write(0x06, SCD30_CMD_CONTINUOUS_MEASUREMENT, pressure);

   delay(5);

   if ( scd30Read(response, len) ) {
      if ( response[4] == (pressure & 0xFF00) >> 8 &&
           response[5] == (pressure & 0x00FF) ) {
         return true;
      } else {
         return false;
      }
   } else {
      return false;
   }
}
bool SCD30_Modbus::stopContinuousMeasurement() {
   uint8_t len = 8;
   unsigned char response[len];

   scd30Write(0x06, SCD30_CMD_STOP_MEASUREMENTS, 0x001);

   delay(5);

   if ( scd30Read(response, len) ) {
      if ( response[4] == 0x00 &&
           response[5] == 0x01 ) {
         return true;
      } else {
         return false;
      }
   } else {
      return false;
   }
}

/*
 * @brief Get ambient pressure offset
 * @return the pressure
 */

uint16_t SCD30_Modbus::getAmbientPressureOffset(void) {
   uint8_t len = 7;
   unsigned char response[len];
   uint16_t data = 0;

   scd30Write(0x03, SCD30_CMD_CONTINUOUS_MEASUREMENT, 0x0001);

   delay(5);

   if ( scd30Read(response, len) ) {
      data = response[3] << 8;
      data = data | response[4];
      return data;
   } else {
      return 0;
   }
}

/*
 * @brief Set altitude offset 
 * @param altitude Format uint16, height over sea level in [m] above 0
 * @return If the command was sent correctly
 */

bool SCD30_Modbus::setAltitudeOffset(uint16_t altitude) {
   uint8_t len = 8;
   unsigned char response[len];

   scd30Write(0x06, SCD30_CMD_SET_ALTITUDE_COMPENSATION, altitude);

   delay(5);

   if ( scd30Read(response, len) ) {
      if ( response[4] == (altitude & 0xFF00) >> 8 &&
           response[5] == (altitude & 0x00FF) ) {
         return true;
      } else {
         return false;
      }
   } else {
      return false;
   }
}

/*
 * @brief Get altitude offset 
 * @return The altitude offset
 */

uint16_t SCD30_Modbus::getAltitudeOffset(void) {
   uint8_t len = 7;
   unsigned char response[len];
   uint16_t data = 0;

   scd30Write(0x03, SCD30_CMD_SET_ALTITUDE_COMPENSATION, 0x0001);

   delay(5);

   if ( scd30Read(response, len) ) {
      data = response[3] << 8;
      data = data | response[4];
      return data;
   } else {
      return 0;
   }
}

/*
 * @brief Set temperature offset
 * @param temp_offset Format uint16, unit [ºC x 100], ie:
 * one tick corresponds to 0.01ºC
 * @return If the command was sent correctly
 */

bool SCD30_Modbus::setTemperatureOffset(uint16_t temp_offset) {
   uint8_t len = 8;
   unsigned char response[len];

   scd30Write(0x06, SCD30_CMD_SET_TEMPERATURE_OFFSET, temp_offset);

   delay(5);

   if ( scd30Read(response, len) ) {
      if ( response[4] == (temp_offset & 0xFF00) >> 8 &&
           response[5] == (temp_offset & 0x00FF) ) {
         return true;
      } else {
         return false;
      }
   } else {
      return false;
   }
}

/*
 * @brief Get temperature offset
 * @return The temperature offset
 */

uint16_t SCD30_Modbus::getTemperatureOffset(void) {
   uint8_t len = 7;
   unsigned char response[len];
   uint16_t data = 0;

   scd30Write(0x03, SCD30_CMD_SET_TEMPERATURE_OFFSET, 0x0001);

   delay(5);

   if ( scd30Read(response, len) ) {
      data = response[3] << 8;
      data = data | response[4];
      return data;
   } else {
      return 0;
   }
}

/*
 * @brief Set forced recalibration value (FCR)
 * @param reference Format uint16, CO2 concentration in [ppm]
 * @return If the command was sent correctly
 */

bool SCD30_Modbus::forceRecalibrationWithReference(uint16_t reference) {
   uint8_t len = 8;
   unsigned char response[len];

   scd30Write(0x06, SCD30_CMD_SET_FORCED_RECALIBRATION_REF, reference);

   delay(5);

   if ( scd30Read(response, len) ) {
      if ( response[4] == (reference & 0xFF00) >> 8 &&
           response[5] == (reference & 0x00FF) ) {
         return true;
      } else {
         return false;
      }
   } else {
      return false;
   }
}

/*
 * @brief Get forced recalibration value (FCR)
 * @return The reference value
 */

uint16_t SCD30_Modbus::getForcedCalibrationReference(void) {
   uint8_t len = 7;
   unsigned char response[len];
   uint16_t data = 0;

   scd30Write(0x03, SCD30_CMD_SET_FORCED_RECALIBRATION_REF, 0x0001);

   delay(5);

   if ( scd30Read(response, len) ) {
      data = response[3] << 8;
      data = data | response[4];
      return data;
   } else {
      return 0;
   }
}
