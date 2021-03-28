#ifndef SCD30_MODBUS_H
#define SCD30_MODBUS_H

#include <Arduino.h>

// SCD30 default modbus baudrate
#define SCD30_MODBUSBAUDRATE_DEFAULT 19200 
// SCD30 default modbus address
#define SCD30_MODBUSADDR_DEFAULT 0x61 
// Main data register
#define SCD30_CMD_READ_MEASUREMENT 0x0028
// Command to start continuous measurement
#define SCD30_CMD_CONTINUOUS_MEASUREMENT 0x0036
// Command to stop measurements 
#define SCD30_CMD_STOP_MEASUREMENTS 0x0037
// Command to set measurement interval
#define SCD30_CMD_SET_MEASUREMENT_INTERVAL 0x0025
// Data ready reg
#define SCD30_CMD_GET_DATA_READY 0x0027 
// enables/disables auto calibration
#define SCD30_CMD_AUTOMATIC_SELF_CALIBRATION 0x003A
// Forces calibration with given value
#define SCD30_CMD_SET_FORCED_RECALIBRATION_REF 0x0039
// Specifies the temp offset
#define SCD30_CMD_SET_TEMPERATURE_OFFSET 0x003B
// Specifies altitude offset
#define SCD30_CMD_SET_ALTITUDE_COMPENSATION 0x0038
// Soft reset!
#define SCD30_CMD_SOFT_RESET 0x0034
// Firmware revision number
#define SCD30_CMD_READ_REVISION 0x0020

class SCD30_Modbus {

   private:
      HardwareSerial *serial;

      unsigned char *calculateCRC(unsigned char *buf, uint16_t len);
      bool checkCRC(unsigned char *buf, uint16_t len);
      void scd30Write(uint8_t func, uint16_t cmd, uint16_t data,
                      uint8_t addr = SCD30_MODBUSADDR_DEFAULT);
      bool scd30Read(unsigned char *buf, uint8_t bytes);

   public:
      void begin(HardwareSerial *serialPort, int baudrate = SCD30_MODBUSBAUDRATE_DEFAULT);
      
      void reset(void);
      
      bool dataReady(void);

      bool read(void);
      
      bool setMeasurementInterval(uint16_t interval);
      uint16_t getMeasurementInterval(void);

      bool selfCalibrationEnabled(bool);
      bool selfCalibrationEnabled(void);

      bool startContinuousMeasurement(uint16_t pressure = 0);
      bool stopContinuousMeasurement();
      uint16_t getAmbientPressureOffset(void);

      bool setAltitudeOffset(uint16_t altitude);
      uint16_t getAltitudeOffset(void);

      bool setTemperatureOffset(uint16_t temp_offset);
      uint16_t getTemperatureOffset(void);

      bool forceRecalibrationWithReference(uint16_t reference);
      uint16_t getForcedCalibrationReference(void);

      float CO2,               // The most recent CO2 reading
            temperature,       // The most recent temperature reading
            relative_humidity; // The most recent relative_humidity reading

      unsigned char CO2_raw[4],               // The most recent CO2 reading
                       temperature_raw[4],       // The most recent temperature 
                                              // reading
                       relative_humidity_raw[4]; // The most recent 
                                              // relative_humidity reading
};

#endif