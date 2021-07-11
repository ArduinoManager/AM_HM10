/*
   This program is for calibrating the MPU-9250 for the Compass Widget

   Author: Fabrizio Boco - fabboco@gmail.com

   Version: 1.0

   07/11/2021

   All rights reserved

   This work is based on code by LINGIB
    https://www.instructables.com/member/lingib/instructables/
*/

/*

   AMController libraries, examples (“The Software”) and the related documentation (“The Documentation”) are supplied to you
   by the Author in consideration of your agreement to the following terms, and your use or installation of The Software and the use of The Documentation
   constitutes acceptance of these terms.
   If you do not agree with these terms, please do not use or install The Software.
   The Author grants you a personal, non-exclusive license, under author's copyrights in this original software, to use The Software.
   Except as expressly stated in this notice, no other rights or licenses, express or implied, are granted by the Author, including but not limited to any
   patent rights that may be infringed by your derivative works or by other works in which The Software may be incorporated.
   The Software and the Documentation are provided by the Author on an "AS IS" basis.  THE AUTHOR MAKES NO WARRANTIES, EXPRESS OR IMPLIED, INCLUDING WITHOUT
   LIMITATION THE IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE, REGARDING THE SOFTWARE OR ITS USE AND OPERATION
   ALONE OR IN COMBINATION WITH YOUR PRODUCTS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, INDIRECT, INCIDENTAL OR CONSEQUENTIAL DAMAGES (INCLUDING,
   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) ARISING IN ANY WAY OUT OF THE USE,
   REPRODUCTION AND MODIFICATION OF THE SOFTWARE AND OR OF THE DOCUMENTATION, HOWEVER CAUSED AND WHETHER UNDER THEORY OF CONTRACT, TORT (INCLUDING NEGLIGENCE),
   STRICT LIABILITY OR OTHERWISE, EVEN IF THE AUTHOR HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

  -------------
  -------------
  -------------
  -------------
   WARNING:
  -------------
  -------------
  -------------
  -------------
   Do NOT use this compass in situations involving safety to life including but not limited to navigation at sea.
  -------------
  -------------
  -------------
  -------------
*/


/******* Instructions

    1) Upload this program to your board
    2) Open the Serial Monitor and set speed to 115200 baud
    3) When the calibration is completed, copy the displayed values to the corresponding variables in the ArduinoManagerCompass.ino

********/

// ----- Libraries
#include <Wire.h>

// ----- Gyro
#define MPU9250_I2C_address 0x68                  // I2C address for MPU9250
#define MPU9250_I2C_master_enable 0x6A            // USER_CTRL[5] = I2C_MST_EN
#define MPU9250_Interface_bypass_mux_enable 0x37  // INT_PIN_CFG[1]= BYPASS_EN

#define Frequency 125     // 8mS sample interval
#define Sensitivity 65.5  // Gyro sensitivity (see data sheet)

#define Sensor_to_deg 1 / (Sensitivity * Frequency)  // Convert sensor reading to degrees
#define Sensor_to_rad Sensor_to_deg* DEG_TO_RAD      // Convert sensor reading to radians

#define Loop_time 1000000 / Frequency  // Loop time (uS)
long Loop_start;                       // Loop start time (uS)

int Gyro_x, Gyro_y, Gyro_z;
long Gyro_x_cal, Gyro_y_cal, Gyro_z_cal;
float Gyro_pitch, Gyro_roll, Gyro_yaw;
float Gyro_pitch_output, Gyro_roll_output;

// ----- Accelerometer
long Accel_x, Accel_y, Accel_z, Accel_total_vector;
float Accel_pitch, Accel_roll;

// ----- Magnetometer
#define AK8963_I2C_address 0x0C            // I2C address for AK8963
#define AK8963_cntrl_reg_1 0x0A            // CNTL[4]=#bits, CNTL[3:0]=mode
#define AK8963_status_reg_1 0x02           // ST1[0]=data ready
#define AK8963_data_ready_mask 0b00000001  // Data ready mask
#define AK8963_overflow_mask 0b00001000    // Magnetic sensor overflow mask
#define AK8963_data 0x03                   // Start address of XYZ data
#define AK8963_fuse_ROM 0x10               // X,Y,Z fuse ROM


int Mag_x_offset = -347, Mag_y_offset = -184, Mag_z_offset = 10;   // Hard-iron offsets
float Mag_x_scale = 0.69, Mag_y_scale = 1.20, Mag_z_scale = 1.40;  // Soft-iron scale factors
float ASAX = 1.21, ASAY = 1.21, ASAZ = 1.17;                       // (A)sahi (S)ensitivity (A)djustment fuse ROM values.


// -----------------
//  Setup
// -----------------
void setup() {
  // ----- Serial communication
  Serial.begin(115200);  //Use only for debugging
  Wire.begin();          //Start I2C as master
  Wire.setClock(400000);

  // ----- Start-up message
  Serial.println(" Arduinoo Manager Compass Calibration V1.0");
  delay(2000);  // Allow time to read

  // ----- Configure the magnetometer
  configure_magnetometer();

  // ----- Calibrate the magnetometer
  calibrate_magnetometer();
}

// ----------------------------
// Main loop
// ----------------------------
void loop() {
}

// ----------------------------
//  Configure magnetometer
// ----------------------------
void configure_magnetometer() {
  /*
     The MPU-9250 contains an AK8963 magnetometer and an
     MPU-6050 gyro/accelerometer within the same package.

     To access the AK8963 magnetometer chip The MPU-9250 I2C bus
     must be changed to pass-though mode. To do this we must:
      - disable the MPU-9250 slave I2C and
      - enable the MPU-9250 interface bypass mux
  */
  // ----- Disable MPU9250 I2C master interface
  Wire.beginTransmission(MPU9250_I2C_address);  // Open session with MPU9250
  Wire.write(MPU9250_I2C_master_enable);        // Point USER_CTRL[5] = I2C_MST_EN
  Wire.write(0x00);                             // Disable the I2C master interface
  Wire.endTransmission();

  // ----- Enable MPU9250 interface bypass mux
  Wire.beginTransmission(MPU9250_I2C_address);      // Open session with MPU9250
  Wire.write(MPU9250_Interface_bypass_mux_enable);  // Point to INT_PIN_CFG[1] = BYPASS_EN
  Wire.write(0x02);                                 // Enable the bypass mux
  Wire.endTransmission();

  // ----- Access AK8963 fuse ROM
  /* The factory sensitivity readings for the XYZ axes are stored in a fuse ROM.
     To access this data we must change the AK9863 operating mode.
  */
  Wire.beginTransmission(AK8963_I2C_address);  // Open session with AK8963
  Wire.write(AK8963_cntrl_reg_1);              // CNTL[3:0] mode bits
  Wire.write(0b00011111);                      // Output data=16-bits; Access fuse ROM
  Wire.endTransmission();
  delay(100);  // Wait for mode change

  // ----- Get factory XYZ sensitivity adjustment values from fuse ROM
  /* There is a formula on page 53 of "MPU-9250, Register Map and Decriptions, Revision 1.4":
      Hadj = H*(((ASA-128)*0.5)/128)+1 where
      H    = measurement data output from data register
      ASA  = sensitivity adjustment value (from fuse ROM)
      Hadj = adjusted measurement data (after applying
  */
  Wire.beginTransmission(AK8963_I2C_address);  // Open session with AK8963
  Wire.write(AK8963_fuse_ROM);                 // Point to AK8963 fuse ROM
  Wire.endTransmission();
  Wire.requestFrom(AK8963_I2C_address, 3);  // Request 3 bytes of data
  while (Wire.available() < 3)
    ;                                          // Wait for the data
  ASAX = (Wire.read() - 128) * 0.5 / 128 + 1;  // Adjust data
  ASAY = (Wire.read() - 128) * 0.5 / 128 + 1;
  ASAZ = (Wire.read() - 128) * 0.5 / 128 + 1;

  // ----- Power down AK8963 while the mode is changed
  /*
     This wasn't necessary for the first mode change as the chip was already powered down
  */
  Wire.beginTransmission(AK8963_I2C_address);  // Open session with AK8963
  Wire.write(AK8963_cntrl_reg_1);              // Point to mode control register
  Wire.write(0b00000000);                      // Set mode to power down
  Wire.endTransmission();
  delay(100);  // Wait for mode change

  // ----- Set output to mode 2 (16-bit, 100Hz continuous)
  Wire.beginTransmission(AK8963_I2C_address);  // Open session with AK8963
  Wire.write(AK8963_cntrl_reg_1);              // Point to mode control register
  Wire.write(0b00010110);                      // Output=16-bits; Measurements = 100Hz continuous
  Wire.endTransmission();
  delay(100);  // Wait for mode change
}

// -------------------------------
//  Calibrate magnetometer
// -------------------------------
void calibrate_magnetometer() {
  // ----- Locals
  int mag_x, mag_y, mag_z;
  int status_reg_2;  // ST2 status register

  int mag_x_min = 32767;  // Raw data extremes
  int mag_y_min = 32767;
  int mag_z_min = 32767;
  int mag_x_max = -32768;
  int mag_y_max = -32768;
  int mag_z_max = -32768;

  float chord_x, chord_y, chord_z;  // Used for calculating scale factors
  float chord_average;

  // ----- Display calibration message
  Serial.println("Rotate Compass");

  // ----- Record min/max XYZ compass readings
  for (int counter = 0; counter < 16000; counter++)  // Run this code 16000 times
  {
    Loop_start = micros();  // Start loop timer
    if (counter % 1000 == 0)
      Serial.print(".");  // Print a dot on the LCD every 1000 readings

    // ----- Point to status register 1
    Wire.beginTransmission(AK8963_I2C_address);  // Open session with AK8963
    Wire.write(AK8963_status_reg_1);             // Point to ST1[0] status bit
    Wire.endTransmission();
    Wire.requestFrom(AK8963_I2C_address, 1);  // Request 1 data byte
    while (Wire.available() < 1)
      ;                                        // Wait for the data
    if (Wire.read() & AK8963_data_ready_mask)  // Check data ready bit
    {
      // ----- Read data from each axis (LSB,MSB)
      Wire.requestFrom(AK8963_I2C_address, 7);  // Request 7 data bytes
      while (Wire.available() < 7)
        ;                                               // Wait for the data
      mag_x = (Wire.read() | Wire.read() << 8) * ASAX;  // Combine LSB,MSB X-axis, apply ASA corrections
      mag_y = (Wire.read() | Wire.read() << 8) * ASAY;  // Combine LSB,MSB Y-axis, apply ASA corrections
      mag_z = (Wire.read() | Wire.read() << 8) * ASAZ;  // Combine LSB,MSB Z-axis, apply ASA corrections
      status_reg_2 = Wire.read();                       // Read status and signal data read

      // ----- Validate data
      if (!(status_reg_2 & AK8963_overflow_mask))  // Check HOFL flag in ST2[3]
      {
        // ----- Find max/min xyz values
        mag_x_min = min(mag_x, mag_x_min);
        mag_x_max = max(mag_x, mag_x_max);
        mag_y_min = min(mag_y, mag_y_min);
        mag_y_max = max(mag_y, mag_y_max);
        mag_z_min = min(mag_z, mag_z_min);
        mag_z_max = max(mag_z, mag_z_max);
      }
    }
    delay(4);  // Time interval between magnetometer readings
  }

  // ----- Calculate hard-iron offsets
  Mag_x_offset = (mag_x_max + mag_x_min) / 2;  // Get average magnetic bias in counts
  Mag_y_offset = (mag_y_max + mag_y_min) / 2;
  Mag_z_offset = (mag_z_max + mag_z_min) / 2;

  // ----- Calculate soft-iron scale factors
  chord_x = ((float)(mag_x_max - mag_x_min)) / 2;  // Get average max chord length in counts
  chord_y = ((float)(mag_y_max - mag_y_min)) / 2;
  chord_z = ((float)(mag_z_max - mag_z_min)) / 2;

  chord_average = (chord_x + chord_y + chord_z) / 3;  // Calculate average chord length

  Mag_x_scale = chord_average / chord_x;  // Calculate X scale factor
  Mag_y_scale = chord_average / chord_y;  // Calculate Y scale factor
  Mag_z_scale = chord_average / chord_z;  // Calculate Z scale factor

  // ----- Record magnetometer offsets
  Serial.println();

  // ----- Display hard-iron offsets
  Serial.println("- Hard-iron -");
  Serial.print("Mag_x_offset ");
  Serial.println(Mag_x_offset);
  Serial.print("Mag_y_offset ");
  Serial.println(Mag_y_offset);
  Serial.print("Mag_z_offset ");
  Serial.println(Mag_z_offset);
  Serial.println("");

  // ----- Display soft-iron scale factors
  Serial.println("- Soft-iron -");
  Serial.print("Mag_x_scale ");
  Serial.println(Mag_x_scale);
  Serial.print("Mag_y_scale ");
  Serial.println(Mag_y_scale);
  Serial.print("Mag_z_scale ");
  Serial.println(Mag_z_scale);
  Serial.println("");

  // ----- Display fuse ROM values
  Serial.println("- ASA- ");
  Serial.print("ASAX ");
  Serial.println(ASAX);
  Serial.print("ASAY ");
  Serial.println(ASAY);
  Serial.print("ASAZ ");
  Serial.println(ASAZ);
}


// -----------------------------------
//  Configure the gyro & accelerometer
// -----------------------------------
void config_gyro() {
  // ----- Activate the MPU-6050
  Wire.beginTransmission(0x68);  //Open session with the MPU-6050
  Wire.write(0x6B);              //Point to power management register
  Wire.write(0x00);              //Use internal 20MHz clock
  Wire.endTransmission();        //End the transmission

  // ----- Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);  //Open session with the MPU-6050
  Wire.write(0x1C);              //Point to accelerometer configuration reg
  Wire.write(0x10);              //Select +/-8g full-scale
  Wire.endTransmission();        //End the transmission

  // ----- Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);  //Open session with the MPU-6050
  Wire.write(0x1B);              //Point to gyroscope configuration
  Wire.write(0x08);              //Select 500dps full-scale
  Wire.endTransmission();        //End the transmission
}


// --------------------
//  Read MPU 6050 data
// --------------------
void read_mpu_6050_data() {
  /*
    Subroutine for reading the raw gyro and accelerometer data
  */

  // ----- Locals
  int temperature;  // Needed when reading the MPU-6050 data ... not used

  // ----- Point to data
  Wire.beginTransmission(0x68);  // Start communicating with the MPU-6050
  Wire.write(0x3B);              // Point to start of data
  Wire.endTransmission();        // End the transmission

  // ----- Read the data
  Wire.requestFrom(0x68, 14);  // Request 14 bytes from the MPU-6050
  while (Wire.available() < 14)
    ;                                            // Wait until all the bytes are received
  Accel_x = Wire.read() << 8 | Wire.read();      // Combine MSB,LSB Accel_x variable
  Accel_y = Wire.read() << 8 | Wire.read();      // Combine MSB,LSB Accel_y variable
  Accel_z = Wire.read() << 8 | Wire.read();      // Combine MSB,LSB Accel_z variable
  temperature = Wire.read() << 8 | Wire.read();  // Combine MSB,LSB temperature variable
  Gyro_x = Wire.read() << 8 | Wire.read();       // Combine MSB,LSB Gyro_x variable
  Gyro_y = Wire.read() << 8 | Wire.read();       // Combine MSB,LSB Gyro_x variable
  Gyro_z = Wire.read() << 8 | Wire.read();       // Combine MSB,LSB Gyro_x variable
}
