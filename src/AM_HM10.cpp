/*

   AMController libraries, example sketches (“The Software”) and the related documentation (“The Documentation”) are supplied to you
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

   Author: Fabrizio Boco - fabboco@gmail.com

   All rights reserved

*/
#include "AM_HM10.h"

#if defined(ARDUINO_AVR_UNO)
#include <SoftwareSerial.h>
SoftwareSerial deviceSerial(7, 8);
#endif

#if defined(ARDUINO_AVR_MEGA2560)
#define deviceSerial Serial3
#endif

#define ALARMCHECKDELAY 		  64000
#define ALARMCHECKDELAY_CONNECTED 50

#define LEAP_YEAR(Y)     ( ((1970+Y)>0) && !((1970+Y)%4) && ( ((1970+Y)%100) || !((1970+Y)%400) ) )

static  const uint8_t monthDays[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31}; // API starts months from 1, this array starts from 0

#ifdef ALARMS_SUPPORT
AMController::AMController(
  void (*doWork)(void),
  void (*doSync)(void),
  void (*processIncomingMessages)(char *variable, char *value),
  void (*processOutgoingMessages)(void),
  void (*processAlarms)(char *alarm),
  void (*deviceConnected)(void),
  void (*deviceDisconnected)(void))
{
  _var = true;
  _idx = 0;
  _doWork = doWork;
  _doSync = doSync;
  _processIncomingMessages = processIncomingMessages;
  _processOutgoingMessages = processOutgoingMessages;
  _processAlarms = processAlarms;
  _deviceConnected = deviceConnected;
  _deviceDisconnected = deviceDisconnected;

  _variable[0] = '\0';
  _value[0]    = '\0';

  _startTime = 0;
  _lastAlarmCheck = 0;

  this->inizializeAlarms();

  deviceSerial.begin(HM10_COM_SPEED);
}
#endif


AMController::AMController(
  void (*doWork)(void),
  void (*doSync)(void),
  void (*processIncomingMessages)(char *variable, char *value),
  void (*processOutgoingMessages)(void),
  void (*deviceConnected)(void),
  void (*deviceDisconnected)(void))
{
  _var = true;
  _idx = 0;
  _doWork = doWork;
  _doSync = doSync;
  _processIncomingMessages = processIncomingMessages;
  _processOutgoingMessages = processOutgoingMessages;
  _deviceConnected = deviceConnected;
  _deviceDisconnected = deviceDisconnected;

  _variable[0] = '\0';
  _value[0]    = '\0';

  deviceSerial.begin(HM10_COM_SPEED);
}

void AMController::loop() {
  this->loop(500);
}

void AMController::loop(unsigned long _delay) {

#ifdef ALARMS_SUPPORT

  if (_processAlarms != NULL) {
	  unsigned long now = this->now();
	  
    if ( (now - _lastAlarmCheck) > ALARM_CHECK_INTERVAL) {
      _lastAlarmCheck = now;
      this->checkAndFireAlarms();
    }
  }

#endif

  _doWork();

  // Read incoming messages if any
  this->readVariable();

#ifdef DEBUG
  if (strlen(_variable) > 0) {
    Serial.print("Received "); Serial.print(_variable); Serial.print(" "); Serial.println(_value);
  }
#endif

  if (strcmp(_variable, "Sync") == 0 && strlen(_value) > 0) {
    // Process sync messages for the variable _value
    _doSync();
    return;
  }
  else {

#ifdef ALARMS_SUPPORT
    // Manages Alarm creation and update requests

    char id[8];

    if (strcmp(_variable, "$AlarmId$") == 0 && strlen(_value) > 0) {
      strcpy(id, _value);
    } else if (strcmp(_variable, "$AlarmT$") == 0 && strlen(_value) > 0) {

      _tmpTime = atol(_value);
    }
    else if (strcmp(_variable, "$AlarmR$") == 0 && strlen(_value) > 0) {
      if (_tmpTime == 0)
        this->removeAlarm(id);
      else
        this->createUpdateAlarm(id, _tmpTime, atoi(_value));

#ifdef DEBUG
      this->dumpAlarms();
#endif
    }
    else
#endif
#ifdef SD_SUPPORT
      if (strlen(_variable) > 0 && strcmp(_variable, "SD") == 0) {
#ifdef DEBUG
        Serial.println("List of Files");
#endif
        _root = SD.open("/", FILE_READ);
        if (!_root) {
          Serial.println("Cannot open root dir");
        }
        
        _root.rewindDirectory();
        _entry =  _root.openNextFile();

        while (_entry) {
          if (!_entry.isDirectory()) {
#ifdef DEBUG
            Serial.println(_entry.name());
#endif
            this->writeTxtMessage("SD", _entry.name());
          }
          _entry.close();
          _entry = _root.openNextFile();
        }

        _root.close();
        deviceSerial.print("SD=$EFL$#");

#ifdef DEBUG
        Serial.println("File list sent");
#endif
      }
      else if (strlen(_variable) > 0 && strcmp(_variable, "$SDDL$") == 0) {
#ifdef DEBUG
        Serial.print("File: "); Serial.println(_value);
#endif
        _entry = SD.open(_value, FILE_READ);
        if (_entry) {
#ifdef DEBUG
          Serial.println("File Opened");
#endif
          unsigned long n = 0;
          uint8_t buffer[64];
          deviceSerial.print("SD=$C$#");
          delay(3000);
          while (_entry.available()) {
            n = _entry.read(buffer, sizeof(buffer));
            deviceSerial.write(buffer, n * sizeof(uint8_t));
          }
          _entry.close();
          deviceSerial.print("SD=$E$#");
#ifdef DEBUG
          Serial.println("Fine Sent");
#endif
        }
        deviceSerial.flush();
      }
#endif
    if (strlen(_variable) > 0 && strlen(_value) > 0) {
#ifdef ALARMS_SUPPORT
      if (strcmp(_variable, "$Time$") == 0) {
        _startTime = atol(_value) - millis() / 1000;
#ifdef DEBUG
        Serial.print("Time Synchronized ");
        Serial.print(_startTime);
        Serial.print(" ");
        this->printTime(_startTime);
#endif
				return;
      }
#endif
      // Process incoming messages
      _processIncomingMessages(_variable, _value);
    }
  }

#ifdef ALARMS_SUPPORT
  // Check and Fire Alarms
  if (_processAlarms != NULL) {
	  unsigned long now = this->now();
	  
    if ( (now - _lastAlarmCheck) > ALARM_CHECK_INTERVAL) {
      _lastAlarmCheck = now;
      this->checkAndFireAlarms();
    }
  }
#endif

#ifdef SDLOGGEDATAGRAPH_SUPPORT
  if (strlen(_variable) > 0 && strcmp(_variable, "$SDLogData$") == 0) {

#ifdef DEBUG
    Serial.print("Logged data request for: ");
    Serial.println(_value);
#endif
    this->sdSendLogData(_value);

#ifdef DEBUG
    Serial.println("Logged data sent");
#endif
  }
#endif
  // Write outgoing messages
  _processOutgoingMessages();

  delay(_delay);
}

void AMController::readVariable(void) {

  _variable[0] = '\0';
  _value[0] = '\0';
  char buffer[VARIABLELEN + VALUELEN + 2];
  short  idx = 0;

  memset(buffer, 0, VARIABLELEN + VALUELEN + 2);
  _var = true;

  while (deviceSerial.available() > 0) {

    char c = deviceSerial.read();

    //Serial.print(c);

    buffer[idx++] = c;
    
    //Serial.print(">"); Serial.print(buffer); Serial.println("<");

    if (memcmp(buffer, "OK+CONN", 7) == 0) {
      _deviceConnected();
      memset(buffer, 0, 7);
      idx = 0;
    }

    if (memcmp(buffer, "OK+LOST", 7) == 0) {
      _deviceDisconnected();
      memset(buffer, 0, 7);
      idx = 0;
    }
    
    //Serial.print(" c -> "); Serial.print(c); Serial.print(" id -> "); Serial.println(idx); 
    
    if (_var == true) {
    	if ((char)c == '=') {
      	_variable[_idx] = '\0';
      	_var = false;
      	_idx = 0;
      	
      	//Serial.print("final vr -> "); Serial.println(_variable);
    	}
    	else if (c != '\0') {
    		_variable[_idx++] = c;
    		_variable[_idx] = '\0';
    		//Serial.print("vr -> "); Serial.println(_variable);
    	}
    }
		else {
			if ((char)c == '#') {
				 _value[_idx] = '\0';
        _var = true;
        _idx = 0;
        //Serial.print(_variable); Serial.print("->");Serial.println(_value);
        return;
			}
			else {
				_value[_idx++] = c;	
			}
		}
  }
}


void AMController::writeMessage(const char *variable, float value)
{
  char buffer[VARIABLELEN + VALUELEN + 3];
  char vbuffer[VALUELEN];

  if (!deviceSerial)
    return;

  dtostrf(value, 0, 3, vbuffer);
  snprintf(buffer, VARIABLELEN + VALUELEN + 3, "%s=%s#", variable, vbuffer);

  deviceSerial.write((const uint8_t *)buffer, strlen(buffer)*sizeof(char));
}

void AMController::writeTripleMessage(const char *variable, float vX, float vY, float vZ) {

  char buffer[VARIABLELEN + VALUELEN + 3];
  char vbufferAx[VALUELEN];
  char vbufferAy[VALUELEN];
  char vbufferAz[VALUELEN];

  dtostrf(vX, 0, 2, vbufferAx);
  dtostrf(vY, 0, 2, vbufferAy);
  dtostrf(vZ, 0, 2, vbufferAz);
  snprintf(buffer, VARIABLELEN + VALUELEN + 3, "%s=%s:%s:%s#", variable, vbufferAx, vbufferAy, vbufferAz);

  deviceSerial.write((const uint8_t *)buffer, strlen(buffer)*sizeof(char));
}


void AMController::writeTxtMessage(const char *variable, const char *value)
{
  if (!deviceSerial)
    return;

  /*
      char buffer[128];

      snprintf(buffer,128, "%s=%s#", variable, value);

      deviceSerial.write((const uint8_t *)buffer, strlen(buffer)*sizeof(char));
  */

  int i = 0;
  while (variable[i] != '\0')
  {
    deviceSerial.write(variable[i++]);
  }

  deviceSerial.write('=');

  i = 0;
  while (value[i] != '\0')
  {
    deviceSerial.write(value[i++]);
  }

  deviceSerial.write('#');
}

void AMController::log(const char *msg)
{
  this->writeTxtMessage("$D$", msg);
}

void AMController::log(int msg)
{
  char buffer[11];
  itoa(msg, buffer, 10);

  this->writeTxtMessage("$D$", buffer);
}


void AMController::logLn(const char *msg)
{
  this->writeTxtMessage("$DLN$", msg);
}

void AMController::logLn(int msg)
{
  char buffer[11];
  itoa(msg, buffer, 10);

  this->writeTxtMessage("$DLN$", buffer);
}

void AMController::logLn(long msg)
{
  char buffer[11];
  ltoa(msg, buffer, 10);

  this->writeTxtMessage("$DLN$", buffer);
}

void AMController::logLn(unsigned long msg) {

  char buffer[11];
  ultoa(msg, buffer, 10);

  this->writeTxtMessage("$DLN$", buffer);
}

void AMController::temporaryDigitalWrite(uint8_t pin, uint8_t value, unsigned long ms) {

  int previousValue = digitalRead(pin);

  digitalWrite(pin, value);
  delay(ms);
  digitalWrite(pin, previousValue);
}


// Time Management

#ifdef ALARMS_SUPPORT

unsigned long AMController::now() {
  if (_startTime == 0) {
    return 0;
  }
  unsigned long now = _startTime + millis() / 1000;
  return now;
}

void AMController::breakTime(unsigned long time, int *seconds, int *minutes, int *hours, int *Wday, long *Year, int *Month, int *Day) {
  // break the given time_t into time components
  // this is a more compact version of the C library localtime function
  // note that year is offset from 1970 !!!

  unsigned long year;
  uint8_t month, monthLength;
  unsigned long days;

  *seconds = time % 60;
  time /= 60; // now it is minutes
  *minutes = time % 60;
  time /= 60; // now it is hours
  *hours = time % 24;
  time /= 24; // now it is days
  *Wday = ((time + 4) % 7) + 1;  // Sunday is day 1

  year = 1970;
  days = 0;
  while ((unsigned)(days += (LEAP_YEAR(year) ? 366 : 365)) <= time) {
    year++;
  }
  *Year = year; // year is offset from 1970

  days -= LEAP_YEAR(year) ? 366 : 365;
  time  -= days; // now it is days in this year, starting at 0

  days = 0;
  month = 0;
  monthLength = 0;
  for (month = 0; month < 12; month++) {
    if (month == 1) { // february
      if (LEAP_YEAR(year)) {
        monthLength = 29;
      }
      else {
        monthLength = 28;
      }
    }
    else {
      monthLength = monthDays[month];
    }

    if (time >= monthLength) {
      time -= monthLength;
    }
    else {
      break;
    }
  }
  *Month = month + 1;  // jan is month 1
  *Day = time + 1;     // day of month
}


#ifdef DEBUG
void AMController::printTime(unsigned long time) {

  int seconds;
  int minutes;
  int hours;
  int Wday;
  long Year;
  int Month;
  int Day;

  this->breakTime(time, &seconds, &minutes, &hours, &Wday, &Year, &Month, &Day);

  Serial.print(Day);
  Serial.print("/");
  Serial.print(Month);
  Serial.print("/");
  Serial.print(Year);
  Serial.print(" ");
  Serial.print(hours);
  Serial.print(":");
  Serial.print(minutes);
  Serial.print(":");
  Serial.println(seconds);
}
#endif

void AMController::createUpdateAlarm(char *id, unsigned long time, bool repeat) {

  char lid[12];

  lid[0] = 'A';
  strcpy(&lid[1], id);

  // Update

  for (int i = 0; i < 5; i++) {

    alarm a;

    eeprom_read_block((void*)&a, (void*)(i * sizeof(a)), sizeof(a));

    if (strcmp(a.id, lid) == 0) {
      a.time = time;
      a.repeat = repeat;

      eeprom_write_block((const void*)&a, (void*)(i * sizeof(a)), sizeof(a));

      return;
    }
  }

  // Create

  for (int i = 0; i < 5; i++) {

    alarm a;

    eeprom_read_block((void*)&a, (void*)(i * sizeof(a)), sizeof(a));

    if (a.id[1] == '\0') {

      strcpy(a.id, lid);
      a.time = time;
      a.repeat = repeat;

      eeprom_write_block((const void*)&a, (void*)(i * sizeof(a)), sizeof(a));

      return;
    }
  }
}

void AMController::removeAlarm(char *id) {

  char lid[12];

  lid[0] = 'A';
  strcpy(&lid[1], id);

  for (int i = 0; i < 5; i++) {

    alarm a;

    eeprom_read_block((void*)&a, (void*)(i * sizeof(a)), sizeof(a));

    if (strcmp(a.id, lid) == 0) {

      a.id[1] = '\0';
      a.time = 0;
      a.repeat = 0;

      eeprom_write_block((const void*)&a, (void*)(i * sizeof(a)), sizeof(a));
    }
  }
}

void AMController::inizializeAlarms() {

  for (int i = 0; i < 5; i++) {

    alarm a;

    eeprom_read_block((void*)&a, (void*)(i * sizeof(a)), sizeof(a));

    if (a.id[0] != 'A') {

      a.id[0] = 'A';
      a.id[1] = '\0';
      a.time = 0;
      a.repeat = 0;

      eeprom_write_block((const void*)&a, (void*)(i * sizeof(a)), sizeof(a));
    }
  }
}

#ifdef DEBUG
void AMController::dumpAlarms() {

  Serial.println("\t----Dump Alarms -----");

  for (int i = 0; i < 5; i++) {

    alarm al;

    eeprom_read_block((void*)&al, (void*)(i * sizeof(al)), sizeof(al));

    Serial.print("\t");
    Serial.print(al.id);
    Serial.print(" ");
    Serial.print(al.time);
    Serial.print(" ");
    Serial.println(al.repeat);
  }
}
#endif

void AMController::checkAndFireAlarms(void) {

    unsigned long now = this->now();

#ifdef DEBUG
    Serial.print("checkAndFireAlarms ");
    this->printTime(now);
    this->dumpAlarms();
#endif

    for (int i = 0; i < 5; i++) {

      alarm a;

      eeprom_read_block((void*)&a, (void*)(i * sizeof(a)), sizeof(a));

      if (a.id[1] != '\0' && a.time < now) {

#ifdef DEBUG
        Serial.println(a.id);
#endif
        // First character of id is A and has to be removed
        _processAlarms(&a.id[1]);

        if (a.repeat) {

          a.time += 86400; // Scheduled again tomorrow

#ifdef DEBUG
          Serial.print("Alarm rescheduled at ");
          this->printTime(a.time);
#endif
        }
        else {
          //     Alarm removed

          a.id[1] = '\0';
          a.time = 0;
          a.repeat = 0;
        }

        eeprom_write_block((const void*)&a, (void*)(i * sizeof(a)), sizeof(a));
#ifdef DEBUG
        this->dumpAlarms();
#endif
      }
    }
}
#endif

#ifdef SDLOGGEDATAGRAPH_SUPPORT

void AMController::sdLogLabels(const char *variable, const char *label1) {

  this->sdLogLabels(variable, label1, NULL, NULL, NULL, NULL);
}

void AMController::sdLogLabels(const char *variable, const char *label1, const char *label2) {

  this->sdLogLabels(variable, label1, label2, NULL, NULL, NULL);
}

void AMController::sdLogLabels(const char *variable, const char *label1, const char *label2, const char *label3) {

  this->sdLogLabels(variable, label1, label2, label3, NULL, NULL);
}

void AMController::sdLogLabels(const char *variable, const char *label1, const char *label2, const char *label3, const char *label4) {

  this->sdLogLabels(variable, label1, label2, label3, label4, NULL);
}

void AMController::sdLogLabels(const char *variable, const char *label1, const char *label2, const char *label3, const char *label4, const char *label5) {

  File dataFile = SD.open(variable, FILE_WRITE);

  if (dataFile)
  {
    dataFile.print("-");
    dataFile.print(";");
    dataFile.print(label1);
    dataFile.print(";");

    if (label2 != NULL)
      dataFile.print(label2);
    else
      dataFile.print("-");
    dataFile.print(";");

    if (label3 != NULL)
      dataFile.print(label3);
    else
      dataFile.print("-");
    dataFile.print(";");

    if (label4 != NULL)
      dataFile.print(label4);
    else
      dataFile.print("-");
    dataFile.print(";");

    if (label5 != NULL)
      dataFile.println(label5);
    else
      dataFile.println("-");

    dataFile.println("\n");

    dataFile.flush();
    dataFile.close();
  }
}


void AMController::sdLog(const char *variable, unsigned long time, float v1) {

  File dataFile = SD.open(variable, FILE_WRITE);

  if (dataFile && time > 0)
  {
    dataFile.print(time);
    dataFile.print(";");
    dataFile.print(v1);
    dataFile.print(";");

    dataFile.print(v1);

    dataFile.print(";-;-;-\n");

    dataFile.flush();
    dataFile.close();
  }
}

void AMController::sdLog(const char *variable, unsigned long time, float v1, float v2) {

  File dataFile = SD.open(variable, FILE_WRITE);

  if (dataFile && time > 0)
  {
    dataFile.print(time);
    dataFile.print(";");
    dataFile.print(v1);
    dataFile.print(";");

    dataFile.print(v2);

    dataFile.print(";-;-;-\n");

    dataFile.flush();
    dataFile.close();
  }
}

void AMController::sdLog(const char *variable, unsigned long time, float v1, float v2, float v3) {

  File dataFile = SD.open(variable, FILE_WRITE);

  if (dataFile && time > 0)
  {
    dataFile.print(time);
    dataFile.print(";");
    dataFile.print(v1);
    dataFile.print(";");

    dataFile.print(v2);
    dataFile.print(";");

    dataFile.print(v3);

    dataFile.print(";-;-\n");

    dataFile.flush();
    dataFile.close();
  }
}

void AMController::sdLog(const char *variable, unsigned long time, float v1, float v2, float v3, float v4) {

  File dataFile = SD.open(variable, FILE_WRITE);

  if (dataFile && time > 0)
  {
    dataFile.print(time);
    dataFile.print(";");
    dataFile.print(v1);
    dataFile.print(";");

    dataFile.print(v2);
    dataFile.print(";");

    dataFile.print(v3);
    dataFile.print(";");

    dataFile.print(v4);

    dataFile.println(";-\n");

    dataFile.flush();
    dataFile.close();
  }
}

void AMController::sdLog(const char *variable, unsigned long time, float v1, float v2, float v3, float v4, float v5) {

  File dataFile = SD.open(variable, FILE_WRITE);

  if (dataFile && time > 0)
  {
    dataFile.print(time);
    dataFile.print(";");
    dataFile.print(v1);
    dataFile.print(";");

    dataFile.print(v2);
    dataFile.print(";");

    dataFile.print(v3);
    dataFile.print(";");

    dataFile.print(v4);
    dataFile.print(";");

    dataFile.println(v5);

    dataFile.println("\n");

    dataFile.flush();
    dataFile.close();
  }
}

void AMController::sdSendLogData(const char *variable) {

  //	cli();

  delay(250);

  _entry = SD.open(variable, FILE_WRITE);

  if (_entry) {

    char c;
    char buffer[128];
    char buffer1[128];
    int i = 0;

    _entry.seek(0);

    delay(250);

    while ( (c = _entry.read()) != -1 ) {

      if (c == '\n') {

        buffer[i++] = '\0';

        //this->writeTxtMessage(variable,buffer);

        snprintf(buffer1, 128, "%s=%s#", variable, buffer);
        deviceSerial.write((const uint8_t *)buffer1, strlen(buffer1)*sizeof(char));

        //delay(250);

        i = 0;
      }
      else
        buffer[i++] = c;
    }

    _entry.close();
  }

  this->writeTxtMessage(variable, "");

  //  	sei();
}


void AMController::sdPurgeLogData(const char *variable) {

  cli();

  SD.remove(variable);

  sei();
}

#endif