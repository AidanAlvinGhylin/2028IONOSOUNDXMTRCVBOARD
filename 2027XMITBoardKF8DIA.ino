//  2027XMITBoardKF8DIA.ino

//  This code is intended to run on the Teensy 4.0 on the COSTAS XMT/RCV Board Ver 1.20, built by John Gibbons N8OBJ.

//  Revision History
//  Rev #     Date      Author      Description
//  1.0       10-8-26   KF8DIA      Initial code working with hard fault detection.
//  1.10      10-8-26   KF8DIA      Implemented watchdog timer; still working out heap allocation faults.
//  1.20      10-12-26  KF8DIA      First attempt at solving heap allocation faults; used char array instead of std::string to compile bytes for NMEA sentences.
//  1.30      10-12-26  KF8DIA      Repaired WDT1 software reset routine to reset the system in the event of a hard fault.
//  1.40      10-17-26  KF8DIA      Fixed issue with resetting XMIT bit when GPS signal is lost.
//  1.50      10-17-26  KF8DIA      Modified PTT control to where we don't reset the XMIT when we're resetting the phase. I was doing this because I was previously hearing
//                                  pulses caused by the phase resets, but I think this was before I started resetting the phase to 270 instead of 0.
//  1.60      10-17-26  KF8DIA      The code was not configuring the NMEA data from the Adafruit. Implemented this.
//  1.70      10-18-26  KF8DIA      Re-implemented automatic SRS on system hard fault. It still hard faults periodically, although not as often as it did previously, so I 
//                                  still need to figure out what's causing this.

#include <_Teensy.h>
#include <Arduino.h>
#include <iostream>
#include <string>
#include <cmath>
#include <vector>

//  Hardware timer definitions
IntervalTimer GPS_RX_timer;
IntervalTimer GPS_TX_timer;
IntervalTimer UTC_tick_timer;
IntervalTimer timed_silence_timer;
IntervalTimer BARKER_code_timer;
IntervalTimer COSTAS_array_timer;
IntervalTimer constant_audio_timer;
IntervalTimer station_ID_timer;

//  Interrupt bit masks
unsigned long PIN_7_MASK = 1 << 17;     //  Pin 7 connects to bit 17 of GPIO7
unsigned long PIN_6_MASK = 1 << 10;     //  Pin 6 connects to bit 10 of GPIO7
unsigned long PIN_2_MASK = 1 << 4;      //  Pin 2 connects to bit 4 of GPIO9

//  Teensy pin definitions
const int GPS_RX_pin = 7;
const int GPS_TX_pin = 8;
const int PPS_interrupt_pin = 2;
const int GPS_fix_interrupt_pin = 6;
const int LED_indicator_red = 3;
const int LED_indicator_green = 4;
const int LED_indicator_blue = 5;
const int FreqUp_diagnostic_pin = 19;
const int FreqDown_diagnostic_pin = 18;
const int XMIT_enable_jumper = 17;
const int XMIT = 21;
const int AD9850_SGCLK = 12;               //  Used as CLK line when writing frequency and phase tuning words to the AD9850.
const int AD9850_SGDAT = 11;               //  Used as DATA line when writing frequency and phase tuning words to the AD9850.
const int AD9850_SGLD = 13;                //  When toggled, previously written frequency and phase tuning words are loaded to the registers of the AD9850.
const int AD9850_SGRST = 20;               //  When toggled, the frequency and phase registers of the AD9850 are reset. This toggled during every silent period.

//  Counter definitions
volatile int GPS_fix_PPS_counter = 0;
volatile int PPS_counter = 0;

//  Flag definitions
volatile bool GPS_lock = false;                                                //  Flag to indicate whether we have a GPS fix.
volatile bool transmission_active = false;                                    //  Flag to indicate whether transmission is active.

//  NMEA byte receiving structure. Individual bits are clocked in to a buffer array of length 8. Once we have a full byte, the byte is loaded into a dynamic array and printed.

std::array<int, 8> RX_buffer = {};
volatile int RX_buffer_pointer = 0;

volatile char current_sentence[100];                                          //  Fixed 100-byte array, used to construct the NMEA sentence coming in from the GPS receiver.
volatile char previous_sentence[100];                                         //  Fixed 100-byte array, used to store the previous NMEA sentence after it is fully constructed and verified. Used for data extraction.
volatile int current_sentence_index = 0;                                      //  Tracks the position within current_sentence we're appending the received byte to through the GPS RX process.
volatile uint8_t received_byte;

//  NMEA byte transmitting structure.

std::vector<uint8_t> TX_buffer_bytes;                                         //  Use this buffer to store all ASCII characters that need to be transmitted to the GPS receiver.
std::array<int, 8> TX_buffer_bits = {};                                       //  Use this buffer to store all the bits of the byte currently being transmitted to the GPS receiver.
volatile int TX_buffer_bits_pointer = 0;                                      
volatile bool GPS_TX_active = false;                                          //  Flag to indicate whether we're transmitting to the GPS receiver.
volatile bool TX_buffer_bits_filled = false;                                  //  Flag to indicate whether TX_buffer_bits is full
volatile bool GPS_TX_start_bit = false;                                       //  Flag to indicate whether the next bit to be transmitted to the GPS receiver is a start bit.
volatile bool GPS_TX_stop_bit = false;                                        //  Flag to indicate whether the next bit to be transmitted to the GPS receiver is a stop bit.
volatile bool process_GPS_RX_byte = false;                                    //  Flag to indicate whether a full byte has been received and needs to be processed.
volatile bool load_GPS_TX_byte = false;

//  Used to regulate information extraction from NMEA data from the GPS receiver.
volatile bool NMEA_sentence_complete = false;

//  Used to store information from the NMEA sentences from the GPS receiver.
//  Second values
volatile int current_second = -1;                                             
volatile int next_second = -1;
volatile int previous_second = -1;
std::string extracted_second;
//  Minute values
volatile int current_minute = -1;
volatile int next_minute = -1;
volatile int previous_minute = -1;
std::string extracted_minute;
//  Hour values
volatile int current_hour = -1;
volatile int next_hour = -1;
volatile int previous_hour = -1;
std::string extracted_hour;

//  Boolean that indicates whether the system thinks it is tracking real time; used for fault detection
volatile bool time_lock = false;

//  Carrier frequency and phase tuning word definitions
unsigned int UTC_minute_tick_FTW = 6442451;                                             //  1.5 kHz
unsigned int UTC_second_tick_FTW = 4294967;                                             //  1 kHz
unsigned int BARKER_code_freq = 3435973;                                                //  800 Hz
std::array<unsigned int, 13> BARKER_code_phases = {192, 192, 192, 192, 192, 64, 64, 192, 192, 64, 192, 64, 192}; //  BPSK throughout BARKER code (either 270 or 90 degrees)
std::array<unsigned int, 21> COSTAS_array_freqs = {2834678, 1288490, 5411658, 2319282, 10565619, 4380866, 9019431, 8504035, 5927054, 4896262, 11596411, 9534827, 11081015, 6957847, 10050223, 1803886, 7988639, 3350074, 3865470, 6442450, 7473243}; //  FSK throughout COSTAS array (21 unique frequencies, lowest 300 Hz, spaced in 120 Hz increments)
unsigned int COSTAS_array_phase = 192;                                                  //  270 degrees
std::array<unsigned int, 2> constant_audio_freqs = {2147484, 2576980};                  //  Constant audio tone carriers, 500 Hz and 600 Hz.
unsigned int constant_audio_phase = 192;                                                //  270 degrees

//  Counters definitions used for signal elements
volatile int BARKER_code_index = 0;
volatile int COSTAS_array_index = 0;
volatile int constant_audio_index = 0;
volatile int station_ID_index = 0;

//  XMIT flags used for signal elements
volatile bool BARKER_code_XMIT_flag = false;
volatile bool COSTAS_array_XMIT_flag = false;
volatile bool constant_audio_XMIT_flag = false;
volatile bool guarding_silence = false;

//  Stop flags used for signal elements
volatile bool BARKER_code_stop_flag = false;
volatile bool COSTAS_array_stop_flag = false;
volatile bool station_ID_stop_flag = false;

//  Parameters used for station identification
unsigned int station_ID_carrier_freq = 3006477;               //  700 Hz
std::vector<unsigned int> station_ID_freqs = {}; 
unsigned int station_ID_phase = 192;                          //  270 degrees
std::string morse_ID_raw = ".-- ---.. . -.. ..- -..-. . .-";  //  Raw morse ID string. Used to construct the freqs array.
volatile bool station_ID_active = false;

//  System fault flag, raised if any errors occur.
volatile bool system_fault_flag = false;
volatile bool loop_started_running = false;
volatile bool loop_stopped_running = false;

//  Timestamp for when the system faulted, in case the watchdog timer still isn't working
volatile int fault_hour = -1;
volatile int fault_minute = -1;
volatile int fault_second = -1;

void setup() {
  // put your setup code here, to run once:
  attachInterrupt(GPS_RX_pin, GPS_RX_start, FALLING);
  attachInterrupt(PPS_interrupt_pin, PPS_ISR, RISING);
  attachInterrupt(GPS_fix_interrupt_pin, GPS_fix_ISR, RISING);

  //  Configure Teensy pins
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(7, INPUT);
  pinMode(8, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(17, INPUT);
  pinMode(18, OUTPUT);
  pinMode(19, OUTPUT);
  pinMode(20, OUTPUT);

  //  Reserve memory space for the extracted UTC values, to avoid heap allocation problems
  extracted_minute.reserve(2);
  extracted_second.reserve(2);
  extracted_hour.reserve(2);

  Serial.begin(115200);

  Serial.println("------------------------------");
  Serial.println("2027XMITBoardKF8DIA.ino Rev. 1.70");
  Serial.println("------------------------------");

  load_GPS_TX_buffer("$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n");                              //  PMTK314 (sets NMEA output) to only Global Positioning Fix Data (GGA) sentences
  load_GPS_TX_buffer("$PMTK220,100*2F\r\n"); 

  digitalWrite(AD9850_SGRST, HIGH);
  digitalWrite(AD9850_SGRST, LOW);
  digitalWrite(XMIT, LOW);
  digitalWrite(GPS_TX_pin, HIGH);

  construct_station_ID_freqs();

  // delay(500);

  //  Initiate the watchdog timer with a timeout of 10 seconds to automatically reset the system in the event of a fault.
  wdog1_init(19);
}

void PPS_ISR() {
  // Serial.print("Entering PPS_ISR() with next_minute = ");
  // Serial.println(next_minute);
  // Serial.print("Entered PPS_ISR(). transmission_active = ");
  // Serial.println(transmission_active);
  //  This value needs to be incremented every time, whether we have a GPS signal or not.
  GPS_fix_PPS_counter++;

  previous_second = current_second;
  previous_minute = current_minute;
  previous_hour = current_hour;

  current_second = next_second;
  current_minute = next_minute;
  current_hour = next_hour;

  // Serial.print("Updated UTC time to ");
  // Serial.print(current_hour);
  // Serial.print(":");
  // Serial.print(current_minute);
  // Serial.print(":");
  // Serial.println(current_second);

  // Serial.print("Previous time: ");
  // Serial.print(previous_minute);
  // Serial.print(":");
  // Serial.println(previous_second);
  // Serial.print("Current time: ");
  // Serial.print(current_minute);
  // Serial.print(":");
  // Serial.println(current_second);
  // Serial.print("Next time: ");
  // Serial.print(next_minute);
  // Serial.print(":");
  // Serial.println(next_second);
  // Serial.print("system_fault_flag = ");
  // Serial.println(system_fault_flag);
  // Serial.println("--------------------------------");

  if (current_second != (previous_second + 1) % 60 && (previous_minute != -1 && previous_second != -1 && previous_hour != -1)) {
    // Serial.println("time synchronization lost");
    raise_fault_flag();
  }

  //  Temporarily commenting this out as the Teensy is having problems that I am figuring out
  // current_second = next_second;
  if (!station_ID_active) {
    if (!digitalRead(XMIT_enable_jumper) && GPS_lock && !system_fault_flag) {
      digitalWrite(XMIT, HIGH);
      toggle_AD9850_FQ_UD();
      if (current_second == 0 && (current_minute % 5 == 0)) {
        bool status = station_ID_timer.begin(station_ID_ISR, 49000);
        if (!status) {
          // Serial.println("station_ID_timer failed to start");
          raise_fault_flag();
        }
        load_AD9850_registers(station_ID_freqs[1], 192);
        station_ID_index++;
        station_ID_active = true;
      }
      else if (current_second == 0) {
        bool status = UTC_tick_timer.begin(UTC_tick_ISR, 800000);
        if (!status) {
          // Serial.println("UTC_tick_timer failed to start");
          raise_fault_flag();
        }
      }
      else {
        bool status = UTC_tick_timer.begin(UTC_tick_ISR, 5000);
        if (!status) {
          // Serial.println("UTC_tick_timer failed to start");
          raise_fault_flag();
        }
      }
      load_AD9850_registers(0, 192);
      PPS_counter++;
      if (!transmission_active) {
        transmission_active = true;
      }
      GPS_lock = true;
    }
    else {
      digitalWrite(XMIT, LOW);
      PPS_counter = 0;
      transmission_active = false;                                                                                 //  Disable transmission
      GPS_lock = false;
      toggle_AD9850_FQ_UD();
      load_AD9850_registers(0, 192);
      toggle_AD9850_FQ_UD();
    }
  }

  //  DIAGNOSTIC PRINTS
  // Serial.print("loop_started_running == ");
  // Serial.println(loop_started_running);
  // Serial.print("loop_stopped_running == ");
  // Serial.println(loop_stopped_running);
  // Serial.println("-----------------");

  //  Hard fault detection in the event that the main program loop stops running after a power cycle
  if (!(loop_started_running && loop_stopped_running)) {
    loop_stopped_running = true;
  }
  else {
    // Serial.println("main loop stopped running");
    raise_fault_flag();
    digitalWrite(LED_indicator_red, 1);
    digitalWrite(LED_indicator_green, 0);
    digitalWrite(LED_indicator_blue, 0);
    Serial.print("System faulted at ");
    Serial.print(fault_hour);
    Serial.print(":");
    Serial.print(fault_minute);
    Serial.print(":");
    Serial.println(fault_second);
    wdog1_software_reset();
  }

  //  If we have a GPS lock, we service the watchdog timer to ensure that as long as the system fault flag is not raised, we are servicing it once per second.
  if (GPS_lock) {
    wdog1_serv();
  }
}

void GPS_fix_ISR() {
  if (GPS_fix_PPS_counter > 0) {
    GPS_lock = true;
    GPS_fix_PPS_counter = 0;
  }
  else {
    GPS_lock = false;
    time_lock = false;
    transmission_active = false;                                                          
    toggle_AD9850_FQ_UD();                
    load_AD9850_registers(0, 192);
    toggle_AD9850_FQ_UD();
    digitalWrite(XMIT, LOW);
    previous_hour = -1;
    current_hour = -1;
    next_hour = -1;
    previous_minute = -1;
    current_minute = -1;
    next_minute = -1;
    previous_second = -1;
    current_second = -1;
    next_second = -1;
  }

  //  If we do not have a GPS lock, we service the watchdog timer to ensure that as long as the system fault flag is not raised, we are servicing it once per second.
  if (!GPS_lock) {
    wdog1_serv();
  }
}

void GPS_RX_start() {
  RX_buffer = {};                                                             //  Make sure the RX buffer is clear before we begin clocking in bits
  RX_buffer_pointer = 0;                                                      //  Reset the RX buffer pointer
  GPIO7_IMR &= ~PIN_7_MASK;                                                   //  Reset the IMR bit corresponding to pin 7, so we don't interrupt on any data bits.
  bool status = GPS_RX_timer.begin(GPS_RX_ISR, 156);                          //  Begin the GPS_RX_ISR function at 156 us, which is 1.5 times the length of a single bit
  if (!status) {
    // Serial.println("GPS RX failed to start");
    raise_fault_flag();
  }
  GPS_RX_timer.update(104);                                                   //  The GPS_RX_ISR should fire after 156 us for the first bit, and then after 104 us for all subsequent bits.
}

void GPS_RX_ISR() {
  RX_buffer[RX_buffer_pointer] = digitalRead(GPS_RX_pin);                     //  Read the bit on the TX line using digitalRead()
  RX_buffer_pointer++;                                                        //  Increment the RX_buffer_pointer
  //  Check if that was the last data bit.
  if (RX_buffer_pointer == RX_buffer.size()) {                  
    GPS_RX_timer.end();                                                       //  Stop the timer if that is the case.
    RX_buffer_pointer = 0;                                                    //  Reset the RX buffer pointer.
    GPIO7_ISR = PIN_7_MASK;                                                   //  Set the ISR bit corresponding to the interrupt on pin 7.
    GPIO7_IMR |= PIN_7_MASK;                                                  //  Also the IMR bit corresponding to the interrupt on pin 7, so that we interrupt on the next start bit.

    received_byte = construct_byte(RX_buffer);                                //  Construct the byte value from the eight bits.
    if (received_byte == 36) {
      NMEA_sentence_complete = true;
      current_sentence[current_sentence_index] = '\0';                        //  Null terminate the current sentence.

      memcpy(previous_sentence, current_sentence, 100);
      current_sentence_index = 0;
    }
    current_sentence[current_sentence_index] = (char) received_byte;
    current_sentence_index++;
  }
}

uint8_t construct_byte(const std::array<int, 8>& bits) {
  unsigned int result = 0;
  unsigned int bit_scaler = 1;
  for (int i = 0; i < bits.size(); i++) {
    result += bits[i] * bit_scaler;
    bit_scaler *= 2;                                                          //  Scale this by 2 every bit to do the addition correctly.
  }
  return result;
}

void GPS_TX_ISR() {
  //  First check to see if TX_buffer_bits has bits that need to be transmitted
  if (TX_buffer_bits_filled) {
    //  Toggle FreqUp_diagnostic_pin if we're transmitting a data bit
    // digitalWrite(FreqUp_diagnostic_pin, HIGH);

    int current_bit = TX_buffer_bits[TX_buffer_bits_pointer];
    digitalWrite(GPS_RX_pin, current_bit);
    TX_buffer_bits_pointer++;
    if (TX_buffer_bits_pointer >= TX_buffer_bits.size()) {
      TX_buffer_bits_filled = false;                                            //  Reset this flag to indicate we're done transmitting the current byte
      GPS_TX_stop_bit = true;                                                 //  Make sure we transmit a stop bit afterwards.
    }
    // Serial.print("Transmitting bit ");
    // Serial.println(TX_buffer_bits_pointer - 1);

    // digitalWrite(FreqUp_diagnostic_pin, LOW);
  }
  //  Otherwise, check if we just finished transmitting 8 data bits and need to transmit a stop bit.
  else if (GPS_TX_stop_bit) {
    //  Toggle FreqDown_diagnostic_pin if we're transmitting a stop bit
    // digitalWrite(FreqDown_diagnostic_pin, HIGH);

    digitalWrite(GPS_RX_pin, HIGH);                                               //  The stop bit should always be a 1.
    GPS_TX_stop_bit = false;                                                  //  Reset this flag
    // Serial.println("Transmitting STOP BIT");

    // digitalWrite(FreqDown_diagnostic_pin, LOW);
  }
  //  Otherwise, check if TX_buffer_bytes has any bytes that need to be transmitted. We first transmit a start bit and load TX_buffer_bits with the first byte in TX_buffer bytes.
  else if (!TX_buffer_bytes.empty() && !load_GPS_TX_byte) {
    //  Toggle FreqDown_diagnostic_pin if we're transmitting a start bit
    // digitalWrite(FreqDown_diagnostic_pin, HIGH);

    digitalWrite(GPS_RX_pin, LOW);                                                //  The start bit should always be a 0.
    load_GPS_TX_byte = true;                                                    //  Set a flag that indicates that the TX buffer should be loaded with the bits for the next byte.
    // GPS_TX_start_bit = false;                                                 //  Reset this flag
    // Serial.println("Transmitting START BIT");

    // digitalWrite(FreqDown_diagnostic_pin, LOW);
  }
  //  Otherwise, shut down GPS_TX_timer
  else {
    GPS_TX_timer.end();
    GPS_TX_active = false;                                                    //  Reset this flag
  }
}

void load_GPS_TX_buffer(const std::string msg) {
  //  Toggle FreqDown diagnostic pin when loading the TX buffer
  // digitalWrite(FreqDown_diagnostic_pin, HIGH);

  for (char c : msg) {
    uint8_t corresponding_byte = (uint8_t) c;                                         //  Convert the ASCII character to a byte
    TX_buffer_bytes.push_back(corresponding_byte);                                  //  Load this byte into the TX buffer
  }

  //  Add carriage return and line feed
  TX_buffer_bytes.push_back((uint8_t) 13);                                    //  Carriage return
  TX_buffer_bytes.push_back((uint8_t) 10);                                    //  Line feed

  if (!GPS_TX_active) {
    GPS_TX_active = true;                                                     //  Set the GPS_TX_active flag
    // GPS_TX_start_bit = true;                                                  //  Set this flag high to indicate that the first bit that needs to be transmitted is a start bit.
    bool status = GPS_TX_timer.begin(GPS_TX_ISR, 104);             //  Begin the GPS_TX_timer at 104 us (the period corresponding to 9600 baud)
    if (!status) {
      // Serial.println("GPS TX failed to start");
      raise_fault_flag();
    }
  }

  // digitalWrite(FreqDown_diagnostic_pin, LOW);
}

void load_TX_buffer_bits(uint8_t byte) {
  //  Load TX_buffer_bits with the bits corresponding to this byte
  for (int i = 0; i < 8; i++) {
    TX_buffer_bits[i] = ((1 << i) & byte) ? 1 : 0;
  }
  TX_buffer_bits_filled = true;                                                 //  Set this flag to indicate TX_buffer_bits has been filled
  TX_buffer_bits_pointer = 0;                                                 //  Reset TX_buffer_bits_pointer
}

void station_ID_ISR() {
  toggle_AD9850_FQ_UD();
  
  if (station_ID_stop_flag) {
    // digitalWrite(XMIT, LOW);
    station_ID_timer.end();
    station_ID_index = 0;
    station_ID_stop_flag = false;
    station_ID_active = false;
    phase_reset();
  }
  else {
    station_ID_index++;
    if (station_ID_index >= station_ID_freqs.size()) {
      station_ID_stop_flag = true;
      load_AD9850_registers(0, 192);
    }
    else {
      load_AD9850_registers(station_ID_freqs[station_ID_index], 192);
    }
  }
}

void UTC_tick_ISR() {
  toggle_AD9850_FQ_UD();
  // digitalWrite(XMIT, LOW);
  UTC_tick_timer.end();
  bool status = timed_silence_timer.begin(timed_silence_ISR, 95000);
  if (!status) {
    Serial.println("timed silence failed to start");
    raise_fault_flag();
  }
  guarding_silence = true;
  phase_reset();
}

void timed_silence_ISR() {
  timed_silence_timer.end();
  // digitalWrite(XMIT, HIGH);
  if (BARKER_code_XMIT_flag) {
    // Serial.println("Beginning BARKER transmission");
    BARKER_code_ISR();
    bool status = BARKER_code_timer.begin(BARKER_code_ISR, 65000);
    if (!status) {
      Serial.println("BARKER code failed to start");
      raise_fault_flag();
    }
    BARKER_code_XMIT_flag = false;
  }
  else if (COSTAS_array_XMIT_flag) {
    // Serial.println("Beginning COSTAS array");
    COSTAS_array_ISR();
    bool status = COSTAS_array_timer.begin(COSTAS_array_ISR, 40000);
    if (!status) {
      Serial.println("COSTAS array failed to start");
      raise_fault_flag();
    }
    COSTAS_array_XMIT_flag = false;
  }
  else if (constant_audio_XMIT_flag) {
    // Serial.println("Beginning constant audio");
    toggle_AD9850_FQ_UD();
    bool status = constant_audio_timer.begin(constant_audio_ISR, 800000);
    if (!status) {
      Serial.println("constant audio failed to start");
      raise_fault_flag();
    }
    constant_audio_XMIT_flag = false;
    load_AD9850_registers(0, 192);
  }
}

void BARKER_code_ISR() {
  toggle_AD9850_FQ_UD();
  if (BARKER_code_stop_flag) {
    // digitalWrite(XMIT, LOW);
    BARKER_code_timer.end();
    BARKER_code_index = 0;
    BARKER_code_stop_flag = false;
    phase_reset();
  }
  //  First check if there are more symbols in the BARKER code
  else {
    BARKER_code_index++;
    if (BARKER_code_index >= BARKER_code_phases.size()) {
      load_AD9850_registers(0, 192);
      BARKER_code_stop_flag = true;
    }
    else {
      load_AD9850_registers(BARKER_code_freq, BARKER_code_phases[BARKER_code_index]);
    }
  }
}

void COSTAS_array_ISR() {
  toggle_AD9850_FQ_UD();
  if (COSTAS_array_stop_flag) {
    // digitalWrite(XMIT, LOW);
    COSTAS_array_timer.end();
    COSTAS_array_index = 0;
    COSTAS_array_stop_flag = false;
    phase_reset();
  }
  //  First check if there are more symbols in the COSTAS array
  else {
    COSTAS_array_index++;
    if (COSTAS_array_index >= COSTAS_array_freqs.size()) {
      load_AD9850_registers(0, 192);
      COSTAS_array_stop_flag = true;
    }
    else {
      load_AD9850_registers(COSTAS_array_freqs[COSTAS_array_index], COSTAS_array_phase);
    }
  }
}

void constant_audio_ISR() {
  // digitalWrite(XMIT, LOW);
  toggle_AD9850_FQ_UD();
  constant_audio_timer.end();
  phase_reset();
}

//  Write a 40 bit tuning word to the AD9850 on the SGCLK and SGDAT lines, which includes a 32 bit frequency tuning word and an 8 bit phase tuning word
void load_AD9850_registers(unsigned int freq, unsigned int phase) {
  //  DIAGNOSTIC PRINTS
  // Serial.print("Successfully wrote to AD9850 frequency and phase registers. FTW = ");
  // Serial.print(freq);
  // Serial.print(", PTW = ");
  // Serial.println(phase);

  //  Force AD9850 SGCLK and SGDAT lines low to begin.
  digitalWrite(AD9850_SGCLK, LOW);
  digitalWrite(AD9850_SGDAT, LOW);
  
  //  Write the 32 bit frequency tuning word.
  for (int i = 0; i < 32; i++) {
    digitalWrite(AD9850_SGDAT, (freq & ((unsigned long) 1 << i)) ? HIGH : LOW);   //  Use masking to extract the individual bits of the frequency tuning word.
    digitalWrite(AD9850_SGCLK, HIGH);
    digitalWrite(AD9850_SGCLK, LOW);
  }

  for (int i = 0; i < 8; i++) {
    digitalWrite(AD9850_SGDAT, (phase & ((unsigned long) 1 << i)) ? HIGH : LOW);  //  Use masking to extract the individual bits of the phase tuning word.
    digitalWrite(AD9850_SGCLK, HIGH);
    digitalWrite(AD9850_SGCLK, LOW);
  }
}

//  Toggle the FQ_UD pin on the AD9850. Used to load previously written frequency and phase tuning words to the registers of the AD9850.
void toggle_AD9850_FQ_UD() {
  // Serial.println("Successfully loaded previously written FTW and PTW to AD9850 registers.");
  digitalWrite(AD9850_SGLD, HIGH);
  digitalWrite(AD9850_SGLD, LOW);
}

void construct_station_ID_freqs() {
  //  DIAGNOSTIC PRINTS
  // Serial.println("Checking to make sure the station_ID_freqs array is being constructed correctly: ");
  for (char c : morse_ID_raw) {
    if (c == '.') {
      station_ID_freqs.emplace_back(station_ID_carrier_freq);
      station_ID_freqs.emplace_back(0);
      //  DIAGNOSTIC PRINTS
      // Serial.println(station_ID_carrier_freq);
      // Serial.println(0);
    }
    else if (c == '-') {
      for (int i = 0; i < 3; i++) {
        station_ID_freqs.emplace_back(station_ID_carrier_freq);
        //  DIAGNOSTIC PRINTS
        // Serial.println(station_ID_carrier_freq);
      }
      station_ID_freqs.emplace_back(0);
      //  DIAGNOSTIC PRINTS
      // Serial.println(0);
    }
    else if (c == ' ') {
      for (int i = 0; i < 3; i++) {
        station_ID_freqs.emplace_back(0);
        //  DIAGNOSTIC PRINTS
        // Serial.println(0);
      }
    }
  }
}

void phase_reset() {
  digitalWrite(AD9850_SGRST, HIGH);
  digitalWrite(AD9850_SGRST, LOW);
  load_AD9850_registers(0, 192);
  toggle_AD9850_FQ_UD();
  if (!guarding_silence) {
    if ((current_minute + 1) % 5 == 0 && current_second == 59) {
      load_AD9850_registers(station_ID_freqs[0], 192);
    }
    else if (current_second == 59) {
      load_AD9850_registers(UTC_minute_tick_FTW, 192);
    }
    else {
      load_AD9850_registers(UTC_second_tick_FTW, 192);
    }
  }
  else {
    guarding_silence = false;
    if (current_second % 5 == 2) {
      // Serial.println("Loading AD9850 with first tuning words of BARKER code");
      load_AD9850_registers(BARKER_code_freq, BARKER_code_phases[0]);
      BARKER_code_XMIT_flag = true;
    }
    else if (current_second % 5 == 3) {
      // Serial.println("Loading AD9850 with first tuning words of COSTAS array");
      load_AD9850_registers(COSTAS_array_freqs[0], 192);
      COSTAS_array_XMIT_flag = true;
    }
    else if (current_second % 5 == 4) {
      // Serial.print("Loading AD9850 with first tuning words of constant audio tone");
      load_AD9850_registers(constant_audio_freqs[constant_audio_index], 192);
      constant_audio_index = (constant_audio_index + 1) % 2;
      constant_audio_XMIT_flag = true;
    }
    else {
      load_AD9850_registers(UTC_second_tick_FTW, 192);
    }
  }
}

//  Initializes the watchdog timer
void wdog1_init(uint8_t timeout) {
  WDOG1_WMCR = 0;                     //  Reset this bit to make sure we avoid an unintended initialization timeout.
  WDOG1_WCR = (timeout << 8) | 27;    //  Set the watchdog control register to store the timeout value, and to set bit 4 (resets the system on watchdog timeout), bit 1 (software reset enable), and bit 0 (watchdog timer enable)
  // Serial.println("Initializing WDT1");
}

//  Services the watchdog timer
void wdog1_serv() {
  //  We don't want any interrupts to fire while we're servicing the watchdog timer
  __disable_irq();
  //  To service WDT1 on the Teensy 4.0, we need to set the WSR to 0x5555 and then 0xAAAA.
  WDOG1_WSR = 21845;                  //  Set the WSR to 0x5555
  WDOG1_WSR = 43690;                  //  Set the WSR to 0xAAAA
  __enable_irq();
  // Serial.println("Servicing WDT1");
}

//  Routine to assert a chip reset in software
void wdog1_software_reset() {
  // Serial.println("Attempting to assert WDT1 software reset...");
  WDOG1_WCR &= ~(1 << 4);                 //  Reset bit 1 of the WCR to assert a reset from software.
}

//  Helper routine to raise and timestamp the system fault flag
void raise_fault_flag() {
  system_fault_flag = true;
  fault_hour = current_hour;
  fault_minute = current_minute;
  fault_second = current_second;
}

void loop() {
  loop_started_running = true;
  loop_stopped_running = false;

  // Serial.println("Entering loop()");
  if (NMEA_sentence_complete) {
    extracted_hour.push_back(previous_sentence[7]);
    extracted_hour.push_back(previous_sentence[7]);
    extracted_minute.push_back(previous_sentence[9]);
    extracted_minute.push_back(previous_sentence[10]);
    extracted_second.push_back(previous_sentence[11]);
    extracted_second.push_back(previous_sentence[12]);

    next_second = (std::stoi(extracted_second) + 1) % 60;
    if (next_second == 0) {
      // Serial.println("Detected that next_second == 0");
      next_minute = (std::stoi(extracted_minute) + 1) % 60;
      // Serial.print("Setting next_minute to ");
      // Serial.println(next_minute);
      if (next_minute == 0) {
        next_hour = (std::stoi(extracted_hour) + 1) % 60;
      }
      else {
        next_hour = std::stoi(extracted_hour);
      }
    }
    else {
      next_minute = std::stoi(extracted_minute);
      next_hour = std::stoi(extracted_hour);
    }

    extracted_hour.clear();
    extracted_minute.clear();
    extracted_second.clear();

    NMEA_sentence_complete = false;
  }
  if (load_GPS_TX_byte) {
    uint8_t byte_to_load = TX_buffer_bytes.front();
    // Serial.print("Loading character: ");
    // Serial.println((char) byte_to_load);
    TX_buffer_bytes.erase(TX_buffer_bytes.begin());
    load_TX_buffer_bits(byte_to_load);
    load_GPS_TX_byte = false;
  }
  if (transmission_active) {
    digitalWrite(LED_indicator_red, 0);
    digitalWrite(LED_indicator_green, 1);
    digitalWrite(LED_indicator_blue, 0);
  }
  else if (!(transmission_active || system_fault_flag)) {
    digitalWrite(LED_indicator_red, 0);
    digitalWrite(LED_indicator_green, 0);
    digitalWrite(LED_indicator_blue, 1);
  }
  else if (system_fault_flag) {
    digitalWrite(LED_indicator_red, 1);
    digitalWrite(LED_indicator_green, 0);
    digitalWrite(LED_indicator_blue, 0);
    // Serial.println("System fault flag raised, we shouldn't be servicing the WDT1 until the chip is reset.");
    Serial.print("System faulted at ");
    Serial.print(fault_hour);
    Serial.print(":");
    Serial.print(fault_minute);
    Serial.print(":");
    Serial.println(fault_second);
    wdog1_software_reset();
  }
}
