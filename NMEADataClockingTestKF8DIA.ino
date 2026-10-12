#include <_Teensy.h>
#include <iostream>
#include <string>
#include <cmath>
#include <vector>

//  Hardware timer definitions
IntervalTimer GPS_RX_timer;
IntervalTimer GPS_TX_timer;
IntervalTimer audio_tone_timer;

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
const int AD9850_SGCLK = 12;               //  Used as CLK line when writing frequency and phase tuning words to the AD9850.
const int AD9850_SGDAT = 11;               //  Used as DATA line when writing frequency and phase tuning words to the AD9850.
const int AD9850_SGLD = 13;                //  When toggled, previously written frequency and phase tuning words are loaded to the registers of the AD9850.
const int AD9850_SGRST = 20;               //  When toggled, the frequency and phase registers of the AD9850 are reset. This toggled during every silent period.

//  Counter definitions
// volatile int PPS_counter = 0;
volatile int GPS_fix_PPS_counter = 0;
volatile int PPS_counter = 0;

//  Flag definitions
// volatile bool GPS_lock = false;                                               //  Flag to indicate whether we have a GPS fix.
volatile bool GPS_lock = false;                                                //  Force this true as a temporary diagnostic.
volatile bool transmission_active = false;                                    //  Flag to indicate whether transmission is active.

//  NMEA byte receiving structure. Individual bits are clocked in to a buffer array of length 8. Once we have a full byte, the byte is loaded into a dynamic array and printed.

std::array<int, 8> RX_buffer = {};
volatile int RX_buffer_pointer = 0;

std::vector<uint8_t> NMEA_bytes;
volatile uint8_t next_NMEA_byte = 0;
std::string current_sentence = "";                                            //  Used to construct the NMEA sentence coming in from the GPS receiver.
std::string previous_sentence = "";                                           //  Used to store the previous NMEA sentence after it is fully constructed and verified. Used for data extraction.
volatile bool byte_available = false;
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
std::string extracted_UTC_second_string = "";
volatile bool NMEA_sentence_complete = false;

//  Used to store information from the NMEA sentences from the GPS receiver.
volatile int current_second = -1;                                             
volatile int next_second = -1;

//  Test tone list
std::array<int, 6> tone_FTWs = {1717986, 3435973, 5153960, 6871947, 8589934, 10307921};
volatile bool new_second = false;

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

  Serial.begin(115200);

  Serial.println("------------------------------");
  Serial.println("NMEADataClockingTestKF8DIA.ino");
  Serial.println("------------------------------");

  load_GPS_TX_buffer("$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n");                              //  PMTK314 (sets NMEA output) to only Global Positioning Fix Data (GGA) sentences
  load_GPS_TX_buffer("$PMTK220,100*2F\r\n"); 

  digitalWrite(AD9850_SGRST, HIGH);
  digitalWrite(AD9850_SGRST, LOW);
}

void PPS_ISR() {
  //  This value needs to be incremented every time, whether we have a GPS signal or not.
  GPS_fix_PPS_counter++;

  current_second = next_second;

  //  Temporarily commenting this out as the Teensy is having problems that I am figuring out
  // current_second = next_second;
  if (!digitalRead(XMIT_enable_jumper) && GPS_lock) {
    toggle_AD9850_FQ_UD();
    PPS_counter++;
    if (!transmission_active) {
      transmission_active = true;
    }
    GPS_lock = true;
    new_second = true;
  }
  else {
    PPS_counter = 0;
    transmission_active = false;                                                                                 //  Disable transmission
    GPS_lock = false;
    toggle_AD9850_FQ_UD();
    load_AD9850_registers(0, 192);
    toggle_AD9850_FQ_UD();
  }
}

void GPS_fix_ISR() {
  if (GPS_fix_PPS_counter > 0) {
    GPS_lock = true;
    GPS_fix_PPS_counter = 0;
  }
  else {
    GPS_lock = false;
    transmission_active = false;                                                          
    toggle_AD9850_FQ_UD();                
    load_AD9850_registers(0, 192);
    toggle_AD9850_FQ_UD();
  }
}

void GPS_RX_start() {
  RX_buffer = {};                                                             //  Make sure the RX buffer is clear before we begin clocking in bits
  RX_buffer_pointer = 0;                                                      //  Reset the RX buffer pointer
  GPIO7_IMR &= ~PIN_7_MASK;                                                   //  Reset the IMR bit corresponding to pin 7, so we don't interrupt on any data bits.
  GPS_RX_timer.begin(GPS_RX_ISR, 156);                                        //  Begin the GPS_RX_ISR function at 156 us, which is 1.5 times the length of a single bit
  GPS_RX_timer.update(104);                                                   //  The GPS_RX_ISR should fire after 156 us for the first bit, and then after 104 us for all subsequent bits.
}

void GPS_RX_ISR() {
  RX_buffer[RX_buffer_pointer] = digitalRead(GPS_RX_pin);                     //  Read the bit on the TX line using digitalRead()
  RX_buffer_pointer++;                                                        //  Increment the RX_buffer_pointer
  //  Check if that was the last data bit.
  if (RX_buffer_pointer == RX_buffer.size()) {                  
    GPS_RX_timer.end();                                                       //  Stop the timer if that is the case.
    RX_buffer_pointer = 0;                                                    //  Reset the RX buffer pointer.
    received_byte = construct_byte(RX_buffer);                                //  Construct the byte value from the eight bits.
    process_GPS_RX_byte = true;                                               //  Set a flag that indicates that a full byte has been received and needs to be processed.
    GPIO7_ISR = PIN_7_MASK;                                                   //  Set the ISR bit corresponding to the interrupt on pin 7.
    GPIO7_IMR |= PIN_7_MASK;                                                  //  Also the IMR bit corresponding to the interrupt on pin 7, so that we interrupt on the next start bit.
    if (received_byte == 36) {
      NMEA_sentence_complete = true;
      previous_sentence = current_sentence;
      current_sentence = "";
    }
    current_sentence += (char) received_byte;
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
    GPS_TX_timer.begin(GPS_TX_ISR, 104);             //  Begin the GPS_TX_timer at 104 us (the period corresponding to 9600 baud)
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

void loop() {
  if (NMEA_sentence_complete) {
    const int extracted_second = std::stoi(previous_sentence.substr(11, 2));
    if (next_second != ((extracted_second + 1) % 60)) {
      next_second = (extracted_second + 1) % 60;
    }
    NMEA_sentence_complete = false;
  }
  if (new_second) {
    // Serial.println("Detected new_second == true in loop()");
    load_AD9850_registers(tone_FTWs[(current_second + 1) % 6], 192);
    new_second = false;
  }
  if (transmission_active) {
    digitalWrite(LED_indicator_red, 0);
    digitalWrite(LED_indicator_green, 1);
    digitalWrite(LED_indicator_blue, 0);
  }
  else {
    digitalWrite(LED_indicator_red, 0);
    digitalWrite(LED_indicator_green, 0);
    digitalWrite(LED_indicator_blue, 1);
  }
}
