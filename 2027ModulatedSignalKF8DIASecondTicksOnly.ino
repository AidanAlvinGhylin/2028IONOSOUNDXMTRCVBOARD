#include <_Teensy.h>
#include <iostream>
#include <string>

//  2027ModulatedSignalKF8DIA.ino
//
//  This sketch is meant to transmit five different modulated elements on intervals:
//  - A UTC second tick, 5 ms in length with a 1.5 kHz carrier, on every 1 PPS rising edge.
//  - The origin station's call sign, at 24.5 WPM with a 700 Hz carrier, at the beginning 
//    of any transmission and once every five minutes following.
//  - The n=13 BARKER code, 65 ms per chirp with an 800 Hz carrier, once every five seconds during transmission.
//  - The n=21 COSTAS array {3, 0, 8, 2, 18, 6, 15, 14, 9, 7, 20, 16, 19, 11, 17, 1, 13, 4, 5, 10, 12}, 40 ms per 
//    chirp, the lowest frequency being 300 Hz and the chirps spaced at multiples of 120 Hz, once every five seconds during transmission.
//  - Two different constant audio tones, 500 Hz and 600 Hz, each 800 ms in length, each once every ten seconds during transmission.
//
//  This sketch is meant to run on the COSTAS XMT/RCV board Rev. 1.10 or 1.20

//  Hardware Timer Definitions. These are used to time the different modulated signal elements precisely.

IntervalTimer UTC_second_timer;
IntervalTimer timed_silence_timer;
IntervalTimer BARKER_code_timer;
IntervalTimer COSTAS_array_timer;
IntervalTimer constant_audio_timer;
IntervalTimer station_ID_timer;

//  Frequency and Phase Tuning Word Definitions. These are used to program the frequency and phase of the output of the AD9850.

unsigned int power_down_freq = 0;                 
unsigned int power_down_phase = 4;                            //  Asserts the power down pin on the AD9850

unsigned int silent_freq = 0;                                 //  0 Hz
unsigned int silent_phase = 192;                              //  270 degrees

unsigned int UTC_second_freq = 6012954;                       //  1.4 kHz
unsigned int UTC_second_phase = 192;                          //  270 degrees

unsigned int BARKER_code_freq = 3435973;                      //  800 Hz
std::array<int, 13> BARKER_code_phases = {192, 192, 192, 192, 192, 64, 64, 192, 192, 64, 192, 64, 192}; //  BPSK throughout BARKER code (either 270 or 90 degrees)

std::array<int, 21> COSTAS_array_freqs = {2834678, 1288490, 5411658, 2319282, 10565619, 4380866, 9019431, 8504035, 5927054, 4896262, 11596411, 9534827, 11081015, 6957847, 10050223, 1803886, 7988639, 3350074, 3865470, 6442450, 7473243}; //  FSK throughout COSTAS array (21 unique frequencies, lowest 300 Hz, spaced in 120 Hz increments)
unsigned int COSTAS_array_phase = 192;                        //  270 degrees

std::array<int, 2> constant_audio_freqs = {2147483, 2576980}; //  Two different carriers, 500 Hz and 600 Hz, used for the constant audio tones.
unsigned int constant_audio_phase = 192;                      //  270 degrees

unsigned int station_ID_carrier_freq = 3006477;               //  700 Hz
std::vector<int> station_ID_freqs = {};                       //  
unsigned int station_ID_phase = 192;                          //  270 degrees
std::string morse_ID_raw = ".-- ---.. . -.. ..- -..-. . .-";  //  Raw morse ID string. Used to construct the freqs array.

//  State Variable Definition. This value indicates the state the machine is in at any given time.

volatile bool transmitting = false;         //  Indicates whether or not we are transmitting the second tick.

//  Program Control Variable Definitions. These variables determine program flow between the states.

volatile int PPS_counter = 0;               //  Incremented every time the PPS pin on the Adafruit GPS Breakout v3 Board pulses high. This is used to determine when to enter station identification mode.
volatile int GPS_fix_PPS_counter = 0;       //  Also incremented every time the PPS pin pulses high. This is used to track whether the GPS receiver has a signal.
volatile bool GPS_lock = false;             //  Set if a the GPS receiver has a signal; reset if it does not.

//  Counter Variable Definitions. These variables are incremented throughout transmission of individual signal elements to determine the correct frequency and phase tuning words.

volatile int BARKER_code_counter = 0;
volatile int COSTAS_array_counter = 0;
volatile int constant_audio_counter = 0;
volatile int station_ID_counter = 0;

//  Timer Function Stop Flags. These variables are set if a timer function needs to be stopped on the next cycle.

volatile bool station_ID_stop_flag = false;
volatile bool UTC_second_stop_flag = false;
volatile bool timed_silence_stop_flag = false;
volatile bool BARKER_code_stop_flag = false;
volatile bool COSTAS_array_stop_flag = false;
volatile bool constant_audio_stop_flag = false;

//  Teensy I/O pin definitions.

  const byte AD9850_SGCLK = 12;               //  Used as CLK line when writing frequency and phase tuning words to the AD9850.
  const byte AD9850_SGDAT = 11;               //  Used as DATA line when writing frequency and phase tuning words to the AD9850.
  const byte AD9850_SGLD = 13;                //  When toggled, previously written frequency and phase tuning words are loaded to the registers of the AD9850.
  const byte AD9850_SGRST = 20;               //  When toggled, the frequency and phase registers of the AD9850 are reset. This toggled during every silent period.
  const byte PPS_interrupt_pin = 2;           //  Connected to the PPS output of the Adafruit GPS v3 Breakout Board, used as the 1 PPS signal for timing the main program loop.
  const byte LED_indicator_red = 3;           //  Connected to the red bit of the LED indicator on the board.
  const byte LED_indicator_green = 4;         //  Connected to the green bit of the LED indicator on the board.
  const byte LED_indicator_blue = 5;          //  Connected to the blue bit of the LED indicator on the board.
  const byte GPS_fix_interrupt_pin = 6;                     //  Connected to the FIX pin of the Adafruit GPS v3 Breakout Board. Used outside the main program loop to track whether the 
                                              //  receiver has a signal.
  const byte GPS_lock_LED_indicator = 10;         //  Connected to the GPS lock LED indicator on the board.
  const byte XMIT_enable_pin = 17;            //  Connected to the XMIT enable jumper on the board, active low.
  const byte XMIT = 21;                       //  Used to control the push to talk on the radio transceiver.
  const byte FreqUp_diagnostic_pin = 19;      //  Connected to the FreqUp diagnostic pin on board, used for triggering in system verification.
  const byte FreqDown_diagnostic_pin = 18;    //  Connected to FreqUp diagnostic pin on board, also used for triggering in system verification.

//  OTHER DIAGNOSTIC FLAGS

volatile bool system_fault_flag = false;  //  Indicates a problem in the system
volatile bool system_idle = true; //  Indicates if the system is working properly but IDLE (not transmitting)

volatile bool last_AD9850_communication = false;  //  Indicates the previous write/load operation in the wired communication with the AD9850. false = write, true = load.
volatile bool phase_reset_executed_previously = false;

void setup() {
  construct_station_ID_freqs();               //  Construct the station ID freqs array with the raw morse ID

  //  Declare Teensy I/O pins as inputs or outputs
  pinMode(AD9850_SGCLK, OUTPUT);
  pinMode(AD9850_SGDAT, OUTPUT);
  pinMode(AD9850_SGLD, OUTPUT);
  pinMode(AD9850_SGRST, OUTPUT);
  pinMode(LED_indicator_red, OUTPUT);
  pinMode(LED_indicator_green, OUTPUT);
  pinMode(LED_indicator_blue, OUTPUT);
  pinMode(GPS_lock_LED_indicator, OUTPUT);
  pinMode(XMIT_enable_pin, INPUT);
  pinMode(XMIT, OUTPUT);
  pinMode(FreqUp_diagnostic_pin, OUTPUT);
  pinMode(FreqDown_diagnostic_pin, OUTPUT);

  //  Hardware interrupt definitions.
  attachInterrupt(PPS_interrupt_pin, PPS_ISR, RISING);          //  Calls PPS_ISR on every rising edge of the PPS_interrupt_pin
  attachInterrupt(GPS_fix_interrupt_pin, GPS_fix_ISR, RISING);  //  Calls GPS_fix_ISR on every rising edge of the GPS_fix_interrupt_pin

  Serial.begin(115200);
  Serial.println("---------------------------------------");
  Serial.println("2027ModulatedSignalKF8DIA.ino Rev 1.0.");
  Serial.println("---------------------------------------");
}

//  On the rising edge of the 1 PPS pin, the ISR determines whether transmission should be enabled or disabled, and whether to enter station ID or UTC second tick.
void PPS_ISR() {
  //  DIAGNOSTIC PRINTS
  // Serial.print("PPS_ISR() entered. digitalread(XMIT_enable_pin) == ");
  // Serial.print(digitalRead(XMIT_enable_pin));
  // Serial.print(". GPS_lock == ");
  // Serial.print(GPS_lock);
  // Serial.print(". state_ID == ");
  // Serial.println(state_ID);
  
  GPS_fix_PPS_counter++;  //  Increment GPS_fix_PPS_counter outside the conditional statement, so transmission can start at the beginning of the program.
  
  //  Check to see if both of the following conditions are satisfied:
  //  1.  The XMIT_enable_pin is grounded
  //  2.  The GPS receiver has a signal
  if (!digitalRead(XMIT_enable_pin) & GPS_lock) {
    digitalWrite(XMIT, HIGH);
    UTC_second_function();
    UTC_second_timer.begin(UTC_second_function, 5000);
    system_idle = false;
    transmitting = true;

    PPS_counter++;
  }
  else {
    system_idle = true;
    //  Transmission should be inactive in this case
    if (transmitting) {
      // load_AD9850_registers(silent_freq, silent_phase);   //  TEMPORARY DIAGNOSTICS
      load_AD9850_registers(UTC_second_freq, UTC_second_phase);
      transmitting = false;
    }
  }
}

//  Sets the GPS_lock indicator based on whether the GPS receiver has a signal.
void GPS_fix_ISR() {
  //  DIAGNOSTIC PRINTS
  // Serial.print("Entered GPS_fix_ISR(). GPS_fix_PPS_counter == ");
  // Serial.println(GPS_fix_PPS_counter);
  
  if (GPS_fix_PPS_counter > 0) {    //  If this is greater than 0, that means the previous pulse was more than 1 second prior and the GPS receiver has a signal.
    GPS_lock = true;
    digitalWrite(GPS_lock_LED_indicator, HIGH);
    GPS_fix_PPS_counter = 0;
  }
  else {                            //  Otherwise, the FIX pin is pulsing at 1 Hz, meaning there is no signal.
    GPS_lock = false;
    digitalWrite(GPS_lock_LED_indicator, LOW);
  }
}

//  UTC second timer function
void UTC_second_function() {
  // Serial.println("Entered UTC_second_function()");
  digitalWrite(FreqUp_diagnostic_pin, HIGH);
  Serial.println("Calling toggle_AD9850_FQ_UD() in UTC_second_function()");
  toggle_AD9850_FQ_UD();  //  Load the previously written frequency and phase values to the registers on the AD9850.
  digitalWrite(FreqUp_diagnostic_pin, LOW);

  //  Check the second tick stop flag.
  if (UTC_second_stop_flag) {
    digitalWrite(XMIT, LOW);
    untimed_silence_phase_reset();    //  begin the untimed silence phase reset routine.
    UTC_second_timer.end();   //  End the UTC second timer function.

    //  DIAGNOSTIC PRINT
    // Serial.print("state_ID++ in UTC_second_function(). Incremented from ");
    // Serial.print(state_ID - 1);
    // Serial.print(" to ");
    // Serial.println(state_ID);

    UTC_second_stop_flag = false; //  Reset the second tick stop flag.
  }
  else {
    UTC_second_stop_flag = true;  //  Set the UTC second stop flag.
    load_AD9850_registers(silent_freq, silent_phase);
  }
}

//  Untimed silence phase reset routine. This involves toggling the AD9850 RESET pin, resetting the phase register to cosine 0.
void untimed_silence_phase_reset() {
  digitalWrite(FreqDown_diagnostic_pin, HIGH);

  //  DIAGNOSTIC PRINTS
  // Serial.println("Entered untimed_silence_phase_reset()");
  
  //  First, toggle the RESET pin on the AD9850.
  Serial.println("Toggling AD9850_SGRST pin."); //  DIAGNOSTIC PRINT
  digitalWrite(AD9850_SGRST, HIGH);
  digitalWrite(AD9850_SGRST, LOW);

  load_AD9850_registers(silent_freq, silent_phase); //  Load the AD9850 frequency and phase registers with silent values.
  Serial.println("Calling toggle_AD9850_FQ_UD() in untimed_silence_phase_reset()");
  toggle_AD9850_FQ_UD();  //  Toggle the FQ_UD pin on the AD9850.
  // load_AD9850_registers(silent_freq, silent_phase); //  TEMPORARY DIAGNOSTIC
  load_AD9850_registers(UTC_second_freq, UTC_second_phase); //  Load the AD9850 frequency and phase registers with UTC second values, as that will always follow untimed silence.

  phase_reset_executed_previously = true; //  Set this flag for error detection

  digitalWrite(FreqDown_diagnostic_pin, LOW);
}

//  DIAGNOSTIC: Power down reset routine. This involves writing and loading a set of tuning words to the AD9850 that asserts the power down pin, supposedly resetting the frequency and phase registers.
void untimed_silence_power_down_reset() {
  digitalWrite(FreqDown_diagnostic_pin, HIGH);
  Serial.println("Asserting AD9850 power down.");
  load_AD9850_registers(power_down_freq, power_down_phase); //  Load the AD9850 frequency and phase registers with the power down values.
  Serial.println("Calling toggle_AD9850_FQ_UD() in untimed_silence_power_down_reset()");
  toggle_AD9850_FQ_UD();  //  Toggle the FQ_UD pin on the AD9850.
  load_AD9850_registers(UTC_second_freq, UTC_second_phase); //  Load the AD9850 frequency and phase registers with UTC second values, as that will always follow untimed silence.

  phase_reset_executed_previously = true; //  Set this flag for error detection

  digitalWrite(FreqDown_diagnostic_pin, LOW);
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

  //  Ensure we didn't transmit two consecutive sets of tuning words without toggling the FQ_UD pin, if so indicate a fault
  if (!last_AD9850_communication) {
    system_fault_flag = true;
    Serial.println("System fault flag raised in load_AD9850_registers(): transmitted two consecutive tuning word sets without loading the first one.");
  }
  else {
    last_AD9850_communication = false; //  Set to true for a write operation.
  }
}

//  Toggle the FQ_UD pin on the AD9850. Used to load previously written frequency and phase tuning words to the registers of the AD9850.
void toggle_AD9850_FQ_UD() {
  // Serial.println("Successfully loaded previously written FTW and PTW to AD9850 registers.");
  digitalWrite(AD9850_SGLD, HIGH);
  digitalWrite(AD9850_SGLD, LOW);

  //  Ensure we didn't toggle the FQ_UD pin twice consecutively without loading a set of tuning words.
  if (last_AD9850_communication) {
    system_fault_flag = true;
    Serial.println("System fault flag raised in toggle_AD9850_FQ_UD(): toggled the FQ_UD pin twice consecutively without loading a set of tuning words.");
  }
  else {
    last_AD9850_communication = true; //  Set to true for a load operation.
  }
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

void loop() {
  if (system_fault_flag) {
    //  Red LED indicates fault in the system
    digitalWrite(LED_indicator_red, 1);
    digitalWrite(LED_indicator_green, 0);
    digitalWrite(LED_indicator_blue, 0);
  }
  else if (system_idle) {
    //  Blue LED indicates the system is working properly but is IDLE
    digitalWrite(LED_indicator_red, 0);
    digitalWrite(LED_indicator_green, 0);
    digitalWrite(LED_indicator_blue, 1);
  }
  else {
    //  Green LED indicates the system is working properly
    digitalWrite(LED_indicator_red, 0);
    digitalWrite(LED_indicator_green, 1);
    digitalWrite(LED_indicator_blue, 0);
  }
}