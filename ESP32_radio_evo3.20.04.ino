// ################################################################################################################### //
// Evo Internet Radio                                                                                                  //
// ################################################################################################################### //
// Author:   Robgold 2026, Made in Poland                                                                              //
// Translate to English by "Roverius" 2026 Belgium                                                                     //
// Translate version: https://github.com/Roverius/EVO-Radio/tree/main/ESP32_radio_evo3.20.04.ino                       //
//                                                                                                                     //
// Source -> https://github.com/dzikakuna/ESP32_radio_evo3/tree/main/src/ESP32_radio_evo3.20                           //
// ################################################################################################################### //
//                                                                                                                     //
// Compilator: Arduino, Platformio                                                                                     //
// ESP32 by Espressif:                          3.3.7  plus recomiled TCP/IP libs for FLAC                             //
// ESP32-audioI2S-master by schreibfaul1:       3.4.4x with commits 23.02.2026,                                        //
// ESP Async WebServer:                         3.10.0                                                                 //
// Async TCP:                                   3.4.10                                                                 // 
// ezButton:                                    1.0.6                                                                  //
// U8g2 by Oliver Olikraus:                     2.35.30                                                                //
// WiFiManager by tzapu                         2.0.17                                                                 //
// Board:                                       ESP32S3 Deve Module                                                    //
// Flash Mode:                                  QIO 80Mhz                                                              //
// PSRAM                                        OPI PSRAM                                                              //
// Partition Scheme:                            8M with spiffs (3MM APP/1.5MB SPIFFS)                                  //
// voiceTime language                           Change line 11275 for language voiceTimePL, voiceTimeEN, voiceTimeNL   //
// ################################################################################################################### //

#include "Arduino.h"           // Standard Arduino header that provides basic functions and definitions
#include <WiFiManager.h>       // Library for managing WiFi network configuration, description of how to set up WiFi connection on first startup is described here: https://github.com/tzapu/WiFiManager
#include <Audio.h>             // Library for handling sound and audio-related functions
#include "SPI.h"               // Library for handling SPI communication
#include "FS.h"                // Library for file system support
#include <U8g2lib.h>           // Display support library
#include <ezButton.h>          // Library for operating an encoder with a button
#include <HTTPClient.h>        // Library for making HTTP requests, enables communication with servers via the HTTP protocol
#include <Ticker.h>            // Ticker mechanism to refresh the 1s timer, helpful for cyclical actions in the main loop
#include <EEPROM.h>            // EEPROM emulation support library
#include <Time.h>              // Library for handling time-related functions, e.g. reading the date and time
#include <ESPAsyncWebServer.h> // Asynchronous web server library
#include <AsyncTCP.h>          // TCP library for web server
#include <Update.h>            // Library for OTA updates
#include <ESPmDNS.h>           // mDNS Library for ESP
// #include <Adafruit_NeoPixel.h> // RGB NeoPixel LED
#include "soc/rtc_cntl_reg.h"   // ESP libraries to be able to do a full reset 
#include "soc/rtc_cntl_struct.h"// ESP libraries to be able to do a full reset 
#include <esp_sleep.h>
#include "esp_system.h"
// #include <IRremoteESP8266.h>
#include "esp_heap_caps.h"

//#include "rom/gpio.h"     // CS_SD pin mapping library
//#include "esp_wifi.h"     // Library for setting the 20MHz band


// --------------- RADIO VERSION AND HOST NAME DEFINITION ---------------
#define softwareRev "v3.20.04"  // Radio software version
#define hostname "evoradio"     // Hostname definition visible on the network
// -------------------------------------------------------------------- //

   
// ################ USER CONFIGURATION - START ################
// -- OLED --
#define SSD1322     // 3.12 inch OLED display, SSD1322-256x64px
//#define SSD1309     // 2.42 inch OLED display, SSD1306-128x64px ONLY DisplayMode7!
//#define SH1122      // 2.08 inch OLED display, SSD1122-256x64px
//#define SSD1363     // 3.12 inch OLED display, SSD1322-256x128px works but no full screen mode

//-- SPI current efficiency --
//#define LOWNOISESPI   // Sets the current efficiency of the SPI pins (0 - lowest, low noise; 3 - highest, high noise), EMI reduction

//-- MEMORY STORAGE --
#define AUTOSTORAGE   // Definition of whether autostorage is enabled (automatic selection of SD card/LittleFS or SPIFFS memory depending on the following definition)
#define USE_SD        // We use an SD card, leave it in case of AUTOSTORAGE or when we only want to use the SD card
//#define USE_SPIFFS  // We define whether it will be SPIFFS or LittleFS when using internal memory. Uncomment ONLY one of LittleFS or SPIFFS!
#define USE_LittleFS  // See above

// -- SECOND ENCODER --
//#define twoEncoders // We turn on the second encoder

// -- ACCESSORIES - RS232 Communication, TAS Amplifier etc.. --
//#define SERIALCOM   // Bidirectional RS232 communication
//#define TAS5805       // Compilation for the version with TAS5805 amplifier with built-in DAC, comment in the case of standard PCM5102

const bool f_logoSlowBrightness = 1; // Enabling slow fade-in of the logo

// --------------- BUTTONS and LEDs ---------------
//#define SW_POWER 8        // Dedicated Power button
#define STANDBY_LED 17    // LED to indicate standby mode
#define IR_LED 17        // IR receiver activity LED
//#define SD_LED 17       // SD card activity LED, card CS signal indication, pin clone useful when using an SD reader
#define IR_LED_ON 1
#define IR_LED_OFF 0
// ################ USER CONFIGURATION - END ################



// --------------- TAS5805 / PCM5102 ---------------
#ifdef TAS5805 //PowerAmp with I2S & DSP                 
  #include <Wire.h>           // I2C dla komunikacji z TI TAS5805
  #define I2C_CLK 9           // GPIO CLOCK I2C
  #define I2C_DATA 8          // GPIO DATA I2C
  #define TAS5805_ADDR 0x2D   // Adres I2C TAS5805, pull-up na ADR/#FAULT 10k podbicie adresu o 1
  
  // --------------- TAS5805 - AMP pin definition with converter ---------------
  #define I2S_DOUT 16        // Connecting to the DIN pin on the TAS DAC
  #define I2S_BCLK 14        // Connecting to the BCK pin on the TAS DAC
  #define I2S_LRC 15         // Connecting to the LCK pin on the TAS DAC 
  #define recv_pin 7
#else //PCM5012 simple DAC
  // --------------- PCM5102A - converter pin definition ---------------
  #define I2S_DOUT 13             // Connecting to the DIN pin on the DAC
  #define I2S_BCLK 12             // Connecting to the BCK pin on the DAC
  #define I2S_LRC 14              // Connecting to the LCK pin on the DAC 
  //#define I2S_MCLK 18           // Full I2S with MCLK for SPDIF/TOSLINK
  
  // --------------- IR infrared receiver ---------------
  #define recv_pin 15
#endif


// --------------- SD CARD READER (Optional) ---------------
#define SD_CS 47    // Pin CS (Chip Select) for SD card selected as SPI interface
#define SD_SCLK 45  // Pin SCK (Serial Clock) for SD card
#define SD_MISO 21  // Pin MISO (Master In Slave Out) for SD card
#define SD_MOSI 48  // pin MOSI (Master Out Slave In) for SD card

// ------ AUTO STORAGE DETECTION - SD Card / LittleFS -----
#ifdef AUTOSTORAGE
  
  #ifdef USE_SPIFFS 
    #include "SPIFFS.h"
  #endif

  #ifdef USE_LittleFS
    #include "LittleFS.h"
  #endif
  
  #include "SD.h"
  #define STORAGE (*_storage)
  bool initStorage();
  bool useSD = false;
  const char* storageTextName = nullptr;
  fs::FS* _storage = nullptr;
#else
  #ifdef USE_SD
    #include "SD.h"
    #define STORAGE SD
    #define STORAGE_BEGIN() SD.begin(SD_CS, customSPI)
    const bool useSD = true;
    #define storageTextName "SD"

  #elif defined(USE_LittleFS) 
    #include "LittleFS.h"
    #define STORAGE LittleFS
    #define STORAGE_BEGIN() LittleFS.begin(true)  // For LittleFS, it works more stable and doesn't interrupt audio
    const bool useSD = false;
    #define storageTextName "LittleFS"
  
  #elif defined(USE_SPIFFS)
    #include "SPIFFS.h" 
    #define STORAGE LittleFS
    #define STORAGE_BEGIN() SPIFFS.begin(true)  // Old file system for compatibility
    const bool useSD = false;
    #define storageTextName "SPIFFS"
  #endif 
#endif
 
// --------------- OLED Display - Pin Definition ---------------
#define SPI_MOSI_OLED 39  // MOSI (Master Out Slave In) pin for SPI OLED interface
#define SPI_MISO_OLED 0   // MISO (Master In Slave Out) pin missing for OLED display
#define SPI_SCK_OLED 38   //SCK (Serial Clock) pin for the SPI OLED interface
#define CS_OLED 42        // CS (Chip Select) pin for OLED interface
#define DC_OLED 40        // DC (Data/Command) pin for OLED interface
#define RESET_OLED 41//-1 //41     // Reset Pin for OLED Interface


// OLED display size (needed to display graphics)
#define SCREEN_WIDTH 256        // Screen width in pixels
#define SCREEN_HEIGHT 64        // Screen height in pixels

 

// --------------- ENCODER 2 (MAIN) ---------------
// If only then Volume/Stations/Banks/PowerOff/PowerOn, if with Encoder1 then Stations/Banks
#define CLK_PIN2 10             // Connection from pin 10 to CLK on the encoder
#define DT_PIN2 11              // Connection from pin 11 to DT on the left encoder
#define SW_PIN2 1               // Connection from pin 1 to SW on the left encoder (button)

// --------------- ENCODER 1 (additional) ---------------
// If defined then Volume/Mute/PowerOff/PowerOn
#ifdef twoEncoders
  #define CLK_PIN1 6              // Connection from pin 6 to CLK on the right encoder
  #define DT_PIN1 5               // Connection from pin 5 to DT on the right encoder
  #define SW_PIN1 4               // Connection from pin 4 to SW on the right encoder (button)
  const bool use2encoders = true;
  // Zmienne enkodera 1
  int CLK_state1 = 0;
  int prev_CLK_state1 = 0;
  int buttonLongPressTime1 = 150;
  int buttonSuperLongPressTime1 = 2000;
  bool action4Taken = false;        // Action Flag 4 - Encoder 1, PowerOFF
  bool action5Taken = false;        // Action Flag 5 - Encoder 1, Menu Bank
#else
  const bool use2encoders = false;
#endif


// --------------- SETTING THE TIMER FOR POWEROFF MODE --------------- 
#define WAKEUP_INTERVAL_US (60ULL * 1000000ULL) // 1 minuta

// definition of the length of the number of stations in the bank, the length of the station name in PSRAM/EEPROM, the maximum number of audio files (player)
#define MAX_STATIONS 99          // Maximum number of radio stations that can be stored in one bank
#define STATION_NAME_LENGTH 220  // Station name with bank and station number to be displayed on the first line of the screen
#define MAX_FILES 100            // Maximum number of files or directories in the directory table
#define bank_nr_max 18           // The maximum number that can be reached by the variable bank_nr, i.e. the number of banks
#define displayModeMax 6         // Limiting the maximum number of OLED display modes

// DEBUG PRINTS - ON/OFF
#define f_debug_web_on 0         // Debug print enable flag_web
#define f_debug_on 0             // Debug print enable flag

//String STATIONS_URL1 = "https://raw.githubusercontent.com/dzikakuna/ESP32_radio_streams/main/bank01.txt";   // URL to the file with the list of radio stations
#define STATIONS_URL1  "https://raw.githubusercontent.com/Roverius/EVO-Radio/refs/heads/main/ESP32_radio_streams-main/bank01.txt" // URL to the file with the list of radio stations
#define STATIONS_URL2  "https://raw.githubusercontent.com/Roverius/EVO-Radio/refs/heads/main/ESP32_radio_streams-main/bank02.txt"  // URL to the file with the list of radio stations
#define STATIONS_URL3  "https://raw.githubusercontent.com/Roverius/EVO-Radio/refs/heads/main/ESP32_radio_streams-main/bank03.txt"  // URL to the file with the list of radio stations
#define STATIONS_URL4  "https://raw.githubusercontent.com/Roverius/EVO-Radio/refs/heads/main/ESP32_radio_streams-main/bank04.txt"  // URL to the file with the list of radio stations
#define STATIONS_URL5  "https://raw.githubusercontent.com/Roverius/EVO-Radio/refs/heads/main/ESP32_radio_streams-main/bank05.txt"  // URL to the file with the list of radio stations
#define STATIONS_URL6  "https://raw.githubusercontent.com/Roverius/EVO-Radio/refs/heads/main/ESP32_radio_streams-main/bank06.txt"  // URL to the file with the list of radio stations
#define STATIONS_URL7  "https://raw.githubusercontent.com/Roverius/EVO-Radio/refs/heads/main/ESP32_radio_streams-main/bank07.txt"  // URL to the file with the list of radio stations
#define STATIONS_URL8  "https://raw.githubusercontent.com/Roverius/EVO-Radio/refs/heads/main/ESP32_radio_streams-main/bank08.txt"  // URL to the file with the list of radio stations
#define STATIONS_URL9  "https://raw.githubusercontent.com/Roverius/EVO-Radio/refs/heads/main/ESP32_radio_streams-main/bank09.txt"  // URL to the file with the list of radio stations
#define STATIONS_URL10 "https://raw.githubusercontent.com/Roverius/EVO-Radio/refs/heads/main/ESP32_radio_streams-main/bank10.txt"  // URL to the file with the list of radio stations
#define STATIONS_URL11 "https://raw.githubusercontent.com/Roverius/EVO-Radio/refs/heads/main/ESP32_radio_streams-main/bank11.txt"  // URL to the file with the list of radio stations
#define STATIONS_URL12 "https://raw.githubusercontent.com/Roverius/EVO-Radio/refs/heads/main/ESP32_radio_streams-main/bank12.txt"  // URL to the file with the list of radio stations
#define STATIONS_URL13 "https://raw.githubusercontent.com/Roverius/EVO-Radio/refs/heads/main/ESP32_radio_streams-main/bank13.txt"  // URL to the file with the list of radio stations
#define STATIONS_URL14 "https://raw.githubusercontent.com/Roverius/EVO-Radio/refs/heads/main/ESP32_radio_streams-main/bank14.txt"  // URL to the file with the list of radio stations
#define STATIONS_URL15 "https://raw.githubusercontent.com/Roverius/EVO-Radio/refs/heads/main/ESP32_radio_streams-main/bank15.txt"  // URL to the file with the list of radio stations
#define STATIONS_URL16 "https://raw.githubusercontent.com/Roverius/EVO-Radio/refs/heads/main/ESP32_radio_streams-main/bank16.txt"  // URL to the file with the list of radio stations
#define STATIONS_URL17 "https://raw.githubusercontent.com/Roverius/EVO-Radio/refs/heads/main/ESP32_radio_streams-main/bank17.txt"  // URL to the file with the list of radio stations
#define STATIONS_URL18 "https://raw.githubusercontent.com/Roverius/EVO-Radio/refs/heads/main/ESP32_radio_streams-main/bank18.txt"  // URL to the file with the list of radio stations



// --------------- TEXT DEFINITION ---------------
#define SLEEP_STRING "SLEEP "
#define SLEEP_STRING_VAL " MINUTES"
#define SLEEP_STRING_OFF "OFF"

#ifdef LANG_NL
static const char* months[] = {
    "JANUARI",
    "FEBRUARI",
    "MAART",
    "APRIL",
    "MEI",
    "JUNI",
    "JULI",
    "AUGUSTUS",
    "SEPTEMBER",
    "OKTOBER",
    "NOVEMBER",
    "DECEMBER"
};
#endif

// --------------- RADIO MODULE COMMUNICATION VIA RS232 ---------------
#ifdef SERIALCOM
  #include "serialcom.h"   // Integration as a module with RS232 communication
#endif


// --------------- Variables FOR IR REMOTE CONTROL in NEC standard - moved to txt file ---------------
uint16_t rcCmdVolumeUp = 0;   // Loudness +
uint16_t rcCmdVolumeDown = 0; // Loudness -
uint16_t rcCmdArrowRight = 0; // right arrow - next station
uint16_t rcCmdArrowLeft = 0;  // left arrow - previous station
uint16_t rcCmdArrowUp = 0;    // up arrow - list of stations one step up
uint16_t rcCmdArrowDown = 0;  // down arrow - list of stations step down
uint16_t rcCmdBack = 0;	      // Back button
uint16_t rcCmdOk = 0;         // Enter button - confirm station
uint16_t rcCmdSrc = 0;        // Switching the radio or player source
uint16_t rcCmdMute = 0;       // Sound mute
uint16_t rcCmdAud = 0;        // Sound equalizer
uint16_t rcCmdDirect = 0;     // Screen brightness, two modes 1/16 or full brightness    
uint16_t rcCmdBankMinus = 0;  // Display bank selection
uint16_t rcCmdBankPlus = 0;   // Display bank selection
uint16_t rcCmdRed = 0;        // Toggles SD card bank loading - GitHub server in the bank menu
uint16_t rcCmdGreen = 0;      // VU off, VU mode 1, VU mode 2, clock
uint16_t rcCmdKey0 = 0;       // Button "0"
uint16_t rcCmdKey1 = 0;       // Button "1"
uint16_t rcCmdKey2 = 0;       // Button "2"
uint16_t rcCmdKey3 = 0;       // Button "3"
uint16_t rcCmdKey4 = 0;       // Button "4"
uint16_t rcCmdKey5 = 0;       // Button "5"
uint16_t rcCmdKey6 = 0;       // Button "6"
uint16_t rcCmdKey7 = 0;       // Button "7"
uint16_t rcCmdKey8 = 0;       // Button "8"
uint16_t rcCmdKey9 = 0;       // Button "9"
uint16_t rcCmdPower = 0;       // Button Power
// End of IR Remote Definition

bool  f_callInfo = 0;       // Flag or "INFO" also on OLED
bool  f_logo = 0;           // Flag or do we display the logo?
bool  f_simpleMode3 =0;

int currentSelection = 0;                     // Current selection number on OLED screen
int firstVisibleLine = 0;                     // The number of the first visible line on the OLED screen
uint8_t station_nr = 0;                       // The number of the currently selected radio station from the list
int stationFromBuffer = 0;                    // Radio station number stored in the buffer to be restored to the screen after inactivity
uint8_t bank_nr;                              // The number of the currently selected station bank from the list
uint8_t previous_bank_nr = 0;                 // Bank number before entering the bank change menu
//int bankFromBuffer = 0;                       // The number of the currently selected station bank from the list to be restored to the screen after inactivity.

//Enkoder 2 (podstawowy)
int CLK_state2;                               // Current CLK state of the left encoder
int prev_CLK_state2;                          // Left encoder CLK previous state
int stationsCount = 0;                        // Current number of stored stations in the table
int buttonLongPressTime2 = 2000;              // Encoder 2 long press response time
int buttonShortPressTime2 = 500;              // Response time to short press of encoder 2
int buttonSuperLongPressTime2 = 3000;         // Encoder 2 super long press response time

uint8_t volumeValue = 10;                     // Volume value, default set to 10
uint8_t maxVolume = 21;
bool maxVolumeExt =  false;                   // 0(false) -  zakres standardowy Volume 1-21 , 1 (true) - zakres rozszerzony 0-42
uint8_t volumeBufferValue = 0;                // Volume value, default set to 10
int maxVisibleLines = 4;                      // Maximum number of visible lines on an OLED screen
int bitrateStringInt = 0;                     // Declaration of a variable to convert Bitrate string to Int value to divide bitrate by 1000
int SampleRate = 0;
int SampleRateRest = 0;
int audioBufferTime = 0;                      // A variable that holds information about the number of seconds of music in the audio buffer.

uint8_t stationNameLenghtCut = 24;            // 24-> 25 characters, 25-> 26 characters, a variable defining the length of the station name in the Bank files, counted from 0 - the set value
const uint8_t yPositionDisplayScrollerMode0 = 33;   // The height(s) of the scrolling/fixed station text display in a given mode.
const uint8_t yPositionDisplayScrollerMode1 = 61;   // The height(s) of the scrolling/fixed station text display in a given mode.
const uint8_t yPositionDisplayScrollerMode2 = 25;   // The height(s) of the scrolling/fixed station text display in a given mode.
const uint8_t yPositionDisplayScrollerMode3 = 49;   // The height(s) of the scrolling/fixed station text display in a given mode.
const uint8_t yPositionDisplayScrollerMode5 = 35;   // The height(s) of the scrolling/fixed station text display in a given mode.
const uint8_t stationNamePositionYmode3 = 31;
const uint8_t stationNamePositionYmode5 = 26;
uint16_t stationStringScrollLength = 0;
uint8_t maxStationVisibleStringScrollLength = 46;
bool stationNameFromStream = 0;               // A flag defining whether the station name is displayed from the Memory Bank files or from the information transmitted in the stream.
bool f_powerOff = 0;                          // Active power off mode flag (ESP light sleep)
bool f_sleepTimerOn = false;
uint8_t sleepTimerValueSet = 0;           // Variable storing the set time - currently not used
uint8_t sleepTimerValueCounter = 0;         // Sleep timer counter decremented by Timer3 every 1 minute
uint8_t sleepTimerValueStep = 15;           // Sleep timer step by how much the timer increases
uint8_t sleepTimerValueMax = 90;            // The maximum sleep timer value that can be set
//uint8_t bank_nr_max = 16;


// ---- Voice playback of time every hour ---- //
bool f_requestVoiceTimePlay = false; 
bool timeVoiceInfoEveryHour = true;
unsigned long voiceTimeMillis = 0;
uint16_t voiceTimeReturnTime = 4000;


// ---- Auto dimmer / auto display dimming ---- //
uint8_t displayDimmerTimeCounter = 0;  // Variable incremented in timer2 interrupt to dim the display
uint8_t dimmerDisplayBrightness = 10;   // Display dimming value after inactivity time
uint8_t dimmerSleepDisplayBrightness = 1; // Display dimming value after inactivity time
uint8_t displayBrightness = 180;        // Default maximum display brightness
uint16_t displayAutoDimmerTime = 5;   // Time after which the display will dim, counted in seconds
bool displayAutoDimmerOn = false;       // Automatic display dimming, enabled by default
bool displayDimmerActive = false;       // Dim mode active
uint16_t displayPowerSaveTime = 30;    // Time after which the OLED display will turn off (power save mode)
uint16_t displayPowerSaveTimeCounter = 0;    // OLED display Power Save mode timer
bool displayPowerSaveEnabled = false;   // Flag indicating whether OLED display shutdown mode is enabled
uint8_t displayMode = 0;               // Display mode 0-displayRadio with scrolling "scroller" / 1-Clock / 2- mode with 3 fixed lines of station text

// ---- Equalizer ---- //
int8_t toneLowValue = 0;               // Bass filter value
int8_t toneMidValue = 0;               // Mid-tone filter value
int8_t toneHiValue = 0;                // Treble filter value
int8_t balanceValue = 0;                // Balance value
uint8_t toneSelect = 1;                // A variable that determines which equalizer filter we are adjusting, starting from the first one

bool equalizerMenuEnable = false;      // Equalizer menu display flag

// ------ Entering numbers from the remote control/keyboard ------ //
uint8_t rcInputDigit1 = 0xFF;      // The first digit when entering the station number from the remote control
uint8_t rcInputDigit2 = 0xFF;      // The second digit when entering the station number from the remote control
bool f_Presets = false;     // A flag that determines whether we switch stations, enter a station number, or 1-10 as favorite stations.
bool f_Presets2 = false;
bool f_Presets3 = false;



// ---- Configuration variables ---- //
#define CONFIG_COUNT 26
uint16_t configArray[CONFIG_COUNT] = {0};
uint8_t rcPage = 0;
#define CONFIGREMOTE_COUNT 26
uint16_t configRemoteArray[CONFIGREMOTE_COUNT] = {0};   // Table storing remote control codes when reading from a file

/*
struct RemoteMap
{
  const char* name;
  //uint8_t index;
};
RemoteMap remoteMap[CONFIGREMOTE_COUNT] = {
    {"cmdVolumeUp"},
    {"cmdVolumeDown"},
    {"cmdArrowRight"},
    {"cmdArrowLeft"},
    {"cmdArrowUp"},
    {"cmdArrowDown"},
    {"cmdBack"},
    {"cmdOk"},
    {"cmdSrc"},
    {"cmdMute"},
    {"cmdAud"},
    {"cmdDirect"},
    {"cmdBankMinus"},
    {"cmdBankPlus"},
    {"cmdRed"},
    {"cmdGreen"},
    {"cmdKey0"},
    {"cmdKey1"},
    {"cmdKey2"},
    {"cmdKey3"},
    {"cmdKey4"},
    {"cmdKey5"},
    {"cmdKey6"},
    {"cmdKey7"},
    {"cmdKey8"},
    {"cmdKey9"}
};

*/

uint16_t configAdcArray[24] = {0};      // An array storing ADC values ​​for the keyboard buttons.
bool configExist = true;                  // Flag indicating whether a configuration file exists.
bool f_displayPowerOffClock = false;      // A flag that determines whether the clock should be displayed in sleep mode.
bool f_sleepAfterPowerFail = false;       // Flag whether we go to powerOFF after power is restored
bool f_saveVolumeStationAlways = false;   // A flag that determines whether stations, banks and volume levels are saved always or only when power is OFF.
//bool f_statusLED = false;
bool f_powerOffAnimation = false;             // Animation when power OFF
bool remoteConfig = false;

bool encoderButton2 = false;      // Flag indicating whether encoder button 2 has been pressed
bool encoderFunctionOrder = false; // Flag specifying the order of encoder 2 functions
bool displayActive = false;       // Flag indicating whether the display is active

bool mp3 = false;                 // Flag indicating whether the current audio file is in the format MP3
bool flac = false;                // Flag indicating whether the current audio file is in the format FLAC
bool aac = false;                 // Flag indicating whether the current audio file is in the format AAC
bool vorbis = false;              // Flag indicating whether the current audio file is in the format VORBIS
bool opus = false;
bool timeDisplay = true;          // Flag specifying when to show the time on the display, by default immediately after start
bool listedStations = false;      // A flag that determines whether a list of stations to choose from is shown on the screen.
bool bankMenuEnable = false;      // Flag indicating whether the bank selection menu is displayed on the screen
bool bitratePresent = false;      // A flag indicating whether bitrate information appeared on the terminal serial - as the last data flowing from info
bool bankNetworkUpdate = false;   // Flag to select bank update from network or SD card - True update from internet
bool volumeMute = false;          // Flag specifying the state of the mute function - Mute
bool volumeSet = false;           // Decoder 2 volume control menu input flag

bool action3Taken = false;        // Action Flag 3

bool ActionNeedUpdateTime = false;// A variable that specifies the need for displayRadio to read the time update.
bool debugAudioBuffor = false;    // Displaying the Audio Buffer
bool f_audioInfoRefreshStationString = false;    // Flag forcing a required refresh due to a change in the info stream - stationstring, title, 
bool f_audioInfoRefreshDisplayRadio = false;    // Flag forcing required de-simplification due to stream info change - line kbps, khz, stream format
bool resumePlay = false;            // Flag to require playback to start after the voice message ends
bool fwupd = false;               // Flag blocking the main loop during software update
bool configIrExist = false;       // A flag informing about the existence of a correct IR remote control configuration.
bool wsAudioRefresh = false;      // Flag informing about the need to refresh Station Text via Web Socket
bool f_voiceTimeBlocked = false;
bool f_displaySleepTime = false;    // Flag for displaying the timer menu
bool f_displaySleepTimeSet = false;    // Flag for displaying the sleep menu

bool noSDcard = false;              // flag set when no SD card is detected
bool noStorage = false;             // flag set when there is a problem with SPIFFS/LittleFS memory

unsigned long debounceDelay = 300;    // Debouncing duration in milliseconds
unsigned long displayTimeout = 3000;  // Time the message is displayed on the screen in milliseconds
unsigned long displayStartTime = 0;   // Message display start time
unsigned long seconds = 0;            // Timer seconds counter
unsigned int PSRAM_lenght = MAX_STATIONS * (STATION_NAME_LENGTH) + MAX_STATIONS; // PSRAM memory length declaration
unsigned long lastCheckTime = 0;      // No stream audio blink
//uint8_t stationNameStreamWidth = 0;   // Full Station Name Test
uint64_t seconds2nextMinute;
uint64_t micros2nextMinute;

// ---- VU Meter ---- //
unsigned long vuMeterTime;                 // VU display refresh delay time in milliseconds
uint8_t vuMeterL;                          // VU value for L channel range 0-255
uint8_t vuMeterR;                          // VU value for R channel range 0-255
uint8_t peakL = 0;                         // PeakHold value for the left channel
uint8_t peakR = 0;                         // PeakHold value for the right channel
uint8_t peakHoldTimeL = 0;                 // PeakHold counter for the left channel
uint8_t peakHoldTimeR = 0;                 // PeakHold counter for the right channel
const uint8_t peakHoldThreshold = 5;       // Number of cycles before the peak drops
const uint8_t vuLy = 41;                   // Y coordinate of the L-left VU indicator (above)
const uint8_t vuRy = 47;                   // Y coordinate of the R-right VU indicator (lower)
const uint8_t vuLyMode3 = 54;              // Y coordinate of the L-left VU indicator (above)
const uint8_t vuRyMode3 = 60;              // Y coordinate of the R-right VU indicator (below)
const uint8_t vuLyMode5 = 43;                   // Y coordinate of the L-left VU indicator (above)
const uint8_t vuRyMode5 = 49;                   // Y coordinate of the R-right VU indicator (lower)
const uint8_t vuThicknessMode3 = 1;        // VU meter line thickness in Mode3
const int vuCenterXmode3 = 128;            // Mode 3 VU Start Center Position
const int vuYmode3 = 62;                   // VU height (Y) position in Mode 3
bool vuPeakHoldOn = 1;                     // Flag indicating whether the Peak & Hold feature on the VUmeter is enabled
bool vuMeterOn = true;                     // Flag to enable VU meters
bool vuMeterMode = false;                  // vuMeter drawing mode
uint8_t displayVuL = 0;                    // VU value to be displayed after the smooth process
uint8_t displayVuR = 0;                    // VU value to be displayed after the smooth process
uint8_t vuRiseSpeed = 24;                  // VU slew rate
uint8_t vuFallSpeed = 6;                   // VU fall rate
bool vuSmooth = 1;                         // Smooth enable flag (default enabled=1)
uint8_t vuRiseNeedleSpeed = 2;                  // VU slew rate
uint8_t vuFallNeedleSpeed = 6;                   // VU fall rate
bool reversColorMode4 = true;

// Volume Fade
unsigned long lastFadeTime = 0;
const unsigned long fadeStepDelay = 20; // ms
bool fadingIn = false;
uint8_t targetVolume = 0;
uint8_t currentVolume = 0;
bool f_volumeFadeOn = 0;
uint8_t volumeFadeOutTime = 20; //ms * volumeValue
uint8_t volumeSleepFadeOutTime = 50;

unsigned long scrollingStationStringTime;  // Time to refresh scorling
uint8_t scrollingRefresh = 50;              // Scroller text scrolling time in ms

uint8_t vuMeterRefreshCounterSet = 0;      // Multiplier every loopRefreshTime loop is to be refreshed VU Meter
uint8_t scrollerRefreshCounterSet = 0;     // Denominator: after how many loops loopRefreshTime should the scroller be refreshed and scrolled by 1px

uint16_t stationStringScrollWidth;         // width of the station name string in the Scroller function
uint16_t xPositionStationString = 0;       // Starting position for scrolling StationString text
uint16_t offset;                           // Changing the offset for the Scroll function - scrolling the stream title on the OLED screen
unsigned char * psramData;                 // variable to hold station data in PSRAM

unsigned long vuMeterMilisTimeUpdate;           // Variable storing the time for the millis VU Meter refresh function
uint8_t vuMeterRefreshTime = 50;                // VUmeter refresh time in ms
uint8_t wifiSignalLevel = 0;
uint8_t wifiSignalLevelLast = 7;          // SignalLevel Previous State Memory

// ---- Serwer Web ---- //
unsigned long currentTime = millis();
unsigned long previousTime = 0;
const long timeoutTime = 2000;
bool urlToPlay = false;
bool urlPlaying = false;


// ---- Checking the remote control function, variables for measuring the time difference ---- //
unsigned long runTime = 0;
unsigned long runTime1 = 0;
unsigned long runTime2 = 0;


String stationStringScroll = "";     // A variable that stores the text to be scrolled on the screen.
String stationName;                  // Name of the currently selected radio station
String stationString;                // Additional radio station data (if any)
String stationStringWeb;                // Additional radio station data (if any)
String bitrateString;                // A variable that stores information about bitrate
String sampleRateString;             // A variable that stores information about sample rate
String bitsPerSampleString;          // A variable that stores information about the number of bits per sample.
String currentIP;
String stationNameStream;           // The station name is extracted from the data sent by the stream.
String streamCodec;
String stationLogoUrl;
String timezone;


String header;                      // Variable for the web server
String sliderValue = "0";
String url2play = "";


File myFile;  // File handle

//U8G2_SSD1322_NHD_256X64_F_4W_HW_SPI u8g2(U8G2_R2, /* cs=*/CS_OLED, /* dc=*/DC_OLED, /* reset=*/RESET_OLED);  // Hardware SPI 3.12inch OLED
//U8G2_SSD1309_128X64_NONAME0_F_4W_HW_SPI u8g2(U8G2_R2, /* cs=*/CS_OLED, /* dc=*/DC_OLED, /* reset=*/RESET_OLED);  
//U8G2_SH1122_256X64_F_4W_HW_SPI u8g2(U8G2_R2, /* cs=*/ CS_OLED, /* dc=*/ DC_OLED, /* reset=*/ RESET_OLED);		// Hardware SPI  2.08inch OLED
//U8G2_SSD1363_256X128_F_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/CS_OLED, /* dc=*/DC_OLED, /* reset=*/RESET_OLED);  // Hardware SPI 3.12inch OLED

#if defined(SSD1322)
  U8G2_SSD1322_NHD_256X64_F_4W_HW_SPI u8g2(U8G2_R2, CS_OLED, DC_OLED, RESET_OLED);
#elif defined(SSD1309)
  U8G2_SSD1309_128X64_NONAME0_F_4W_HW_SPI u8g2(U8G2_R2, CS_OLED, DC_OLED, RESET_OLED);
#elif defined(SH1122)
  U8G2_SH1122_256X64_F_4W_HW_SPI u8g2(U8G2_R2, CS_OLED, DC_OLED, RESET_OLED);
#elif defined(SSD1363) 
  U8G2_SSD1363_256X128_F_4W_HW_SPI u8g2(U8G2_R0, CS_OLED, DC_OLED, RESET_OLED);
#else
  #error "No display type selected!"
#endif


// We assign the www server port
AsyncWebServer server(80);
AsyncWebSocket ws("/ws"); //Websocket support starts


// Initializing WiFiManager
WiFiManager wifiManager;


// Configuring a new SPI with selected pins for the SD card reader
SPIClass customSPI = SPIClass(HSPI);  // We use HSPI, but with our own pins

#ifdef twoEncoders
  ezButton button1(SW_PIN1);  // Creating a button object from the ezButton encoder 2
#endif

ezButton button2(SW_PIN2);  // Creating a button object from the ezButton encoder 2
Audio audio;                // An object for handling sound and audio-related functions

Ticker timer1;             // Timer to update the Clock
Ticker timer2;             // Dimmer function timer
Ticker timer3;             // Sleep timer
Ticker timer4;             // LED flashing start timer
WiFiClient client;         // Object for handling WiFi connection for HTTP client


// ------------------- HTML pages embedded in the code -------------------
const char stylehead_html[] PROGMEM = R"rawliteral(
  <!DOCTYPE HTML><html>
  <head>
    <link rel='icon' href='/favicon.ico' type='image/x-icon'>
    <link rel="shortcut icon" href="/favicon.ico" type="image/x-icon">
    <link rel="apple-touch-icon" sizes="180x180" href="/icon.png">
    <link rel="icon" type="image/png" sizes="192x192" href="/icon.png">

    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>Evo Web Radio</title>
    <style>
      html {font-family: Arial; display: inline-block; text-align: center;}
      h2 {font-size: 1.3rem;}
      p {font-size: 0.95rem;}
      table {border: 1px solid black; border-collapse: collapse; margin: 0px 0px;}
      td, th {font-size: 0.8rem; border: 1px solid gray; border-collapse: collapse;}
      td:hover {font-weight:bold;}
      a {color: black; text-decoration: none;}
      body {max-width: 1380px; margin:0px auto; padding-bottom: 25px; background: #D0D0D0;}
      .slider {-webkit-appearance: none; margin: 14px; width: 330px; height: 10px; background: #4CAF50; outline: none; -webkit-transition: .2s; transition: opacity .2s; border-radius: 5px;}
      .slider::-webkit-slider-thumb {-webkit-appearance: none; appearance: none; width: 35px; height: 25px; background: #4a4a4a; cursor: pointer; border-radius: 5px;}
      .slider::-moz-range-thumb { width: 35px; height: 35px; background: #4a4a4a; cursor: pointer; border-radius: 5px;} 
      .button { background-color: #4CAF50; border: 1; color: white; padding: 10px 20px; border-radius: 5px;}
      .buttonBank { background-color: #4CAF50; border: 1; color: white; padding: 8px 8px; border-radius: 5px; width: 35px; height: 35px; margin: 0 1.5px;}
      .buttonBankSelected { background-color: #505050; border: 1; color: white; padding: 8px 8px; border-radius: 5px; width: 35px; height: 35px; margin: 0 1.5px;}
      .buttonBank:active {background-color: #4a4a4a; box-shadow: 0 4px #666; transform: translateY(2px);}
      .buttonBank:hover {background-color: #4a4a4a;}
      .button:hover {background-color: #4a4a4a;}
      .button:active {background-color: #4a4a4a; box-shadow: 0 4px #666; transform: translateY(2px);}
      .column {padding: 5px; display: flex; justify-content: space-between;}
      .columnlist {padding: 10px; display: flex; justify-content: center;}
      .stationList {text-align:left; margin-top: 0px; width: 280px; margin-bottom:0px;cursor: pointer;}
      .stationNumberList {text-align:center; margin-top: 0px; width: 35px; margin-bottom:0px;}
      .stationListSelected {text-align:left; margin-top: 0px; width: 280px; margin-bottom:0px;cursor: pointer; background-color: #4CAF50;}
      .stationNumberListSelected {text-align:center; margin-top: 0px; width: 35px; margin-bottom:0px; background-color: #4CAF50;}
      .station-name {}  
    </style>
  </head>
)rawliteral";

const char index_html[] PROGMEM = R"rawliteral(
  <!DOCTYPE HTML><html>
  <head>
    <link rel='icon' href='/favicon.ico' type='image/x-icon'>
    <link rel="shortcut icon" href="/favicon.ico" type="image/x-icon">
    <link rel="apple-touch-icon" sizes="180x180" href="/icon.png">
    <link rel="icon" type="image/png" sizes="192x192" href="/icon.png">

    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>Evo Web Radio</title>
    <style>
      html {font-family: Arial; display: inline-block; text-align: center;}
      h2 {font-size: 1.3rem;}
      p {font-size: 0.95rem;}
      table {border: 1px solid black; border-collapse: collapse; margin: 0px 0px;}
      td, th {font-size: 0.8rem; border: 1px solid gray; border-collapse: collapse;}
      td:hover {font-weight:bold;}
      a {color: black; text-decoration: none;}
      body {max-width: 1380px; margin:0px auto; padding-bottom: 25px; background: #D0D0D0;}
      .slider {-webkit-appearance: none; margin: 14px; width: 330px; height: 10px; background: #4CAF50; outline: none; -webkit-transition: .2s; transition: opacity .2s; border-radius: 5px;}
      .slider::-webkit-slider-thumb {-webkit-appearance: none; appearance: none; width: 35px; height: 25px; background: #4a4a4a; cursor: pointer; border-radius: 5px;}
      .slider::-moz-range-thumb { width: 35px; height: 35px; background: #4a4a4a; cursor: pointer; border-radius: 5px;} 
      .button { background-color: #4CAF50; border: 1; color: white; padding: 10px 20px; border-radius: 5px;}
      .buttonBank { background-color: #4CAF50; border: 1; color: white; padding: 8px 8px; border-radius: 5px; width: 35px; height: 35px; margin: 0 1.5px;}
      .buttonBankSelected { background-color: #505050; border: 1; color: white; padding: 8px 8px; border-radius: 5px; width: 35px; height: 35px; margin: 0 1.5px;}
      .buttonBank:active {background-color: #4a4a4a box-shadow: 0 4px #666; transform: translateY(2px);}
      .buttonBank:hover {background-color: #4a4a4a;}
      .button:hover {background-color: #4a4a4a;}
      .button:active {background-color: #4a4a4a; box-shadow: 0 4px #666; transform: translateY(2px);}
      .column { align: center; padding: 5px; display: flex; justify-content: space-between;}
      .columnlist { align: center; padding: 10px; display: flex; justify-content: center;}
      .stationList {text-align:left; margin-top: 2px; width: 280px; height:18px; margin-bottom:0px;cursor: pointer;}
      .stationNumberList {text-align:center; margin-top: 2px; width: 35px; height:18px; margin-bottom:0px;}
      .stationListSelected {text-align:left; margin-top: 2px; width: 280px; height:18px; margin-bottom:0px;cursor: pointer; background-color: #4CAF50;}
      .stationNumberListSelected {text-align:center; margin-top: 2px; width: 35px; height:18px; margin-bottom:0px; background-color: #4CAF50;} 
      .wifi-wrapper {display: flex; align-items: flex-end; gap: 1px; height: 13px; margin-top: -3px; margin-bottom: 3px; justify-content: center; }
      .wifiBar {width: 2px;background-color: #555; border-radius: 2px; transition: background-color 0.2s;}
      .bar1 { height: 2px; }
      .bar2 { height: 4px; }
      .bar3 { height: 6px; }
      .bar4 { height: 8px; }
      .bar5 { height: 10px; }
      .bar6 { height: 12px; }
    </style>
  </head>

  <body>
    <h2>Evo Web Radio</h2>
  
    <div id="display" style="display: inline-block; padding: 6px; border: 2px solid #4CAF50; border-radius: 15px; background-color: #4a4a4a; font-size: 1.45rem; 
      color: #AAA; width: 345px; text-align: center; white-space: nowrap; box-shadow: 0 0 20px #4CAF50;" onClick="flashBackground(); displayMode()">
      
      <div style="margin-bottom: 10px; font-weight: bold; overflow: hidden; text-overflow: ellipsis; -webkit-text-stroke: 0.3px black; text-stroke: 0.3px black;">
        <span id="textStationName"><b>STATIONNAME</b></span>
      </div>
      
      <div style="width: 345px; margin-bottom: 10px;">
        <div id="stationTextDiv" style="display: block; text-overflow: ellipsis; white-space: normal; font-size: 1.0rem; color: #999; margin-bottom: 10px; text-align: center; align-items: center; height: 4.2em; justify-content: center; overflow: hidden; line-height: 1.4em;">
          <span id="stationText">STATIONTEXT</span>
        </div>
        <div id="muteIndicatorDiv" style="text-align:center; color:#4CAF50; font-weight:bold; font-size:1.0rem; height:1.4em; margin-top:-5px;">
        <span id="muteIndicator"></span>
        </div>
      </div>
      
      <div style="height: 1px; background-color: #4CAF50; margin: 5px 0;"></div>

      
      <div style="display: flex; justify-content: center; gap: 13px; font-size: 1.00rem; color: #999;">
        <div><span id="bankValue">Bank: --</span></div>
          <div style="display: flex; justify-content: center; gap: 6px; font-size: 0.55rem; color: #999;margin: 4px 0;">
            <div><span id="samplerate">---.-kHz</span></div>
            <div><span id="bitrate">---kbps</span></div>
            <div><span id="bitpersample">---bit</span></div>
            <div><span id="streamformat">----</span></div>
            WiFi: <div class="wifi-wrapper">
            <div class="wifiBar bar1" id="wifiBar1"></div>
            <div class="wifiBar bar2" id="wifiBar2"></div>
            <div class="wifiBar bar3" id="wifiBar3"></div>
            <div class="wifiBar bar4" id="wifiBar4"></div>
            <div class="wifiBar bar5" id="wifiBar5"></div>
            <div class="wifiBar bar6" id="wifiBar6"></div>
  	      </div>
        </div>
        <div><span id="stationNumber">Station: --</span></div>
      </div>

  </div>
  <br>
  <br>
  <script>

  var websocket;

  function updateSliderVolume(element) 
  {
    var sliderValue = document.getElementById("volumeSlider").value;
    document.getElementById("textSliderValue").innerText = sliderValue;

    if (websocket && websocket.readyState === WebSocket.OPEN) 
    {
      websocket.send("volume:" + sliderValue);
    } 
    else 
    {
      console.warn("WebSocket not connected");
    }
  }

  function changeStation(number) 
  {
    if (websocket.readyState === WebSocket.OPEN) 
    {
      websocket.send("station:" + number);
    } 
    else 
    {
      console.log("WebSocket is not open");
    }
  }

  function changeBank(number) 
  {
    if (websocket.readyState === WebSocket.OPEN) 
    {
      websocket.send("bank:" + number);
    } 
    else 
    {
      console.log("WebSocket is not open");
    }
  }


  function displayMode() 
  {
    //fetch("/displayMode")
    if (websocket.readyState === WebSocket.OPEN) 
    {
      websocket.send("displayMode");
    } 
    else 
    {
      console.log("WebSocket nie jest otwarty");
    }
  }
  
  function setWifi(level) 
  {
    for (let i = 1; i <= 6; i++) 
    {
      const bar = document.getElementById("wifiBar" + i);
      if (bar) {bar.style.backgroundColor = (i <= level) ? "#4CAF50" : "#555";}
    }
  }


  function connectWebSocket() 
  {
    websocket = new WebSocket('ws://' + window.location.hostname + '/ws');
    websocket.onopen = function () 
    {
      console.log("WebSocket polaczony");
    };
    
    websocket.onclose = function (event) 
    {
      console.log("WebSocket zamkniety. Proba ponownego polaczenia za 3 sekundy...");
      setTimeout(connectWebSocket, 3000); // próba ponownego połączenia
    };

    websocket.onerror = function (error) 
    {
      console.error("Blad WebSocket: ", error);
      websocket.close(); // zamyka połączenie, by wywołać reconnect
    };
    
    websocket.onmessage = function(event) 
    {
      if (event.data === "reload") 
      {
        location.reload();
      }  
      
      if (event.data.startsWith("volume:")) 
      {
        var vol = parseInt(event.data.split(":")[1]);
        document.getElementById("volumeSlider").value = vol;
        document.getElementById("textSliderValue").innerText = vol;
      }
      
      if (event.data.startsWith("station:")) 
      {
        var station = parseInt(event.data.split(":")[1]);
        highlightStation(station);
        //document.getElementById('stationNumber').innerText = event.data.split(':')[1];
        document.getElementById('stationNumber').innerText ='Station: ' + station; 
      }
      
      if (event.data.startsWith("stationname:")) 
      {
        var value = event.data.split(":")[1];
        document.getElementById("textStationName").innerHTML = `<b>${value}</b>`;
        //checkStationTextLength();
      }  

      if (event.data.startsWith("stationtext$")) 
      {
        //var stationtext = event.data.split("$")[1];
        //document.getElementById("stationText").innerHTML = `${stationtext}`;
        
        var stationtext = event.data.substring(event.data.indexOf("$") + 1);
        //document.getElementById("stationText").innerHTML = stationtext;       
        document.getElementById("stationText").textContent = stationtext;
      }  

      if (event.data.startsWith("bank:")) 
      {
        var bankValue = parseInt(event.data.split(":")[1]);
        document.getElementById('bankValue').innerText = 'Bank: ' + bankValue;
      } 

      //if (event.data.startsWith("samplerate:")) 
      //{
      //  var samplerate = parseInt(event.data.split(":")[1]);
      //  var formattedRate = (samplerate / 1000).toFixed(1) + "kHz";
      //  document.getElementById('samplerate').innerText = formattedRate;
      //}
      
      if (event.data.startsWith("samplerate:")) 
      {
        var raw = event.data.split(":")[1];
        var samplerate = parseInt(raw);

        if (isNaN(samplerate)) 
        {
          document.getElementById('samplerate').innerText = '---.-kHz';
        } 
        else 
        {
          var formattedRate = (samplerate / 1000).toFixed(1) + "kHz";
          document.getElementById('samplerate').innerText = formattedRate;
        }
      }

      //if (event.data.startsWith("bitrate:")) 
      //{	
      //  var bitrate = parseInt(event.data.split(":")[1]);
      //  document.getElementById('bitrate').innerText = bitrate + 'kbps';
      //}

      if (event.data.startsWith("bitrate:")) 
      {	
        var raw = event.data.split(":")[1];
        var bitrate = parseInt(raw);

        if (isNaN(bitrate)) 
        {
          document.getElementById('bitrate').innerText = '---kbps';
        } 
        else 
        {
          document.getElementById('bitrate').innerText = bitrate + 'kbps';
        }
      }

      
      //if (event.data.startsWith("bitpersample:")) 
      //{	
      //  var bitpersample = parseInt(event.data.split(":")[1]);
      //  document.getElementById('bitpersample').innerText = bitpersample + 'bit';
      //}

      if (event.data.startsWith("bitpersample:")) 
      {	
        var raw = event.data.split(":")[1];
        var bitpersample = parseInt(raw);

        if (isNaN(bitpersample)) 
        {
          document.getElementById('bitpersample').innerText = '---bit';
        } 
        else 
        {
          document.getElementById('bitpersample').innerText = bitpersample + 'bit';
        }
      }

      
      if (event.data.startsWith("streamformat:")) 
      {	
        var streamformat = event.data.split(":")[1];
        document.getElementById('streamformat').innerText = streamformat;
      }  

      if (event.data.startsWith("mute:")) 
      {
        //var muteInd = parseInt(event.data.split(":")[1]);
        const muteInd = event.data.split(":")[1] === "1" ? 1 : 0;
        document.getElementById('muteIndicator').textContent = (muteInd === 1) ? 'MUTED' : '';       
      }

      if (event.data.startsWith("wifi:")) 
      {
        const level = parseInt(event.data.split(":")[1]);
        if (!isNaN(level)) {setWifi(Math.min(Math.max(level, 1), 6));}
      }


    }

  };
  
  function highlightStation(stationId) 
  {
    // Remove previous selections
    document.querySelectorAll(".stationList").forEach(el => {
    el.classList.remove("stationListSelected");
    //el.innerHTML = el.dataset.stationName || el.innerText; // restore the original number
    el.innerText = el.dataset.stationName || el.innerText;


    });

    document.querySelectorAll(".stationNumberList").forEach(el => {
      el.classList.remove("stationNumberListSelected");
      el.innerHTML = el.dataset.stationNumber || el.innerText; // restore the original number
    });

    // Select a new station
    const numCells = document.querySelectorAll(".stationNumberList");
    const stationCells = document.querySelectorAll(".stationList");

    const numCell = numCells[stationId - 1];
    const stationCell = stationCells[stationId - 1];

    if (numCell && stationCell) 
    {
      numCell.classList.add("stationNumberListSelected");
      stationCell.classList.add("stationListSelected");
      
      // Bold the station name
      stationCell.dataset.stationName = stationCell.innerText; // save the original text
      stationCell.innerHTML = `<b>${stationCell.innerText}</b>`; // bold station name

      //const original = stationCell.dataset.originalName || stationCell.innerText;
      //stationCell.dataset.originalName = original;
      //stationCell.innerText = original;
      //stationCell.style.fontWeight = "bold";

        
      // Bold the numberr
      numCell.dataset.stationNumber = numCell.innerText; // write down the original number
      numCell.innerHTML = `<b>${numCell.innerText}</b>`; 
      //const originalNum = numCell.dataset.originalNumber || numCell.innerText;
      //numCell.dataset.originalNumber = originalNum;
      //numCell.innerText = originalNum;
      //numCell.style.fontWeight = "bold";


    }
  }
  
  function flashBackground() 
	{
	  const div = document.getElementById('display');
	  const originalColor = div.style.backgroundColor;

	  const textSpan = document.getElementById('textStationName');
    const originalText = textSpan.innerText;


	  div.style.backgroundColor = '#115522'; // kolor flash
	  //textSpan.innerText = 'OLED Display Mode Changed';
    textSpan.innerHTML = '<b>Display Mode Changed</b>';  

	  setTimeout(() => 
	  {
		  div.style.backgroundColor = originalColor;
		  //textSpan.innerText = originalText;
      textSpan.innerHTML = `<b>${originalText}</b>`;
	  }, 150); // czas w ms flasha
	}


  document.addEventListener("DOMContentLoaded", function () 
  {
    connectWebSocket(); // podlaczamy websockety

    const slider = document.getElementById("volumeSlider");
    slider.addEventListener("wheel", function (event) 
    {
      event.preventDefault(); // zapobiega przewijaniu strony

      let currentValue = parseInt(slider.value);
      const step = parseInt(slider.step) || 1;
      const max = parseInt(slider.max);
      const min = parseInt(slider.min);

      if (event.deltaY < 0) {
        // przewijanie w górę (zwiększ)
        slider.value = Math.min(currentValue + step, max);
      } else {
        // przewijanie w dół (zmniejsz)
        slider.value = Math.max(currentValue - step, min);
      }

      updateSliderVolume(slider); // wywołaj aktualizację
    });
  });

  window.onpageshow = function(event) 
  {
    if (event.persisted) 
    {
      window.location.reload();
    }
  };

 </script>
)rawliteral";

const char list_html[] PROGMEM = R"rawliteral(
  <!DOCTYPE HTML><html>
  <head>
    <link rel='icon' href='/favicon.ico' type='image/x-icon'>
    <link rel="shortcut icon" href="/favicon.ico" type="image/x-icon">
    <link rel="apple-touch-icon" sizes="180x180" href="/icon.png">
    <link rel="icon" type="image/png" sizes="192x192" href="/icon.png">

    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>Evo Web Radio</title>
    <style>
      html {font-family: Arial; display: inline-block; text-align: center;}
      h2 {font-size: 1.3rem;}
      p {font-size: 1.1rem;}
      table {border: 1px solid black; border-collapse: collapse; margin: 0px 0px;}
      td, th {font-size: 0.8rem; border: 1px solid gray; border-collapse: collapse;}
      td:hover {font-weight:bold;}
      a {color: black; text-decoration: none;}
      body {max-width: 1380px; margin:0px auto; padding-bottom: 25px;}
      .columnlist { align: center; padding: 10px; display: flex; justify-content: center;}
    </style>
  </head>
)rawliteral";

const char config_html[] PROGMEM = R"rawliteral(
  <!DOCTYPE HTML>
  <html>
  <head>
    <link rel='icon' href='/favicon.ico' type='image/x-icon'>
    <link rel="shortcut icon" href="/favicon.ico" type="image/x-icon">
    <link rel="apple-touch-icon" sizes="180x180" href="/icon.png">
    <link rel="icon" type="image/png" sizes="192x192" href="/icon.png">

    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>Evo Web Radio</title>
    <style>
      html {font-family: Arial; display: inline-block; text-align: center;}
      h1 {font-size: 2.3rem;}
      h2 {font-size: 1.3rem;}
      table {border: 1px solid black; border-collapse: collapse; margin: 20px auto; width: 80%;}
      th, td {font-size: 1rem; border: 1px solid gray; padding: 8px; text-align: left;}
      td:hover {font-weight: bold;}
      a {color: black; text-decoration: none;}
      body {max-width: 1380px; margin:0 auto; padding-bottom: 25px;}
      .tableSettings {border: 2px solid #4CAF50; border-collapse: collapse; margin: 10px auto; width: 60%;}
      input[type="checkbox"] {accent-color: #66BB6A; color: #FFFF50; width: 13px; height: 13px; }
      input[type="text"] { width: 60px; box-sizing: border-box;}
      @media (max-width:768px){.about-box,.tableSettings{width:90%!important;} table{width:100%!important;} th,td{font-size:0.9rem;}}
    </style>
    </head>

  <body>
  <h2>Evo Web Radio - Settings</h2>
  <form action="/configupdate" method="POST">
  <table class="tableSettings">
  <tr><th>Display Setting</th><th>Value</th></tr>
  <tr><td>Normal Display Brightness (0-255), default:180</td><td><input type="number" name="displayBrightness" min="1" max="255" value="%D1"></td></tr>
  <tr><td>Dimmed Display Brightness (0-200), default:20</td><td><input type="number" name="dimmerDisplayBrightness" min="1" max="200" value="%D2"></td></tr>
  <tr><td>Dimmed Display Brightness in Sleep Mode (0-50), default:1</td><td><input type="number" name="dimmerSleepDisplayBrightness" min="1" max="50" value="%D12"></td></tr>
  <tr><td>Auto Dimmer Delay Time (1-255 sec.), default:5</td><td><input type="number" name="displayAutoDimmerTime" min="1" max="255" value="%D3"></td></tr>
  <tr><td>Auto Dimmer, default:On</td><td><input type="checkbox" name="displayAutoDimmerOn" value="1" %S1_checked></td></tr>
  <tr><td>OLED Display Clock in Sleep Mode default:Off</td><td><input type="checkbox" name="f_displayPowerOffClock" value="1" %S19_checked></td></tr>
  <tr><td>OLED Display Power Save Mode, default:Off</td><td><input type="checkbox" name="displayPowerSaveEnabled" value="1" %S9_checked></td></tr>
  <tr><td>OLED Display Power Save Time (1-600sek.), default:20</td><td><input type="number" name="displayPowerSaveTime" min="1" max="600" value="%D9"></td></tr>
  <tr><td>OLED Display Mode - 0-Radio basic, 1-Clock, 2-Three lines, 3-Centered, 4-VU analog, 5-VU digital</td><td><input type="number" name="displayMode" min="0" max="5" value="%D6"></td></tr>

  <tr><th>Other Setting</th><th></th></tr>
  <tr><td>Time Voice Info Every Hour, default:On</td><td><input type="checkbox" name="timeVoiceInfoEveryHour" value="1" %S3_checked></td></tr>
  <tr><td>Encoder Function Order (0-1), 0-Volume, click for station list, 1-Station list, click for Volume</td><td><input type="number" name="encoderFunctionOrder" min="0" max="1" value="%D5"></td></tr>
  <tr><td>Radio Scroller & VU Meter Refresh Time (15-100ms), default:50</td><td><input type="number" name="scrollingRefresh" min="15" max="100" value="%D8"></td></tr>
  <tr><td>ADC Keyboard Enabled, default:Off</td><td><input type="checkbox" name="adcKeyboardEnabled" value="1" %S7_checked></td></tr>
  <tr><td>Volume Steps 1-21 [Off], 1-42 [On], default:Off</td><td><input type="checkbox" name="maxVolumeExt" value="1" %S11_checked></td></tr>
  <tr><td>Station Name Read From Stream [On-From Stream, Off-From Bank] EXPERIMENTAL</td><td><input type="checkbox" name="stationNameFromStream" value="1" %S17_checked></td></tr>
  <tr><td>Radio switch to Standby After Power Fail, default:On</td><td><input type="checkbox" name="f_sleepAfterPowerFail" value="1" %S21_checked></td></tr>
  <tr><td>Volume Fade on Station Change and Power Off, default:Off</td><td><input type="checkbox" name="f_volumeFadeOn" value="1" %S22_checked></td></tr>
  <tr><td>Save ALWAYS Station No., Bank No., Volume, default:Off</td><td><input type="checkbox" name="f_saveVolumeStationAlways" value="1" %S23_checked></td></tr>
  <tr><td>Power Off Animation, default:Off</td><td><input type="checkbox" name="f_powerOffAnimation" value="1" %S24_checked></td></tr>
  <tr><td>Timezone Settings</td><td><button type="button" class="button" onclick="location.href='/timezone'">Set</button></td></tr>
  <tr><td>Remote Control Settings</td><td><button type="button" class="button" onclick="location.href='/remoteconfig'">Set</button></td></tr>
  <tr><td>Presets On (Key 0-9 switching stations 1 to 10), default:Off</td><td><input type="checkbox" name="f_Presets" value="1" %S25_checked></td></tr>

  <tr><th><b>VU Meter Settings</b></th></tr>
  <tr><td>VU Meter Mode (0-1), 0-dashed lines, 1-continuous lines</td><td><input type="number" name="vuMeterMode" min="0" max="1" value="%D4"></td></tr>
  <tr><td>VU Meter Visible (Mode 0, 3 only), default:On</td><td><input type="checkbox" name="vuMeterOn" value="1" %S5_checked></td></tr>
  <tr><td>VU Meter Peak & Hold Function, default:On</td><td><input type="checkbox" name="vuPeakHoldOn" value="1" %S13_checked></td></tr>
  <tr><td>VU Meter Smooth Function, default:On</td><td><input type="checkbox" name="vuSmooth" value="1" %S15_checked></td></tr>
  <tr><td>VU Meter Smooth Rise Speed [1 low - 32 High], default:24</td><td><input type="number" name="vuRiseSpeed" min="1" max="32" value="%D10"></td></tr>
  <tr><td>VU Meter Smooth Fall Speed [1 low - 32 High], default:6</td><td><input type="number" name="vuFallSpeed" min="1" max="32" value="%D11"></td></tr>
  </table>
  <input type="submit" value="Update">
  </form>


  <p style='font-size: 0.8rem;'><a href='/menu'>Go Back</a></p>
  </body>
  </html>
)rawliteral";

const char remoteconfig_html[] PROGMEM = R"rawliteral(
  <!DOCTYPE HTML>
  <html>
  <head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Evo Web Radio</title>

  <link rel="icon" href="/favicon.ico" type="image/x-icon">
  <link rel="shortcut icon" href="/favicon.ico" type="image/x-icon">
  <link rel="apple-touch-icon" sizes="180x180" href="/icon.png">
  <link rel="icon" type="image/png" sizes="192x192" href="/icon.png">

  <style>
  html{font-family:Arial;display:inline-block;text-align:center;}
  body{max-width:1380px;margin:0 auto;padding-bottom:25px;}
  h1{font-size:2.3rem;}h2{font-size:1.3rem;}
  table{border:1px solid #000;border-collapse:collapse;margin:20px auto;width:40%;}
  th,td{font-size:1rem;border:1px solid gray;padding:8px;text-align:left;}
  td:hover{font-weight:bold;}
  tr.selected{background:#d0f0d0;}
  a{color:black;text-decoration:none;}
  .tableSettings{border:2px solid #4CAF50;border-collapse:collapse;margin:5px auto;width:40%;}
  input[type="checkbox"]{accent-color:#66BB6A;color:#FFFF50;width:13px;height:13px;}
  input[type="text"]{width:70px;box-sizing:border-box;}
  #wsBox{border:2px solid #4CAF50;padding:10px;margin:15px auto;width:38%;background:#f5fff5;font-size:1.1rem;}
  #wsDetails{padding:5px;margin:5px auto;width:100%;background:#f5fff5;font-size:0.8rem;}
  p{font-size:0.7rem;}
  #AddrValue { margin-right: 10px; font-size:0.8rem;}
  #CmdValue { margin-right: 10px; font-size:0.8rem;}
  #Learn{background:#f5fff5;font-size:0.8rem; color:#4CAF50; font-weight:bold;}
  @media(max-width:768px){.about-box,.tableSettings,#wsBox{width:90%!important;}table{width:100%!important;}th,td{font-size:.9rem;}}
  </style>
  </head>

  <body>

  <h2>Evo Web Radio - Remote Control Config</h2>

  <div id="wsBox">
    <strong>Received code:</strong> <span id="codeValue">—</span>
    <div id="wsDetails">
      <p>Address:<span id="AddrValue">—</span>  Command:<span id="CmdValue">—</span></p>
    </div>
    <div id="Learn"><span id="LearnIndicator"></span></div>
  </div>

  <p></p>

  <div style="margin:10px;">
    <button onclick="toggleRemoteConfig()">Learning Mode - ON/OFF</button>
    <label><input type="checkbox" id="autoMode">AUTO Mode</label>&nbsp;&nbsp;
    <button type="button" onclick="skipCurrent()">Skip Sellected</button>
  </div>

  <p> Single Mouse Click on Row - Select, Double Mouse Click - Reset Value (set 0xFEFE)</p>
  <p> Auto Mode, auto increment and select next cell affer receiving IR code</p>

  <form action="/remoteconfigupdate" method="POST">
  <table class="tableSettings">
  <tr><th>Codes:</th><th>Value</th></tr>

  <tr><td>Volume Up +</td><td><input type="text" name="cmdVolumeUp" value="%D0" pattern="0x[0-9A-Fa-f]{4}" maxlength="6" title="Format: 0xFFFF"></td></tr>
  <tr><td>Volume Down -</td><td><input type="text" name="cmdVolumeDown" value="%D1" pattern="0x[0-9A-Fa-f]{4}" maxlength="6" title="Format: 0xFFFF"></td></tr>
  <tr><td>Arrow Right (Next Station / Next Bank)</td><td><input type="text" name="cmdArrowRight" value="%D2" pattern="0x[0-9A-Fa-f]{4}" maxlength="6" title="Format: 0xFFFF"></td></tr>
  <tr><td>Arrow Left (Prev Station / Prev Bank)</td><td><input type="text" name="cmdArrowLeft" value="%D3" pattern="0x[0-9A-Fa-f]{4}" maxlength="6" title="Format: 0xFFFF"></td></tr>
  <tr><td>Arrow Up (Station List Up)</td><td><input type="text" name="cmdArrowUp" value="%D4" pattern="0x[0-9A-Fa-f]{4}" maxlength="6" title="Format: 0xFFFF"></td></tr>
  <tr><td>Arrow Down (Station List Down)</td><td><input type="text" name="cmdArrowDown" value="%D5" pattern="0x[0-9A-Fa-f]{4}" maxlength="6" title="Format: 0xFFFF"></td></tr>

  <tr><td>Back</td><td><input type="text" name="cmdBack" value="%D6" pattern="0x[0-9A-Fa-f]{4}" maxlength="6" title="Format: 0xFFFF"></td></tr>
  <tr><td>OK / Enter</td><td><input type="text" name="cmdOk" value="%D7" pattern="0x[0-9A-Fa-f]{4}" maxlength="6" title="Format: 0xFFFF"></td></tr>

  <tr><td>Display Screen Mode</td><td><input type="text" name="cmdSrc" value="%D8" pattern="0x[0-9A-Fa-f]{4}" maxlength="6" title="Format: 0xFFFF"></td></tr>
  <tr><td>Mute</td><td><input type="text" name="cmdMute" value="%D9" pattern="0x[0-9A-Fa-f]{4}" maxlength="6" title="Format: 0xFFFF"></td></tr>
  <tr><td>Equalizer</td><td><input type="text" name="cmdAud" value="%D10" pattern="0x[0-9A-Fa-f]{4}" maxlength="6" title="Format: 0xFFFF"></td></tr>

  <tr><td>Display Brightness / Bank Network Update</td><td><input type="text" name="cmdDirect" value="%D11" pattern="0x[0-9A-Fa-f]{4}" maxlength="6" title="Format: 0xFFFF"></td></tr>

  <tr><td>Bank Menu (-)</td><td><input type="text" name="cmdBankMinus" value="%D12" pattern="0x[0-9A-Fa-f]{4}" maxlength="6" title="Format: 0xFFFF"></td></tr>
  <tr><td>Bank Menu (+)</td><td><input type="text" name="cmdBankPlus" value="%D13" pattern="0x[0-9A-Fa-f]{4}" maxlength="6" title="Format: 0xFFFF"></td></tr>

  <tr><td>Power On / Off</td><td><input type="text" name="cmdRedPower" value="%D14" pattern="0x[0-9A-Fa-f]{4}" maxlength="6" title="Format: 0xFFFF"></td></tr>
  <tr><td>Sleep</td><td><input type="text" name="cmdGreen" value="%D15" pattern="0x[0-9A-Fa-f]{4}" maxlength="6" title="Format: 0xFFFF"></td></tr>

  <tr><td>Key 0</td><td><input type="text" name="cmdKey0" value="%D16" pattern="0x[0-9A-Fa-f]{4}" maxlength="6" title="Format: 0xFFFF"></td></tr>
  <tr><td>Key 1</td><td><input type="text" name="cmdKey1" value="%D17" pattern="0x[0-9A-Fa-f]{4}" maxlength="6" title="Format: 0xFFFF"></td></tr>
  <tr><td>Key 2</td><td><input type="text" name="cmdKey2" value="%D18" pattern="0x[0-9A-Fa-f]{4}" maxlength="6" title="Format: 0xFFFF"></td></tr>
  <tr><td>Key 3</td><td><input type="text" name="cmdKey3" value="%D19" pattern="0x[0-9A-Fa-f]{4}" maxlength="6" title="Format: 0xFFFF"></td></tr>
  <tr><td>Key 4</td><td><input type="text" name="cmdKey4" value="%D20" pattern="0x[0-9A-Fa-f]{4}" maxlength="6" title="Format: 0xFFFF"></td></tr>
  <tr><td>Key 5</td><td><input type="text" name="cmdKey5" value="%D21" pattern="0x[0-9A-Fa-f]{4}" maxlength="6" title="Format: 0xFFFF"></td></tr>
  <tr><td>Key 6</td><td><input type="text" name="cmdKey6" value="%D22" pattern="0x[0-9A-Fa-f]{4}" maxlength="6" title="Format: 0xFFFF"></td></tr>
  <tr><td>Key 7</td><td><input type="text" name="cmdKey7" value="%D23" pattern="0x[0-9A-Fa-f]{4}" maxlength="6" title="Format: 0xFFFF"></td></tr>
  <tr><td>Key 8</td><td><input type="text" name="cmdKey8" value="%D24" pattern="0x[0-9A-Fa-f]{4}" maxlength="6" title="Format: 0xFFFF"></td></tr>
  <tr><td>Key 9</td><td><input type="text" name="cmdKey9" value="%D25" pattern="0x[0-9A-Fa-f]{4}" maxlength="6" title="Format: 0xFFFF"></td></tr>

  </table>

  <input type="submit" value="Update">
  </form>

  <p style="font-size:.8rem;"><a href="/menu">Go Back</a></p>

  <script>
  let websocket;
  let activeInput = null;
  let inputs = [];
  let currentIndex = -1;
  let autoMode = false;

  function toggleRemoteConfig() {
    const xhr = new XMLHttpRequest();
    xhr.open("POST", "/toggleRemoteConfig", true);
    xhr.send();
  }

  function clearSelection() {
    document.querySelectorAll(".tableSettings tr")
      .forEach(r => r.classList.remove("selected"));
    activeInput = null;
    currentIndex = -1;
  }

  function selectInputByIndex(index) {
    if (index >= inputs.length) {
      autoMode = false;
      document.getElementById("autoMode").checked = false;
      clearSelection();
      return;
    }

    clearSelection();
    currentIndex = index;
    activeInput = inputs[index];
    const row = activeInput.closest("tr");
    row.classList.add("selected");
    activeInput.focus();
  }

  function moveNext() {
    selectInputByIndex(currentIndex + 1);
  }

  function skipCurrent() {
    if (!activeInput) return;
    activeInput.value = "0xFEFE";
    if (autoMode) moveNext();
  }

  document.getElementById("autoMode").addEventListener("change", function () {
    autoMode = this.checked;
    if (autoMode) selectInputByIndex(0);
    else clearSelection();
  });

  document.addEventListener("DOMContentLoaded", function () {
    inputs = Array.from(
      document.querySelectorAll('.tableSettings input[type="text"]')
    );

    inputs.forEach((input, index) => {
      const row = input.closest("tr");

      row.onclick = () => selectInputByIndex(index);
      row.ondblclick = () => {
        input.value = "0xFEFE";
        selectInputByIndex(index);
      };
    });

    websocket = new WebSocket('ws://' + window.location.hostname + '/ws');

    websocket.onmessage = function (event) {
      if (event.data.startsWith("addr:"))
        document.getElementById('AddrValue').innerText = event.data.split(":")[1];

      if (event.data.startsWith("cmd:"))
        document.getElementById('CmdValue').innerText = event.data.split(":")[1];

      if (event.data.startsWith("learn:"))
        document.getElementById("LearnIndicator").innerText =
          parseInt(event.data.split(":")[1]) ? "LEARNING MODE ON" : "";

      if (event.data.startsWith("ircode:")) {
        const codeStr = event.data.split(":")[1].trim();
        document.getElementById('codeValue').innerText = codeStr;

        if (activeInput) {
          activeInput.value = codeStr;
          if (autoMode) moveNext();
        }
      }
    };
  });
  </script>

  </body>
  </html>
)rawliteral";

const char adc_html[] PROGMEM = R"rawliteral(
  <!DOCTYPE HTML>
  <html>
  <head>
    <link rel='icon' href='/favicon.ico' type='image/x-icon'>
    <link rel="shortcut icon" href="/favicon.ico" type="image/x-icon">
    <link rel="apple-touch-icon" sizes="180x180" href="/icon.png">
    <link rel="icon" type="image/png" sizes="192x192" href="/icon.png">

    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>Evo Web Radio</title>
    <style>
      html {font-family: Arial; display: inline-block; text-align: center;}
      h1 {font-size: 2.3rem;}
      h2 {font-size: 1.3rem;}
      table {border: 1px solid black; border-collapse: collapse; margin: 10px auto; width: 40%;}
      th, td {font-size: 1rem; border: 1px solid gray; padding: 8px; text-align: left;}
      td:hover {font-weight: bold;}
      a {color: black; text-decoration: none;}
      body {max-width: 1380px; margin:0 auto; padding-bottom: 15px;}
      .tableSettings {border: 2px solid #4CAF50; border-collapse: collapse; margin: 10px auto; width: 40%;}
      .button { background-color: #4CAF50; border: 1; color: white; padding: 10px 20px; border-radius: 5px; width:200px;}
      .button:hover {background-color: #4a4a4a;}
      .button:active {background-color: #4a4a4a; box-shadow: 0 4px #666; transform: translateY(2px);}
    </style>
    </head>

  <body>
  <h2>Evo Web Radio - ADC Settings</h2>
  <form action="/configadc" method="POST">
  <table class="tableSettings">
  <tr><th>Button</th><th>Value</th></tr>
  <tr><td>Key 0</td><td><input type="number" name="keyboardButtonThreshold_0" min="0" max="4100" value="%D0"></td></tr>
  <tr><td>Key 1</td><td><input type="number" name="keyboardButtonThreshold_1" min="0" max="4100" value="%D1"></td></tr>
  <tr><td>Key 2</td><td><input type="number" name="keyboardButtonThreshold_2" min="0" max="4100" value="%D2"></td></tr>
  <tr><td>Key 3</td><td><input type="number" name="keyboardButtonThreshold_3" min="0" max="4100" value="%D3"></td></tr>
  <tr><td>Key 4</td><td><input type="number" name="keyboardButtonThreshold_4" min="0" max="4100" value="%D4"></td></tr>
  <tr><td>Key 5</td><td><input type="number" name="keyboardButtonThreshold_5" min="0" max="4100" value="%D5"></td></tr>
  <tr><td>Key 6</td><td><input type="number" name="keyboardButtonThreshold_6" min="0" max="4100" value="%D6"></td></tr>
  <tr><td>Key 7</td><td><input type="number" name="keyboardButtonThreshold_7" min="0" max="4100" value="%D7"></td></tr>
  <tr><td>Key 8</td><td><input type="number" name="keyboardButtonThreshold_8" min="0" max="4100" value="%D8"></td></tr>
  <tr><td>Key 9</td><td><input type="number" name="keyboardButtonThreshold_9" min="0" max="4100" value="%D9"></td></tr>
  <tr><td>OK / Enter Key</td><td><input type="number" name="keyboardButtonThreshold_Ok" min="0" max="4100" value="%D10"></td></tr>
  <tr><td>Display Bank Menu Key</td><td><input type="number" name="keyboardButtonThreshold_BankMenu" min="0" max="4100" value="%D11"></td></tr>
  <tr><td>Back Key</td><td><input type="number" name="keyboardButtonThreshold_Back" min="0" max="4100" value="%D12"></td></tr>
  <tr><td>OLED Display Mode Key</td><td><input type="number" name="keyboardButtonThreshold_DisplayMode" min="0" max="4100" value="%D13"></td></tr>
  <tr><td>Dimmer / Network Bank Update Key</td><td><input type="number" name="keyboardButtonThreshold_Dimmer" min="0" max="4100" value="%D14"></td></tr>
  <tr><td>Mute Key</td><td><input type="number" name="keyboardButtonThreshold_Mute" min="0" max="4100" value="%D15"></td></tr>
  
  <tr><td>Navigation Left - Previous Station / Bank</td><td><input type="number" name="keyboardButtonThreshold_ArrowLeft" min="0" max="4100" value="%D19"></td></tr>
  <tr><td>Navigation Right - Next Statition / Bank</td><td><input type="number" name="keyboardButtonThreshold_ArrowRight" min="0" max="4100" value="%D20"></td></tr>
  <tr><td>Navigation Up - Station List Up</td><td><input type="number" name="keyboardButtonThreshold_ArrowUp" min="0" max="4100" value="%D21"></td></tr>
  <tr><td>Navigation Down - Station List Down</td><td><input type="number" name="keyboardButtonThreshold_ArrowDown" min="0" max="4100" value="%D22"></td></tr>
  
  <tr><td>Presets On Key</td><td><input type="number" name="keyboardButtonThreshold_PresetsOn" min="0" max="4100" value="%D23"></td></tr>

  <tr><td>Keyboard Buttons Threshold Tolerance</td><td><input type="number" name="keyboardButtonThresholdTolerance" min="0" max="50" value="%D16"></td></tr>
  <tr><td>Keyboard Buttons Neutral</td><td><input type="number" name="keyboardButtonNeutral" min="0" max="4100" value="%D17"></td></tr>
  <tr><td>Keyboard Sample Delay (30-300ms)</td><td><input type="number" name="keyboardSampleDelay" min="30" max="300" value="%D18"></td></tr>
  </table>
  <input type="submit" value="ADC Thresholds Update">
  </form>
  <br>
  <button onclick="toggleAdcDebug()">ADC Debug ON/OFF</button>
  <script>
  function toggleAdcDebug() {
      const xhr = new XMLHttpRequest();
      xhr.open("POST", "/toggleAdcDebug", true);
      xhr.setRequestHeader("Content-Type", "application/x-www-form-urlencoded");
      xhr.onload = function() {
          if (xhr.status === 200) {
              alert("ADC Debug is now " + (xhr.responseText === "1" ? "ON" : "OFF"));
          } else {
              alert("Error: " + xhr.statusText);
          }
      };
      xhr.send(); // Wysyłanie pustego zapytania POST
  }
  </script>

  <p style='font-size: 0.8rem;'><a href='/menu'>Go Back</a></p>
  </body>
  </html>
)rawliteral";

const char menu_html[] PROGMEM = R"rawliteral(
  <!DOCTYPE HTML><html>
  <head>
  <link rel='icon' href='/favicon.ico' type='image/x-icon'>
  <link rel="shortcut icon" href="/favicon.ico" type="image/x-icon">
  <link rel="apple-touch-icon" sizes="180x180" href="/icon.png">
  <link rel="icon" type="image/png" sizes="192x192" href="/icon.png">

  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Evo Web Radio</title>
  <style>
    html {font-family: Arial; display: inline-block; text-align: center;}
    h2 {font-size: 1.3rem;}
    p {font-size: 1.1rem;}
    a {color: black; text-decoration: none;}
    body {max-width: 1380px; margin:0px auto; padding-bottom: 25px;}
    .button { background-color: #4CAF50; border: 1; color: white; padding: 10px 20px; border-radius: 5px; width:200px;}
    .button:hover {background-color: #4a4a4a;}
    .button:active {background-color: #4a4a4a; box-shadow: 0 4px #666; transform: translateY(2px);}
    
  </style>
  </head>
  <body>
  <h2>Evo Web Radio - Menu</h2>
  <!-- <br><button class="button" onclick="location.href='/fwupdate'">OTA Update (Old)</button><br> -->
  <br><button class="button" onclick="location.href='/info'">Info</button><br>
  <br><button class="button" onclick="location.href='/ota'">OTA Update</button><br>
  <br><button class="button" onclick="location.href='/adc'">ADC Keyboard Settings</button><br>
  <br><button class="button" onclick="location.href='/list'">Storage Memory Explorer</button><br>
  <br><button class="button" onclick="location.href='/editor'">Memory Bank Editor</button><br>
  <br><button class="button" onclick="location.href='/browser'">Radio Browser API</button><br>
  <br><button class="button" onclick="location.href='/playurl'">Play from URL</button><br>
  <br><button class="button" onclick="location.href='/config'">Settings</button><br>
  <br><p style='font-size: 0.8rem;'><a href="#" onclick="window.location.replace('/')">Go Back</a></p>
  </body></html>

)rawliteral";

const char info_html[] PROGMEM = R"rawliteral(
  <!DOCTYPE HTML>
  <html>
  <head>
    <link rel='icon' href='/favicon.ico' type='image/x-icon'>
    <link rel="shortcut icon" href="/favicon.ico" type="image/x-icon">
    <link rel="apple-touch-icon" sizes="180x180" href="/icon.png">
    <link rel="icon" type="image/png" sizes="192x192" href="/icon.png">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>Evo Web Radio</title>
    <style>
      html{font-family:Arial;display:inline-block;text-align:center;} 
      h2{font-size:1.3rem;} 
      table{border:1px solid black;border-collapse:collapse;margin:10px auto;width:40%;} 
      th,td{font-size:1rem;border:1px solid gray;padding:8px;text-align:left;} 
      td:hover{font-weight:bold;} 
      a{color:black;text-decoration:none;} 
      body{max-width:1380px;margin:0 auto;padding-bottom:15px;} 
      .tableSettings{border:2px solid #4CAF50;border-collapse:collapse;margin:10px auto;width:40%;} 
      .signal-bars{display:inline-block;vertical-align:middle;margin-left:10px;} 
      .bar{display:inline-block;width:5px;margin-right:2px;background-color:#F2F2F2;height:10px;} 
      .bar.active{background-color:#4CAF50;} 
      .about-box{border:2px solid #4CAF50;border-collapse:collapse;margin:10px auto;width:40%;text-align:left;font-size:0.9rem;line-height:1.4rem;color:#222;} 
      .about-box h3{color:#2f7a2f;margin-top:0;text-align:center;font-size:1.05rem;} 
      .about-box a{color:#2b5bb3;text-decoration:none;} 
      .about-box a:hover{text-decoration:underline;}
      @media (max-width:768px){.about-box,.tableSettings{width:90%!important;} table{width:100%!important;} th,td{font-size:0.9rem;}}
    </style>
  </head>
  <body>
  <h2>Evo Web Radio - Info</h2>

  <div class="about-box">
    <h3>Project Info</h3>
    <p>This is a project of an Internet radio streamer called <strong>"Evo"</strong>. The hardware was built using an <strong>ESP32-S3</strong> microcontroller and a <strong>PCM5102A DAC codec</strong>.</p>
    <p>The design allows listening to various music stations from all around the world and works properly with streams encoded in <strong>MP3</strong>, <strong>AAC</strong>, 
    <strong>VORBIS</strong>, <strong>OPUS</strong>, and <strong>FLAC</strong> (up to 2000kbs, 24bit).</p>
    <p>All operations (volume control, station change, memory bank selection, power on/off) are handled via a single rotary encoder. It also supports infrared remote controls working on the <strong>NEC standard (38&nbsp;kHz)</strong>.</p>
    <p></p>
    <p>Source code and documentation are available at: <b><a href="https://github.com/dzikakuna/ESP32_radio_evo3" target="_blank">Link: https://github.com/dzikakuna/ESP32_radio_evo3</a></b></p>
    <p>To change WIFI Access Point,<br>Press and hold encoder button and power on the EVO-Radio.<br>A options menu will be shown on the display</p>
    <p>Robgold 2026, Made in Poland</p>
  </div>

  <table class="tableSettings">
      <tr><td>ESP Serial Number:</td><td><input name="espSerial" value="%D0"></td></tr>
      <tr><td>Firmware Version:</td><td><input name="espFw" value="%D1"></td></tr>
      <tr><td>Hostname:</td><td><input name="hostnameValue" value="%D2"></td></tr>
      <tr><td>WiFi Signal Strength [dBm]:</td><td><input id="wifiSignal" value="%D3"></td></tr>
      <tr><td>WiFi SSID:</td><td><input name="wifiSsid" value="%D4"></td></tr>
      <tr><td>IP Address:</td><td><input name="ipValue" value="%D5"></td></tr>
      <tr><td>MAC Address:</td><td><input name="macValue" value="%D6"></td></tr>
      <tr><td>Memory type:</td><td><input name="memValue" value="%D7"></td></tr>
    </table>
  <br>
  <p style="font-size:0.8rem;"><a href="/menu">Go Back</a></p>
  </body>
  </html>
)rawliteral";

const char urlplay_html[] PROGMEM = R"rawliteral(
  <!DOCTYPE HTML>
  <html>
    <head>
      <meta name="viewport" content="width=device-width, initial-scale=1">
      <title>Evo Web Radio</title>

      <style>
        html {font-family: Arial; text-align: center;}
        body {max-width: 1380px; margin: 0 auto; background: #D0D0D0; padding-bottom: 25px;}
        h2 {font-size: 1.3rem;}
        p {font-size: 0.95rem;}
        .slider{-webkit-appearance:none;margin:14px;width:330px;height:10px;background:#4CAF50;outline:none;-webkit-transition:.2s;transition:opacity .2s;border-radius:5px}
        .slider::-webkit-slider-thumb{-webkit-appearance:none;appearance:none;width:35px;height:25px;background:#4a4a4a;cursor:pointer;border-radius:5px}
        .slider::-moz-range-thumb{width:35px;height:35px;background:#4a4a4a;cursor:pointer;border-radius:5px}
        .button {background-color:#4CAF50;border:1; color:white; padding:10px 20px; border-radius:5px;cursor:pointer;}
        .button:hover {background-color:#4a4a4a;}
        .button.active {background-color: #115522; transform: scale(1.05); transition: all 0.15s ease;}
        #urlInput {width:350px; padding:5px; border-radius:5px; font-size:0.9rem;}
        #playBtn {padding:5px 10px; font-size:0.9rem;}
        #display {display:inline-block; padding:5px; border:2px solid #4CAF50; border-radius:15px; background-color:#4a4a4a; font-size:1.45rem; color:#AAA; width:345px; text-align:center; white-space:nowrap; box-shadow:0 0 20px #4CAF50;}
        #stationTextDiv {display:block; text-overflow:ellipsis; white-space:normal; font-size:1.0rem; color:#999; margin-bottom:10px; text-align:center; height:4.2em; overflow:hidden; line-height:1.4em;}
        #muteIndicatorDiv {text-align:center; color:#4CAF50; font-weight:bold; font-size:1.0rem; height:1.4em;}
        #urlInput.flash {background-color: #4CAF50;color: #fff; transition: background-color 0.3s ease;}
        .wifi-wrapper {display: flex; align-items: flex-end; gap: 1px; height: 13px; margin-top: -3px; margin-bottom: 3px; justify-content: center; }
        .wifiBar {width: 2px;background-color: #555; border-radius: 2px; transition: background-color 0.2s;}
        .bar1 { height: 2px; }
        .bar2 { height: 4px; }
        .bar3 { height: 6px; }
        .bar4 { height: 8px; }
        .bar5 { height: 10px; }
        .bar6 { height: 12px; }
      </style>
  </head>

  <body>
  <h2>Evo Web Radio - URL Play</h2>

  <div id="display" onclick="flashBackground(); displayMode();">
      <div style="margin-bottom:10px; font-weight:bold; overflow:hidden; text-overflow:ellipsis;">
          <span id="textStationName"><b>STATIONNAME</b></span>
      </div>

      <div style="width:345px; margin-bottom:10px;">
          <div id="stationTextDiv">
              <span id="stationText">STATIONTEXT</span>
          </div>
          <div id="muteIndicatorDiv">
              <span id="muteIndicator"></span>
          </div>
      </div>

      <div style="height:1px; background-color:#4CAF50; margin:5px 0;"></div>

      <div style="display:flex; justify-content:center; gap:13px; font-size:1.0rem; color:#999;">
          <div><span id="bankValue">Bank: --</span></div>
            <div style="display:flex; gap:6px; font-size:0.55rem; color:#999;margin:4px 0;">
              <div><span id="samplerate">---.-kHz</span></div>
              <div><span id="bitrate">---kbps</span></div>
              <div><span id="bitpersample">---bit</span></div>
              <div><span id="streamformat">----</span></div>
              WiFi: <div class="wifi-wrapper">
              <div class="wifiBar bar1" id="wifiBar1"></div>
              <div class="wifiBar bar2" id="wifiBar2"></div>
              <div class="wifiBar bar3" id="wifiBar3"></div>
              <div class="wifiBar bar4" id="wifiBar4"></div>
              <div class="wifiBar bar5" id="wifiBar5"></div>
              <div class="wifiBar bar6" id="wifiBar6"></div>
  	        </div>
          </div>
          <div><span id="stationNumber">Station: --</span></div>
      </div>
  </div>

  <br><br>

  <!-- URL + PLAY -->
  <p><input id="urlInput" type="text" placeholder="Put your URL here"></p>
  <p><button id="playBtn" class="button" onclick="playStream()">PLAY</button></p>

  <br>

  <!-- Volume -->
  <p>Volume: <span id="textSliderValue">--</span></p>
  <p><input type="range" onchange="updateSliderVolume()" id="volumeSlider" min="1" max="21" value="1" step="1" class="slider"></p>

  <p style='font-size: 0.8rem;'><a href="#" onclick="window.location.replace('/menu')">Go Back</a></p>

  <script>
  var websocket;

  function setWifi(level) 
  {
    for (let i = 1; i <= 6; i++) 
    {
      const bar = document.getElementById("wifiBar" + i);
      if (bar) {bar.style.backgroundColor = (i <= level) ? "#4CAF50" : "#555";}
    }
  }


  function connectWebSocket() 
  {
      websocket = new WebSocket('ws://' + window.location.hostname + '/ws');

      websocket.onopen = function() {
          console.log("WebSocket połączony");
      };

      websocket.onclose = function() {
          console.log("WebSocket zamknięty, reconnect za 3s...");
          setTimeout(connectWebSocket, 3000);
      };

      websocket.onerror = function(err) {
          console.error("Błąd WebSocket:", err);
          websocket.close();
      };

      websocket.onmessage = function(event) {
          const data = event.data;

          if (data.startsWith("volume:")) {
              const vol = parseInt(data.split(":")[1]);
              document.getElementById("volumeSlider").value = vol;
              document.getElementById("textSliderValue").innerText = vol;
          }

          if (data.startsWith("stationname:")) {
              const value = data.split(":")[1];
              document.getElementById("textStationName").innerHTML = "<b>" + value + "</b>";
          }

          if (data.startsWith("stationtext$")) {
              const text = data.substring(data.indexOf("$") + 1);
              document.getElementById("stationText").textContent = text;
          }

          if (data.startsWith("station:")) {
              const station = parseInt(data.split(":")[1]);
              document.getElementById("stationNumber").innerText = "Station: " + station;
          }

          if (data.startsWith("bank:")) {
              const bank = parseInt(data.split(":")[1]);
              document.getElementById("bankValue").innerText = "Bank: " + bank;
          }

          if (data.startsWith("samplerate:")) {
              const rate = parseInt(data.split(":")[1]);
              document.getElementById("samplerate").innerText = (rate / 1000).toFixed(1) + "kHz";
          }

          if (data.startsWith("bitrate:")) {
              document.getElementById("bitrate").innerText = data.split(":")[1] + "kbps";
          }

          if (data.startsWith("bitpersample:")) {
              document.getElementById("bitpersample").innerText = data.split(":")[1] + "bit";
          }

          if (data.startsWith("streamformat:")) {
              document.getElementById("streamformat").innerText = data.split(":")[1];
          }

          if (data.startsWith("mute:")) {
              const mute = parseInt(data.split(":")[1]);
              document.getElementById("muteIndicator").innerText = mute === 1 ? "MUTED" : "";
          }

          if (event.data.startsWith("wifi:")) {
            const level = parseInt(event.data.split(":")[1]);
            if (!isNaN(level)) {setWifi(Math.min(Math.max(level, 1), 6));}
          }
      };
  }

  function updateSliderVolume() {
      const slider = document.getElementById("volumeSlider");
      const value = slider.value;
      document.getElementById("textSliderValue").innerText = value;

      if (websocket && websocket.readyState === WebSocket.OPEN) {
          websocket.send("volume:" + value);
      }
  }

  function playStream() {
      const urlInput = document.getElementById("urlInput");
      const url = urlInput.value.trim();
      const host = window.location.hostname;

      urlInput.classList.add("flash");
      setTimeout(() => urlInput.classList.remove("flash"), 300);

      if (!url) return;

      fetch("http://" + host + "/update?url=" + encodeURIComponent(url))
          .catch(err => console.error("Błąd połączenia:", err));
  }

  function flashBackground() {
      const div = document.getElementById("display");
      const text = document.getElementById("textStationName");
      const origColor = div.style.backgroundColor;
      const origText = text.innerHTML;

      div.style.backgroundColor = "#115522";
      text.innerHTML = "<b>Display Mode Changed</b>";

      setTimeout(() => {
          div.style.backgroundColor = origColor;
          text.innerHTML = origText;
      }, 150);
  }

  function displayMode() {
      if (websocket && websocket.readyState === WebSocket.OPEN) {
          websocket.send("displayMode");
      }
  }

  document.addEventListener("DOMContentLoaded", () => {
      connectWebSocket();
  });
  </script>
  </body>
  </html>
)rawliteral";

const char timezone_html[] PROGMEM = R"rawliteral(
  <!DOCTYPE html>
  <html>
  <head>
  <meta charset="UTF-8">
  <title>Timezone Settings</title>
  <style>
  body { font-family: Arial; background: #D0D0D0; text-align: center; padding: 20px; }
  .tzField { width: 380px; padding: 8px; border-radius: 6px; border: 2px solid #4CAF50; font-size: 1rem; margin-bottom: 10px; text-align: left; }
  input#tzCustom { width: 380px; display: block; margin: 10px auto; text-align: center; }
  button { padding: 10px 20px; background: #4CAF50; color: white; border: 1; border-radius: 6px; cursor: pointer; font-size: 1rem; }
  button:hover { background: #3e8e41;}
  #msg { margin-top: 15px; font-weight: bold; }
  </style>
  </head>
  <body>

  <h2>Evo Web Radio - Timezone Set</h2>
  <p>Current timezone:
  <b id="currentTZ">Loading...</b></p>

  <p>Select timezone from list</p>

  <select id="tzSelect" class="tzField" onchange="selectChanged()">
      <option value="custom">Custom...</option>

      <!-- Europe -->
      <optgroup label="Europe">
          <option value="UTC0">UTC / GMT - UTC0</option>
          <option value="GMT0BST,M3.5.0/1,M10.5.0/2">United Kingdom - GMT0BST,M3.5.0/1,M10.5.0/2</option>
          <option value="CET-1CEST,M3.5.0/2,M10.5.0/3">Central Europe - CET-1CEST,M3.5.0/2,M10.5.0/3</option>
          <option value="EET-2EEST,M3.5.0/3,M10.5.0/4">Eastern Europe - EET-2EEST,M3.5.0/3,M10.5.0/4</option>
          <option value="GMT-1">Azores - GMT-1</option>
          <option value="WET0WEST,M3.5.0/1,M10.5.0/2">Portugal - WET0WEST,M3.5.0/1,M10.5.0/2</option>
          <option value="MSK-3">Moscow - MSK-3</option>
      </optgroup>

      <!-- North America -->
      <optgroup label="North America">
          <option value="EST5EDT,M3.2.0/2,M11.1.0/2">US Eastern - EST5EDT,M3.2.0/2,M11.1.0/2</option>
          <option value="CST6CDT,M3.2.0/2,M11.1.0/2">US Central - CST6CDT,M3.2.0/2,M11.1.0/2</option>
          <option value="MST7MDT,M3.2.0/2,M11.1.0/2">US Mountain - MST7MDT,M3.2.0/2,M11.1.0/2</option>
          <option value="PST8PDT,M3.2.0/2,M11.1.0/2">US Pacific - PST8PDT,M3.2.0/2,M11.1.0/2</option>
          <option value="AKST9AKDT,M3.2.0/2,M11.1.0/2">Alaska - AKST9AKDT,M3.2.0/2,M11.1.0/2</option>
          <option value="HST10">Hawaii - HST10</option>
      </optgroup>

      <!-- South America -->
      <optgroup label="South America">
          <option value="ART-3">Argentina - ART-3</option>
          <option value="BRT-3">Brazil - BRT-3</option>
          <option value="CLT-4CLST,M9.1.0/0,M4.1.0/0">Chile - CLT-4CLST,M9.1.0/0,M4.1.0/0</option>
      </optgroup>

      <!-- Asia -->
      <optgroup label="Asia">
          <option value="IST-5.5">India - IST-5.5</option>
          <option value="CST-8">China - CST-8</option>
          <option value="JST-9">Japan - JST-9</option>
          <option value="KST-9">Korea - KST-9</option>
          <option value="SGT-8">Singapore - SGT-8</option>
          <option value="WIB-7">Indonesia (WIB) - WIB-7</option>
          <option value="ICT-7">Thailand - ICT-7</option>
      </optgroup>

      <!-- Australia / Oceania -->
      <optgroup label="Australia">
          <option value="AEST-10AEDT,M10.1.0/2,M4.1.0/3">Australia Eastern - AEST-10AEDT,M10.1.0/2,M4.1.0/3</option>
          <option value="ACST-9.5ACDT,M10.1.0/2,M4.1.0/3">Australia Central - ACST-9.5ACDT,M10.1.0/2,M4.1.0/3</option>
          <option value="AWST-8">Australia Western - AWST-8</option>
      </optgroup>

      <!-- Africa -->
      <optgroup label="Africa">
          <option value="CAT-2">Central Africa - CAT-2</option>
          <option value="EAT-3">East Africa - EAT-3</option>
          <option value="WAT-1">West Africa - WAT-1</option>
          <option value="SAST-2">South Africa - SAST-2</option>
      </optgroup>
  </select>

  <input id="tzCustom" class="tzField" type="text" placeholder="Enter custom Timezone string..." readonly>

  <p id="msg"></p>
  <br>
  <button onclick="saveTZ()">Save</button>

  <p style="font-size: 0.8rem;"><a href="/menu">Go Back</a></p>

  <script>
  function loadTZ() {
      fetch("/getTZ").then(r => r.text()).then(tz => {
          document.getElementById("currentTZ").textContent = tz;
          let select = document.getElementById("tzSelect");
          let input = document.getElementById("tzCustom");
          let found = false;

          for (let i = 0; i < select.options.length; i++) {
              if (select.options[i].value === tz) {
                  select.selectedIndex = i;
                  input.value = tz;
                  input.readOnly = true;
                  input.style.display = "block";
                  found = true;
                  break;
              }
          }

          if (!found) {
              select.value = "custom";
              input.value = tz;
              input.readOnly = false;
              input.style.display = "block";
          }
      });
  }

  function selectChanged() {
      let select = document.getElementById("tzSelect");
      let input = document.getElementById("tzCustom");

      if (select.value === "custom") {
          input.readOnly = false;
          input.value = "";
          input.style.display = "block";
      } else {
          input.value = select.value;
          input.readOnly = true;
          input.style.display = "block";
      }
  }

  function saveTZ() {
      let input = document.getElementById("tzCustom");
      let tzToSave = input.value.trim();

      if (!tzToSave) {
          document.getElementById("msg").style.color = "red";
          document.getElementById("msg").textContent = "Please enter a timezone!";
          return;
      }

      fetch("/setTZ?value=" + encodeURIComponent(tzToSave))
          .then(r => r.text())
          .then(() => {
              document.getElementById("msg").style.color = "green";
              document.getElementById("msg").textContent = "Timezone saved and set.";
              loadTZ();
          })
          .catch(err => {
              document.getElementById("msg").style.color = "red";
              document.getElementById("msg").textContent = "Error: " + err;
          });
  }
  loadTZ();
  </script>
  </body>
  </html>
)rawliteral";

const char ota_html[] PROGMEM = R"rawliteral(
  <!DOCTYPE html>
  <html lang='en'>
  <head>
  <meta charset='UTF-8' />
  <meta name='viewport' content='width=device-width, initial-scale=1.0'/>
  <title>ESP OTA Update</title>
  <style>
  body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; background-color: #f3f4f6; color: #333; padding: 40px; text-align: center; }
  h2 { margin-bottom: 30px; color: #111; font-size: 1.3rem;}
  #uploadSection { background-color: white; padding: 30px; border-radius: 8px; display: inline-block; box-shadow: 0 0 10px rgba(0, 0, 0, 0.1); }
  input[type='file'] { padding: 10px; }
  button { margin-top: 20px; padding: 10px 20px; background-color: #4CAF50; border: none; color: white; font-size: 16px; border-radius: 5px; cursor: pointer; }
  button:disabled { background-color: #ccc; cursor: not-allowed; }
  progress { width: 100%; height: 20px; margin-top: 20px; border-radius: 5px; appearance: none; -webkit-appearance: none; }
  progress::-webkit-progress-bar { background-color: #eee; border-radius: 5px; }
  progress::-webkit-progress-value { background-color: #4CAF50; border-radius: 5px; }
  progress::-moz-progress-bar { background-color: #4CAF50; border-radius: 5px; }
  #status { margin-top: 15px; font-weight: bold; }
  #fileInfo { color: #555; margin-top: 10px; }
  </style>
  </head>
  <body>
  <div id='uploadSection'>
  <h2>Evo Web Radio - OTA Firmware Update</h2>
  <input type='file' id='fileInput' name='update' /><br />
  <div id='fileInfo'>No file selected</div>
  <button id='uploadBtn'>Upload</button>
  <p id='status'></p>
  </div>

  <script>
  const fileInput = document.getElementById('fileInput');
  const uploadBtn = document.getElementById('uploadBtn');
  const status = document.getElementById('status');
  const fileInfo = document.getElementById('fileInfo');

  fileInput.addEventListener('change', function () {
    const file = this.files[0];
    if (file) {
      fileInfo.textContent = `Selected: ${file.name} (${(file.size / 1024).toFixed(2)} KB)`;
    } else {
      fileInfo.textContent = 'No file selected';
    }
  });

  uploadBtn.addEventListener('click', function () {
    const file = fileInput.files[0];
    if (!file) { alert('Please select a file first.'); return; }
    uploadBtn.disabled = true;
    status.textContent = 'Uploading...';

    const xhr = new XMLHttpRequest();
    const formData = new FormData();
    formData.append('update', file);

    xhr.open('POST', '/firmwareota', true);

    xhr.onload = function () {
      if (xhr.status === 200) {
        status.textContent = '✅ Upload completed, reboot in 3 sec.';
        setTimeout(() => { window.location.href = '/'; }, 10000);
      } else {
        status.textContent = '❌ Upload failed.';
      }
      uploadBtn.disabled = false;
    };

    xhr.onerror = function () {
      status.textContent = '❌ Network error.';
      uploadBtn.disabled = false;
    };

    xhr.send(formData);
  });
  </script>

  </body>
  </html>
)rawliteral";


// ------------------- END OF HTML PAGES embedded in the code -------------------

char stations[MAX_STATIONS][STATION_NAME_LENGTH + 1];  // An array storing links to radio stations (one per station) +1 for the null terminator

const char *ntpServer1 = "pool.ntp.org";  // NTP server address used for time synchronization
const char *ntpServer2 = "time.nist.gov"; // NTP server address used for time synchronization
//const long gmtOffset_sec = 3600;          // UTC time offset in seconds
//const int daylightOffset_sec = 3600;      // Daylight saving time shift in seconds, for Belgium it is 1 hour

const int LOGO_SIZE = (SCREEN_WIDTH * SCREEN_HEIGHT) / 8;
uint8_t logo_bits[LOGO_SIZE];

//SPLEEN font with Polish characters for radio stream tittle
const uint8_t spleen6x12PL[2958] U8G2_FONT_SECTION("spleen6x12PL") =
  "\340\1\3\2\3\4\1\3\4\6\14\0\375\10\376\11\377\1\225\3]\13q \7\346\361\363\237\0!\12"
  "\346\361#i\357`\316\0\42\14\346\361\3I\226dI\316/\0#\21\346\361\303I\64HI\226dI"
  "\64HIN\6$\22\346q\205CRK\302\61\311\222,I\206\60\247\0%\15\346\361cQK\32\246"
  "I\324\316\2&\17\346\361#Z\324f\213\22-Zr\42\0'\11\346\361#i\235\237\0(\13\346\361"
  "ia\332s\254\303\0)\12\346\361\310\325\36\63\235\2*\15\346\361S\243L\32&-\312\31\1+\13"
  "\346\361\223\323l\320\322\234\31,\12\346\361\363)\15s\22\0-\11\346\361s\32t\236\0.\10\346\361"
  "\363K\316\0/\15\346q\246a\32\246a\32\246\71\15\60\21\346\361\3S\226DJ\213\224dI\26\355"
  "d\0\61\12\346\361#\241\332\343N\6\62\16\346\361\3S\226\226\246\64\35t*\0\63\16\346\361\3S"
  "\226fr\232d\321N\6\64\14\346q\247\245\236\6\61\315\311\0\65\16\346q\17J\232\16qZ\31r"
  "\62\0\66\20\346\361\3S\232\16Q\226dI\26\355d\0\67\13\346q\17J\226\206\325v\6\70\20\346"
  "\361\3S\226d\321\224%Y\222E;\31\71\17\346\361\3S\226dI\26\15ii'\3:\11\346\361"
  "\263\346L\71\3;\13\346\361\263\346\264\64\314I\0<\12\346\361cak\334N\5=\13\346\361\263\15"
  ":\60\350\334\0>\12\346\361\3qk\330\316\2\77\14\346\361\3S\226\206\325\34\314\31@\21\346\361\3"
  "S\226dI\262$K\262\304CN\5A\22\346\361\3S\226dI\226\14J\226dI\226S\1B\22"
  "\346q\17Q\226d\311\20eI\226d\311\220\223\1C\14\346\361\3C\222\366<\344T\0D\22\346q"
  "\17Q\226dI\226dI\226d\311\220\223\1E\16\346\361\3C\222\246C\224\226\207\234\12F\15\346\361"
  "\3C\222\246C\224\266\63\1G\21\346\361\3C\222V\226,\311\222,\32r*\0H\22\346qgI"
  "\226d\311\240dI\226dI\226S\1I\12\346\361\3c\332\343N\6J\12\346\361\3c\332\233\316\2"
  "K\21\346qgI\226D\321\26\325\222,\311r*\0L\12\346q\247}\36r*\0M\20\346qg"
  "\211eP\272%Y\222%YN\5N\20\346qg\211\224HI\77)\221\222\345T\0O\21\346\361\3"
  "S\226dI\226dI\226d\321N\6P\17\346q\17Q\226dI\226\14QZg\2Q\22\346\361\3"
  "S\226dI\226dI\226d\321\252\303\0R\22\346q\17Q\226dI\226\14Q\226dI\226S\1S"
  "\16\346\361\3C\222\306sZ\31r\62\0T\11\346q\17Z\332w\6U\22\346qgI\226dI\226"
  "dI\226d\321\220S\1V\20\346qgI\226dI\226dI\26m;\31W\21\346qgI\226d"
  "I\226\264\14\212%\313\251\0X\21\346qgI\26%a%\312\222,\311r*\0Y\20\346qgI"
  "\226dI\26\15ie\310\311\0Z\14\346q\17j\330\65\35t*\0[\13\346\361\14Q\332\257C\16"
  "\3\134\15\346q\244q\32\247q\32\247\71\14]\12\346\361\14i\177\32r\30^\12\346\361#a\22e"
  "\71\77_\11\346\361\363\353\240\303\0`\11\346\361\3q\235_\0a\16\346\361S\347hH\262$\213\206"
  "\234\12b\20\346q\247\351\20eI\226dI\226\14\71\31c\14\346\361S\207$m\36r*\0d\21"
  "\346\361ci\64$Y\222%Y\222ECN\5e\17\346\361S\207$K\262dP\342!\247\2f\14"
  "\346\361#S\32\16Y\332\316\2g\21\346\361S\207$K\262$K\262hN\206\34\1h\20\346q\247"
  "\351\20eI\226dI\226d\71\25i\13\346\361#\71\246v\325\311\0j\13\346\361C\71\230\366\246S"
  "\0k\16\346q\247\245J&&YT\313\251\0l\12\346\361\3i\237u\62\0m\15\346\361\23\207("
  "\351\337\222,\247\2n\20\346\361\23\207(K\262$K\262$\313\251\0o\16\346\361S\247,\311\222,"
  "\311\242\235\14p\21\346\361\23\207(K\262$K\262d\210\322*\0q\20\346\361S\207$K\262$K"
  "\262hH[\0r\14\346\361S\207$K\322v&\0s\15\346\361S\207$\236\323d\310\311\0t\13"
  "\346\361\3i\70\246\315:\31u\20\346\361\23\263$K\262$K\262h\310\251\0v\16\346\361\23\263$"
  "K\262$\213\222\60gw\17\346\361\23\263$KZ\6\305\222\345T\0x\16\346\361\23\263$\213\266)"
  "K\262\234\12y\22\346\361\23\263$K\262$K\262hH\223!G\0z\14\346\361\23\7\65l\34t"
  "*\0{\14\346\361iiM\224\323\262\16\3|\10\346q\245\375;\5}\14\346\361\310iY\324\322\232"
  "N\1~\12\346\361s\213\222D\347\10\177\7\346\361\363\237\0\200\6\341\311\243\0\201\6\341\311\243\0\202"
  "\6\341\311\243\0\203\6\341\311\243\0\204\6\341\311\243\0\205\6\341\311\243\0\206\6\341\311\243\0\207\6\341"
  "\311\243\0\210\6\341\311\243\0\211\6\341\311\243\0\212\6\341\311\243\0\213\6\341\311\243\0\214\16\346\361e"
  "C\222\306sZ\31r\62\0\215\6\341\311\243\0\216\6\341\311\243\0\217\14\346qe\203T\354\232\16:"
  "\25\220\6\341\311\243\0\221\6\341\311\243\0\222\6\341\311\243\0\223\6\341\311\243\0\224\6\341\311\243\0\225"
  "\6\341\311\243\0\226\6\341\311\243\0\227\16\346\361eC\222\306sZ\31r\62\0\230\6\341\311\243\0\231"
  "\6\341\311\243\0\232\6\341\311\243\0\233\6\341\311\243\0\234\16\346\361\205\71\66$\361\234&CN\6\235"
  "\6\341\311\243\0\236\21\346\361#%i\210\6eP\206(\221*\71\25\237\15\346\361\205\71\64\250a\343"
  "\240S\1\240\7\346\361\363\237\0\241\23\346\361\3S\226dI\226\14J\226dI\26\306\71\0\242\21\346"
  "\361\23\302!\251%Y\222%\341\220\345\24\0\243\14\346q\247-\231\230\306CN\5\244\22\346\361\3S"
  "\226dI\226\14J\226dI\26\346\4\245\22\346\361\3S\226dI\226\14J\226dI\26\346\4\246\16"
  "\346\361eC\222\306sZ\31r\62\0\247\17\346\361#Z\224\245Z\324\233\232E\231\4\250\11\346\361\3"
  "I\316\237\1\251\21\346\361\3C\22J\211\22)\221bL\206\234\12\252\15\346\361#r\66\325vd\310"
  "\31\1\253\17\346\361\223\243$J\242\266(\213r\42\0\254\14\346qe\203T\354\232\16:\25\255\10\346"
  "\361s\333y\3\256\21\346\361\3C\22*\226d\261$c\62\344T\0\257\14\346qe\203\32vM\7"
  "\235\12\260\12\346\361#Z\324\246\363\11\261\20\346\361S\347hH\262$\213\206\64\314\21\0\262\14\346\361"
  "#Z\224\206\305!\347\6\263\13\346\361\3i\252\251\315:\31\264\11\346\361Ca\235\337\0\265\14\346\361"
  "\23\243\376i\251\346 \0\266\16\346\361\205\71\66$\361\234&CN\6\267\10\346\361s\314y\4\270\11"
  "\346\361\363\207\64\14\1\271\20\346\361S\347hH\262$\213\206\64\314\21\0\272\15\346\361#Z\324\233\16"
  "\15\71#\0\273\17\346\361\23\243,\312\242\226(\211r\62\0\274\15\346\361\205\71\64\250a\343\240S\1"
  "\275\17\346\361\204j-\211\302\26\245\24\26\207\0\276\21\346\361hQ\30'\222\64\206ZR\33\302\64\1"
  "\277\15\346\361#\71\64\250a\343\240S\1\300\21\346\361\304\341\224%Y\62(Y\222%YN\5\301\21"
  "\346\361\205\341\224%Y\62(Y\222%YN\5\302\22\346q\205I\66eI\226\14J\226dI\226S"
  "\1\303\23\346\361DI\242MY\222%\203\222%Y\222\345T\0\304\20\346\361S\347hH\262$\213\206"
  "\64\314\21\0\305\16\346\361eC\222\306sZ\31r\62\0\306\14\346\361eC\222\366<\344T\0\307\15"
  "\346\361\3C\222\366<di\30\2\310\17\346\361\304\341\220\244\351\20\245\361\220S\1\311\17\346\361\205\341"
  "\220\244\351\20\245\361\220S\1\312\20\346\361\3C\222\246C\224\226\207\64\314\21\0\313\17\346\361\324\241!"
  "I\323!J\343!\247\2\314\13\346\361\304\341\230v\334\311\0\315\13\346\361\205\341\230v\334\311\0\316\14"
  "\346q\205I\66\246\35w\62\0\317\13\346\361\324\241\61\355\270\223\1\320\15\346\361\3[\324\262D}\332"
  "\311\0\321\20\346\361EIV\221\22)\351'%\322\251\0\322\20\346\361\304\341\224%Y\222%Y\222E"
  ";\31\323\20\346\361\205\341\224%Y\222%Y\222E;\31\324\21\346q\205I\66eI\226dI\226d"
  "\321N\6\325\22\346\361DI\242MY\222%Y\222%Y\264\223\1\326\21\346\361\324\241)K\262$K"
  "\262$\213v\62\0\327\14\346\361S\243L\324\242\234\33\0\330\20\346qFS\226DJ_\244$\213\246"
  "\234\6\331\21\346\361\304Y%K\262$K\262$\213\206\234\12\332\21\346\361\205Y%K\262$K\262$"
  "\213\206\234\12\333\23\346q\205I\224%Y\222%Y\222%Y\64\344T\0\334\22\346\361\324\221,\311\222"
  ",\311\222,\311\242!\247\2\335\17\346\361\205Y%K\262hH+CN\6\336\21\346\361\243\351\20e"
  "I\226dI\226\14QN\3\337\17\346\361\3Z\324%\213j\211\224$:\31\340\20\346q\305\71\64G"
  "C\222%Y\64\344T\0\341\20\346\361\205\71\66GC\222%Y\64\344T\0\342\11\346\361Ca\235\337"
  "\0\343\21\346\361DI\242Cs\64$Y\222ECN\5\344\20\346\361\3I\16\315\321\220dI\26\15"
  "\71\25\345\20\346q\205I\30\316\321\220dI\26\15\71\25\346\15\346\361Ca\70$i\363\220S\1\347"
  "\15\346\361S\207$m\36\262\64\14\1\350\20\346q\305\71\64$Y\222%\203\22\17\71\25\351\20\346\361"
  "\205\71\66$Y\222%\203\22\17\71\25\352\20\346\361S\207$K\262dP\342!\254C\0\353\21\346\361"
  "\3I\16\15I\226d\311\240\304CN\5\354\13\346q\305\71\244v\325\311\0\355\13\346\361\205\71\246v"
  "\325\311\0\356\14\346q\205I\16\251]u\62\0\357\14\346\361\3I\16\251]u\62\0\360\21\346q$"
  "a%\234\262$K\262$\213v\62\0\361\21\346\361\205\71\64DY\222%Y\222%YN\5\362\20\346"
  "q\305\71\64eI\226dI\26\355d\0\363\20\346\361\205\71\66eI\226dI\26\355d\0\364\20\346"
  "q\205I\16MY\222%Y\222E;\31\365\21\346\361c\222\222HI\226dI\66\15\221N\4\366\20"
  "\346\361\3I\16MY\222%Y\222E;\31\367\13\346\361\223sh\320\241\234\31\370\17\346\361\223\242)"
  "RZ\244$\213\246\234\6\371\21\346q\305\71\222%Y\222%Y\222ECN\5\372\21\346\361\205\71\224"
  "%Y\222%Y\222ECN\5\373\22\346q\205I\216dI\226dI\226d\321\220S\1\374\22\346\361"
  "\3I\216dI\226dI\226d\321\220S\1\375\23\346\361\205\71\224%Y\222%Y\222ECZ\31\42"
  "\0\376\22\346q\247\351\20eI\226dI\226\14Q\232\203\0\377\23\346\361\3I\216dI\226dI\226"
  "d\321\220V\206\10\0\0\0\4\377\377\0"
;

//Font converted to momospace to support codec and bitrate strings (e.g.: MP3/AAC/FLAC)
const uint8_t mono04b03b[912] U8G2_FONT_SECTION("mono04b03b") = 
  "`\1\3\3\3\3\1\1\4\5\6\0\377\5\377\5\0\1$\2c\3s \6s{D\0!\10r"
  "\212HZ\14\0\42\10t\214Hv\64\0#\14v\236\244R$T\212\304!\0$\12u\235I\22)"
  "\22\231\3%\14v\16QD\22L\221\204\344\0&\13v\216Y\264\22\12\321!\0'\7r\212H\34"
  "\4(\10s\233\244\264 \0)\10s\213X(%\12*\11t\214H(%\16\6+\10t\334\320("
  "\16\3,\7s{p$\4-\7t|\310\34\14.\7rzH\14\0/\7v\316\274\3\1\60\14"
  "u\15J(\22\212\204\42d\0\61\10u\35a\266\61\0\62\11u\15b\204\22$\3\63\11u\15b"
  "\204\30!\3\64\14u\215P$\24\11E\210a\0\65\11u\15J\220\30!\3\66\12u\215Q\220\22"
  "\212\220\1\67\11u\15b,\61\16\1\70\13u\15J(B\11E\310\0\71\12u\15J(B\14\215"
  "\1:\7r\252X\24\0;\7r\252X$\6<\7t\254\214\251\0=\7t\314\351\34\4>\10t"
  "\214`R:\0\77\12u\15bh\16\210C\0@\14v\216J,\22\231d\241C\0A\14u\15J"
  "(B\11EBa\0B\13u\215Q$D\11E\310\0C\10u\15J\60\221\14D\13u\215QJ"
  "(\22\212\314\1E\11u\15J\220\22$\3F\12u\15J\220\22\214\203\0G\13u\15J\60\42\11"
  "E\310\0H\15u\215P$\24\241\204\42\241\60\0I\10u\235Y\60m\14J\11u-aJ(B"
  "\6K\14u\215P$\24\31\245\204\302\0L\10u\215`F\62\0M\10v\216J\376\357\0N\15u"
  "\215PD\222\42\11EBa\0O\14u\15J(\22\212\204\42d\0P\14u\15J(\22\212P\342"
  " \0Q\14u\15J(\22\212D$d\0R\13u\15J(BI\212\210\1S\11u\15J\220\30"
  "!\3T\10t\214Q,\63\0U\15u\215P$\24\11EB\21\62\0V\15u\215P$\24I\212"
  "\304\342\20\0W\11v\216H\376\227:\0X\14u\215P$\24\22\245\204\302\0Y\13u\215P$\24"
  "!F\310\0Z\11u\15bH\24$\3[\10s\13I(I\10\134\7v\216p\356\0]\10s\13"
  "Q\26!\0^\10t\234P$\216\6_\7u}d\62\0`\7s\213X\34\14a\11u}\340$"
  "\24!\3b\11u\335 %\24!\3c\10t|\310$\66\5d\12u}H\204\22\212\220\1e\10"
  "u}\30%m\14f\11u\355Q\214\24\207\0g\12u}\30%\24\241\205\0h\12u\335 %\24"
  "\11\205\1i\7r\252X$\6j\10r\252X$\5\0k\11u\335`(\62J\6l\7r\252H"
  "\66\0m\10v~h%\337\1n\12u}\30%\24\11\205\1o\11u}\30%\24!\3p\12u"
  "}\30%\24\241\4\1q\12u}\30%\24!F\0r\10t|\310$\26\7s\10u}\30)F"
  "\6t\10t\334\320(&\5u\12u}X(\22\212\220\1v\12u}X(\22\12\311\1w\11v"
  "~h$/u\0x\11t|HRJ\24\0y\13u}X(\22\212\20#\0z\10u}\30-"
  "D\6{\10t\34QbL\12|\7r\212Hn\0}\11t\14Y\60\24\22\3~\7u\235\334\261"
  "\0\177\7t\14\221\316\0\0\0\0\4\377\377\0"
;

// SD card icon displayed when no card is present during startup
static unsigned char sdcard[] PROGMEM = {
  0xf0, 0xff, 0xff, 0x0f, 0xf8, 0xff, 0xff, 0x1f, 0xf8, 0xcf, 0xf3, 0x3f,
  0x38, 0x49, 0x92, 0x3c, 0x38, 0x49, 0x92, 0x3c, 0x38, 0x49, 0x92, 0x3c,
  0x38, 0x49, 0x92, 0x3c, 0x38, 0x49, 0x92, 0x3c, 0x38, 0x49, 0x92, 0x3c,
  0x38, 0x49, 0x92, 0x3c, 0xf8, 0xff, 0xff, 0x3f, 0xf8, 0xff, 0xff, 0x3f,
  0xf8, 0xff, 0xff, 0x3f, 0xf8, 0xff, 0xff, 0x3f, 0xf8, 0xff, 0xff, 0x3f,
  0xfc, 0xff, 0xff, 0x3f, 0xfe, 0xff, 0xff, 0x3f, 0xff, 0xff, 0xff, 0x3f,
  0xff, 0xff, 0xff, 0x3f, 0xff, 0xff, 0xff, 0x3f, 0xff, 0xff, 0xff, 0x3f,
  0xfc, 0xff, 0xff, 0x3f, 0xfc, 0xc0, 0x80, 0x3f, 0x7c, 0xc0, 0x00, 0x3f,
  0x3c, 0xc0, 0x00, 0x3e, 0x1c, 0xfc, 0x38, 0x3e, 0x1e, 0xfc, 0x78, 0x3c,
  0x1f, 0xe0, 0x78, 0x3c, 0x3f, 0xc0, 0x78, 0x3c, 0x7f, 0x80, 0x78, 0x3c,
  0xff, 0x87, 0x78, 0x3c, 0xff, 0x87, 0x38, 0x3e, 0x3f, 0x80, 0x00, 0x3e,
  0x1f, 0xc0, 0x00, 0x3f, 0x3f, 0xe0, 0x80, 0x3f, 0xff, 0xff, 0xff, 0x3f,
  0xff, 0xff, 0xff, 0x3f, 0xff, 0xff, 0xff, 0x3f, 0xfe, 0xff, 0xff, 0x1f,
  0xfc, 0xff, 0xff, 0x0f 
};


// NOTE image displayed at startup
#define notes_width 256
#define notes_height 46
static unsigned char notes[] PROGMEM = {
  0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x80, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x0c, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x40, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x0c, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x20, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x0e, 0x00,
  0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x0e, 0x00, 0x02, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x20, 0x07, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xa0, 0x07, 0x00,
  0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x03, 0x80, 0x05, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00,
  0x00, 0xf0, 0x01, 0xc0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0xf8, 0x00, 0x40,
  0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0x00, 0x08,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00,
  0x00, 0x01, 0x00, 0x00, 0x00, 0x7c, 0x00, 0x40, 0x08, 0x00, 0x00, 0x00,
  0x00, 0x60, 0x00, 0x00, 0x00, 0x5e, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00,
  0x60, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00,
  0x00, 0x5e, 0x00, 0x80, 0x08, 0x00, 0x00, 0x07, 0x00, 0xe0, 0x01, 0x00,
  0xc0, 0x47, 0x00, 0x08, 0x00, 0x00, 0x07, 0x00, 0xe0, 0x01, 0x00, 0x00,
  0x40, 0x00, 0x18, 0x00, 0x80, 0x00, 0x00, 0x0e, 0x00, 0x4e, 0x00, 0x00,
  0x19, 0x00, 0xf0, 0x07, 0x00, 0x20, 0x01, 0x00, 0xf8, 0x41, 0x00, 0x18,
  0x00, 0xf0, 0x07, 0x00, 0x20, 0x01, 0x00, 0x18, 0x40, 0x00, 0x78, 0x00,
  0x80, 0x00, 0xe0, 0x0f, 0x00, 0x47, 0x00, 0x00, 0x10, 0x00, 0xf0, 0x07,
  0x00, 0x20, 0x03, 0x00, 0x3c, 0x40, 0x00, 0x10, 0x00, 0xf0, 0x07, 0x00,
  0x20, 0x03, 0x00, 0x1c, 0x40, 0x00, 0xe8, 0x00, 0x80, 0x00, 0xe0, 0x0f,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 0x41, 0x00, 0x00,
  0x10, 0x00, 0x3f, 0x04, 0x00, 0xe0, 0x04, 0xe0, 0x11, 0x40, 0x00, 0x10,
  0x00, 0x3f, 0x04, 0x00, 0xe0, 0x04, 0xe0, 0x11, 0x40, 0x00, 0x38, 0x01,
  0x40, 0x00, 0x7e, 0x08, 0xc0, 0xe0, 0x0f, 0x00, 0x10, 0x00, 0x0f, 0x04,
  0x00, 0xb0, 0x01, 0x78, 0x10, 0x40, 0x00, 0x10, 0x00, 0x0f, 0x04, 0x00,
  0xb0, 0x01, 0x78, 0x10, 0x40, 0x00, 0x68, 0x01, 0x40, 0x00, 0x1e, 0x08,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x40, 0x70, 0x3e, 0x00,
  0x1c, 0x00, 0x01, 0x04, 0x00, 0x10, 0x00, 0x0e, 0x10, 0x40, 0x00, 0x18,
  0x00, 0x01, 0x04, 0x00, 0x10, 0x00, 0x0e, 0x10, 0x40, 0x00, 0xa8, 0x00,
  0x2f, 0x00, 0x02, 0x08, 0x40, 0x98, 0x78, 0x00, 0x1f, 0x00, 0x01, 0x04,
  0x00, 0x10, 0x00, 0x06, 0x10, 0x40, 0x00, 0x1f, 0x00, 0x01, 0x04, 0x00,
  0x10, 0x00, 0x06, 0x10, 0x40, 0x00, 0x48, 0xc0, 0x1f, 0x00, 0x02, 0x08,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x40, 0x88, 0x70, 0x80,
  0x0f, 0x00, 0x01, 0x04, 0x80, 0x17, 0x00, 0x02, 0x10, 0x7c, 0x80, 0x0f,
  0x00, 0x01, 0x04, 0x80, 0x17, 0x00, 0x02, 0x10, 0x7c, 0x00, 0x08, 0x80,
  0x0f, 0x00, 0x02, 0x08, 0x40, 0x88, 0x70, 0x00, 0x07, 0x00, 0x01, 0x04,
  0xc0, 0x1f, 0x00, 0x02, 0x10, 0x7e, 0x00, 0x07, 0x00, 0x01, 0x04, 0xc0,
  0x1f, 0x00, 0x02, 0x10, 0x7e, 0x80, 0x0f, 0x00, 0x00, 0x00, 0x02, 0x08,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 0x00, 0x71, 0x00,
  0x00, 0x00, 0xc1, 0x07, 0xe0, 0x07, 0x00, 0x02, 0x1e, 0x00, 0x00, 0x00,
  0x00, 0xc1, 0x07, 0xe0, 0x07, 0x00, 0x02, 0x1e, 0x00, 0xc0, 0x07, 0x00,
  0x00, 0x00, 0x82, 0x0f, 0x80, 0x01, 0x39, 0x00, 0x00, 0x00, 0xe1, 0x07,
  0xc0, 0x03, 0x00, 0x02, 0x1f, 0x00, 0x00, 0x00, 0x00, 0xe1, 0x07, 0xc0,
  0x03, 0x00, 0x02, 0x1f, 0x00, 0x80, 0x03, 0x00, 0x00, 0x00, 0xc2, 0x0f,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x0f, 0x1e, 0x00,
  0x00, 0xf0, 0x81, 0x03, 0x00, 0x00, 0x80, 0x03, 0x0e, 0x00, 0x00, 0x00,
  0xf0, 0x81, 0x03, 0x00, 0x00, 0x80, 0x03, 0x0e, 0x00, 0x00, 0x00, 0x00,
  0x00, 0xe0, 0x03, 0x07, 0x00, 0xfc, 0x0f, 0x00, 0x00, 0xf8, 0x01, 0x00,
  0x00, 0x00, 0xc0, 0x03, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x01, 0x00, 0x00,
  0x00, 0xc0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x03, 0x00,
  0x00, 0xf0, 0x03, 0x00, 0x00, 0xf8, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x03,
  0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x03, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00,
  0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x01, 0x00, 0x00, 0x00, 0x00,
  0x70, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x0c, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x02, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x0c, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x01, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};


// MSL-LSB <-> LSB-MSB bit inversion function needed for pilot command simulation 
uint32_t reverse_bits(uint32_t inval, int bits)
{
  if ( bits > 0 )
  {
    bits--;
    return reverse_bits(inval >> 1, bits) | ((inval & 1) << bits);
  }
  return 0;
}


/*---------- Port definition and variable declarations for IR receiver support ----------*/

// IR remote control operation variables
bool pulse_ready = false;                // Ready Signal Flag
unsigned long pulse_start_high = 0;      // Pulse start time
unsigned long pulse_end_high = 0;        // Pulse end time
unsigned long pulse_duration = 0;        // Pulse duration
unsigned long pulse_duration_9ms = 0;    // For analysis only - Pulse duration
unsigned long pulse_duration_4_5ms = 0;  // For analysis only - Pulse duration
unsigned long pulse_duration_560us = 0;  // For analysis only - Pulse duration
unsigned long pulse_duration_1690us = 0; // For analysis only - Pulse duration

bool pulse_ready9ms = false;            // Ready Signal Flag puls 9ms
bool pulse_ready_low = false;           // Ready Signal Flag 
unsigned long pulse_start_low = 0;      // Pulse start time
unsigned long pulse_end_low = 0;        // Pulse end time
unsigned long pulse_duration_low = 0;   // Pulse duration


volatile unsigned long ir_code = 0;  // Variable to store IR code
volatile int bit_count = 0;          // Bit counter in the received code

// Switching thresholds for IR remote control timing signals
const int LEAD_HIGH = 9050;         // 9 ms high signal (initial)
const int LEAD_LOW = 4500;          // 4.5 ms low signal (initial)
const int TOLERANCE = 150;          // Tolerance (in microseconds)
const int HIGH_THRESHOLD = 1690;    // Signal "1"
const int LOW_THRESHOLD = 600;      // Signal "0"

bool data_start_detected = false;  // Flag for preliminary signal
bool rcInputDigitsMenuEnable = false;

//================ Definition of ports and pins for the numeric keypad ===========================//
const int keyboardPin = 9;  // keyboard input (ADC) 

unsigned long keyboardValue = 0;
unsigned long keyboardLastSampleTime = 0;
unsigned long keyboardSampleDelay = 50;


// ---- ADC switching thresholds for the 5x3 matrix keyboard in the Sony ST-120 tuner ---- //
int keyboardButtonThresholdTolerance = 20; // ADC measurement tolerance
int keyboardButtonNeutral = 4095;          // Neutral position
int keyboardButtonThreshold_0 = 2375;      // Button 0
int keyboardButtonThreshold_1 = 10;        // Button 1
int keyboardButtonThreshold_2 = 545;       // Button 2
int keyboardButtonThreshold_3 = 1390;      // Button 3
int keyboardButtonThreshold_4 = 1925;      // Button 4
int keyboardButtonThreshold_5 = 2285;      // Button 5
int keyboardButtonThreshold_6 = 385;       // Button 6
int keyboardButtonThreshold_7 = 875;       // Button 7
int keyboardButtonThreshold_8 = 1585;      // Button 8
int keyboardButtonThreshold_9 = 2055;      // Button 9
int keyboardButtonThreshold_Ok = 2455;  // Shift - Enter/OK function
int keyboardButtonThreshold_BankMenu = 2170; // Memory - Bank menu function
int keyboardButtonThreshold_Back = 1640;   // Band button - Back function
int keyboardButtonThreshold_DisplayMode = 730;    // Auto button - switches between Radio/Clock
int keyboardButtonThreshold_Dimmer = 1760;   // Scan button - OLED screen dimmer function
int keyboardButtonThreshold_Mute = 1130;   // Mute button - MUTE function

int keyboardButtonThreshold_ArrowLeft = 4100;   // Left arrow (previous station, bank)
int keyboardButtonThreshold_ArrowRight = 4100;  // Right arrow (next station, tank)
int keyboardButtonThreshold_ArrowUp = 4100;     // Up arrow (scroll list)
int keyboardButtonThreshold_ArrowDown = 4100;   // Down arrow (scroll list)

int keyboardButtonThreshold_PresetsOn = 4100;  // Switching between saving stations and presets

// Flags for monitoring keyboard status
bool keyboardButtonPressed = false; // Pressing a key
bool debugKeyboard = false;         // Disables function calls and leaves only the ADC measurement printout
bool adcKeyboardEnabled = false;    // Flag enabling ADC keyboard operation

// ===== Function PROTOTYPES =====
void displayDimmer(bool dimmerON);
void displayDimmerTimer();
void displayPowerSave(bool mode);
void setPreset(uint8_t preset);
void displayInfo();

#ifdef IR_LED
  void irLedBlink();
#endif


#ifdef AUTOSTORAGE
  #define STORAGE_BEGIN() initStorage()
  bool initStorage() 
  {
    if (SD.begin(SD_CS, customSPI)) 
    {
      _storage = &SD;
      useSD = true;
      storageTextName = "SD";
      Serial.println("debug STORAGE -> We use SD cards");
      //#define SD_LED 17          // LED - CS signaling, SD card activity, pin clone useful when using the rear SD reader on the PCB
      noSDcard = false;
      return true;
    }

    Serial.println("debug STORAGE -> No SD card, switching to LittleFS");
    noSDcard = true; // No SD card flag

    #ifdef USE_LittleFS
    if (LittleFS.begin(true)) 
    {
      _storage = &LittleFS;
      useSD = false;
      storageTextName = "LittleFS";
      noSDcard = true;
      return true;
    }
    #endif
    
    Serial.println("debug STORAGE -> No SD card, switching to SPIFFS");
    noSDcard = true; // No SD card flag

    #ifdef USE_SPIFFS
    if (SPIFFS.begin(true)) 
    {
      _storage = &SPIFFS;
      useSD = false;
      storageTextName = "SPIFFS";
      noSDcard = true;
      return true;
    }
    #endif

    Serial.println("debug SD -> ERROR: no file system, writing only basic values ​​to EEPROM");
    storageTextName = "EEPROM";
    return false;
  }
#endif

/*
void mDnsRestart()
{
  if (!MDNS.isRunning()) 
  {
    Serial.println("debug net -> Error: mDNS not working");
    MDNS.end();                // keeps old mDNS
    delay(50);
    if (MDNS.begin(hostname)) {Serial.println("debug net -> mDNS restarted!");} 
    else {Serial.println("debug net -> mDNS restart error");}
  } 
  else 
  {
    Serial.println("debug net -> mDNS is working!");
  }

}
*/

void updateStandbyLED()
{
  //digitalWrite(STANDBY_LED, HIGH);
  digitalWrite(STANDBY_LED, !digitalRead(STANDBY_LED));
}

void switchOffstartupLED()
{
  digitalWrite(STANDBY_LED, LOW); 
}

void removeUtf8Bom(String &text) 
{
  if (text.length() >= 3 &&
      (uint8_t)text[0] == 0xEF &&
      (uint8_t)text[1] == 0xBB &&
      (uint8_t)text[2] == 0xBF) {
        text.remove(0, 3);  // usuń bajty EF BB BF
      }
}

// The function processes the text, replacing Polish diacritics

void processText(String &text) 
{
  removeUtf8Bom(text); // We remove the BOM characters at the beginning of the song name

  for (int i = 0; i < text.length() - 1; i++) 
  {
    if (i + 1 >= text.length()) break; // protection against going beyond the buffer

    switch ((uint8_t)text[i]) 
    {
      case 0xC3:
        switch ((uint8_t)text[i + 1]) {
          case 0xB3: text.setCharAt(i, 0xF3); break;  // ó
          case 0x93: text.setCharAt(i, 0xD3); break;  // Ó
        }
        text.remove(i + 1, 1);
        break;

      case 0xC4:
        switch ((uint8_t)text[i + 1]) {
          case 0x85: text.setCharAt(i, 0xB9); break;  // ą
          case 0x84: text.setCharAt(i, 0xA5); break;  // Ą
          case 0x87: text.setCharAt(i, 0xE6); break;  // ć
          case 0x86: text.setCharAt(i, 0xC6); break;  // Ć
          case 0x99: text.setCharAt(i, 0xEA); break;  // ę
          case 0x98: text.setCharAt(i, 0xCA); break;  // Ę
        }
        text.remove(i + 1, 1);
        break;

      case 0xC5:
        switch ((uint8_t)text[i + 1]) {
          case 0x82: text.setCharAt(i, 0xB3); break;  // ł
          case 0x81: text.setCharAt(i, 0xA3); break;  // Ł
          case 0x84: text.setCharAt(i, 0xF1); break;  // ń
          case 0x83: text.setCharAt(i, 0xD1); break;  // Ń
          case 0x9B: text.setCharAt(i, 0x9C); break;  // ś
          case 0x9A: text.setCharAt(i, 0x8C); break;  // Ś
          case 0xBA: text.setCharAt(i, 0x9F); break;  // ź
          case 0xB9: text.setCharAt(i, 0x8F); break;  // Ź
          case 0xBC: text.setCharAt(i, 0xBF); break;  // ż
          case 0xBB: text.setCharAt(i, 0xAF); break;  // Ż
        }
        text.remove(i + 1, 1);
        break;
    }
  }
}

bool sanitizeUtf8(String& s)
{
  String out;
  out.reserve(s.length() * 2);

  const uint8_t* p = (const uint8_t*)s.c_str();

  while (*p)
  {
    uint8_t c = *p;

    // ASCII
    if (c < 0x80) {
      out += (char)c;
      p++;
      continue;
    }

    // Valid UTF-8 (2–4 bytes)
    uint8_t len = 0;
    if ((c & 0xE0) == 0xC0) len = 2;
    else if ((c & 0xF0) == 0xE0) len = 3;
    else if ((c & 0xF8) == 0xF0) len = 4;

    if (len) {
      bool ok = true;
      for (uint8_t i = 1; i < len; i++) {
        if ((p[i] & 0xC0) != 0x80) {
          ok = false;
          break;
        }
      }
      if (ok) {
        for (uint8_t i = 0; i < len; i++)
          out += (char)p[i];
        p += len;
        continue;
      }
    }

    // Win-1250 na UTF-8 (PL)
    switch (c)
    {
      case 0xA5: out += "Ą"; break;
      case 0xB9: out += "ą"; break;
      case 0xC6: out += "Ć"; break;
      case 0xE6: out += "ć"; break;
      case 0xCA: out += "Ę"; break;
      case 0xEA: out += "ę"; break;
      case 0xA3: out += "Ł"; break;
      case 0xB3: out += "ł"; break;
      case 0xD1: out += "Ń"; break;
      case 0xF1: out += "ń"; break;
      case 0xD3: out += "Ó"; break;
      case 0xF3: out += "ó"; break;
      case 0xA6: out += "Ś"; break;
      case 0xB6: out += "ś"; break;
      case 0xAF: out += "Ż"; break;
      case 0xBF: out += "ż"; break;
      case 0xAC: out += "Ź"; break;
      case 0xBC: out += "ź"; break;
      case 0x8C: out += "Ś"; break;
      case 0x9C: out += "ś"; break;
      case 0x8F: out += "Ź"; break;
      case 0x9F: out += "ź"; break;
      default:   Serial.print("debug - > UTF8 Web Sanitizer, Unknown sign:"); Serial.println(c, HEX); out += '?'; break;
    }

    p++;
  }

  if (out != s) {
    s = out;
    return true;   // something has been improved
  }
  return false;    // it was OK
}

void wsStationChange(uint8_t stationId, uint8_t bankId) 
{
  // iterating over all connected clients
  for (auto& client : ws.getClients()) 
  {
 
    client.text("stationname:" + String(stationName.substring(0, stationNameLenghtCut)));

    if (urlPlaying) 
    {
      client.text("bank:" + String(0));
      client.text("station:" + String(0));
    } 
    else 
    {
      client.text("bank:" + String(bankId));
      client.text("station:" + String(stationId));
    }

    client.text("volume:" + String(volumeValue));
    client.text("mute:" + String(volumeMute)); 

    client.text("bitrate:" + String(bitrateString)); 
    client.text("samplerate:" + String(sampleRateString)); 
    client.text("bitpersample:" + String(bitsPerSampleString)); 

    if (mp3)    client.text("streamformat:MP3");
    if (flac)   client.text("streamformat:FLAC");
    if (aac)    client.text("streamformat:AAC");
    if (vorbis) client.text("streamformat:VORBIS");
    if (opus)   client.text("streamformat:OPUS");
    
    

  }
}

void wsRefreshPage() // Page refresh function using WebSocket
{
  ws.textAll("reload");  // send a message to all customers
}

void wsStreamInfoRefresh()
{
  // iterating over all connected clients
  for (auto& client : ws.getClients()) 
  {
 
    client.text("stationname:" + String(stationName.substring(0, stationNameLenghtCut)));

    if (audio.isRunning() == true)
    {  
      sanitizeUtf8(stationStringWeb);
      client.text("stationtext$" + stationStringWeb); 
    }
    else
    {
      client.text("stationtext$... No audio stream ! ...");
    }

    /*
    if (urlPlaying) 
    {
      client.text("bank:" + String(0));
      client.text("station:" + String(0));
    } 
    else 
    {
      client.text("bank:" + String(bank_nr));
      client.text("station:" + String(station_nr));
    }
    */

    //client.text("volume:" + String(volumeValue));
    //client.text("mute:" + String(volumeMute)); 

    client.text("bitrate:" + String(bitrateString)); 
    client.text("samplerate:" + String(sampleRateString)); 
    client.text("bitpersample:" + String(bitsPerSampleString)); 

    if (mp3)    client.text("streamformat:MP3");
    if (flac)   client.text("streamformat:FLAC");
    if (aac)    client.text("streamformat:AAC");
    if (vorbis) client.text("streamformat:VORBIS");
    if (opus)   client.text("streamformat:OPUS");

  }
}

void wsVolumeChange()  
{
  ws.textAll("volume:" + String(volumeValue)); // sends the volume value to all connected clients
  ws.textAll("mute:" + String(volumeMute)); // sends the mute value to all connected clients
}

void wsIrCodeClear()
{
  for (auto& client : ws.getClients()) 
  {
    client.text("learn:" + String(remoteConfig));
    client.text(String("clear"));
  }
}

// WebSockets for learning remote control
void wsIrCode(uint16_t code, uint8_t add, uint8_t cm)
{
  char buf1[7];                    
  char buf2[5];
  char buf3[5];

  sprintf(buf1, "0x%04X", code); // Formatowanie "0xFFFF" + \0
  sprintf(buf2, "0x%02X", add);
  sprintf(buf3, "0x%02X", cm);

  Serial.print("wsIRCode:"); Serial.println(buf1);
  //Serial.print("addr:"); Serial.print(buf2);
  //Serial.print(" cmd:"); Serial.println(buf3);

  for (auto& client : ws.getClients()) 
  {
    client.text(String("ircode:") + buf1);
    client.text(String("addr:") + buf2);
    client.text(String("cmd:") + buf3);
    client.text("learn:" + String(remoteConfig));
    
    //client.text("pulse9" + String(pulse_duration_9ms));
    //client.text("pulse45:" + String(pulse_duration_4_5ms));
    //client.text("pulse1690:" + String(pulse_duration_1690us));
    //client.text("pulse560:" + String(pulse_duration_560us));
  }
}

void wsSignalLevel()
{
  ws.textAll("wifi:" + String(wifiSignalLevel)); // sends the Wi-Fi signal value to all connected clients
}

//The function responsible for saving station information to the PSRAM memory
void saveStationToPSRAM(const char *station) 
{
  
  // Check if there is still room for another station in the PSRAM memory
  if (stationsCount < MAX_STATIONS) {
    int length = strlen(station);

    // Check that the link length does not exceed the established maximum.
    if (length <= STATION_NAME_LENGTH) {
      // Zapisz długość linku jako pierwszy bajt.
      psramData[stationsCount * (STATION_NAME_LENGTH + 1)] = length;
      // Save the link as consecutive bytes in PSRAM
      for (int i = 0; i < length; i++) 
      {
        psramData[stationsCount * (STATION_NAME_LENGTH + 1) + 1 + i] = station[i];
      }

      // Print information about the saved station on Serial.
      Serial.println(String(stationsCount + 1) + "   " + String(station));  // Printing on the serial from number 1 like in a bank on the server

      // Increase the saved stations counter.
      stationsCount++;

      // progress bar of downloaded stations
      u8g2.setFont(spleen6x12PL);  
      u8g2.drawStr(21, 36, "Progress:");
      //u8g2.drawStr(21, 36, "Loading: ");
      u8g2.drawStr(75, 36, String(stationsCount).c_str());  // Write the counter of downloaded stations

      u8g2.drawRFrame(21, 42, 212, 12, 3);  // Station loading progress bar frame w>8 h>8
      
      int x = (stationsCount * 2) + 8;          // We add when stationCount=1 + 8 to maintain the condition for rounded drawRBox - width W>6 h>6 has to be W>=2*(r+1), h >= 2*(r+1)
      u8g2.drawRBox(23, 44, x, 8, 2);       // Progress bar for loading the station from the server or SD card / SPIFFS       
      
      u8g2.sendBuffer();  
    } else {
      // Error message if the link to the station is too long.
      Serial.println("Error: Station link is too long");
    }
  } else {
    // Error message when the maximum number of stations is reached.
    Serial.println("Error: Maximum number of saved stations reached");
  }
}

// The function processes and saves the station to the PSRAM memory
void sanitizeAndSaveStation(const char *station) {
  // Buffer for processed station - one character longer than the maximum link length
  char sanitizedStation[STATION_NAME_LENGTH + 1];

  // Auxiliary index for processing
  int j = 0;

  // Browse through each station character and check if it is a printable ASCII character
  for (int i = 0; i < STATION_NAME_LENGTH && station[i] != '\0'; i++) {
    // Check if the character is a printable ASCII character
    if (isprint(station[i])) {
      // If so, add to processed station
      sanitizedStation[j++] = station[i];
    }
  }

  // Add a terminating character to the processed station
  sanitizedStation[j] = '\0';

  // Zapisz przetworzoną stację do pamięci PSRAM
  saveStationToPSRAM(sanitizedStation);
}

// Reading a bank from PIFFS or SD card (if the bank already exists on that card)
void readSDStations() 
{
  stationsCount = 0;
  Serial.println("debug SD -> The Bank file exists locally, we read ONLY from the card");
  mp3 = flac = aac = vorbis = opus = false;
  stationString.remove(0);  // Remove all characters from the stationString object

  // We create a bank file name
  String fileName = String("/bank") + (bank_nr < 10 ? "0" : "") + String(bank_nr) + ".txt";

  // We check if the file exists
  if (!STORAGE.exists(fileName)) 
  {
    Serial.println("Error: Bank file does not exist.");
    return;
  }

  // We open the file in read mode
  File bankFile = STORAGE.open(fileName, FILE_READ);
  if (!bankFile)  // if the file is missing then...
  {
    Serial.println("Error: Unable to open bank file.");
    return;
  }

  // We go to the appropriate line of the file
  int currentLine = 0;
  String stationUrl = "";

  while (bankFile.available())  // & currentLine <= MAX_STATIONS)
  {
    //if (currentLine < MAX_STATIONS)
    //{
    String line = bankFile.readStringUntil('\n');
    currentLine++;

    stationName = line.substring(0, 42);
    int urlStart = line.indexOf("http");  // We are looking for the place where the URL begins
    if (urlStart != -1) 
    {
      stationUrl = line.substring(urlStart);  // We extract the URL from "http"
      stationUrl.trim();                      // We remove whitespace at the beginning and end
      //Serial.print(" URL station:");
      //Serial.println(stationUrl);
      //String station = currentLine + "   " + stationName + "  " + stationUrl;
      String station = stationName + "  " + stationUrl;
      sanitizeAndSaveStation(station.c_str());  // rewriting the station to EEPROM (RAM)
    }
  }

  Serial.print("We close the bankFile file at the currentLine value:");
  Serial.println(currentLine);
  bankFile.close();  // We close the file after reading
}


void fetchStationsFromServer() 
{
  displayActive = true;
  u8g2.setFont(spleen6x12PL);
  u8g2.clearBuffer();
  u8g2.setCursor(21, 23);
  u8g2.print("Loading bank:" + String(bank_nr) + " from:");
  u8g2.sendBuffer();

  currentSelection = 0;
  firstVisibleLine = 0;
  station_nr = 1;
  previous_bank_nr = bank_nr; // if we load stations, we set the previous_bank variable

  HTTPClient http; // Create an HTTP client object
  String url; // Station URL for a given bank

  // ---------------------- BANK URL SELECTION ----------------------
  switch (bank_nr) 
  {
    case 1:  url = STATIONS_URL1; break;
    case 2:  url = STATIONS_URL2; break;
    case 3:  url = STATIONS_URL3; break;
    case 4:  url = STATIONS_URL4; break;
    case 5:  url = STATIONS_URL5; break;
    case 6:  url = STATIONS_URL6; break;
    case 7:  url = STATIONS_URL7; break;
    case 8:  url = STATIONS_URL8; break;
    case 9:  url = STATIONS_URL9; break;
    case 10: url = STATIONS_URL10; break;
    case 11: url = STATIONS_URL11; break;
    case 12: url = STATIONS_URL12; break;
    case 13: url = STATIONS_URL13; break;
    case 14: url = STATIONS_URL14; break;
    case 15: url = STATIONS_URL15; break;
    case 16: url = STATIONS_URL16; break;
    case 17: url = STATIONS_URL17; break;
    case 18: url = STATIONS_URL18; break;
    default:
      Serial.println("Incorrect bank account number");
      return;
  }

  // Creating a file name for a given bank
  String fileName = String("/bank") + (bank_nr < 10 ? "0" : "") + String(bank_nr) + ".txt";

  // ---------------------- IF FILE EXISTS ----------------------
  if (STORAGE.exists(fileName) && bankNetworkUpdate == false) 
  {
    Serial.println("debug SD -> Bank file " + fileName + " already exists.");
    //if (useSD) { u8g2.print("SD Card"); } else { u8g2.print("SPIFFS"); }
    u8g2.print(storageTextName); 
    u8g2.sendBuffer();

    readSDStations(); // If a given bank file exists, we read it ONLY from the card
    wsRefreshPage();
    return;
  }

  bankNetworkUpdate = false;

  u8g2.print("GitHub");
  u8g2.sendBuffer();

  // ---------------------- CREATING AN EMPTY FILE ----------------------
  {
    File bankFile = STORAGE.open(fileName, FILE_WRITE);
    if (bankFile) 
    {
      Serial.println("debug SD -> Bank file created: " + fileName);
      bankFile.close(); // Closing the file after creation
    } 
    else 
    {
      Serial.println("debug SD -> File creation error: " + fileName);
    }
  }

  // ---------------------- HTTP DATA DOWNLOAD ----------------------

  Serial.println("debug http -> Downloading URL: " + url);

  http.begin(url); // Initiate an HTTP request to the given URL
  http.setUserAgent("ESP32-WebRadio");   // userAgent settings
  
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  http.useHTTP10(true);                 // <<< THE MOST IMPORTANT
  http.addHeader("Connection", "close");
  
  http.setTimeout(5000);                // 5 second timeout
  
  http.setConnectTimeout(3000);

  int httpCode = http.GET();  
  Serial.print("debug http -> Code HTTP: ");
  Serial.println(httpCode); // Print additional information about the http code

  // ----------- SUPPORT 301 / 302 REDIRECTS (GitHub does this often!) -----------
  if (httpCode == HTTP_CODE_MOVED_PERMANENTLY || httpCode == HTTP_CODE_FOUND) // if GitHub has set a redirect
  {
    String newUrl = http.getLocation();
    Serial.println("debug http -> Redirect: " + newUrl);

    http.end();

    http.begin(newUrl);
    http.setUserAgent("ESP32-WebRadio");
  
    http.useHTTP10(true);                 // <<< THE MOST IMPORTANT
    http.addHeader("Connection", "close");
    
    http.setTimeout(5000);                // 5 second timeout
  
    http.setConnectTimeout(3000);

    httpCode = http.GET();
    Serial.println("debug http -> HTTP code after redirect: " + String(httpCode));
  }

  // ---------------------- DOWNLOAD FAILED ----------------------
  if (httpCode != HTTP_CODE_OK) 
  {
    Serial.printf("debug http -> Download error. HTTP code: %d\n", httpCode);
    http.end();
    return;
  }

  // ---------------------- GETSTRING DOWNLOAD ----------------------
  String payload = http.getString();

  Serial.println("debug http -> Length of downloaded data: " + String(payload.length()));

  // Protection against empty string
  if (payload.length() == 0) 
  {
    Serial.println("debug http -> ERROR! Download returned empty payload. Deleting file.");
    STORAGE.remove(fileName);
    http.end();
    return;
  }

  // ---------------------- WRITING TO FILE ----------------------
  File bankFile = STORAGE.open(fileName, FILE_WRITE);
  if (bankFile) 
  {
    bankFile.print(payload);
    bankFile.close();
    Serial.println("debug SD -> Data saved to file: " + fileName);
  } 
  else 
  {
    Serial.println("debug SD -> Error: Could not open file for writing!");
  }

  http.end();

  // ---------------------- SANITIZATION OF THE STATION  ----------------------
  int startIndex = 0;
  int endIndex;
  stationsCount = 0;

  while ((endIndex = payload.indexOf('\n', startIndex)) != -1 && stationsCount < MAX_STATIONS) 
  {
    String station = payload.substring(startIndex, endIndex);
    if (!station.isEmpty())
      sanitizeAndSaveStation(station.c_str());

    startIndex = endIndex + 1;
  }

  wsRefreshPage();
}


void readEEPROM() // DEBUG control function for reading EEPROM, not used by other functions
{
  EEPROM.get(0, station_nr);
  EEPROM.get(1, bank_nr);
  EEPROM.get(2, volumeValue); 
}


void calcNec() // A function that allows reverse conversion to "imitate" NEC IR remote control buttons.
{
  // we compose the remote control code in the form ADDR/CMD/CMD/ADDR to have 4 bytes
  uint8_t CMD = (ir_code >> 8) & 0xFF; 
  uint8_t ADDR = ir_code & 0xFF;        
  ir_code =  ADDR;
  ir_code = (ir_code << 8) | CMD;
  ir_code = (ir_code << 8) | CMD;
  ir_code = (ir_code << 8) | ADDR;
  ADDR = (ir_code >> 24) & 0xFF;           // First byte
  uint8_t IADDR = (ir_code >> 16) & 0xFF; // Second byte (address inversion)
  CMD = (ir_code >> 8) & 0xFF;            // Third byte (command)
  uint8_t ICMD = ir_code & 0xFF;          // Fourth byte (command inversion)
  
  // We are making up the missing reversed bytes
  IADDR = IADDR ^ 0xFF;
  ICMD = ICMD ^ 0xFF;

  // We put the bytes together into one string         
  ir_code =  ICMD;
  ir_code = (ir_code << 8) | ADDR;
  ir_code = (ir_code << 8) | IADDR;
  ir_code = (ir_code << 8) | CMD;
  ir_code = reverse_bits(ir_code,32);     // bit rotation to LSB-MSB order as in NEC       
}


// Formatting function for scorler stationString/stationName **** stationStringScroll ****
void stationStringFormatting() 
{
  // ----------- MODE 0 -> STATION, Stream, two VU meters, status bar at the bottom -----------
  if (displayMode == 0) 
  {   
    if (stationString == "") // If stationString is empty and the station does not transmit it, replace the empty stationString with the station name - stationNameStream
    {    
      if (stationNameStream == "") // if there is no Station Name, we put 3 lines
      { 
        stationStringScroll = "---" ;
        stationStringWeb = "---" ;
      } 
      else // if there is a station name, we type "-- NAME --" and send it to the scroller
      { 
        stationStringScroll = ("-- " + stationNameStream + " --");
        stationStringWeb = ("-- " + stationNameStream + " --");
      }  // The stationStringScroller variable takes the value of stationNameStream
    }
    else // If stationString contains data, we assign it to stationStringScroll to the scroller function
    {
      stationStringWeb = stationString;
      processText(stationString);  // we process Polish characters
      stationStringScroll = stationString + "    "; // we add a separator to the scrolling text if it does not fit on the screen
    }             
    
    // We count the length of the stationStringScroll string
    stationStringScrollWidth = stationStringScroll.length() * 6;    
    if (f_debug_on) 
    {
      Serial.print("debug -> StationStringScroll Lenght [chars]:");  Serial.println(stationStringScroll.length());
      Serial.print("debug -> StationStringScroll Width (Lenght * 6px) [px]:"); Serial.println(stationStringScrollWidth);
      
      Serial.print("debug -> Display Mode-0 stationStringScroll Text: @");
      Serial.print(stationStringScroll);
      Serial.println("@");
    }
  }
  // ------ MODE1 - LARGE CLOCK -------
  else if (displayMode == 1) // Clock display mode with 1 radio line at the bottom
  {
    char StationNrStr[4];
    snprintf(StationNrStr, sizeof(StationNrStr), "%02d", station_nr);  // Formatting station and bank information to 00
    
    if (urlPlaying) 
    {
      strncpy(StationNrStr, "URL", sizeof(StationNrStr) - 1);
      StationNrStr[sizeof(StationNrStr) - 1] = '\0';  
    }

    int StationNameEnd = stationName.indexOf("  "); // We cut out the station name only up to the double space
    stationName = stationName.substring(0, StationNameEnd);
 
    if (stationString == "")                // If stationString is empty and the station does not transmit it
    {   
      if (stationNameStream == "")          // and if there is no station name also
      {
        stationStringScroll = String(StationNrStr) + "." + stationName + ", ---" ;
        stationStringWeb = "---" ;
      }      // we insert three lines to display
      else  // if there is no "stationString" but there is a "stationName" then we put together the NR. Station name from the file, transmitted stationNameStream + gap separator
      {       
        stationStringScroll = String(StationNrStr) + "." + stationName + ", " + stationNameStream + "     ";
        //stationStringScroll = String(StationNrStr) + "." + stationName;

        // Truncating the scroller to 43 characters - TO MAKE IT FIXED TEXT mode 1
        //if (stationStringScroll.length() > 43) {stationStringScroll = stationStringScroll.substring(0,40) + "..."; }  
        stationStringWeb = stationNameStream;
      }
    }
    else //stationString != "" -> has value
    {
      stationStringWeb = stationString;
      processText(stationString);  // we process Polish characters
      
      stationStringScroll = String(StationNrStr) + "." + stationName + ", " + stationString + "     "; 
      //stationStringScroll = String(StationNrStr) + "." + stationName; 
      
      // Truncating the scroller to 43 characters - TO MAKE IT FIXED TEXT mode 1
      //stationStringScroll = String(StationNrStr) + "." + stationName + ", " + stationString; 
      //if (stationStringScroll.length() > 43) {stationStringScroll = stationStringScroll.substring(0,40) + "..."; }
      
      //Serial.println(stationStringScroll);
    }
    //Serial.print("debug -> Display1 (clock) stationStringScroll: ");
    //Serial.println(stationStringScroll);

    // We count the length of the stationStringScrollWidth string 
    stationStringScrollWidth = stationStringScroll.length() * 6;
  }
  // ----------- MODE 2 screen without scroller 3 lines of text ----------- //
  else if (displayMode == 2)
  {             
    // If the station does not transmit the station String, replace the empty station String with the station name - stationNameStream
    if (stationString == "") // If stationString is empty and the station does not transmit it
    {    
      if (stationNameStream == "") // if there is no station name also
      { 
        stationStringScroll = "---" ;
        stationStringWeb = "---" ;
      } // we insert three lines to display
      else // if there is a station name, we frame it in "-- NAME --" and send it to the scroller
      { 
        stationStringScroll = ("-- " + stationNameStream + " --");
        stationStringWeb = stationNameStream;
      }  // The stationStringScroller variable takes the value of stationNameStream
    }
    else // If stationString contains data, we assign it to stationStringScroll to the scroller function
    {
      stationStringWeb = stationString;
      processText(stationString);  // we process Polish characters
      stationStringScroll = stationString;
    }  
  }
  else if (displayMode == 3) // Display mode mode 3 and mode 5 (small spectrum)
  {
    if (stationString == "") // If stationString is empty and the station does not transmit it, replace the empty stationString with the station name - stationNameStream
    {    
      if (stationNameStream == "") // if there is no Station Name, we put 3 lines
      { 
        stationStringScroll = "---" ;
        stationStringWeb = "---" ;
      } 
      else // if there is a station name, we frame it in "-- NAME --" and send it to the scroller
      { 
        stationStringScroll = ("-- " + stationNameStream + " --");
        stationStringWeb = ("-- " + stationNameStream + " --");
      }  // The stationStringScroller variable takes the value of stationNameStream
    }
    else // If stationString contains data, we assign it to stationStringScroll to the scroller function
    {
      stationStringWeb = stationString;
      processText(stationString);  // we process Polish characters
      stationStringScroll = "  " + stationString + "  " ; // We do not add a separator to the text so that it displays evenly in the center
    }             
    //Liczymy długość napisu stationStringScroll 
    stationStringScrollWidth = stationStringScroll.length() * 6;
    //Serial.print("debug -> Display Mode-3 stationStringScroll:@");
    //Serial.print(stationStringScroll);
    //Serial.println("@");
  }
  else if (displayMode == 4) // Display mode 4 formatter for web needs
  {
    if (stationString == "") // If stationString is empty and the station does not transmit it, replace the empty stationString with the station name - stationNameStream
    {    
      if (stationNameStream == "") // if there is no Station Name, we put 3 lines
      { 
        stationStringScroll = "---" ;
        stationStringWeb = "---" ;
      } 
      else // if there is a station name, we frame it in "-- NAME --" and send it to the scroller
      { 
        stationStringScroll = ("-- " + stationNameStream + " --");
        stationStringWeb = ("-- " + stationNameStream + " --");
      }  // The stationStringScroller variable takes the value of stationNameStream
    }
    else // If stationString contains data, we assign it to stationStringScroll to the scroller function
    {
      stationStringWeb = stationString;
      processText(stationString);  // przetwarzamy polsie znaki
      stationStringScroll = "  " + stationString + "  " ; // We do not add a separator to the text so that it displays evenly in the center
    }             
    //Liczymy długość napisu stationStringScroll 
    stationStringScrollWidth = stationStringScroll.length() * 6;
    //Serial.print("debug -> Display Mode-3 stationStringScroll:@");
    //Serial.print(stationStringScroll);
    //Serial.println("@");
  }
  else if (displayMode == 7)
  {   
    if (stationString == "") // If stationString is empty and the station does not transmit it, replace the empty stationString with the station name - stationNameStream
    {    
      if (stationNameStream == "") // if there is no Station Name, we put 3 lines
      { 
        stationStringScroll = "---" ;
        stationStringWeb = "---" ;
      } 
      else // if there is a station name, we type "-- NAME --" and send it to the scroller
      { 
        stationStringScroll = ("-- " + stationNameStream + " --");
        stationStringWeb = ("-- " + stationNameStream + " --");
      }  // The stationStringScroller variable takes the value of stationNameStream
    }
    else // If stationString contains data, we assign it to stationStringScroll to the scroller function
    {
      stationStringWeb = stationString;
      processText(stationString);  // we process Polish characters
      stationStringScroll = stationString + "    "; // we add a separator to the scrolling text if it does not fit on the screen
    }             
    
    // We count the length of the stationStringScroll string
    stationStringScrollWidth = stationStringScroll.length() * 6;    
    if (f_debug_on) 
    {
      Serial.print("debug -> StationStringScroll Lenght [chars]:");  Serial.println(stationStringScroll.length());
      Serial.print("debug -> StationStringScroll Width (Lenght * 6px) [px]:"); Serial.println(stationStringScrollWidth);
      
      Serial.print("debug -> Display Mode-0 stationStringScroll Text: @");
      Serial.print(stationStringScroll);
      Serial.println("@");
    }
  }

}

// ----------- MAIN DISPLAY OPERATION IN THE INTERNET RADIO FUNCTION ----------- //
void displayRadio() 
{
  int StationNameEnd = stationName.indexOf("  "); // We cut out the station name only up to the double space
  stationName = stationName.substring(0, StationNameEnd);

  if (displayMode == 0)
  {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_helvB14_tr);
    u8g2.drawStr(24, 16, stationName.substring(0, stationNameLenghtCut - 1).c_str());
    u8g2.drawRBox(1, 1, 21, 16, 4);  // White square (background) under the station number
        
    // Bank number display function at the bottom of the screen
    u8g2.setFont(spleen6x12PL);
    char BankStr[8];  
    snprintf(BankStr, sizeof(BankStr), "B-%02d", bank_nr); // We format the bank number to 00

    // We display the Bank number on the bottom line
    
    if (!urlPlaying) 
    {
      u8g2.drawBox(161, 54, 1, 12);  // we draw a 1px strip before the word "Bank" for symmetry
      u8g2.setDrawColor(0);
      u8g2.setCursor(162, 63);  // Position of the Bank0x inscription at the bottom of the screen
      u8g2.print(BankStr);
    }
    
    u8g2.setDrawColor(0);
    char StationNrStr[3];
    snprintf(StationNrStr, sizeof(StationNrStr), "%02d", station_nr);  // Formatting station and bank information to 00
    // Station number position at the top left of the screen
    
    // Distinguishing whether we enter the station number or the URL string if we are playing from a link from a website
    if (!urlPlaying) 
    {
      u8g2.setCursor(4, 14);
      u8g2.setFont(u8g2_font_spleen8x16_mr); 
      u8g2.print(StationNrStr);} 
    else 
    {
      u8g2.setCursor(3, 13);
      u8g2.setFont(spleen6x12PL);
      u8g2.print("URL");
    }
    
    u8g2.setDrawColor(1);
      
    // 3xZZZ logo in SLEEP timer mode
    if (f_sleepTimerOn) 
    {
      u8g2.setFont(u8g2_font_04b_03_tr); 
      u8g2.drawStr(189,63, "z");
      u8g2.drawStr(192,61, "z");
      u8g2.drawStr(195,59, "z");
    }
    
    stationStringFormatting(); //We format the stationString displayed by the Scroller function
    u8g2.drawLine(0, 52, 255, 52); // Bottom dividing line
    
    u8g2.setFont(spleen6x12PL); 
    u8g2.drawStr(135, 63, streamCodec.c_str()); // we add the codec shifted slightly by 1px to fit the bank number text
    String displayString = String(SampleRate) + "." + String(SampleRateRest) + "kHz " + bitsPerSampleString + "bit " + bitrateString + "kbps";
    u8g2.setFont(spleen6x12PL);
    u8g2.drawStr(0, 63, displayString.c_str());
  }
  
  else if (displayMode == 1) // CLOCK - Clock display mode with 1 radio line at the bottom
  {
    u8g2.clearBuffer();
    u8g2.setDrawColor(1);
    u8g2.setFont(spleen6x12PL);
    u8g2.drawLine(0, 50, 255, 50); // Clock separation line, radio bottom line

    // 3xZZZ logo in SLEEP timer mode
    if (f_sleepTimerOn) 
    {
      u8g2.setFont(u8g2_font_04b_03_tr); 
      u8g2.drawStr(200,47, "z");
      u8g2.drawStr(203,45, "z");
      u8g2.drawStr(206,43, "z");
      
    }
    
    // --- SPEAKER AND VOLUME LEVEL ---
    u8g2.setFont(spleen6x12PL);
    u8g2.drawGlyph(215,47, 0x9E); // 0x9E in the Spleen font is the encoded speaker symbol
    u8g2.drawStr(223,47, String(volumeValue).c_str());

    stationStringFormatting(); // We format the stationString displayed by the Scroller function 
  }
  
  else if (displayMode == 2) // 3 LINES - Display mode 2 - 3 lines of text, no scrolling
  {
    u8g2.clearBuffer();
    u8g2.setFont(spleen6x12PL);
        
    if (!urlPlaying) 
    {
      u8g2.drawStr(23, 11, stationName.substring(0, stationNameLenghtCut).c_str()); // Cropping and displaying this way we do not change the content of the station Name variable
      u8g2.drawRBox(1, 1, 18, 13, 4);  // Rbox under the station number
    }
    else // we play with the URL, we enlarge the field from the station number to fit the URL text
    {
      u8g2.drawStr(29, 11, stationName.substring(0, stationNameLenghtCut).c_str()); // Cropping and displaying this way we do not change the content of the station Name variable
      u8g2.drawRBox(1, 1, 24, 13, 4);  // Rbox under the station number
    }

    // Bank number display function at the bottom of the screen
    char BankStr[8];  
    snprintf(BankStr, sizeof(BankStr), "B-%02d", bank_nr); // We format the bank number to 00

    // We display the Bank number on the bottom line
    if (!urlPlaying) 
    {
      u8g2.drawBox(161, 54, 1, 12);  // we draw a 1px strip before the word "Bank" for symmetry
      u8g2.setDrawColor(0);
      u8g2.setCursor(162, 63);  // Position of the Bank0x inscription at the bottom of the screen
      u8g2.print(BankStr);
    } //else {u8g2.print("URL");}
   
    u8g2.setDrawColor(0);
    char StationNrStr[3];
    snprintf(StationNrStr, sizeof(StationNrStr), "%02d", station_nr);  // Formatting station and bank information to 00
                                                // Station number position at the top left of the screen
    if (!urlPlaying) { u8g2.setCursor(4, 11); u8g2.print(StationNrStr);} else { u8g2.setCursor(5, 12); u8g2.print("URL");}
    u8g2.setDrawColor(1);

    // 3xZZZ logo in SLEEP timer mode
    if (f_sleepTimerOn) 
    {
      u8g2.setFont(u8g2_font_04b_03_tr); 
      u8g2.drawStr(189,63, "z");
      u8g2.drawStr(192,61, "z");
      u8g2.drawStr(195,59, "z");
    }

    stationStringFormatting(); // We format the stationString displayed by the Scroller function

    u8g2.drawLine(0, 52, 255, 52);
    u8g2.setFont(spleen6x12PL);
    u8g2.drawStr(135, 63, streamCodec.c_str()); // we add the codec shifted slightly by 1px to fit the bank number text
    String displayString = String(SampleRate) + "." + String(SampleRateRest) + "kHz " + bitsPerSampleString + "bit " + bitrateString + "kbps";
    u8g2.drawStr(0, 63, displayString.c_str());  
  }
  
  else if (displayMode == 3) // Display mode 3 - Center aligned, status line (station, bank time) at the top and bottom (stream format, Wi-Fi range)
  {
    u8g2.clearBuffer();
        
    //-- "ICON" SLEEP TIMER -- 
    if (f_sleepTimerOn)
    {
      u8g2.setFont(u8g2_font_04b_03_tr); 
      u8g2.drawStr(165,10, "z");
      u8g2.drawStr(168,8, "z");
      u8g2.drawStr(171,6, "z");
    }
    
    // -- IKONA VOLUME I WARTOSC --    
    u8g2.setFont(spleen6x12PL);
    u8g2.drawGlyph(180,10, 0x9E); // 0x9E in the Spleen font is the encoded speaker symbol
    u8g2.drawStr(189,10, String(volumeValue).c_str());        
    
    
    // -- LOGO FLAC/MP3/AACi BITRATE --
    u8g2.setFont(mono04b03b); // width 5, height 6
    uint8_t x_codec = 103;  // X coordinate for codec information
    
    if (!f_simpleMode3) {x_codec = 47;}
    
    if (flac == true || opus == true) 
    {
      if (u8g2.getStrWidth(bitrateString.c_str()) > 19) {u8g2.drawFrame(x_codec,2,45,9);}  // 128 ->18px, we adjust the rank depending on whether the bitrate is 4 or 3 digits
      else { u8g2.drawFrame(x_codec,2,39,9);}
      
      u8g2.drawBox(x_codec+1,2,21,9);
      u8g2.setDrawColor(0);
      u8g2.drawStr(x_codec+2,9, streamCodec.c_str());
      u8g2.setDrawColor(1);
      u8g2.drawStr(x_codec+23,9, bitrateString.c_str());
    } else 
    {
      u8g2.drawFrame(x_codec+6,2,38,9);
      u8g2.drawBox(x_codec+6,2,19,9);
      u8g2.setDrawColor(0);
      u8g2.drawStr(x_codec+8,9, streamCodec.c_str());
      u8g2.setDrawColor(1);
      u8g2.drawStr(x_codec+27,9, bitrateString.c_str());
    }

    // -- DISPLAY BANK AND STATION NUMBER --
    u8g2.setFont(spleen6x12PL);
    u8g2.setCursor(1, 10);
    
    if (!urlPlaying) // If we are not playing from the URL sent from the website
    {       
      char StationNrStr[3]; snprintf(StationNrStr, sizeof(StationNrStr), "%02d", station_nr);  // Formatting station and bank information to 00
      
      if (f_simpleMode3)
      {
        char BankStr[8]; snprintf(BankStr, sizeof(BankStr), "%1d", bank_nr); // We format the bank number
        u8g2.print("CH.");   
        u8g2.print(BankStr);
        //u8g2.print(".");
        u8g2.setFont(spleen6x12PL);
        u8g2.print(StationNrStr);
      }
      else
      {
        char BankStr[8]; snprintf(BankStr, sizeof(BankStr), "%02d", bank_nr); // We format the bank number to 00
        u8g2.print("BANK:");
        u8g2.print(BankStr);     
        u8g2.setCursor(99, 10);
        u8g2.print("STATION:");
        u8g2.print(StationNrStr);
      }
    }
    else
    { 
      if (f_simpleMode3) {u8g2.print("URL");} else {u8g2.setCursor(119, 10); u8g2.print("URL");}
    }
   
    //u8g2.setFont(u8g2_font_helvB14_tr);
    //u8g2.setFont(u8g2_font_fub14_tr);
    //u8g2.setFont(u8g2_font_smart_patrol_nbp_tf);
    u8g2.setFont(u8g2_font_UnnamedDOSFontIV_tr);
    int stationNameWidth = u8g2.getStrWidth(stationName.substring(0, stationNameLenghtCut).c_str()); // Counts the positions to display stationName centered
    int stationNamePositionX = (256 - stationNameWidth) / 2;
    
    u8g2.drawStr(stationNamePositionX, stationNamePositionYmode3, stationName.substring(0, stationNameLenghtCut).c_str()); // Cropping and displaying this way we do not change the content of the station Name variable

    stationStringFormatting(); // We format the stationString displayed by the Scroller function
    u8g2.setFont(spleen6x12PL);
  }

  else if (displayMode == 4) // Large VU indicators - no radio station information displayed, only VU indicators and refresh for www
  {
    u8g2.clearBuffer();
    u8g2.setFont(spleen6x12PL);
  }
  
  else if (displayMode == 5) // Large VU bars
  {
   u8g2.clearBuffer();
   stationStringFormatting(); 
  }
  
  else if(displayMode == 6)  // Radio info
  {
    u8g2.clearBuffer();
    stationStringFormatting(); 
    displayInfo();
  }

  else if (displayMode == 7) // Test mode for 128x64 display
  {
    u8g2.clearBuffer();
    
    u8g2.drawLine(128,0,128,64); // A test line that virtually determines the screen size.
    
    u8g2.drawLine(13,5,46,5); 
    u8g2.drawLine(82,5,115,5);
    
    u8g2.setFont(spleen6x12PL);
    int stationNameWidth = u8g2.getStrWidth(stationName.substring(0, stationNameLenghtCut - 2).c_str()); // Liczy pozycje aby wyswietlic stationName na wycentrowane środku
    int stationNamePositionX = (127 - stationNameWidth) / 2;
    u8g2.drawStr(stationNamePositionX, 22, stationName.substring(0, stationNameLenghtCut - 2).c_str());
        
    u8g2.setFont(u8g2_font_04b_03_tr);
    char BankStr[8];  
    char StationNrStr[3];
    snprintf(BankStr, sizeof(BankStr), "%02d", bank_nr); // We format the bank number to 00
    snprintf(StationNrStr, sizeof(StationNrStr), "%02d", station_nr);  //Formatting station and bank information to 00
        
    u8g2.drawRBox(0, 0, 13, 10, 3);  // White square (background) under the station number
    u8g2.drawRBox(115, 0, 13, 10, 3);  //White square (background) under the station number
    if (!urlPlaying) 
    {
      //u8g2.drawBox(87, 58, 1, 5);  // we draw a 1px strip before the word "Bank" for symmetry
      u8g2.setDrawColor(0);
      //u8g2.setCursor(88,63);  // Position of the Bank0x inscription at the bottom of the screen
      u8g2.setCursor(117,8);  // Position of the Bank0x inscription at the bottom of the screen
      u8g2.print(BankStr);
    }
   
       
    // Station number position at the top left of the screen
    if (!urlPlaying) 
    {
      //u8g2.setFont(spleen6x12PL);
      u8g2.setCursor(2, 8);
      //u8g2.print("Station:");  
      u8g2.print(StationNrStr);
    } 
    else 
    {
      u8g2.setCursor(1, 10);
      u8g2.setFont(spleen6x12PL);
      u8g2.print("URL");
    }
    u8g2.setDrawColor(1);
      
    // 3xZZZ logo in SLEEP timer mode
    if (f_sleepTimerOn) 
    {
      u8g2.setFont(u8g2_font_04b_03_tr); 
      u8g2.drawStr(189,63, "z");
      u8g2.drawStr(192,61, "z");
      u8g2.drawStr(195,59, "z");
    }
    
    stationStringFormatting(); //We format the stationString displayed by the Scroller function
    u8g2.drawLine(0, 54, 127, 54); // Bottom dividing line
    
    
    
    u8g2.setFont(u8g2_font_04b_03_tr); 
    //u8g2.drawStr(42, 63, String(bitrateString + "k").c_str());
    //u8g2.drawStr(65, 63, streamCodec.c_str()); // we add the codec shifted slightly by 1px to fit the bank number text
       

    String displayString = String(SampleRate) + "." + String(SampleRateRest) + "kHz " + bitsPerSampleString +"bit " + bitrateString + "kbps  ";
    //u8g2.setFont(u8g2_font_04b_03_tr);
    u8g2.setCursor(0, 63);
    //u8g2.drawStr(0, 63, displayString.c_str());
    u8g2.print(displayString);

    u8g2.setFont(mono04b03b);
    u8g2.print(String(streamCodec));

  }  

 

}


// Audio info callback support for library 3.4.1 and newer.
void my_audio_info(Audio::msg_t m)
{
  switch(m.e)
  {
    case Audio::evt_info:  
    {
      String msg = String(m.msg);   // conversion to String
      msg.trim(); // remove spaces and \r\n

      // --- BitRate ---
      int bitrateIndex = msg.indexOf("Bitrate (b/s):");  
      if (bitrateIndex != -1)
      {
        int endIndex = msg.indexOf('\n', bitrateIndex);
        if (endIndex == -1) endIndex = msg.length();
        bitrateString = msg.substring(bitrateIndex + 14, endIndex);
        bitrateString.trim();

        //Przliczenie bps na Kbps
        bitrateStringInt = bitrateString.toInt();  
        bitrateStringInt = bitrateStringInt / 1000;
        bitrateString = String(bitrateStringInt); 
        
        f_audioInfoRefreshDisplayRadio = true;
        wsAudioRefresh = true;  //Web Socket - audio refresh
        Serial.printf("bitrate: .... %s\n", m.msg); // icy-bitrate or bitrate from metadata
      }
      
      // --- FLAC BitsPerSample ---
      int bitrateIndexFlac = msg.indexOf("FLAC bitspersample:");
      if ((bitrateIndexFlac != -1) && (flac == true))
      {
        Serial.printf("bitrate FLAC: .... %s\n", m.msg); // icy-bitrate or bitrate from metadata
        int endIndex = msg.indexOf('\n', bitrateIndexFlac);
        if (endIndex == -1) endIndex = msg.length();
        bitrateString = msg.substring(bitrateIndex + 19, endIndex);
        bitrateString.trim();

        // przliczenie bps na Kbps
        bitrateStringInt = bitrateString.toInt();  
        bitrateStringInt = bitrateStringInt / 1000;
        bitrateString = String(bitrateStringInt);
            
        f_audioInfoRefreshDisplayRadio = true;
        wsAudioRefresh = true;  //Web Socket - audio refresh 
      }


      // --- SampleRate ---
      int sampleRateIndex = msg.indexOf("SampleRate (Hz):");
      if (sampleRateIndex != -1)
      {
        int endIndex = msg.indexOf('\n', sampleRateIndex);
        if (endIndex == -1) endIndex = msg.length();
        //sampleRateString = msg.substring(sampleRateIndex + 11, endIndex);
        sampleRateString = msg.substring(sampleRateIndex + 17, endIndex);
        sampleRateString.trim();
           
        SampleRate = sampleRateString.toInt();
        SampleRateRest = SampleRate % 1000;
        SampleRateRest = SampleRateRest / 100;
        SampleRate = SampleRate / 1000;
        
        f_audioInfoRefreshDisplayRadio = true;
        wsAudioRefresh = true;  //Web Socket - audio refresh    
      }

      // --- BitsPerSample ---
      int bitsPerSampleIndex = msg.indexOf("BitsPerSample:");
      if (bitsPerSampleIndex != -1)
      {     
        int endIndex = msg.indexOf('\n', bitsPerSampleIndex);
        if (endIndex == -1) endIndex = msg.length();
        bitsPerSampleString = msg.substring(bitsPerSampleIndex + 15, endIndex);
        bitsPerSampleString.trim();
        f_audioInfoRefreshDisplayRadio = true;	
        wsAudioRefresh = true;  //Web Socket - audio refresh    
      }
    
      // --- Rozpoznawanie dekodera / formatu ---
      if (msg.indexOf("MP3Decoder") != -1)
      {
        mp3 = true; 
        flac = false; aac = false; vorbis = false; opus = false;
        streamCodec = "MP3";
        f_audioInfoRefreshDisplayRadio = true; // refresh displayRadio screen
        wsAudioRefresh = true;  //Web Socket - audio refresh    
      }
      if (msg.indexOf("FLACDecoder") != -1)
      {
        flac = true; 
        mp3 = false; aac = false; vorbis = false; opus = false;
        streamCodec = "FLAC";
        f_audioInfoRefreshDisplayRadio = true; // refresh displayRadio screen
        wsAudioRefresh = true;  //Web Socket - audio refresh    
      }
      if (msg.indexOf("AACDecoder") != -1)
      {
        aac = true;
        flac = false; mp3 = false; vorbis = false; opus = false;
        streamCodec = "AAC";
        f_audioInfoRefreshDisplayRadio = true; // refresh displayRadio screen
        wsAudioRefresh = true;  //Web Socket - audio refresh    
      }
      if (msg.indexOf("VORBISDecoder") != -1)
      {
        vorbis = true;
        aac = false; flac = false; mp3 = false; opus = false;
        streamCodec = "VRB";
        f_audioInfoRefreshDisplayRadio = true; // refresh displayRadio screen
        wsAudioRefresh = true;  //Web Socket - audio refresh    
      }
      if (msg.indexOf("OPUSDecoder") != -1)
      {
        opus = true;
        aac = false; flac = false; mp3 = false; vorbis = false;
        streamCodec = "OPUS";
        f_audioInfoRefreshDisplayRadio = true; // refresh displayRadio screen
        wsAudioRefresh = true;  //Web Socket - audio refresh    
      }

      // --- Debug ---
      Serial.printf("info: ....... %s\n", m.msg);      
    }
    break;


    case Audio::evt_eof:
    {
      Serial.printf("end of file:  %s\n", m.msg);
      if (resumePlay == true)
      {
        ir_code = rcCmdOk; // We assign the remote control code - OK
        bit_count = 32;
        calcNec();        // We convert the remote control code into the full NEC code
        resumePlay = false;
      }
      
    }
    break;

    case Audio::evt_bitrate:
    {        
      bitrateString = String(m.msg);
      bitrateString.trim();
      
      // bps to kbps conversion
      bitrateStringInt = bitrateString.toInt();  
      bitrateStringInt = bitrateStringInt / 1000;
      bitrateString = String(bitrateStringInt);
      
      f_audioInfoRefreshDisplayRadio = true;
      wsAudioRefresh = true;  //Web Socket - audio refresh
      Serial.printf("info: ....... evt_bitrate: %s\n", m.msg); break; // icy-bitrate or bitrate from metadata
    }
    case Audio::evt_icyurl:         Serial.printf("icy URL: .... %s\n", m.msg); break;
    case Audio::evt_id3data:        Serial.printf("ID3 data: ... %s\n", m.msg); break;
    case Audio::evt_lasthost:       Serial.printf("last URL: ... %s\n", m.msg); break;
    case Audio::evt_name:           
	  {
      stationNameStream = String(m.msg);
      stationNameStream.trim();
      
      // If we play from a WEB URL or the stationNameFromStream=1 flag, we read the station name from the stream, not from the bank file.
      //if ((bank_nr == 0) || (stationNameFromStream)) stationName = stationNameStream; 
      if ((urlPlaying) || (stationNameFromStream)) stationName = stationNameStream; 

      f_audioInfoRefreshStationString = true;
      wsAudioRefresh = true;
      Serial.printf("info: ....... station name: %s\n", m.msg); // station name or icy-name
	  }
    break; 

    case Audio::evt_streamtitle: // Save the song title
    {
      stationString = String(m.msg);
		  stationString.trim();
		
      //ActionNeedUpdateTime = true;
      f_audioInfoRefreshStationString = true;	
      wsAudioRefresh = true;
      
      // Debug
      Serial.printf("info: ....... stream title: %s\n", m.msg);
      //Serial.printf("info: ....... %s\n", m.msg);
    }
    break;
    
    case Audio::evt_icylogo:        
    {
      stationLogoUrl = String(m.msg);
      stationLogoUrl.trim();
      Serial.println("info: ....... stationLogoUrl: " + stationLogoUrl);
      Serial.printf("icy logo: ... %s\n", m.msg); 
    }
    break;
    
    case Audio::evt_icydescription: Serial.printf("icy descr: .. %s\n", m.msg); break;
    case Audio::evt_image: for(int i = 0; i < m.vec.size(); i += 2) { Serial.printf("cover image:  segment %02i, pos %07lu, len %05lu\n", i / 2, m.vec[i], m.vec[i + 1]);} break; // APIC
    case Audio::evt_lyrics:         Serial.printf("sync lyrics:  %s\n", m.msg); break;
    case Audio::evt_log   :         Serial.printf("audio_logs:   %s\n", m.msg); break;
    default:                        Serial.printf("message:..... %s\n", m.msg); break;
  }
}


void encoderFunctionOrderChange()
{
  displayActive = true;
  displayStartTime = millis();
  volumeSet = false;
  timeDisplay = false;
  bankMenuEnable = false;
  encoderFunctionOrder = !encoderFunctionOrder;
  u8g2.clearBuffer();
  u8g2.setFont(spleen6x12PL);
  u8g2.drawStr(1,14,"Encoder function order change:");
  if (encoderFunctionOrder == false) {u8g2.drawStr(1,28,"Rotate for volume, press for station list");}
  if (encoderFunctionOrder == true ) {u8g2.drawStr(1,28,"Rotate for station list, press for volume");}
  u8g2.sendBuffer();
}

void bankMenuDisplay()
{
  if (bank_nr < 1) {bank_nr = 1;}
  const uint8_t cornerRadius = 3;
  float segmentWidth = (float)212 / bank_nr_max;

  displayStartTime = millis();
  Serial.println("debug -> BankMenuDisplay");
  volumeSet = false;
  bankMenuEnable = true;
  timeDisplay = false;
  displayActive = true;
  //currentOption = BANK_LIST;  // Setting the list of banks to scroll and select
  String bankNrStr = String(bank_nr);
  Serial.println("debug -> Displaying a list of banks");
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_fub14_tf);
  u8g2.drawStr(80, 33, "BANK ");
  u8g2.drawStr(145, 33, String(bank_nr).c_str());  // numer banku
  //if ((bankNetworkUpdate == true) || (noSDcard == true))
  if ((bankNetworkUpdate == true) || (storageTextName == "EEPROM"))
  {
    u8g2.setFont(spleen6x12PL);
    u8g2.drawStr(185, 24, "NETWORK ");
    u8g2.drawStr(188, 34, "UPDATE  ");
    
    //if (noSDcard == true)
    if (storageTextName == "EEPROM")
    {
      //u8g2.drawStr(24, 24, "NO SD");
      u8g2.drawStr(24, 34, "NO CARD");
    }
  
  }
  
  u8g2.drawRFrame(21, 42, 214, 14, cornerRadius);                // Frame for bank slider
    
  uint16_t sliderX = 23 + (uint16_t)round((bank_nr - 1)* segmentWidth); // Calculation of the X variable with rounding up for the slider position
  uint16_t sliderWidth = (uint16_t)round(segmentWidth - 2); // Calculating the slider width depending on the number of banks
  u8g2.drawRBox(sliderX, 44, sliderWidth, 10, 2);

  //u8g2.drawRBox((bank_nr * 13) + 10, 44, 15, 10, 2);  // filling the slider box for the fixed value max_bank = 16, old version
  u8g2.sendBuffer();
  
}

// =========== Function for encoder buttons, debouncing and long press ==============//
void handleButtons() 
{
  static unsigned long buttonPressTime2 = 0;  // Variable to store the time encoder button 2 was pressed
  static bool isButton2Pressed = false;       // Flag to track whether encoder button 2 is pressed
  static bool action2Taken = false;           // Flag to track if the action for encoder 2 has been executed

  static unsigned long lastPressTime2 = 0;  // Variable to control debouncing (last press time)
  const unsigned long debounceDelay2 = 50;  // Debouncing delay

  // ===== Encoder Button 2 Operation =====
  int reading2 = digitalRead(SW_PIN2);

  // Debouncing for encoder button 2
  if (reading2 == LOW)  // Button is pressed (low)
  {
    if (millis() - lastPressTime2 > debounceDelay2) 
    {
      lastPressTime2 = millis();  // We update the last press time
      // We check if the button was pressed for 3 seconds
      if (!isButton2Pressed) 
      {
        buttonPressTime2 = millis();  // We set the pressing time
        isButton2Pressed = true;      // We set the flag that the button is pressed
        action2Taken = false;         // Resetting the action flag for encoder 2
        action3Taken = false;         // Let's reset the Super Long Press Encoder 2 action flag
        volumeSet = false;
      }

      if (millis() - buttonPressTime2 >= buttonLongPressTime2 && millis() - buttonPressTime2 >= buttonSuperLongPressTime2 && action3Taken == false) 
      {
        ir_code = rcCmdPower; // We pretend to be the remote control code Back
        bit_count = 32;
        calcNec();          // We convert the remote control code into the full original NEC code
        action3Taken = true;
      }


      // If the button is pressed for at least 3 seconds and the action has not been performed yet
      if (millis() - buttonPressTime2 >= buttonLongPressTime2 && !action2Taken && millis() - buttonPressTime2 < buttonSuperLongPressTime2) {
        Serial.println("debug--Bank Menu");
        bankMenuDisplay();
        
        // We set the action flag so that it executes only once
        action2Taken = true;
      }
    }
  } 
  else 
  {
    isButton2Pressed = false;  // Resetting the encoder 2 button press flag
    action2Taken = false;      // Resetting the action flag for encoder 2
    action3Taken = false;
  }

    
  #ifdef twoEncoders

    static unsigned long buttonPressTime1 = 0;  // Variable to store the time encoder button 2 was pressed
    static bool isButton1Pressed = false;       // Flag to track whether encoder button 2 is pressed
    
    static unsigned long lastPressTime1 = 0;  // Variable to control debouncing (last press time)
    const unsigned long debounceDelay1 = 50;  // Debouncing delay

    // ===== Encoder Button Operation 1 =====
    int reading1 = digitalRead(SW_PIN1);
    // Debouncing for encoder button 1

    if (reading1 == LOW)  // Button is pressed (low)
    {
      if (millis() - lastPressTime1 > debounceDelay1) 
      {
        lastPressTime1 = millis();  // We update the last press time

        // We check if the button was pressed for 3 seconds
        if (!isButton1Pressed) 
        {
          buttonPressTime1 = millis();  // We set the pressing time
          isButton1Pressed = true;      // We set the flag that the button is pressed
          action4Taken = false;         // Resetting the action flag for encoder 2
        }
        
        
        if (millis() - buttonPressTime1 >= buttonLongPressTime1 && millis() - buttonPressTime1 >= buttonSuperLongPressTime1 && action4Taken == false) 
        {
          ir_code = rcCmdPower; // We pretend to be the remote control code Back
          bit_count = 32;
          calcNec();          // We convert the remote control code into the full original NEC code      
          action4Taken = true;
        }
        /*
        if (millis() - buttonPressTime1 >= buttonLongPressTime1 && !action5Taken && millis() - buttonPressTime1 < buttonSuperLongPressTime1) 
        {
          bankMenuDisplay();
          // We set the action flag so that it executes only once
          action5Taken = true;
        }
        */
      }
    }  
    else
    {
      isButton1Pressed = false;  // Resetting the encoder 1 button press flag
      action4Taken = false;
      action5Taken = false;
    }     
  #endif
  



}

int maxSelection() 
{
  //if (currentOption == INTERNET_RADIO) 
  //{
    return stationsCount - 1;
  //} 
  //return 0;  // Returns 0 if none of the conditions are met
}

// Scroll down function
void scrollDown() 
{
  if (currentSelection < maxSelection()) 
  {
    currentSelection++;
    if (currentSelection >= firstVisibleLine + maxVisibleLines) 
    {
      firstVisibleLine++;
    }
  }
    else
  {
    // If the maximum value is reached, go to the smallest (0)
    currentSelection = 0;
    firstVisibleLine = 0; // Restore to first visible line
  }
    
  Serial.print("Scroll Down: CurrentSelection = ");
  Serial.println(currentSelection);
}

// Function to scroll up
void scrollUp() 
{
  if (currentSelection > 0) 
  {
    currentSelection--;
    if (currentSelection < firstVisibleLine) 
    {
      firstVisibleLine = currentSelection;
    }
  }
  else
  {
    // If value 0 is reached, go to the highest value
    currentSelection = maxSelection(); 
    firstVisibleLine = currentSelection - maxVisibleLines + 1; // Set the first visible line to the highest
  }
  
  Serial.print("Scroll Up: CurrentSelection = ");
  Serial.println(currentSelection);
}

void readVolumeFromSD() 
{
  // Check if the SD card is available
  //if (!STORAGE.begin(SD_CS)) 
  if (!STORAGE_BEGIN())
  {
    Serial.println("debug SD -> Cannot find SD card, setting Volume value from EEPROM.");
    Serial.print("debug vol -> Volume Value: ");
    EEPROM.get(2, volumeValue);
    if (volumeValue > maxVolume) {volumeValue = 10;} // protection against empty EEPROM cell with value FF (255)
    
    if (f_volumeFadeOn == false) {audio.setVolume(volumeValue);}  // zakres 0...21...42
    volumeBufferValue = volumeValue; 
    
    Serial.println(volumeValue);
    return;
  }
  // Check if the file volume.txt exists
  if (STORAGE.exists("/volume.txt")) 
  {
    myFile = STORAGE.open("/volume.txt");
    if (myFile) 
    {
      volumeValue = myFile.parseInt();
	    myFile.close();
      
      Serial.print("debug SD -> Loaded volume.txt from SD card, value read: ");
      Serial.println(volumeValue);    
    } 
	  else 
	  {
      Serial.println("debug SD -> Error opening file volume.txt");
    }
  } 
  else 
  {
    Serial.print("debug SD -> File volume.txt does not exist, Volume value default: ");
	  Serial.println(volumeValue);    
  }
  if (volumeValue > maxVolume) {volumeValue = 10;}  // We set a safe value
  //audio.setVolume(volumeValue);  // range 0...21...42
  volumeBufferValue = volumeValue;
}

void saveVolumeOnSD() 
{
  volumeBufferValue = volumeValue; // Alignment of Volume and Volume buffer values ​​on write
  
  // Check if the file volume.txt exists
  Serial.print("debug SD -> Saving the Volume value to a file: ");
  Serial.println(volumeValue); 
  
  // Check if the file exists
  if (STORAGE.exists("/volume.txt")) 
  {
    Serial.println("debug SD -> The file volume.txt already exists.");

    // Open the file for saving and overwrite the current value of the equalizer filters
    myFile = STORAGE.open("/volume.txt", FILE_WRITE);
    if (myFile) 
	  {
      myFile.println(volumeValue);
      myFile.close();
      Serial.println("debug SD -> Updating volume.txt on SD card");
    } 
	  else 
	  {
      Serial.println("debug SD -> Error opening file volume.txt.");
    }
  } 
  else 
  {
    Serial.println("debug SD -> File volume.txt does not exist. Creating...");

    // Create a file and save the current volume value in it
    myFile = STORAGE.open("/volume.txt", FILE_WRITE);
    if (myFile) 
	  {
      myFile.println(volumeValue);
      myFile.close();
      Serial.println("debug SD -> Created and saved volume.txt on SD card");
    } 
    else 
    {
      Serial.println("debug SD -> Error creating volume.txt file.");
    }
  }
  if (noSDcard == true) {EEPROM.write(2,volumeValue); EEPROM.commit(); Serial.println("debug eeprom -> Writing volume to EEPROM");}
}

void drawSignalPower(uint8_t xpwr, uint8_t ypwr, bool print, bool mode)
{
  // Values ​​based on ->  https://www.intuitibits.com/2016/03/23/dbm-to-percent-conversion/
  int signal_dBM[] = { -100, -99, -98, -97, -96, -95, -94, -93, -92, -91, -90, -89, -88, -87, -86, -85, -84, -83, -82, -81, -80, -79, -78, -77, -76, -75, -74, -73, -72, -71, -70, -69, -68, -67, -66, -65, -64, -63, -62, -61, -60, -59, -58, -57, -56, -55, -54, -53, -52, -51, -50, -49, -48, -47, -46, -45, -44, -43, -42, -41, -40, -39, -38, -37, -36, -35, -34, -33, -32, -31, -30, -29, -28, -27, -26, -25, -24, -23, -22, -21, -20, -19, -18, -17, -16, -15, -14, -13, -12, -11, -10, -9, -8, -7, -6, -5, -4, -3, -2, -1};
  int signal_percent[] = {0, 0, 0, 0, 0, 0, 4, 6, 8, 11, 13, 15, 17, 19, 21, 23, 26, 28, 30, 32, 34, 35, 37, 39, 41, 43, 45, 46, 48, 50, 52, 53, 55, 56, 58, 59, 61, 62, 64, 65, 67, 68, 69, 71, 72, 73, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 90, 91, 92, 93, 93, 94, 95, 95, 96, 96, 97, 97, 98, 98, 99, 99, 99, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100};

  int signalpwr = WiFi.RSSI();
  //uint8_t wifiSignalLevel = 0;
    
  
  // Vertical lines, width 11px
  
  // We clean the area under the lines
  u8g2.setDrawColor(0);
  u8g2.drawBox(xpwr,ypwr-10,11,11);
  u8g2.setDrawColor(1);

  
  // Drawing an antenna
  int x = xpwr - 3; 
  int y = ypwr - 8;
  u8g2.drawLine(x,y,x,y+7); // middle line
  u8g2.drawLine(x,y+4,x-3,y); // left arm
  u8g2.drawLine(x,y+4,x+3,y); // right arm


  // We draw 1x1px bases under each line
  u8g2.drawBox(xpwr,ypwr - 1, 1, 1);
  u8g2.drawBox(xpwr + 2, ypwr - 1, 1, 1);
  u8g2.drawBox(xpwr + 4, ypwr - 1, 1, 1);
  u8g2.drawBox(xpwr + 6, ypwr - 1, 1, 1);
  u8g2.drawBox(xpwr + 8, ypwr - 1, 1, 1);
  u8g2.drawBox(xpwr + 10, ypwr - 1, 1, 1);

 if ((WiFi.status() == WL_CONNECTED) && (mode == 0))
 {
      // We draw lines
    if (signalpwr > -88) { wifiSignalLevel = 1; u8g2.drawBox(xpwr, ypwr - 2, 1, 1); }     // 0-14
    if (signalpwr > -81) { wifiSignalLevel = 2; u8g2.drawBox(xpwr + 2, ypwr - 3, 1, 2); } // > 28
    if (signalpwr > -74) { wifiSignalLevel = 3; u8g2.drawBox(xpwr + 4, ypwr - 4, 1, 3); } // > 42
    if (signalpwr > -66) { wifiSignalLevel = 4; u8g2.drawBox(xpwr + 6, ypwr - 5, 1, 4); } // > 56
    if (signalpwr > -57) { wifiSignalLevel = 5; u8g2.drawBox(xpwr + 8, ypwr - 6, 1, 5); } // > 70
    if (signalpwr > -50) { wifiSignalLevel = 6; u8g2.drawBox(xpwr + 10, ypwr - 8, 1, 7);} // > 84
  }

  if ((WiFi.status() == WL_CONNECTED) && (mode == 1))
  {
      // We draw lines
    if (signalpwr > -88) { wifiSignalLevel = 1; u8g2.drawBox(xpwr, ypwr - 2, 1, 1); }     // 0-14
    if (signalpwr > -81) { wifiSignalLevel = 2; u8g2.drawBox(xpwr + 2, ypwr - 3, 1, 2); } // > 28
    if (signalpwr > -74) { wifiSignalLevel = 3; u8g2.drawBox(xpwr + 4, ypwr - 4, 1, 3); } // > 42
    if (signalpwr > -66) { wifiSignalLevel = 4; u8g2.drawBox(xpwr + 6, ypwr - 5, 1, 4); } // > 56
    if (signalpwr > -57) { wifiSignalLevel = 5; u8g2.drawBox(xpwr + 8, ypwr - 6, 1, 5); } // > 70
    if (signalpwr > -50) { wifiSignalLevel = 6; u8g2.drawBox(xpwr + 10, ypwr - 7, 1, 6);} // > 84
  }


  if (print == true) // If the print flag =1, we print the signal strength in % on a scale of 1-6 on the serial line.
  {
    for (int j = 0; j < 100; j++) 
    {
      if (signal_dBM[j] == signalpwr) 
      {
        Serial.print("debug WiFi -> WiFi Signal: ");
        Serial.print(signal_percent[j]);
        Serial.print("%  Level: ");
        Serial.print(wifiSignalLevel);
        Serial.print("   dBm: ");
        Serial.println(signalpwr);
        
        break;
      }
    }
  }
}


void rcInputKey(uint8_t i)
{
  rcInputDigitsMenuEnable = true;
  if (bankMenuEnable == true)
  {
    
    if (i == 0) {i = 10;}
    bank_nr = i;
    bankMenuDisplay();
  }
  else
  {
    timeDisplay = false;
    displayActive = true;
    displayStartTime = millis(); 
    
    if (rcInputDigit1 == 0xFF)
    {
      rcInputDigit1 = i;
    }
    else 
    {
      rcInputDigit2 = i;
    }

    int y = 35;
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_fub14_tf); // 14x11 cm
    u8g2.drawStr(65, y, "Station:"); 
    if (rcInputDigit1 != 0xFF)
    {u8g2.drawStr(153, y, String(rcInputDigit1).c_str());} 
    //else {u8g2.drawStr(153, x,"_" ); }
    else {u8g2.drawStr(153, y,"_" ); }
    
    if (rcInputDigit2 != 0xFF)
    {u8g2.drawStr(164, y, String(rcInputDigit2).c_str());} 
    else {u8g2.drawStr(164, y,"_" ); }



    if ((rcInputDigit1 != 0xFF) && (rcInputDigit2 != 0xFF)) // if both values ​​are not empty
    {
      station_nr = (rcInputDigit1 *10) + rcInputDigit2;
    }
    else if ((rcInputDigit1 != 0xFF) && (rcInputDigit2 == 0xFF))  // if we only gave one digit
    { 
      station_nr = rcInputDigit1;
    }

    if (station_nr > stationsCount)  // we check whether the entered value does not exceed the number of stations in a given bank
    {
      station_nr = stationsCount;  // if the entered value is greater than the number of stations, we set the value
    }
    
    if (station_nr < 1)
    {
      station_nr = stationFromBuffer;
    }

    // Reading the station for a given PSRAM memory cell:
    char station[STATION_NAME_LENGTH + 1];  // An array for the station name with a maximum length defined by STATION_NAME_LENGTH
    memset(station, 0, sizeof(station));    // Clearing the table with zeros before writing the data
    int length = psramData[(station_nr - 1) * (STATION_NAME_LENGTH + 1)];   // Read station name length from PSRAM for current station index
    
    for (int j = 0; j < min(length, STATION_NAME_LENGTH); j++) { // Read the station name from PSRAM as a sequence of bytes, up to a maximum of STATION_NAME_LENGTH
      station[j] = psramData[(station_nr - 1) * (STATION_NAME_LENGTH + 1) + 1 + j];  // Read the station name character by character
    }
    u8g2.setFont(spleen6x12PL);
    String stationNameText = String(station);
    stationNameText = stationNameText.substring(0, 25); // We trim to 23 characters

    u8g2.drawLine(0,48,256,48);
    u8g2.setFont(spleen6x12PL);
    u8g2.setCursor(0, 60);
    u8g2.print("Bank:" + String(bank_nr) + ", 1-" + String(stationsCount) + "     " + stationNameText);
    u8g2.sendBuffer();

    if ((rcInputDigit1 !=0xFF) && (rcInputDigit2 !=0xFF)) // if we entered both numbers, we clear the fields so that we can enter them again
    {
      rcInputDigit1 = 0xFF; // we clear the number 1, the empty variable flag F so that when we press it again we can print the number without waiting 6 seconds
      rcInputDigit2 = 0xFF; // we clear the number 2, the empty variable flag F
    }
  }
  #ifdef IR_LED
    irLedBlink(); 
  #endif 
}

// Function for saving station number and bank number on SD card
void saveStationOnSD() 
{
  if ((station_nr !=0) && (bank_nr != 0))
  {
    // Check if the station_nr.txt file exists

    Serial.print("debug SD -> We save the bank: ");
    Serial.println(bank_nr);
    Serial.print("debug SD -> ZWe save the station: ");
    Serial.println(station_nr);

    // Check if the station_nr.txt file exists
    if (STORAGE.exists("/station_nr.txt")) {
      Serial.println("debug SD -> The file station_nr.txt already exists.");

      // Open the file for writing and overwrite the current value of station_nr
      myFile = STORAGE.open("/station_nr.txt", FILE_WRITE);
      if (myFile) {
        myFile.println(station_nr);
        myFile.close();
        Serial.println("debug SD -> Updating station_nr.txt on SD card");
      } else {
        Serial.println("debug SD -> Error opening file station_nr.txt.");
      }
    } else {
      Serial.println("debug SD -> File station_nr.txt does not exist. Creating...");

      // Create a file and save the current value of station_nr in it
      myFile = STORAGE.open("/station_nr.txt", FILE_WRITE);
      if (myFile) {
        myFile.println(station_nr);
        myFile.close();
        Serial.println("debug SD -> Created and saved station_nr.txt on SD card");
      } else {
        Serial.println("debug SD -> Error creating station_nr.txt file.");
      }
    }

    // Check if the file bank_nr.txt exists
    if (STORAGE.exists("/bank_nr.txt")) 
    {
      Serial.println("debug SD -> The file bank_nr.txt already exists.");

      // Open the file for saving and overwrite the current value of bank_nr
      myFile = STORAGE.open("/bank_nr.txt", FILE_WRITE);
      if (myFile) {
        myFile.println(bank_nr);
        myFile.close();
        Serial.println("debug SD -> Updating bank_nr.txt on the STORAGE tab.");
      } else {
        Serial.println("debug SD -> Error opening file bank_nr.txt.");
      }
    } 
    else 
    {
      Serial.println("debug SD -> File bank_nr.txt does not exist. Creating...");

      // Create a file and save the current value of bank_nr in it
      myFile = STORAGE.open("/bank_nr.txt", FILE_WRITE);
      if (myFile) {
        myFile.println(bank_nr);
        myFile.close();
        Serial.println("debug SD -> Created and saved bank_nr.txt on SD card");
      } else {
        Serial.println("debug SD -> Error creating file bank_nr.txt.");
      }
    }
    
    if (noSDcard == true)
    {
      Serial.println("debug SD -> No SD card, write to EEPROM");
      EEPROM.write(0, station_nr);
      EEPROM.write(1, bank_nr);
      EEPROM.commit();
    }
  }
  else
  {
    Serial.println("debug ChangeStation -> We play from the URL, we do not save the station and bank");  
  }
}


void startFadeIn(uint8_t target)
{
  targetVolume = target;
  currentVolume = 0;
  audio.setVolume(0);
  fadingIn = true;
  lastFadeTime = millis();
}

void handleFadeIn()
{
  if (!fadingIn) return;  // if fade-in is inactive, do nothing
  if (millis() - lastFadeTime >= fadeStepDelay)
  {
    lastFadeTime = millis();

    if (currentVolume < targetVolume)
    {
      audio.setVolume(currentVolume);
      currentVolume++;
    }
    else
    {
      fadingIn = false; // end fade-in
      audio.setVolume(targetVolume);
    }
  }
}

void volumeFadeOut(uint8_t time_ms)
{
  for (uint8_t i = volumeValue; i > 0; i--)
  {
    audio.setVolume(i);
    delay(time_ms);
  } 
}


void changeStation() 
{
  //audio.setVolume(0);
  fwupd = false;
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_fub14_tf); // 14x11 cm
  u8g2.drawStr(34, 33, "Loading stream..."); // 8 characters x 11 wide
  //u8g2.drawStr(51, 33, "Loading stream"); // 8 characters x 11 wide
  u8g2.sendBuffer();

  mp3 = flac = aac = vorbis = opus = false;
  streamCodec = "-";
  //stationLogoUrl = "";
  
  if (urlPlaying) {bank_nr = previous_bank_nr;}  // We restore the last bank number after playing with ULR where we set the bank to 0
    
  // Removing all characters from objects
  stationString.remove(0);  
  stationNameStream.remove(0);
  stationStringWeb.remove(0);
  stationLogoUrl.remove(0);

  bitrateString.remove(0);
  sampleRateString.remove(0);
  bitsPerSampleString.remove(0); 
  SampleRate = 0;
  SampleRateRest = 0;

  sampleRateString = "--.-";
  bitsPerSampleString = "--";
  bitrateString = "---";

  Serial.println("debug changeStation -> Read station from PSRAM");
  String stationUrl = "";

  // Reading the station for a given PSRAM memory cell:
  char station[STATION_NAME_LENGTH + 1];  // An array for the station name with a maximum length defined by STATION_NAME_LENGTH
  memset(station, 0, sizeof(station));    // Clearing the table with zeros before writing the data
  int length = psramData[(station_nr - 1) * (STATION_NAME_LENGTH + 1)];   // Read station name length from PSRAM for current station index
      
  for (int j = 0; j < min(length, STATION_NAME_LENGTH); j++) 
  { // Read the station name from PSRAM as a sequence of bytes, up to a maximum of STATION_NAME_LENGTH
    station[j] = psramData[(station_nr - 1) * (STATION_NAME_LENGTH + 1) + 1 + j];  // Read the station name character by character
  }
  
  //Serial.println("---WE ARE CURRENTLY PLAYING-- ");
  //Serial.print(station_nr - 1);
  //Serial.print(" ");
  //Serial.println(String(station));

  String line = String(station); // We assign the data read from PSRAM to the line variable
  
  // Extract the first 42 characters and assign it to stationName
  stationName = line.substring(0, 41);  //42 Copy the first 42 characters from the line
  Serial.print("debug changeStation -> Station name: ");
  Serial.println(stationName);

  // Find part of the URL in a line, e.g. after the station number
  int urlStart = line.indexOf("http");  // We are looking for the place where the URL begins
  if (urlStart != -1) 
  {
    stationUrl = line.substring(urlStart);  // We extract the URL from "http"
    stationUrl.trim();                      // We remove whitespace at the beginning and end
  }
  else
  {
	return;
  }
  
  if (stationUrl.isEmpty()) // if the URL link is empty
  {
    Serial.println("debug changeStation -> Error: No station found for the given number.");
    return;
  }
  
  //Serial.print("URL: ");
  //Serial.println(stationUrl);

  // Verifying if the link contains "http" or "https"
  if (stationUrl.startsWith("http://") || stationUrl.startsWith("https://")) 
  {
    // Print the station name and link on the show
    Serial.print("debug changeStation -> Currently selected station: ");
    Serial.println(station_nr);
    Serial.print("debug changeStation -> Link to the station: ");
    Serial.println(stationUrl);
    
    u8g2.setFont(spleen6x12PL);  // we print out what stream and what station is being loaded
    
    int StationNameEnd = stationName.indexOf("  "); // We cut out the station name only up to the double space
    stationName = stationName.substring(0, StationNameEnd);
    
    int stationNameWidth = u8g2.getStrWidth(stationName.substring(0, stationNameLenghtCut).c_str()); // Counts the positions to display stationName in the center
    int stationNamePositionX = (SCREEN_WIDTH - stationNameWidth) / 2;
    
    u8g2.drawStr(stationNamePositionX, 55, String(stationName.substring(0, stationNameLenghtCut)).c_str());
    u8g2.sendBuffer();
    
    // Smooth fade before changing stations if enabled
    if (f_volumeFadeOn && !volumeMute) {volumeFadeOut(volumeFadeOutTime);}
    
    // Connect to a specific station
    audio.connecttohost(stationUrl.c_str());
    
    // We only enable mute if MUTE is off. If MUTE is off and mute is off, we set the volume according to volumeValue.
    if (f_volumeFadeOn && !volumeMute) {startFadeIn(volumeValue);} else if (!volumeMute) {audio.setVolume(volumeValue);}   
    
    // We write down what station number and which bank we are playing only if they have changed
    //if ((station_nr != stationFromBuffer || bank_nr != previous_bank_nr) && f_saveVolumeStationAlways) {saveStationOnSD();}
    
    
    if (station_nr != 0 ) {stationFromBuffer = station_nr;} 
    if (f_saveVolumeStationAlways) {saveStationOnSD();} 
    //saveStationOnSD(); // We write down what station number and which bank we are playing

    urlPlaying = false; // We remove the playback flag from the address sent from the website
       
  } 
  else 
  {
    Serial.println("debug changeStation -> Error: Station link does not contain 'http' or 'https''");
    Serial.println("debug changeStation -> URL read: " + stationUrl);
  }
  
  currentSelection = station_nr - 1; // we set the stations on the list to the currently playing one after changing the station
  firstVisibleLine = currentSelection + 1; // the first visible line is the playing station at the start
  if (currentSelection + 1 >= stationsCount - 1) 
  {
   firstVisibleLine = currentSelection - 3;
  }
  ActionNeedUpdateTime = true;
   
  wsStationChange(station_nr, bank_nr); // From now on, Event Audio is responsible for station info
  wsAudioRefresh = true;
  
  //audio.setVolume(volumeValue); // We restore the volume setting
}

// A function for displaying a list of radio stations with the option of selecting by marking in negative
void displayStations() 
{
  listedStations = true;
  u8g2.clearBuffer();  // Clear the buffer before drawing to prepare the screen for new content.
  u8g2.setFont(spleen6x12PL);
  u8g2.setCursor(20, 10);                                          // Set cursor position (x=60, y=10) for header
  u8g2.print("BANK: " + String(bank_nr));                                          // Display "BANK:" header
  u8g2.setCursor(68, 10);                                          // Set cursor position (x=60, y=10) for header
  u8g2.print(" - RADIO STATIONS: ");                                // Display the "Radio Stations:" heading
  u8g2.print(String(station_nr) + " / " + String(stationsCount));  // Add the current station number and the counter for all stations
  u8g2.drawLine(0,11,256,11);
  // "BANK: 16 - RADIO STATIONS: 99 / 99


  int displayRow = 1;  // Variable for line number, starting from the second (first is the header)
  
  //erial.print("FirstVisibleLine:");
  //Serial.print(firstVisibleLine);

  // Displaying stations starting from the second line (y=21)
  for (int i = firstVisibleLine; i < min(firstVisibleLine + maxVisibleLines, stationsCount); i++) 
  {
    char station[STATION_NAME_LENGTH + 1];  // An array for the station name with a maximum length defined by STATION_NAME_LENGTH
    memset(station, 0, sizeof(station));    // Clearing the table with zeros before writing the data

    // Read station name length from PSRAM for current station index
    int length = psramData[i * (STATION_NAME_LENGTH + 1)];  //----------------------------------------------

    // Read the station name from PSRAM as a sequence of bytes, up to a maximum of STATION_NAME_LENGTH
    for (int j = 0; j < min(length, STATION_NAME_LENGTH); j++) 
    {
      station[j] = psramData[i * (STATION_NAME_LENGTH + 1) + 1 + j];  // Read the station name character by character
    }

    // Check if the current station is the one currently selected
    if (i == currentSelection) 
    {
      u8g2.setDrawColor(1);                           // Set drawing color to white
      //u8g2.drawBox(0, displayRow * 13 - 2, 256, 13);  // Draw a rectangle as a background for the selected station (x=0, width 256, height 10)
      u8g2.drawBox(0, displayRow * 13, 256, 13);  // Draw a rectangle as a background for the selected station (x=0, width 256, height 10)
      u8g2.setDrawColor(0);                           // Change the drawing color to black for the selected station text
    } else {
      u8g2.setDrawColor(1);  // Set the text color to plain white for selected stations
    }
    // Display the station name by positioning the cursor at the appropriate position
    //         u8g2.drawStr(0, displayRow * 13 + 8, String(station).c_str());
    u8g2.drawStr(0, displayRow * 13 + 10, String(station).c_str());
    //u8g2.print(station);  // Display station name

    // Go to next line (next line on screen)
    displayRow++;
  }
  // Restore default drawing color settings (white text on black background)
  u8g2.setDrawColor(1);  // White drawing color
  u8g2.sendBuffer();     // Send buffer contents to OLED screen to display changes
  Serial.print("CurrentSelection = "); Serial.println(currentSelection);
  Serial.print("firstVisibleLine = "); Serial.println(firstVisibleLine);
}

void updateTimerFlag() 
{
  ActionNeedUpdateTime = true;
}

void displayPowerSave(bool saveON)
{
  if ((saveON == 1) && (displayActive == false) && (fwupd == false) && (audio.isRunning() == true)) {u8g2.setPowerSave(1);}
  if (saveON == 0) {u8g2.setPowerSave(0);}
}



// A function called every second by the timer to update the time on the display
void updateTime() 
{
  u8g2.setDrawColor(1);  // Set color to white
  bool showDots;

  if (timeDisplay == true) 
  {

    if ((timeDisplay == true) && (audio.isRunning() == true))
    {
      // A structure that stores time information
      struct tm timeinfo;
      
      // Check if the time was successfully retrieved from the local real-time clock
      if (!getLocalTime(&timeinfo, 3000)) 
      {
        // Display time fetch failure message
        Serial.println("debug time -> Couldn't get time");
        return;  // Terminate function when time failed
      }
      
      showDots = (timeinfo.tm_sec % 2 == 0); // Even second = show colon

      // Convert hour, minute and second to string in "HH:MM:SS" format"
      char timeString[9];  // A buffer that stores the time in text form
      if ((timeinfo.tm_min == 0) && (timeinfo.tm_sec == 0) && (timeVoiceInfoEveryHour == true) && (f_voiceTimeBlocked == false)) {f_requestVoiceTimePlay = true; f_voiceTimeBlocked = true;} // We check whether the hour has struck, if the voice announcement is turned on every hour, we play it
      if ((displayMode == 0) || (displayMode == 2)) // Basic mode and 3 lines of text
      { 
        //snprintf(timeString, sizeof(timeString), "%2d:%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
        if (showDots) snprintf(timeString, sizeof(timeString), "%2d:%02d", timeinfo.tm_hour, timeinfo.tm_min);
        else snprintf(timeString, sizeof(timeString), "%2d %02d", timeinfo.tm_hour, timeinfo.tm_min);
        u8g2.setFont(spleen6x12PL);
        //u8g2.drawStr(208, 63, timeString);
        u8g2.drawStr(226, 63, timeString);
      }
      else if (displayMode == 1) // Tryb - Duzy zegar
      {
        int xtime = 0;
        u8g2.setFont(u8g2_font_7Segments_26x42_mn);
        
        //snprintf(timeString, sizeof(timeString), "%2d:%02d", timeinfo.tm_hour, timeinfo.tm_min);

        //Flashing dots
        if (showDots) snprintf(timeString, sizeof(timeString), "%2d:%02d", timeinfo.tm_hour, timeinfo.tm_min);
        else snprintf(timeString, sizeof(timeString), "%2d %02d", timeinfo.tm_hour, timeinfo.tm_min);
        u8g2.drawStr(xtime+7, 45, timeString); 
        
        u8g2.setFont(u8g2_font_fub14_tf); // 14x11
        snprintf(timeString, sizeof(timeString), "%02d", timeinfo.tm_mday);
        u8g2.drawStr(203,17, timeString);

        String month = "";
        switch (timeinfo.tm_mon) {
        case 0: month = "JAN"; break;     
        case 1: month = "FEB"; break;     
        case 2: month = "MAR"; break;
        case 3: month = "APR"; break;
        case 4: month = "MEI"; break;
        case 5: month = "JUN"; break;
        case 6: month = "JUL"; break;
        case 7: month = "AUG"; break;
        case 8: month = "SEP"; break;
        case 9: month = "OKT"; break;
        case 10: month = "NOV"; break;
        case 11: month = "DEC"; break;                                
        }
        u8g2.setFont(spleen6x12PL);
        u8g2.drawStr(232,14, month.c_str());

        String dayOfWeek = "";
        switch (timeinfo.tm_wday) {
        case 0: dayOfWeek = " Zondag  "; break;     
        case 1: dayOfWeek = " Maandag  "; break;     
        case 2: dayOfWeek = " Dinsdag "; break;
        case 3: dayOfWeek = "Woensdag"; break;
        case 4: dayOfWeek = "Donderdag "; break;
        case 5: dayOfWeek = " Vrijdag  "; break;
        case 6: dayOfWeek = "Zaterdag "; break;
        }
        
        u8g2.drawRBox(198,20,58,15,3);  // Box with rounded corners, white under the day of the week
        u8g2.drawLine(198,20,256,20); // Day of month/day of week separation line
        u8g2.setDrawColor(0);
        u8g2.drawStr(201,31, dayOfWeek.c_str());
        u8g2.setDrawColor(1);
        u8g2.drawRFrame(198,0,58,35,3); // Frame on the entire calendar

        snprintf(timeString, sizeof(timeString), ":%02d", timeinfo.tm_sec);
        u8g2.drawStr(xtime+163, 45, timeString);
      }
      else if (displayMode == 3)
      { 
        if (showDots) snprintf(timeString, sizeof(timeString), "%2d:%02d", timeinfo.tm_hour, timeinfo.tm_min);
        else snprintf(timeString, sizeof(timeString), "%2d %02d", timeinfo.tm_hour, timeinfo.tm_min);
        u8g2.setFont(spleen6x12PL);
        //u8g2.drawStr(113, 11, timeString);
        u8g2.drawStr(226, 10, timeString);
      }
      else if (displayMode == 7)
      { 
        if (showDots) snprintf(timeString, sizeof(timeString), "%2d:%02d", timeinfo.tm_hour, timeinfo.tm_min);
        else snprintf(timeString, sizeof(timeString), "%2d %02d", timeinfo.tm_hour, timeinfo.tm_min);
        u8g2.setFont(spleen6x12PL);
        u8g2.drawStr(50, 8, timeString);
      }
    
    }
    else if ((timeDisplay == true) && (audio.isRunning() == false))
    {
      displayPowerSave(0); 
      // A structure that stores time information
      struct tm timeinfo;
      
      if (!getLocalTime(&timeinfo,3000)) 
      {
        Serial.println("debug time -> Couldn't get time");
        return;  // Terminate function when time failed
      }
      showDots = (timeinfo.tm_sec % 2 == 0); // Even second = show colon
      char timeString[9];  // A buffer that stores the time in text form
      
      //snprintf(timeString, sizeof(timeString), "%02d:%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
      if (showDots) snprintf(timeString, sizeof(timeString), "%2d:%02d", timeinfo.tm_hour, timeinfo.tm_min);
      else snprintf(timeString, sizeof(timeString), "%2d %02d", timeinfo.tm_hour, timeinfo.tm_min);
      u8g2.setFont(spleen6x12PL);
      
      if ((displayMode == 0) || (displayMode == 2)) { u8g2.drawStr(0, 63, "                         ");}
      if (displayMode == 1) { u8g2.drawStr(0, 33, "                         ");}
      if (displayMode == 3) { u8g2.drawStr(53, 62, "                         ");}

      if (millis() - lastCheckTime >= 1000)
      {
        if ((displayMode == 0) || (displayMode == 2)) { u8g2.drawStr(0, 63, "... No audio stream ! ...");}
        if (displayMode == 1) { u8g2.drawStr(0, 33, "... No audio stream ! ...");}
        if (displayMode == 3) { u8g2.drawStr(53, 62 , "... No audio stream ! ...");}
        lastCheckTime = millis(); // Update last checked time
      }       
      if ((displayMode == 0) || (displayMode == 1) || (displayMode == 2)) { u8g2.drawStr(226, 63, timeString);}
      if (displayMode == 3) { u8g2.drawStr(226, 10, timeString);}
    }
  }
}

void saveEqualizerOnSD() 
{
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_fub14_tf); // 14x11 cm
  u8g2.drawStr(1, 33, "Saving equalizer settings"); // 8 characters x 11 wide
  u8g2.sendBuffer();
  
  
  // Check if the equalizer.txt file exists

  Serial.print("Filtr High: ");
  Serial.println(toneHiValue);
  
  Serial.print("Filtr Mid: ");
  Serial.println(toneMidValue);
  
  Serial.print("Filtr Low: ");
  Serial.println(toneLowValue);
  
  Serial.print("Balance: "); 
  Serial.println(balanceValue);
  
  // Check if the file exists
  if (STORAGE.exists("/equalizer.txt")) 
  {
    Serial.println("debug SD -> The file equalizer.txt already exists.");

    // Open the file for saving and overwrite the current value of the equalizer filters
    myFile = STORAGE.open("/equalizer.txt", FILE_WRITE);
    if (myFile) 
	  {
      myFile.println(toneHiValue);
	    myFile.println(toneMidValue);
	    myFile.println(toneLowValue);
      myFile.println(balanceValue);
      myFile.close();
      Serial.println("debug SD -> Update equalizer.txt on SD card");
    } 
	  else 
	  {
      Serial.println("debug SD -> Error opening equalizer.txt file.");
    }
  } 
  else 
  {
    Serial.println("debug SD -> The file equalizer.txt does not exist. Creating...");

    // Create a file and save the current value of the equalizer filters in it
    myFile = STORAGE.open("/equalizer.txt", FILE_WRITE);
    if (myFile) 
	  {
      myFile.println(toneHiValue);
	    myFile.println(toneMidValue);
	    myFile.println(toneLowValue);
      myFile.println(balanceValue);
      myFile.close();
      Serial.println("debug SD -> Created and saved equalizer.txt on SD card");
    } 
	  else 
	  {
      Serial.println("debug SD -> Error creating equalizer.txt file.");
    }
  }
}

void readEqualizerFromSD() 
{
  // Check if the SD card is available
  //if (!STORAGE.begin(SD_CS))
  if (!STORAGE_BEGIN()) 
  {
    Serial.println("debug SD -> Cannot find SD card, sets Equalizer filter values ​​to default.");
    toneHiValue = 0;  // Default filter value when no SD card is present
	  toneMidValue = 0; // Default filter value when no SD card is present
	  toneLowValue = 0; // Default filter value when no SD card is present
    balanceValue = 0;
    return;
  }

  // Check if the equalizer.txt file exists
  if (STORAGE.exists("/equalizer.txt")) 
  {
    myFile = STORAGE.open("/equalizer.txt");
    if (myFile) 
    {
      toneHiValue = myFile.parseInt();
	    toneMidValue = myFile.parseInt();
	    toneLowValue = myFile.parseInt();
      balanceValue = myFile.parseInt();
      myFile.close();
      
      Serial.println("debug SD -> Equalizer.txt loaded from SD card: ");
	  
      Serial.print("debug SD -> High equalizer filter read from SD: ");
      Serial.println(toneHiValue);
    
      Serial.print("debug SD -> Mid equalizer filter read from SD: ");
      Serial.println(toneMidValue);
    
      Serial.print("debug SD -> Low equalizer filter read from SD: ");
      Serial.println(toneLowValue);

      Serial.print("debug SD -> Balance value read from SD: ");
      Serial.println(balanceValue);
        
    } 
	  else 
	  {
      Serial.println("debug SD -> Error opening equalizer.txt file.");
    }
  } 
  else 
  {
    Serial.println("debug SD -> The equalizer.txt file does not exist.");
    toneHiValue = 0;  // Default filter value when no SD card is present
	  toneMidValue = 0; // Default filter value when no SD card is present
	  toneLowValue = 0; // Default filter value when no SD card is present
    balanceValue = 0;
  }
  audio.setTone(toneLowValue, toneMidValue, toneHiValue); // We set the filters - adjustment range -40 + 6dB as int8_t with sign
  audio.setBalance(balanceValue);
}

// Function for reading radio station data from an SD card
void readStationFromSD() 
{
  // Check if the SD card is available
  //if (!STORAGE.begin(SD_CS)) 
  if (!STORAGE_BEGIN())
  {
    //Serial.println("Cannot find STORAGE card. Setting default values: Station=1, Bank=1.");
    Serial.println("debug SD -> Cannot find SD card, setting values ​​from EEPROM");
    //station_nr = 1;  // Default station number when no SD card is present
    //bank_nr = 1;     // Default bank number when no SD card is present
    EEPROM.get(0, station_nr);
    EEPROM.get(1, bank_nr);
    
    Serial.print("debug EEPROM -> EEPROM Reading Station: ");
    Serial.println(station_nr);
    Serial.print("debug EEPROM -> EEPROM Reading Bank: ");
    Serial.println(bank_nr);

    if ((station_nr > 99) || (station_nr == 0)) { station_nr = 1;} // protection in case of incorrect reading of EEPROM or value
    if ((bank_nr > bank_nr_max) || (bank_nr == 0)) { bank_nr = 1;}
    
    return;
  }

  // Check if the station_nr.txt file exists
  if (STORAGE.exists("/station_nr.txt")) {
    myFile = STORAGE.open("/station_nr.txt");
    if (myFile) {
      station_nr = myFile.parseInt();
      myFile.close();
      Serial.print("debug SD -> Loaded station_nr from SD card: ");
      Serial.println(station_nr);
    } else {
      Serial.println("debug SD -> Error opening file station_nr.txt.");
    }
  } else {
    Serial.println("debug SD -> The file station_nr.txt does not exist.");
    station_nr = 9;  // we set stations if there is no file on the card
  }

  // Check if the file bank_nr.txt exists
  if (STORAGE.exists("/bank_nr.txt")) {
    myFile = STORAGE.open("/bank_nr.txt");
    if (myFile) {
      bank_nr = myFile.parseInt();
      myFile.close();
      Serial.print("debug SD -> Bank_no. loaded from SD card: ");
      Serial.println(bank_nr);
    } else {
      Serial.println("debug SD -> Error opening file bank_nr.txt.");
    }
  } else {
    Serial.println("debug SD -> The file bank_nr.txt does not exist.");
    bank_nr = 1;  // // we set the bank in case of no file on the card
  }
}

void vuMeterMode0() 
{
  uint16_t raw = audio.getVUlevel();
  vuMeterR = (raw >> 8) & 0xFF;
  vuMeterL = raw & 0xFF;

  //vuMeterL = constrain(vuMeterL, 0, 240);
  //vuMeterR = constrain(vuMeterR, 0, 240);

  // prevents sudden drops in very strong signal
  //if (vuMeterL < displayVuL - 8) vuMeterL = displayVuL - 8;
  //if (vuMeterR < displayVuR - 8) vuMeterR = displayVuR - 8;

  
  //vuMeterR = audio.getVUlevel() & 0xFF;  // we extract the L channel from the int16 type variable
  //vuMeterL = audio.getVUlevel() >> 8;  // we pull out the P channel from the higher shelf

  //vuMeterL = constrain(vuMeterL, 0, 243);
  //vuMeterR = constrain(vuMeterR, 0, 243);

  //vuMeterR = map(vuMeterR, 0, 255, 0, 240);
  //vuMeterL = map(vuMeterL, 0, 255, 0, 240);  // 244 VU + start from x=10 + peak hold 2px

  if (vuMeterR < 1) vuMeterR = 1;
  if (vuMeterL < 1) vuMeterL = 1;

  //if (vuMeterR > 230) vuMeterR = 230;
  //if (vuMeterL > 230) vuMeterL = 230;
  //vuMeterL = constrain(vuMeterL, 0, 230);
 // vuMeterR = constrain(vuMeterR, 0, 230);

  if (vuSmooth)
  {
    // LEFT
    /*
    if (vuMeterL > displayVuL) 
    {
      displayVuL += vuRiseSpeed;
      if (displayVuL > vuMeterL) 
      displayVuL = vuMeterL;
    } 
    */

    if (vuMeterL > displayVuL) 
    {
      if (displayVuL > 255 - vuRiseSpeed) displayVuL = 255;
      else
      displayVuL += vuRiseSpeed;

      if (displayVuL > vuMeterL) displayVuL = vuMeterL;
    }


    
    
    else if (vuMeterL < displayVuL) 
    {
      if (displayVuL > vuFallSpeed) 
      {
        displayVuL -= vuFallSpeed;
      } 
      else 
      {
        //displayVuL = 0;
        displayVuL = vuMeterL;
      }
    }

    // RIGHT
    /*
    if (vuMeterR > displayVuR) 
    {
      displayVuR += vuRiseSpeed;
      if (displayVuR > vuMeterR) displayVuR = vuMeterR;
    } 
    */

    if (vuMeterR > displayVuR) 
    {
      if (displayVuR > 255 - vuRiseSpeed) displayVuR = 255;
      else
      displayVuR += vuRiseSpeed;

      if (displayVuR > vuMeterR) displayVuR = vuMeterR;
    }  

    else if (vuMeterR < displayVuR) 
    {
      if (displayVuR > vuFallSpeed) 
      {
        displayVuR -= vuFallSpeed;
      } 
      else 
      {
        //displayVuR = 0;
        displayVuR = vuMeterR;
      }
    }
  }

  // Left channel peak&hold update
  if (vuSmooth)
  {
    if (vuPeakHoldOn)
    {
      // --- Peak L ---
      if (displayVuL >= peakL) 
      {
        peakL = displayVuL;
        peakHoldTimeL = 0;
      } 
      else 
      {
        if (peakHoldTimeL < peakHoldThreshold) 
        {
          peakHoldTimeL++;
        } 
        else 
        {
          if (peakL > 0) peakL--;
        }
      }

      // --- Peak R ---
      if (displayVuR >= peakR) 
      {
        peakR = displayVuR;
        peakHoldTimeR = 0;
      } 
      else 
      {
        if (peakHoldTimeR < peakHoldThreshold) 
        {
          peakHoldTimeR++;
        } 
        else 
        {
          if (peakR > 0) peakR--;
        }
      }
    }    
  }
  else
  {
    if (vuPeakHoldOn)
    {
      if (vuMeterL >= peakL) 
      {
        peakL = vuMeterL;
        peakHoldTimeL = 0;
      } 
      else 
      {
        if (peakHoldTimeL < peakHoldThreshold) 
        {
          peakHoldTimeL++;
        } 
        else 
        {
          if (peakL > 0) peakL--;
        }
      }
      // Peak&hold update for Right channel
      if (vuMeterR >= peakR) 
      {
        peakR = vuMeterR;
        peakHoldTimeR = 0;
      } 
      else 
      {
        if (peakHoldTimeR < peakHoldThreshold) 
        {
          peakHoldTimeR++;
        } 
        else 
        {
          if (peakR > 0) peakR--;
        }
      }
    }
  }


  if (volumeMute == false)  
  {
    if (vuSmooth)
    {
      u8g2.setDrawColor(0);
      u8g2.drawBox(7, vuLy, 249, 3);  //cleaning the screen under the VU meter
      u8g2.drawBox(7, vuRy, 249, 3);
      u8g2.setDrawColor(1);

      // Biale pola pod literami L i R
      u8g2.drawBox(0, vuLy - 3, 7, 7);  
      u8g2.drawBox(0, vuRy - 3, 7, 7);  

      // Rysujemy litery L i R
      u8g2.setDrawColor(0);
      u8g2.setFont(u8g2_font_04b_03_tr);
      u8g2.drawStr(2, vuLy + 3, "L");
      u8g2.drawStr(2, vuRy + 3, "R");
      u8g2.setDrawColor(1);  // We are bringing back white drawing

      if (vuMeterMode == 1)  // mode 1 continuous stripes
      {
        u8g2.setDrawColor(1);
        //u8g2.drawBox(10, vuLy, vuMeterL, 2);  // we draw lines of a length corresponding to the VU value
        //u8g2.drawBox(10, vuRy, vuMeterR, 2);

        u8g2.drawBox(10, vuLy, displayVuL, 2);  // we draw lines of a length corresponding to the VU value
        u8g2.drawBox(10, vuRy, displayVuR, 2);


        // Rysowanie peaków jako cienka kreska
        u8g2.drawBox(9 + peakL, vuLy, 1, 2);
        u8g2.drawBox(9 + peakR, vuRy, 1, 2);
      } 
      else  // vuMeterMode == 0  basic mode, dashes with gaps
      {      
        for (uint8_t vusize = 0; vusize < displayVuL; vusize++)
        {
          if ((vusize % 9) < 8) u8g2.drawBox(9 + vusize, vuLy, 1, 2);//u8g2.drawBox(9 + vusize, vuLy, 1, 2); // draw only 8 pixels, then 1px gap, 9 in the x axis is the space for the letter
        }  
        for (uint8_t vusize = 0; vusize < displayVuR; vusize++)
        {
          if ((vusize % 9) < 8) u8g2.drawBox(9 + vusize, vuRy, 1, 2); // draw only 8 pixels, then 1px gap, 9 in the x axis is the space for the letter
        }    
        
        if (vuPeakHoldOn)
        {
          // Peak - dashes in dashed mode
          u8g2.drawBox(9 + peakL, vuLy, 1, 2);
          u8g2.drawBox(9 + peakR, vuRy, 1, 2);
        }
      }
         
    }
    else
    {
      u8g2.setDrawColor(0);
      u8g2.drawBox(7, vuLy, 255, 3);  //cleaning the screen under the VU meter
      u8g2.drawBox(7, vuRy, 255, 3);
      u8g2.setDrawColor(1);

      // Biale pola pod literami L i R
      u8g2.drawBox(0, vuLy - 3, 7, 7);  
      u8g2.drawBox(0, vuRy - 3, 7, 7);  

      // Rysujemy litery L i R
      u8g2.setDrawColor(0);
      u8g2.setFont(u8g2_font_04b_03_tr);
      u8g2.drawStr(2, vuLy + 3, "L");
      u8g2.drawStr(2, vuRy + 3, "R");
      u8g2.setDrawColor(1);  // We are bringing back white drawing

      if (vuMeterMode == 1)  // mode 1 continuous stripes
      {
        u8g2.setDrawColor(1);
        u8g2.drawBox(10, vuLy, vuMeterL, 2);  // we draw lines of a length corresponding to the VU value
        u8g2.drawBox(10, vuRy, vuMeterR, 2);

        // Drawing peaks as a thin line
        u8g2.drawBox(9 + peakL, vuLy, 1, 2);
        u8g2.drawBox(9 + peakR, vuRy, 1, 2);
      } 
      else  // vuMeterMode == 0 basic mode, dashes with gaps
      {      
        for (uint8_t vusize = 0; vusize < vuMeterL; vusize++) 
        {
          if ((vusize % 9) < 8) u8g2.drawBox(9 + vusize, vuLy, 1, 2);//u8g2.drawBox(9 + vusize, vuLy, 1, 2); // draw only 8 pixels, then 1px gap, 9 in the x axis is the space for the letter
        }  
        for (uint8_t vusize = 0; vusize < vuMeterR; vusize++) 
        {
          if ((vusize % 9) < 8) u8g2.drawBox(9 + vusize, vuRy, 1, 2); // draw only 8 pixels, then 1px gap, 9 in the x axis is the space for the letter
        } 
       
        if (vuPeakHoldOn)
        {
          // Peak - dashes in dashed mode
          u8g2.drawBox(9 + peakL, vuLy, 1, 2);
          u8g2.drawBox(9 + peakR, vuRy, 1, 2);
        }
      }  
    } // else smooth
  }  // moute off
}

void vuMeterMode3() 
{
  // Download VU level
  vuMeterR = min(audio.getVUlevel() & 0xFF, 255);
  vuMeterL = min(audio.getVUlevel() >> 8, 255);

  vuMeterR = map(vuMeterR, 0, 255, 0, 127);
  vuMeterL = map(vuMeterL, 0, 255, 0, 127);

  // Smoothing
  if (vuSmooth)
  {
    /*
    if (vuMeterL > displayVuL) 
    {
      displayVuL += vuRiseSpeed;
      if (displayVuL > vuMeterL) displayVuL = vuMeterL;
    }
    */

    if (vuMeterL > displayVuL) 
    {
      if (displayVuL > 127 - vuRiseSpeed) displayVuL = 127;
      else
      displayVuL += vuRiseSpeed;

      if (displayVuL > vuMeterL) displayVuL = vuMeterL;
    }
    else 
    {
      if (displayVuL > vuFallSpeed) {
        displayVuL -= vuFallSpeed;
      } else {
        displayVuL = 0;
      }
    }

    /*
    if (vuMeterR > displayVuR) {
      displayVuR += vuRiseSpeed;
      if (displayVuR > vuMeterR) displayVuR = vuMeterR;
    } 
    */

    if (vuMeterR > displayVuR) 
    {
      if (displayVuR > 127 - vuRiseSpeed) displayVuR = 127;
      else
      displayVuR += vuRiseSpeed;

      if (displayVuR > vuMeterR) displayVuR = vuMeterR;
    }
    else 
    {
      if (displayVuR > vuFallSpeed) {
        displayVuR -= vuFallSpeed;
      } else {
        displayVuR = 0;
      }
    }
  }

  // Peak & Hold
  if (vuPeakHoldOn)
  {
    // LEFT
    if ((vuSmooth ? displayVuL : vuMeterL) >= peakL) {
      peakL = vuSmooth ? displayVuL : vuMeterL;
      peakHoldTimeL = 0;
    } else {
      if (peakHoldTimeL < peakHoldThreshold) {
        peakHoldTimeL++;
      } else if (peakL > 0) {
        peakL--;
      }
    }

    // RIGHT
    if ((vuSmooth ? displayVuR : vuMeterR) >= peakR) {
      peakR = vuSmooth ? displayVuR : vuMeterR;
      peakHoldTimeR = 0;
    } else {
      if (peakHoldTimeR < peakHoldThreshold) {
        peakHoldTimeR++;
      } else if (peakR > 0) {
        peakR--;
      }
    }
  }

  if (!volumeMute)
  {
    // VU Line Cleaning
    u8g2.setDrawColor(0);
    u8g2.drawBox(0, vuYmode3, 240, vuThicknessMode3);
    u8g2.setDrawColor(1);

    // Selecting values ​​for drawing: smooth or rough
    int leftLevel = vuSmooth ? displayVuL : vuMeterL;
    int rightLevel = vuSmooth ? displayVuR : vuMeterR;

    // Drawing the LEFT channel - from the center to the left
    for (int i = 0; i < leftLevel; i++) {
      if ((i % 9) < 8) {  // 8px kreska + 1px przerwa
        int x = vuCenterXmode3 - 2 - i;  // -1 żeby nie nakładać się na środek
        if (x >= 0) u8g2.drawBox(x, vuYmode3, 1, vuThicknessMode3);
      }
    }

    // Drawing the RIGHT channel - from the center to the right
    for (int i = 0; i < rightLevel; i++) {
      if ((i % 9) < 8) {
        int x = vuCenterXmode3 + 2 + i;
        if (x < 240) u8g2.drawBox(x, vuYmode3, 1, vuThicknessMode3);
      }
    }

    // Drawing PEAKS
    if (vuPeakHoldOn) {
      int peakLeftX = vuCenterXmode3 - 1 - peakL;
      int peakRightX = vuCenterXmode3 + peakR;
      if (peakLeftX >= 0) u8g2.drawBox(peakLeftX, vuYmode3, 1, vuThicknessMode3);
      if (peakRightX < 240) u8g2.drawBox(peakRightX, vuYmode3, 1, vuThicknessMode3);
    }
  }
}

/*
void vuMeterMode4() // Mode4 large semi-circular VU meters
{
  // Download VU levels (0–255)
  uint8_t rawL = audio.getVUlevel() >> 8;
  uint8_t rawR = audio.getVUlevel() & 0xFF;

  // Scaling to 0–100
  int vuL = map(rawL, 0, 255, 0, 100);
  int vuR = map(rawR, 0, 255, 0, 100);

  // Analog inertia
  if (vuSmooth) {
    // Left
    if (vuL > displayVuL) {
      displayVuL += max(1, (vuL - displayVuL) / vuRiseNeedleSpeed);  // szybciej w górę
    } else if (vuL < displayVuL) {
      displayVuL -= max(1, (displayVuL - vuL) / vuFallNeedleSpeed); // wolniej w dół
    }

    // Right
    if (vuR > displayVuR) {
      displayVuR += max(1, (vuR - displayVuR) / vuRiseNeedleSpeed);
    } else if (vuR < displayVuR) {
      displayVuR -= max(1, (displayVuR - vuR) / vuFallNeedleSpeed);
    }

    displayVuL = constrain(displayVuL, 0, 100);
    displayVuR = constrain(displayVuR, 0, 100);
  } else {
    displayVuL = vuL;
    displayVuR = vuR;
  }

  // Indicator parameters
  const int radius = 45;
  const int frameWidth = 120;
  const int frameHeight = 60;

  int centerXL = 65;
  int centerYL = 63;

  int centerXR = 195;
  int centerYR = 63;

  const int arcStartDeg = -60;
  const int arcEndDeg = 60;

  // Cleaning the screen
  u8g2.setDrawColor(0);
  u8g2.drawBox(0, 0, 256, 64);
  u8g2.setDrawColor(1);
  u8g2.setFont(u8g2_font_6x10_tr);

  // Structure of dB descriptions
  struct Mark {
    int angle;
    const char* label;
  };

  const Mark scaleMarks[] = {
    { -60, "-20" },
    { -40, "-10" },
    { -20, "-5"  },
    {   0,  "0"   },
    {  20, "+3"  },
    {  40, "+6"  },
    {  60, "+9"  },
  };

  // A function that draws a pointer arc
  auto drawVUArc = [&](int cx, int cy, const char* label) 
  {
    // Frame
    u8g2.drawFrame((cx - frameWidth / 2), cy - radius - 18, frameWidth, frameHeight);

    // Arc scale
    for (int a = arcStartDeg; a <= arcEndDeg; a += 6) 
    {
      float angle = radians(a);
      int x1 = cx + sin(angle) * (radius - 4);
      int y1 = cy - cos(angle) * (radius - 4);
      int x2 = cx + sin(angle) * radius;
      int y2 = cy - cos(angle) * radius;
      u8g2.drawLine(x1, y1, x2, y2);
    }

    // dB Descriptions
    for (const Mark& m : scaleMarks) 
    {
      float angle = radians(m.angle);
      int tx = cx + sin(angle) * (radius + 10);
      int ty = cy - cos(angle) * (radius + 10);
      u8g2.setCursor(tx - 8, ty + 5);
      u8g2.print(m.label);
    }

    // Channel Description (L/R)
    u8g2.setCursor(cx - 2, cy - 18);
    u8g2.print(label);
  };

  // Drawing the L and R indicators
  drawVUArc(centerXL, centerYL,"L"); //x,y,label (L)
  drawVUArc(centerXR, centerYR,"R"); //x,y,label (R)

  // Left needle
  float angleL = radians(arcStartDeg + (arcEndDeg - arcStartDeg) * displayVuL / 100.0);
  int xNeedleL = centerXL + sin(angleL) * (radius - 6);
  int yNeedleL = centerYL - cos(angleL) * (radius - 6);
  u8g2.drawLine(centerXL, centerYL - 8, xNeedleL, yNeedleL);

  // Right needle
  float angleR = radians(arcStartDeg + (arcEndDeg - arcStartDeg) * displayVuR / 100.0);
  int xNeedleR = centerXR + sin(angleR) * (radius - 6);
  int yNeedleR = centerYR - cos(angleR) * (radius - 6);
  u8g2.drawLine(centerXR, centerYR - 8, xNeedleR, yNeedleR);
}
*/

void vuMeterMode4() // Mode4 – Analog VU with rectangular scale
{
  // ===== VU reading =====
  uint8_t rawL = audio.getVUlevel() >> 8;
  uint8_t rawR = audio.getVUlevel() & 0xFF;

  int vuL = map(rawL, 0, 255, 0, 100);
  int vuR = map(rawR, 0, 255, 0, 100);

  if (vuL < 1) vuL = 1;
  if (vuR < 1) vuR = 1;
  
  // ===== Inertia =====
  if (vuSmooth) {
    if (vuL > displayVuL)
      displayVuL += max(1, (vuL - displayVuL) / vuRiseNeedleSpeed);
    else
      displayVuL -= max(1, (displayVuL - vuL) / vuFallNeedleSpeed);

    if (vuR > displayVuR)
      displayVuR += max(1, (vuR - displayVuR) / vuRiseNeedleSpeed);
    else
      displayVuR -= max(1, (displayVuR - vuR) / vuFallNeedleSpeed);

    displayVuL = constrain(displayVuL, 0, 100);
    displayVuR = constrain(displayVuR, 0, 100);
  } else {
    displayVuL = vuL;
    displayVuR = vuR;
  }

  // ===== Indicator parameters =====
  const int frameWidth = 120;
  const int frameHeight = 60;
  const int scaleWidth = 100;
  const int scaleYTopOffset = 23;
  const int majorTickHeight = 9;
  const int minorTickHeight = 5;
  const int baseLineThick = 2;

  int centerXL = 65;
  int centerXR = 195;
  int centerY  = 30;

  // ===== Cleaning =====
  u8g2.setDrawColor(0);
  u8g2.drawBox(0, 0, 256, 64);
  u8g2.setDrawColor(1);
  //u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.setFont(mono04b03b); 
  
  // ===== Filled indicator frames =====
  if (reversColorMode4) 
  {
    u8g2.drawRBox(centerXL - frameWidth / 2, centerY - frameHeight / 2, frameWidth, frameHeight,3); // Boxy z zaokragleniami
    u8g2.drawRBox(centerXR - frameWidth / 2, centerY - frameHeight / 2, frameWidth, frameHeight,3);
    u8g2.setDrawColor(0); // Rysujemy wszystko w negatywnie z białbym tlem VUmeterow
  }
  else
  {
    u8g2.drawRFrame(centerXL - frameWidth / 2, centerY - frameHeight / 2, frameWidth, frameHeight,3); // Ramki z zaokragleniami
    u8g2.drawRFrame(centerXR - frameWidth / 2, centerY - frameHeight / 2, frameWidth, frameHeight,3);
  }
  
  struct Label {
    uint8_t pos;       // 0–100
    const char* txt;
  };
  
  
  // ===== LARGE scale bar positions (0–100) =====
  const uint8_t majorPos[] = {
    0,   // -20
    18,  // -10
    35,  // -5
    50,  // środek
    62,  // 0 dB
    74,  // +3
    87,  // +6
    100  // +9
  };

  const uint8_t MAJOR_COUNT = sizeof(majorPos) / sizeof(majorPos[0]);


  const Label scaleLabels[] = {
    { 0,   "-20" },
    { 18,  "-10" },
    { 35,  "-5"  },
    { 62,  "0"   },
    { 72,  "+3"  },
    { 86,  "+6"  },
    { 100, "+9"  }
  };
  const uint8_t LABEL_COUNT = sizeof(scaleLabels) / sizeof(scaleLabels[0]);  
  
  auto drawLinearScale = [&](int centerX)
  {
    int leftX  = centerX - scaleWidth / 2;
    int railY1 = centerY - frameHeight / 2 + scaleYTopOffset;
    int railY2 = railY1 - 3;

    // === Helper: tick ===
    auto drawTick = [&](int x, int h)
    {
      int midX = leftX + scaleWidth / 2;
      int dx = 0;

      if (x < midX - 1)      dx = -h / 2;
      else if (x > midX + 1) dx =  h / 2;
      // else: środek = pionowo

      u8g2.drawLine(x, railY1, x + dx, railY1 - h);
    };

    // === Rails ===
    u8g2.drawLine(leftX, railY2+1, leftX + scaleWidth, railY2+1);
    u8g2.drawLine(leftX + 62 , railY2 + 2, leftX + scaleWidth, railY2+2);

    // --- Bottom line of the indicator ---
    u8g2.drawLine(leftX, railY1, leftX + scaleWidth, railY1);
    u8g2.drawLine(leftX, railY1 + 1, leftX + scaleWidth, railY1 + 1); // porgrubienie


    // === Major ticks ===
    for (uint8_t i = 0; i < MAJOR_COUNT; i++) {
      int x = leftX + (majorPos[i] * scaleWidth) / 100;
      drawTick(x, majorTickHeight);
    }
    
    // === Scale descriptions ===
    for (uint8_t i = 0; i < LABEL_COUNT; i++) {
      int x = leftX + (scaleLabels[i].pos * scaleWidth) / 100;
      u8g2.setCursor(x - 5, railY1 + 10);
      u8g2.print(scaleLabels[i].txt);
    }

    // === Minor ticks: 2 between EACH major pair ===
    for (uint8_t i = 0; i < MAJOR_COUNT - 1; i++)
    {
      //if (majorPos[i] == 50) continue;
      int x1 = leftX + (majorPos[i]     * scaleWidth) / 100;
      int x2 = leftX + (majorPos[i + 1] * scaleWidth) / 100;

      int step = (x2 - x1) / 3;
      drawTick(x1 + step,     minorTickHeight);
      drawTick(x1 + 2 * step, minorTickHeight);
    }
  };


  // ===== Indicator frames =====
  
  //u8g2.drawRBox(centerXL - frameWidth / 2, centerY - frameHeight / 2, frameWidth, frameHeight,3);
  //u8g2.drawRBox(centerXR - frameWidth / 2, centerY - frameHeight / 2, frameWidth, frameHeight,3);

  //u8g2.drawRFrame(centerXL - frameWidth / 2, centerY - frameHeight / 2, frameWidth, frameHeight,3);
  //u8g2.drawRFrame(centerXR - frameWidth / 2, centerY - frameHeight / 2, frameWidth, frameHeight,3);

  // ===== Drawing scales =====
  drawLinearScale(centerXL);
  drawLinearScale(centerXR);

  // ===== Channel description =====
  //u8g2.setCursor(centerXL - 3, centerY + 12); u8g2.print("dB");
  //u8g2.setCursor(centerXR - 3, centerY + 12); u8g2.print("dB");

  
  u8g2.setFont(u8g2_font_6x10_tr);
  //u8g2.setCursor(centerXL - 2, centerY - frameHeight/2 + 5);
  //u8g2.setCursor(centerXL - 2, centerY + 17); u8g2.print("L");
  //u8g2.setCursor(centerXL - 2, centerY + 18); u8g2.print("L");
  u8g2.setCursor(centerXL - 5, centerY + 18); u8g2.print("VU");
  //u8g2.setCursor(centerXR - 2, centerY - frameHeight/2 + 5);
  //u8g2.setCursor(centerXR - 2, centerY + 17); u8g2.print("R");
  u8g2.setCursor(centerXR - 5, centerY + 18); u8g2.print("VU");

  // ===== Needles =====
  const int radius = 45;
  //float angleL = radians(-60 + 120.0 * displayVuL / 100.0);
  //float angleR = radians(-60 + 120.0 * displayVuR / 100.0);
  float angleL = radians(-70 + 140.0 * displayVuL / 100.0);
  float angleR = radians(-70 + 140.0 * displayVuR / 100.0);

  int xNeedleL = centerXL + sin(angleL) * (radius - 6);
  int yNeedleL = centerY + 15 - cos(angleL) * (radius - 6);
  //int yNeedleL = centerY - cos(angleL) * (radius - 3);
  
  int xNeedleR = centerXR + sin(angleR) * (radius - 6);
  int yNeedleR = centerY + 15 - cos(angleR) * (radius - 6);
  //int yNeedleR = centerY - cos(angleR) * (radius - 3);

  
  u8g2.drawLine(centerXL, centerY + 24, xNeedleL, yNeedleL);
  u8g2.drawLine(centerXR, centerY + 24, xNeedleR, yNeedleR);
  u8g2.setDrawColor(1);

}


void vuMeterMode5() // Large VU bars covering the entire screen
{
  uint16_t raw = audio.getVUlevel();
  vuMeterL = (raw >> 8) & 0xFF;
  vuMeterR = raw & 0xFF;

  //vuMeterR = map(vuMeterR, 0, 255, 0, 240);
  //vuMeterL = map(vuMeterL, 0, 255, 0, 240);  // 244 VU + start od x=10 +  peak hold 2px

  if (vuMeterR < 1) vuMeterR = 0;
  if (vuMeterL < 1) vuMeterL = 0;
    
  //if (vuMeterR > 255) vuMeterR = 255;
  //if (vuMeterL > 255) vuMeterL = 255;


  if (vuSmooth)
  {
    // LEFT
    /*
    if (vuMeterL > displayVuL) 
    {
      displayVuL += vuRiseSpeed;
      if (displayVuL > vuMeterL) displayVuL = vuMeterL;
    }
    */
    if (vuMeterL > displayVuL) 
    {
      if (displayVuL > 255 - vuRiseSpeed) displayVuL = 255;
      else
      displayVuL += vuRiseSpeed;

      if (displayVuL > vuMeterL) displayVuL = vuMeterL;
    }




    else if (vuMeterL < displayVuL) 
    {
      if (displayVuL > vuFallSpeed) 
      {
        displayVuL -= vuFallSpeed;
      } 
      else 
      {
        displayVuL = 0;
      }
    }

    // RIGHT
    /*
    if (vuMeterR > displayVuR) 
    {
      displayVuR += vuRiseSpeed;
      if (displayVuR > vuMeterR) displayVuR = vuMeterR;
    }
    */
    if (vuMeterR > displayVuR) 
    {
      if (displayVuR > 255 - vuRiseSpeed) displayVuR = 255;
      else
      displayVuR += vuRiseSpeed;

      if (displayVuR > vuMeterR) displayVuR = vuMeterR;
    }


    else if (vuMeterR < displayVuR) 
    {
      if (displayVuR > vuFallSpeed) 
      {
        displayVuR -= vuFallSpeed;
      } 
      else 
      {
        displayVuR = 0;
      }
    }
  }

  // Left channel peak&hold update
  if (vuSmooth)
  {
    if (vuPeakHoldOn)
    {
      // --- Peak L ---
      if (displayVuL >= peakL) 
      {
        peakL = displayVuL;
        peakHoldTimeL = 0;
      } 
      else 
      {
        if (peakHoldTimeL < peakHoldThreshold) 
        {
          peakHoldTimeL++;
        } 
        else 
        {
          if (peakL > 0) peakL--;
        }
      }

      // --- Peak R ---
      if (displayVuR >= peakR) 
      {
        peakR = displayVuR;
        peakHoldTimeR = 0;
      } 
      else 
      {
        if (peakHoldTimeR < peakHoldThreshold) 
        {
          peakHoldTimeR++;
        } 
        else 
        {
          if (peakR > 0) peakR--;
        }
      }
    }    
  }

  if (volumeMute == false)  
  {
    if (vuSmooth)
    {
      u8g2.setDrawColor(0);
      u8g2.drawBox(0, 5, 257, 20);  //cleaning the screen under the VU meter
      u8g2.drawBox(0, 40, 257, 20);
      u8g2.setDrawColor(1);

      // Biale pola pod literami L i R
      //u8g2.drawBox(0, vuLy - 3, 7, 7);  
      //u8g2.drawBox(0, vuRy - 3, 7, 7);  

      // Rysujemy litery L i R
      //u8g2.setDrawColor(0);
      //u8g2.setFont(u8g2_font_04b_03_tr);
      u8g2.setFont(spleen6x12PL);
      u8g2.drawStr(0, 5 + 14, "L");
      u8g2.drawStr(0, 40 + 14, "R");
      u8g2.setDrawColor(1);  // We are bringing back white drawing

         
      for (uint8_t vusize = 0; vusize < displayVuL; vusize++)
      {       
        if ((vusize % 5) < 4) u8g2.drawBox(9 + vusize, 5, 1, 20);//u8g2.drawBox(9 + vusize, vuLy, 1, 2); // draw only 8 pixels, then 1px gap, 9 in the x axis is the space for the letter
      }
      
      for (uint8_t vusize = 0; vusize < displayVuR; vusize++)
      {
        if ((vusize % 5) < 4) u8g2.drawBox(9 + vusize, 40, 1, 20); // draw only 8 pixels, then 1px gap, 9 in the x axis is the space for the letter
      }    
      
      if (vuPeakHoldOn)
      {
        // Peak - dashes in dashed mode
        u8g2.drawBox(9 + peakL, 5, 1, 20);
        u8g2.drawBox(9 + peakR, 40, 1, 20);
      }
      
         
    }
  } 
}

void vuMeterMode7() // VU for 128x64 display, experiment with SSD1306 in displayMode5
{
  uint16_t raw = audio.getVUlevel();
  vuMeterL = (raw >> 8) & 0xFF;
  vuMeterR = raw & 0xFF;

  vuMeterR = map(vuMeterR, 0, 255, 0, 116);
  vuMeterL = map(vuMeterL, 0, 255, 0, 116);  // 244 VU + start from x=10 + peak hold 2px


  if (vuSmooth)
  {
    // LEFT
    if (vuMeterL > displayVuL) 
    {
      displayVuL += vuRiseSpeed;
      if (displayVuL > vuMeterL) displayVuL = vuMeterL;
    } 
    else if (vuMeterL < displayVuL) 
    {
      if (displayVuL > vuFallSpeed) 
      {
        displayVuL -= vuFallSpeed;
      } 
      else 
      {
        displayVuL = 0;
      }
    }

    // RIGHT
    if (vuMeterR > displayVuR) 
    {
      displayVuR += vuRiseSpeed;
      if (displayVuR > vuMeterR) displayVuR = vuMeterR;
    } 
    else if (vuMeterR < displayVuR) 
    {
      if (displayVuR > vuFallSpeed) 
      {
        displayVuR -= vuFallSpeed;
      } 
      else 
      {
        displayVuR = 0;
      }
    }
  }

  // Left channel peak&hold update
  if (vuSmooth)
  {
    if (vuPeakHoldOn)
    {
      // --- Peak L ---
      if (displayVuL >= peakL) 
      {
        peakL = displayVuL;
        peakHoldTimeL = 0;
      } 
      else 
      {
        if (peakHoldTimeL < peakHoldThreshold) 
        {
          peakHoldTimeL++;
        } 
        else 
        {
          if (peakL > 0) peakL--;
        }
      }

      // --- Peak R ---
      if (displayVuR >= peakR) 
      {
        peakR = displayVuR;
        peakHoldTimeR = 0;
      } 
      else 
      {
        if (peakHoldTimeR < peakHoldThreshold) 
        {
          peakHoldTimeR++;
        } 
        else 
        {
          if (peakR > 0) peakR--;
        }
      }
    }    
  }
  else
  {
    if (vuPeakHoldOn)
    {
      if (vuMeterL >= peakL) 
      {
        peakL = vuMeterL;
        peakHoldTimeL = 0;
      } 
      else 
      {
        if (peakHoldTimeL < peakHoldThreshold) 
        {
          peakHoldTimeL++;
        } 
        else 
        {
          if (peakL > 0) peakL--;
        }
      }
      // Peak&hold update for Right channel
      if (vuMeterR >= peakR) 
      {
        peakR = vuMeterR;
        peakHoldTimeR = 0;
      } 
      else 
      {
        if (peakHoldTimeR < peakHoldThreshold) 
        {
          peakHoldTimeR++;
        } 
        else 
        {
          if (peakR > 0) peakR--;
        }
      }
    }
  }


  if (volumeMute == false)  
  {
    if (vuSmooth)
    {
      u8g2.setDrawColor(0);
      u8g2.drawBox(7, vuLyMode5, 121, 3);  //cleaning the screen under the VU meter
      u8g2.drawBox(7, vuRyMode5, 121, 3);
      u8g2.setDrawColor(1);

      // Biale pola pod literami L i R
      u8g2.drawBox(0, vuLyMode5 - 3, 7, 7);  
      u8g2.drawBox(0, vuRyMode5 - 3, 7, 7);  

      // Rysujemy litery L i R
      u8g2.setDrawColor(0);
      u8g2.setFont(u8g2_font_04b_03_tr);
      u8g2.drawStr(2, vuLyMode5 + 3, "L");
      u8g2.drawStr(2, vuRyMode5 + 3, "R");
      u8g2.setDrawColor(1);  // We are bringing back white drawing

      if (vuMeterMode == 1)  // mode 1 continuous stripes
      {
        u8g2.setDrawColor(1);
        //u8g2.drawBox(10, vuLyMode5, vuMeterL, 2);  // we draw lines of a length corresponding to the VU value
        //u8g2.drawBox(10, vuRyMode5, vuMeterR, 2);

        u8g2.drawBox(10, vuLyMode5, displayVuL, 2);  // we draw lines of a length corresponding to the VU value
        u8g2.drawBox(10, vuRyMode5, displayVuR, 2);


        // Drawing peaks as a thin line
        u8g2.drawBox(9 + peakL, vuLyMode5, 1, 2);
        u8g2.drawBox(9 + peakR, vuRyMode5, 1, 2);
      } 
      else  // vuMeterMode == 0 basic mode, dashes with gaps
      {      
        for (uint8_t vusize = 0; vusize < displayVuL; vusize++)
        {
          if ((vusize % 9) < 8) u8g2.drawBox(9 + vusize, vuLyMode5, 1, 2);//u8g2.drawBox(9 + vusize, vuLyMode5, 1, 2); // draw only 8 pixels, then 1px gap, 9 in the x axis is the space for the letter
        }  
        for (uint8_t vusize = 0; vusize < displayVuR; vusize++)
        {
          if ((vusize % 9) < 8) u8g2.drawBox(9 + vusize, vuRyMode5, 1, 2); // draw only 8 pixels, then 1px gap, 9 in the x axis is the space for the letter
        }    
        
        if (vuPeakHoldOn)
        {
          // Peak - dashes in dashed mode
          u8g2.drawBox(9 + peakL, vuLyMode5, 1, 2);
          u8g2.drawBox(9 + peakR, vuRyMode5, 1, 2);
        }
      }
         
    }
    else
    {
      u8g2.setDrawColor(0);
      u8g2.drawBox(7, vuLyMode5, 121, 3);  //cleaning the screen under the VU meter
      u8g2.drawBox(7, vuRyMode5, 121, 3);
      u8g2.setDrawColor(1);

      // White fields under the letters L and R
      u8g2.drawBox(0, vuLyMode5 - 3, 7, 7);  
      u8g2.drawBox(0, vuRyMode5 - 3, 7, 7);  

      // We draw the letters L and R
      u8g2.setDrawColor(0);
      u8g2.setFont(u8g2_font_04b_03_tr);
      u8g2.drawStr(2, vuLyMode5 + 3, "L");
      u8g2.drawStr(2, vuRyMode5 + 3, "R");
      u8g2.setDrawColor(1);  // We are bringing back white drawing

      if (vuMeterMode == 1)  // mode 1 continuous stripes
      {
        u8g2.setDrawColor(1);
        u8g2.drawBox(10, vuLyMode5, vuMeterL, 2);  // we draw lines of a length corresponding to the VU value
        u8g2.drawBox(10, vuRyMode5, vuMeterR, 2);

        // Drawing peaks as a thin line
        u8g2.drawBox(9 + peakL, vuLyMode5, 1, 2);
        u8g2.drawBox(9 + peakR, vuRyMode5, 1, 2);
      } 
      else  // vuMeterMode == 0 basic mode, dashes with gaps
      {      
        for (uint8_t vusize = 0; vusize < vuMeterL; vusize++) 
        {
          if ((vusize % 9) < 8) u8g2.drawBox(9 + vusize, vuLyMode5, 1, 2);//u8g2.drawBox(9 + vusize, vuLyMode5, 1, 2); // draw only 8 pixels, then 1px gap, 9 in the x axis is the space for the letter
        }  
        for (uint8_t vusize = 0; vusize < vuMeterR; vusize++) 
        {
          if ((vusize % 9) < 8) u8g2.drawBox(9 + vusize, vuRyMode5, 1, 2); // draw only 8 pixels, then 1px gap, 9 in the x axis is the space for the letter
        } 
       
        if (vuPeakHoldOn)
        {
          // Peak - dashes in dashed mode
          u8g2.drawBox(9 + peakL, vuLyMode5, 1, 2);
          u8g2.drawBox(9 + peakR, vuRyMode5, 1, 2);
        }
      }  
    } // else smooth
  }  // moute off
}

void showIP(uint16_t xip, uint16_t yip)
{
  u8g2.setFont(spleen6x12PL);
  u8g2.setDrawColor(1);
  u8g2.setCursor(xip,yip);
  u8g2.print("IP: " + currentIP);
}

void displayClearUnderScroller() // Function responsible for scrolling stream title or stringstation information
{
  if (displayMode == 0) // Normal mode Mode 0- radio
  {
    u8g2.setDrawColor(1);
    u8g2.setFont(spleen6x12PL);
    u8g2.drawStr(0,yPositionDisplayScrollerMode0, "                                           "); //43 spaces - clearing the screen 
   } 
  else if (displayMode == 1)  // Tryb zegara - Mode 1
  {
    u8g2.drawStr(0,yPositionDisplayScrollerMode1, "                                           "); //43 characters screen clearing
  }
  else if (displayMode == 2)  // Tryb mały tekst - Mode 2
  {
    u8g2.setDrawColor(1);
    u8g2.setFont(spleen6x12PL);   
    u8g2.drawStr(0,yPositionDisplayScrollerMode2, "                                           "); //43 characters screen clearing
    u8g2.drawStr(0,yPositionDisplayScrollerMode2 + 12, "                                           "); //43 characters screen clearing
    u8g2.drawStr(0,yPositionDisplayScrollerMode2 + 12 + 12, "                                           "); //43 characters screen clearing
  }
  else if (displayMode == 3)
  {
    u8g2.setDrawColor(1);
    u8g2.setFont(spleen6x12PL);
    u8g2.drawStr(0,yPositionDisplayScrollerMode3, "                                           "); //43 spaces - clearing the screen  
  }
  else if (displayMode == 7) 
  {
    u8g2.setDrawColor(1);
    u8g2.setFont(spleen6x12PL);
    u8g2.drawStr(0,yPositionDisplayScrollerMode5, "                        "); //43 spaces - clearing the screen 
  }
  
  u8g2.sendBuffer();  // we draw the entire contents of the screen. 
}

void displayRadioScroller() // The function responsible for scrolling the stream title or stringstation information
{
  // If the length of the displayed stationString changes, clean the OLED screen in the Scroller areas.
  if (stationStringScroll.length() != stationStringScrollLength) 
  {
    //Serial.print("debug -- we clear the scroller area stationStringScrollLength - old value: ");
    //Serial.print(stationStringScrollLength);
    //Serial.print(" <-> new value: ");
    //Serial.println(stationStringScroll.length());
    
    stationStringScrollLength = stationStringScroll.length();
    displayClearUnderScroller();
  }

  if (displayMode == 0) // Tryb normalny - Mode 0, radio + VUmeter
  {
    if (stationStringScroll.length() > maxStationVisibleStringScrollLength) //42 + 4 separator space characters. In reality, we see 42 characters.
    {    
      xPositionStationString = offset;
      u8g2.setFont(spleen6x12PL);
      u8g2.setDrawColor(1);
      do {
        u8g2.drawStr(xPositionStationString, yPositionDisplayScrollerMode0, stationStringScroll.c_str());
        xPositionStationString = xPositionStationString + stationStringScrollWidth;
      } while (xPositionStationString < 256);
      
      offset = offset - 1;
      if (offset < (65535 - stationStringScrollWidth)) { 
        offset = 0;
      }

    } else {
      xPositionStationString = 0;
      u8g2.setDrawColor(1);
      u8g2.setFont(spleen6x12PL);    
      u8g2.drawStr(xPositionStationString, yPositionDisplayScrollerMode0, stationStringScroll.c_str());
      
    }
  } 
  else if (displayMode == 1)  // Tryb zegara
  {
    
    if (stationStringScroll.length() > maxStationVisibleStringScrollLength) 
    {     
      xPositionStationString = offset;
      u8g2.setFont(spleen6x12PL);
      u8g2.setDrawColor(1);
      do {
        u8g2.drawStr(xPositionStationString, yPositionDisplayScrollerMode1, stationStringScroll.c_str());

        xPositionStationString = xPositionStationString + stationStringScrollWidth;
      } while (xPositionStationString < 256);
      
      offset = offset - 1;
      if (offset < (65535 - stationStringScrollWidth)) {
        offset = 0;
      }

    } 
    else 
    {
      xPositionStationString = 0;
      u8g2.setDrawColor(1);
      u8g2.setFont(spleen6x12PL);       
      u8g2.drawStr(xPositionStationString, yPositionDisplayScrollerMode1, stationStringScroll.c_str());
    }

  }
  else if (displayMode == 2)  // Mode 2, Radio, 3 lines of station text
  {

    // Parameters for handling the display in 3 consecutive lines with division into full words
    const int maxLineLength = 41;  // Maximum length of one line in characters
    String currentLine = "";       // Current line
    int yPosition = yPositionDisplayScrollerMode2; // Initial position Y


    // Divide the text into words
    String word;
    int wordStart = 0;

    for (int i = 0; i <= stationStringScroll.length(); i++)
    {
      // Check if we have reached the end of a word or the end of the text
      if (i == stationStringScroll.length() || stationStringScroll.charAt(i) == ' ')
      {
        // Download the word
        String word = stationStringScroll.substring(wordStart, i);
        wordStart = i + 1;

        // Check that adding a word to the current line will not exceed maxLineLength
        if (currentLine.length() + word.length() <= maxLineLength)
        {
          // Dodaj słowo do bieżącej linii
          if (currentLine.length() > 0)
          {
            currentLine += " ";  // Add a space between words
          }
          currentLine += word;
        }
        else
        {
          // If the word doesn't match, display the current line and go to the next line
          u8g2.setFont(spleen6x12PL);
          u8g2.drawStr(0, yPosition, currentLine.c_str());
          yPosition += 12;  // Przesunięcie w dół dla kolejnej linii
          // Zresetuj bieżącą linię i dodaj nowe słowo
          currentLine = word;
        }
      }
    }
    // Display last line if anything is left
    if (currentLine.length() > 0)
    {
      u8g2.setFont(spleen6x12PL);
      u8g2.drawStr(0, yPosition, currentLine.c_str());
    }
  }
  else if (displayMode == 3)
  {
   if (stationStringScroll.length() > maxStationVisibleStringScrollLength) //42 + 4 znaki spacji separatora. Realnie widzimy 42 znaki
    {    
      xPositionStationString = offset;
      u8g2.setFont(spleen6x12PL);
      u8g2.setDrawColor(1);
      do {
        u8g2.drawStr(xPositionStationString, yPositionDisplayScrollerMode3, stationStringScroll.c_str());
        xPositionStationString = xPositionStationString + stationStringScrollWidth;
      } while (xPositionStationString < 256);
      
      offset = offset - 1;
      if (offset < (65535 - stationStringScrollWidth)) { 
        offset = 0;
      }
    } else {
      xPositionStationString = 0;
      //xPositionStationString = u8g2.getStrWidth(stationStringScroll.c_str());
      xPositionStationString = (SCREEN_WIDTH - stationStringScrollWidth) / 2;
           
      u8g2.setDrawColor(1);
      u8g2.setFont(spleen6x12PL);    
      u8g2.drawStr(xPositionStationString, yPositionDisplayScrollerMode3, stationStringScroll.c_str()); 
    } 
  }
  
  else if (displayMode == 7)
  {
   //if (stationStringScroll.length() > maxStationVisibleStringScrollLength) //42 + 4 space separator characters. In reality, we see 42 characters
   if (stationStringScroll.length() > 24) ///42 + 4 space separator characters. In reality, we see 42 characters
    {    
      xPositionStationString = offset;
      u8g2.setFont(spleen6x12PL);
      u8g2.setDrawColor(1);
      do {
        u8g2.drawStr(xPositionStationString, yPositionDisplayScrollerMode5, stationStringScroll.c_str());
        xPositionStationString = xPositionStationString + stationStringScrollWidth;
      } while (xPositionStationString < 128);
      
      offset = offset - 1;
      if (offset < (65535 - stationStringScrollWidth)) { 
        offset = 0;
      }
    } else {
      xPositionStationString = 0;
      //xPositionStationString = u8g2.getStrWidth(stationStringScroll.c_str());
      //xPositionStationString = (SCREEN_WIDTH - stationStringScrollWidth) / 2;
      
      xPositionStationString = (128 - stationStringScrollWidth) / 2;
           
      u8g2.setDrawColor(1);
      u8g2.setFont(spleen6x12PL);    
      u8g2.drawStr(xPositionStationString, yPositionDisplayScrollerMode5, stationStringScroll.c_str()); 
    } 
  }
  
}

void handleKeyboard()
{
  uint8_t key = 17;
  keyboardValue = 0;

  // --- Dummy read (for ESP32 ADC stability)
  analogRead(keyboardPin);
  delayMicroseconds(5);

  for (int i = 0; i < 32; i++)
  {
    keyboardValue = keyboardValue + analogRead(keyboardPin);
  }
  //keyboardValue = keyboardValue / 32;  
  keyboardValue >>= 5;


  if (debugKeyboard == 1) 
  { 
    displayActive = true;
    displayStartTime = millis();
    u8g2.clearBuffer();
    u8g2.setCursor(10,10); u8g2.print("ADC:   " + String(keyboardValue));
    u8g2.setCursor(10,23); u8g2.print("BUTTON:" + String(keyboardButtonPressed));
    u8g2.sendBuffer(); 
    Serial.print("debug - ADC reading: ");
    Serial.print(keyboardValue);
    Serial.print(" flag button pressed:");
    Serial.println(keyboardButtonPressed);
  }
  // We clear the pressed button flag only if nothing is pressed
  if (keyboardValue > keyboardButtonNeutral-keyboardButtonThresholdTolerance)
  {  
    keyboardButtonPressed = false;
  }
  // If nothing is pressed and nothing was pressed, we analyze the button
  if (keyboardValue < keyboardButtonNeutral-keyboardButtonThresholdTolerance)
  {  
    if (keyboardButtonPressed == false)
    {
      // We check the condition of the keyboard
      if ((keyboardValue <= keyboardButtonThreshold_1 + keyboardButtonThresholdTolerance ) && (keyboardValue >= keyboardButtonThreshold_1 - keyboardButtonThresholdTolerance ))
      { key=1;keyboardButtonPressed = true;}

      if ((keyboardValue <= keyboardButtonThreshold_2 + keyboardButtonThresholdTolerance ) && (keyboardValue >= keyboardButtonThreshold_2 - keyboardButtonThresholdTolerance ))
      { key=2;keyboardButtonPressed = true;}

      if ((keyboardValue <= keyboardButtonThreshold_3 + keyboardButtonThresholdTolerance ) && (keyboardValue >= keyboardButtonThreshold_3 - keyboardButtonThresholdTolerance ))
      { key=3;keyboardButtonPressed = true;}

      if ((keyboardValue <= keyboardButtonThreshold_4 + keyboardButtonThresholdTolerance ) && (keyboardValue >= keyboardButtonThreshold_4 - keyboardButtonThresholdTolerance ))
      { key=4;keyboardButtonPressed = true;}

      if ((keyboardValue <= keyboardButtonThreshold_5 + keyboardButtonThresholdTolerance ) && (keyboardValue >= keyboardButtonThreshold_5 - keyboardButtonThresholdTolerance ))
      { key=5;keyboardButtonPressed = true;}

      if ((keyboardValue <= keyboardButtonThreshold_6 + keyboardButtonThresholdTolerance ) && (keyboardValue >= keyboardButtonThreshold_6 - keyboardButtonThresholdTolerance ))
      { key=6;keyboardButtonPressed = true;}

      if ((keyboardValue <= keyboardButtonThreshold_7 + keyboardButtonThresholdTolerance ) && (keyboardValue >= keyboardButtonThreshold_7 - keyboardButtonThresholdTolerance ))
      { key=7;keyboardButtonPressed = true;}

      if ((keyboardValue <= keyboardButtonThreshold_8 + keyboardButtonThresholdTolerance ) && (keyboardValue >= keyboardButtonThreshold_8 - keyboardButtonThresholdTolerance ))
      { key=8;keyboardButtonPressed = true;}

      if ((keyboardValue <= keyboardButtonThreshold_9 + keyboardButtonThresholdTolerance ) && (keyboardValue >= keyboardButtonThreshold_9 - keyboardButtonThresholdTolerance ))
      { key=9;keyboardButtonPressed = true;}

      if ((keyboardValue <= keyboardButtonThreshold_0 + keyboardButtonThresholdTolerance ) && (keyboardValue >= keyboardButtonThreshold_0 - keyboardButtonThresholdTolerance ))
      { key=0;keyboardButtonPressed = true;}

      if ((keyboardValue <= keyboardButtonThreshold_BankMenu + keyboardButtonThresholdTolerance ) && (keyboardValue >= keyboardButtonThreshold_BankMenu - keyboardButtonThresholdTolerance ))
      { key=11;keyboardButtonPressed = true;}

      if ((keyboardValue <= keyboardButtonThreshold_Ok + keyboardButtonThresholdTolerance ) && (keyboardValue >= keyboardButtonThreshold_Ok - keyboardButtonThresholdTolerance ))
      { key=12;keyboardButtonPressed = true;}

      if ((keyboardValue <= keyboardButtonThreshold_DisplayMode + keyboardButtonThresholdTolerance ) && (keyboardValue >= keyboardButtonThreshold_DisplayMode - keyboardButtonThresholdTolerance ))
      { key=13;keyboardButtonPressed = true;}

      if ((keyboardValue <= keyboardButtonThreshold_Back + keyboardButtonThresholdTolerance ) && (keyboardValue >= keyboardButtonThreshold_Back - keyboardButtonThresholdTolerance ))
      { key=14;keyboardButtonPressed = true;}

      if ((keyboardValue <= keyboardButtonThreshold_Mute + keyboardButtonThresholdTolerance ) && (keyboardValue >= keyboardButtonThreshold_Mute - keyboardButtonThresholdTolerance ))
      { key=15;keyboardButtonPressed = true;}

      if ((keyboardValue <= keyboardButtonThreshold_Dimmer + keyboardButtonThresholdTolerance ) && (keyboardValue >= keyboardButtonThreshold_Dimmer - keyboardButtonThresholdTolerance ))
      { key=16;keyboardButtonPressed = true;}

      // Strzalki nawigacyjne
      if ((keyboardValue <= keyboardButtonThreshold_ArrowLeft + keyboardButtonThresholdTolerance ) && (keyboardValue >= keyboardButtonThreshold_ArrowLeft - keyboardButtonThresholdTolerance ))
      { key=17;keyboardButtonPressed = true;}

      if ((keyboardValue <= keyboardButtonThreshold_ArrowRight + keyboardButtonThresholdTolerance ) && (keyboardValue >= keyboardButtonThreshold_ArrowRight - keyboardButtonThresholdTolerance ))
      { key=18;keyboardButtonPressed = true;}

      if ((keyboardValue <= keyboardButtonThreshold_ArrowUp + keyboardButtonThresholdTolerance ) && (keyboardValue >= keyboardButtonThreshold_ArrowUp - keyboardButtonThresholdTolerance ))
      { key=19;keyboardButtonPressed = true;}

      if ((keyboardValue <= keyboardButtonThreshold_ArrowDown + keyboardButtonThresholdTolerance ) && (keyboardValue >= keyboardButtonThreshold_ArrowDown - keyboardButtonThresholdTolerance ))
      { key=20;keyboardButtonPressed = true;}

      if ((keyboardValue <= keyboardButtonThreshold_PresetsOn + keyboardButtonThresholdTolerance ) && (keyboardValue >= keyboardButtonThreshold_PresetsOn - keyboardButtonThresholdTolerance ))
      { key=21;keyboardButtonPressed = true;}

      

      if ((key < 22) && (keyboardButtonPressed == true))
      {
        Serial.print("ADC reading correct: ");
        Serial.print(keyboardValue);
        Serial.print(" button: ");
        Serial.println(key);
        keyboardValue = 0;
     
        if ((debugKeyboard == false) && (key < 10)) // For buttons 0-9 we perform actions as on the remote control
        { 
          if (f_Presets && !bankMenuEnable) {setPreset(key); return;} else {rcInputKey(key);}      
          //rcInputKey(key);
        }
        else if ((debugKeyboard == false) && (key == 11)) {bankMenuDisplay();} // The Memory button brings up the Bank selection menu.
        else if ((debugKeyboard == false) && (key == 12)) // The Shift button confirms the change of station or bank, it works like "ON" on the remote control.
        { 
         ir_code = rcCmdOk; // Auto Tuning button pretends to be OK
          bit_count = 32;
          calcNec();  // we convert the remote control code to the original full NEC code
        }
        else if ((debugKeyboard == false) && (key == 13)) // Auto button - switches between Clock/Radio mode
        { 
          ir_code = rcCmdSrc; // Auto Tuning button pretends to be Srceen
          bit_count = 32;
          calcNec();  // we convert the remote control code to the original full NEC code
        }
        else if ((debugKeyboard == false) && (key == 14)) // Band button pretends to be Back
        {  
          ir_code = rcCmdBack; // We pretend to give pilot commands
          bit_count = 32;
          calcNec();  // we convert the remote control code to the original full NEC code
        }
        else if ((debugKeyboard == false) && (key == 15)) // Przycisk Mute
        { 
          ir_code = rcCmdMute; // We assign the command code from the remote control
          bit_count = 32; // we set the information that we have the full NEC code for analysis 
          calcNec();  // we convert the remote control code to the original full NEC code     
        }
        else if ((debugKeyboard == false) && (key == 16)) // Memory Scan button - changing the display brightness - "Dimmer" function
        { 
          ir_code = rcCmdDirect; // We pretend to give pilot commands
          bit_count = 32;
          calcNec();  // we convert the remote control code to the original full NEC code
        }
        else if ((debugKeyboard == false) && (key == 17)) {ir_code = rcCmdArrowLeft; bit_count = 32; calcNec();}
        else if ((debugKeyboard == false) && (key == 18)) {ir_code = rcCmdArrowRight; bit_count = 32; calcNec();}
        else if ((debugKeyboard == false) && (key == 19)) {ir_code = rcCmdArrowUp; bit_count = 32; calcNec();}
        else if ((debugKeyboard == false) && (key == 20)) {ir_code = rcCmdArrowDown; bit_count = 32; calcNec();}
        else if ((debugKeyboard == false) && (key == 21)) {ir_code = rcCmdBankPlus; bit_count = 32; calcNec();}  
        
        key=22; // Reset "KEY" to a position outside of keyboard range
      }
    }
  } 
  
}

// ---- DISPLAY VOLUME (no value change) ----
void volumeDisplay()
{
  displayStartTime = millis();
  timeDisplay = false;
  displayActive = true;
  volumeSet = true;
  //volumeMute = false;  

  Serial.print("debug volumeDisplay -> Volume value: ");
  Serial.println(volumeValue);
  //audio.setVolume(volumeValue);  // zakres 0...21
  String volumeValueStr = String(volumeValue);  // Converting a VOLUME number to a string
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_fub14_tf);
  u8g2.drawStr(65, 33, "VOLUME");
  u8g2.drawStr(163, 33, volumeValueStr.c_str());

  u8g2.drawRFrame(21, 42, 214, 14, 3);                                                   // Let's draw a frame for the volume progress bar
  if (maxVolume == 42 && volumeValue > 0) { u8g2.drawRBox(23, 44, volumeValue * 5, 10, 2);}  // Progress bar volume
  if (maxVolume == 21 && volumeValue > 0) { u8g2.drawRBox(23, 44, volumeValue * 10, 10, 2);} // Progress bar volume
  u8g2.sendBuffer();
  
  //wsVolumeChange(volumeValue); // Send an update via WebSocket to a website
  wsVolumeChange(); // Send an update via WebSocket to a website
  }


// ---- LOUDER +1 ----
void volumeUp()
{
  volumeSet = true;
  timeDisplay = false;
  displayActive = true;
  volumeMute = false;
  displayStartTime = millis();   
  volumeValue++;

  if (volumeValue > maxVolume) {volumeValue = maxVolume;}
  audio.setVolume(volumeValue);  // zakres 0...21 lub 0...42
  volumeDisplay();
}

// ---- QUIET -1 ----
void volumeDown()
{
  volumeSet = true;
  timeDisplay = false;
  displayActive = true;
  displayStartTime = millis();
  volumeMute = false;
 
  volumeValue--;
  if (volumeValue < 1) {volumeValue = 1;}
  audio.setVolume(volumeValue);  // range 0...21 or 0...42
  volumeDisplay();
}

// ---- DELETE ALL FLAGS ---- 
//Clears all flags for menus, functions, etc. Allows you to return to the main radio screen.
void clearFlags()  
{
  timeDisplay = true;

  debugKeyboard = false;
  displayActive = false;
  listedStations = false;
  volumeSet = false;
  bankMenuEnable = false;
  bankNetworkUpdate = false;
  equalizerMenuEnable = false;
  rcInputDigitsMenuEnable = false;
  f_voiceTimeBlocked = false;
  if (f_displaySleepTime && f_sleepTimerOn) {f_displaySleepTimeSet = true;}
  f_displaySleepTime = false;

  rcInputDigit1 = 0xFF; // we clear the number 1, the empty variable flag is FF
  rcInputDigit2 = 0xFF; //we clear the number 2, the empty variable flag is FF
  station_nr = stationFromBuffer; // We restore the station number from the buffer
  bank_nr = previous_bank_nr;     // We restore the bank number from the buffer
}


void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) 
{
  if (type == WS_EVT_CONNECT) 
  {
    Serial.println("debug Web -> WebSocket client connected");

    client->text("station:" + String(station_nr));
    client->text("stationname:" + stationName.substring(0, stationNameLenghtCut));
    client->text("volume:" + String(volumeValue)); 
    client->text("mute:" + String(volumeMute));
    client->text("bank:" + String(bank_nr));

    if (audio.isRunning() == true)
    {
     sanitizeUtf8(stationStringWeb);
     client->text("stationtext$" + stationStringWeb);
    }
    else
    {
     client->text("stationtext$... No audio stream ! ...");
    }
    
    client->text("bitrate:" + String(bitrateString)); 
    client->text("samplerate:" + String(sampleRateString)); 
    client->text("bitpersample:" + String(bitsPerSampleString)); 

  
  if (mp3 == true) {client->text("streamformat:MP3"); }
  if (flac == true) {client->text("streamformat:FLAC"); }
  if (aac == true) {client->text("streamformat:AAC"); }
  if (vorbis == true) {client->text("streamformat:VORBIS"); }
  if (opus == true) {client->text("streamformat:OPUS"); }

  client->text("wifi:" + String(wifiSignalLevel)); // sends the Wi-Fi signal value to all connected clients

  } 
  else if (type == WS_EVT_DATA) 
  {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 && info->len == len) 
    {
      String msg = "";
      for (size_t i = 0; i < len; i++) 
      {
        msg += (char) data[i];
      }

      Serial.println("debug Web -> WS received: " + msg);

      if (msg.startsWith("volume:")) 
      {
        int newVolume = msg.substring(7).toInt();
        volumeValue = newVolume;
        volumeMute = false;
        audio.setVolume(volumeValue);  // zakres 0...21 lub 0...42
        volumeDisplay();   // display the Volume value on the OLED display
      }
      else if (msg.startsWith("station:")) 
      {
        int newStation = msg.substring(8).toInt();
        station_nr = newStation;
        bank_nr = previous_bank_nr;
        urlPlaying = false;

        ir_code = rcCmdOk; // We assign the command code from the remote control
        bit_count = 32; // we set the information that we have the full NEC code for analysis
        calcNec();  // we convert the remote control code to the original full NEC code    
      }
      else if (msg.startsWith("bank:")) 
      {
        int newBank = msg.substring(5).toInt();
        bank_nr = newBank;
        
        //bankMenuEnable = true;        
        //displayStartTime = millis();
        //timeDisplay = false;
        
        bankMenuDisplay();
        fetchStationsFromServer();
        clearFlags();
        urlPlaying = false;

        station_nr = 1;
        
        //Approval of station change
        ir_code = rcCmdOk; // We assign the command code from the remote control
        bit_count = 32; // we set the information that we have the full NEC code for analysis
        calcNec();  // we convert the remote control code to the original full NEC code    
        
      }
      else if (msg.startsWith("displayMode")) 
      {
        ir_code = rcCmdSrc; // We simulate the SRC remote control code - changing the display mode 
        bit_count = 32;
        calcNec();          // We convert the remote control code into the full original NEC code
      }

    }
  }

}

void bufforAudioInfo()
{
  int bitRateBps = audio.getBitRate() / 1000;
  int bytesPerSecond = 0;

  if (bitRateBps > 0) { bytesPerSecond = bitRateBps / 8;}   // kB/s

  int inputBufferBytes = audio.inBufferFilled() / 1000;

  // protection against division by zero and other errors
  if (bytesPerSecond > 0 && inputBufferBytes > 8) 
  {
    audioBufferTime = inputBufferBytes / bytesPerSecond;
  } 
  else 
  {
    audioBufferTime = 0;
  }

  
  
  Serial.print("debug-- Audio Buffer Free: ");
  Serial.print(audio.inBufferFree());
  Serial.print(" / ");
  Serial.print("occupied: ");
  Serial.print(audio.inBufferFilled());
  
  Serial.print("  Music in the buffer on: " + String(audioBufferTime) + " sek. " );
  Serial.print("  bitrate: " );
  Serial.print(audio.getBitRate());
  Serial.print("  " );
  /*
  if (audioBufferTime >= 10) {Serial.println("##########");}
  if (audioBufferTime >= 9) {Serial.println("#########_");}
  if (audioBufferTime >= 8) {Serial.println("########__");}
  if (audioBufferTime >= 7) {Serial.println("#######___");}
  if (audioBufferTime >= 6) {Serial.println("######____");}
  if (audioBufferTime >= 5) {Serial.println("#####_____");}
  if (audioBufferTime == 4) {Serial.println("####______");}
  if (audioBufferTime == 3) {Serial.println("###_______");}
  if (audioBufferTime == 2) {Serial.println("##________");}
  if (audioBufferTime == 1) {Serial.println("#_________");}
  if (audioBufferTime == 0) {Serial.println("__________");}
  */
  
  if (audioBufferTime < 0)  audioBufferTime = 0;
  if (audioBufferTime > 10) audioBufferTime = 10;

  /*
  char bar[11];
  for (int i = 0; i < 10; i++) {bar[i] = (i < audioBufferTime) ? '#' : '_'; }
  bar[10] = '\0';
  Serial.println(bar);
  */
  
  Serial.print("[");
  for (int i = 0; i < 10; i++) {if (i < audioBufferTime) Serial.print('#'); else Serial.print('_'); }
  Serial.println("]");

  Serial.print("Free internal RAM: ");
  Serial.println(heap_caps_get_free_size(MALLOC_CAP_INTERNAL));

  Serial.print("Free PSRAM: ");
  Serial.println(heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
  
  Serial.print("Free heap: ");
  Serial.println(ESP.getFreeHeap());

  Serial.print("Total heap: ");
  Serial.println(ESP.getHeapSize());

  Serial.print("Min free heap (od startu): ");
  Serial.println(ESP.getMinFreeHeap());

  Serial.print("Free stack (words): ");
  Serial.println(uxTaskGetStackHighWaterMark(NULL));

  Serial.print("Loop task free stack (bytes): ");
  Serial.println(uxTaskGetStackHighWaterMark(NULL) * 4);

}

// Interrupt handling function (reaction to a pin state change)
void IRAM_ATTR pulseISR()
{
  if (digitalRead(recv_pin) == HIGH)
  {
    pulse_start_high = micros();  // Recording the beginning of the pulse
  }
  else
  {
    pulse_end_high = micros();    // End of pulse recording
    pulse_ready = true;
  }

  if (digitalRead(recv_pin) == LOW)
  {
    pulse_start_low = micros();  // Recording the beginning of the pulse
  }
  else
  {
    pulse_end_low = micros();    // End of pulse recording
    pulse_ready_low = true;
  }

   // ----------- PULSE ANALYSIS -----------------------------
  if (pulse_ready_low) // we check if the state is low for 9ms - start of the frame
  {
    pulse_duration_low = pulse_end_low - pulse_start_low;
  
    if (pulse_duration_low > (LEAD_HIGH - TOLERANCE) && pulse_duration_low < (LEAD_HIGH + TOLERANCE))
    {
      pulse_duration_9ms = pulse_duration_low; // assign the Low pulse duration to the variable pulse 9ms
      pulse_ready9ms = true; // 9ms pulse detection correct flag within tolerance
    }

  }

  // Checking if the pulse is ready for analysis
  if ((pulse_ready== true) && (pulse_ready9ms = true))
  {
    pulse_ready = false;
    pulse_ready9ms = false; // kasujemy flage wykrycia pulsu 9ms

    // Calculation of pulse duration
    pulse_duration = pulse_end_high - pulse_start_high;
    //Serial.println(pulse_duration); reading the pulse length from the remote control - debug
    if (!data_start_detected)
    {
    
      // Waiting for signal 4.5 ms high
      if (pulse_duration > (LEAD_LOW - TOLERANCE) && pulse_duration < (LEAD_LOW + TOLERANCE))
      {
        pulse_duration_4_5ms = pulse_duration;
        // Signal start: 4.5 ms high
        
        data_start_detected = true;  // Setting a flag when a pre-signal is detected
        bit_count = 0;               // Reset bit_count before receiving data
        ir_code = 0;                 // Reset IR code before receiving data
      }
    }
    else
    {
      // Signals for bytes (address ADDR, IADDR, command CMD, ICMD) start after the initial signal
      if (pulse_duration > (HIGH_THRESHOLD - TOLERANCE) && pulse_duration < (HIGH_THRESHOLD + TOLERANCE))
      {
        ir_code = (ir_code << 1) | 1;  // Dodanie "1" do kodu IR
        //bit_count++;
        bit_count = bit_count + 1;
        pulse_duration_1690us = pulse_duration;
       

      }
      else if (pulse_duration > (LOW_THRESHOLD - TOLERANCE) && pulse_duration < (LOW_THRESHOLD + TOLERANCE))
      {
        ir_code = (ir_code << 1) | 0;  // Adding "0" to the IR code
        //bit_count++;
        bit_count = bit_count + 1;
        pulse_duration_560us = pulse_duration;
       
      }

      // Checking if the full 32-bit IR code was received
      if (bit_count == 32)
      {
        // IR LED activity simulation
        //#ifdef IR_LED
          //digitalWrite(IR_LED, HIGH);
        //#endif
        
        // Rozbicie kodu na 4 bajty
        uint8_t ADDR = (ir_code >> 24) & 0xFF;  // First byte
        uint8_t IADDR = (ir_code >> 16) & 0xFF; // Second byte (address inversion)
        uint8_t CMD = (ir_code >> 8) & 0xFF;    // Third byte (command)
        uint8_t ICMD = ir_code & 0xFF;          // Fourth byte (command inversion)

        // Schecking the correctness (inversion) of the address and command bytes
        if ((ADDR ^ IADDR) == 0xFF && (CMD ^ ICMD) == 0xFF)
        {
          data_start_detected = false;
          //bit_count = 0;
        }
        else
        {
          ir_code = 0; 
          data_start_detected = false;
          //bit_count = 0;        
        }

      }
		  
    }
  }
 //runTime2 = esp_timer_get_time();
}

void displayEqualizer() // 3-point equalizer menu drawing function
{

  displayStartTime = millis();  // We are updating the time for the auto-return function from the menu
  equalizerMenuEnable = true;   // We set the equalizer menu flag
  timeDisplay = false;          // We turn off the clock
  displayActive = true;         // Active display
  
  Serial.println("--Equalizer--");
  Serial.print("Low Tone Value:   ");
  Serial.println(toneLowValue);
  Serial.print("Mid Tone Value:  ");
  Serial.println(toneMidValue);
  Serial.print("High tone value: ");
  Serial.println(toneHiValue);
  
  Serial.print("The Value of Balance: ");
  Serial.println(balanceValue);

  audio.setTone(toneLowValue, toneMidValue, toneHiValue); // Adjustment range -40 + 6dB as signed int8_t
  audio.setBalance(balanceValue);

  u8g2.setDrawColor(1);
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_fub14_tf);
  u8g2.drawStr(60, 14, "EQUALIZER");
  //u8g2.drawStr(1, 14, "EQUALIZER");
  u8g2.setFont(spleen6x12PL);
  //u8g2.setCursor(138,12);
  //u8g2.print("P1    P2    P3    P4");
  uint8_t xTone;
  uint8_t yTone;  
  
  // ---- High Tones ----
  xTone=0; 
  yTone=28;
  u8g2.setCursor(xTone,yTone);
  if (toneHiValue >= 0) {u8g2.print("High:  " + String(toneHiValue) + "dB");}  // Dla wartosci dodatnich piszemy normalnie
  if ((toneHiValue < 0) && (toneHiValue >= -9)) {u8g2.print("High: " + String(toneHiValue) + "dB");}    // Dla wartosci ujemnych piszemy o 1 znak wczesniej
  if ((toneHiValue < 0) && (toneHiValue < -9)) {u8g2.print("High:" + String(toneHiValue) + "dB");}    // Dla wartosci ujemnych ponizej -9 piszemy o 2 znak wczesniej
  
  xTone=20;
  if (toneSelect == 1) 
  { 
    u8g2.drawRBox(xTone+56,yTone-9,154,9,1); // Rysujemy biały pasek pod regulacją danego tonu  
    u8g2.setDrawColor(0); 
    u8g2.drawLine(xTone+57,yTone-5,xTone+208,yTone-5);
    if (toneHiValue >= 0){ u8g2.drawRBox((6 * toneHiValue) + xTone + 128,yTone-7,10,5,1);}
    if (toneHiValue < 0) { u8g2.drawRBox((6 * toneHiValue) + xTone + 128,yTone-7,10,5,1);}
    u8g2.setDrawColor(1);
  }
  else 
  {
    u8g2.drawLine(xTone+57,yTone-5,xTone+208,yTone-5);
    if (toneHiValue >= 0){ u8g2.drawRBox((6 * toneHiValue) + xTone + 128,yTone-7,10,5,1);}
    if (toneHiValue < 0) { u8g2.drawRBox((6 * toneHiValue) + xTone + 128,yTone-7,10,5,1);}
  }

  // ---- Midrange tones ----
  xTone=0;
  yTone=40;
  u8g2.setCursor(xTone,yTone);
  if (toneMidValue >= 0) {u8g2.print("Mid:   " + String(toneMidValue) + "dB");}  // Dla wartosci dodatnich piszemy normalnie
  if ((toneMidValue < 0) && (toneMidValue >= -9)) {u8g2.print("Mid:  " + String(toneMidValue) + "dB");}
  if ((toneMidValue < 0) && (toneMidValue < -9))  {u8g2.print("Mid: " + String(toneMidValue) + "dB");} 
  
  xTone=20;
  if (toneSelect == 2) 
  { 
    u8g2.drawRBox(xTone+56,yTone-9,154,9,1);
    u8g2.setDrawColor(0);
    u8g2.drawLine(xTone+57,yTone-5,xTone + 208,yTone-5);
    //u8g2.drawLine(xTone+57,yTone-4,xTone + 208,yTone-4);
    if (toneMidValue >= 0) { u8g2.drawRBox((6 * toneMidValue) + xTone + 128,yTone-7,10,5,1);}
    if (toneMidValue < 0)  { u8g2.drawRBox((6 * toneMidValue) + xTone + 128,yTone-7,10,5,1);}
    u8g2.setDrawColor(1);
  }
  else
  {
    u8g2.drawLine(xTone+57,yTone-5,xTone + 208,yTone-5);
    //u8g2.drawLine(xTone+57,yTone-4,xTone + 208,yTone-4);
    if (toneMidValue >= 0) { u8g2.drawRBox((6 * toneMidValue) + xTone + 128,yTone-7,10,5,1);}
    if (toneMidValue < 0)  { u8g2.drawRBox((6 * toneMidValue) + xTone + 128,yTone-7,10,5,1);}
    //u8g2.drawRBox((3 * toneMidValue) + xTone + 138,yTone-7,10,6,1);
  }


  // Low tones
  xTone=0; 
  yTone=52;
  u8g2.setCursor(xTone,yTone);
  if (toneLowValue >= 0) { u8g2.print("Low:   " + String(toneLowValue) + "dB");}
  if ((toneLowValue < 0) && (toneLowValue >= -9)) { u8g2.print("Low:  " + String(toneLowValue) + "dB");}
  if ((toneLowValue < 0) && (toneLowValue < -9)) { u8g2.print("Low: " + String(toneLowValue) + "dB");}
  xTone=20;
  if (toneSelect == 3) 
  { 
    u8g2.drawRBox(xTone+56,yTone-9,154,9,1);
    u8g2.setDrawColor(0);
    u8g2.drawLine(xTone + 57,yTone-5,xTone + 208,yTone-5);
    if ( toneLowValue >= 0 ) { u8g2.drawRBox((6 * toneLowValue) + xTone + 128,yTone-7,10,5,1);}
    if ( toneLowValue < 0 )  { u8g2.drawRBox((6 * toneLowValue) + xTone + 128,yTone-7,10,5,1);}
    u8g2.setDrawColor(1);
  }
  else
  {
    u8g2.drawLine(xTone + 57,yTone-5,xTone + 208,yTone-5);
    if ( toneLowValue >= 0 ) { u8g2.drawRBox((6 * toneLowValue) + xTone + 128,yTone-7,10,5,1);}
    if ( toneLowValue < 0 )  { u8g2.drawRBox((6 * toneLowValue) + xTone + 128,yTone-7,10,5,1);}
  }

// Balans
  xTone=0; 
  yTone=64;
  u8g2.setCursor(xTone,yTone);
  if (balanceValue >= 0) { u8g2.print("Bal:   " + String(balanceValue) + "dB");}
  if ((balanceValue < 0) && (balanceValue >= -9)) { u8g2.print("Bal:  " + String(balanceValue) + "dB");}
  if ((balanceValue < 0) && (balanceValue < -9)) { u8g2.print("Bal: " + String(balanceValue) + "dB");}
  xTone=20;
  if (toneSelect == 4) 
  { 
    u8g2.drawRBox(xTone+56,yTone-9,154,9,1);
    u8g2.setDrawColor(0);
    u8g2.drawLine(xTone + 64,yTone-5,xTone + 201,yTone-5);
    u8g2.setFont(u8g2_font_04b_03_tr);
    u8g2.drawStr(xTone + 58, yTone-2, "L");
    u8g2.drawStr(xTone + 204, yTone-2, "R");
    if ( balanceValue >= 0 ) { u8g2.drawRBox((4 * balanceValue) + xTone + 128,yTone-7,10,5,1);}
    if ( balanceValue < 0 )  { u8g2.drawRBox((4 * balanceValue) + xTone + 128,yTone-7,10,5,1);}
    u8g2.setDrawColor(1);
  }
  else
  {
    u8g2.drawLine(xTone + 64,yTone-5,xTone + 201,yTone-5);
    u8g2.setFont(u8g2_font_04b_03_tr);
    u8g2.drawStr(xTone + 58, yTone -2, "L");
    u8g2.drawStr(xTone + 204, yTone-2, "R");
    if ( balanceValue >= 0 ) { u8g2.drawRBox((4 * balanceValue) + xTone + 128,yTone-7,10,5,1);}
    if ( balanceValue < 0 )  { u8g2.drawRBox((4 * balanceValue) + xTone + 128,yTone-7,10,5,1);}

  }

  u8g2.sendBuffer(); 
}


void displayInfo()
{
  displayStartTime = millis();  // We are updating the time for the auto-return function from the menu
  timeDisplay = false;          // We turn off the clock
  displayActive = true;         // Active display
  
  uint64_t chipid = ESP.getEfuseMac();

  char buf[100];
  snprintf(buf, sizeof(buf),
          "ESP32 SN:%04X%08X,  FW Ver.: %s",
          (uint16_t)(chipid >> 32),
          (uint32_t)chipid,
          softwareRev);

String chipSN = buf;
  u8g2.clearBuffer();
  u8g2.setFont(spleen6x12PL);
  u8g2.drawStr(0, 10, "Info:");
  //u8g2.setCursor(0,25); u8g2.print("ESP32 SN:" + String(ESP.getEfuseMac()) + ",  FW Ver.:" + String(softwareRev));
  u8g2.setCursor(0,25); u8g2.print(chipSN);
  u8g2.setCursor(0,38); u8g2.print("Hostname:" + String(hostname) + ",  WiFi Signal:" + String(WiFi.RSSI()) + "dBm");
  u8g2.setCursor(0,51); u8g2.print("WiFi SSID:" + String(wifiManager.getWiFiSSID()));
  u8g2.setCursor(0,64); u8g2.print("IP:" + currentIP + "  MAC:" + String(WiFi.macAddress()) );
  
  u8g2.sendBuffer();
}
/*
void audioProcessing(void *p)
{
  while (true) {
  audio.loop();
  vTaskDelay(1 / portTICK_PERIOD_MS); // 1 millisecond delay
  }
}
*/


//  OTA update callback for Wifi Manager and Recovery Mode
void handlePreOtaUpdateCallback()
{
  Update.onProgress([](unsigned int progress, unsigned int total) 
  {
    u8g2.setCursor(1,56); u8g2.printf("Update: %u%%\r", (progress / (total / 100)) );
    u8g2.setCursor(80,56); u8g2.print(String(progress) + "/" + String(total));
    u8g2.sendBuffer();
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
}

void recoveryModeCheck()
{
  unsigned long lastTurnTime = 0;

  if (digitalRead(SW_PIN2) == 0)
  {
    int8_t recoveryMode = 0;
    u8g2.clearBuffer();
    u8g2.setFont(spleen6x12PL);
    u8g2.drawStr(1,14, "RECOVERY / RESET MODE - release encoder");
    u8g2.sendBuffer();
    delay(2000);

    while (digitalRead(SW_PIN2) == 0) {delay(10);}  
    u8g2.drawStr(1,14, "Please Wait...                         ");
    u8g2.sendBuffer();
    delay(1000);
    u8g2.clearBuffer();
     
    prev_CLK_state2 = digitalRead(CLK_PIN2);
    
    while (true)
    {
      if (millis() - lastTurnTime > 50)
      {  
        CLK_state2 = digitalRead(CLK_PIN2);
        //if (CLK_state2 != prev_CLK_state2 && CLK_state2 == HIGH)  // Check if the CLK status has changed to high
        if (CLK_state2 != prev_CLK_state2 && CLK_state2 == LOW)  // Check if the CLK status has changed to high
        //if (CLK_state2 != prev_CLK_state2)
        {
          //if (digitalRead(DT_PIN2) == HIGH)
          if (digitalRead(DT_PIN2) != CLK_state2)
          { recoveryMode++; }
          else
          { recoveryMode--; }

          if (recoveryMode > 1) {recoveryMode =1;}
          if (recoveryMode < 0) {recoveryMode =0;}
          lastTurnTime = millis();
        }
        prev_CLK_state2 = CLK_state2;
      }

      u8g2.drawStr(1,14, "[ -- Rotate Enckoder -- ]            ");

      if (recoveryMode == 0)
      {
        u8g2.drawStr(1,28, ">> RESET BANK=1, STATION=1 <<");
        u8g2.drawStr(1,42, "   RESET WIFI SSID, PASSWD   ");  
      }
      else if (recoveryMode == 1)
      {
        u8g2.drawStr(1,28, "   RESET BANK=1, STATION=1   ");
        u8g2.drawStr(1,42, ">> RESET WIFI SSID, PASSWD <<");      
      }
      u8g2.sendBuffer();
      
      if (digitalRead(SW_PIN2) == 0)
      {
        if (recoveryMode == 0)
        {
          bank_nr = 1;
          station_nr = 1;
          saveStationOnSD();
          u8g2.clearBuffer(); 
          u8g2.drawStr(1,14, "SET BANK=1, STATION=1         ");
          u8g2.drawStr(1,28, "ESP will RESET in 3sec.       ");
          u8g2.sendBuffer();
          delay(3000);
          ESP.restart();
        }  
        else if (recoveryMode == 1)
        {
          u8g2.clearBuffer();
          u8g2.drawStr(1,14, "WIFI SSID, PASSWD CLEARED   ");
          u8g2.drawStr(1,28, "ESP will RESET in 3sec.     ");
          u8g2.sendBuffer();
          wifiManager.resetSettings();
          delay(3000);
          ESP.restart();
        }
      }
      
      delay(20);
    }   
  } 
}


void displayDimmerTimer()
{
  displayDimmerTimeCounter++;
  
  if ((displayActive == true) ) //&& (displayDimmerActive == true) If the display is active and dimmed
  { 
    displayDimmerTimeCounter = 0;
    //Serial.println("debug displayDimmerTimer -> displayDimmerTimeCounter = 0");
    if (displayDimmerActive == true) {displayDimmer(0);}
  }
  // If the time after which we are supposed to dim the display has passed and the dimming function is enabled and I am not in the OTA update function, then
  if ((displayDimmerTimeCounter >= displayAutoDimmerTime) && (displayAutoDimmerOn == true) && (fwupd == false)) 
  {
    if (displayDimmerActive == false) // if the display is not dimmed yet
    {
      displayDimmer(1); // we call the dim function with parameter 1 (enable)
      timer2.detach();
    }
    displayDimmerTimeCounter = 0;
  }

  displayPowerSaveTimeCounter++;
  
  if (displayPowerSaveTimeCounter >= displayPowerSaveTime && displayPowerSaveEnabled && !fwupd) 
  {
    displayPowerSaveTimeCounter = 0;
    displayPowerSave(1);
    timer2.detach();
  }
}

void displayDimmer(bool dimmerON)
{
  displayDimmerActive = dimmerON;
  Serial.print("debug displayDimmer -> displayDimmerActive: ");
  Serial.println(displayDimmerActive);
  
  if ((dimmerON == 1) && (displayActive == false) && (fwupd == false)) { u8g2.setContrast(dimmerDisplayBrightness);}
  if (dimmerON == 0) 
  { 
    displayPowerSave(0);
    u8g2.setContrast(displayBrightness); 
    displayDimmerTimeCounter = 0;
    displayPowerSaveTimeCounter = 0;
    timer2.attach(1, displayDimmerTimer);
  }
}

#ifdef twoEncoders
  void handleEncoder1()
  {
    CLK_state1 = digitalRead(CLK_PIN1);                       // Reading the current state of the CLK pin of encoder 1
    if (CLK_state1 != prev_CLK_state1 && CLK_state1 == HIGH)  // Check if the CLK status has changed to high
    {
      timeDisplay = false;
      displayActive = true;
      displayStartTime = millis();

      if (bankMenuEnable == false)  // Scrolling through the list of radio stations
      {
        station_nr = currentSelection + 1;
        if (digitalRead(DT_PIN1) == HIGH) 
        {
          station_nr--;
          //if (listedStations == 1) station_nr--;
          if (station_nr < 1) 
          {
            station_nr = stationsCount;//1;
          }
          Serial.print("debug ENC1 -> Station number backwards: ");
          Serial.println(station_nr);
          scrollUp();
        } 
        else 
        {
          station_nr++;
          if (station_nr > stationsCount) 
          {
            station_nr = 1;//stationsCount;
          }
          Serial.print("debug ENC1 -> Station number forward: ");
          Serial.println(station_nr);
          scrollDown();
        }
        displayStations();
      } 
      else // Scrolling the list of banks
      {
        if (bankMenuEnable == true)  // Scrolling through the list of radio station banks
        {
          if (digitalRead(DT_PIN1) == HIGH) 
          {
            bank_nr--;
            if (bank_nr < 1) 
            {
              bank_nr = bank_nr_max;
            }
          } 
          else 
          {
            bank_nr++;
            if (bank_nr > bank_nr_max) 
            {
              bank_nr = 1;
            }
          }
          bankMenuDisplay();
        }
      }
    }
    prev_CLK_state1 = CLK_state1;
    


    if ((button1.isReleased()) && (listedStations == false)) 
    {
      timeDisplay = false;
      volumeSet = false;
      displayActive = true;
      displayStartTime = millis();

      if (bankMenuEnable == true)
      {
        bankMenuEnable = false;
        fetchStationsFromServer();
        changeStation();
        displayRadio();
        clearFlags();
        return;
      }
      else if (bankMenuEnable == false)
      {
        bankMenuEnable = true;  
        bankMenuDisplay();// Pressing encoder 1 displays the Banks menu
        return;
      }
    }
    
    
    /*
    if ((button1.isReleased()) && (listedStations == false) && (bankMenuEnable == true)) 
    {
      bankMenuEnable = false;
      displayStartTime = millis();
      timeDisplay = false;
      volumeSet = false;
      displayActive = true;
      previous_bank_nr = bank_nr;
      station_nr = 1;
      fetchStationsFromServer();
      changeStation();
      displayRadio();
      clearFlags();
      return;
    }
    */

    /*
    if (button1.isPressed() && listedStations == false && bankMenuEnable == false) 
    {
      
      bankMenuEnable = true;
      listedStations = false;
      volumeSet = false;
      timeDisplay = false;
      volumeSet = false;
      displayActive = true;
      
      displayStartTime = millis();
      bankMenuDisplay();// Pressing encoder 1 displays the Banks menu
      return;
      
    }
    //*/
    

    if (button1.isReleased() && listedStations) 
    {
      listedStations = false;
      volumeSet = false;
      changeStation();
      displayRadio();
      //u8g2.sendBuffer();
      clearFlags();
      return;
    }

    

    
  }
#endif


void handleEncoder2StationsVolumeClick()
{
  CLK_state2 = digitalRead(CLK_PIN2);                       // Reading the current state of the CLK pin of encoder 2
  if (CLK_state2 != prev_CLK_state2 && CLK_state2 == HIGH)  // Check if the CLK status has changed to high
  {
    timeDisplay = false;
    displayActive = true;
    displayStartTime = millis();

    if ((volumeSet == false) && (bankMenuEnable == false))  // Scrolling through the list of radio stations
    {
      currentSelection = station_nr - 1;

      if (digitalRead(DT_PIN2) == HIGH) 
      {  
        if (listedStations == true)
        {
          station_nr--;
          if (station_nr < 1) {station_nr = stationsCount;} //1;
          Serial.print("Station number backwards: "); Serial.println(station_nr);
          
          scrollUp();
        }
        else
        {
          if (maxSelection() - currentSelection < maxVisibleLines) // 98 - 98 = 0 < 4 = YES
          {
            firstVisibleLine = maxSelection() - maxVisibleLines + 1; // 98 - 4- 1 -> 93, too far, must be 95
          } 
          else 
          {
            firstVisibleLine = currentSelection;
          }
          /*
          if (currentSelection > 0) // if the current meaning is less than the first displayed line
          { 
            if (currentSelection < firstVisibleLine) {firstVisibleLine = currentSelection;}
          } 
          else // If value 0 is reached, go to the highest value 
          {  
            if (currentSelection == maxSelection()) {firstVisibleLine = currentSelection - maxVisibleLines + 1;} // Set the first visible line to the highest
            
          } 
          */  
        } 
      }  
      else 
      {
        if (listedStations == true)
        {
          station_nr++;
          //if (listedStations == 1) station_nr++;
          if (station_nr > stationsCount) {station_nr = 1;}//stationsCount;
          Serial.print("Station number forward: "); Serial.println(station_nr);

          scrollDown();
        }
        else
        {    
                 
          //currentSelection = station_nr - 1; // We restore the highlighting of the currently playing station
          
          if (maxSelection() - currentSelection < maxVisibleLines) {firstVisibleLine = maxSelection() - maxVisibleLines + 1;} else {firstVisibleLine = currentSelection;}
          /*
          if (currentSelection > 0)
          {
            if (currentSelection < firstVisibleLine) // if the current meaning is less than the first displayed line
            {
              firstVisibleLine = currentSelection;
            }
          } 
          else 
          {  // If value 0 is reached, go to the highest value
            if (currentSelection = maxSelection())
            {
              firstVisibleLine = currentSelection - maxVisibleLines + 1;  // Set the first visible line to the highest
            }
          }
          */ 
        }
      }
      displayStations();
    } 
    else 
    {
      if (bankMenuEnable == false)
      {
        if (digitalRead(DT_PIN2) == HIGH) // turning encoder 2
        {volumeDown();} 
        else 
        {volumeUp();}
      }
    }

    if (bankMenuEnable == true)  // Scrolling through the list of radio station banks
    {
      if (digitalRead(DT_PIN2) == HIGH) 
      {
        bank_nr--;
        if (bank_nr < 1) {bank_nr = bank_nr_max;}
      } 
      else 
      {
        bank_nr++;
        if (bank_nr > bank_nr_max) {bank_nr = 1;}
      }
      bankMenuDisplay();
    }
  }
  prev_CLK_state2 = CLK_state2;
      
  if ((button2.isReleased()) && (listedStations == true)) 
  {
    listedStations = false;
    volumeSet = false;
    changeStation();
    displayRadio();
    u8g2.sendBuffer();
    clearFlags();
  }

  if ((button2.isPressed()) && (listedStations == false) && (bankMenuEnable == false)) 
  {
    displayStartTime = millis();
    timeDisplay = false;
    volumeSet = true;
    displayActive = true;
    volumeDisplay();  // After pressing encoder 2 we display the volume menu
  }

  if ((button2.isPressed()) && (bankMenuEnable == true)) 
  {
    station_nr = 1;
    fetchStationsFromServer();
    changeStation();
    u8g2.clearBuffer();
    displayRadio();

    volumeSet = false;
    bankMenuEnable = false;
  }

}



void handleEncoder2VolumeStationsClick()
{
  CLK_state2 = digitalRead(CLK_PIN2);                       // Reading the current state of the CLK pin of encoder 2
  if (CLK_state2 != prev_CLK_state2 && CLK_state2 == HIGH)  // Check if the CLK status has changed to high
  {
    timeDisplay = false;
    displayActive = true;
    displayStartTime = millis();

    if ((listedStations == false) && (bankMenuEnable == false))  // Scrolling through the list of radio stations
    {
      if (digitalRead(DT_PIN2) == HIGH) 
      {
        volumeDown();
      } 
      else 
      {
        volumeUp();
      }
    } 
    else 
    {
      if ((bankMenuEnable == false) && (volumeSet == true)) 
      {
        if (digitalRead(DT_PIN2) == HIGH)  // turning encoder 2
        {
          volumeDown();
        } 
        else 
        {
          volumeUp();
        }
      
      }
    }
    #ifndef twoEncoders // We compile this block if there is no "two Encoders" defined, for 2 encoders it is unnecessary
      if ((listedStations == true) && (bankMenuEnable == false)) 
      {
        station_nr = currentSelection + 1;
        if (digitalRead(DT_PIN2) == HIGH) 
        {
          station_nr--;
          if (station_nr < 1) { station_nr = stationsCount; }
          scrollUp();
        } 
        else 
        {
          station_nr++;
          if (station_nr > stationsCount) { station_nr = 1; }  //stationsCount;
          scrollDown();
        }
        displayStations();
      }

      if (bankMenuEnable == true)  // Scrolling through the list of radio station banks
      {
        if (digitalRead(DT_PIN2) == HIGH) 
        {
          bank_nr--;
          if (bank_nr < 1) 
          {
            bank_nr = bank_nr_max;
          }
        } 
        else 
        {
          bank_nr++;
          if (bank_nr > bank_nr_max) 
          {
            bank_nr = 1;
          }
        }
        bankMenuDisplay();
      }
    #endif    
  }
  prev_CLK_state2 = CLK_state2;


  if ((button2.isReleased()) && (encoderButton2 == true))  // we are already in the station list menu, we change stations after pressing the button
  {
    encoderButton2 = false;
    //  Serial.println("debug--------------------------------> SET ENCODER button 2 FALSE");
  }

  #ifdef twoEncoders
    if ((button2.isPressed())) // we're changing the station
    {
      volumeMute = !volumeMute;
      if (volumeMute == true)
      {
        audio.setVolume(0);   
      }
      else if (volumeMute == false)
      {
        audio.setVolume(volumeValue);   
      }
      displayRadio();
      return;
    }
  #endif

  if ((button2.isPressed())) // we're changing the station
  {
    
    if ((encoderButton2 == false) && (listedStations == true) && (bankMenuEnable == false) && (volumeSet == false))  // we are already in the station list menu, we change stations after pressing the button
    {
      encoderButton2 = true;
      u8g2.clearBuffer();
      changeStation();
      displayRadio();
      clearFlags();       
    }
  
    else if ((encoderButton2 == false) && (listedStations == false) && (bankMenuEnable == false) && (volumeSet == false))  // we enter the list
    {
      displayStartTime = millis();
      timeDisplay = false;
      displayActive = true;
      listedStations = true;
      currentSelection = station_nr - 1; 

      if (maxSelection() - currentSelection < maxVisibleLines) 
      {
        firstVisibleLine = maxSelection() - maxVisibleLines + 1;
      } 
      else 
      {
        firstVisibleLine = currentSelection;
      }
      displayStations();
    } 
      
    else if ((bankMenuEnable == true) && (volumeSet == false)) 
    {
      displayStartTime = millis();
      timeDisplay = false;
      displayActive = true;
      
      //previous_bank_nr = bank_nr;
      station_nr = 1;

      listedStations = false;
      volumeSet = false;
      bankMenuEnable = false;
      bankNetworkUpdate = false;

      fetchStationsFromServer();
      changeStation();
      u8g2.clearBuffer();
      displayRadio();
    }  
  }
}


void stationNameSwap()
{
  stationNameFromStream = !stationNameFromStream;
  if (stationNameFromStream) stationNameLenghtCut = 40; else stationNameLenghtCut = 24;
}

void saveConfig() 
{
  // Check if the file exists
  if (STORAGE.exists("/config.txt")) 
  {
    Serial.println("The config.txt file already exists.");

    // Open the file for writing and overwrite the current configuration value
    myFile = STORAGE.open("/config.txt", FILE_WRITE);
    if (myFile) 
	  {
      myFile.println("#### Evo Web Radio Config File ####");
      myFile.print("Display Brightness =");    myFile.print(displayBrightness); myFile.println(";");
      myFile.print("Dimmer Display Brightness =");    myFile.print(dimmerDisplayBrightness); myFile.println(";");
      myFile.print("Auto Dimmer Time ="); myFile.print(displayAutoDimmerTime); myFile.println(";");
      myFile.print("Auto Dimmer ="); myFile.print(displayAutoDimmerOn); myFile.println(";");
      myFile.println("Voice Info Every Hour =" + String(timeVoiceInfoEveryHour) + ";");
      myFile.println("VU Meter Mode =" + String(vuMeterMode) + ";");
      myFile.println("Encoder Function Order  =" + String(encoderFunctionOrder) + ";");
      myFile.println("Startup Display Mode  =" + String(displayMode) + ";");
      myFile.println("VU Meter On  =" + String(vuMeterOn) + ";");
	    myFile.println("VU Meter Refresh Time  =" + String(vuMeterRefreshTime) + ";");
      myFile.println("Scroller Refresh Time =" + String(scrollingRefresh) + ";");
      myFile.println("ADC Keyboard Enabled =" + String(adcKeyboardEnabled) + ";");
      myFile.println("Display Power Save Enabled =" + String(displayPowerSaveEnabled) + ";");  
      myFile.println("Display Power Save Time =" + String(displayPowerSaveTime) + ";");
      myFile.println("Max Volume Extended =" + String(maxVolumeExt) + ";");
      myFile.println("VU Meter Peak Hold On =" + String(vuPeakHoldOn) + ";");
      myFile.println("VU Meter Smooth On =" + String(vuSmooth) + ";");
      myFile.println("VU Meter Rise Speed =" + String(vuRiseSpeed) + ";");
      myFile.println("VU Meter Fall Speed =" + String(vuFallSpeed) + ";");
      myFile.println("Display Clock in Sleep =" + String(f_displayPowerOffClock) + ";");
      myFile.print("Dimmer Sleep Display Brightness =");    myFile.print(dimmerSleepDisplayBrightness); myFile.println(";");
      myFile.print("Radio switch to standby after Power Fail =");    myFile.print(f_sleepAfterPowerFail); myFile.println(";");
      myFile.println("Volume fade on station change and power off =" + String(f_volumeFadeOn) + ";");
      myFile.println("Save Always Station Bank Volume or only during power off =" + String(f_saveVolumeStationAlways) + ";");
      myFile.println("Power Off Animation =" + String(f_powerOffAnimation) + ";");
      myFile.println("Presets On =" + String(f_Presets) + ";");

      myFile.close();
      Serial.println("Updating config.txt on SD card");
    } 
	  else 
	  {
      Serial.println("Error opening config.txt file.");
    }
  } 
  else 
  {
    Serial.println("The config.txt file does not exist. Creating...");

    // Create a file and save the current configuration values ​​in it
    myFile = STORAGE.open("/config.txt", FILE_WRITE);
    if (myFile) 
	  {
      myFile.println("#### Evo Web Radio Config File ####");
      myFile.print("Display Brightness =");    myFile.print(displayBrightness); myFile.println(";");
      myFile.print("Dimmer Display Brightness =");    myFile.print(dimmerDisplayBrightness); myFile.println(";");
      myFile.print("Auto Dimmer Time ="); myFile.print(displayAutoDimmerTime); myFile.println(";");
      myFile.print("Auto Dimmer On ="); myFile.print(displayAutoDimmerOn); myFile.println(";");
		  myFile.println("Voice Info Every Hour =" + String(timeVoiceInfoEveryHour) + ";");
      myFile.println("VU Meter Mode =" + String(vuMeterMode) + ";");
      myFile.println("Encoder Function Order  =" + String(encoderFunctionOrder) + ";");
      myFile.println("Startup Display Mode  =" + String(displayMode) + ";");
      myFile.println("VU Meter On  =" + String(vuMeterOn) + ";");
      myFile.println("VU Meter Refresh Time  =" + String(vuMeterRefreshTime) + ";");
      myFile.println("Scroller Refresh Time =" + String(scrollingRefresh) + ";");
      myFile.println("ADC Keyboard Enabled =" + String(adcKeyboardEnabled) + ";");
      myFile.println("Display Power Save Enabled =" + String(displayPowerSaveEnabled) + ";");  
      myFile.println("Display Power Save Time =" + String(displayPowerSaveTime) + ";");
      myFile.println("Max Volume settings Extended =" + String(maxVolumeExt) + ";");
      myFile.println("VU Meter Peak Hold On =" + String(vuPeakHoldOn) + ";");
      myFile.println("VU Meter Smooth On =" + String(vuSmooth) + ";");
      myFile.println("VU Meter Rise Speed =" + String(vuRiseSpeed) + ";");
      myFile.println("VU Meter Fall Speed =" + String(vuFallSpeed) + ";");
      myFile.println("Display Clock in Sleep =" + String(f_displayPowerOffClock) + ";");
      myFile.print("Dimmer Sleep Display Brightness =");    myFile.print(dimmerSleepDisplayBrightness); myFile.println(";");
      myFile.print("Radio goes to sleep after Power Fail =");    myFile.print(f_sleepAfterPowerFail); myFile.println(";");
      myFile.println("Volume fade on station change and power off =" + String(f_volumeFadeOn) + ";");
      myFile.println("Save Always Station Bank Volume or only during power off =" + String(f_saveVolumeStationAlways) + ";");
      myFile.println("Power Off Animation =" + String(f_powerOffAnimation) + ";");
      myFile.println("Presets On =" + String(f_Presets) + ";");

      myFile.close();
      Serial.println("Created and saved config.txt to SD card");
    } 
	  else 
	  {
      Serial.println("Error creating config.txt file.");
    }
  }


}

void saveAdcConfig() 
{

  // Check if the file exists
  if (STORAGE.exists("/adckbd.txt")) 
  {
    Serial.println("The file adckbd.txt already exists.");

    // Open the file for saving and overwrite the current ADC keyboard configuration value
    myFile = STORAGE.open("/adckbd.txt", FILE_WRITE);
    if (myFile) 
	  {
      myFile.println("#### Evo Web Radio Config File - ADC Keyboard ####");
      myFile.println("keyboardButtonThreshold_0 =" + String(keyboardButtonThreshold_0) + ";");
      myFile.println("keyboardButtonThreshold_1 =" + String(keyboardButtonThreshold_1) + ";");
      myFile.println("keyboardButtonThreshold_2 =" + String(keyboardButtonThreshold_2) + ";");
      myFile.println("keyboardButtonThreshold_3 =" + String(keyboardButtonThreshold_3) + ";");
      myFile.println("keyboardButtonThreshold_4 =" + String(keyboardButtonThreshold_4) + ";");
      myFile.println("keyboardButtonThreshold_5 =" + String(keyboardButtonThreshold_5) + ";");
      myFile.println("keyboardButtonThreshold_6 =" + String(keyboardButtonThreshold_6) + ";");
      myFile.println("keyboardButtonThreshold_7 =" + String(keyboardButtonThreshold_7) + ";");
      myFile.println("keyboardButtonThreshold_8 =" + String(keyboardButtonThreshold_8) + ";");
      myFile.println("keyboardButtonThreshold_9 =" + String(keyboardButtonThreshold_9) + ";");
      myFile.println("keyboardButtonThreshold_Ok =" + String(keyboardButtonThreshold_Ok) + ";");
      myFile.println("keyboardButtonThreshold_BankMenu =" + String(keyboardButtonThreshold_BankMenu) + ";");
      myFile.println("keyboardButtonThreshold_Back =" + String(keyboardButtonThreshold_Back) + ";");
      myFile.println("keyboardButtonThreshold_DisplayMode =" + String(keyboardButtonThreshold_DisplayMode) + ";");
      myFile.println("keyboardButtonThreshold_Dimmer =" + String(keyboardButtonThreshold_Dimmer) + ";");
      myFile.println("keyboardButtonThreshold_Mute =" + String(keyboardButtonThreshold_Mute) + ";");
      myFile.println("keyboardButtonThresholdTolerance =" + String(keyboardButtonThresholdTolerance) + ";");
      myFile.println("keyboardButtonNeutral =" + String(keyboardButtonNeutral) + ";");
      myFile.println("keyboardSampleDelay =" + String(keyboardSampleDelay) + ";");
      
      myFile.println("keyboardButtonThreshold_ArrowLeft =" + String(keyboardButtonThreshold_ArrowLeft) + ";");
      myFile.println("keyboardButtonThreshold_ArrowRight =" + String(keyboardButtonThreshold_ArrowRight) + ";");
      myFile.println("keyboardButtonThreshold_ArrowUp =" + String(keyboardButtonThreshold_ArrowUp) + ";");
      myFile.println("keyboardButtonThreshold_ArrowDown =" + String(keyboardButtonThreshold_ArrowDown) + ";");
      
      myFile.println("keyboardButtonThreshold_PresetsOn =" + String(keyboardButtonThreshold_PresetsOn) + ";");
      

      myFile.close();
      Serial.println("Updating adckbd.txt on SD card/Storage");
    } 
	  else 
	  {
      Serial.println("Error opening adckbd.txt file.");
    }
  } 
  else 
  {
    Serial.println("The adckbd.txt file does not exist. Creating...");

    // Create a file and save the current configuration values ​​in it
    myFile = STORAGE.open("/adckbd.txt", FILE_WRITE);
    if (myFile) 
	  {
      myFile.println("#### Evo Web Radio Config File - ADC Keyboard ####");
      myFile.println("keyboardButtonThreshold_0 =" + String(keyboardButtonThreshold_0) + ";");
      myFile.println("keyboardButtonThreshold_1 =" + String(keyboardButtonThreshold_1) + ";");
      myFile.println("keyboardButtonThreshold_2 =" + String(keyboardButtonThreshold_2) + ";");
      myFile.println("keyboardButtonThreshold_3 =" + String(keyboardButtonThreshold_3) + ";");
      myFile.println("keyboardButtonThreshold_4 =" + String(keyboardButtonThreshold_4) + ";");
      myFile.println("keyboardButtonThreshold_5 =" + String(keyboardButtonThreshold_5) + ";");
      myFile.println("keyboardButtonThreshold_6 =" + String(keyboardButtonThreshold_6) + ";");
      myFile.println("keyboardButtonThreshold_7 =" + String(keyboardButtonThreshold_7) + ";");
      myFile.println("keyboardButtonThreshold_8 =" + String(keyboardButtonThreshold_8) + ";");
      myFile.println("keyboardButtonThreshold_9 =" + String(keyboardButtonThreshold_9) + ";");
      myFile.println("keyboardButtonThreshold_Ok =" + String(keyboardButtonThreshold_Ok) + ";");
      myFile.println("keyboardButtonThreshold_BankMenu =" + String(keyboardButtonThreshold_BankMenu) + ";");
      myFile.println("keyboardButtonThreshold_Back =" + String(keyboardButtonThreshold_Back) + ";");
      myFile.println("keyboardButtonThreshold_DisplayMode =" + String(keyboardButtonThreshold_DisplayMode) + ";");
      myFile.println("keyboardButtonThreshold_Dimmer =" + String(keyboardButtonThreshold_Dimmer) + ";");
      myFile.println("keyboardButtonThreshold_Mute =" + String(keyboardButtonThreshold_Mute) + ";");
      myFile.println("keyboardButtonThresholdTolerance =" + String(keyboardButtonThresholdTolerance) + ";");
      myFile.println("keyboardButtonNeutral =" + String(keyboardButtonNeutral) + ";");
      myFile.println("keyboardSampleDelay =" + String(keyboardSampleDelay) + ";");

      myFile.println("keyboardButtonThreshold_ArrowLeft =" + String(keyboardButtonThreshold_ArrowLeft) + ";");
      myFile.println("keyboardButtonThreshold_ArrowRight =" + String(keyboardButtonThreshold_ArrowRight) + ";");
      myFile.println("keyboardButtonThreshold_ArrowUp =" + String(keyboardButtonThreshold_ArrowUp) + ";");
      myFile.println("keyboardButtonThreshold_ArrowDown =" + String(keyboardButtonThreshold_ArrowDown) + ";");
      
      myFile.println("keyboardButtonThreshold_PresetsOn =" + String(keyboardButtonThreshold_PresetsOn) + ";");

      myFile.close();
      Serial.println("Created and saved adckbd.txt to SD card/Storage");
    } 
	  else 
	  {
      Serial.println("Error creating adckbd.txt file.");
    }
  }


}


void readConfig() 
{

  Serial.println("Reading the config.txt file from the card");
  String fileName = String("/config.txt"); // We create a file name

  if (!STORAGE.exists(fileName))               // We check if the file exists
  {
    Serial.println("Error: File does not exist.");
    configExist = false;
    return;
  }
 
  File configFile = STORAGE.open(fileName, FILE_READ);// We open the file in read mode
  if (!configFile)  // if the file is missing then...
  {
    Serial.println("Error: Could not open configuration file");
    return;
  }
  // We go to the appropriate line of the file
  int currentLine = 0;
  String configValue = "";
  while (configFile.available() && currentLine < CONFIG_COUNT) 

  {
    String line = configFile.readStringUntil(';'); //('\n');
    
    int lineStart = line.indexOf("=") + 1;  // We are looking for the place where the value of the variable begins
    if ((lineStart != -1)) //&& (currentLine != 0)) // We skip the first line where the file description is
	  {
      configValue = line.substring(lineStart);  // We extract the number from the places a"="
      configValue.trim();                       // We remove whitespace at the beginning and end
      Serial.print("debug SD -> Configuration variable number read:" + String(currentLine) + " value:");
      Serial.println(configValue);
      configArray[currentLine] = configValue.toInt();
    }
    currentLine++;
  }
  Serial.print("We close the config file at the currentLine value:");
  Serial.println(currentLine);
  
  //Odczyt kontrolny
  //for (int i = 0; i < 16; i++) 
  //{
  //  Serial.print("value: " + String(s) + " from the configuration table:");
  //  Serial.println(configArray[i]);
  //}

  configFile.close();  // We close the file after reading the remote control codes

  displayBrightness = configArray[0];
  dimmerDisplayBrightness = configArray[1];
  displayAutoDimmerTime = configArray[2];
  displayAutoDimmerOn = configArray[3];
  timeVoiceInfoEveryHour = configArray[4];
  vuMeterMode = configArray[5];
  encoderFunctionOrder = configArray[6];
  displayMode = configArray[7];
  vuMeterOn = configArray[8];
  vuMeterRefreshTime = configArray[9];
  scrollingRefresh = configArray[10];
  adcKeyboardEnabled = configArray[11];
  displayPowerSaveEnabled = configArray[12];
  displayPowerSaveTime = configArray[13];
  maxVolumeExt = configArray[14];
  vuPeakHoldOn = configArray[15];
  vuSmooth = configArray[16];
  vuRiseSpeed = configArray[17];
  vuFallSpeed = configArray[18];
  f_displayPowerOffClock = configArray[19];
  dimmerSleepDisplayBrightness = configArray[20];
  f_sleepAfterPowerFail = configArray[21];
  f_volumeFadeOn = configArray[22];
  f_saveVolumeStationAlways = configArray[23];
  f_powerOffAnimation = configArray[24];
  f_Presets = configArray[25];

  if (maxVolumeExt == 1)
  { 
    maxVolume = 42;
  }
  else
  {
    maxVolume = 21;
  }
  audio.setVolumeSteps(maxVolume);
  //stationNameSwap();
}


void readAdcConfig() 
{

  Serial.println("Reading the ADC keyboard configuration file adckbd.txt from the card");
  String fileName = String("/adckbd.txt"); // We create a file name

  if (!STORAGE.exists(fileName))               // We check if the file exists
  {
    Serial.println("Error: File does not exist.");
    return;
  }
 
  File configFile = STORAGE.open(fileName, FILE_READ);// We open the file in read mode
  if (!configFile)  // if the file is missing then...
  {
    Serial.println("Error: Unable to open ADC keyboard configuration file");
    return;
  }
  // We go to the appropriate line of the file
  int currentLine = 0;
  String configValue = "";
  while (configFile.available()) 
  {
    String line = configFile.readStringUntil(';'); //('\n');
    
    int lineStart = line.indexOf("=") + 1;  // We are looking for the place where the value of the variable begins
    if ((lineStart != -1)) //&& (currentLine != 0)) // We skip the first line where the file description is
	  {
      configValue = line.substring(lineStart);  // We extract the URL from "http"
      configValue.trim();                      // We remove whitespace at the beginning and end
      Serial.print(" Read ADC setting no.:" + String(currentLine) + " value:");
      Serial.println(configValue);
      configAdcArray[currentLine] = configValue.toInt();
    }
    currentLine++;
  }
  Serial.print("We close the config file at the currentLine value:");
  Serial.println(currentLine);
  
  //Control reading
  //for (int i = 0; i < 16; i++) 
  //{
  //  Serial.print("ADC level value: " + String(i) + " from array:");
  //  Serial.println(configAdcArray[i]);
 // }

  configFile.close();  // We close the file after reading the remote control codes

 // ---- ADC switching thresholds for the 5x3 matrix keyboard in the Sony ST-120 tuner ---- //
        // Neutral position
  keyboardButtonThreshold_0 = configAdcArray[0];          // Button 0
  keyboardButtonThreshold_1 = configAdcArray[1];          // Button 1
  keyboardButtonThreshold_2 = configAdcArray[2];          // Button 2
  keyboardButtonThreshold_3 = configAdcArray[3];          // Button 3
  keyboardButtonThreshold_4 = configAdcArray[4];          // Button 4
  keyboardButtonThreshold_5 = configAdcArray[5];          // Button 5
  keyboardButtonThreshold_6 = configAdcArray[6];        // Button 6
  keyboardButtonThreshold_7 = configAdcArray[7];        // Button 7
  keyboardButtonThreshold_8 = configAdcArray[8];        // Button 8
  keyboardButtonThreshold_9 = configAdcArray[9];        // Button 9
  keyboardButtonThreshold_Ok = configAdcArray[10];   // Shift - Enter/OK function
  keyboardButtonThreshold_BankMenu = configAdcArray[11];  // Memory - Bank menu function
  keyboardButtonThreshold_Back = configAdcArray[12];    // Band button - Back function
  keyboardButtonThreshold_DisplayMode = configAdcArray[13];    // Auto button - switches between Radio/Clock
  keyboardButtonThreshold_Dimmer = configAdcArray[14];    // Scan button - OLED screen dimmer function
  keyboardButtonThreshold_Mute = configAdcArray[15];      // Mute button - MUTE function
  keyboardButtonThresholdTolerance = configAdcArray[16];  // ADC measurement tolerance
  keyboardButtonNeutral = configAdcArray[17];             // Neutral position
  keyboardSampleDelay = configAdcArray[18];               // Time to read the keyboard in how many ms
  
  keyboardButtonThreshold_ArrowLeft = configAdcArray[19];
  keyboardButtonThreshold_ArrowRight = configAdcArray[20];
  keyboardButtonThreshold_ArrowUp = configAdcArray[21];
  keyboardButtonThreshold_ArrowDown = configAdcArray[22];
  keyboardButtonThreshold_PresetsOn = configAdcArray[23];
}



void readPSRAMstations()  // Test-debug function, for reading PSRAM, not used by other functions
{
  Serial.println("-------- STATION LIST TOP ---------- ");
  for (int i = 0; i < stationsCount; i++) 
  {
    // Reading the station for a given PSRAM memory cell:
    char station[STATION_NAME_LENGTH + 1];  // An array for the station name with a maximum length defined by STATION_NAME_LENGTH
    memset(station, 0, sizeof(station));    // Clearing the table with zeros before writing the data
    int length = psramData[(i) * (STATION_NAME_LENGTH + 1)];   // Read station name length from PSRAM for current station index
      
    for (int j = 0; j < min(length, STATION_NAME_LENGTH); j++) 
    { // Read the station name from PSRAM as a sequence of bytes, up to a maximum of STATION_NAME_LENGTH
        station[j] = psramData[(i) * (STATION_NAME_LENGTH + 1) + 1 + j];  // Read the station name character by character
    }
    String stationNameText = String(station);
    
    Serial.print(i+1);
    Serial.print(" ");
    Serial.println(stationNameText);
  }	
  
  Serial.println("-------- END OF STATION LIST ---------- ");

  String stationUrl = "";

  // Reading the station for a given PSRAM memory cell:
  char station[STATION_NAME_LENGTH + 1];  // An array for the station name with a maximum length defined by STATION_NAME_LENGTH
  memset(station, 0, sizeof(station));    // Clearing the table with zeros before writing the data
  int length = psramData[(station_nr - 1) * (STATION_NAME_LENGTH + 1)];   // Read station name length from PSRAM for current station index
      
  for (int j = 0; j < min(length, STATION_NAME_LENGTH); j++) 
  { // Read the station name from PSRAM as a sequence of bytes, up to a maximum of STATION_NAME_LENGTH
    station[j] = psramData[(station_nr - 1) * (STATION_NAME_LENGTH + 1) + 1 + j];  // Read the station name character by character
  }
  
  //String stationNameText = String(station);
  
  Serial.println("--CURRENTLY WE ARE PLAYING-- ");
  Serial.print(station_nr - 1);
  Serial.print(" ");
  Serial.println(String(station));

}

void webUrlStationPlay() 
{
  audio.stopSong();

  // Removing all characters from objects
  stationString.remove(0);  
  stationNameStream.remove(0);
  stationStringWeb.remove(0);
  stationLogoUrl.remove(0);

  bitrateString.remove(0);
  sampleRateString.remove(0);
  bitsPerSampleString.remove(0); 
  SampleRate = 0;
  SampleRateRest = 0;

  sampleRateString = "--.-";
  bitsPerSampleString = "--";
  bitrateString = "-?-";
    
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_fub14_tf); // 14x11 cm
  u8g2.drawStr(34, 33, "Loading stream..."); // 8 characters x 11 wide
  u8g2.sendBuffer();

  mp3 = flac = aac = vorbis = opus = false;
  streamCodec = "";
    
  Serial.println("debug-- Read station from WEB URL");
    
  if (url2play != "") 
  {
    url2play.trim(); // We remove whitespace at the beginning and end
  }
  else
  {
	  return;
  }
  
  if (url2play.isEmpty()) // if the URL link is empty
  {
    Serial.println("Error: No station found for the given number.");
    return;
  }
    
  // Verifying if the link contains "http" or "https"
  if (url2play.startsWith("http://") || url2play.startsWith("https://")) 
  {
    // Print the station name and link on the show
    Serial.print("Link to station: ");
    Serial.println(url2play);
    
    u8g2.setFont(spleen6x12PL);  // we print out what stream and what station is being loaded
    u8g2.drawStr(34, 55, String(url2play).c_str());
    u8g2.sendBuffer();
    
    // Połącz z daną stacją
    audio.connecttohost(url2play.c_str());
    urlPlaying = true;
    station_nr = 0;
    bank_nr = 0;
    //stationName = stationNameStream;
    stationName = "WEB URL";
    //wsStationChange(station_nr); // From now on, Event Audio is responsible for station info
    //f_audioInfoRefreshDisplayRadio = true;
    wsStationChange(0,0);
    wsAudioRefresh = true;
  } 
  else 
  {
    Serial.println("Error: Station link does not contain 'http' or 'https'");
    Serial.println("URL read: " + url2play);
  }

  //url2play = "";
}

String stationBankListHtmlMobile()
{
  String html1;
  
  html1 += "<p>Volume: <span id='textSliderValue'>--</span></p>" + String("\n");
  html1 += "<p><input type='range' onchange='updateSliderVolume(this)' id='volumeSlider' min='1' max='" + String(maxVolume) + "' value='1' step='1' class='slider'></p>" + String("\n");
  html1 += "<p>Memory Bank Selection:</p>" + String("\n");

  html1 += "<center>";  
  html1 += "<p>";
  //for (int i = 1; i < 17; i++) // Przyciski Banków
  for (int i = 1; i < bank_nr_max + 1; i++) // Przyciski Banków
  {
    
    if (i == bank_nr)
    {
      html1 += "<button class='buttonBankSelected' onClick=\"changeBank('" + String(i) + "');\" id=\"Bank\">" + String(i) + "</button>" + String("\n");
    }
    else
    {
      html1 += "<button class=\"buttonBank\" onClick=\"changeBank('" + String(i) + "');\" id=\"Bank\">" + String(i) + "</button>" + String("\n");
    }
    //if (i == 8) {html1 += "</p><p>";}
    if (i % 8 == 0) {html1 += "</p><p>";}
  }
  html1 += "</p>";
  html1 += "</center>";
  html1 += "<center>"; 
 
  html1 += "<table>";


  for (int i = 0; i < stationsCount; i++) // list of stations
  {
    char station[STATION_NAME_LENGTH + 1];  // An array for the station name with a maximum length defined by STATION_NAME_LENGTH
    memset(station, 0, sizeof(station));    // Clearing the table with zeros before writing the data
    int length = psramData[i * (STATION_NAME_LENGTH + 1)];
    for (int j = 0; j < min(length, STATION_NAME_LENGTH); j++) 
    {
      station[j] = psramData[i * (STATION_NAME_LENGTH + 1) + 1 + j];  // Read the station name character by character
    }
    
    html1 += "<tr>";
    html1 += "<td><p class='stationNumberList'>" + String(i + 1) + "</p></td>";
    html1 += "<td><p class='stationList' onClick=\"changeStation('" + String(i + 1) +  "');\">" + String(station).substring(0, stationNameLenghtCut) + "</p></td>";
    html1 += "</tr>" + String("\n");
          
  }
 
  html1 += "</table>" + String("\n");
  html1 += "</div>" + String("\n");

  html1 += "<p style=\"font-size: 0.8rem;\">Web Radio, mobile, Evo: " + String(softwareRev) + "</p>" + String("\n");
  html1 += "<p style='font-size: 0.8rem;'>IP: "+ currentIP + "</p>" + String("\n");
  
  html1 += "<a href='/menu' class='button' style='padding: 0.2rem; font-size: 0.7rem; height: auto; line-height: 1;color: white; width: 65px; border: 1px solid black; display: inline-block; border-radius: 5px; text-decoration: none;'>Menu</a>";
  html1 += "</center></body></html>";
  
  return html1;
}

String stationBankListHtmlPC()
{
  String html2;
  

  html2 += "<p>Volume: <span id='textSliderValue'>--</span></p>" + String("\n");
  html2 += "<p><input type='range' onchange='updateSliderVolume(this)' id='volumeSlider' min='1' max='" + String(maxVolume) + "' value='1' step='1' class='slider'></p>" + String("\n");
  html2 += "<p>Memory Bank Selection:</p>" + String("\n");
  
  
  html2 += "<p>";
  //for (int i = 1; i < 17; i++)
  for (int i = 1; i < bank_nr_max + 1; i++) // Bank Buttons
  {
    
    if (i == bank_nr)
    {
      html2 += "<button class=\"buttonBankSelected\" onClick=\"changeBank('" + String(i) + "');\" id=\"Bank\">" + String(i) + "</button>" + String("\n");
    }
    else
    {
      html2 += "<button class=\"buttonBank\" onClick=\"changeBank('" + String(i) + "');\" id=\"Bank\">" + String(i) + "</button>" + String("\n");
    }
  }
  
  html2 += "</p>";
  html2 += "<center>" + String("\n");
  html2 += "<div class=\"column\">" + String("\n");
  for (int i = 0; i < MAX_STATIONS; i++) 
  //for (int i = 0; i < stationsCount; i++) 
  {
    char station[STATION_NAME_LENGTH + 1];  // An array for the station name with a maximum length defined by STATION_NAME_LENGTH
    memset(station, 0, sizeof(station));    // Clearing the table with zeros before writing the data

    int length = psramData[i * (STATION_NAME_LENGTH + 1)];
    for (int j = 0; j < min(length, STATION_NAME_LENGTH); j++) 
    {
      station[j] = psramData[i * (STATION_NAME_LENGTH + 1) + 1 + j];  // Read the station name character by character
    }     

    
    if ((i == 0) || (i == 25) || (i == 50) || (i == 75))
    { 
      html2 += "<table>" + String("\n");
    } 
    
    // 0-98   >98
    if (i >= stationsCount) { station[0] ='\0'; } // If we have less than 99 stations, we fill the remaining cells with empty values
                 
    html2 += "<tr>";
    html2 += "<td><p class='stationNumberList'>" + String(i + 1) + "</p></td>";
    html2 += "<td><p class='stationList' onClick=\"changeStation('" + String(i + 1) +  "');\">" + String(station).substring(0, stationNameLenghtCut) + "</p></td>";
    html2 += "</tr>" + String("\n");

    if ((i == 24) || (i == 49) || (i == 74)) //||(i == 98))
    { 
      html2 += "</table>" + String("\n");
    }
           
  }

  html2 += "</table>" + String("\n");
  html2 += "</div>" + String("\n");
  html2 += "<p style=\"font-size: 0.8rem;\">Web Radio, desktop, Evo: " + String(softwareRev) + "</p>" + String("\n");
  html2 += "<p style='font-size: 0.8rem;'>IP: " + currentIP + "</p>" + String("\n");
  html2 += "<a href='/menu' class='button' style='padding: 0.2rem; font-size: 0.7rem; height: auto; line-height: 1;color: white; width: 65px; border: 1px solid black; display: inline-block; border-radius: 5px; text-decoration: none;'>Menu</a>";
  html2 += "</center></body></html>"; 

  return html2;
}

/*
String stationBankListHtml(bool mobilePage)
{
  String html3;

  html3 += "<p>Volume: <span id='textSliderValue'>%%SLIDERVALUE%%</span></p>";
  html3 += "<p><input type='range' onchange='updateSliderVolume(this)' id='volumeSlider' min='1' max='" + String(maxVolume) + "' value='%%SLIDERVALUE%%' step='1' class='slider'></p>";
  html3 += "<br>";
  html3 += "<p>Bank selection:</p>";


  html3 += "<p>";
  for (int i = 1; i < 17; i++)
  {
    
    if (i == bank_nr)
    {
      html3 += "<button class=\"buttonBankSelected\" onClick=\"bankLoad('" + String(i) + "');\" id=\"Bank\">" + String(i) + "</button>" + String("\n");
    }
    else
    {
      html3 += "<button class=\"buttonBank\" onClick=\"bankLoad('" + String(i) + "');\" id=\"Bank\">" + String(i) + "</button>" + String("\n");
    }

    if ((mobilePage == 1) && (i == 8)) {html3 += "</p><p>";}
  }
  
  html3 += "</p>";
  html3 += "<center>" + String("\n");


  if (mobilePage == 1)
  {
   html3 += "<table>";
  }
  else if (mobilePage == 0)
  { 
    html3 += "<div class=\"column\">" + String("\n");
  }
  
  int limit = (mobilePage == 1) ? stationsCount : MAX_STATIONS;

  for (int i = 0; i < limit; i++)
  {
    char station[STATION_NAME_LENGTH + 1];  // An array for the station name with a maximum length defined by STATION_NAME_LENGTH
    memset(station, 0, sizeof(station));    // Clearing the table with zeros before writing the data

    int length = psramData[i * (STATION_NAME_LENGTH + 1)];
    for (int j = 0; j < min(length, STATION_NAME_LENGTH); j++) 
    {
      station[j] = psramData[i * (STATION_NAME_LENGTH + 1) + 1 + j];  // Read the station name character by character
    }     

    
    if ((mobilePage == 0) && ((i == 0) || (i == 25) || (i == 50) || (i == 75)))
    { 
      html3 += "<table>" + String("\n");
    } 
 
    // 0-98   >98
    if (mobilePage == 0) 
    {
      if (i >= stationsCount) { station[0] ='\0'; } // If we have less than 99 stations, we fill the remaining cells with empty values
    }

    html3 += "<tr>";
    html3 += "<td><p class='stationNumberList'>" + String(i + 1) + "</p></td>";
    html3 += "<td><p class='stationList' onClick=\"changeStation('" + String(i + 1) +  "');\">" + String(station).substring(0, stationNameLenghtCut) + "</p></td>";
    html3 += "</tr>" + String("\n");
   
    if ((mobilePage == 0) && ((i == 24) || (i == 49) || (i == 74))) //||(i == 98))
    { 
      html3 += "</table>" + String("\n");
    }
           
  }

  html3 += "</table>" + String("\n");
  html3 += "</div>" + String("\n");
  if (mobilePage == 0) { html3 += "<p style=\"font-size: 0.8rem;\">Web Radio, desktop, Evo: " + String(softwareRev) + "</p>" + String("\n"); }
  if (mobilePage == 1) { html3 += "<p style=\"font-size: 0.8rem;\">Web Radio, mobile, Evo: " + String(softwareRev) + "</p>" + String("\n"); }
  html3 += "<p style='font-size: 0.8rem;'>IP: " + currentIP + "</p>" + String("\n");
  html3 += "<p style='font-size: 0.8rem;'><a href='/menu'>MENU</a></p>" + String("\n");

  html3 += "</center></body></html>"; 

  return html3;
}
*/

void voiceTime()
{
  resumePlay = true;
  voiceTimeMillis = millis();
  char chbuf[30];      //Voice-reading clock text message buffer
  String time_s;
  struct tm timeinfo;
  char timeString[9];  // A buffer that stores the time in text form
  if (!getLocalTime(&timeinfo, 1000))
  {
    Serial.println("debug voice time PL -> Nie udało się uzyskać czasu funkcji voice time");
    return;  // Terminate function when time failed
  }

  snprintf(timeString, sizeof(timeString), "%02d:%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
  time_s = String(timeString);
  int h = time_s.substring(0,2).toInt();
  snprintf(chbuf, sizeof (chbuf), "Jest godzina %i:%02i:00", h, time_s.substring(3,5).toInt());
  Serial.print("debug voice time PL -> ");
  Serial.println(chbuf);
  
    audio.connecttospeech(chbuf, "pl");
}

void voiceTimeEn()
{
  resumePlay = true;
  char chbuf[30];      //Voice-reading clock text message buffer
  String time_s;
  char am_pm[5] = "am.";
  struct tm timeinfo;
  char timeString[9];  // A buffer that stores the time in text form

  if (!getLocalTime(&timeinfo, 1000)) 
  {
    Serial.println("debug voice time EN -> Failed to get voice time");
    return;  // Terminate function when time failed
  }

  snprintf(timeString, sizeof(timeString), "%02d:%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
  time_s = String(timeString);
  int h = time_s.substring(0,2).toInt();
  if(h > 12){h -= 12; strcpy(am_pm,"pm.");}
   sprintf(chbuf, "It is now %i%s and %i minutes", h, am_pm, time_s.substring(3,5).toInt());
  Serial.print("debug voice time EN -> ");
  Serial.println(chbuf);
  audio.connecttospeech(chbuf, "en");
}

void voiceTimeNL()
{
  resumePlay = true;
  char chbuf[30];      //Voice-reading clock text message buffer
  String time_s;
  // char am_pm[5] = "am.";
  struct tm timeinfo;
  char timeString[9];  // A buffer that stores the time in text form

  if (!getLocalTime(&timeinfo, 1000)) 
  {
    Serial.println("debug voice time NL -> Kan stem tijd niet laden");
    return;  // Terminate function when time failed
  }

  snprintf(timeString, sizeof(timeString), "%02d:%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
  time_s = String(timeString);
  int h = time_s.substring(0,2).toInt();
  // if(h > 12){h -= 12; strcpy(am_pm,"pm.");}
   snprintf(chbuf, sizeof (chbuf), "het is nu %i:%02i", h, time_s.substring(3,5).toInt());
   Serial.print("debug voice time NL -> ");
  Serial.println(chbuf);
  audio.connecttospeech(chbuf, "nl");
}

/*
void saveRemoteControlConfig()
{
  Serial.println("RC - Config, saving the IR remote configuration file remote.txt");
  //String fileName = String("/remote.txt");

  const char* filename = "/remote.txt";

  File myFile = STORAGE.open(filename, FILE_WRITE);
  if (!myFile)
  {
    Serial.println("Error opening remote.txt");
    return;
  }

  myFile.println("#### Evo Web Radio Config File - IR Remote ####");

  for (uint8_t i = 0; i < CONFIGREMOTE_COUNT; i++)
  {
    myFile.printf("%s = 0x%04X;\n", remoteMap[i].name, configRemoteArray[i]);
  }
  myFile.close();
  Serial.println("Saved remote.txt");
}
*/

void saveRemoteControlConfig()
{
  Serial.println("RC - Config, saving the IR remote configuration file remote.txt");

    // Sprawdź, czy plik istnieje
  if (STORAGE.exists("/remote.txt")) 
  {
	  Serial.println("RC - Config, remote.txt file already exists.");
	  
    // Open the file for writing and overwrite the current configuration value
    myFile = STORAGE.open("/remote.txt", FILE_WRITE);
	  if (myFile)
	  {
      myFile.println("#### Evo Web Radio Config File - IR Remote ####");
      myFile.printf("%s = 0x%04X;\n", "cmdVolumeUp", configRemoteArray[0]);
      myFile.printf("%s = 0x%04X;\n", "cmdVolumeDown", configRemoteArray[1]);
      myFile.printf("%s = 0x%04X;\n", "cmdArrowRight", configRemoteArray[2]);
      myFile.printf("%s = 0x%04X;\n", "cmdArrowLeft", configRemoteArray[3]);
      myFile.printf("%s = 0x%04X;\n", "cmdArrowUp", configRemoteArray[4]);
      myFile.printf("%s = 0x%04X;\n", "cmdArrowDown", configRemoteArray[5]);
      myFile.printf("%s = 0x%04X;\n", "cmdBack", configRemoteArray[6]);
      myFile.printf("%s = 0x%04X;\n", "cmdOk", configRemoteArray[7]);
      myFile.printf("%s = 0x%04X;\n", "cmdSrc", configRemoteArray[8]);
      myFile.printf("%s = 0x%04X;\n", "cmdMute", configRemoteArray[9]);
      myFile.printf("%s = 0x%04X;\n", "cmdAud", configRemoteArray[10]);
      myFile.printf("%s = 0x%04X;\n", "cmdDirect", configRemoteArray[11]);
      myFile.printf("%s = 0x%04X;\n", "cmdBankMinus", configRemoteArray[12]);
      myFile.printf("%s = 0x%04X;\n", "cmdBankPlus", configRemoteArray[13]);
      myFile.printf("%s = 0x%04X;\n", "cmdRed", configRemoteArray[14]);
      myFile.printf("%s = 0x%04X;\n", "cmdGreen", configRemoteArray[15]);
      myFile.printf("%s = 0x%04X;\n", "cmdKey0", configRemoteArray[16]);
      myFile.printf("%s = 0x%04X;\n", "cmdKey1", configRemoteArray[17]);
      myFile.printf("%s = 0x%04X;\n", "cmdKey2", configRemoteArray[18]);
      myFile.printf("%s = 0x%04X;\n", "cmdKey3", configRemoteArray[19]);
      myFile.printf("%s = 0x%04X;\n", "cmdKey4", configRemoteArray[20]);
      myFile.printf("%s = 0x%04X;\n", "cmdKey5", configRemoteArray[21]);
      myFile.printf("%s = 0x%04X;\n", "cmdKey6", configRemoteArray[22]);
      myFile.printf("%s = 0x%04X;\n", "cmdKey7", configRemoteArray[23]);
      myFile.printf("%s = 0x%04X;\n", "cmdKey8", configRemoteArray[24]);
      myFile.printf("%s = 0x%04X;\n", "cmdKey9", configRemoteArray[25]);
      myFile.close();
      Serial.println("RC - Config, remote.txt update successful");
	  }
	  else
	  {
		  Serial.println("RC - Config, error opening remote.txt file");
	  }
  }
  else
  {
    Serial.println("RC - Config, remote.txt file does not exist. Creating...");

    // Create a file and save the current configuration values ​​in it
    myFile = STORAGE.open("/remote.txt", FILE_WRITE);
    if (myFile)
    {
      myFile.println("#### Evo Web Radio Config File - IR Remote ####");
      myFile.printf("%s = 0x%04X;\n", "cmdVolumeUp", configRemoteArray[0]);
      myFile.printf("%s = 0x%04X;\n", "cmdVolumeDown", configRemoteArray[1]);
      myFile.printf("%s = 0x%04X;\n", "cmdArrowRight", configRemoteArray[2]);
      myFile.printf("%s = 0x%04X;\n", "cmdArrowLeft", configRemoteArray[3]);
      myFile.printf("%s = 0x%04X;\n", "cmdArrowUp", configRemoteArray[4]);
      myFile.printf("%s = 0x%04X;\n", "cmdArrowDown", configRemoteArray[5]);
      myFile.printf("%s = 0x%04X;\n", "cmdBack", configRemoteArray[6]);
      myFile.printf("%s = 0x%04X;\n", "cmdOk", configRemoteArray[7]);
      myFile.printf("%s = 0x%04X;\n", "cmdSrc", configRemoteArray[8]);
      myFile.printf("%s = 0x%04X;\n", "cmdMute", configRemoteArray[9]);
      myFile.printf("%s = 0x%04X;\n", "cmdAud", configRemoteArray[10]);
      myFile.printf("%s = 0x%04X;\n", "cmdDirect", configRemoteArray[11]);
      myFile.printf("%s = 0x%04X;\n", "cmdBankMinus", configRemoteArray[12]);
      myFile.printf("%s = 0x%04X;\n", "cmdBankPlus", configRemoteArray[13]);
      myFile.printf("%s = 0x%04X;\n", "cmdRed", configRemoteArray[14]);
      myFile.printf("%s = 0x%04X;\n", "cmdGreen", configRemoteArray[15]);
      myFile.printf("%s = 0x%04X;\n", "cmdKey0", configRemoteArray[16]);
      myFile.printf("%s = 0x%04X;\n", "cmdKey1", configRemoteArray[17]);
      myFile.printf("%s = 0x%04X;\n", "cmdKey2", configRemoteArray[18]);
      myFile.printf("%s = 0x%04X;\n", "cmdKey3", configRemoteArray[19]);
      myFile.printf("%s = 0x%04X;\n", "cmdKey4", configRemoteArray[20]);
      myFile.printf("%s = 0x%04X;\n", "cmdKey5", configRemoteArray[21]);
      myFile.printf("%s = 0x%04X;\n", "cmdKey6", configRemoteArray[22]);
      myFile.printf("%s = 0x%04X;\n", "cmdKey7", configRemoteArray[23]);
      myFile.printf("%s = 0x%04X;\n", "cmdKey8", configRemoteArray[24]);
      myFile.printf("%s = 0x%04X;\n", "cmdKey9", configRemoteArray[25]);
      myFile.close();
      Serial.println("RC - Config, remote.txt created");
    }
    else
	  {
		  Serial.println("RC - Config, error while creating remote.txt file");
	  }
  }
}




void readRemoteControlConfig() 
{

  Serial.println("RC - Config, reading the IR remote configuration file remote.txt");
  
    // Tworzymy nazwę pliku
  String fileName = String("/remote.txt");

  // We check if the file exists
  if (!STORAGE.exists(fileName)) 
  {
    Serial.println("RC - Config, error, IR remote configuration file remote.txt does not exist.");
    configIrExist = false;
    return;
  }

  // We open the file in read mode
  File configRemoteFile = STORAGE.open(fileName, FILE_READ);
  if (!configRemoteFile)  // if the file is missing then...
  {
    Serial.println("RC - Config, error, unable to open IR remote configuration file");
    configIrExist = false;
    return;
  }
  // We go to the appropriate line of the file
  configIrExist = true;
  int currentLine = 0;
  String configValue = "";
  while (configRemoteFile.available())
  {
    String line = configRemoteFile.readStringUntil(';'); //('\n');
    int lineStart = line.indexOf("0x") + 2;  // We are looking for the place where the value of the variable begins
    if ((lineStart != -1)) //&& (currentLine != 0)) // We skip the first line where the file description is
	  {
      configValue = line.substring(lineStart, lineStart + 5);  // We extract the address and command
      configValue.trim();                      // We remove whitespace at the beginning and end
      Serial.print(" Code read: " + String(currentLine) + " value:");
      Serial.print(configValue + " wartosc ConfigArray: ");
      configRemoteArray[currentLine] = strtol(configValue.c_str(), NULL, 16);
      Serial.println(configRemoteArray[currentLine], HEX);
    }
    currentLine++;
  }
  //Serial.print("We close the config file at the currentLine value:");
  //Serial.println(currentLine);
  configRemoteFile.close();  // We close the file after reading the remote control codes
}

void assignRemoteCodes()
{
  Serial.println("RC - Config, # assignRemoteCodes #");
  Serial.print("RC - Config, remote control configuration file exists, configIrExist: ");
  Serial.println(configIrExist);

  //if ((noSDcard == false) && (configIrExist == true))
  if (configIrExist == true)  
  {
  Serial.println("RC - Config, file exists, assigns values ​​from Remote.txt file");
  rcCmdVolumeUp = configRemoteArray[0];    // Loudness +
  rcCmdVolumeDown = configRemoteArray[1];  // Volume -
  rcCmdArrowRight = configRemoteArray[2];  // right arrow - next station
  rcCmdArrowLeft = configRemoteArray[3];   // left arrow - previous station 
  rcCmdArrowUp = configRemoteArray[4];     // up arrow - list of stations one step up
  rcCmdArrowDown = configRemoteArray[5];   // down arrow - list of stations step down
  rcCmdBack = configRemoteArray[6]; 	     // Back button
  rcCmdOk = configRemoteArray[7];          // Ent button - confirm station
  rcCmdSrc = configRemoteArray[8];         // Switching the radio or player source
  rcCmdMute = configRemoteArray[9];        // Sound mute
  rcCmdAud = configRemoteArray[10];        // Sound equalizer
  rcCmdDirect = configRemoteArray[11];     // Screen brightness, two modes 1/16 or full brightness    
  rcCmdBankMinus = configRemoteArray[12];  // Display bank selection
  rcCmdBankPlus = configRemoteArray[13];   // Display bank selection
  rcCmdRed = configRemoteArray[14];        // Toggles SD card bank loading - GitHub server in the bank menu
  rcCmdPower = configRemoteArray[14];      // POWER BUTTON WITH RED PHONE CODE
  rcCmdGreen = configRemoteArray[15];      // VU off, VU mode 1, VU mode 2, clock
  rcCmdKey0 = configRemoteArray[16];       // Button "0"
  rcCmdKey1 = configRemoteArray[17];       // Button "1"
  rcCmdKey2 = configRemoteArray[18];       // Button "2"
  rcCmdKey3 = configRemoteArray[19];       // Button "3"
  rcCmdKey4 = configRemoteArray[20];       // Button "4"
  rcCmdKey5 = configRemoteArray[21];       // Button "5"
  rcCmdKey6 = configRemoteArray[22];       // Button "6"
  rcCmdKey7 = configRemoteArray[23];       // Button "7"
  rcCmdKey8 = configRemoteArray[24];       // Button "8"
  rcCmdKey9 = configRemoteArray[25];       // Button "9"
  }
  
  //else if ((noSDcard == true) || (configIrExist == false)) // If there is no SD card, we assign standard values ​​for the Kenwood RC-406 remote control
  else if (configIrExist == false) // If there is no SD card, we assign standard values ​​for the Kenwood RC-406 remote control
  {
    Serial.println("IR Config - NO remote configuration, assigns default values");
    rcCmdVolumeUp = 0xB914;   // Loudness +
    rcCmdVolumeDown = 0xB915; // Volume -
    rcCmdArrowRight = 0xB90B; // right arrow - next station
    rcCmdArrowLeft = 0xB90A;  // left arrow - previous station  
    rcCmdArrowUp = 0xB987;    // up arrow - list of stations one step up
    rcCmdArrowDown = 0xB986;  // down arrow - list of stations step down
    rcCmdBack = 0xB985;	   	  // Back button
    rcCmdOk = 0xB90E;         // Ent button - confirm station
    rcCmdSrc = 0xB913;        // Switching the radio or player source
    rcCmdMute = 0xB916;       // Sound mute
    rcCmdAud = 0xB917;        // Sound equalizer
    rcCmdDirect = 0xB90F;     // Screen brightness, two modes 1/16 or full brightness    
    rcCmdBankMinus = 0xB90C;  // Display bank selection
    rcCmdBankPlus = 0xB90D;   // Display bank selection
    rcCmdRed = 0xB988;        // Red handset button - currently power
    rcCmdPower = 0xB988;      // Power button - currently missing on the remote control
    rcCmdGreen = 0xB992;      // Green handset button - currently sleep
    rcCmdKey0 = 0xB900;       // Button "0"
    rcCmdKey1 = 0xB901;       // Button "1"
    rcCmdKey2 = 0xB902;       // Button "2"
    rcCmdKey3 = 0xB903;       // Button "3"
    rcCmdKey4 = 0xB904;       // Button "4"
    rcCmdKey5 = 0xB905;       // Button "5"
    rcCmdKey6 = 0xB906;       // Button "6"
    rcCmdKey7 = 0xB907;       // Button "7"
    rcCmdKey8 = 0xB908;       // Button "8"
    rcCmdKey9 = 0xB909;       // Button "9"

    configRemoteArray[0]  = rcCmdVolumeUp;
    configRemoteArray[1]  = rcCmdVolumeDown;
    configRemoteArray[2]  = rcCmdArrowRight;
    configRemoteArray[3]  = rcCmdArrowLeft;
    configRemoteArray[4]  = rcCmdArrowUp;
    configRemoteArray[5]  = rcCmdArrowDown;
    configRemoteArray[6]  = rcCmdBack;
    configRemoteArray[7]  = rcCmdOk;
    configRemoteArray[8]  = rcCmdSrc;
    configRemoteArray[9]  = rcCmdMute;
    configRemoteArray[10] = rcCmdAud;
    configRemoteArray[11] = rcCmdDirect;
    configRemoteArray[12] = rcCmdBankMinus;
    configRemoteArray[13] = rcCmdBankPlus;
    configRemoteArray[14] = rcCmdRed;
    configRemoteArray[15] = rcCmdGreen;
    configRemoteArray[16] = rcCmdKey0;
    configRemoteArray[17] = rcCmdKey1;
    configRemoteArray[18] = rcCmdKey2;
    configRemoteArray[19] = rcCmdKey3;
    configRemoteArray[20] = rcCmdKey4;
    configRemoteArray[21] = rcCmdKey5;
    configRemoteArray[22] = rcCmdKey6;
    configRemoteArray[23] = rcCmdKey7;
    configRemoteArray[24] = rcCmdKey8;
    configRemoteArray[25] = rcCmdKey9;
    
  }
  /*
  Serial.println("IR Config - Assigned Values:");
  for (uint8_t i = 0; i < CONFIGREMOTE_COUNT; i++)
  {
    Serial.print(i);
    Serial.print(". ");
    Serial.println(configRemoteArray[i], HEX);
  }
  */
}

void listFiles(String path, String &html) 
{
    File root = STORAGE.open(path);
    if (!root) {
        Serial.println("Cannot open SD card!");
        return;
    }

    // Browsing files in a directory
    File file = root.openNextFile();
    while (file) {
        if (!file.isDirectory()) {
            html += "<tr>";

            html += "<td>" + String(file.name()) + "</td>";
            html += "<td>" + String(file.size()) + "</td>";
            html += "<td>" + String("\n");
            
            // Deleting a file
            html += "<form action=\"/delete\" method=\"POST\" style=\"display:inline; margin-right: 35px;\">";
            html += "<input type=\"hidden\" name=\"filename\" value=\"" + String(file.name()) + "\">";
            html += "<input type=\"submit\" value=\"Delete\">";
            html += "</form>" + String("\n");

            // File preview
            html += "<form action=\"/view\" method=\"GET\" style=\"display:inline;\">";
            html += "<input type=\"hidden\" name=\"filename\" value=\"" + String(file.name()) + "\">";
            html += "<input type=\"submit\" value=\"View\">";
            html += "</form>" + String("\n");
            
            // Edit file
            html += "<form action=\"/edit\" method=\"GET\" style=\"display:inline;\">";
            html += "<input type=\"hidden\" name=\"filename\" value=\"" + String(file.name()) + "\">";
            html += "<input type=\"submit\" value=\"Edit\">";
            html += "</form>" + String("\n");            
            
            // Downloading a file
            html += "<form action=\"/download\" method=\"GET\" style=\"display:inline;\">";
            html += "<input type=\"hidden\" name=\"filename\" value=\"" + String(file.name()) + "\">";
            html += "<input type=\"submit\" value=\"Download\">";
            html += "</form>" + String("\n");

            html += "</td>";
            html += "</tr>" + String("\n");
        }
        file = root.openNextFile();
    }
    root.close();
}



void saveTimezone(String timezone) 
{
  if (STORAGE.exists("/timezone.txt")) 
  {
    Serial.println("debug SD -> The file timezone.txt already exists.");

    // Otwórz plik do zapisu i nadpisz aktualną wartość flitrów equalizera
    myFile = STORAGE.open("/timezone.txt", FILE_WRITE);
    if (myFile) 
    {
      myFile.println(timezone);
      myFile.close();
      Serial.println("debug SD -> Updating timezone.txt on SD card");
    } 
    else 
    {
      Serial.println("debug SD -> Error opening timezone.txt file");
    }
  } 
  else 
  {
    Serial.println("debug SD -> The file timezone.txt does not exist. Creating...");

    // Utwórz plik i zapisz w nim aktualną wartość głośności
    myFile = STORAGE.open("/timezone.txt", FILE_WRITE);
    if (myFile) 
    {
      myFile.println(timezone);
      myFile.close();
      Serial.println("debug SD -> Created and saved timezone.txt on SD card");
    } 
    else 
    {
      Serial.println("debug SD -> Error creating timezone.txt file");
    }
  }
}

void readTimezone() 
{
  // Sprawdzamy czy plik  istnieje
  if (STORAGE.exists("/timezone.txt")) 
  {
    File myFile = STORAGE.open("/timezone.txt", FILE_READ);

    // Plik istnieje, ale nie udało się go otworzyć
    if (!myFile) 
    {
      timezone = "CET-1CEST,M3.5.0/2,M10.5.0/3";
      Serial.println("debug SD -> Error in the timezone.txt file, setting the time zone:" + timezone);
      return;
    }

    // Plik udało się otworzyć
    timezone = myFile.readString();
    timezone.trim();
    myFile.close();
    Serial.println("debug SD -> Read timezone.txt, zone:" + timezone);
  } 
  else 
  {
    // Plik nie istnieje ustawiamy wartosc domyslna
    timezone = "CET-1CEST,M3.5.0/2,M10.5.0/3";
    saveTimezone(timezone);
    Serial.println("debug SD -> The timezone.txt file does not exist");
  }
}


//####################################################################################### POWER OFF / SLEEP OPERATION ####################################################################################### //

// ---- This function displays the text in the center of the OLED in large font. -----
void displayCenterBigText(String stringText, int stringText_Y)
{
  displayStartTime = millis(); 
  displayActive = true;
  volumeSet = false;
  bankMenuEnable = false;
  timeDisplay = false;
  
  u8g2.setContrast(displayBrightness); // We set the maximum brightness
  //u8g2.setFont(u8g2_font_fub14_tr);
  u8g2.setFont(u8g2_font_helvB14_tr);
  int stringTextWidth = u8g2.getStrWidth(stringText.c_str()); // Counts the positions to display TEXT in the center
  int stringText_X = (SCREEN_WIDTH - stringTextWidth) / 2;
  u8g2.clearBuffer();
  u8g2.drawStr(stringText_X, stringText_Y, stringText.c_str());     
  u8g2.sendBuffer();
  //Serial.print("debug sleep -> Chart Max Height: ");       
  //Serial.println(u8g2.getMaxCharHeight(stringText.substring(0,1).c_str()));
  Serial.println(u8g2.getMaxCharHeight());
}

void sleepTimer()
{
  if (f_debug_on) {Serial.println("debug sleep -> Sleep timer check");}
  Serial.println("debug sleep -> Sleep timer check");       
  if (f_sleepTimerOn)
  {
    sleepTimerValueCounter--;
    //if (f_debug_on) 
    //{
      Serial.print("debug sleep -> Sleep timer value counter: ");       
      Serial.println(sleepTimerValueCounter);
    //}
    
    if (sleepTimerValueCounter == 0) 
    {   
      f_sleepTimerOn = false;
      displayActive = true;
      displayStartTime = millis();
      timeDisplay = false;
             
      displayCenterBigText("SLEEP",36);
      //Fadeout volume
      
      ir_code = rcCmdPower; // Power off - let's get the remote control code
      bit_count = 32;
      calcNec();          // We convert the remote control code into the full original NEC code

    }
  }
}

void powerOffAnimation() 
{
  int width  = u8g2.getDisplayWidth();
  int height = u8g2.getDisplayHeight();

  // Animation of "closing" the screen from the top and bottom
  for (int i = 0; i < height / 2; i += 2) 
  {
    u8g2.clearBuffer();

    // We draw a rectangle in the center of the screen that shrinks
    int boxHeight = height - 2 * i;
    u8g2.drawBox(0, i, width, boxHeight);

    u8g2.sendBuffer();
    delay(20); // time between animation steps
  }

  // After closing - "fading" of the midline
  for (int t = 0; t < 3; t++) 
  {
    u8g2.clearBuffer();
    u8g2.drawHLine(0, height / 2, width);
    u8g2.sendBuffer();
    delay(50);
  }

  // Complete extinction
  u8g2.clearBuffer();
  u8g2.sendBuffer();
}

void displaySleepTimer()
{
  f_displaySleepTime = true;
  displayActive = true;
  displayStartTime = millis();
  timeDisplay = false;
  //u8g2.clearBuffer();
  //u8g2.setFont(u8g2_font_fub14_tf);
  String stringText ="";
  if (sleepTimerValueCounter != 0) {stringText = SLEEP_STRING + String(sleepTimerValueCounter) + SLEEP_STRING_VAL;}
  else {stringText = String(SLEEP_STRING) + SLEEP_STRING_OFF;}
  displayCenterBigText(stringText,36);  // Text, Y cordinate value
  //u8g2.sendBuffer();
}

void sleepTimerSet()
{
  /* // DEBUG
  Serial.print("debug sleep -> f_displaySleepTime: ");       
  Serial.println(f_displaySleepTime);  

  Serial.print("debug sleep -> f_displaySleepTimeSet: ");       
  Serial.println(f_displaySleepTimeSet); 

  Serial.print("debug sleep -> f_sleepTimerOn: ");       
  Serial.println(f_sleepTimerOn); 

  Serial.print("debug sleep -> sleepTimerValueSet: ");       
  Serial.println(sleepTimerValueSet);

  Serial.print("debug sleep -> sleepTimerValueCounter: ");       
  Serial.println(sleepTimerValueCounter);   
  */
  
  if (f_displaySleepTime)
  {  
    if (f_displaySleepTimeSet)
    {
      sleepTimerValueSet = 0;
      sleepTimerValueCounter = 0;
      f_displaySleepTimeSet = false;
      f_sleepTimerOn = false;
      f_displaySleepTime = false;
    }
    else
    {
      // We count down from MAX with STEP steps
      if (sleepTimerValueSet == 0) {sleepTimerValueSet = sleepTimerValueMax;}
      else {sleepTimerValueSet = sleepTimerValueSet - sleepTimerValueStep;}

      // We count up from 0 with step steps
      //sleepTimerValueSet = sleepTimerValueSet + sleepTimerValueStep;
      if (sleepTimerValueSet > sleepTimerValueMax) {sleepTimerValueSet = 0;}
      sleepTimerValueCounter = sleepTimerValueSet;
        
    }

    if (sleepTimerValueSet !=0)
    {
      f_sleepTimerOn = true;
      timer3.attach(60, sleepTimer);      // We set the sleep function flag and turn on Timer 3
    }  
    else 
    {
      f_sleepTimerOn = false; 
      f_displaySleepTimeSet = false;
      timer3.detach();
    }                                      // We reset the sleep function flag and disconnect Timer 3 so that it does not work in sleep mode.
  }
  else 
  {
    if (f_displaySleepTimeSet == 0) // If the Sleep Timer menu is not displayed and the Timer itself is OFF, pressing it first sets it to ValueMAX instead of displaying the current state, i.e. OFF.
    {
      // We count down from MAX with STEP steps
        if (sleepTimerValueSet == 0) {sleepTimerValueSet = sleepTimerValueMax;}
        else {sleepTimerValueSet = sleepTimerValueSet - sleepTimerValueStep;}

        // We count up from 0 with step steps
        //sleepTimerValueSet = sleepTimerValueSet + sleepTimerValueStep;
        if (sleepTimerValueSet > sleepTimerValueMax) {sleepTimerValueSet = 0;}
        sleepTimerValueCounter = sleepTimerValueSet;

        if (sleepTimerValueSet !=0)
      {
        f_sleepTimerOn = true;
        timer3.attach(60, sleepTimer);      // We set the sleep function flag and turn on Timer 3
      }
      else 
      {
        f_sleepTimerOn = false; 
        f_displaySleepTimeSet = false;
        timer3.detach();
      }
    }    
  }

  displaySleepTimer();                   // Message handling on OLED
}

void powerOffClock()
{
  //u8g2.clearBuffer();
  u8g2.setPowerSave(0);
  u8g2.setContrast(dimmerSleepDisplayBrightness);
  u8g2.setDrawColor(1);  // Ustaw kolor na biały

  // A structure that stores time information
  struct tm timeinfo;

  if (!getLocalTime(&timeinfo, 500)) 
  {
    // Display time fetch failure message
	  Serial.println("debug time powerOffClock -> Couldn't get time");
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_7Segments_26x42_mn);
    u8g2.drawStr(40, 52, "--:--");
    u8g2.sendBuffer();
    return;  // Terminate function when time failed
  }
      
  
  seconds2nextMinute = 60 - timeinfo.tm_sec;
  micros2nextMinute = seconds2nextMinute * 1000000ULL - (esp_timer_get_time() % 1000000ULL);
  
  // ---------- SAFEGUARD ----------
  if (micros2nextMinute < 1000) micros2nextMinute = 1000;
  // -----------------------------------



  //showDots = (timeinfo.tm_sec % 2 == 0); // Even second = show colon

  // Convert hour, minute and second to string in "HH:MM:SS" format
  char timeString[9];  // A buffer that stores the time in text form
  u8g2.setFont(u8g2_font_7Segments_26x42_mn);
  snprintf(timeString, sizeof(timeString), "%2d:%02d", timeinfo.tm_hour, timeinfo.tm_min);
  u8g2.clearBuffer();
  u8g2.drawStr(53, 52, timeString);
  u8g2.sendBuffer();
}

void powerOff()
{
  // We turn off the IR LED - together with the Standby LED
  //delay(30); 
  //digitalWrite(IR_LED, LOW);
  
  
  // ---- POWER OFF INSTRUCTION DISPLAYED IN THE CENTER ----
  displayCenterBigText("POWER OFF",36); // Text, position Y
    
  // ---- WE ARE CLOSING THE ENTIRE AUDIO FACILITY ----
  ws.closeAll();
  if (!volumeMute && f_volumeFadeOn) {volumeFadeOut(volumeSleepFadeOutTime);}
  audio.setVolume(0);
  delay(500);
  audio.stopSong();
  delay(500);
  
  // ---- SAVE LAST STATION NUMBER, BANK NUMBER, VOLUME LEVEL if the saveAlways function is disabled ----
  if (!f_saveVolumeStationAlways) {saveVolumeOnSD(); saveStationOnSD();}
  
  
  // ---- POWER OFF ANIMATION, if enabled ----
  if (f_powerOffAnimation) {Serial.println("debug Power -> PowerOff Animation"); powerOffAnimation(); u8g2.clearBuffer();}

  // ---- CLOCK IN POWER OFF MODE first triggered ntp sync target----
  if (f_displayPowerOffClock)
  {
    u8g2.setPowerSave(0);  // If the clock is on, we do not switch the OLED to power save mode
    Serial.println("debug Power -> Clock ON");
    powerOffClock();
  } 
  // Turn off the screen if the clock should be turned off
  else 
  {
    u8g2.setPowerSave(1);
  } 
  
  delay(25);
  
  // ----- WE RESET THE SLEEP TIMER -----
  sleepTimerValueSet = 0;
  sleepTimerValueCounter = 0;
  f_displaySleepTimeSet = false;
  f_sleepTimerOn = false;
  f_displaySleepTime = false;
  timer3.detach();
  timer4.detach();

  f_powerOff = true; 
  Serial.println("debug Power -> I put ESP to sleep, power off");

  // -------------- WE TURN OFF THE PERIPHERALS ------------------
  WiFi.mode(WIFI_OFF);
  btStop();
  // -------------- Turn off the LED on the ESP32 module ------------------
  pinMode(TX, OUTPUT);
  pinMode(RX, OUTPUT);
  digitalWrite(TX, HIGH);
  digitalWrite(RX, HIGH);
  
  Serial.end();
    
  // ---- WE SET THE BUTTONS ON THE ENCODERS TO POWER ON -----
  
  pinMode(SW_PIN2, INPUT_PULLUP); // Encoder 2 SW pin
  
  #ifdef SW_POWER
    pinMode(SW_POWER, INPUT_PULLUP);
  #endif
  
  #ifdef twoEncoders
    pinMode(SW_PIN1, INPUT_PULLUP);
  #endif

  // --------LED STANDBY config ----------------
  //pinMode(STANDBY_LED, OUTPUT);  
  
  // ---------------- USTAWIAMY WAKEUP ----------------    
  #ifdef twoEncoders
  gpio_wakeup_enable((gpio_num_t)SW_PIN1, GPIO_INTR_LOW_LEVEL);
  #endif
    
  #ifdef SW_POWER
  gpio_wakeup_enable((gpio_num_t)SW_POWER, GPIO_INTR_LOW_LEVEL);
  #endif
  
  gpio_wakeup_enable((gpio_num_t)SW_PIN2, GPIO_INTR_LOW_LEVEL);

  //gpio_wakeup_enable((gpio_num_t)recv_pin, GPIO_INTR_LOW_LEVEL);
  
  esp_sleep_enable_gpio_wakeup();
  
  while (f_powerOff)
  {
    // ---------------- IR STATE RESET ----------------
    bit_count = 0;
    ir_code = 0;
    detachInterrupt(digitalPinToInterrupt(recv_pin)); // WE TURN OFF THE INTERRUPTION
    delay(10);

    // ---------------- DEACTIVATE PERIPHERALS and ACTIVATE CLOCK (if enabled) ----------------
    if (f_displayPowerOffClock) 
    {
      powerOffClock();
      esp_sleep_enable_timer_wakeup(micros2nextMinute); 
    }
    else 
    {
      u8g2.setPowerSave(1);
    }
      
    // ---------------- WE SET A WAKEUP ----------------    
    gpio_wakeup_enable((gpio_num_t)recv_pin, GPIO_INTR_LOW_LEVEL);

    // ---------------- WE SET THE LED STANDBY ----------------
    #ifdef TAS5805
      digitalWrite(STANDBY_LED, LOW); // For TAS this is the #PWDN pin driven low
    #else
    digitalWrite(STANDBY_LED, HIGH); // For LED off in Power ON mode and on in Standby mode
    //digitalWrite(STANDBY_LED, LOW); // For LED on in Power ON mode
    #endif

    
    // ---------------- SLEEP ----------------
    delay(10);
    esp_light_sleep_start();

    // ------------- WE WILL WAKE UP HERE ----------------

    // If the timer woke us up, we update the seger and go back to the beginning of the while loop
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    if (cause == ESP_SLEEP_WAKEUP_TIMER)
    {
      if (f_displayPowerOffClock) {powerOffClock();}
      delay(5);
      continue;   // goes back to while – ESP goes back to sleep
    }

    // IR interrupt configuration
    attachInterrupt(digitalPinToInterrupt(recv_pin), pulseISR, CHANGE);
    delay(100); // stabilizacja
                
    // IR analysis
    ir_code = reverse_bits(ir_code, 32);
    uint8_t CMD  = (ir_code >> 16) & 0xFF;
    uint8_t ADDR = ir_code & 0xFF;
    ir_code = (ADDR << 8) | CMD;
  
  
    // POWER UP condition
    #ifdef twoEncoders
    bool POWER_UP =
        (bit_count == 32 && ir_code == rcCmdPower && f_powerOff) ||
        #ifdef SW_POWER 
          (digitalRead(SW_POWER) == LOW) ||
        #endif
        (digitalRead(SW_PIN1) == LOW) ||
        (digitalRead(SW_PIN2) == LOW);
    #else
    bool POWER_UP =
        (bit_count == 32 && ir_code == rcCmdPower && f_powerOff) ||
        #ifdef SW_POWER 
          (digitalRead(SW_POWER) == LOW) ||
        #endif
        (digitalRead(SW_PIN2) == LOW);
    #endif    

    // ------------------ WE WILL RISE after the code has been correctly recognized (or not, else) -----------------
    if (POWER_UP)
    {
      detachInterrupt(digitalPinToInterrupt(recv_pin));
      esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
    
      f_powerOff = false;
      displayActive = true;

      u8g2.setPowerSave(0);
      displayCenterBigText("POWER ON", 36);
      
      // ------------ LED STANDBY -----------
      //pinMode(STANDBY_LED, OUTPUT);
      //digitalWrite(STANDBY_LED, HIGH); // For LED on in Power ON mode
      digitalWrite(STANDBY_LED, LOW); // For LED off in Power ON mode and on in Standby mode
      
      delay(800);

      // Hard restart
      REG_WRITE(RTC_CNTL_OPTIONS0_REG, RTC_CNTL_SW_SYS_RST);
      break;
    }

    // No, it's not POWER_UP -> we disconnect the IR interrupt
    detachInterrupt(digitalPinToInterrupt(recv_pin));
  }
}

// ---- LOADING THE STARTUP LOGO if the file exists ----
bool loadXBM(const char* filename) 
{
    File logo = STORAGE.open(filename, FILE_READ);
    if (!logo) return false;
  
    if (logo.size() == 0) {
        logo.close();
        return false;
    }

    int index = 0;

    while (logo.available() && index < LOGO_SIZE) 
    {
        char c = logo.read();

        if (c == '0' && logo.peek() == 'x') 
        {
            logo.read(); // skip 'x'
            
            char hex1 = logo.read();
            char hex2 = logo.read();

            char hexStr[3] = {hex1, hex2, 0};
            logo_bits[index++] = strtol(hexStr, NULL, 16);
        }
    }

    logo.close();
  
    if (index < LOGO_SIZE) return false;

    return true;
}

void startOta()
{
  timeDisplay = false;
  displayActive = true;
  fwupd = true;
  
  ws.closeAll();
  audio.stopSong();
  delay(250);
  if (!f_saveVolumeStationAlways){saveStationOnSD(); saveVolumeOnSD();}
  
  for (int i = 0; i < 3; i++) 
  {
    u8g2.clearBuffer();
    delay(25);
  }        
  unsigned long now = millis();
  u8g2.clearBuffer();
  u8g2.setDrawColor(1);
  u8g2.setFont(spleen6x12PL);     
  u8g2.setCursor(5, 12); u8g2.print("Evo Radio, OTA Firwmare Update");
  u8g2.sendBuffer();
}

void setPreset(uint8_t preset)
{
  displayStartTime = millis();
  
  displayActive = true;
  volumeSet = false;
  bankMenuEnable = false;
  timeDisplay = false;

   
  Serial.print("debug Presets -> Speed ​​Dial - Station:");
  Serial.println(preset);
  
  if (preset == 0) {station_nr = 10;} else {station_nr = preset;}

  #ifdef IR_LED
    irLedBlink(); 
  #endif

  changeStation();
  clearFlags();  // We clear all the presence flags in the various menus
  displayRadio();
  u8g2.sendBuffer();
}

//LED flashes when a valid IR code is received
void irLedBlink()
{
  digitalWrite(IR_LED, IR_LED_ON);
  delay(30); 
  digitalWrite(IR_LED, IR_LED_OFF);
} 

/*---------------------  IR REMOTE FUNCTION / IR remote control support in NEC code ---------------------*/ 
void handleRemote()
{
  if (bit_count == 32) // we check whether we have read the full 32 bits of the NEC IR code in the interrupt
  {
    if (ir_code != 0) // we check whether the ir_code variable is not equal to 0
    {
      
      detachInterrupt(recv_pin);            // We unbind the interrupt
      
      // ---- IR LED activity simulation ----
      #ifdef IR_LED
        irLedBlink(); 
      #endif
      
      Serial.print("debug IR -> Code NEC OK:");
      Serial.print(ir_code, HEX);
      ir_code = reverse_bits(ir_code,32);   // Bit rotation - changing the order from LSB-MSB to MSB-LSB
      Serial.print("  MSB-LSB: ");
      Serial.print(ir_code, HEX);
    
      uint8_t CMD = (ir_code >> 16) & 0xFF; // Second byte (address inversion)
      uint8_t ADDR = ir_code & 0xFF;        // Fourth byte (command inversion)
      
      Serial.print("  ADR:");
      Serial.print(ADDR, HEX);
      Serial.print(" CMD:");
      Serial.println(CMD, HEX);
      ir_code = ADDR << 8 | CMD;      // We combine ADDR and CMD into one variable 0x ADR CMD

      // Info o przebiegach czasowytch kodu pilota IR
      Serial.print("debug IR -> puls 9ms:"); 
      Serial.print(pulse_duration_9ms);
      Serial.print("  4.5ms:");
      Serial.print(pulse_duration_4_5ms);
      Serial.print("  1690us:");
      Serial.print(pulse_duration_1690us);
      Serial.print("  560us:");
      Serial.println(pulse_duration_560us);


      fwupd = false;        // We clear the OTA update flag if it was set
      //displayActive = true; // If we receive the code from the remote control, activate the display and turn off the OLED dimming
      //displayDimmer(0); // If we receive the code from the remote control, we turn off the dimming of the OLED display
      displayPowerSave(0);
      
      // Learning the remote control - invoking a WebSocket
      if (remoteConfig) 
      {
        wsIrCode(ir_code, ADDR, CMD);
        ir_code = 0;
        bit_count = 0;
      }

      if (ir_code == rcCmdVolumeUp)  { volumeUp(); }         // Volume up button
      else if (ir_code == rcCmdVolumeDown) { volumeDown(); } // Volume down button
      else if (ir_code == rcCmdArrowRight) // right arrow - next station, bank or equalizer settings
      {  
        if (bankMenuEnable == true)
        {
          bank_nr++;
          if (bank_nr > bank_nr_max) 
          {
            bank_nr = 1;
          }
        bankMenuDisplay();
        }
        else if (equalizerMenuEnable == true)
        {
          if (toneSelect == 1) {toneHiValue++;}
          if (toneSelect == 2) {toneMidValue++;}
          if (toneSelect == 3) {toneLowValue++;}
          if (toneSelect == 4) {balanceValue++;}
          
          if (toneHiValue > 12) {toneHiValue = 12;}
          if (toneMidValue > 12) {toneMidValue = 12;}
          if (toneLowValue > 12) {toneLowValue = 12;}
          if (balanceValue > 16) {balanceValue = 16;}
          displayEqualizer();
        }     
        else if (listedStations == true) // Fast forward 5 stations
        {
          timeDisplay = false;
          displayActive = true;
          displayStartTime = millis();    
          station_nr = currentSelection + 1;

          station_nr = station_nr + 5;
          for (int i = 0; i < 5; i++) {scrollDown();}
    
          if (station_nr > stationsCount) {station_nr = station_nr - stationsCount; } //Protection to scroll only through the number of stations in stationCount
          
          displayStations();
        }
        else
        {
          station_nr++;
          //if (station_nr > stationsCount) { station_nr = stationsCount; }
          if (station_nr > stationsCount) { station_nr = 1; } // Scrolling the list of stations in a loop, after reaching the last station of the bank, scroll to the first one.
          changeStation();
          displayRadio();
          u8g2.sendBuffer();
        }
      }
      else if (ir_code == rcCmdArrowLeft) // left arrow - previous station, bank or equalizer settings
      {  
        if (bankMenuEnable == true)
        {
          bank_nr--;
          if (bank_nr < 1) 
          {
            bank_nr = bank_nr_max;
          }
        bankMenuDisplay();  
        }
        else if (equalizerMenuEnable == true)
        {
          if (toneSelect == 1) {toneHiValue--;}
          if (toneSelect == 2) {toneMidValue--;}
          if (toneSelect == 3) {toneLowValue--;}
          if (toneSelect == 4) {balanceValue--;}
          
          if (toneHiValue < -12)  {toneHiValue = -12;}
          if (toneMidValue < -12) {toneMidValue = -12;}
          if (toneLowValue < -12) {toneLowValue = -12;}
          if (balanceValue < -16) {balanceValue = -16;}
         
          displayEqualizer();
        }     
        else if (listedStations == true)
        {
          timeDisplay = false;
          displayActive = true;
          displayStartTime = millis();    
          station_nr = currentSelection + 1;

          station_nr = station_nr - 5;
          
          for (int i = 0; i < 5; i++) {scrollUp();}
      
          //Protection to scroll only through the number of stations in stationCount
          if (station_nr > stationsCount) { station_nr = (stationsCount - (255 - station_nr)) - 1 ;} 
          
          // If we are at station no. 5, to avoid station_nr = 0 we scroll to the last one
          if (station_nr == 0) {station_nr = stationsCount;} 

          displayStations();
        }      
        else
        {        
          station_nr--;
          //if (station_nr < 1) { station_nr = 1; }
          if (station_nr < 1) { station_nr = stationsCount; } // Scrolling the list of stations in a loop, after reaching the last station of the bank, scroll to the first one.
          changeStation();
          displayRadio();
          u8g2.sendBuffer();
        }
      }
      else if ((ir_code == rcCmdArrowUp) && (volumeSet == false) && (equalizerMenuEnable == true))
      {
        toneSelect--;
        if (toneSelect < 1){toneSelect = 1;}
        displayEqualizer();
      }
      else if ((ir_code == rcCmdArrowUp) && (equalizerMenuEnable == false))// Przycisk w góre
      {  
        if ((volumeSet == true) && (volumeBufferValue != volumeValue))
        {
          if (f_saveVolumeStationAlways) {saveVolumeOnSD();}
          volumeSet = false;
        }
        
        timeDisplay = false;
        displayActive = true;
        displayStartTime = millis();    
        station_nr = currentSelection + 1;

        if (listedStations == true) {station_nr--; scrollUp();} //station_nr-- only if already displaying station list;
        else
        {        
          currentSelection = station_nr - 1; // We restore the highlighting of the currently playing station
          
          if (currentSelection >= 0)
          {
            if (currentSelection < firstVisibleLine) // if the current meaning is less than the first displayed line
            {
              firstVisibleLine = currentSelection;
            }
          } 
          else 
          {  // If value 0 is reached, go to the highest value
            if (currentSelection = maxSelection())
            {
            firstVisibleLine = currentSelection - maxVisibleLines + 1;  // Set the first visible line to the highest
            }
          }   
        }
        
        if (station_nr < 1) { station_nr = stationsCount; } // if we reach the beginning of the station list, we scroll to the end
        
        
       
       
      displayStations();
      }
      else if ((ir_code == rcCmdArrowDown) && (volumeSet == false) && (equalizerMenuEnable == true))
      {
        toneSelect++;
        if (toneSelect > 4){toneSelect = 4;}
        displayEqualizer();
      }
      else if ((ir_code == rcCmdArrowDown) && (equalizerMenuEnable == false)) // Down button
      {  
        if ((volumeSet == true) && (volumeBufferValue != volumeValue))
        {
          if (f_saveVolumeStationAlways) {saveVolumeOnSD();}
          volumeSet = false;
        }
        
        timeDisplay = false;
        displayActive = true;
        displayStartTime = millis();     
        station_nr = currentSelection + 1;
        
        //station_nr++;
        if (listedStations == true) {station_nr++; scrollDown(); } //station_nr++ only if already displaying station list;
        else
        {        
          currentSelection = station_nr - 1; // We restore the highlighting of the currently playing station

          if (currentSelection >= 0)
          {
            if (currentSelection < firstVisibleLine) // if the current meaning is less than the first displayed line
            {
              firstVisibleLine = currentSelection;
            }
          } 
          else 
          {  // If value 0 is reached, go to the highest value
            if (currentSelection = maxSelection())
            {
            firstVisibleLine = currentSelection - maxVisibleLines + 1;  // Set the first visible line to the highest
            }
          }
             
        }
        
        if (station_nr > stationsCount) 
	      {
          station_nr = 1;//stationsCount;
        }
        
        //Serial.println(station_nr);

         
        displayStations();
      }    
      else if (ir_code == rcCmdOk)
      {
        if (bankMenuEnable == true)
        {
          station_nr = 1;
          fetchStationsFromServer(); // We load stations from a card or server
          bankMenuEnable = false;
        }  
        if (equalizerMenuEnable == true) { saveEqualizerOnSD();}    // saving equalizer settings
        //if (volumeSet == true) { saveVolumeOnSD();}                 // saving volume settings after pressing OK, disabled so that you can switch to www stations without waiting
        //if ((equalizerMenuEnable == false) && (volumeSet == false)) // If we did not save the equalizer and volume, we call the following functions
        if ((equalizerMenuEnable == false)) // if we didn't save the equalizer 
        {
          if ((!urlPlaying) || (listedStations)) {changeStation();}
          if (rcInputDigitsMenuEnable) {changeStation();}
          else if (urlPlaying) {webUrlStationPlay();}

          clearFlags();                                             // We clear all the presence flags in the various menus
          displayRadio();
          u8g2.sendBuffer();
        }
        equalizerMenuEnable = false; // We clear the equalizer setting flag
        volumeSet = false; // We clear the volume setting flag
      } 
      else if (ir_code == rcCmdKey0) {if (f_Presets && !bankMenuEnable) {setPreset(0);} else {rcInputKey(0);}}
      else if (ir_code == rcCmdKey1) {if (f_Presets && !bankMenuEnable) {setPreset(1) ;} else {rcInputKey(1);}}     
      else if (ir_code == rcCmdKey2) {if (f_Presets && !bankMenuEnable) {setPreset(2) ;} else {rcInputKey(2);}}     
      else if (ir_code == rcCmdKey3) {if (f_Presets && !bankMenuEnable) {setPreset(3) ;} else {rcInputKey(3);}}
      else if (ir_code == rcCmdKey4) {if (f_Presets && !bankMenuEnable) {setPreset(4) ;} else {rcInputKey(4);}}
      else if (ir_code == rcCmdKey5) {if (f_Presets && !bankMenuEnable) {setPreset(5) ;} else {rcInputKey(5);}}
      else if (ir_code == rcCmdKey6) {if (f_Presets && !bankMenuEnable) {setPreset(6) ;} else {rcInputKey(6);}}  
      else if (ir_code == rcCmdKey7) {if (f_Presets && !bankMenuEnable) {setPreset(7) ;} else {rcInputKey(7);}}
      else if (ir_code == rcCmdKey8) {if (f_Presets && !bankMenuEnable) {setPreset(8) ;} else {rcInputKey(8);}}
      else if (ir_code == rcCmdKey9) {if (f_Presets && !bankMenuEnable) {setPreset(9) ;} else {rcInputKey(9);}}
      else if (ir_code == rcCmdBack) 
      {   
        //f_simpleMode3 = !f_simpleMode3;
        displayDimmer(0);
        clearFlags();   // We reset all flags
        displayRadio(); // We load the radio screen
        u8g2.sendBuffer(); // We send the buffer to the display
        currentSelection = station_nr - 1; // We restore the highlighting of the currently playing station
      }
      else if (ir_code == rcCmdMute) 
      {
        volumeMute = !volumeMute;
        if (volumeMute == true)
        {
          audio.setVolume(0);   
        }
        else if (volumeMute == false)
        {
          audio.setVolume(volumeValue);   
        }
        displayRadio();
        //wsVolumeChange(volumeValue);
        wsVolumeChange();
      }
      else if (ir_code == rcCmdDirect) // Direct button -> Menu Bank - GitHub update, Menu Equalizer - reset values, Radio Display - screen dimming function
      {
        if ((bankMenuEnable == true) && (equalizerMenuEnable == false))// the flag can only be changed while in the bank selection menu
        { 
          bankNetworkUpdate = !bankNetworkUpdate; // changing the flag whether we update the bank from the network or SD card
          bankMenuDisplay(); 
        }
        if ((bankMenuEnable == false) && (equalizerMenuEnable == true))
        {
          toneHiValue = 0;
          toneMidValue = 0;
          toneLowValue = 0;
          balanceValue = 0;    
          displayEqualizer();
        }
        if ((bankMenuEnable == false) && (equalizerMenuEnable == false) && (volumeSet == false))
        { 
          if (displayMode == 4) // If we are in Display Mode 4, i.e. large VU display, we can enable the audio print buffer.
          {
            debugAudioBuffor=!debugAudioBuffor;
          }
          else
          {
            displayDimmer(!displayDimmerActive); // Dimmer OLED
            Serial.println("Display Dimmer rcCmd Direct enabled");
          }
        }
      }      
      else if (ir_code == rcCmdSrc) 
      {
        displayMode++;
        if (displayMode > displayModeMax) {displayMode = 0;}

        // Option to change colors in mode4 without adding a new mode              
        //if ((displayMode > displayModeMax) & (reversColorMode4)) {displayMode = displayModeMax; reversColorMode4=!reversColorMode4;}
        //else if ((displayMode > displayModeMax) & (!reversColorMode4)) {displayMode = 0; reversColorMode4=!reversColorMode4;}
        
        displayRadio();
        clearFlags();
        ActionNeedUpdateTime = true;
      }
      else if (ir_code == rcCmdRed)   {powerOff();}
      else if (ir_code == rcCmdPower) {powerOff();}
      else if (ir_code == rcCmdGreen) if (listedStations) {voiceTime();} else {sleepTimerSet();}   
      else if (ir_code == rcCmdBankMinus) 
      {
        /*
        if (bankMenuEnable == true) 
        { 
          bank_nr--; 
          if (bank_nr < 1) {bank_nr = bank_nr_max;}
        }
        */       
        bankMenuDisplay();
      }
      else if (ir_code == rcCmdBankPlus) 
      {
        f_Presets = !f_Presets;
        if (f_Presets) displayCenterBigText("PRESETS ON",36); else displayCenterBigText("PRESETS OFF",36);
        /*
        if (bankMenuEnable == true)
        {
          bank_nr++;
          if (bank_nr > bank_nr_max) {bank_nr = 1;}
        }       
        bankMenuDisplay();
        */       
      }     
      else if (ir_code == rcCmdAud) {displayEqualizer();}
      else { Serial.println("Another button"); }
    }
    else
    {
      Serial.println("Error - NEC remote code is invalid!");
      Serial.print("debug IR -> puls 9ms:");
      Serial.print(pulse_duration_9ms);
      Serial.print("  4.5ms:");
      Serial.print(pulse_duration_4_5ms);
      Serial.print("  1690us:");
      Serial.print(pulse_duration_1690us);
      Serial.print("  560us:");
      Serial.println(pulse_duration_560us);    
    }
    
    // ---- IR LED activity simulation ----
    //#ifdef IR_LED
    //  irLedBlink();  
      //delay(30); digitalWrite(IR_LED, LOW);  // Exception not to turn on the Standby LED when receiving the PowerOFF command
    //#endif
        
    ir_code = 0;
    bit_count = 0;
    attachInterrupt(digitalPinToInterrupt(recv_pin), pulseISR, CHANGE);    

  }

}

#ifdef TAS5805
  void tasWrite(uint8_t reg, uint8_t val) 
  {
    Serial.print("TAS Write: ");
    Serial.print(reg,HEX);
    Serial.print(" ");
    Serial.println(val,HEX);

    Wire.beginTransmission(TAS5805_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
  }
   
 void tasRead(uint8_t reg)
 {
  Wire.beginTransmission(TAS5805_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);   // repeated start

  Wire.requestFrom(TAS5805_ADDR, (uint8_t)1);

  Serial.print("TAS Read: ");
  Serial.print(reg, HEX);
  Serial.print(" = ");

  if (Wire.available()) 
    Serial.println(Wire.read(), HEX);
  else
    Serial.println("ERR");
}

  
  
  
  /*

  void tas5805init()
  {
      tasWrite(0x01, 0x01);      // Software reset
      delay(1);

      tasWrite(0x00, 0x00);      // Book 0
      tasWrite(0x7F, 0x00);      // Page 0

      //tasWrite(0x46, 0x00);      // Disable Digital Auto-Mute
      tasWrite(0x03, 0x03);      // I2S, 24-bit, standard
      //tasWrite(0x28, 0x30);      // Master volume = -48 dB
      tasWrite(0x4c, 0x30);      // Master volume = -48 dB
      
      tasWrite(0x30, 0x01);      // Exit Hi-Z
      delay(10);

      tasWrite(0x02, 0x00);      // PLAY
  }
  */
    void tas5805init()
  {
      tasWrite(0x01, 0x01);      // Software reset
      delay(1);

      tasWrite(0x00, 0x00);      // Book 0
      tasWrite(0x7F, 0x00);      // Page 0
      tasWrite(0x03, 0x02);     // switch to High-Z (should be after reset as well)

      tasWrite(0x30, 0x01);      // SDOUT - pre-proccessing, DSP input
      delay(10);
      
      tasWrite(0x02, 0x00);      // RES 000 - 768k 0 0-BTL 00-BD Mode

      tasWrite(0x28, 0x50);      // 1001 BCK Ratio 256FS/ 00 autodedect  0000 FS_mode Autio

      tasWrite(0x33, 0x02);

      tasWrite(0x4c, 0x30);      // Master volume = -48 dB

      tasWrite(0x03, 0x03);      // PLAY
      delay(5);

      //tasWrite(0x28, 0x30);      // Master volume = -48 dB
  }

  void tas5805vol(uint8_t volume)
  {
    tasWrite(0x4c, volume);
  }
#endif

//####################################################################################### SETUP ####################################################################################### //

void setup()

{
    // --------STANDBY LED for HIGH status in Power ON mode ----------------
  pinMode(STANDBY_LED, OUTPUT);
  timer4.attach(0.5, updateStandbyLED);  // Set timer4 to trigger LED flashing to inform about start and initialization of the radio

  // Initialize serial communication (Serial)
  Serial.begin(115200);
  delay(600);
  
  uint64_t chipid = ESP.getEfuseMac();
  Serial.println("");
  Serial.println("------------------ START of Evo Web Radio --------------------");
  Serial.println("-                                                            -");
  Serial.printf("--------- ESP32 SN: %04X%08X,  FW Ver.: %s ---------\n",(uint16_t)(chipid >> 32), (uint32_t)chipid, softwareRev);
  Serial.println("-         FW Config  use SD:" + String(useSD) + ",  Use Two Encoders:" + String(use2encoders) + "           -");
  Serial.println("-                                                            -");
  Serial.println("- Code source: https://github.com/dzikakuna/ESP32_radio_evo3 -");
  Serial.println("--------------------------------------------------------------");
  
  // ESP32 WiFi Bandwidth Limitation to 20MHz
  //esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT40);
  
  #ifdef USE_SD
  // SPI initialization with new pins for SD card reader
    customSPI.begin(SD_SCLK, SD_MISO, SD_MOSI, SD_CS);  // SCLK = 45, MISO = 21, MOSI = 48, CS = 47
  #endif
  
  //pinMode(SD_SCLK, OUTPUT);
  //digitalWrite(SD_SCLK, LOW);

  psramData = (unsigned char *)ps_malloc(PSRAM_lenght * sizeof(unsigned char));

  if (psramInit()) {
    Serial.println("PSRAM memory initialized correctly.");
    Serial.print("Available PSRAM memory: ");
    Serial.println(ESP.getPsramSize());
    Serial.print("Free PSRAM memory: ");
    Serial.println(ESP.getFreePsram());


  } else {
    Serial.println("debug-- PSRAM Memory ERROR");
  }

  wifiManager.setHostname(hostname);

  //WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  //WiFi.setBandwith(WIFI_BW_HT20);
  //esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT20);




  EEPROM.begin(128);

  // ----------------- TAS5805 Initialization -----------------
  #ifdef TAS5805
    Wire.begin(I2C_DATA, I2C_CLK);   // I2C initialization
    Wire.setClock(400000);           // I2C 400kHz
    delay(200);
    tas5805init();
    delay(100);
  #endif

  // ----------------- ENCODER 2 basic -----------------
  pinMode(CLK_PIN2, INPUT_PULLUP);           // Configure encoder 2 pins as inputs
  pinMode(DT_PIN2, INPUT_PULLUP); 
  pinMode(SW_PIN2, INPUT_PULLUP);            // Initializing encoder buttons as input
  prev_CLK_state2 = digitalRead(CLK_PIN2);   // Read the initial state of the encoder CLK pin
  

  // ----------------- ENCODER 1 additional (if enabled) -----------------
  #ifdef twoEncoders
    pinMode(CLK_PIN1, INPUT_PULLUP);           // Configure encoder 1 pins as inputs
    pinMode(DT_PIN1, INPUT_PULLUP);
    pinMode(SW_PIN1, INPUT_PULLUP);           // Initializing encoder buttons as input
    prev_CLK_state1 = digitalRead(CLK_PIN1);  // Read the initial state of the encoder CLK pin
  #endif  

  // ----------------- SW_POWER additional (if enabled) -----------------
  #ifdef SW_POWER
    pinMode(SW_POWER, INPUT_PULLUP);
  #endif


  // ----------------- SD card CS LED - clone (if enabled) -----------------
  #ifdef SD_LED
    pinMode(SD_LED, OUTPUT);
    //PIN, CLONE PIN, INVERTED, EN_INV -Cloning the VirtualSPI (VSPI) controller CS signal to the LED to have information about card activity from the reader on the back of the PCB
    gpio_matrix_out(SD_LED, SPI3_CS0_OUT_IDX, false, false);
  #endif


  // ----------------- IR RECEIVER - Pin Configuration -----------------
  pinMode(recv_pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(recv_pin), pulseISR, CHANGE);


  // ----------------- ADC KEYBOARD - Configuration -----------------
  analogReadResolution(12);                 // Set resolution to 11 bits (range 0-4095)
  analogSetAttenuation(ADC_11db);           // ADC Gain (0dB for 0-3.3V range)

  #ifdef I2S_MCLK
    audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT, I2S_MCLK);  // Configure pinout for I2S audio interface
  #else
    audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);  // Configure pinout for I2S audio interface
  #endif


  Audio::audio_info_callback = my_audio_info; // Assigning your own callback function to handle events and audio information
  audio.setVolume(0);
  

  // Initialize display SPI interface
  SPI.begin(SPI_SCK_OLED, SPI_MISO_OLED, SPI_MOSI_OLED);
  #ifdef LOWNOISESPI // Controlling GPIO output drivers to reduce EMI
    gpio_set_drive_capability((gpio_num_t)SPI_MOSI_OLED, GPIO_DRIVE_CAP_2);
    gpio_set_drive_capability((gpio_num_t)SPI_SCK_OLED, GPIO_DRIVE_CAP_2);
    gpio_set_drive_capability((gpio_num_t)CS_OLED, GPIO_DRIVE_CAP_1);
    gpio_set_drive_capability((gpio_num_t)DC_OLED, GPIO_DRIVE_CAP_1);
  #endif
  SPI.setFrequency(2000000);


  // Initialize display and wait 250 milliseconds to turn on
  u8g2.begin();
  delay(250);

  // ----------------- SD CARD / SPIFFS/LittleFS MEMORY - Initialization -----------------
  // Initializing STORAGE
  #ifdef AUTOSTORAGE
    if (!initStorage()) 
    {
      while (1);
    }
  #else
    // ----------------- SD CARD / SPIFFS MEMORY - Initialization -----------------
    if (!STORAGE_BEGIN())
    {
      Serial.println("STORAGE memory initialization error!"); // Information about problems or missing SD card
      noSDcard = true;                                // No SD card flag
    }
    else
    {
      Serial.println("STORAGE memory initialized successfully.");
      noSDcard = false;
    }
  #endif
  delay(50);
  
  // Reading configuration
  readConfig();          
  if (configExist == false) { saveConfig(); readConfig();} // JIf there is no config.txt file, we create it
  

  if ((esp_reset_reason() != ESP_RST_POWERON) || (!f_sleepAfterPowerFail))
  {
    u8g2.clearBuffer();
    if (f_logoSlowBrightness) {u8g2.setContrast(0);}

    if (!loadXBM("/logo.xbm")) 
    {
      Serial.println("No logo in storage memory, loading notes...");
      u8g2.drawXBMP(0, 5, notes_width, notes_height, notes);  // obrazek - nutki
      u8g2.setFont(u8g2_font_fub14_tf);
      u8g2.drawStr(38, 17, "Evo Internet Radio");
      u8g2.sendBuffer();
    }
    else
    {
      
      u8g2.drawXBMP(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, logo_bits); // obrazek - logo z pamieci
      u8g2.sendBuffer();     
      f_logo = 1;
    }   

    // ---- SLOWLY FADE IN THE NOTE LOGO / LOGO UPLOADED ----
    if (f_logoSlowBrightness)
    {
      for (int i = 0; i <= displayBrightness; i++)
        {
          u8g2.setContrast(i);
          //uint8_t corrected = (i * i) / 255;   // gamma ≈ 2.0
          //u8g2.setContrast(corrected);
          delay(10); 
        }
      u8g2.setContrast(displayBrightness); // Additional brightness overload         
    }
     
    u8g2.setFont(spleen6x12PL);
    u8g2.drawStr(208, 62, softwareRev);
    u8g2.sendBuffer();
  }
  
  #ifndef AUTOSTORAGE
  // Information on the display about problems or missing SD card
  if (noSDcard) // the noSDcard flag is set if no card is detected
  {
    u8g2.setFont(spleen6x12PL);
    u8g2.drawStr(5, 62, "Error - Please check SD card");
    u8g2.setDrawColor(0);
    u8g2.drawBox(212, 0, 44, 45);
    u8g2.setDrawColor(1);
    u8g2.drawXBMP(220, 3, 30, 40, sdcard);  // SD card icon
    u8g2.sendBuffer();
  }
  #endif

  u8g2.setFont(spleen6x12PL);
  if ((esp_reset_reason() == ESP_RST_POWERON) && (f_sleepAfterPowerFail)) {u8g2.drawStr(5, 50, "Wakeup after power loss, syncing NTP");}
  if (f_logo) {u8g2.drawStr(5, 62, "Evo Web Radio, connecting...");} else {u8g2.drawStr(5, 62, "Connecting to network...    ");}
  u8g2.sendBuffer();
  //delay(1000); // Let's look at the notes or logo for a longer time :)
  
  #ifdef twoEncoders
    button1.setDebounceTime(50);  // Set the debouncing time for encoder button 2
  #endif

  button2.setDebounceTime(50);  // Set the debouncing time for encoder button 2
  

    
  // Initializing WiFiManager
  wifiManager.setConfigPortalBlocking(false);

  // ---- READING VARIOUS SETTINGS FROM SD CARD/SPIFFS MEMORY ----
  readStationFromSD();       // We read the last saved station and bank from the SD card/EEPROM
  readEqualizerFromSD();     // We read the equalizer filter settings from the SD card
  readVolumeFromSD();        // We read the last volume setting from the SD card/EEPROM
  readAdcConfig();           // Reading ADC keyboard configuration
  readRemoteControlConfig(); // Reading the IR remote configuration
  assignRemoteCodes();       // Assigning IR remote control codes
  readTimezone();

  audio.setVolumeSteps(maxVolume);
  //audio.setVolume(0);                  // Set the volume based on the value of the volumeValue variable in the range 0...21
  
  // ----------------- SCREEN - BRIGHTNESS -----------------
  u8g2.setContrast(displayBrightness); 

  // -------------------- RECOVERY MODE --------------------
  recoveryModeCheck();
  
  previous_bank_nr = bank_nr;  // equalizing the value at radio status to avoid changing bank_nr to 0 after the first menu timeout
  Serial.print("debug1...value bank_nr:");
  Serial.println(bank_nr);
  Serial.print("debug1...value previous_bank_nr:");
  Serial.println(previous_bank_nr);
  Serial.print("debug1...value station_nr:");
  Serial.println(station_nr);

  // Start Wi-Fi setup and connect to a network if necessary
  if (wifiManager.autoConnect("EVO-Radio")) 
  {
    Serial.println("Connected to WiFi network");
    currentIP = WiFi.localIP().toString();  //IP to string conversion
    u8g2.setFont(spleen6x12PL);
    u8g2.drawStr(5, 62, "                               ");  // clearing lines with spaces
    u8g2.sendBuffer();
    u8g2.drawStr(5, 62, "Connected, IP:");  //IP display
    u8g2.drawStr(90, 62, currentIP.c_str());   //IP display
    u8g2.sendBuffer();
    delay(1000);  // wait 1 second before erasing the IP number
    
    if (MDNS.begin(hostname)) { Serial.println("mDNS has launched, adres: " + String(hostname) + ".local w przeglądarce"); MDNS.addService("http", "tcp", 80);}
        
    //configTzTime("CET-1CEST,M3.5.0/2,M10.5.0/3", ntpServer1, ntpServer2);
    configTzTime(timezone.c_str(), ntpServer1, ntpServer2);
    struct tm timeinfo;
    int retry = 0;
    Serial.println("debug RTC -> NTP synchronization for the first time since launch");
    while (!getLocalTime(&timeinfo) && retry < 10) 
    {
      Serial.println("debug RTC -> Waiting for NTP time synchronization...");
      delay(1000);
      retry++;
    }


    timer1.attach(0.5, updateTimerFlag);  // Set the timer to call the updateTimer function
    //timer1.attach(0.7, updateTimerFlag);  // Set the timer to call the updateTimer function
    timer2.attach(1, displayDimmerTimer);
    //timer3.attach(60, sleepTimer);   // Set timer3,

    if ((esp_reset_reason() == ESP_RST_POWERON) && (f_sleepAfterPowerFail)) // We check if the radio turned on after the power cut, if so, we go to sleep when the power returns
    {
      Serial.println("debug PWR -> power OFF after power is restored");
      Serial.print("debug PWR -> Clock function f_displayPowerOffClock: ");
      Serial.println(f_displayPowerOffClock);
      //prePowerOff();
      powerOff();
    }


    uint8_t temp_station_nr = station_nr; // We hide the station read from the SD card for a moment so that the last played station is not changed when loading the bank
    fetchStationsFromServer();            // We load the list of stations from the SD card or GitHub server
    station_nr = temp_station_nr ;        // We restore the number after reading the station
    changeStation();                      // We are setting up the station
    
    // ########################################### WEB Server ######################################################
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
    
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
    {
      String userAgent = request->header("User-Agent");
      String html =""; 
      //html.reserve(48000);  // reserves a buffer (e.g. 24KB)

      if (userAgent.indexOf("Mobile") != -1 || userAgent.indexOf("Android") != -1 || userAgent.indexOf("iPhone") != -1) 
      {
        html = stationBankListHtmlMobile();
      } 
      else //We are on the computer
      {
        html = stationBankListHtmlPC();
      }
      
      String finalhtml = String(index_html) + html;  // We combine the html constant part with the dynamically generated part
      //request->send_P(200, "text/html", finalhtml.c_str());
      //request->send(200, "text/html", finalhtml.c_str());
      request->send(200, "text/html", finalhtml);
    });

    server.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest *request){
          request->send(STORAGE, "/favicon.ico", "image/x-icon");       
    });
    
    server.on("/icon.png", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(STORAGE, "/icon.png", "image/png");       
    });

        server.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest *request){
          request->send(STORAGE, "favicon.ico", "image/x-icon");       
    });
    
    server.on("/icon.png", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(STORAGE, "icon.png", "image/png");       
    });

    server.on("/menu", HTTP_GET, [](AsyncWebServerRequest *request){   
      String html = String(menu_html);
      request->send(200, "text/html", html);
    });
    
    server.on("/firmwareota", HTTP_POST, [](AsyncWebServerRequest *request) 
    {
      request->send(200, "text/plain", "Update done");
    },
    [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) 
    {

      static size_t total = 0;
      static size_t contentLength = 0;
      static unsigned long lastPrint = 0;

      if (index == 0) 
      {
        // Only with the first package
        Serial.printf("OTA Start: %s\n", filename.c_str());
        total = 0;
        contentLength = request->contentLength();  // pełna długość pliku
        lastPrint = millis();

        size_t sketchSpace = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;

        if (contentLength > sketchSpace) 
        {
          Serial.printf("Error: firmware too big (%u > %u bytes)\n", contentLength, sketchSpace);
          return;
        }

        if (!Update.begin(sketchSpace)) 
        {
          Update.printError(Serial);
          return;
        }
      }

      if (Update.write(data, len) != len) 
      {
        Update.printError(Serial);
      } 
      else 
      {
        total += len;
          
        // Display progress bar every 300 ms
        unsigned long now = millis();
        if (now - lastPrint >= 200 || final) 
        {     
          int percent = (total * 100) / contentLength;
          Serial.printf("Progress: %d%% (%u/%u bytes)\n", percent, total, contentLength);
          u8g2.setCursor(5, 24); u8g2.print("File: " + String(filename));
          u8g2.setCursor(5, 36); u8g2.print("Flashing... " + String(total / 1024) + " KB");
          u8g2.sendBuffer();
          lastPrint = now;
        }
      }
      if (final) 
      {
        if (Update.end(true)) 
        {
          request->send(200, "text/plain", "Update done - reset in 3sec");
          Serial.println("Update complete");
          u8g2.setCursor(5, 48); u8g2.print("Completed - reset in 3sec");
          u8g2.sendBuffer();
              
          AsyncWebServerRequest *reqCopy = request;
          reqCopy->onDisconnect([]() 
          {
            delay(3000);
            //ESP.restart();
            REG_WRITE(RTC_CNTL_OPTIONS0_REG, RTC_CNTL_SW_SYS_RST);
          });
        } 
        else 
        {
          Update.printError(Serial);
          request->send(500, "text/plain", "Update failed");
        }
      }
    });

    server.on("/editor", HTTP_GET, [](AsyncWebServerRequest *request)
    {
      request->send(STORAGE, "/editor.html", "text/html"); //"application/octet-stream");
    });

     server.on("/browser", HTTP_GET, [](AsyncWebServerRequest *request)
    {
      request->send(STORAGE, "/browser.html", "text/html"); //"application/octet-stream");
    });

    server.on("/ota", HTTP_GET, [](AsyncWebServerRequest *request)
    {
      
      timeDisplay = false;
      displayActive = true;
      fwupd = true;
           
      ws.closeAll();
      ws.cleanupClients();
      audio.stopSong();

      delay(250);
      if (!f_saveVolumeStationAlways){saveStationOnSD(); saveVolumeOnSD();}
      
      for (int i = 0; i < 3; i++) 
      {
        u8g2.clearBuffer();
        delay(25);
      }        
      unsigned long now = millis();
      u8g2.clearBuffer();
      u8g2.setDrawColor(1);
      u8g2.setFont(spleen6x12PL);     
      u8g2.setCursor(5, 12); u8g2.print("Evo Radio, OTA Firwmare Update");
      u8g2.sendBuffer();

      request->send(200, "text/html", String(ota_html));
      //request->send(SD, "/ota.html", "text/html"); //"application/octet-stream");
    });

    server.on("/page1", HTTP_GET, [](AsyncWebServerRequest *request)
    { // Page for testing purposes loaded from STORAGE (SD card or FS memory)
      request->send(STORAGE, "/page1.html", "text/html"); //"application/octet-stream");
    });

    server.on("/playurl", HTTP_GET, [](AsyncWebServerRequest *request)
    { 
      //String html =""; 
      //String finalhtml = String(urlplay_html);  // We combine the html constant part with the dynamically generated part
      //request->send(200, "text/html", finalhtml);
      request->send(200, "text/html", String(urlplay_html));
      //request->send(STORAGE, "/playurl.html", "text/html");
    });

    server.on("/edit", HTTP_GET, [](AsyncWebServerRequest *request) 
    {
      if (!request->hasParam("filename")) 
      {
          request->send(400, "text/plain", "Missing filename parameter");
          return;
      }

      String filename = request->getParam("filename")->value();
      if (!filename.startsWith("/")) { filename = "/" + filename;}
      File file = STORAGE.open(filename, FILE_READ);
      if (!file) {
          request->send(404, "text/plain", "Cannot open file");
          return;
      }

      String html = "<html><head><title>Edit File</title></head><body>";
      html += "<h2 style='font-size: 1.3rem;'>Editing: " + filename + "</h2>";
      html += "<form method='POST' action='/save'>";
      html += "<input type='hidden' name='filename' value='" + filename + "'>";
      html += "<textarea name='content' rows='100' cols='130'>";

      while (file.available()) 
      {
          html += (char)file.read();
      }

      file.close();

      html += "</textarea><br>";
      html += "<input type='submit' value='Save'>";
      html += "</form>";
      html += "<p><a href='/list'>Back to list</a></p>";
      html += "</body></html>";

      request->send(200, "text/html", html);
    });

    server.on("/save", HTTP_POST, [](AsyncWebServerRequest *request) 
    {
      if (!request->hasParam("filename", true) || !request->hasParam("content", true)) 
      {
        request->send(400, "text/plain", "Brakuje danych");
        return;
      }

      String filename = request->getParam("filename", true)->value();
      String content = request->getParam("content", true)->value();

      File file = STORAGE.open(filename, FILE_WRITE);
      if (!file) 
      {
        request->send(500, "text/plain", "Nie można zapisać pliku");
        return;
      }

      file.print(content);
      file.close();

      request->send(200, "text/html", "<p>File Saved!</p><p><a href='/list'>Back to list</a></p>");
    });

    server.on("/list", HTTP_GET, [](AsyncWebServerRequest *request) 
    {
      String html = String(list_html) + String("\n");
      
      //html += "<body><h2 style='font-size: 1.3rem;'>Evo Web Radio - SD / SPIFFS:</h2>" + String("\n");
      html += "<body><h2 style='font-size: 1.3rem;'>Evo Web Radio - ";
      html += storageTextName;
      html += "</h2>" + String("\n");
      
      //if (useSD) html += "<body><h2 style='font-size: 1.3rem;'>Evo Web Radio - SD card:</h2>" + String("\n");
      //else html += "<body><h2 style='font-size: 1.3rem;'>Evo Web Radio - SPIFFS memory:</h2>" + String("\n");

      html += "<form action=\"/upload\" method=\"POST\" enctype=\"multipart/form-data\">";
      html += "<input type=\"file\" name=\"file\">";
      html += "<input type=\"submit\" value=\"Upload\">";
      html += "</form>";
      html += "<div class=\"columnlist\">" + String("\n");
      html += "<table><tr><th>File</th><th>Size (Bytes)</th><th>Action</th></tr>";

      listFiles("/", html);
      html += "</table></div>";
      html += "<p style='font-size: 0.8rem;'><a href='/menu'>Go Back</a></p>" + String("\n"); 
      html += "</body></html>";     
      request->send(200, "text/html", html);
    });

    server.on("/delete", HTTP_POST, [](AsyncWebServerRequest *request) {
    String filename = "/"; // Dodajemy sciezke do głownego folderu
      if (request->hasParam("filename", true)) {
        filename += request->getParam("filename", true)->value();
        if (STORAGE.remove(filename.c_str())) {
            Serial.println("File deleted: " + filename);
        } else {
            Serial.println("Cannot delete file: " + filename);
        }
      }
      request->redirect("/list"); // We are redirecting to the list page
    });

      server.on("/config", HTTP_GET, [](AsyncWebServerRequest *request) {
        //String html = String(config_html);
        String html = FPSTR(config_html);  // czyta z PROGMEM

        // liczby
        html.replace(F("%D10"), String(vuRiseSpeed));
        html.replace(F("%D11"), String(vuFallSpeed));
        html.replace(F("%D12"), String(dimmerSleepDisplayBrightness));
        
        html.replace(F("%D1"), String(displayBrightness));
        html.replace(F("%D2"), String(dimmerDisplayBrightness));
        html.replace(F("%D3"), String(displayAutoDimmerTime));
        html.replace(F("%D4"), String(vuMeterMode));
        html.replace(F("%D5"), String(encoderFunctionOrder));
        html.replace(F("%D6"), String(displayMode));
        html.replace(F("%D7"), String(vuMeterRefreshTime));
        html.replace(F("%D8"), String(scrollingRefresh));
        html.replace(F("%D9"), String(displayPowerSaveTime));

        //Opcje CheckBox
        html.replace(F("%S11_checked"), maxVolumeExt ? " checked" : "");
        html.replace(F("%S13_checked"), vuPeakHoldOn ? " checked" : "");
        html.replace(F("%S15_checked"), vuSmooth ? " checked" : "");
        html.replace(F("%S17_checked"), stationNameFromStream ? " checked" : "");
        html.replace(F("%S19_checked"), f_displayPowerOffClock ? " checked" : "");
        html.replace(F("%S21_checked"), f_sleepAfterPowerFail ? " checked" : "");
        html.replace(F("%S22_checked"), f_volumeFadeOn ? " checked" : "");
        html.replace(F("%S23_checked"), f_saveVolumeStationAlways ? " checked" : "");
        html.replace(F("%S24_checked"), f_powerOffAnimation ? " checked" : "");
        html.replace(F("%S25_checked"), f_Presets ? " checked" : "");        

        html.replace(F("%S1_checked"), displayAutoDimmerOn ? " checked" : "");
        html.replace(F("%S3_checked"), timeVoiceInfoEveryHour ? " checked" : "");
        html.replace(F("%S5_checked"), vuMeterOn ? " checked" : "");
        html.replace(F("%S7_checked"), adcKeyboardEnabled ? " checked" : "");
        html.replace(F("%S9_checked"), displayPowerSaveEnabled ? " checked" : "");
              

        request->send(200, "text/html", html);
      });

    server.on("/adc", HTTP_GET, [](AsyncWebServerRequest *request) 
    {
      String html = String(adc_html);
      html.replace("%D10", String(keyboardButtonThreshold_Ok).c_str());
      html.replace("%D11", String(keyboardButtonThreshold_BankMenu).c_str()); 
      html.replace("%D12", String(keyboardButtonThreshold_Back).c_str()); 
      html.replace("%D13", String(keyboardButtonThreshold_DisplayMode).c_str()); 
      html.replace("%D14", String(keyboardButtonThreshold_Dimmer).c_str()); 
      html.replace("%D15", String(keyboardButtonThreshold_Mute).c_str()); 
      html.replace("%D16", String(keyboardButtonThresholdTolerance).c_str()); 
      html.replace("%D17", String(keyboardButtonNeutral).c_str());
      html.replace("%D18", String(keyboardSampleDelay).c_str()); 

      html.replace("%D19", String(keyboardButtonThreshold_ArrowLeft).c_str()); 
      html.replace("%D20", String(keyboardButtonThreshold_ArrowRight).c_str()); 
      html.replace("%D21", String(keyboardButtonThreshold_ArrowUp).c_str()); 
      html.replace("%D22", String(keyboardButtonThreshold_ArrowDown).c_str()); 

      html.replace("%D23", String(keyboardButtonThreshold_PresetsOn).c_str()); 
      

      html.replace("%D0", String(keyboardButtonThreshold_0).c_str()); 
      html.replace("%D1", String(keyboardButtonThreshold_1).c_str()); 
      html.replace("%D2", String(keyboardButtonThreshold_2).c_str()); 
      html.replace("%D3", String(keyboardButtonThreshold_3).c_str()); 
      html.replace("%D4", String(keyboardButtonThreshold_4).c_str()); 
      html.replace("%D5", String(keyboardButtonThreshold_5).c_str()); 
      html.replace("%D6", String(keyboardButtonThreshold_6).c_str()); 
      html.replace("%D7", String(keyboardButtonThreshold_7).c_str()); 
      html.replace("%D8", String(keyboardButtonThreshold_8).c_str()); 
      html.replace("%D9", String(keyboardButtonThreshold_9).c_str()); 
      
      request->send(200, "text/html", html);
    });


    //server.on("/remoteconfigupdate", HTTP_POST, handleRemoteConfigUpdate);
    server.on("/remoteconfigupdate", HTTP_POST, [](AsyncWebServerRequest *request)
    {
      if (request->hasParam("cmdVolumeUp", true))   {configRemoteArray[0] = strtol(request->getParam("cmdVolumeUp", true)->value().c_str(), NULL, 16);}
      if (request->hasParam("cmdVolumeDown", true)) {configRemoteArray[1] = strtol(request->getParam("cmdVolumeDown", true)->value().c_str(), NULL, 16);}
      if (request->hasParam("cmdArrowRight", true)) {configRemoteArray[2] = strtol(request->getParam("cmdArrowRight", true)->value().c_str(), NULL, 16);}
      if (request->hasParam("cmdArrowLeft", true))  {configRemoteArray[3] = strtol(request->getParam("cmdArrowLeft", true)->value().c_str(), NULL, 16);}
      if (request->hasParam("cmdArrowUp", true))    {configRemoteArray[4] = strtol(request->getParam("cmdArrowUp", true)->value().c_str(), NULL, 16);}
      if (request->hasParam("cmdArrowDown", true))  {configRemoteArray[5] = strtol(request->getParam("cmdArrowDown", true)->value().c_str(), NULL, 16);}
      if (request->hasParam("cmdBack", true)) {configRemoteArray[6] = strtol(request->getParam("cmdBack", true)->value().c_str(), NULL, 16);}
      if (request->hasParam("cmdOk", true)) {configRemoteArray[7] = strtol(request->getParam("cmdOk", true)->value().c_str(), NULL, 16);}
      if (request->hasParam("cmdSrc", true)) {configRemoteArray[8] = strtol(request->getParam("cmdSrc", true)->value().c_str(), NULL, 16);}
      if (request->hasParam("cmdMute", true)) {configRemoteArray[9] = strtol(request->getParam("cmdMute", true)->value().c_str(), NULL, 16);}
      if (request->hasParam("cmdAud", true)) {configRemoteArray[10] = strtol(request->getParam("cmdAud", true)->value().c_str(), NULL, 16);}
      if (request->hasParam("cmdDirect", true)) {configRemoteArray[11] = strtol(request->getParam("cmdDirect", true)->value().c_str(), NULL, 16);}
      if (request->hasParam("cmdBankMinus", true)) {configRemoteArray[12] = strtol(request->getParam("cmdBankMinus", true)->value().c_str(), NULL, 16);}
      if (request->hasParam("cmdBankPlus", true)) {configRemoteArray[13] = strtol(request->getParam("cmdBankPlus", true)->value().c_str(), NULL, 16);}
      if (request->hasParam("cmdRed", true)) {configRemoteArray[14] = strtol(request->getParam("cmdRed", true)->value().c_str(), NULL, 16);}
      if (request->hasParam("cmdGreen", true)) {configRemoteArray[15] = strtol(request->getParam("cmdGreen", true)->value().c_str(), NULL, 16);}
      if (request->hasParam("cmdKey0", true)) {configRemoteArray[16] = strtol(request->getParam("cmdKey0", true)->value().c_str(), NULL, 16);}
      if (request->hasParam("cmdKey1", true)) {configRemoteArray[17] = strtol(request->getParam("cmdKey1", true)->value().c_str(), NULL, 16);}
      if (request->hasParam("cmdKey2", true)) {configRemoteArray[18] = strtol(request->getParam("cmdKey2", true)->value().c_str(), NULL, 16);}
      if (request->hasParam("cmdKey3", true)) {configRemoteArray[19] = strtol(request->getParam("cmdKey3", true)->value().c_str(), NULL, 16);}
      if (request->hasParam("cmdKey4", true)) {configRemoteArray[20] = strtol(request->getParam("cmdKey4", true)->value().c_str(), NULL, 16);}
      if (request->hasParam("cmdKey5", true)) {configRemoteArray[21] = strtol(request->getParam("cmdKey5", true)->value().c_str(), NULL, 16);}
      if (request->hasParam("cmdKey6", true)) {configRemoteArray[22] = strtol(request->getParam("cmdKey6", true)->value().c_str(), NULL, 16);}
      if (request->hasParam("cmdKey7", true)) {configRemoteArray[23] = strtol(request->getParam("cmdKey7", true)->value().c_str(), NULL, 16);}
      if (request->hasParam("cmdKey8", true)) {configRemoteArray[24] = strtol(request->getParam("cmdKey8", true)->value().c_str(), NULL, 16);}
      if (request->hasParam("cmdKey9", true)) {configRemoteArray[25] = strtol(request->getParam("cmdKey9", true)->value().c_str(), NULL, 16);}

      /*
      for (uint8_t i = 0; i < CONFIGREMOTE_COUNT; i++)
      {
        if (!request->hasArg(remoteMap[i].name)) continue;

        String val = request->arg(remoteMap[i].name);
        val.trim();

        // walidacja HEX 0xFFFF
        if (!val.startsWith("0x")) continue;
        configRemoteArray[i] = strtol(val.c_str(), NULL, 16);
        Serial.printf("%s = 0x%04X\n", remoteMap[i].name, configRemoteArray[i]);
      }
      */
      remoteConfig = false;
      saveRemoteControlConfig();
      readRemoteControlConfig();
      assignRemoteCodes();
      //request->send(200, "text/html", "<h1>Remote Control Config Codes Updated!</h1><a href='/'>Go to Main</a>");
      request->send(200, "text/html", "<!DOCTYPE html><html><head><meta http-equiv='refresh' content='2;url=/'></head><body><h1>Remote Control Config Codes Updated!</h1></body></html>");
      
    });


    server.on("/remoteconfig", HTTP_GET, [](AsyncWebServerRequest *request) 
    {
      String html = String(remoteconfig_html);
      
     // for (int i = 25; i >= 0; i--)
     // {
     //   html.replace("%D" + String(i), String(configRemoteArray[i]).c_str());
     // }
      
      for (int i = 25; i >= 0; i--)
      {
        char buf[7];                    // "0xFFFF" + \0
        sprintf(buf, "0x%04X", configRemoteArray[i]);
        html.replace("%D" + String(i), buf);
      }
      
      wsIrCodeClear();
      request->send(200, "text/html", html);
      //request->send(STORAGE, "/remoteconfig.html", "text/html"); //"application/octet-stream");
    });


    server.on("/configadc", HTTP_POST, [](AsyncWebServerRequest *request) 
    {
      if (request->hasParam("keyboardButtonThreshold_Ok", true)) {
        keyboardButtonThreshold_Ok = request->getParam("keyboardButtonThreshold_Ok", true)->value().toInt();
      }
      if (request->hasParam("keyboardButtonThreshold_BankMenu", true)) {
        keyboardButtonThreshold_BankMenu = request->getParam("keyboardButtonThreshold_BankMenu", true)->value().toInt();
      }
      if (request->hasParam("keyboardButtonThreshold_Back", true)) {
        keyboardButtonThreshold_Back = request->getParam("keyboardButtonThreshold_Back", true)->value().toInt();
      }
      if (request->hasParam("keyboardButtonThreshold_DisplayMode", true)) {
        keyboardButtonThreshold_DisplayMode = request->getParam("keyboardButtonThreshold_DisplayMode", true)->value().toInt();
      }
      if (request->hasParam("keyboardButtonThreshold_Dimmer", true)) {
        keyboardButtonThreshold_Dimmer = request->getParam("keyboardButtonThreshold_Dimmer", true)->value().toInt();
      }
      if (request->hasParam("keyboardButtonThreshold_Mute", true)) {
        keyboardButtonThreshold_Mute = request->getParam("keyboardButtonThreshold_Mute", true)->value().toInt();
      }
      if (request->hasParam("keyboardButtonThresholdTolerance", true)) {
        keyboardButtonThresholdTolerance = request->getParam("keyboardButtonThresholdTolerance", true)->value().toInt();
      }
      if (request->hasParam("keyboardButtonNeutral", true)) {
        keyboardButtonNeutral = request->getParam("keyboardButtonNeutral", true)->value().toInt();
      }
      if (request->hasParam("keyboardSampleDelay", true)) {
        keyboardSampleDelay = request->getParam("keyboardSampleDelay", true)->value().toInt();
      } 
      if (request->hasParam("keyboardButtonThreshold_0", true)) {
        keyboardButtonThreshold_0 = request->getParam("keyboardButtonThreshold_0", true)->value().toInt();
      } 
      if (request->hasParam("keyboardButtonThreshold_1", true)) {
        keyboardButtonThreshold_1 = request->getParam("keyboardButtonThreshold_1", true)->value().toInt();
      } 
      if (request->hasParam("keyboardButtonThreshold_2", true)) {
        keyboardButtonThreshold_2 = request->getParam("keyboardButtonThreshold_2", true)->value().toInt();
      } 
      if (request->hasParam("keyboardButtonThreshold_3", true)) {
        keyboardButtonThreshold_3 = request->getParam("keyboardButtonThreshold_3", true)->value().toInt();
      } 
      if (request->hasParam("keyboardButtonThreshold_4", true)) {
        keyboardButtonThreshold_4 = request->getParam("keyboardButtonThreshold_4", true)->value().toInt();
      } 
      if (request->hasParam("keyboardButtonThreshold_5", true)) {
        keyboardButtonThreshold_5 = request->getParam("keyboardButtonThreshold_5", true)->value().toInt();
      } 
      if (request->hasParam("keyboardButtonThreshold_6", true)) {
        keyboardButtonThreshold_6 = request->getParam("keyboardButtonThreshold_6", true)->value().toInt();
      } 
      if (request->hasParam("keyboardButtonThreshold_7", true)) {
        keyboardButtonThreshold_7 = request->getParam("keyboardButtonThreshold_7", true)->value().toInt();
      } 
      if (request->hasParam("keyboardButtonThreshold_8", true)) {
        keyboardButtonThreshold_8 = request->getParam("keyboardButtonThreshold_8", true)->value().toInt();
      } 
      if (request->hasParam("keyboardButtonThreshold_9", true)) {
        keyboardButtonThreshold_9 = request->getParam("keyboardButtonThreshold_9", true)->value().toInt();
      }
      // --- Arrow Left ---
      if (request->hasParam("keyboardButtonThreshold_ArrowLeft", true)) {
        keyboardButtonThreshold_ArrowLeft = request->getParam("keyboardButtonThreshold_ArrowLeft", true)->value().toInt();
      }
      // --- Arrow Right ---
      if (request->hasParam("keyboardButtonThreshold_ArrowRight", true)) {
        keyboardButtonThreshold_ArrowRight = request->getParam("keyboardButtonThreshold_ArrowRight", true)->value().toInt();
      }
      // --- Arrow Up ---
      if (request->hasParam("keyboardButtonThreshold_ArrowUp", true)) {
        keyboardButtonThreshold_ArrowUp = request->getParam("keyboardButtonThreshold_ArrowUp", true)->value().toInt();
      }
      // --- Arrow Down ---
      if (request->hasParam("keyboardButtonThreshold_ArrowDown", true)) {
         keyboardButtonThreshold_ArrowDown = request->getParam("keyboardButtonThreshold_ArrowDown", true)->value().toInt();
      }
      // Preset ON / OFF
      if (request->hasParam("keyboardButtonThreshold_PresetsOn", true)) {
         keyboardButtonThreshold_PresetsOn = request->getParam("keyboardButtonThreshold_PresetsOn", true)->value().toInt();
      }

      

      request->send(200, "text/html", "<h1>ADC Keyboard Thresholds Updated!</h1><a href='/menu'>Go Back</a>");
      
      saveAdcConfig(); 
       
      //Refreshing the OLED screen after configuration changes
      ir_code = rcCmdBack; // We pretend to be the remote control code Back
      bit_count = 32;
      calcNec();          // We convert the remote control code into the full original NEC code
    });

    server.on("/configupdate", HTTP_POST, [](AsyncWebServerRequest *request) 
    {
      if (request->hasParam("displayBrightness", true)) { displayBrightness = request->getParam("displayBrightness", true)->value().toInt(); }
      if (request->hasParam("dimmerDisplayBrightness", true)) {dimmerDisplayBrightness = request->getParam("dimmerDisplayBrightness", true)->value().toInt();}
      if (request->hasParam("dimmerSleepDisplayBrightness", true)) {dimmerSleepDisplayBrightness = request->getParam("dimmerSleepDisplayBrightness", true)->value().toInt();}
      if (request->hasParam("displayAutoDimmerTime", true)) {displayAutoDimmerTime = request->getParam("displayAutoDimmerTime", true)->value().toInt();}

      //  CHECKBOXES – ALWAYS 0 or 1
      //displayAutoDimmerOn = request->hasParam("displayAutoDimmerOn", true) ? true : false;
      displayAutoDimmerOn        = request->hasParam("displayAutoDimmerOn", true);
      timeVoiceInfoEveryHour     = request->hasParam("timeVoiceInfoEveryHour", true);
      vuMeterOn                  = request->hasParam("vuMeterOn", true);
      adcKeyboardEnabled         = request->hasParam("adcKeyboardEnabled", true);
      displayPowerSaveEnabled    = request->hasParam("displayPowerSaveEnabled", true);
      maxVolumeExt               = request->hasParam("maxVolumeExt", true);
      vuPeakHoldOn               = request->hasParam("vuPeakHoldOn", true);
      vuSmooth                   = request->hasParam("vuSmooth", true);
      f_displayPowerOffClock     = request->hasParam("f_displayPowerOffClock", true);
      f_sleepAfterPowerFail      = request->hasParam("f_sleepAfterPowerFail", true);
      f_volumeFadeOn             = request->hasParam("f_volumeFadeOn", true);
      f_saveVolumeStationAlways  = request->hasParam("f_saveVolumeStationAlways", true);
      f_powerOffAnimation        = request->hasParam("f_powerOffAnimation", true);
      f_Presets                  = request->hasParam("f_Presets", true);

      // If the parameter exists and the checkbox was checked, then TRUE
      // If it is not there and the checkbox was not checked, then FALSE

      if (request->hasParam("vuMeterMode", true)) {vuMeterMode = request->getParam("vuMeterMode", true)->value().toInt();}
      if (request->hasParam("encoderFunctionOrder", true)) {encoderFunctionOrder = request->getParam("encoderFunctionOrder", true)->value().toInt();}
      if (request->hasParam("displayMode", true)) {displayMode = request->getParam("displayMode", true)->value().toInt();}
      if (request->hasParam("vuMeterRefreshTime", true)) {vuMeterRefreshTime = request->getParam("vuMeterRefreshTime", true)->value().toInt();}
      if (request->hasParam("scrollingRefresh", true)) {scrollingRefresh = request->getParam("scrollingRefresh", true)->value().toInt();}
      if (request->hasParam("displayPowerSaveTime", true)) {displayPowerSaveTime = request->getParam("displayPowerSaveTime", true)->value().toInt();}
      if (request->hasParam("vuRiseSpeed", true)) {vuRiseSpeed = request->getParam("vuRiseSpeed", true)->value().toInt();}
      if (request->hasParam("vuFallSpeed", true)) {vuFallSpeed = request->getParam("vuFallSpeed", true)->value().toInt();}

      saveConfig();
      readConfig();
      //clearFlags();
      
      //Refresh
      ir_code = rcCmdBack; // We pretend to be the remote control code Back
      bit_count = 32;
      calcNec();          // We convert the remote control code into the full original NEC code

      //request->send(200, "text/html","<h1>Config Updated!</h1><a href='/menu'>Go Back</a>");
      request->send(200, "text/html","<!DOCTYPE html><html><head><meta http-equiv='refresh' content='2;url=/'></head><body><h1>Config Updated!</h1></body></html>");
    });
    
    server.on("/toggleAdcDebug", HTTP_POST, [](AsyncWebServerRequest *request) 
    {
      // Switching values
      u8g2.clearBuffer();
      displayActive = true;
      displayStartTime = millis();
      debugKeyboard = !debugKeyboard;
      //adcKeyboardEnabled = !adcKeyboardEnabled;
      Serial.print("ADC ON/OFF value measurement:");
      Serial.println(debugKeyboard);
      request->send(200, "text/plain", debugKeyboard ? "1" : "0"); // Sending current value
      //debugKeyboard = !debugKeyboard;
      
      if (debugKeyboard == 0)
      {
        ir_code = rcCmdBack; // We assign the command code from the remote control
        bit_count = 32; // we set the information that we have the full NEC code for analysis 
        calcNec();  // we convert the remote control code to the original full NEC code 
      }
        
    });

    server.on("/toggleRemoteConfig", HTTP_POST, [](AsyncWebServerRequest *request) 
    {
      displayActive = true;
      displayStartTime = millis();
      remoteConfig = !remoteConfig;
      if (remoteConfig) {displayCenterBigText("RC Learning Mode ON",46);} else {displayCenterBigText("RC Learning Mode OFF",46);}
      
      Serial.print("RC Remote Control Learning - ON/OFF:");
      Serial.println(remoteConfig);
      //if (remoteConfig) wsIrCode(0x0000,0,0); else 
      wsIrCodeClear();
      request->send(200, "text/plain", remoteConfig ? "1" : "0"); // Wysyłanie aktualnej wartości       
    });
    
    server.on("/mute", HTTP_POST, [](AsyncWebServerRequest *request) 
    {
      ir_code = rcCmdMute; // We assign the command code from the remote control
      bit_count = 32; // we set the information that we have the full NEC code for analysis
      calcNec();  // we convert the remote control code to the original full NEC code 
      request->send(204);       // no response (204)    
    });
    

    server.on("/view", HTTP_GET, [](AsyncWebServerRequest *request) 
    {
      String filename = "/";
      if (request->hasParam("filename")) 
      {
        filename += request->getParam("filename")->value();
        //String fullPath = "/" + filename;

        File file = STORAGE.open(filename);
        if (file) 
        {
            String content = "<html><body><h1>File content: " + filename + "</h1><pre>";
            while (file.available()) {
                content += (char)file.read();
            }
            content += "</pre><a href=\"/list\">Back to list</a></body></html>";
            request->send(200, "text/html", content);
            file.close();
          } 
          else 
          {
            request->send(404, "text/plain", "File not found");
          }
        } 
        else 
        {
        request->send(400, "text/plain", "No file name");
      }
    });
    // Format view web ver2 for external HTML Bank Editor
    server.on("/viewweb2", HTTP_GET, [](AsyncWebServerRequest *request)  
    {
      String filename = "/";
      if (request->hasParam("filename")) 
      {
        filename += request->getParam("filename")->value();
        //String fullPath = "/" + filename;

        File file = STORAGE.open(filename);
        if (file) 
        {
            String content; //= "<html><body>";
            while (file.available()) {
                //line = file.readStringUntil('\n');
                content += file.readStringUntil('\n') + String("\n");
            }
            //content +="</body></html>";
            request->send(200, "text/html", content);
            file.close();
          } 
          else 
          {
            request->send(404, "text/plain", "File not found");
          }
        } 
        else 
        {
        request->send(400, "text/plain", "No file name");
      }
    });
  
    /*
    server.on("/editordata", HTTP_GET, [](AsyncWebServerRequest *request) 
    {
      if (!request->hasParam("filename")) {
          request->send(400, "text/plain", "No file name");
          return;
      }

      String filename = request->getParam("filename")->value();
      File file = STORAGE.open(filename);
      if (!file) {
          request->send(404, "text/plain", "File not found");
          return;
      }

      // We create a text streaming response
      AsyncWebServerResponse *response = request->beginResponseStream("text/plain");
      AsyncWebServerResponseStream *stream = static_cast<AsyncWebServerResponseStream*>(response);

      while (file.available()) {
          String line = file.readStringUntil('\n');
          stream->print(line + "\n");  // tu działa print
      }

      file.close();
      request->send(response);
    });
    */
    server.on("/editordata", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (!request->hasParam("filename")) {
        request->send(400, "text/plain", "No file name");
        return;
    }

    String filename = "/" + request->getParam("filename")->value();

    File file = STORAGE.open(filename);
    if (!file) {
        request->send(404, "text/plain", "File not found");
        return;
    }

    // Tworzymy strumień odpowiedzi typu text/plain
    AsyncResponseStream *response = request->beginResponseStream("text/plain");

    while (file.available()) {
        String line = file.readStringUntil('\n');
        response->print(line + "\n");  // działa tylko z AsyncResponseStream
    }

    file.close();
    request->send(response);
});



    server.on("/download", HTTP_ANY, [](AsyncWebServerRequest *request) {
      if (request->hasParam("filename")) {
        String filename = request->getParam("filename")->value();
        String fullPath = "/" + filename;
        //String fullPath = filename;

        Serial.println("Downloading a file: " + fullPath);

        File file = STORAGE.open(fullPath);
        //Serial.println("SD: " + SD + "/" + filename);

        if (file) 
        {
            if (file.size() > 0) 
            {         
              request->send(STORAGE, fullPath, "application/octet-stream", true); //"application/octet-stream");"text/plain"
              file.close();
              Serial.println("File sent: " + filename);
            } 
            else 
            {
              request->send(404, "text/plain", "Plik jest pusty");
            }
        } 
        else 
        {
          Serial.println("File not found: " + fullPath);
          request->send(404, "text/plain", "File not found");
        }
      } 
      else 
      {
        Serial.println("Missing filename parameter");
        request->send(400, "text/plain", "No file name");
      }
    });
 
    server.on("/upload", HTTP_POST, [](AsyncWebServerRequest *request) {
        request->send(200, "text/plain", "File Uploaded!");
      },
      //nullptr,
      [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
      if (!filename.startsWith("/")) {
        filename = "/" + filename;
      }

      Serial.print("Upload File Name: ");
      Serial.println(filename);

      // Open file on SD card
      static File file; // Use a static variable to open a file only once
      if (index == 0) {
          file = STORAGE.open(filename, FILE_WRITE);
          if (!file) {
              Serial.println("Failed to open file for writing");
              request->send(500, "text/plain", "Failed to open file for writing");
              return;
          }
      }

      // Save data to file
      size_t written = file.write(data, len);
      if (written != len) {
          Serial.println("Error writing data to file");
          file.close();
          request->send(500, "text/plain", "Error writing data to file");
          return;
      }

      // If this is the last fragment, close the file and send the response to the client
      if (final) {
          file.close();
          Serial.println("File upload completed successfully.");
          request->send(200, "text/plain", "File upload successful");
      } else {
          Serial.println("Received chunk of size: " + String(len));
      }
    });

    server.on("/update", HTTP_GET, [] (AsyncWebServerRequest *request) {
      if (request->hasParam("url")) 
      {
        String inputMessage = request->getParam("url")->value();
        url2play = inputMessage.c_str();
        urlToPlay = true;
        Serial.println("Received URL: " + inputMessage);
        request->send(200, "text/plain", "URL received");
      } 
      else 
      {
        request->send(400, "text/plain", "Missing URL parameter");
      }  
    });

    server.on("/info", HTTP_GET, [](AsyncWebServerRequest *request) {
      String html = String(info_html);
      String signal = "";
      uint64_t chipid = ESP.getEfuseMac();
      char chipStr[20];
      sprintf(chipStr, "%04X%08X", (uint16_t)(chipid >> 32), (uint32_t)chipid);
      
      html.replace("%D1", String(softwareRev).c_str()); 
      html.replace("%D2", String(hostname).c_str());                
      html.replace("%D3", String(WiFi.RSSI()).c_str()); 
      html.replace("%D4", String(wifiManager.getWiFiSSID()).c_str()); 
      html.replace("%D5", currentIP.c_str()); 
      html.replace("%D6", WiFi.macAddress().c_str()); 
      html.replace("%D7", String(storageTextName).c_str()); 
      //if (useSD) html.replace("%D7", String("SD").c_str()); 
      //else html.replace("%D7", String("SPIFFS").c_str()); 
      html.replace("%D0", chipStr); 
      //f_callInfo = true;
      
      request->send(200, "text/html", html);

    });

    server.on("/timezone", HTTP_GET, [](AsyncWebServerRequest *request){
      //request->send(STORAGE, "/timezone.html", "text/html");
      request->send(200, "text/html", String(timezone_html));
    });

    server.on("/getTZ", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send(200, "text/plain", timezone);
    });

    server.on("/setTZ", HTTP_GET, [](AsyncWebServerRequest *request){
      if (!request->hasParam("value")) 
      {
        request->send(400, "text/plain", "Missing timezone value");
        return;
      }

      String newTZ = request->getParam("value")->value();
      newTZ.trim();
      timezone = newTZ;

      saveTimezone(newTZ);

      //configTzTime(timezone.c_str(), "pool.ntp.org", "time.nist.gov");
      configTzTime(timezone.c_str(), ntpServer1, ntpServer2);
      
      request->send(200, "text/plain", "OK");
    });

    ws.onEvent(onWsEvent);
    server.addHandler(&ws);
    server.begin();
    currentSelection = station_nr - 1; // we set the stations on the list to the one currently playing when the radio starts
    firstVisibleLine = currentSelection + 1; // the first visible line is the playing station at the start
    if (currentSelection + 1 >= stationsCount - 1) 
    {
      firstVisibleLine = currentSelection - 3;
    }  
    displayRadio();
    ActionNeedUpdateTime = true;
    
    timer4.detach();
    switchOffstartupLED();
    //fadeInVolume
    // Limiting Wi-Fi power, disabling sleep
    //WiFi.setTxPower(WIFI_POWER_8_5dBm);
    //esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT20);
    //WiFi.bandwidth(WIFI_BW_HT20);
  } 
  else 
  {
    timer4.attach(0.2, updateStandbyLED);
    Serial.println("No WiFi connection");  // If there is no Wi-Fi connection - send a message to the serial
    u8g2.clearBuffer();
    u8g2.setFont(spleen6x12PL);
    u8g2.drawStr(5, 13, "No network connection");  // If there is no Wi-Fi connection - display a message on the OLED display
    u8g2.drawStr(5, 26, "Connect to WiFi: ESP Internet Radio");
    u8g2.drawStr(5, 39, "Open web page http://192.168.4.1");
    u8g2.sendBuffer();
    while(true)
    { 
      wifiManager.process(); 
    

      /*---------------------  IR REMOTE FUNCTION in NO Wi-Fi mode ---------------------*/ 
      if (bit_count == 32) // swe check whether we have read the full 32 bits of the NEC IR code in the interrupt
      {
        if (ir_code != 0) // we check whether the ir_code variable is not equal to 0
        {
          
          detachInterrupt(recv_pin);            // We unbind the interrupt
          Serial.print("debug IR -> Code NEC OK:");
          Serial.print(ir_code, HEX);
          ir_code = reverse_bits(ir_code,32);   // Bit rotation - changing the order from LSB-MSB to MSB-LSB
          Serial.print("  MSB-LSB: ");
          Serial.print(ir_code, HEX);
        
          uint8_t CMD = (ir_code >> 16) & 0xFF; // Second byte (address inversion)
          uint8_t ADDR = ir_code & 0xFF;        // Fourth byte (command inversion)
          
          Serial.print("  ADR:");
          Serial.print(ADDR, HEX);
          Serial.print(" CMD:");
          Serial.println(CMD, HEX);
          ir_code = ADDR << 8 | CMD;      // We combine ADDR and CMD into one variable 0x ADR CMD

          // Info about the timing of the IR remote control code
          Serial.print("debug IR -> puls 9ms:"); 
          Serial.print(pulse_duration_9ms);
          Serial.print("  4.5ms:");
          Serial.print(pulse_duration_4_5ms);
          Serial.print("  1690us:");
          Serial.print(pulse_duration_1690us);
          Serial.print("  560us:");
          Serial.println(pulse_duration_560us);

          fwupd = false;        // We clear the OTA update flag if it was set
          displayPowerSave(0);
          
          if (ir_code == rcCmdPower) 
          {     
            //prePowerOff();
            powerOff();
          }
          else if (ir_code == rcCmdBack)
          {
            displayCenterBigText("RESTART",36);
            delay(2000);
            REG_WRITE(RTC_CNTL_OPTIONS0_REG, RTC_CNTL_SW_SYS_RST); // Full hardware restart as with the reset button
          }
                    
          else { Serial.println("Another button"); }
        }
        else
        {
          Serial.println("Error - NEC remote code is invalid!");
          Serial.print("debug-- puls 9ms:");
          Serial.print(pulse_duration_9ms);
          Serial.print("  4.5ms:");
          Serial.print(pulse_duration_4_5ms);
          Serial.print("  1690us:");
          Serial.print(pulse_duration_1690us);
          Serial.print("  560us:");
          Serial.println(pulse_duration_560us);    
        }
        ir_code = 0;
        bit_count = 0;
        attachInterrupt(digitalPinToInterrupt(recv_pin), pulseISR, CHANGE);  
      } 

      /*---------------------  CONNECTION INFORMATION and RESET FUNCTION ---------------------*/ 
      if (WiFi.status() == WL_CONNECTED)
      {
        timer4.attach(0.5, updateStandbyLED);
        
        displayCenterBigText("CONNECTED",34);
        currentIP = WiFi.localIP().toString();
        showIP(1,51);
        u8g2.drawStr(1, 62, "Wait, reseting...");
        u8g2.sendBuffer();
        delay(2000);
        REG_WRITE(RTC_CNTL_OPTIONS0_REG, RTC_CNTL_SW_SYS_RST); // Full hardware restart as with the reset button
      }
    
    } // Infinite loop with Wifi processing to avoid going to radio screen when Wifi is not available
  }
}
// #######################################################################################  LOOP  ####################################################################################### //
void loop() 

{
    runTime1 = esp_timer_get_time();
  audio.loop();           // Performs the main loop for an audio object
  button2.loop();         // Loops for button2 object (checks button state from encoder 2)
  handleButtons();        // Calls an additional function that handles encoder buttons.
  handleFadeIn();         // Support for muting stations when switching
  
  #ifdef SERIALCOM
    handleSerialRX();
  #endif
  
  vTaskDelay(2);          // Short latency, frees up CPU time for other tasks
  
  /*---------------------  IR REMOTE FUNCTION / IR remote control support in NEC code ---------------------*/ 
  handleRemote();         

  /*-- KEYBOARD FUNCTION / Reading the ADC keyboard status under GPIO 9 ---------------------*/
  if ((millis() - keyboardLastSampleTime >= keyboardSampleDelay) && (adcKeyboardEnabled)) // ADC - keyboard check 
  {
    keyboardLastSampleTime = millis();
    handleKeyboard();
  }
    
  /*-- ENCODER 1 - support for optional encoder 1 ---------------------*/
  #ifdef twoEncoders
    button1.loop();
    handleEncoder1();
  #endif

  /*-- ENCODER 2 - Basic, encoder 2 support ---------------------*/
  if (encoderFunctionOrder) { handleEncoder2StationsVolumeClick();} else {handleEncoder2VolumeStationsClick();}


  //if (encoderFunctionOrder == 0) { handleEncoder2VolumeStationsClick(); } 
  //else if (encoderFunctionOrder == 1) { handleEncoder2StationsVolumeClick(); }
  
  // Obsługa przycisku Power ON/OFF
  #ifdef SW_POWER
    if (digitalRead(SW_POWER) == LOW) {powerOff();}
  #endif

  /*---------------------  DIMMER FUNCTION ---------------------*/
  if ((displayActive == true) && (displayDimmerActive == true) && (fwupd == false)) {displayDimmer(0);}  

  /*---------------------  BACK FUNCTION from all Menu options, Settings, etc. ---------------------*/
  if ((fwupd == false) && (displayActive) && (millis() - displayStartTime >= displayTimeout))  // Restore previous screen content after 6 seconds
  {
    if (volumeBufferValue != volumeValue && f_saveVolumeStationAlways) { saveVolumeOnSD(); }    
    if ((rcInputDigitsMenuEnable == true) && (station_nr != stationFromBuffer)) { changeStation(); }  // If the station number has changed, we load the new station
    
    displayDimmer(0); 
    clearFlags();
    displayRadio();
    u8g2.sendBuffer();
    
    // We restore the highlighting of the currently playing station
    currentSelection = station_nr - 1; 
    //if (maxSelection() - currentSelection < maxVisibleLines) {firstVisibleLine = maxSelection() - 3;} else {firstVisibleLine = currentSelection;}
  }


  /*---------------------  MILLIS SCROLLER LOOP FUNCTION / Refresh VU Meter, Time, Scroller, OLED, WiFi ver. 1 ---------------------*/ 
  if ((millis() - scrollingStationStringTime > scrollingRefresh) && (displayActive == false)) 
  {
    scrollingStationStringTime = millis();
    
    if (ActionNeedUpdateTime == true) // Clock Update, Voice Clock, Audio Debug, Wi-Fi Signal
    {
      ActionNeedUpdateTime = false;
      updateTime();
  
      if (f_requestVoiceTimePlay == true) // Voice clock, we check if the clock flag is set at the full hour
      {
        f_requestVoiceTimePlay = false;
        voiceTimeNL();  //change this line to voiceTimeEN() for english clock speach or voiceTime for Polisch
      }

      if (debugAudioBuffor == true) {bufforAudioInfo();}
      
      if ((displayMode == 0) || (displayMode == 2))  {drawSignalPower(210,63,0,1);}
      if ((displayMode == 1) && (volumeMute == false)) {drawSignalPower(244,47,0,1);}//requestVoiceTimePlay
      if (displayMode == 3) {drawSignalPower(209,10,0,1);}


      if (wifiSignalLevel != wifiSignalLevelLast) {wifiSignalLevelLast = wifiSignalLevel; wsSignalLevel();}


      if ((f_audioInfoRefreshStationString == true) && (displayActive == false)) // Changing the stream title - requires refreshing the display
      { 
        f_audioInfoRefreshStationString = false; //NewAudioInfoSSF
        stationStringFormatting(); // We format the StationString to be displayed by the Scroller
      } 

      if ((f_audioInfoRefreshDisplayRadio == true) && (displayActive == false)) //&& (f_requestVoiceTimePlay == false) Changing the bitrate - requires refreshing the display
      { 
        f_audioInfoRefreshDisplayRadio = false;
        ActionNeedUpdateTime = true;
        displayRadio();
      } 
      
      if (wsAudioRefresh == true) // StremInfo websocket needs a refresh
      {
        wsAudioRefresh = false;
        wsStreamInfoRefresh();
      }
      
    }

    if (volumeMute == false) // If mute OFF we draw VU
    {
      if (vuMeterOn)
      { 
        if (displayMode == 0) {vuMeterMode0();}
        if (displayMode == 3) {vuMeterMode3();}
        if (displayMode == 4) 
        {
          vuMeterMode4(); 
          if (debugAudioBuffor) // Audio buffer indicator in analog indicator mode
          {
            for (int i = 0; i < 10; i++) 
            {
              int y = 1 + (9 - i) * 6; 
              if (audioBufferTime > i) { u8g2.drawBox(126, y, 8, 5);} else {u8g2.drawFrame(126, y, 8, 5);}
            }
          }
        }
        if (displayMode == 5) {vuMeterMode5();}
        if (displayMode == 7) {vuMeterMode7();}
      }
      else if (displayMode == 0) {showIP(1,47);} //y = vuRy
    }
    else // If MUTE, instead of VU we enter the MUTE inscription on the screen
    {
      u8g2.setDrawColor(0);
      if (displayMode == 0) {u8g2.drawStr(0,48, "> MUTED <");}
      if (displayMode == 1) {u8g2.drawStr(200,47, "> MUTED <");}
      if (displayMode == 2) {u8g2.drawStr(0,48, "> MUTED <");}
      if (displayMode == 3) {u8g2.drawStr(101,63, "> MUTED <");}
      if (displayMode == 4) {u8g2.setDrawColor(1); vuMeterMode4(); u8g2.setDrawColor(1); u8g2.setFont(spleen6x12PL); u8g2.setDrawColor(0); u8g2.drawStr(103,57, "> MUTED <");}
      u8g2.setDrawColor(1);
    }  

       
    if (urlToPlay == true) // If the web server has set the "play URL" flag, we run the playback function from the URL sent by the website.
    {
      urlToPlay = false;
      webUrlStationPlay();
      displayRadio();
    }
    
    displayRadioScroller();  // we perform text scrolling station strings we prepare screen buffer
    u8g2.sendBuffer();  // we draw the entire contents of the screen.
    
    //if (f_callInfo) {f_callInfo = false; displayInfo();}  
    
  }
  //runTime2 = esp_timer_get_time();
  //runTime = runTime2 - runTime1;
}
