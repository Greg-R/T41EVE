/*
T41EVE Copyright 2026 Gregory Raven

This file is part of T41EVE.

T41EVE is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

T41EVE is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with T41EVE. If not, see <https://www.gnu.org/licenses/>.

  This comment block must appear in the load page (e.g., main() or setup()) in any source code
  that uses code presented as whole or part of the T41-EP source code.

  (c) Frank Dziock, DD4WH, 2020_05_8
  "TEENSY CONVOLUTION SDR" substantially modified by Jack Purdum, W8TEE, and Al Peter, AC8GY

  This software is made available under the GNU GPLv3 license agreement. If commercial use of this
  software is planned, we would appreciate it if the interested parties contact Jack Purdum, W8TEE, 
  and Al Peter, AC8GY.

  Any and all other uses, written or implied, by the GPLv3 license are forbidden without written 
  permission from from Jack Purdum, W8TEE, and Al Peter, AC8GY.
*/

// User configuration file.

#pragma once

//====================== User Specific Preferences =============
const char RIGNAME[] = "T41-EP SDT";
const int RIGNAME_X_OFFSET = 570;  // Pixel count to rig name field.
//#define DEBUG 		                                                    // Uncommented for debugging, comment out for normal use
//#define TEMP_AND_LOAD                                                     // Uncomment to see temperature and load information.
//#define DEBUG_SWITCH_CAL                                                  // Uncomment to run switch cal by pushing and holding a button at power-up.
// Debug switch cal must be disabled for normal radio operation!
//#define DEBUG_CESSB                                                       // Uncomment to get CESSB operating parameters printed to the serial monitor.
//#define LOOP_TIMER
#define FAST_TUNE                       // Uncomment to activate variable speed fast tune by Harry GM3RVL.
#define DEFAULT_KEYER_WPM 15            // Startup value for keyer wpm
#define FREQ_SEP_CHARACTER '.'          // Some may prefer period, space, or combo
#define MAP_FILE_NAME "Cincinnati.bmp"  // Name you gave to BMP map file. Max is 50 chars
#define MY_LAT 39.07466                 // Coordinates for QTH
#define MY_LON -84.42677
#define MY_CALL "YOUR CALL"  // Default max is 10 chars
#define MY_TIMEZONE "UTC: "  // Set to your desired time zone.
//DB2OO, 29-AUG-23: TIME_24H to display 24 hour times (Utility.cpp). If this is not defined 12h display will be used
#define TIME_24H 1
//DB2OO, 29-AUG-23: ITU_REGION to determine band borders: Upper band limits on 80m (3.8MHz vs 4.0MhHz) and 40m (7.2MHz vs. 7.3MHz)
// A good reference table here:  https://www.arrl.org/frequency-bands
//#define ITU_REGION 1                //for Europe
#define ITU_REGION 2  // for USA
//#define ITU_REGION 3                // Asia/Oceania
// DB2OO, 29.823:. Analog Signal on this pin will be used for an analog S-Meter (250uA full scale) connected via 10kOhm to this output. 1uF capacitor paralle to the S-Meter. --> Display.cpp.
// This might conflict with other hardware modifications, that might use Pin 33 for a different purpose --> please check, before defining this
//#define HW_SMETER              33
// DB2OO, 30-AUG-23: with TCVSDR_SMETER (TCVSDR=Teensy Convolution SDR) defined the S-Meter bar will be consistent with the dBm value and it will not go over the box for S9+40+
#define TCVSDR_SMETER 1
//DB2OO, 10-SEP-23: TCXO_25MHZ defined sets the default ConfigData.ConfigData.freqCorrectionFactor = 0, as the TCXO is supposed to deliver 25.00000MHz
//#define TCXO_25MHZ                1

#define PADDLE_FLIP 0                // 0 = right paddle = DAH, 1 = DIT
#define STRAIGHT_KEY_OR_PADDLES 0    // 0 = straight, 1 = paddles
//#define SDCARD_MESSAGE_LENGTH 3000L  // The number of milliseconds to leave error message on screen

//====================== System specific ===============
#define CURRENT_FREQ_A 7200000  // VFO_A
#define CURRENT_FREQ_B 7030000  // VFO_B
const float DEFAULT_POWER_LEVEL = 10.0;  // Startup power level in watts.
#define SPLASH_DELAY 4000      // How long to show Splash screen. Use 1000 for testing, 4000 normally.
#define STARTUP_BAND 1          // This is the 40M band. see around line 575 in SDT.h

constexpr uint32_t CENTER_SCREEN_X = 400;
constexpr uint32_t CENTER_SCREEN_Y = 245;
constexpr uint32_t IMAGE_CORNER_X = 190;  // ImageWidth = 378 Therefore 800 - 378 = 422 / 2 = 211
constexpr uint32_t IMAGE_CORNER_Y = 40;   // ImageHeight = 302 Therefore 480 - 302 = 178 / 2 = 89
constexpr uint32_t RAY_LENGTH = 190;

// Set multiplication factors for your QSD and QSE boards.  Default values here and below are for V10/V11 boards.


// Uncomment this line for QSE2.
#define QSE2

// Uncomment this line if using an external PLL module.
#define PLLMODULE

// Customizable definitions for center and fine tune defaults and increments.  Larry K3PTO June 24, 2024
constexpr uint32_t CENTER_TUNE_DEFAULT = 1000;  // Set to the desired default in the CENTER_TUNE_ARRAY.
#define CENTER_TUNE_ARRAY { 1000, 10000, 100000, 1000000 }  // The number of elements is not fixed.
constexpr uint32_t FINE_TUNE_DEFAULT = 50;  // Set to the desired default in the FINE_TUNE_ARRAY.
#define FINE_TUNE_ARRAY { 10, 20, 50, 100, 200, 500 }  // The number of elements is not fixed.

// Uncomment for the original T41 audio mute control.
//#define UNMUTEAUDIO LOW
//#define MUTEAUDIO   HIGH
// Use this for external amp with mute LOW, unmute HIGH.
#define UNMUTEAUDIO HIGH
#define MUTEAUDIO LOW

// The audio amplifier gain may need to be adjusted for the best volume range.
constexpr float32_t SPEAKERSCALE = 2.0;   // Increase or decrease this value depending on your amplifier gain.
constexpr float32_t HEADPHONESCALE = 8.0;  // Same as for the speaker.  Adjust to your preference for volume range.

constexpr float32_t RFGAINSCALE = 1000.0;  // This adjusts for RF gain differences in the QSD.  QSD should use a value of 3000.  QSD2 should use a value of 1000.0.

constexpr float32_t FREQUENCYCAL = 100000;  // The nominal frequency calibration.  This can be set here permanently after determining the unique value for your radio.

