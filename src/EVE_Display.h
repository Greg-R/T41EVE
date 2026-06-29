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

#include <SPI.h>
#include "EVE.h"

#define BLACK 0x0000
#define WHITE 0xFFFF

class EVE_Display {

public:

uint32_t num_dl_static = 0; /* amount of bytes in the static part of our display-list */
uint8_t tft_active = 0;
uint16_t num_profile_a = 0;
uint16_t num_profile_b = 0;
char buffer[10];
uint32_t waterfallLines{150};

uint32_t centerLine{256};
 int filterWidth{ 0 };
  int32_t old_hpf_offset{ 0 };
    int32_t oldFilterWidth{ 0 };
      uint32_t x1{0};  // Left x coordinate of tuning bar.
  uint32_t x2{0};  // Right x coordinate of tuning bar.
  uint32_t tuneAlignmentLineX{0};
 std::string filter[4] = { "Off", "Kim", "Spec", "LMS" };
  std::string zoomOptions[5]{"1x ", "2x ", "4x ", "8x ", "16x"};
  enum class Screens {
receiver,
directEntry,
buttonEntry,
encoderEntry,
transmitCal,
receiveCal,
carrierCal,
equalizerAdjust
  };

Screens screenSelect{Screens::receiver};
const uint32_t SMETER_X{ WATERFALL_RIGHT_X + 16 };
const float32_t SMETER_Y{ YPIXELS * 0.22 };  // 480 * 0.22 = 106
//DB2OO, 30-AUG-23: this variable determines the pixels per S step. In the original code it was 12.2 pixels !?
#ifdef TCVSDR_SMETER
const float pixels_per_s = 12;
#else
const float pixels_per_s = 12.2;
#endif

EVE_Display();
void drawSplash();

void initialize();

void drawReceiverScreen(int16_t* fftArray, uint8_t* waterfall, int16_t* audioArray);
void writeWaterFalltoRAM_G(uint8_t* waterfall);
void moveBitmapCells();
void receiverStatic_cmd_list();
void ComputeBandWidthIndicatorBar();

void directEntryStatic_cmd_list();

void drawDirectEntryScreen();
void drawButtonEntryScreen();

void drawEncoderEntryScreen(bool typeFloat);

void buttonEntryStatic_cmd_list();
void encoderEntryStatic_cmd_list();

void drawTransmitterScreen();
void transmitterStatic_cmd_list();

void drawTransmitterCalScreen(int16_t *fftArray);
void transmitterCalStatic_cmd_list();

void drawReceiverCalScreen(int16_t *fftArray);
void receiverCalStatic_cmd_list();

void drawEqualizerAdjustScreen(int EQType);
void equalizerAdjustStatic_cmd_list();

void drawReleaseButtonScreen();
void drawSwitchMatrixCalScreen();
void switchMatrixCalStatic_cmd_list();

void drawTransmitterAlarmScreen(std::string warningMessage);

void Example1();
void Example2();

};