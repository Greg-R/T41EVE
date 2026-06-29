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

// Display class

#pragma once
#include "SDT.h"

class Display
{

public:
  uint32_t TEMP_X_OFFSET{15};
  uint32_t TEMP_Y_OFFSET{465}; // 480 * 0.97 = 465
  const uint32_t SPECTRUM_RES{512};
  int16_t spectrum_x = 10;
  uint8_t waterfall[MAX_WATERFALL_WIDTH]{0};
  int maxYPlot;
  int filterWidthX; // The current filter X.
  uint8_t write_analog_gain = 0;
  int16_t pos_x_time = 390; // 14;
  int16_t pos_y_time = 5;   // 114;
  float xExpand = 1.4;      //
  int16_t spectrum_pos_centre_f = 64 * xExpand;
  int pos_centre_f = 64;
  int smeterLength;
  float CPU_temperature = 0.0;
  float32_t processor_load{0};
  double elapsed_micros_mean;
  int centerLine = (MAX_WATERFALL_WIDTH + SPECTRUM_LEFT_X) / 2 + 1; // The FFT is plotted starting at x = 3.
  bool spectrumErased{false};
  int16_t smeterPad{0};
  float32_t dbm;
  char timeBuffer[15];

  void ShowSpectrum(bool drawSpectrum); // Draws the RF and audio spectrums.
  void DisplaydbM();                    // Display signal level in dBm.
  void ShowTempAndLoad();               // Display the current temperature and load figures for Teensy 4.1.
  void RedrawAll();                     // This function redraws the entire display.
  void DisplayClock();

private:
  const int32_t CLIP_AUDIO_PEAK{115}; // The pixel value where audio peak overwrites S-meter
  const uint32_t INCREMENT_X{WATERFALL_RIGHT_X + 25};
  const uint32_t INCREMENT_Y{WATERFALL_TOP_Y + 70};
  const uint32_t SMETER_X{WATERFALL_RIGHT_X + 16};
  const float32_t SMETER_Y{YPIXELS * 0.22}; // 480 * 0.22 = 106

  uint8_t convert_rgb565_to_rgb332(uint16_t rgb565_color);
  uint8_t signalToRGB332(uint8_t signal);

  float32_t pixel_per_khz{0};
  int pos_left{0};
  int filterWidth{0};
  int32_t old_hpf_offset{0};
  int32_t oldFilterWidth{0};

  const int TIME_X{550}; // Upper-left corner for time
  const float32_t TIME_Y{YPIXELS * 0.07};
};
