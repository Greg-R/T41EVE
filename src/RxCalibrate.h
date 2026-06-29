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


#pragma once

// Re-factoring into class Calibrate.  Greg KF5N June 15, 2024.
// Automatic calibration added.  Greg KF5N June 11, 2024
// Updates to DoReceiveCalibration() and DoXmitCalibrate() functions by KF5N.  July 20, 2023
// Updated PlotCalSpectrum() function to clean up graphics.  KF5N August 3, 2023
// Major clean-up of calibration.  KF5N August 16, 2023
// Re-factored into class RxCalibrate.cpp.  This class does receive calibrate only.
// Greg Raven KF5N October 20, 2025.

class RxCalibrate {
public:

  // Blue and red bar variables.  These should be 256 (bins) apart.
  // These variables identify the center of the bars, not the left edge.
  // The exact bins should be 127 and 383, however, for some reason, the
  // peak is a couple of bins to the right.
  int32_t rx_blue_usb = 385;  // Image frequency.
  int32_t rx_red_usb = 129;   // Receive frequency. Exact bin 127.
  int32_t capture_bins = 32;  // Make bar_width divisible by 2.
  int32_t left_text_edge = 165;

  uint32_t IQCalType = 0;  // 0 is IQ Gain; 1 is Phase.
  bool refineCal = false;
  bool autoCal{false};
  bool radioCal = false;
  bool averageFlag = false;
  bool saveToEeprom = false;
  int val;
  //float increment = 0.002;
  int userScale, userZoomIndex, userxmtMode;
  int transmitPowerLevelTemp, cwFreqOffsetTemp, calFreqTemp;
  uint16_t base_y = 460;  // 247
  int calTypeFlag = 0;
  float adjdB = 0.0;
  float adjdBold = 0.0;  // Used in exponential averager.  KF5N May 19, 2024
  float adjdB_avg = 0.0;
  uint32_t adjdBMinIndex;
  float32_t amplitude = 0.0;
  float32_t phase = 0.0;
  q15_t rawSpectrumPeak = 0;
  uint32_t index = 0;
  uint32_t count = 0;
  uint32_t warmup = 0;
  float32_t increment{0};
  bool exitManual = false;
  bool corrChange = false;
  bool fftActive = false;
  bool fftSuccess = false;
  elapsedMillis milliTimer;
  int mode;
  Sideband tempSideband;
  RadioMode tempMode;
  RadioState tempState;

  enum class State { warmup,
                     refineCal,
                     state0,
                     initialSweepAmp,
                     initialSweepPhase,
                     refineAmp,
                     refinePhase,
                     average,
                     setOptimal,
                     exit };

  void loadCalToneBuffers(float toneFreq);
  void MakeFFTData();
  void warmUpCal();
  void CalibratePreamble(int setZoom);
  void CalibrateEpilogue(bool radioCal, bool saveToEeprom);
  void DoReceiveCalibrate(int calMode, bool radio, bool refine, bool toEeprom);  // Mode determines CW versus SSB.
  void ShowSpectrum();
  float PlotCalSpectrum(int x1, int cal_bins[3], int capture_bins);
  void writeToCalData(float ichannel, float qchannel);
};