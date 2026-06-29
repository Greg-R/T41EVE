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

// Transmit calibrate class.  This calibrates both CW and SSB exciters.


#pragma once

// Re-factoring into class Calibrate.  Greg KF5N June 15, 2024.
// Automatic calibration added.  Greg KF5N June 11, 2024
// Updates to DoReceiveCalibration() and DoXmitCalibrate() functions by KF5N.  July 20, 2023
// Updated PlotCalSpectrum() function to clean up graphics.  KF5N August 3, 2023
// Major clean-up of calibration.  KF5N August 16, 2023
// Re-factor calibration into transmit and receive classes.  KF5N August 2025

class TxCalibrate {
public:


  void DoXmitCalibrate(int calMode, bool radio, bool refine, bool toEeprom);
  void DoXmitCarrierCalibrate(int calMode, bool radio, bool refine, bool toEeprom);
  const char *calFreqs[2]{ "750 Hz", "3.0 kHz" };
  void ShowSpectrum();
  float PlotCalSpectrum(int x1, int cal_bins[3], int capture_bins);
  void RadioCal(int mode, bool refineCal);
  void buttonTasks();
  void writeToCalData(float ichannel, float qchannel);

//private:

  int32_t IQCalType = 0;
  int val;
  float32_t xmitIncrement = 0.002;       // Increment for transmit calibration.
  float32_t carrIncrement = 0.0005;  // Increment for carrier calibration.
  int userScale, userZoomIndex;
  int transmitPowerLevelTemp, cwFreqOffsetTemp, calFreqTemp;
  uint16_t base_y = 460;  // 247
  uint32_t calTypeFlag = 0;
  float adjdB = 0.0;
  float adjdBold = 0.0;  // Used in exponential averager.  KF5N May 19, 2024
  float adjdB_avg = 0.0;
  uint32_t adjdBMinIndex;
  float sineBuffer[512];  // Used to generate CW tone.
  q15_t rawSpectrumPeak = 0;
  uint32_t index = 0;
  uint32_t count = 0;
  uint32_t warmup = 0;
  bool corrChange = false;
  Sideband tempSideband;
  RadioMode tempMode;
  RadioState tempState;
  float32_t amplitude = 0;
  float32_t phase = 0;
  float iOptimal = 1.0;
  float qOptimal = 0.0;
  float32_t iDCoffset = 0.0;
  float32_t qDCoffset = 0.0;
  elapsedMillis milliTimer;
  int mode = 0;
  MenuSelect task = MenuSelect::DEFAULT;
  bool autoCal = false;
  bool refineCal = false;
  bool radioCal = false;
  bool averageFlag = false;
  bool saveToEeprom = false;
  int averageCount = 0;
  bool fftSuccess = false;  // A flag for debugging FFT inadequate data problems.
  bool fftActive = true;    // This variable is used to deactive creation of the FFT result.
                            // This is false when desired to push data through the system
                            // to wring out the transient response.
  bool exitManual = false;

  // Blue and red bar variables:
  int32_t rx_blue_usb = 128;
  int32_t rx_red_usb = 384;
  enum class Cal_Mode { CW_RECEIVE,
                        CW_TRANSMIT,
                        CW_CARRIER,
                        SSB_RECEIVE,
                        SSB_TRANSMIT,
                        SSB_CARRIER,
  };

  void warmUpCal();
  void CalibratePreamble(int setZoom);
  void CalibrateEpilogue();
  void MakeFFTData();
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

  State state = State::exit;
};