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

// CWProcessing class.
// CW mode related functions.

#pragma once

class CWProcessing {

public:

  static constexpr int32_t HISTOGRAM_ELEMENTS = 750;
  static constexpr int32_t MAX_DECODE_CHARS = 37;       // Max chars that can appear on decoder line.  Increased to 32.  KF5N October 29, 2023
  static constexpr uint32_t DECODER_BUFFER_SIZE = 128;  // Max chars in binary search string with , . ?

  byte currentDashJump = DECODER_BUFFER_SIZE;
  byte currentDecoderIndex = 0;

  float32_t aveCorrResultR;  // Used in running averages; must not be inside function.
  float32_t aveCorrResultL;
  float32_t *float_Corr_BufferR = new float32_t[511];
  float32_t *float_Corr_BufferL = new float32_t[511];
  float CWLevelTimer;
  float CWLevelTimerOld;
  float goertzelMagnitude;
  char decodeBuffer[38] = { 0 };  // The buffer for holding the decoded characters.  Increased to 33.  KF5N October 29, 2023
  int endGapFlag = 0;
  int topGapIndex;
  int topGapIndexOld;
  int32_t *gapHistogram = new int32_t[HISTOGRAM_ELEMENTS];
  int32_t *signalHistogram = new int32_t[HISTOGRAM_ELEMENTS];
  long valRef1;
  long valRef2;
  long gapRef1;
  int valFlag = 0;
  long signalStartOld = 0;
  long aveDitLength = 80;
  long aveDahLength = 200;
  float thresholdGeometricMean = 140.0;  // This changes as decoder runs
  float thresholdArithmeticMean;
  int dahLength;
  int gapAtom;  // Space between atoms
  int gapChar;  // Space between characters
  int32_t signalElapsedTime{0};
  int32_t signalStart{0};
  int32_t signalEnd{0};  // Start-end of dit or dah
  uint32_t gapLength{0};
  uint32_t estimatedWPM{0};
  float32_t freq[4] = { 562.5, 656.5, 750.0, 843.75 };
  bool drewGreenLastLoop{ false };  // Variables used to control drawing of CW decode
  bool drewBlackLastLoop{ false };  // indicator square.
  const std::vector<std::string> CWOffsets = { "562.5", "656.5", "750.0", "843.8" };
  const std::vector<std::string> keyChoice = { "Straight Key", "Keyer" };
  const std::vector<std::string> CWFilter = { "0.8kHz", "1.0kHz", "1.3kHz", "1.8kHz", "2.0kHz", " Off" };
  int col = 0;  // Start at lower left
  // charProcessFlag means a character is being decoded.  blankFlag indicates a blank has already been printed.
  bool charProcessFlag, blankFlag;
  int currentTime, interElementGap, noSignalTimeStamp;
  char *bigMorseCodeTree = (char *)"-EISH5--4--V---3--UF--------?-2--ARL---------.--.WP------J---1--TNDB6--.--X/-----KC------Y------MGZ7----,Q------O-8------9--0----";
  bool cwdecode{false};

  // This enum is used by an experimental Morse decoder.
  enum states { state0,
                state1,
                state2,
                state3,
                state4,
                state5,
                state6 };
  states decodeStates = state0;

  void JackClusteredArrayMax(int32_t *array, int32_t elements, int32_t *maxCount, int32_t *maxIndex, int32_t *firstNonZero, int32_t spread);
  void SelectCWFilter();
  void SelectCWOffset();
  void SetKeyerWPM();
  void DoCWReceiveProcessing();
  void SetTransmitDitLength(int wpm);
  void SetKeyType();
  void SetKeyPowerUp();
  void SetSideToneVolume(bool speaker);
  void MorseCharacterClear(void);
  void MorseCharacterDisplay(char currentLetter);
  void ResetHistograms();
  void DoGapHistogram(uint32_t gapLen);
  void DoCWDecoding(int audioValue);
  void DoSignalHistogram(long val);
  float goertzel_mag(int numSamples, int TARGET_FREQUENCY, int SAMPLING_RATE, float *data);
  void DoPaddleFlip();
};