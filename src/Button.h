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

// Button class

#pragma once

class Button
{

public:
  bool mainMenu{false};
  std::string freqString{""};
  int numdigits{0}; // number of digits entered
  std::string buttonParameterName{""};
  int32_t encoderAdjustInt{0}; // This variable is manipulated by the encoder and used in drawEncoderEntryScreen.
  float32_t encoderAdjustFloat{0.0};
  std::string buttonSelection{""};
  std::string commandOptionsName{""};
  int32_t buttonReturnValue{0};
  bool encoderNotFilter{false};
  bool save_last_frequency = false; // Make this the default behavior.  Greg KF5N October 16, 2024.

  // Initialize the object with user selected tuning increments specified by the user in MyConfigurationFile.h.
  Button(std::vector<uint32_t> &fineTuneArray, std::vector<uint32_t> &centerTuneArray)
      : fineTuneArray(fineTuneArray), centerTuneArray(centerTuneArray) {}

  bool buttonInterruptsEnabled = false;
  void EnableButtonInterrupts();
  MenuSelect readButton();
  MenuSelect ProcessButtonPress(int valPin);
  int ReadSelectedPushButton();
  void ExecuteButtonPress(MenuSelect val);
  void ButtonCenterFreqIncrement();
  void ButtonFineFreqIncrement();
  void ButtonMenuIncrease();
  void ButtonMenuDecrease();
  void ButtonBandIncrease();
  void ButtonBandDecrease();
  void BandSet(int band);
  void ButtonZoom();
  void ButtonFilter();
  void ButtonSelectSideband();
  void ButtonMode();
  void ButtonNR();
  void ButtonNotchFilter();
  void ButtonMuteAudio();
  void ExecuteModeChange();
  void ButtonFrequencyEntry();
  void InputParameterButton(const std::string parameterName, std::vector<std::string> selectionList, int32_t &parameter);
  void InputParameterButtonNoWhile(const std::string parameterName, std::vector<std::string> selectionList, int32_t &parameter);
  void InputCommandButton(const std::string parameterName, std::vector<std::string> selectionList);
  void InputParameterEncoderFloat(float32_t minValue, float32_t maxValue, float32_t increment, const std::string parameterName, float32_t &parameter);
  void InputParameterEncoderNoWhile(int32_t minValue, int32_t maxValue, int32_t increment, const std::string parameterName, int32_t &parameter);

private:
  std::vector<uint32_t> &fineTuneArray;
  std::vector<uint32_t> &centerTuneArray;
  std::vector<uint32_t>::iterator result;
  std::vector<std::string>::iterator stringIterator;
  const int32_t TOP_MENU_COUNT{10};
  IntervalTimer buttonInterrupts;
  int buttonRead = 0;
  int minPinRead = 1024;
  int secondaryMenuChoiceMade;

  int directFreqFlag = 0;
  int32_t subMenuMaxOptions = 0; // Holds the number of submenu options.
  uint32_t TxRxFreqOld = 0;
  bool audioState = MUTEAUDIO;
};