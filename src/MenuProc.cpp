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

// User menus and associated graphics drivers.

// CalibrateOptions
// CWOptions
// AGC Options
// Receive Equalizer Options
// SSB Options
// RF Options
// DoPaddleFlip
// VFO Select
// ConfigDataOptions
// CalDataOptions
// SubmenuSelect
// SubmenuSelectString

#include "SDT.h"

// Updates by KF5N to CalibrateOptions() function.  Added SSB Carrier and SSB Transmit cal.  Greg KF5N July 10, 2024
// Updated receive calibration code to clean up graphics.  KF5N August 3, 2023
// ==============  AFP 10-22-22 ==================
/*****
  Purpose: Present the Calibrate options available and return the selection.
  This function is embedded in the mail receiver loop.  It gets called repeatedly during calibration.

  Parameter list:
    void

  Return value
   void
*****/
void MenuProc::CalibrateOptions()
{
  int freqCorrectionFactorOld = 0;
  int32_t increment = 100;
  MenuSelect menu = MenuSelect::BOGUS_PIN_READ; // Used by frequency calibration.

  if (evemenucontrol.subMenuSelect)
  {
    SubmenuSelectString(IQOptions); // This sets the member variable subMenuChoice.
    return;
  }

  calibrateFlag = true;
  switch (subMenuChoice)
  {

  case 0:                                    // Calibrate Frequency  - uses WWV
    evemenucontrol.runOptionFunction = true; // Runs in loop().
    parameterAdjustFlag = true;              // Prevents multiple EEPROM writes.
    CalData.freqCorrectionFactor = GetEncoderValueLive(-200000, 200000, CalData.freqCorrectionFactor, increment);
    if (CalData.freqCorrectionFactor != freqCorrectionFactorOld)
    {
      si5351.set_correction(CalData.freqCorrectionFactor, SI5351_PLL_INPUT_XO);
      freqCorrectionFactorOld = CalData.freqCorrectionFactor;
    }
    menu = button.readButton();
    if (menu != MenuSelect::BOGUS_PIN_READ)
    {
      if (menu == MenuSelect::MENU_OPTION_SELECT)
      {
        calibrateFlag = false;
        parameterAdjustFlag = false;              // Save to EEPROM at conclusion of this function.
        evemenucontrol.runOptionFunction = false; // Deactivate function.
        subMenuChoice = 0;
      }
    }

    break;

  case 1:                                    // CW PA Cal.  Set using encoder.
    evemenucontrol.runOptionFunction = true; // Runs in loop().
    parameterAdjustFlag = true;              // Prevents multiple EEPROM writes.
    CalData.CWPowerCalibrationFactor[ConfigData.currentBand] = GetEncoderValueLive(0.0, 1.0, CalData.CWPowerCalibrationFactor[ConfigData.currentBand], 0.01);
    ConfigData.powerOutCW[ConfigData.currentBand] = sqrt(ConfigData.transmitPowerLevel / 20.0) * CalData.CWPowerCalibrationFactor[ConfigData.currentBand];

    menu = button.readButton();
    if (menu != MenuSelect::BOGUS_PIN_READ)
    {
      if (menu == MenuSelect::MENU_OPTION_SELECT)
      {
        //        eeprom.CalDataWrite();
        calibrateFlag = false;
        parameterAdjustFlag = false;
        evemenucontrol.runOptionFunction = false; // Deactivate function.
        subMenuChoice = 0;
      }
    }

    break;

  case 2:                                                    // CW IQ Receive Cal - Gain and Phase
    rxcalibrater.DoReceiveCalibrate(0, false, false, false); // This function was significantly revised.  KF5N August 16, 2023
    parameterAdjustFlag = false;                             // Save to EEPROM at conclusion of this function.
    evemenucontrol.runOptionFunction = false;                // Deactivate function.
    subMenuChoice = 0;
    break;

  case 3: // CW Xmit Carrier calibration.  Parameters are (mode, radioCal, refineCal, saveToEeprom)

    txcalibrater.DoXmitCarrierCalibrate(0, false, false, false);
    parameterAdjustFlag = false;              // Save to EEPROM at conclusion of this function.
    evemenucontrol.runOptionFunction = false; // Deactivate function.
    subMenuChoice = 0;
    break;

  case 4:
    txcalibrater.DoXmitCalibrate(0, false, false, false); // This function was significantly revised.  KF5N August 16, 2023
    parameterAdjustFlag = false;                          // Save to EEPROM at conclusion of this function.
    evemenucontrol.runOptionFunction = false;             // Deactivate function.
    subMenuChoice = 0;
    break;

  case 5:                                    // SSB PA Cal.  Set using encoder.
    evemenucontrol.runOptionFunction = true; // Runs in loop().
    parameterAdjustFlag = true;              // Prevents multiple EEPROM writes.
    CalData.SSBPowerCalibrationFactor[ConfigData.currentBand] = GetEncoderValueLive(0.0, 1.0, CalData.SSBPowerCalibrationFactor[ConfigData.currentBand], 0.01);
    ConfigData.powerOutSSB[ConfigData.currentBand] = sqrt(ConfigData.transmitPowerLevel / 20.0) * CalData.SSBPowerCalibrationFactor[ConfigData.currentBand];

    menu = button.readButton();
    if (menu != MenuSelect::BOGUS_PIN_READ)
    {
      if (menu == MenuSelect::MENU_OPTION_SELECT)
      {
        calibrateFlag = false;
        parameterAdjustFlag = false;              // Save to EEPROM at conclusion of this function.
        evemenucontrol.runOptionFunction = false; // Deactivate function.
        subMenuChoice = 0;
      }
    }

    break;

  case 6:                                                    // SSB receive cal
    rxcalibrater.DoReceiveCalibrate(1, false, false, false); // This function was significantly revised.  KF5N August 16, 2023
    parameterAdjustFlag = false;                             // Save to EEPROM at conclusion of this function.
    evemenucontrol.runOptionFunction = false;                // Deactivate function.
    subMenuChoice = 0;

    break;

  case 7: // SSB Carrier Cal
    txcalibrater.DoXmitCarrierCalibrate(1, false, false, false);
    parameterAdjustFlag = false;              // Save to EEPROM at conclusion of this function.
    evemenucontrol.runOptionFunction = false; // Deactivate function.
    subMenuChoice = 0;
    break;

  case 8: // SSB Transmit cal
    txcalibrater.DoXmitCalibrate(1, false, false, false);
    parameterAdjustFlag = false;              // Save to EEPROM at conclusion of this function.
    evemenucontrol.runOptionFunction = false; // Deactivate function.
    subMenuChoice = 0;
    break;

  case 9: // CW fully automatic radio calibration.  (mode, initial cal = false, refinecal = true)
    txcalibrater.RadioCal(0, false);
    calibrateFlag = false;
    parameterAdjustFlag = false;              // Save to EEPROM at conclusion of this function.
    evemenucontrol.runOptionFunction = false; // Deactivate function.
    subMenuChoice = 0;
    break;

  case 10: // CW full automatic calibration refinement.
    txcalibrater.RadioCal(0, true);
    calibrateFlag = false;
    parameterAdjustFlag = false;              // Save to EEPROM at conclusion of this function.
    evemenucontrol.runOptionFunction = false; // Deactivate function.
    subMenuChoice = 0;
    break;

  case 11: // SSB fully automatic radio calibration.
    txcalibrater.RadioCal(1, false);
    calibrateFlag = false;
    parameterAdjustFlag = false;              // Save to EEPROM at conclusion of this function.
    evemenucontrol.runOptionFunction = false; // Deactivate function.
    subMenuChoice = 0;
    break;

  case 12: // SSB fully automatic calibration refinement.
    txcalibrater.RadioCal(1, true);
    calibrateFlag = false;
    parameterAdjustFlag = false;              // Save to EEPROM at conclusion of this function.
    evemenucontrol.runOptionFunction = false; // Deactivate function.
    subMenuChoice = 0;
    break;

  case 13:                                   // dBm level cal.
    evemenucontrol.runOptionFunction = true; // Runs in loop().
    CalData.dBm_calibration = GetEncoderValueLive(0, 100, CalData.dBm_calibration, 1);
    if (CalData.dBm_calibration != freqCorrectionFactorOld)
    {
      freqCorrectionFactorOld = CalData.dBm_calibration;
    }
    menu = button.readButton();
    if (menu != MenuSelect::BOGUS_PIN_READ)
    { // Any button press??
      if (menu == MenuSelect::MENU_OPTION_SELECT)
      { // Yep. Make a choice??
        calibrateFlag = false;
        parameterAdjustFlag = false;              // Save to EEPROM at conclusion of this function.
        evemenucontrol.runOptionFunction = false; // Deactivate function.
        subMenuChoice = 0;
      }
    }
    break;

  case 14: // Calibrate buttons
    SaveAnalogSwitchValues();
    calibrateFlag = false;
    parameterAdjustFlag = false;              // Save to EEPROM at conclusion of this function.
    evemenucontrol.runOptionFunction = false; // Deactivate function.
    subMenuChoice = 0;
    break;

  case 15:                                   // Set button repeat rate
    evemenucontrol.runOptionFunction = true; // Runs in loop().
    CalData.buttonRepeatDelay = 1000 * GetEncoderValueLive(0, 5000, CalData.buttonRepeatDelay / 1000, 1);
    menu = button.readButton();
    if (menu != MenuSelect::BOGUS_PIN_READ)
    {
      if (menu == MenuSelect::MENU_OPTION_SELECT)
      {
        calibrateFlag = false;
        parameterAdjustFlag = false;              // Save to EEPROM at conclusion of this function.
        evemenucontrol.runOptionFunction = false; // Deactivate function.
        subMenuChoice = 0;
      }
    }
    break;

  case 16: // Cancelled choice

    calibrateFlag = 0;
    parameterAdjustFlag = false;              // Save to EEPROM at conclusion of this function.
    evemenucontrol.runOptionFunction = false; // Deactivate function.
    subMenuChoice = 0;
    return; // Do not save to EEPROM.
    break;

  default:
    break;
  }

  if (not parameterAdjustFlag)
  {
    eeprom.CalDataWrite(); // All settings require saving calibration to EEPROM.
  }
}

/*****
  Purpose: Present the CW options available to the user.  Change and store to ConfigData.
  Partially updated to revised parameter entry.  Greg Raven KF5N November 2025
  Parameter list:
    void

  Return value
    void
*****/
void MenuProc::CWOptions() // new option for Sidetone and Delay JJP 9/1/22
{
  std::vector<std::string> cwChoices{"Decode Sensitivity", "CW Filter", "CW Offset", "Keyer WPM", "Sidetone Speaker", "Sidetone Headphones",
                                     "Key Type", "Paddle Flip", "Transmit Delay", "Cancel"};
  MenuSelect menu;
  bool cancel = false;

  if (evemenucontrol.subMenuSelect)
  {
    SubmenuSelectString(cwChoices); // This sets subMenuSelect = false when it is done.
    return;                         // Don't continue into the rest of the function until the sub menu is selected.
  }

  switch (subMenuChoice)
  {

  case 0: // Set Morse decoder sensitivity.  This runs in active receive loop!

    morseDecodeAdjustFlag = true; // This is used to show decode adjust in receiver screen.
    parameterAdjustFlag = true;   // Prevents multiple EEPROM writes.

    encoderValueLive = static_cast<uint32_t>(GetEncoderValueLive(100, 10000, ConfigData.morseDecodeSensitivity, 100));
    ConfigData.morseDecodeSensitivity = encoderValueLive;

    menu = button.readButton();
    if (menu != MenuSelect::BOGUS_PIN_READ)
    { // Any button press??
      if (menu == MenuSelect::MENU_OPTION_SELECT)
      { // Yep. Selected result.  Now exit and write to EEPROM.
        morseDecodeAdjustFlag = false;
        evemenucontrol.runOptionFunction = false;
        parameterAdjustFlag = false; // OK to write to EEPROM upon exiting.
        subMenuChoice = 0;
      }
    }
    break;

  case 1: // CW Filter BW.

    cwprocess.SelectCWFilter();

    break;

  case 2: // Select a preferred CW offset frequency.

    cwprocess.SelectCWOffset();

    break;

  case 3: // Keyer WPM.  Encoder input.

    cwprocess.SetKeyerWPM();

    break;

  case 4: // Sidetone volume for speaker.  This has it's own while loop by necessity.

    cwprocess.SetSideToneVolume(true);

    break;

  case 5: // Sidetone volume for headphone.  This has it's own while loop by necessity.

    cwprocess.SetSideToneVolume(false);

    break;

  case 6: // Type of key.  This can't run in the loop due to required GPIO settling.

    cwprocess.SetKeyType(); // Straight key or keyer? Stored in ConfigData.keyType.

    break;

  case 7: // Flip paddles.

    cwprocess.DoPaddleFlip();

    break;

  case 8: // Set transmit delay.

    button.InputParameterEncoderNoWhile(250, 3000, 250, "Transmit Delay:", ConfigData.cwTransmitDelay);

    break;

  case 9: // Cancel

    cancel = true;
    evemenucontrol.runOptionFunction = false;
    parameterAdjustFlag = false; // OK to write to EEPROM upon exiting.
    subMenuChoice = 0;

    break;

  default:
    break;
  }
  if (not parameterAdjustFlag and cancel == false)
  {
    eeprom.ConfigDataWrite();
  }
}

/*****
  Purpose: Adjust the receiver's AGC configuration.

  Parameter list:
    void

  Return value
    none
*****/
void MenuProc::AGCOptions()
{
  std::vector<std::string> AGCChoices = {"AGC On", "AGC Off", "AGC Threshold", "Cancel"};
  bool cancel = false;

  if (evemenucontrol.subMenuSelect)
  {
    SubmenuSelectString(AGCChoices);
    return;
  }

  switch (subMenuChoice)
  {
  case 0: // AGC On.  This may be confusing to the user.  May need to use the button command function.
    ConfigData.AGCMode = true;
    SetAudioOperatingState(radioState);
    evemenucontrol.runOptionFunction = false; // Necessary because this is a simple command.
                                              //    evemenucontrol.subMenuSelect = false;
    parameterAdjustFlag = false;
    subMenuChoice = 0;

    break;

  case 1: // AGC Off.  This may be confusing to the user.  May need to use the button command function.
    ConfigData.AGCMode = false;
    SetAudioOperatingState(radioState);
    evemenucontrol.runOptionFunction = false; // Necessary because this is a simple command.
                                              //    evemenucontrol.subMenuSelect = false;
    parameterAdjustFlag = false;
    subMenuChoice = 0;
    break;

  case 2: // Set AGC threshold.  Does not run in loop, but maybe it should.
    button.InputParameterEncoderFloat(-60.0, -20.0, 1.0, "AGC Threshold:", ConfigData.AGCThreshold);
    initializeAudioPaths();

    break;

  case 3:                                     // Cancel
    evemenucontrol.runOptionFunction = false; // Necessary because this is a simple command.
    parameterAdjustFlag = false;
    subMenuChoice = 0;
    cancel = true;
    break;

  default:
    break;
  } // End case

  if (not parameterAdjustFlag and cancel == false)
  {
    eeprom.ConfigDataWrite();
  }
}

/*****
  Purpose: To process the graphics for the 14 chan equalizer option.  Has a while loop.

  Parameter list:
    int array[]         The array to fill in.  0 is receive, 1 is transmit.
    char *title         The equalizer being set.
  Return value
    void
*****/
void MenuProc::ProcessEqualizerChoices(int EQType)
{
  columnIndex = 0; // Get ready to set values for columns
  MenuSelect menu = MenuSelect::DEFAULT;

  // Nested while loops for adjusting equalizer values.  This is the outer loop.
  while (columnIndex < EQUALIZER_CELL_COUNT)
  {
    // User adjusts the value of the bar in the inner loop.
    while (true)
    {
      if (filterEncoderMove != 0)
      {
        if (EQType == 0)
          ConfigData.equalizerRec[columnIndex] = ConfigData.equalizerRec[columnIndex] + (PIXELS_PER_EQUALIZER_DELTA * filterEncoderMove);
        if (EQType == 1)
          ConfigData.equalizerXmt[columnIndex] = ConfigData.equalizerXmt[columnIndex] + (PIXELS_PER_EQUALIZER_DELTA * filterEncoderMove);

        filterEncoderMove = 0;
      }
      menu = button.readButton();             // Read the ladder value
      if (menu != MenuSelect::BOGUS_PIN_READ) // Note this takes ANY valid button push!
      {
        filterEncoderMove = 0;
        columnIndex++;
        break; // Leave inner loop.
      }
      evedisplay.drawEqualizerAdjustScreen(EQType);
      delay(17);
    } // end inner while
    evedisplay.drawEqualizerAdjustScreen(EQType);
    delay(20);
  } // end outer while
  evemenucontrol.runOptionFunction = false; // Necessary because this is a simple command.
  parameterAdjustFlag = false;
  subMenuChoice = 0;
  SetAudioOperatingState(radioState); // Equalizer adjusted.
}

/*****
  Purpose: Receive EQ set

  Parameter list:
    void

  Return value
    int           an index into the band array
*****/
void MenuProc::EqualizerRecOptions()
{
  std::vector<std::string> RecEQChoices = {"RX EQ On", "RX EQ Off", "RX EQSet", "Cancel"}; // Add code practice oscillator

  if (evemenucontrol.subMenuSelect)
  {
    SubmenuSelectString(RecEQChoices); // This sets subMenuSelect = false when it is done.
    return;                            // Don't continue into the rest of the function until the sub menu is selected.
  }

  switch (subMenuChoice)
  {
  case 0:
    ConfigData.receiveEQFlag = true;
    FilterSetSSB();
    break;
  case 1:
    ConfigData.receiveEQFlag = false;
    break;
  case 2:
    ProcessEqualizerChoices(0);
    break;
  case 3:
    break;
  }
  eeprom.ConfigDataWrite();
  evemenucontrol.runOptionFunction = false;
  parameterAdjustFlag = false;
  subMenuChoice = 0;
}

/*****
  Purpose: Transmit equalizer options.

  Parameter list:
    void

  Return value
    none
*****/
void MenuProc::EqualizerXmtOptions()
{
  std::vector<std::string> XmtEQChoices = {"TX EQ On", "TX EQ Off", "TX EQSet", "Cancel"}; // Add code practice oscillator

  if (evemenucontrol.subMenuSelect)
  {
    SubmenuSelectString(XmtEQChoices); // This sets subMenuSelect = false when it is done.
    return;                            // Don't continue into the rest of the function until the sub menu is selected.
  }

  switch (subMenuChoice)
  {
  case 0:
    ConfigData.xmitEQFlag = true;
    break;
  case 1:
    ConfigData.xmitEQFlag = false;
    break;
  case 2:
    ProcessEqualizerChoices(1);
    break;
  case 3: // Do nothing and exit.
    break;
  }
  eeprom.ConfigDataWrite();
  evemenucontrol.runOptionFunction = false;
  parameterAdjustFlag = false;
  subMenuChoice = 0;
}

/*****
  Purpose: Set options for the SSB exciter.
           This has both commands and encoder parameter adjust.

  Parameter list:
    void

  Return value
    none
*****/
void MenuProc::SSBOptions()
{
  float imdAmplitude = 1.0;

  bool cancel = false;
  MenuSelect menu = MenuSelect::BOGUS_PIN_READ; // Used in IMD test.
  std::vector<std::string> ssbChoices = {"CESSB", "SSB", "FT8 Active", "FT8 Disable", "Compressor On", "Compressor Off", "Microphone Gain", "Compressor Ratio", "Compressor Threshold", "IMD Test ", "Cancel"};

  if (evemenucontrol.subMenuSelect)
  {
    SubmenuSelectString(ssbChoices); // This sets subMenuSelect = false when it is done.
    return;                          // Don't continue into the rest of the function until the sub menu is selected.
  }

  switch (subMenuChoice)
  {

  case 0: // CESSB on.  Command.
    ConfigData.cessb = true;
    cessb1.setProcessing(ConfigData.cessb);
    evemenucontrol.runOptionFunction = false;
    parameterAdjustFlag = false;
    subMenuChoice = 0;

    break;

  case 1: // SSB Voice
    ConfigData.cessb = false;
    cessb1.setProcessing(ConfigData.cessb);
    evemenucontrol.runOptionFunction = false;
    parameterAdjustFlag = false;
    subMenuChoice = 0;
    break;

  case 2: // FT8 active
    ConfigData.cessb = false;
    cessb1.setProcessing(ConfigData.cessb);
    ft8EnableFlag = true;
    evemenucontrol.runOptionFunction = false;
    parameterAdjustFlag = false;
    subMenuChoice = 0;
    break;

  case 3: // FT8 disable
    ft8EnableFlag = false;
    evemenucontrol.runOptionFunction = false;
    parameterAdjustFlag = false;
    subMenuChoice = 0;
    break;

  case 4: // Compressor On
    ConfigData.compressorFlag = true;
    evemenucontrol.runOptionFunction = false;
    parameterAdjustFlag = false;
    subMenuChoice = 0;
    break;

  case 5: // Compressor Off
    ConfigData.compressorFlag = false;
    evemenucontrol.runOptionFunction = false;
    parameterAdjustFlag = false;
    subMenuChoice = 0;
    break;

  case 6: // Adjust mic gain in dB.  Default 0 db.
    button.InputParameterEncoderNoWhile(-20, 20, 1, "Mic Gain dB", ConfigData.micGain);
    break;

  case 7: // Set compression ratio.  Default 5.
    button.InputParameterEncoderNoWhile(1, 1000, 1, "Comp Ratio", ConfigData.micCompRatio);
    break;

  case 8: // Set compressor threshold.  Default -15.0 dB.
    button.InputParameterEncoderNoWhile(-60, 0, 1, "Comp Thresh dB", ConfigData.micThreshold);
    break;

  case 9: // IMD test.  This is a self-contained loop which uses the SSB exciter.

    radioState = RadioState::SSB_IM3TEST_STATE;
    bands.bands[ConfigData.currentBand].mode = RadioMode::SSB_MODE;
    SetAudioOperatingState(radioState);
    SetFreq();
    digitalWrite(RXTX, HIGH); // xmit on

    while (menu != MenuSelect::MENU_OPTION_SELECT)
    {
      menu = button.readButton(); // Use this to quit.
      evedisplay.drawTransmitterScreen();
      // Return IMD amplitude in dB.
      imdAmplitudedB = GetEncoderValueLive(0.0, 100.0, imdAmplitudedB, 1.0);
      imdAmplitude = volumeLog[static_cast<int>(imdAmplitudedB)];
      toneSSBCal1.amplitude(imdAmplitude);
      toneSSBCal2.amplitude(imdAmplitude);
      SSB_ExciterIQData();
    } // End IMD loop.

    radioState = RadioState::SSB_RECEIVE_STATE;
    digitalWrite(RXTX, LOW); // Transmitter off.
    SetAudioOperatingState(radioState);
    button.ExecuteModeChange();
    if (menu != MenuSelect::BOGUS_PIN_READ)
    { // A button press?
      if (menu == MenuSelect::MENU_OPTION_SELECT)
      {
        evemenucontrol.runOptionFunction = false;
        parameterAdjustFlag = false;
        subMenuChoice = 0;
        imdAmplitudedB = 5;  // Put back to a low level.
        return; // Nothing saved or configured upon exit.
      }
    }
    break;

  case 10: // Cancel
    evemenucontrol.runOptionFunction = false;
    parameterAdjustFlag = false;
    subMenuChoice = 0;
    cancel = true;
    break;

  default:
    break;
  }
  // Upon exit:
  if (not parameterAdjustFlag and cancel == false)
  {
    eeprom.ConfigDataWrite();
    evemenucontrol.runOptionFunction = false;
    parameterAdjustFlag = false;
    subMenuChoice = 0;
    updateMic();
  }
}

/*****
  Purpose: RF related options adjustment.

  Parameter list:
    void

  Return value
    void
*****/
void MenuProc::RFOptions()
{
  std::vector<std::string> rfOptions = {"TX Power Set", "Receiver DSP Gain Set", "Cancel"};
  bool cancel = false;
  MenuSelect menu;

  if (evemenucontrol.subMenuSelect)
  {
    SubmenuSelectString(rfOptions); // This sets subMenuSelect = false when it is done.
    return;                         // Don't continue into the rest of the function until the sub menu is selected.
  }

  switch (subMenuChoice)
  {
  case 0: // TX Power Set.

    transmitPowerAdjustFlag = true; //
    parameterAdjustFlag = true;     // Prevents multiple EEPROM writes.  This means adjustment is in progress.

    encoderValueLive = static_cast<int32_t>(GetEncoderValueLive(1, 20, ConfigData.transmitPowerLevel, 1));
    ConfigData.transmitPowerLevel = encoderValueLive;
    initPowerCoefficients();

    menu = button.readButton();
    if (menu != MenuSelect::BOGUS_PIN_READ)
    { // Any button press??
      if (menu == MenuSelect::MENU_OPTION_SELECT)
      { // Yep. Selected result.  Now exit and write to EEPROM.
        transmitPowerAdjustFlag = false;
        evemenucontrol.runOptionFunction = false;
        subMenuChoice = 0;
        parameterAdjustFlag = false; // OK to write to EEPROM upon exiting.
      }
    }

    // Can this run inside the transmitter loop???
    //    button.InputParameterEncoderNoWhile(1, 20, 1, "TX Power", ConfigData.transmitPowerLevel);
    // When the transmit power level is set, this means ALL of the power coefficients must be revised!
    // powerOutCW and powerOutSSB must be updated.

    break;

  case 1: // Manual "RF Gain" (actually DSP gain) set.  Run this inside receiver loop.

    //    button.InputParameterEncoderNoWhile(-60, 20, 5, "RF Gain", ConfigData.rfGain[ConfigData.currentBand]);

    receiverGainAdjustFlag = true; //
    parameterAdjustFlag = true;    // Prevents multiple EEPROM writes.  This means adjustment is in progress.

    encoderValueLive = static_cast<int32_t>(GetEncoderValueLive(-60, 20, ConfigData.rfGain[ConfigData.currentBand], 5));
    ConfigData.rfGain[ConfigData.currentBand] = encoderValueLive;
    initPowerCoefficients();

    menu = button.readButton();
    if (menu != MenuSelect::BOGUS_PIN_READ)
    { // Any button press??
      if (menu == MenuSelect::MENU_OPTION_SELECT)
      { // Yep. Selected result.  Now exit and write to EEPROM.
        receiverGainAdjustFlag = false;
        evemenucontrol.runOptionFunction = false;
        subMenuChoice = 0;
        parameterAdjustFlag = false; // OK to write to EEPROM upon exiting.
      }
    }

    break;

  default: // Cancel
    evemenucontrol.runOptionFunction = false;
    subMenuChoice = 0;
    cancel = true;
    break;
  }
  if (parameterAdjustFlag == false and cancel == false)
  {
    eeprom.ConfigDataWrite();
  }
}

/*****
  Purpose: Used to change the currently active VFO.
           This does not have a sub-menu!!!

  Parameter list:
    void

  Return value
    none
*****/
void MenuProc::VFOSelect()
{
  std::vector<std::string> VFOOptions = {"VFO A", "VFO B"};

  button.InputParameterButtonNoWhile("VFO Option", VFOOptions, ConfigData.activeVFO);

  // Now configure for selected VFO.
  if (not parameterAdjustFlag)
  {
    NCOFreq = 0L;
    switch (ConfigData.activeVFO)
    {
    case VFO_A: // VFO A
      ConfigData.centerFreq = TxRxFreq = ConfigData.currentFreqA;
      ConfigData.activeVFO = VFO_A;
      ConfigData.currentBand = ConfigData.currentBandA;
      break;

    case VFO_B: // VFO B
      ConfigData.centerFreq = TxRxFreq = ConfigData.currentFreqB;
      ConfigData.activeVFO = VFO_B;
      ConfigData.currentBand = ConfigData.currentBandB;

      break;

    default:
      break;
    } // End switch
    eeprom.ConfigDataWrite();
    button.ExecuteModeChange();
    evemenucontrol.subMenuSelect = false; // This does not use a sub-menu.
    calibrateFlag = false;
  }
}

/*****
  Purpose: Allow user to set current user configuration values or restore default settings.
           "Get favorite" and "Set favorite" are currently removed.
  Parameter list:
    void

  Return value
    void
*****/
void MenuProc::ConfigDataOptions()
{ //           0               1                2                  3                  4                  5                  6             7            8
  std::vector<std::string> ConfigDataOpts = {"Save Current", "Load Defaults", "Copy Config->SD", "Copy SD->Config", "EEPROM->Serial", "Default->Serial", "Stack->Serial", "SD->Serial", "Cancel"};
  config_t tempConfig;    // A temporary config_t struct to copy ConfigData data into.
  config_t defaultConfig; // The configuration defaults.

  commandOptionsName = "Configuration Commands";

  if (evemenucontrol.subMenuSelect)
  {
    SubmenuSelectString(ConfigDataOpts); // This sets subMenuSelect = false when it is done.
    return;                              // Don't continue into the rest of the function until the sub menu is selected.
  }

  switch (subMenuChoice)
  {
  case 0: // Save current ConfigData struct to ConfigData non-volatile memory.  Also save the bands struct at the same time!
    eeprom.ConfigDataWrite();
    eeprom.BandsWrite();

    break;

  case 1:                        // Perhaps this should be near the end of the list???
    eeprom.ConfigDataDefaults(); // Load Defaults to ConfigData struct and refresh display.

    break;

  case 2:                                                     // Copy ConfigData->SD.
    eeprom.ConfigDataRead(tempConfig);                        // Read as one large chunk
    json.saveConfiguration(configFilename, tempConfig, true); // Save ConfigData struct to SD

    break;

  case 3:                                               // Copy SD->ConfigData
    json.loadConfiguration(configFilename, ConfigData); // Copy from SD to struct in active memory (on the stack) ConfigData.
    eeprom.ConfigDataWrite();                           // Write to ConfigData non-volatile memory.
    initUserDefinedStuff();                             // Various things must be initialized.  This is normally done in setup().  KF5N February 21, 2024
                                                        //    display.RedrawAll();                                // Assume there are lots of changes and do a heavy-duty refresh.  KF5N August 7, 2023

    break;

  case 4: // EEPROM->Serial
    Serial.println(F("\nBegin ConfigData from EEPROM"));
    // Don't want to overwrite the stack.  Need a temporary struct, read the ConfigData data into that.
    eeprom.ConfigDataRead(tempConfig);
    json.saveConfiguration(configFilename, tempConfig, false); // Write the temporary struct to the serial monitor.
    Serial.println(F("\nEnd ConfigData from EEPROM\n"));

    break;

  case 5: // Defaults->Serial
    Serial.println(F("\nBegin ConfigData defaults"));
    json.saveConfiguration(configFilename, defaultConfig, false); // Write default ConfigData struct to the Serial monitor.
    Serial.println(F("\nEnd ConfigData defaults\n"));
    break;

  case 6: // Stack->Serial
    Serial.println(F("Begin ConfigData on the stack"));
    json.saveConfiguration(configFilename, ConfigData, false); // Write current ConfigData struct to the Serial monitor.
    Serial.println(F("\nEnd ConfigData on the stack\n"));

    break;

  case 7: // SD ConfigData->Serial
    Serial.println(F("Begin ConfigData on the SD card"));
    json.printFile(configFilename); // Write SD card ConfigData struct to the Serial monitor.
    Serial.println(F("End ConfigData on the SD card\n"));

    break;

  default:
    break;
  }
  evemenucontrol.runOptionFunction = false;
  parameterAdjustFlag = false;
  subMenuChoice = 0;
}

/*****
  Purpose: Allow user to set restore calibration values or restore default settings.

  Parameter list:
    void

  Return value
    void
*****/
void MenuProc::CalDataOptions()
{ //           0               1                2               3               4                     5                  6               7          8
  std::vector<std::string> CalDataOpts = {"Save Current", "Load Defaults", "Copy Cal->SD", "Copy SD->Cal", "EEPROM->Serial", "Defaults->Serial", "Stack->Serial", "SD->Serial", "Cancel"};
  calibration_t tempCal;    // A temporary calibration_t struct to copy CalData data into.
  calibration_t defaultCal; // The configuration defaults.

  if (evemenucontrol.subMenuSelect)
  {
    SubmenuSelectString(CalDataOpts); // This sets subMenuSelect = false when it is done.
    return;                           // Don't continue into the rest of the function until the sub menu is selected.
  }

  switch (subMenuChoice)
  {
  case 0: // Save current CalData struct to CalData non-volatile memory.
    eeprom.CalDataWrite();
    break;

  case 1:
    eeprom.CalDataDefaults(); // Restore defaults to CalData struct and refresh display.
    break;

  case 2: // Copy CalData->SD.
    eeprom.CalDataRead(tempCal);
    json.saveCalibration(calFilename, tempCal, true); // Save ConfigData struct to SD
    break;

  case 3:                                       // Copy SD->CalData
    json.loadCalibration(calFilename, CalData); // Copy from SD to struct in active memory (on the stack) ConfigData.
    eeprom.CalDataWrite();                      // Write to ConfigData non-volatile memory.
    break;

  case 4: // CalData EEPROM->Serial
  {
    Serial.println(F("\nBegin CalData from EEPROM"));
    // Don't want to overwrite the stack.  Need a temporary struct, read the CalData data into that.
    eeprom.CalDataRead(tempCal);
    json.saveCalibration(calFilename, tempCal, false); // Write the temporary struct to the serial monitor.
    Serial.println(F("\nEnd CalData from EEPROM\n"));
  }
  break;

  case 5: // Defaults->Serial
    Serial.println(F("\nBegin CalData defaults"));
    json.saveCalibration(calFilename, defaultCal, false); // Write default CalData struct to the Serial monitor.
    Serial.println(F("\nEnd CalData defaults\n"));
    break;

  case 6: // Stack->Serial
    Serial.println(F("Begin CalData on the stack"));
    json.saveCalibration(calFilename, CalData, false); // Write current CalData struct to the Serial monitor.
    Serial.println(F("\nEnd CalData on the stack\n"));
    break;

  case 7: // SD CalData->Serial
    Serial.println(F("Begin CalData on the SD card"));
    json.printFile(calFilename); // Write SD card CalData struct to the Serial monitor.
    Serial.println(F("End CalData on the SD card\n"));
    break;

  default:
    break;
  }
  evemenucontrol.runOptionFunction = false;
  parameterAdjustFlag = false;
  subMenuChoice = 0;
}

/*****
  Purpose: To select an option from a submenu.  This does not have a while loop.

  Parameter list:
    string options[]           submenus
    int numberOfChoices       choices available
    int defaultState          the starting option

  Return value
    int           an index into the band array

NOTE!!!  This function does not use the encoder.  It has a variable which
implies it uses the encoder.  It does not!

*****/
void MenuProc::SubmenuSelectString(std::vector<std::string> options)
{
  MenuSelect menu;

  menu = button.readButton();                                                                                                               // Read the ladder value
  if ((menu != MenuSelect::BOGUS_PIN_READ) and (menu == MenuSelect::MENU_OPTION_SELECT or menu == MenuSelect::MAIN_MENU_UP or menu == MenuSelect::MAIN_MENU_DN)) // Valid choice?
  {
    switch (menu)
    {
    case MenuSelect::MENU_OPTION_SELECT:    // They made a choice
      evemenucontrol.subMenuSelect = false; // Deactivate sub-menu selection.
      break;

    case MenuSelect::MAIN_MENU_UP:
      subMenuChoice++;
      if (subMenuChoice > static_cast<int32_t>(options.size() - 1))
        subMenuChoice = 0;
      break;

    case MenuSelect::MAIN_MENU_DN:
      subMenuChoice--;
      if (subMenuChoice < 0)
        subMenuChoice = options.size() - 1;
      break;

    default:
      subMenuChoice = -1; // An error selection
      break;
    }
  }

  subMenuString = options[subMenuChoice]; // This is used by EVE to display the choice.
}
