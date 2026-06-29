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

#include "SDT.h"

/*
The button interrupt routine implements a first-order recursive filter, or "leaky integrator,"
as described at:

  https://www.edn.com/a-simple-software-lowpass-filter-suits-embedded-system-applications/

Filter bandwidth is dependent on the sample rate and the "k" parameter, as follows:

                                1 Hz
                          k   Bandwidth   Rise time (samples)
                          1   0.1197      3
                          2   0.0466      8
                          3   0.0217      16
                          4   0.0104      34
                          5   0.0051      69
                          6   0.0026      140
                          7   0.0012      280
                          8   0.0007      561

Thus, the default values below create a filter with 10000 * 0.0217 = 217 Hz bandwidth
*/

static unsigned long buttonFilterRegister;
const uint32_t BUTTON_FILTER_SHIFT = 3; // Filter parameter k
static uint32_t buttonState, buttonADCPressed, buttonElapsed;
static volatile int buttonADCOut;
const uint32_t BUTTON_FILTER_SAMPLERATE = 10000; // Hz
const uint32_t BUTTON_DEBOUNCE_DELAY = 5000;     // uSec
const uint32_t BUTTON_STATE_UP = 0;
const uint32_t BUTTON_STATE_DEBOUNCE = 1;
const uint32_t BUTTON_STATE_PRESSED = 2;
const float32_t BUTTON_USEC_PER_ISR = (1000000 / BUTTON_FILTER_SAMPLERATE);
const uint32_t BUTTON_OUTPUT_UP = 1023; // Value to be output when in the UP state

/*****
  Purpose: ISR to read button ADC and detect button presses

  Parameter list:
    none
  Return value;
    void
*****/
void ButtonISR()
{
  int filteredADCValue;

  buttonFilterRegister = buttonFilterRegister - (buttonFilterRegister >> BUTTON_FILTER_SHIFT) + analogRead(BUSY_ANALOG_PIN);
  filteredADCValue = (int)(buttonFilterRegister >> BUTTON_FILTER_SHIFT);

  switch (buttonState)
  {
  case BUTTON_STATE_UP:
    if (filteredADCValue <= CalData.buttonThresholdPressed)
    {
      buttonElapsed = 0;
      buttonState = BUTTON_STATE_DEBOUNCE;
    }

    break;
  case BUTTON_STATE_DEBOUNCE:
    if (buttonElapsed < BUTTON_DEBOUNCE_DELAY)
    {
      buttonElapsed += BUTTON_USEC_PER_ISR;
    }
    else
    {
      buttonADCOut = buttonADCPressed = filteredADCValue;
      buttonElapsed = 0;
      buttonState = BUTTON_STATE_PRESSED;
    }

    break;
  case BUTTON_STATE_PRESSED:
    if (filteredADCValue >= CalData.buttonThresholdReleased)
    {
      buttonState = BUTTON_STATE_UP;
    }
    else if (CalData.buttonRepeatDelay != 0)
    { // buttonRepeatDelay of 0 disables repeat
      if (buttonElapsed < CalData.buttonRepeatDelay)
      {
        buttonElapsed += BUTTON_USEC_PER_ISR;
      }
      else
      {
        buttonADCOut = buttonADCPressed;
        buttonElapsed = 0;
      }
    }

    break;
  }
}

MenuSelect Button::readButton()
{
  int val = 0;
  MenuSelect menu = MenuSelect::DEFAULT;
  val = ReadSelectedPushButton();
  if (val != -1)
  { // -1 is returned by ReadSelectedPushButton in the case of an invalid read.
    menu = button.ProcessButtonPress(val);
  }
  else
    menu = MenuSelect::BOGUS_PIN_READ;
  return menu;
}

/*****
  Purpose: Starts button IntervalTimer and toggles subsequent button
           functions into interrupt mode.

  Parameter list:
    none
  Return value;
    void
*****/
void Button::EnableButtonInterrupts()
{
  buttonADCOut = BUTTON_OUTPUT_UP;
  buttonFilterRegister = buttonADCOut << BUTTON_FILTER_SHIFT;
  buttonState = BUTTON_STATE_UP;
  buttonADCPressed = BUTTON_STATE_UP;
  buttonElapsed = 0;
  buttonInterrupts.begin(ButtonISR, 1000000 / BUTTON_FILTER_SAMPLERATE);
  buttonInterruptsEnabled = true;
}

/*****
  Purpose: Determine which UI button was pressed

  Parameter list:
    int valPin            the ADC value from analogRead()

  Return value;
    int                   -1 if not valid push button, index of push button if valid
*****/
MenuSelect Button::ProcessButtonPress(int valPin)
{
  int switchIndex;

  if (valPin == BOGUS_PIN_READ)
  { // Not valid press
    return MenuSelect::BOGUS_PIN_READ;
  }

  for (switchIndex = 0; switchIndex < NUMBER_OF_SWITCHES; switchIndex++)
  {
    if (abs(valPin - CalData.switchValues[switchIndex]) < WIGGLE_ROOM) // ...because ADC does return exact values every time
    {
      return static_cast<MenuSelect>(switchIndex);
    }
  }

  return MenuSelect::DEFAULT;
}

/*****
  Purpose: Check for UI button press. If pressed, return the ADC value

  Parameter list:
    none

  Return value;
    int                   -1 if not valid push button, ADC value if valid
*****/
int Button::ReadSelectedPushButton()
{
  minPinRead = 0;
  int buttonReadOld = 1023;

  if (buttonInterruptsEnabled)
  {
    noInterrupts();
    buttonRead = buttonADCOut;

    /*
    Clear the button read.  If the button remains pressed, the ISR will reset the value nearly
    instantly.  Clearing the value here rather than in the ISR provides more consistent button
    press "feel" when calls to ReadSelectedPushButton have variable timing.
    */

    buttonADCOut = BUTTON_OUTPUT_UP;
    interrupts();
  }
  else
  {
    while (abs(minPinRead - buttonReadOld) > 3)
    { // do averaging to smooth out the button response
      minPinRead = analogRead(BUSY_ANALOG_PIN);

      buttonRead = .1 * minPinRead + (1 - .1) * buttonReadOld; // See expected values in next function.
      buttonReadOld = buttonRead;
    }
  }

  if (buttonRead > CalData.switchValues[0] + WIGGLE_ROOM)
  { // AFP 10-29-22 per Jack Wilson
    return -1;
  }
  minPinRead = buttonRead;
  if (!buttonInterruptsEnabled)
  {
    delay(100L);
  }
  return minPinRead;
}

/*****
  Purpose: Function is designed to route program control to the proper execution point in response to
           a button press.

  Parameter list:
    MenuSelect val  val is what is returned from readButton();

  Return value;
    void
*****/
void Button::ExecuteButtonPress(MenuSelect val)
{

  switch (val)
  {
  case MenuSelect::MENU_OPTION_SELECT: // 1

    evemenucontrol.top = false;              // Turn off EVE top menu.
    evemenucontrol.subMenuSelect = true;     // Turn on EVE sub-menu.
    evemenucontrol.runOptionFunction = true; // Run the option function from the loop.
    // This is used in the case of selecting MENU_OPTION_SELECT during normal receiver operation.
    functionPtr[mainMenuIndex](); // These are processed in MenuProcessing.cpp
    break;

  case MenuSelect::MAIN_MENU_UP: // 2
    ButtonMenuIncrease();
    evemenucontrol.top = true;
    evemenucontrol.subMenuSelect = false;
    break;

  case MenuSelect::BAND_UP: // 3
    ButtonBandIncrease();
    break;

  case MenuSelect::ZOOM: // 4
    ConfigData.spectrum_zoom++;

    if (ConfigData.spectrum_zoom == MAX_ZOOM_ENTRIES)
    {
      ConfigData.spectrum_zoom = 0;
    }
    if (ConfigData.spectrum_zoom <= 0)
      ConfigData.spectrum_zoom = 0;
    ButtonZoom();
    break;

  case MenuSelect::MAIN_MENU_DN: // 5
    ButtonMenuDecrease();
    //    menuProc.ShowMenu(&topMenus[mainMenuIndex], PRIMARY_MENU);
    evemenucontrol.top = true;
    evemenucontrol.subMenuSelect = false;
    break;

  case MenuSelect::BAND_DN: // 6
    ButtonBandDecrease();
    break;

  case MenuSelect::FILTER: // 7

    ButtonFilter();
    break;

  case MenuSelect::DEMODULATION: // 8

    ButtonSelectSideband();
    break;

  case MenuSelect::SET_MODE: // 9
    ButtonMode();
    break;

  case MenuSelect::NOISE_REDUCTION: // 10
    ButtonNR();
    break;

  case MenuSelect::NOTCH_FILTER: // 11
    ButtonNotchFilter();
    //    display.UpdateNotchField();
    break;

  case MenuSelect::MUTE_AUDIO: // 12.  Was noise floor.  Greg KF5N February 12, 2025
    ButtonMuteAudio();
    break;

  case MenuSelect::FINE_TUNE_INCREMENT: // 13
    ButtonFineFreqIncrement();
    break;

  case MenuSelect::DECODER_TOGGLE: // 14
    ConfigData.decoderFlag = not ConfigData.decoderFlag;
    //    display.UpdateDecoderStatus();
    break;

  case MenuSelect::MAIN_TUNE_INCREMENT: // 15
    ButtonCenterFreqIncrement();
    break;

  case MenuSelect::RESET_TUNING: // 16
    ResetTuning();
    break;

  case MenuSelect::UNUSED_1: // 17
    if (calOnFlag == false)
    {
      //        evemenucontrol.runButtonFunction = true;
      //      evedisplay.screenSelect = EVE_Display::Screens::directEntry;
      ButtonFrequencyEntry();
    }
    break;

    // Bearing is temporarily removed in T41EEE.91.
  case MenuSelect::BEARING: // 18
                            /*
   int doneViewing;
   float retVal;
   MenuSelect menu;
                        
Disabled in this release (0.91).  Greg Raven November 2025
   tft.clearScreen(RA8875_BLACK);
                        
   DrawKeyboard();
   CaptureKeystrokes();
   retVal = BearingHeading(keyboardBuffer);
                        
   if (retVal != -1.0) {  // We have valid country
     bmpDraw((char *)myMapFiles[selectedMapIndex].mapNames, IMAGE_CORNER_X, IMAGE_CORNER_Y);
     doneViewing = false;
   } else {
     tft.setTextColor(RA8875_RED);
     tft.setCursor(380 - (17 * tft.getFontWidth(0)) / 2, 240);  // Center message
     tft.print("Country not found");
     tft.setTextColor(RA8875_WHITE);
   }
   while (true) {
     menu = readButton();                       // Poll UI push buttons
     if (menu != MenuSelect::BOGUS_PIN_READ) {  // If a button was pushed...
       switch (menu) {
         case MenuSelect::BEARING:  // Pressed puchbutton 18
           doneViewing = true;
           break;
         default:
           break;
       }
     }
                        
     if (doneViewing == true) {
       break;
     }
   }
   display.RedrawAll();
   */
    break;

  case MenuSelect::BOGUS_PIN_READ: // 18

    break;

  default: // 19

    break;
  }
}

/*****
  Purpose: To process a center tuning increment button push

  Parameter list:
    void

  Return value:
    void
*****/
void Button::ButtonCenterFreqIncrement()
{
  uint32_t index = 0;
  // Find the index of the current fine tune setting.
  result = std::find(centerTuneArray.begin(), centerTuneArray.end(), ConfigData.centerTuneStep);
  index = std::distance(centerTuneArray.begin(), result);
  index++; // Increment index.
  if (index == centerTuneArray.size())
  { // Wrap around.
    index = 0;
  }
  ConfigData.centerTuneStep = centerTuneArray[index];
  //  display.DisplayIncrementField();
}

/*****
  Purpose: To process a fine tuning increment button push

  Parameter list:
    void

  Return value;
    void
*****/
void Button::ButtonFineFreqIncrement()
{
  uint32_t index = 0;
  // Find the index of the current fine tune setting.
  result = std::find(fineTuneArray.begin(), fineTuneArray.end(), ConfigData.fineTuneStep);
  index = std::distance(fineTuneArray.begin(), result);
  index++; // Increment index.
  if (index == fineTuneArray.size())
  { // Wrap around.
    index = 0;
  }
  ConfigData.fineTuneStep = fineTuneArray[index];
  //  display.DisplayIncrementField();
}

/*****
  Purpose: To process a menu increase button push

  Parameter list:
    void

  Return value:
    void
*****/
void Button::ButtonMenuIncrease()
{
  mainMenuIndex++;
  if (mainMenuIndex == TOP_MENU_COUNT)
  {                    // At last menu option, so...
    mainMenuIndex = 0; // ...wrap around to first menu option
  }
}

/*****
  Purpose: To process a menu decrease button push

  Parameter list:
    void

  Return value:
    void
*****/
void Button::ButtonMenuDecrease()
{
  mainMenuIndex--;
  if (mainMenuIndex < 0)
  {                                     // At last menu option, so...
    mainMenuIndex = TOP_MENU_COUNT - 1; // ...wrap around to first menu option
  }
}

/*****
  Purpose: To process a band increase button push.

  Parameter list:
    void

  Return value:
    void
*****/
void Button::ButtonBandIncrease()
{
  //  int tempIndex;
  //  tempIndex = ConfigData.currentBandA;
  //  if (ConfigData.currentBand == NUMBER_OF_BANDS)
  //  {                             // Incremented too far?
  //    ConfigData.currentBand = 0; // Yep. Roll to list front.
  //  }

  switch (ConfigData.activeVFO)
  {
  case VFO_A:
    /*
      tempIndex = ConfigData.currentBandA;
      if (save_last_frequency == 1)
      {
        ConfigData.lastFrequencies[tempIndex][VFO_A] = TxRxFreq;
      }
      else
      {
        if (save_last_frequency == 0)
        {
          if (directFreqFlag == 1)
          {
            ConfigData.lastFrequencies[tempIndex][VFO_A] = TxRxFreqOld;
          }
          else
          {
            if (directFreqFlag == 0)
            {
              ConfigData.lastFrequencies[tempIndex][VFO_A] = TxRxFreq;
            }
          }
          TxRxFreqOld = TxRxFreq;
        }
      }
        */
    ConfigData.currentBandA++;
    if (ConfigData.currentBandA == NUMBER_OF_BANDS)
    {                              // Incremented too far?
      ConfigData.currentBandA = 0; // Yep. Roll to list front.
    }
    ConfigData.currentBand = ConfigData.currentBandA;
    ConfigData.centerFreq = TxRxFreq = ConfigData.currentFreqA = ConfigData.lastFrequencies[ConfigData.currentBandA][VFO_A]; // + NCOFreq;
    break;

  case VFO_B:
    /*
      tempIndex = ConfigData.currentBandB;
      if (save_last_frequency == 1)
      {
        ConfigData.lastFrequencies[tempIndex][VFO_B] = TxRxFreq;
      }
      else
      {
        if (save_last_frequency == 0)
        {
          if (directFreqFlag == 1)
          {
            ConfigData.lastFrequencies[tempIndex][VFO_B] = TxRxFreqOld;
          }
          else
          {
            if (directFreqFlag == 0)
            {
              ConfigData.lastFrequencies[tempIndex][VFO_B] = TxRxFreq;
            }
          }
          TxRxFreqOld = TxRxFreq;
        }
      }
        */
    ConfigData.currentBandB++;
    if (ConfigData.currentBandB == NUMBER_OF_BANDS)
    {                              // Incremented too far?
      ConfigData.currentBandB = 0; // Yep. Roll to list front.
    }
    ConfigData.currentBand = ConfigData.currentBandB;
    ConfigData.centerFreq = TxRxFreq = ConfigData.currentFreqB = ConfigData.lastFrequencies[ConfigData.currentBandB][VFO_B]; // + NCOFreq;
    break;

    //    case VFO_SPLIT:
    //      DoSplitVFO();
    //      break;
  }
  //  directFreqFlag = 0;
  ExecuteModeChange();
}

/*****
  Purpose: To process a band decrease button push

  Parameter list:
    void

  Return value:
    void
*****/
void Button::ButtonBandDecrease()
{
  //  int tempIndex = ConfigData.currentBand;
  // ConfigData.currentBand--; // decrement band index

  //  if (ConfigData.currentBand < 0)
  //  {                                               // decremented too far?
  //    ConfigData.currentBand = NUMBER_OF_BANDS - 1; // Yep. Roll to list end.
  //  }

  // Save current frequency to lastFrequencies array.
  switch (ConfigData.activeVFO)
  {
  case VFO_A:
    /*
      if (save_last_frequency == 1)
      {
        ConfigData.lastFrequencies[tempIndex][VFO_A] = TxRxFreq;
      }
      else
      {
        if (save_last_frequency == 0)
        {
          if (directFreqFlag == 1)
          {
            ConfigData.lastFrequencies[tempIndex][VFO_A] = TxRxFreqOld;
          }
          else
          {
            if (directFreqFlag == 0)
            {
              ConfigData.lastFrequencies[tempIndex][VFO_A] = TxRxFreq;
            }
          }
          TxRxFreqOld = TxRxFreq;
        }
      }
        */
    // Decrement current band.
    ConfigData.currentBandA--;
    if (ConfigData.currentBandA == NUMBER_OF_BANDS)
    {                              // decremented too far?
      ConfigData.currentBandA = 0; // Yep. Roll to list front.
    }
    if (ConfigData.currentBandA < 0)
    {                                                // Incremented too far?
      ConfigData.currentBandA = NUMBER_OF_BANDS - 1; // Yep. Roll to list front.
    }

    ConfigData.currentBand = ConfigData.currentBandA;
    ConfigData.centerFreq = TxRxFreq = ConfigData.currentFreqA = ConfigData.lastFrequencies[ConfigData.currentBandA][VFO_A]; // + NCOFreq;
    break;

  case VFO_B:
    /*
      if (save_last_frequency == 1)
      {
        ConfigData.lastFrequencies[tempIndex][VFO_B] = TxRxFreq;
      }
      else
      {
        if (save_last_frequency == 0)
        {
          if (directFreqFlag == 1)
          {
            ConfigData.lastFrequencies[tempIndex][VFO_B] = TxRxFreqOld;
          }
          else
          {
            if (directFreqFlag == 0)
            {
              ConfigData.lastFrequencies[tempIndex][VFO_B] = TxRxFreq;
            }
          }
          TxRxFreqOld = TxRxFreq;
        }
      }
        */
    ConfigData.currentBandB--;
    if (ConfigData.currentBandB == NUMBER_OF_BANDS)
    {                              // Incremented too far?
      ConfigData.currentBandB = 0; // Yep. Roll to list front.
    }
    if (ConfigData.currentBandB < 0)
    {                                                // Incremented too far?
      ConfigData.currentBandB = NUMBER_OF_BANDS - 1; // Yep. Roll to list front.
    }

    ConfigData.currentBand = ConfigData.currentBandB;
    ConfigData.centerFreq = TxRxFreq = ConfigData.currentFreqB = ConfigData.lastFrequencies[ConfigData.currentBandB][VFO_B]; // + NCOFreq;
    break;

    //    case VFO_SPLIT:
    //      DoSplitVFO();
    //      break;
  }
  //  directFreqFlag = 0;
  ExecuteModeChange();
}

/*****
  Purpose: Set the radio to a band using the band parameter.  This function is probably obsolete thanks to the new modal states.
           It is still used in TX calibration!  Can it be replaced?

  Parameter list:
    int band

  Return value:
    void
*****/
void Button::BandSet(int band)
{
  //  int tempIndex;
  //  tempIndex = ConfigData.currentBandA;
  //  if (ConfigData.currentBand == NUMBER_OF_BANDS)
  //  {                             // Incremented too far?
  //    ConfigData.currentBand = 0; // Yep. Roll to list front.
  //  }
  NCOFreq = 0;
  switch (ConfigData.activeVFO)
  {
  case VFO_A:
    /*
      tempIndex = ConfigData.currentBandA;
      if (save_last_frequency == 1)
      {
        ConfigData.lastFrequencies[tempIndex][VFO_A] = TxRxFreq;
      }
      else
      {
        if (save_last_frequency == 0)
        {
          if (directFreqFlag == 1)
          {
            ConfigData.lastFrequencies[tempIndex][VFO_A] = TxRxFreqOld;
          }
          else
          {
            if (directFreqFlag == 0)
            {
              ConfigData.lastFrequencies[tempIndex][VFO_A] = TxRxFreq;
            }
          }
          TxRxFreqOld = TxRxFreq;
        }
      }
        */
    ConfigData.currentBandA = band;
    if (ConfigData.currentBandA == NUMBER_OF_BANDS)
    {                              // Incremented too far?
      ConfigData.currentBandA = 0; // Yep. Roll to list front.
    }
    ConfigData.currentBand = ConfigData.currentBandA;
    ConfigData.centerFreq = TxRxFreq = ConfigData.currentFreqA = ConfigData.lastFrequencies[ConfigData.currentBandA][VFO_A]; // + NCOFreq;
    break;

  case VFO_B:
    /*
      tempIndex = ConfigData.currentBandB;
      if (save_last_frequency == 1)
      {
        ConfigData.lastFrequencies[tempIndex][VFO_B] = TxRxFreq;
      }
      else
      {
        if (save_last_frequency == 0)
        {
          if (directFreqFlag == 1)
          {
            ConfigData.lastFrequencies[tempIndex][VFO_B] = TxRxFreqOld;
          }
          else
          {
            if (directFreqFlag == 0)
            {
              ConfigData.lastFrequencies[tempIndex][VFO_B] = TxRxFreq;
            }
          }
          TxRxFreqOld = TxRxFreq;
        }
      }
        */
    ConfigData.currentBandB = band;
    if (ConfigData.currentBandB == NUMBER_OF_BANDS)
    {                              // Incremented too far?
      ConfigData.currentBandB = 0; // Yep. Roll to list front.
    }
    ConfigData.currentBand = ConfigData.currentBandB;
    ConfigData.centerFreq = TxRxFreq = ConfigData.currentFreqB = ConfigData.lastFrequencies[ConfigData.currentBandB][VFO_B]; // + NCOFreq;
    break;
  }
  //  directFreqFlag = 0;
}

/*****
  Purpose: Change the horizontal scale of the frequency display.  AFP 09-27-22

  Parameter list:
    void

  Return value:
    void
*****/
void Button::ButtonZoom()
{
  fftOffset = 0; // This helps solve a problem with moving from 1X to 2X zoom.
  ZoomFFTPrep();
  ResetTuning(); // AFP 10-11-22
  ////  display.UpdateAudioGraphics();
}

/*****
  Purpose: To process a filter button push.  This switches the Filter Encoder from the lower limit to the upper limit
           of the audio filter.

  Parameter list:
    void

  Return value:
    void
*****/
void Button::ButtonFilter()
{
  switchFilterSideband = not switchFilterSideband;
  filterEncoderMove = 0;  // Set to zero so the delimiter lines are not moved by simply pushing Filter button.
  FilterSetSSB(); // Call this so the delimiter is set to the correct color.
}

/*****
  Purpose: Change LSB, USB, but only in CW or SSB modes.

  Parameter list:
    void

  Return value:
    void
*****/
void Button::ButtonSelectSideband()
{

  switch (bands.bands[ConfigData.currentBand].sideband)
  {
  case Sideband::LOWER: // Switch to USB.
    bands.bands[ConfigData.currentBand].sideband = Sideband::UPPER;
    ConfigData.lastSideband[ConfigData.currentBand] = Sideband::UPPER;

    break;
  case Sideband::UPPER: // Switch to LSB.
    // Leave FT8 in USB.
    if (bands.bands[ConfigData.currentBand].mode == RadioMode::FT8_MODE)
    {
      bands.bands[ConfigData.currentBand].sideband = Sideband::UPPER;
      return;
    }
    bands.bands[ConfigData.currentBand].sideband = Sideband::LOWER;
    ConfigData.lastSideband[ConfigData.currentBand] = Sideband::LOWER;

    break;
  case Sideband::BOTH_AM:
  case Sideband::BOTH_SAM:
    //  If already in AM or SAM mode, don't need to alter sidebands.  Demod selection already setup sidebands.
    return;
    break;
  default:
    break;
  }
  SetFreq();
  encoderFilterFlag = true;
  //  display.BandInformation();
}

/*****
  Purpose: Set radio mode for SSB, CW, FT8 or AM receive modes.

  Parameter list:
    void

  Return value:
    void
*****/
void Button::ButtonMode() //  Greg KF5N March 11, 2025
{
  // Don't allow mode change if something is keyed.  User must fix and restart radio.
  isTransmitterKeyed();
  // Toggle modes:
  switch (bands.bands[ConfigData.currentBand].mode)
  {

  case RadioMode::CW_MODE: // Toggle the current mode
    bands.bands[ConfigData.currentBand].mode = RadioMode::SSB_MODE;
    radioState = RadioState::SSB_RECEIVE_STATE;
    bands.bands[ConfigData.currentBand].sideband = ConfigData.lastSideband[ConfigData.currentBand];
    break;
  case RadioMode::SSB_MODE:
    bands.bands[ConfigData.currentBand].mode = RadioMode::FT8_MODE;
    radioState = RadioState::FT8_RECEIVE_STATE;
    bands.bands[ConfigData.currentBand].sideband = Sideband::UPPER; // FT8 always USB.
    break;
  case RadioMode::FT8_MODE: // Toggle the current mode.
    bands.bands[ConfigData.currentBand].mode = RadioMode::AM_MODE;
    radioState = RadioState::AM_RECEIVE_STATE;
    bands.bands[ConfigData.currentBand].sideband = Sideband::BOTH_AM;
    break;
  case RadioMode::AM_MODE: // Toggle the current mode
    bands.bands[ConfigData.currentBand].mode = RadioMode::SAM_MODE;
    radioState = RadioState::SAM_RECEIVE_STATE;
    bands.bands[ConfigData.currentBand].sideband = Sideband::BOTH_SAM;
    break;
  case RadioMode::SAM_MODE: // Toggle the current mode
    bands.bands[ConfigData.currentBand].mode = RadioMode::CW_MODE;
    radioState = RadioState::CW_RECEIVE_STATE;
    bands.bands[ConfigData.currentBand].sideband = ConfigData.lastSideband[ConfigData.currentBand];
    break;
  default:
    break;
  }
}

/*****
  Purpose: To process select noise reduction

  Parameter list:
    void

  Return value:
    void
*****/
void Button::ButtonNR() // AFP 09-19-22 update
{
  ConfigData.nrOptionSelect++;
  if (ConfigData.nrOptionSelect > 3)
  {
    ConfigData.nrOptionSelect = 0;
  }
  if (ConfigData.nrOptionSelect == 3)
    ConfigData.ANR_notch = false; // Turn off AutoNotch if LMS NR is selected.
}

/*****
  Purpose: To set the notch filter

  Parameter list:
    void

  Return value:
    void
*****/
void Button::ButtonNotchFilter()
{
  ConfigData.ANR_notch = not ConfigData.ANR_notch;
  //  If the notch is activated and LMS NR is also active, turn off NR and update display.
  if (ConfigData.ANR_notch && ConfigData.nrOptionSelect == 3)
  {
    ConfigData.nrOptionSelect = 0; // Turn off noise reduction.  Other NR selections will be valid.
  }
}

/*****
  Purpose: Mute speaker, headphone, or both with button pushes.  Greg KF5N February 12, 2025.
           This function rotates through the 4 possible permuations of mute/unmute.
           The state is rotated and then audio configuration updated by controlAudioOut(...).
           The function controlAudioOut() is in the file AudioSignal.h.
  Parameter list:
    void

  Return value:
    void
*****/
void Button::ButtonMuteAudio()
{
  switch (ConfigData.audioOut)
  {
  case AudioState::SPEAKER: // Speaker is on, mute and unmute headphones.
    ConfigData.audioOut = AudioState::HEADPHONE;
    break;
  case AudioState::HEADPHONE: // Mute both speaker and headphones.
    ConfigData.audioOut = AudioState::MUTE_BOTH;
    break;
  case AudioState::MUTE_BOTH: //  Unmute both.
    ConfigData.audioOut = AudioState::BOTH;
    break;
  case AudioState::BOTH: //  Headphones mute, speaker on
    ConfigData.audioOut = AudioState::SPEAKER;
    break;
  default:
    return;
    break;
  }
  controlAudioOut(ConfigData.audioOut, false); // Don't mute all.
  //  display.UpdateAudioOutputField();            // Update display.
}

/*****
  Purpose: Direct Frequency Entry

  Parameter list:
    void

  Return value;
    void
    Base Code courtesy of Harry  GM3RVL
*****/
void Button::ButtonFrequencyEntry()
{
  bool valid_frequency = false;
  //  int centerLine = (MAX_WATERFALL_WIDTH + SPECTRUM_LEFT_X) / 2;
  TxRxFreqOld = TxRxFreq; // Store current frequency in case it should be restored.

#define show_FEHelp
  bool doneFE = false; // set to true when a valid frequency is entered
  long enteredF = 0L;  // desired frequency

  String stringF;
  MenuSelect menu = MenuSelect::DEFAULT;
  int key{0};

  int pushButtonSwitchIndex;
  // Arrays for allocating values associated with keys and switches - choose whether USB keypad or analogue switch matrix
  // USB keypad and analogue switch matrix
  //  const char *DE_Band[] = {"80m", "40m", "20m", "17m", "15m", "12m", "10m"};
  //  const char *DE_Flimit[] = {"4.5", "9", "16", "26", "26", "30", "30"};
  int numKeys[] = {0x0D, 0x7F, 0x58, // values to be allocated to each key push
                   0x37, 0x38, 0x39,
                   0x34, 0x35, 0x36,
                   0x31, 0x32, 0x33,
                   0x30, 0x7F, 0x7F,
                   0x7F, 0x7F, 0x99};
#ifdef show_FEHelp
//  constexpr uint32_t KEYPAD_LEFT{350};
//  constexpr uint32_t KEYPAD_TOP{SPECTRUM_TOP_Y + 35};
//  constexpr uint32_t KEYPAD_WIDTH{150};
//  constexpr uint32_t KEYPAD_HEIGHT{300};
//  constexpr uint32_t BUTTONS_LEFT{KEYPAD_LEFT + 30};
//  constexpr uint32_t BUTTONS_TOP{KEYPAD_TOP + 30};
//  constexpr uint32_t BUTTONS_SPACE{45};
//  constexpr uint32_t BUTTONS_RADIUS{15};
//  constexpr int32_t TEXT_OFFSET{-8};
#endif

  while (doneFE == false)
  {
    evedisplay.drawDirectEntryScreen();
    delay(17);

    menu = readButton();
    pushButtonSwitchIndex = static_cast<int>(menu);
    if (pushButtonSwitchIndex != 18)
    {
      key = numKeys[pushButtonSwitchIndex];
      switch (key)
      {
      case 0x7F: // erase last digit =127
        if (numdigits != 0)
        {
          numdigits--;
          freqString.pop_back();
        }
        break;

      case 0x58: // Exit without updating frequency =88
        doneFE = true;
        valid_frequency = false;
        freqString = {""}; // Clear before exiting.
        numdigits = 0;
        break;

      case 0x0D: // Apply the entered frequency (if valid) =13
        enteredF = std::stoi(freqString);
        if ((numdigits == 1) || (numdigits == 2))
        {
          enteredF = enteredF * 1000000;
          valid_frequency = true;
        }
        if ((numdigits == 4) || (numdigits == 5))
        {
          enteredF = enteredF * 1000;
          valid_frequency = true;
        }
        if ((enteredF > 30000000) || (enteredF < 1250000))
        {
          freqString = "";
          numdigits = 0;
          valid_frequency = false;
        }
        else
        {
          // Return to normal receiver operation.
          doneFE = true;
        }
        break;

      case 0x99: // Toggle "Save Direct to Last Freq".
        save_last_frequency = not save_last_frequency;

        break;

        // The default processes numbers only.
      default: // Don't make the string greater than 5 digits and don't start with zero.
        if ((numdigits == 5) || ((key == 0x30) and (numdigits == 0)))
        {
        }
        else
        {
          //         strF[numdigits] = char(key);
          numdigits++;
          freqString.push_back(char(key));
        }
        break;
      }
    }
  } // End while

  if (key != 0x58 and valid_frequency)
  { // Skip if X button pushed.
    TxRxFreq = enteredF;
    freqString = {""}; // Clear before exiting.
    numdigits = 0;
  }
  NCOFreq = 0;
  //  directFreqFlag = 1;
  ConfigData.centerFreq = TxRxFreq;
  if (ConfigData.activeVFO == VFO_A)
    ConfigData.currentFreqA = TxRxFreq;
  if (ConfigData.activeVFO == VFO_B)
    ConfigData.currentFreqB = TxRxFreq;
  centerTuneFlag = 1; // Put back in so tuning bar is refreshed.  KF5N July 31, 2023
                      //  SetFreq();           // Used here instead of centerTuneFlag.  KF5N July 22, 2023
  // This determines if the entered frequency is permanent or temporary.
  if (save_last_frequency == true and valid_frequency == true)
  {
    ConfigData.lastFrequencies[ConfigData.currentBand][ConfigData.activeVFO] = enteredF; // Permanent.
  }
  //  else
  //  {
  //    if (save_last_frequency == false)
  //    {
  //      ConfigData.lastFrequencies[ConfigData.currentBand][ConfigData.activeVFO] = TxRxFreqOld; // Revert to original.
  //    }
  //  }
  SetFreq();
}

/*****
  Purpose: Execute mode change (including sideband change).
           Consolidation of all functions required to update DSP and graphics
           done in correct order.

  Parameter list:
    void

  Return value;
    void
*****/
void Button::ExecuteModeChange()
{
  encoderFilterFlag = true;
  NCOFreq = 0;
  fftOffset = 0;
  SetBandRelay(); // Set relays in LPF for current band.
  SetFreq();      // Must update frequency, for example moving from SSB to CW, the RX LO is shifted.  KF5N
  powerUp = true;
}

/*****
  Purpose: User Input Parameter using Buttons.  Has a while loop.
           This is used in the cases where signficant post-processing is required.

  Parameter list:
    const std::string parameterName          Name of the parameter being adjusted.
    std::vector<std::string> selectionList   An array of the possible options.
    uint32_t &parameter                      Reference to the currently set parameter.

  Return value;
    void
*****/
void Button::InputParameterButton(const std::string parameterName, std::vector<std::string> selectionList, int32_t &parameter)
{
  bool notDone = true;
  buttonReturnValue = parameter; // Used as an index of selectionList, so start with current value.
  MenuSelect menu = MenuSelect::DEFAULT;

  while(notDone) {
  buttonParameterName = parameterName;                // Show parameter name in screen.
  buttonSelection = selectionList[buttonReturnValue]; // Show current value in screen.
  evedisplay.drawButtonEntryScreen();

  menu = readButton(); // Read the button push.
  if (menu != MenuSelect::BOGUS_PIN_READ)
  { // Valid choice?
    switch (menu)
    {
    case MenuSelect::MENU_OPTION_SELECT: // They made a choice
      notDone = false;
      parameter = buttonReturnValue;
      evemenucontrol.runOptionFunction = false;
      evedisplay.screenSelect = EVE_Display::Screens::receiver;
      parameterAdjustFlag = false;
      menuProc.subMenuChoice = 0;
      audioFFToffset = 100; // This a re-initialization since the receiver screen was not used.
      break;

    case MenuSelect::MAIN_MENU_UP:
      buttonReturnValue++;
      if (buttonReturnValue >= static_cast<int32_t>(selectionList.size()))
        buttonReturnValue = 0;
      break;

    case MenuSelect::MAIN_MENU_DN:
      buttonReturnValue--;
      if (buttonReturnValue < 0)
        buttonReturnValue = selectionList.size() - 1;
      break;

    default:
      //  Do nothing if other buttons were pushed.
      break;
    } // End switch
    parameter = buttonReturnValue;
  } // End if
} // End while

}

/*****
  Purpose: Input Parameter using Buttons.  No while loop.

  Parameter list:
    const std::string parameterName          Name of the parameter being adjusted.
    std::vector<std::string> selectionList   An array of the possible options.
    uint32_t &parameter                      Reference to the currently set parameter.

  Return value;
    void
*****/
void Button::InputParameterButtonNoWhile(const std::string parameterName, std::vector<std::string> selectionList, int32_t &parameter)
{
  parameterAdjustFlag = true;
  buttonReturnValue = parameter; // Used as an index of selectionList, so start with current value.
  MenuSelect menu = MenuSelect::DEFAULT;

  buttonParameterName = parameterName;                // Show parameter name in screen.
  buttonSelection = selectionList[buttonReturnValue]; // Show current value in screen.

  evedisplay.screenSelect = EVE_Display::Screens::buttonEntry;

  menu = readButton(); // Read the button push.
  if (menu != MenuSelect::BOGUS_PIN_READ)
  { // Valid choice?
    switch (menu)
    {
    case MenuSelect::MENU_OPTION_SELECT: // They made a choice
      parameter = buttonReturnValue;
      //      evemenucontrol.subMenuSelect = false;
      evemenucontrol.runOptionFunction = false;
      evedisplay.screenSelect = EVE_Display::Screens::receiver;
      parameterAdjustFlag = false;
      menuProc.subMenuChoice = 0;
      audioFFToffset = 100; // This a re-initialization since the receiver screen was not used.
      break;

    case MenuSelect::MAIN_MENU_UP:
      buttonReturnValue++;
      if (buttonReturnValue >= static_cast<int32_t>(selectionList.size()))
        buttonReturnValue = 0;
      break;

    case MenuSelect::MAIN_MENU_DN:
      buttonReturnValue--;
      if (buttonReturnValue < 0)
        buttonReturnValue = selectionList.size() - 1;
      break;

    default:
      //  Do nothing if other buttons were pushed.
      break;
    } // End switch
    parameter = buttonReturnValue;
  } // End if
}

/*****
  Purpose: Input command using Buttons.  Has a while() loop.

  Parameter list:
    const std::string parameterName          Name of the parameter being adjusted.
    std::vector<std::string> selectionList   An array of the possible options.
    uint32_t &parameter                      Reference to the currently set parameter.

  Return value;
    void
*****/
void Button::InputCommandButton(const std::string parameterName, std::vector<std::string> selectionList)
{
  //  int centerLine = (MAX_WATERFALL_WIDTH + SPECTRUM_LEFT_X) / 2;
  bool notDone{true};
  int32_t buttonReturnValue{0}; // Used as an index of selectionList, so start with current value.

  String stringF;
  MenuSelect menu = MenuSelect::DEFAULT;

  while (notDone)
  {
    menu = readButton(); // Read the button push.
    if (menu != MenuSelect::BOGUS_PIN_READ)
    { // Valid choice?
      switch (menu)
      {
      case MenuSelect::MENU_OPTION_SELECT: // They made a choice
        // Find index of user-selected value.
        stringIterator = std::find(selectionList.begin(), selectionList.end(), selectionList[buttonReturnValue]);
        notDone = false; // Exit while.
        break;

      case MenuSelect::MAIN_MENU_UP:
        buttonReturnValue++;
        if (buttonReturnValue >= static_cast<int32_t>(selectionList.size()))
          buttonReturnValue = 0;
        break;

      case MenuSelect::MAIN_MENU_DN:
        buttonReturnValue--;
        if (buttonReturnValue < 0)
          buttonReturnValue = selectionList.size() - 1;
        break;

      case MenuSelect::BAND_UP: // Exit without change.
        notDone = false;        // Exit while.
        break;

      default:
        buttonReturnValue = -1; // An error selection
        break;
      }
      if (buttonReturnValue != -1)
      {
        //        tft.setTextColor(RA8875_WHITE, RA8875_BLACK);
        //        tft.setCursor(WATERFALL_LEFT_X + 5, SPECTRUM_TOP_Y + 50);
        //        tft.print(parameterName.c_str());
        //        tft.setTextColor(RA8875_RED, RA8875_BLACK);
        // Erase the old value before printing the new value.
        //        tft.setCursor(WATERFALL_LEFT_X + 5 + tft.getFontWidth() * (parameterName.length() + 1), SPECTRUM_TOP_Y + 50);
        //        tft.print(eraserString.c_str());
        //        tft.setCursor(WATERFALL_LEFT_X + 5 + tft.getFontWidth() * (parameterName.length() + 1), SPECTRUM_TOP_Y + 50);
        //        tft.print(selectionList[buttonReturnValue].c_str());
      }
    }
  } // End while.
}

/*****
  Purpose: Input Parameter using Encoder.  Has a while loop.

  Parameter list:
    const std::string parameterName          Name of the parameter being adjusted.
    std::vector<std::string> selectionList   An array of the possible options.
    uint32_t &parameter                      Reference to the currently set parameter.
                                             This is modified by this function!
  Return value;
    void
    Base Code courtesy of Harry  GM3RVL
*****/
void Button::InputParameterEncoderFloat(float32_t minValue, float32_t maxValue, float32_t increment, const std::string parameterName, float32_t &parameter)
{
  parameterAdjustFlag = true; // So filter bandwidth not changed!
  bool notDone{true};
  float32_t currentValue{0};

  buttonParameterName = parameterName;
  currentValue = parameter;       // Keep the original value in case the user exits without change.
  encoderAdjustFloat = parameter; // This value will be manipulated by the encoder.

  MenuSelect menu = MenuSelect::DEFAULT;

  while (notDone)
  {

    evedisplay.drawEncoderEntryScreen(true); // Write a float.
    delay(20);
    if (filterEncoderMove != 0)
    {
      encoderAdjustFloat = encoderAdjustFloat + filterEncoderMove * increment; // Bump up or down...
      if (encoderAdjustFloat < minValue)
        encoderAdjustFloat = minValue;
      if (encoderAdjustFloat > maxValue)
        encoderAdjustFloat = maxValue;

      filterEncoderMove = 0;
    }

    menu = readButton();
    if (menu == MenuSelect::MENU_OPTION_SELECT)
    {

      notDone = false; // Exit.
      evemenucontrol.subMenuSelect = false;
      evemenucontrol.runOptionFunction = false;
      //     evemenucontrol.subMenuSelect = false;
      evedisplay.screenSelect = EVE_Display::Screens::receiver;
      parameter = encoderAdjustFloat; // Save the adjusted value to the parameter.
      parameterAdjustFlag = false;
      menuProc.subMenuChoice = 0;
      calibrateFlag = false;
    }
    if (menu == MenuSelect::BAND_UP)
    {
      notDone = false;          // Exit without change.
      parameter = currentValue; // Restore original value.
      evemenucontrol.subMenuSelect = false;
      evemenucontrol.runOptionFunction = false;
      //      evemenucontrol.subMenuSelect = false;
      evedisplay.screenSelect = EVE_Display::Screens::receiver;
      parameterAdjustFlag = false;
      menuProc.subMenuChoice = 0;
      calibrateFlag = false;
    }
  }
}

/*****
  Purpose: Input Parameter using Encoder.  No while loop.

  Parameter list:
    const std::string parameterName          Name of the parameter being adjusted.
    std::vector<std::string> selectionList   An array of the possible options.
    uint32_t &parameter                      Reference to the currently set parameter.
                                             This is modified by this function!
  Return value;
    void
*****/
void Button::InputParameterEncoderNoWhile(int32_t minValue, int32_t maxValue, int32_t increment, const std::string parameterName, int32_t &parameter)
{
  parameterAdjustFlag = true;
  buttonParameterName = parameterName;
  encoderAdjustInt = parameter; // This value will be manipulated by the encoder.  This is used in drawEncoderEntryScreen.
  MenuSelect menu = MenuSelect::DEFAULT;

  evedisplay.screenSelect = EVE_Display::Screens::encoderEntry;
  /*
    if (filterEncoderMove != 0)
    {
      encoderAdjustInt += filterEncoderMove * increment; // Bump up or down...
      if (encoderAdjustInt < minValue)
        encoderAdjustInt = minValue;
      else if (encoderAdjustInt > maxValue)
        encoderAdjustInt = maxValue;

      filterEncoderMove = 0;
    }
  */

  if (filterEncoderMove != 0)
  {
    parameter += filterEncoderMove * increment; // Bump up or down...
    if (parameter < minValue)
      parameter = minValue;
    else if (parameter > maxValue)
      parameter = maxValue;

    filterEncoderMove = 0;
  }

  menu = readButton();
  if (menu == MenuSelect::MENU_OPTION_SELECT)
  {
    evemenucontrol.subMenuSelect = false;
    evemenucontrol.runOptionFunction = false;
    evemenucontrol.subMenuSelect = false;
    evedisplay.screenSelect = EVE_Display::Screens::receiver;
    parameterAdjustFlag = false;
    menuProc.subMenuChoice = 0;
    displayUpdateCounter = 0;
    audioFFToffset = 100; // This a re-initialization since the receiver screen was not used.
  }
}
