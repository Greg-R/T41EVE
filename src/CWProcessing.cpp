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

// Morse decode (ALPS decoder) and other CW related utilities.

#include "SDT.h"

/*****
  Purpose: This function replaces the arm_max_float32() function that finds the maximum element in an array.
           The histograms are "fuzzy" in the sense that dits and dahs "cluster" around a maximum value rather
           than having a single max value. This algorithm looks at a given cell and the adds in the previous
           (index - 1) and next (index + 1) cells to get the total for that index.

  Parameter list:
    int32_t *array         the base address of the array to search
    int32_t elements       the number of elements of the array to examine
    int32_t *maxCount      the largest clustered value found
    int32_t *maxIndex      the index of the center of the cluster
    int32_t *firstNonZero  the first cell that has a non-zero value
    int32_t clusterSpread  tells how far previous and ahead elements are to be included in the measure.
                            Must be an odd integer > 1.

  Return value;
    void
*****/
void CWProcessing::JackClusteredArrayMax(int32_t *array, int32_t elements, int32_t *maxCount, int32_t *maxIndex, int32_t *firstNonZero, int32_t spread)
{
  int32_t i, j, clusteredIndex;
  int32_t clusteredMax, temp;

  *maxCount = '\0'; // Reset to empty
  *maxIndex = '\0';

  clusteredMax = 0;
  clusteredIndex = -1; // Now we can check for an error

  for (i = spread; i < elements - spread; i++)
  { // Start with 1 so we can look at the previous element's value
    temp = 0;
    for (j = i - spread; j <= i + spread; j++)
    {
      temp += array[j];
    }

    if (temp >= clusteredMax)
    {
      clusteredMax = temp;
      clusteredIndex = i;
    }
  }
  if (clusteredIndex > 0)
  {
    *maxCount = array[clusteredIndex];
    *maxIndex = clusteredIndex;
  }
}

/*=================  AFP10-18-22 ================
  Purpose: Select CW Filter. ConfigData.CWFilterIndex has these values:
           0 = 840Hz
           1 = 1kHz
           2 = 1.3kHz
           3 = 1.8kHz
           4 = 2kHz
           5 = Off

  Parameter list:
    void

  Return value:
    void
*****/
FLASHMEM void CWProcessing::SelectCWFilter()
{

  button.InputParameterButtonNoWhile("CW Filter:", CWFilter, ConfigData.CWFilterIndex);

  if (ConfigData.CWFilterIndex != 5)
    switchFilterSideband = true; // Sets current delimiter to FLow (high-pass filter adjust).
}

/*****
  Purpose: Select CW offset and tone frequency.

  Parameter list:
    void

  Return value:
    void
*****/
FLASHMEM void CWProcessing::SelectCWOffset()
{
  const int numCycles[4] = {6, 7, 8, 9};

  button.InputParameterButtonNoWhile("CW Offset:", CWOffsets, ConfigData.CWOffset);

  // Now generate the values for the buffer which is used to create the CW tone.
  // The values are discrete because there must be whole cycles.
  sineTone(numCycles[ConfigData.CWOffset]);                  // This is for the CW decoder only.
  cwexciter.writeSineBuffer(numCycles[ConfigData.CWOffset]); // For the CW exciter only.
}

/*****
  Purpose: Set keyer WPM.

  Parameter list:
    void

  Return value:
    void
*****/
FLASHMEM void CWProcessing::SetKeyerWPM()
{

  button.InputParameterEncoderNoWhile(5, 60, 1, "Keyer WPM:", ConfigData.currentWPM);
  SetTransmitDitLength(ConfigData.currentWPM); // Afp 09-22-22     // JJP 8/19/23
}

/***** AFP10-18-22 ================
  Purpose: to process CW specific signals

  Parameter list:
    void

  Return value:
    void

*****/
void CWProcessing::DoCWReceiveProcessing()
{ // All New AFP 09-19-22
  float goertzelMagnitude1;
  float goertzelMagnitude2;
  float32_t aveCorrResult;
  float32_t corrResultR;
  uint32_t corrResultIndexR;
  float32_t corrResultL;
  uint32_t corrResultIndexL;
  float32_t combinedCoeff; // AFP 02-06-22
  int audioTemp;           // KF5N

  arm_fir_f32(&FIR_CW_DecodeL, float_buffer_L, float_buffer_L_CW, 256); // AFP 10-25-22  Park McClellan FIR filter const Group delay
  arm_fir_f32(&FIR_CW_DecodeR, float_buffer_R, float_buffer_R_CW, 256);

  // Calculate correlation between sine and incoming signal.  AFP 02-04-22
  arm_correlate_f32(float_buffer_R_CW, 256, sinBuffer, 256, float_Corr_BufferR);
  arm_max_f32(float_Corr_BufferR, 511, &corrResultR, &corrResultIndexR);
  // Running average of corr coeff. R
  aveCorrResultR = .7 * corrResultR + .3 * aveCorrResultR;
  arm_correlate_f32(float_buffer_L_CW, 256, sinBuffer, 256, float_Corr_BufferL);
  // Get max value of correlation
  arm_max_f32(float_Corr_BufferL, 511, &corrResultL, &corrResultIndexL);
  // Running average of corr coeff. L
  aveCorrResultL = .7 * corrResultL + .3 * aveCorrResultL;
  // aveCorrResult reduced by factor of 2 to emphasize Goertzel a little more.
  aveCorrResult = (corrResultR + corrResultL) / 4.0; // Divisor was 2.0.

  // Calculate Goertzel Mahnitude of incoming signal.
  goertzelMagnitude1 = goertzel_mag(256, freq[ConfigData.CWOffset], 24000, float_buffer_L_CW); // AFP 10-25-22
  goertzelMagnitude2 = goertzel_mag(256, freq[ConfigData.CWOffset], 24000, float_buffer_R_CW); // AFP 10-25-22

  goertzelMagnitude = (goertzelMagnitude1 + goertzelMagnitude2) / 2;
  // Combine Correlation and Gowetzel Coefficients.  Tuning coefficient added.  Greg KF5N March 9, 2025
  combinedCoeff = static_cast<float32_t>(ConfigData.morseDecodeSensitivity) * aveCorrResult * goertzelMagnitude;
  //    Serial.printf("combinedCoeff = %f\n", combinedCoeff);  // Use this to tune decoder.

  //  Draw CW decode "lock" indicator.
  if (combinedCoeff > 50)
  {                  // AFP 10-26-22
    cwdecode = true; // This variable is used by EVE to draw decode box.
  }
  else
    cwdecode = false;

  if (combinedCoeff > 50)
  { // if  have a reasonable corr coeff, >50, then we have a keeper. // AFP 10-26-22
    audioTemp = 1;
  }
  else
  {
    audioTemp = 0;
  }
  //==============  acquire data on CW  ================
  CWProcessing::DoCWDecoding(audioTemp);
}

/*****
  Purpose: establish the dit length for code transmission. Crucial since
    all spacing is done using dit length

  Parameter list:
    int wpm

  Return value:
    void
*****/
void CWProcessing::SetTransmitDitLength(int wpm)
{
  transmitDitLength = 1200 / wpm; // JJP 8/19/23

  // Total audio blocks that will be output = 1 (rise) + transmit(Dit|Dah)UnshapedBlocks + 1 (fall)
  // Blocks are assumed to be 10ms long, and the number of unshaped blocks is rounded to acheive
  // the best approximation of the actual desired dit and dah times.
  // rint() rounds to the nearest integer but returns a float.
  if (transmitDitLength < 20)
  {
    transmitDitUnshapedBlocks = 0;
    transmitDahUnshapedBlocks = 0;
  }
  else
  {
    transmitDitUnshapedBlocks = (unsigned long)rint(((double)transmitDitLength - 20.0) / 10.0);
    transmitDahUnshapedBlocks = (unsigned long)rint((((double)transmitDitLength * 3.0) - 20.0) / 10.0);
  }
}

/*****
  Purpose: Select straight key or keyer.  All this does is control
           the pullups on the GPIs connected to the key or keyer paddle.
           This should be part of the menus used by the operator.

  Parameter list:
    void

  Return value:
    void
*****/

void CWProcessing::SetKeyType()
{
  int32_t currentKey{0};

  keyPressedOn = false; // Guard against already fired key interrupt.

  // What to do will depend on the current setting!
  currentKey = ConfigData.keyType;
  // Now the user selects the key type.

  // Must use a function with a while loop due to post processing.
  button.InputParameterButton("Key Type:", keyChoice, ConfigData.keyType);

  // Do post processing.
  // We're switching to a keyer paddle.
  if (currentKey == 0)
  {
    // Set the pullup on the ring.
    pinMode(KEYER_DAH_INPUT_RING, INPUT_PULLUP); // The other keyer paddle.
    // Attach the ring interrupt, because it was not needed for the straight key.
    // The tip interrupt is always attached.
    attachInterrupt(digitalPinToInterrupt(KEYER_DAH_INPUT_RING), KeyRingOn, CHANGE);
  }
  // We're switching to a straight key.
  if (currentKey == 1)
  {
    // Detach the ring interrupt.  It is not needed and can cause undesired interrupts.
    detachInterrupt(digitalPinToInterrupt(KEYER_DAH_INPUT_RING));
    pinMode(KEYER_DAH_INPUT_RING, INPUT); // Remove the pullup.
  }
  // Flip dit and dah as configured by operator.
  if (ConfigData.paddleFlip)
  { // Means right-paddle dit
    ConfigData.paddleDit = KEYER_DAH_INPUT_RING;
    ConfigData.paddleDah = KEYER_DIT_INPUT_TIP;
  }
  else
  {
    ConfigData.paddleDit = KEYER_DIT_INPUT_TIP;
    ConfigData.paddleDah = KEYER_DAH_INPUT_RING;
  }
  delay(1000);          // This is required to allow GPIOs to settle out.
  keyPressedOn = false; // Guard against already fired key interrupt.
}

/*****
  Purpose: Set up key configuration at power-up.  Don't run after power-up!  Greg Raven KF5N Oct 2025
           This function should be in setup(), just after configuration of GPIOs.

  Parameter list:
    void

  Return value:
    void
*****/
FLASHMEM void CWProcessing::SetKeyPowerUp()
{

  pinMode(KEYER_DIT_INPUT_TIP, INPUT_PULLUP); // Straight key and keyer paddle.
  pinMode(KEYER_DAH_INPUT_RING, INPUT);       // The other keyer paddle.  Don't pullup if not used.

  // Straight key, always uses tip.
  if (ConfigData.keyType == 0)
  {
    ConfigData.paddleDit = KEYER_DIT_INPUT_TIP;
    ConfigData.paddleDah = KEYER_DAH_INPUT_RING;
    // Attach the interrupt to the tip.  Ring doesn't need an interrupt.
    attachInterrupt(digitalPinToInterrupt(KEYER_DIT_INPUT_TIP), KeyTipOn, CHANGE);
  }

  // Keyer paddle.
  if (ConfigData.keyType == 1)
  {
    pinMode(KEYER_DAH_INPUT_RING, INPUT_PULLUP); // Activate pullup on dah.
    attachInterrupt(digitalPinToInterrupt(KEYER_DIT_INPUT_TIP), KeyTipOn, CHANGE);
    attachInterrupt(digitalPinToInterrupt(KEYER_DAH_INPUT_RING), KeyRingOn, CHANGE);
    // Flip dit and dah if so configured.
    if (ConfigData.paddleFlip)
    { // Means right-paddle dit
      ConfigData.paddleDit = KEYER_DAH_INPUT_RING;
      ConfigData.paddleDah = KEYER_DIT_INPUT_TIP;
    }
    else
    {
      ConfigData.paddleDit = KEYER_DIT_INPUT_TIP;
      ConfigData.paddleDah = KEYER_DAH_INPUT_RING;
    }
  }
}

/*****
  Purpose: Allow user to set the sidetone volume.
           Sidetone volume is set separately for speaker and headphone.
           Headphone volume has to be set in the Audio Adapter hardware.
           T41EVE makes this a special function because is has to run while
           generating a sidetone.

  Parameter list:
    bool speaker (true for speaker, false for headphone)

  Return value;
    void
*****/
void CWProcessing::SetSideToneVolume(bool speaker)
{
  int sidetoneDisplay;
  bool keyDown;
  MenuSelect menu;
  RadioState temp = radioState;

  button.buttonParameterName = "Sidetone Volume:";

  SetAudioOperatingState(RadioState::CW_TRANSMIT_STRAIGHT_STATE);
  if (speaker)
    sidetoneDisplay = ConfigData.sidetoneSpeaker;
  else
    sidetoneDisplay = ConfigData.sidetoneHeadphone;
  keyDown = false;

  // This is going to run the CW transmitter, however, the power amplifier is not enabled.
  while (true)
  {

    evedisplay.drawEncoderEntryScreen(false); // Write an integer.

    // Run the CW exciter a few times to fill the queues.
    // This keeps the audio going while the display refreshes.
    for (int i = 0; i < 20; i = i + 1)
    {
      if (digitalRead(KEYER_DIT_INPUT_TIP) == LOW)
      {
        if (keyDown)
        {
          cwexciter.CW_ExciterIQData(CW_SHAPING_NONE);
        }
        else
        {
          cwexciter.CW_ExciterIQData(CW_SHAPING_RISE);
          keyDown = true;
        }
      }
      else
      {
        if (keyDown)
        {
          cwexciter.CW_ExciterIQData(CW_SHAPING_FALL);
          keyDown = false;
        }
      }
    }

    if (filterEncoderMove != 0)
    {
      sidetoneDisplay = sidetoneDisplay + filterEncoderMove; // * 0.001;  // ConfigData.sidetoneVolume range is 0.0 to 1.0 in 0.001 steps.  KF5N August 29, 2023
      if (sidetoneDisplay < 0)
        sidetoneDisplay = 0;
      else if (sidetoneDisplay > 100) // 100% max
        sidetoneDisplay = 100;
      if (speaker)
        ConfigData.sidetoneSpeaker = sidetoneDisplay;
      else
        ConfigData.sidetoneHeadphone = sidetoneDisplay;
      filterEncoderMove = 0;
    }
    button.encoderAdjustInt = sidetoneDisplay;
    speakerVolume.setGain(volumeLog[ConfigData.sidetoneSpeaker]);
    // This is the only practical way to set headphone sidetone volume.
    sgtl5000_1.volume(static_cast<float32_t>(ConfigData.sidetoneHeadphone) / 100.0); // This control has a range of 0.0 to 1.0.
    menu = button.readButton();
    if (menu == MenuSelect::MENU_OPTION_SELECT)
    { // Exit
      evemenucontrol.runOptionFunction = false;
      parameterAdjustFlag = false; // OK to write to EEPROM upon exiting.
      menuProc.subMenuChoice = 0;
      break;
    }
  } // end while loop
  SetAudioOperatingState(temp); // Restore the radio state.
  keyPressedOn = false;         // Prevent the transmitter from being keyed upon exit.
}

/*****
    DB2OO, 29-AUG-23: added
  Purpose: This function clears the morse code text buffer

  Parameter list: void

  Return value
    void
*****/
void CWProcessing::MorseCharacterClear(void)
{
  col = 0;
  decodeBuffer[col] = '\0'; // Make it a string
}

/*****
  Purpose: This function displays the decoded Morse code below waterfall. Arranged as:

  Parameter list:
    char currentLetter

  Return value
    void
*****/
void CWProcessing::MorseCharacterDisplay(char currentLetter)
{

  if (col < MAX_DECODE_CHARS)
  { // Start scrolling??
    decodeBuffer[col] = currentLetter;
    col++;
    decodeBuffer[col] = '\0'; // Make it a string.
  }
  else
  {
    // DB2OO, 25-AUG-23: use memmove instead of memcpy(), to avoid the warning
    memmove(decodeBuffer, &decodeBuffer[1], MAX_DECODE_CHARS - 1); // Slide array down 1 character.
    decodeBuffer[col - 1] = currentLetter;                         // Add to end.
    decodeBuffer[col] = '\0';                                      // Make it a string.
  }
}

/*****
  Purpose: This function uses the current WPM to set an estimate ditLength any time the tune
           encoder is changed

  Parameter list:
    void

  Return value
    void
*****/
void CWProcessing::ResetHistograms()
{
  gapAtom = transmitDitLength;
  gapChar = dahLength = transmitDitLength * 3;
  thresholdGeometricMean = (transmitDitLength + dahLength) / 2; // Use simple mean for starters so we don't have 0
  aveDitLength = transmitDitLength;
  aveDahLength = dahLength;
  valRef1 = 0;
  valRef2 = 0;
  // Clear graph arrays
  memset(signalHistogram, 0, HISTOGRAM_ELEMENTS * sizeof(uint32_t));
  memset(gapHistogram, 0, HISTOGRAM_ELEMENTS * sizeof(uint32_t));
}

/*****
  Purpose: This function creates a distribution of the gaps between signals, expressed
           in milliseconds. The result is a tri-modal distribution around three timings:
            1. inter-atom time (one dit length)
            2. inter-character (three dit lengths)
            3. word end (seven dit lengths)

  Parameter list:
    long val  The duration of the signal gap (ms).

  Return value;
    void
*****/
void CWProcessing::DoGapHistogram(uint32_t gapLen)
{
  int32_t tempAtom, tempChar;
  int32_t atomIndex, charIndex, firstDit, temp;
  uint32_t offset;

  if (gapHistogram[gapLen] > 10)
  { // Need over 1 so we don't have fractional value
    for (int k = 0; k < HISTOGRAM_ELEMENTS; k++)
    {
      gapHistogram[k] = (uint32_t)(.8 * (float)gapHistogram[k]);
    }
  }

  gapHistogram[gapLen]++; // Add new signal to distribution

  atomIndex = charIndex = 0;
  if (gapLen <= thresholdGeometricMean)
  {                                                                                                                      // Find new dit length
    JackClusteredArrayMax(gapHistogram, (uint32_t)thresholdGeometricMean, &tempAtom, &atomIndex, &firstDit, (int32_t)1); // Find max dit gap
    if (atomIndex)
    { // if something found
      gapAtom = atomIndex;
    }
    for (int j = 0; j < HISTOGRAM_ELEMENTS; j++)
    { // count down
      if (gapHistogram[HISTOGRAM_ELEMENTS - j] > 0 && endGapFlag == 0)
      { // Look for non-zero entries in the histogram
        if (HISTOGRAM_ELEMENTS - j < gapAtom * 2)
        {                                       // limit search to probable gapAtom entries
          topGapIndex = HISTOGRAM_ELEMENTS - j; // Upper end of gapAtom range
          endGapFlag = 1;                       // set flag so we know tha this is the top of the gapAtom range
        }
      }
      if (topGapIndex > 2 * gapAtom)
        topGapIndex = topGapIndexOld; // discard outliers
    }
    endGapFlag = 0;               // reset flag
    topGapIndexOld = topGapIndex; // Keep good value for reference
  }
  else
  { // dah calculation
    if (gapLen <= thresholdGeometricMean * 2)
    {
      offset = (uint32_t)(thresholdGeometricMean * 2); // Find number of elements to check
      JackClusteredArrayMax(&gapHistogram[(int32_t)thresholdGeometricMean + 1], offset, &tempChar, &charIndex, &temp, (int32_t)3);
      if (charIndex) // if something found
        gapChar = charIndex;
    }
  }
  if (atomIndex)
  {
    gapAtom = atomIndex;
  }
  if (charIndex)
  {
    gapChar = charIndex;
  }
}

/* This function was re-factored into a state machine by KF5N October 29, 2023.
  Purpose: Called when in CW mode and decoder flag is set. Function assumes:

      dit           = 1
      dah           = dit * 3
      inter-atom    = dit
      inter-letter  = dit * 3
      inter-word    = dit * 7

      You can distinguish between dah and inter-letter by presence/absence of signal. Same for inter-atom.

  Parameter list:
    int audioValue        the strength of audio signal

  Return value;
    void
*****/
void CWProcessing::DoCWDecoding(int audioValue)
{

  // This for loop runs twice.
  for (int i = 0; i < 2; i = i + 1)
  {
    switch (decodeStates)
    {
    // State 0.  Detects start of signal and starts timer.
    case state0:
      // Detect signal and redirect to appropriate state.
      if (audioValue == 1)
      {
        signalStart = millis();              // Time stamp beginning of signal.
        gapLength = signalStart - signalEnd; // Calculate the time gap between the start of this new signal and the end of the last one.
                                             //       Serial.printf("gapLength state0 = %d\n", gapLength);
        if (gapLength > LOWEST_ATOM_TIME and gapLength < (uint32_t)(thresholdGeometricMean * 3))
        {                            // range  LOWEST_ATOM_TIME = 20
          DoGapHistogram(gapLength); // Map the gap in the signal
        }
        decodeStates = state1; // Go to "signalStart" state.
        break;                 // Go to state1;
      }
      noSignalTimeStamp = millis();
      interElementGap = noSignalTimeStamp - signalEnd;
      if ((interElementGap > (gapAtom * 2)) and charProcessFlag)
      {                        // use thresholdGeometricMean??? was ditLength. End of character!  65 * 2
        decodeStates = state3; // Character ended, print it!
        break;
      }
      if (interElementGap > (gapAtom * 5) and not blankFlag and not charProcessFlag)
      { // A big gap, print a blank, but don't repeat a blank.  85 * 3.5
        decodeStates = state4;
        break;
      }
      decodeStates = state0; // Stay in state0; no signal.
      break;                 // End state0
    case state1:             // This state times a signal and measures its duration.  The next state determines if the signal is a dit or a dah.
      if (audioValue == 0)
      {
        currentTime = millis();
        signalElapsedTime = currentTime - signalStart; // Calculate the duration of the signal.
                                                       // Serial.printf("signalElapsedTime = %d dahLength = %d\n", signalElapsedTime, dahLength);
        // Ignore short noisy signal bursts:
        if (signalElapsedTime < LOWEST_ATOM_TIME)
        {                        // A hiccup or a real signal?  Make this a fraction of ditLength instead???
          decodeStates = state0; // False signal, start over.
          break;
        } // 750
        if (signalElapsedTime > LOWEST_ATOM_TIME and signalElapsedTime < HISTOGRAM_ELEMENTS)
        {                                       // Valid elapsed time?
          DoSignalHistogram(signalElapsedTime); // Yep
        }
        signalEnd = currentTime; // Time gap to next signal.
        decodeStates = state2;   // Proceed to state2.  A timed signal is available and must be processed.
        break;
      }
      decodeStates = state1; // Signal still present, stay in state1.
      break;                 // End state1

    case state2: // Determine if a timed signal was a dit or a dah and increment the decode tree.
                 //        if (signalElapsedTime > (0.5 * transmitDitLength) and signalElapsedTime < (1.5 * dahLength)) {  // All this does is provide a wide boundary for dit and dah lengths.
      if (signalElapsedTime < (1.5 * dahLength))
      {
        currentDashJump = currentDashJump >> 1; // Fast divide by 2
        if (signalElapsedTime < static_cast<int32_t>(thresholdGeometricMean))
        { // It was a dit
          // Serial.printf("thresholdGeometricMean = %d\n", static_cast<int32_t>(thresholdGeometricMean));
          charProcessFlag = true;
          currentDecoderIndex++;
          // Serial.printf("dit\n");
        }
        else
        { // It's a dah!
          // Serial.printf("dah\n");
          charProcessFlag = true;
          currentDecoderIndex += currentDashJump;
        }
      }
      decodeStates = state0; // Begin process again.
      break;                 // End state2
    case state3:
      MorseCharacterDisplay(bigMorseCodeTree[currentDecoderIndex]); // This always prints.  How do blanks get printed.
      currentDecoderIndex = 0;                                      // Reset everything if char or word
      currentDashJump = DECODER_BUFFER_SIZE;
      charProcessFlag = false; // Char printed and no longer in progress.
      decodeStates = state0;   // Start process for next incoming character.
      blankFlag = false;

      break;     // End state5
    case state4: //  Blank printing state.
      MorseCharacterDisplay(' ');

      // Compute estimated WPM and write to variable.
      estimatedWPM = 1200L / (dahLength / 3);

      blankFlag = true;
      decodeStates = state0; // Start process for next incoming character.
      break;
    default:
      break;
    }
  }
}

/*****
  Purpose: This function creates a distribution of the dit and dahs lengths, expressed in
  milliseconds. The result is a bi-modal distribution around those two timings. The
  modal value is then used for the timing of the decoder. The range should be between 20
  (60wpm) and 240 (5wpm)

  Parameter list:
  long val        the strength of audio signal

  Return value;
  void

*****/
void CWProcessing::DoSignalHistogram(long val)
{
  float compareFactor = 2.0;
  int32_t firstNonEmpty;
  int32_t tempDit, tempDah;
  int32_t offset;

  if (valFlag == 0)
  {
    valRef1 = signalElapsedTime;
    signalStartOld = millis();
    valFlag = 1;
  }

  if (millis() - signalStartOld > LOWEST_ATOM_TIME && valFlag == 1)
  {
    gapRef1 = gapLength;
    valRef2 = signalElapsedTime;
    valFlag = 0;
  }

  if ((valRef2 >= valRef1 * compareFactor && gapRef1 <= valRef1 * compareFactor) || (valRef1 >= valRef2 * compareFactor && gapRef1 <= valRef2 * compareFactor))
  {
    // See if consecutive signal lengths in approximate dit to dah ratio and which one is larger
    if (valRef2 >= valRef1)
    {
      aveDitLength = (long)(0.9 * aveDitLength + 0.1 * valRef1); // Do some dit length averaging
      aveDahLength = (long)(0.9 * aveDahLength + 0.1 * valRef2);
    }
    else
    {
      aveDitLength = (long)(0.9 * aveDitLength + 0.1 * valRef2); // Use larger one. Note reversal of calc order
      aveDahLength = (long)(0.9 * aveDahLength + 0.1 * valRef1); // Do some dah length averaging
    }
  }
  thresholdGeometricMean = sqrt(aveDitLength * aveDahLength);   // calculate geometric mean
  thresholdArithmeticMean = (aveDitLength + aveDahLength) >> 1; // Fast divide by 2 on integer data

  signalHistogram[val]++; // Don't care which half it's in, just put it in

  offset = (uint32_t)thresholdGeometricMean - 1; // Only do cast once
  // Dit calculation
  // 2nd parameter means we only look for dits below the geomean.

  for (int32_t j = (int32_t)thresholdGeometricMean; j; j--)
  {
    if (signalHistogram[j] != 0)
    {
      firstNonEmpty = j;
      break;
    }
  }

  JackClusteredArrayMax(signalHistogram, offset, &tempDit, (int32_t *)&transmitDitLength, &firstNonEmpty, (int32_t)1);
  // dah calculation
  // Elements above the geomean. Note larger spread: higher variance
  JackClusteredArrayMax(&signalHistogram[offset], HISTOGRAM_ELEMENTS - offset, &tempDah, (int32_t *)&dahLength, &firstNonEmpty, (uint32_t)3);
  dahLength += (uint32_t)offset;

  if (tempDit > SCALE_CONSTANT && tempDah > SCALE_CONSTANT)
  { // Adaptive dit signalHistogram[]
    for (int k = 0; k < HISTOGRAM_ELEMENTS; k++)
    {
      signalHistogram[k] = ADAPTIVE_SCALE_FACTOR * signalHistogram[k];
    }
  }
}

/*****
  Purpose: Calculate Goertzel Algorithn to enable decoding CW

  Parameter list:
    int numSamples,         // number of sample in data array
    int TARGET_FREQUENCY,   // frequency for which the magnitude of the transform is to be found
    int SAMPLING_RATE,      // Sampling rate in our case 24ksps
    float* data             // pointer to input data array

  Return value;
    float magnitude     //magnitude of the transform at the target frequency

*****/
float CWProcessing::goertzel_mag(int numSamples, int TARGET_FREQUENCY, int SAMPLING_RATE, float *data)
{
  int k, i;
  float floatnumSamples;
  float omega, sine, cosine, coeff, q0, q1, q2, magnitude, real, imag;

  float scalingFactor = numSamples / 2.0;

  floatnumSamples = (float)numSamples;
  k = (int)(0.5 + ((floatnumSamples * TARGET_FREQUENCY) / SAMPLING_RATE));
  omega = (2.0 * M_PI * k) / floatnumSamples;
  sine = sin(omega);
  cosine = cos(omega);
  coeff = 2.0 * cosine;
  q0 = 0;
  q1 = 0;
  q2 = 0;

  for (i = 0; i < numSamples; i++)
  {
    q0 = coeff * q1 - q2 + data[i];
    q2 = q1;
    q1 = q0;
  }
  real = (q1 - q2 * cosine) / scalingFactor; // calculate the real and imaginary results scaling appropriately
  imag = (q2 * sine) / scalingFactor;

  magnitude = sqrtf(real * real + imag * imag);
  return magnitude;
}

/*****
  Purpose: This option reverses the dit and dah paddles on the keyer.

  Parameter list:
    void

  Return value
    void
*****/
void CWProcessing::DoPaddleFlip()
{
  std::vector<std::string> paddleState = {"Right paddle = dah", "Right paddle = dit"};

  ConfigData.paddleDah = KEYER_DAH_INPUT_RING; // Defaults
  ConfigData.paddleDit = KEYER_DIT_INPUT_TIP;

  // Must use a function with a while loop due to post processing.
  button.InputParameterButton("Paddle Flip:", paddleState, ConfigData.paddleFlip);

  // Do post processing.
  if (ConfigData.paddleFlip)
  { // Means right-paddle dit
    ConfigData.paddleDit = KEYER_DAH_INPUT_RING;
    ConfigData.paddleDah = KEYER_DIT_INPUT_TIP;
    ConfigData.paddleFlip = 1; // KD0RC
  }
  else
  {
    ConfigData.paddleDit = KEYER_DIT_INPUT_TIP;
    ConfigData.paddleDah = KEYER_DAH_INPUT_RING;
    ConfigData.paddleFlip = 0; // KD0RC
  }
  delay(1000);          // This is required to allow GPIOs to settle out.
  keyPressedOn = false; // Guard against already fired key interrupt.
}
