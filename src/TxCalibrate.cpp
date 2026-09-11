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

// Class TxCalibrate.  Greg KF5N July 10, 2024
// To EVE June 2026.

#include "SDT.h"

/*****
  Purpose: Run MakeFFTData() a few times to load and settle out buffers.  KF5N May 22, 2024
           Compute FFT in order to find maximum signal peak prior to beginning calibration.
  Parameter list:
    void

  Return value:
    void
*****/
elapsedMillis txTimer;
void TxCalibrate::warmUpCal()
{
  uint32_t index_of_max{0};
  uint32_t count{0};
  uint32_t i;

  // MakeFFTData() has to be called enough times for transients to settle out before computing FFT.
  for (i = 0; i < 32; i = i + 1)
  {
//    fftActive = true;
    updateDisplayFlag = true;
    TxCalibrate::MakeFFTData(); // Note, FFT not called if buffers are not sufficiently filled.

    arm_max_q15(pixelnew, 512, &rawSpectrumPeak, &index_of_max);
    if (index_of_max > 251 and index_of_max < 260)
    { // The peak is in the correct bin?
      count = count + 1;
    }
    else
      count = 0; // Reset count in case of failure.
    if (count == 5)
      break; // If five in a row, exit the loop.  Warm-up is complete.
  } // End peak detection loop.

//  fftActive = true;
  updateDisplayFlag = false;
  // Find peak of spectrum, which is 512 wide.  Use this to adjust spectrum peak to top of spectrum display.
  arm_max_q15(pixelnew, 512, &rawSpectrumPeak, &index_of_max);
  if (index_of_max < 251 or index_of_max > 260)
  {
    Serial.printf("Problem with TX warmUpCal\n");
    Serial.printf("index_of_max = %d\n", index_of_max);
  }
  ADC_RX_I.clear();
  ADC_RX_Q.clear();
  Q_in_L_Ex.clear();
  Q_in_R_Ex.clear();
}

/*****
  Purpose: Set up prior to IQ calibrations.  New function.  KF5N August 14, 2023
  These things need to be saved here and restored in the epilogue function:
  Vertical scale in dB  (set to 10 dB during calibration)
  Zoom, set to 1X in receive and 4X in transmit calibrations.
  Transmitter power, set to 5W during both calibrations.
   Parameter List:
      int setZoom   (This parameter should be 0 for receive (1X) and 2 (4X) for transmit)

   Return value:
      void
 *****/
void TxCalibrate::CalibratePreamble(int setZoom)
{
  controlAudioOut(ConfigData.audioOut, true); // Mute all receiver audio.
  calOnFlag = true;                           // Used for the special display during calibration and also high-dynamic range FFT.
  exitManual = false;
  transmitPowerLevelTemp = ConfigData.transmitPowerLevel; // AFP 05-11-23
  cwFreqOffsetTemp = ConfigData.CWOffset;
  // Remember the mode and state, and restore in the Epilogue.
  tempMode = bands.bands[ConfigData.currentBand].mode;
  tempState = radioState;
  if (mode == 0)
    bands.bands[ConfigData.currentBand].mode = RadioMode::CW_MODE;
  else
    bands.bands[ConfigData.currentBand].mode = RadioMode::SSB_MODE;
  // Calibrate requires upper or lower sideband.  Change if currently in an AM mode.  Put back in Epilogue.
  if (bands.bands[ConfigData.currentBand].sideband == Sideband::BOTH_AM or bands.bands[ConfigData.currentBand].sideband == Sideband::BOTH_SAM)
  {
    tempSideband = bands.bands[ConfigData.currentBand].sideband;
    // Use the last upper or lower sideband.
    bands.bands[ConfigData.currentBand].sideband = ConfigData.lastSideband[ConfigData.currentBand];
  }
  else
    tempSideband = bands.bands[ConfigData.currentBand].sideband;
  ConfigData.CWOffset = 2;                  // 750 Hz for TX calibration.  Epilogue restores user selected offset.
                                            //  userxmtMode = ConfigData.xmtMode;          // Store the user's mode setting.  KF5N July 22, 2023
  userZoomIndex = ConfigData.spectrum_zoom; // Save the zoom index so it can be reset at the conclusion.  KF5N August 12, 2023
  ConfigData.spectrum_zoom = setZoom;
  button.ButtonZoom();

  userScale = ConfigData.currentScale; //  Remember user preference so it can be reset when done.  KF5N
  ConfigData.currentScale = 1;         //  Set vertical scale to 10 dB during calibration.  KF5N
  updateDisplayFlag = false;
  ConfigData.centerFreq = TxRxFreq;
  NCOFreq = 0;
  digitalWrite(MUTE, MUTEAUDIO); // Mute Audio.
  digitalWrite(RXTX, HIGH);      // Turn on transmitter.
  rawSpectrumPeak = 0;
  if (mode == 0)
    radioState = RadioState::CW_CALIBRATE_STATE;
  if (mode == 1)
    radioState = RadioState::SSB_CALIBRATE_STATE;
  SetAudioOperatingState(radioState); // Do this last!  This turns the queues on.
}

/*****
  Purpose: Shut down and clean up after IQ calibrations.  New function.  KF5N August 14, 2023

   Parameter List:
      void

   Return value:
      void
 *****/
void TxCalibrate::CalibrateEpilogue()
{
  /*
  Serial.printf("lastState=%d radioState=%d memory_used=%d memory_used_max=%d f32_memory_used=%d f32_memory_used_max=%d\n",
                lastState,
                radioState,
                (int)AudioStream::memory_used,
                (int)AudioStream::memory_used_max,
                (int)AudioStream_F32::f32_memory_used,
                (int)AudioStream_F32::f32_memory_used_max);
  AudioStream::memory_used_max = 0;
  AudioStream_F32::f32_memory_used_max = 0;
 */

  digitalWrite(RXTX, LOW); // Turn off the transmitter.
  updateDisplayFlag = false;
  SampleRate = SAMPLE_RATE_192K; // Return to receiver sample rate.
  SetI2SFreq(SR[SampleRate].rate);
  InitializeDataArrays(); // Re-initialize the filters back to 192ksps.
  if (bands.bands[ConfigData.currentBand].sideband == Sideband::BOTH_AM or bands.bands[ConfigData.currentBand].sideband == Sideband::BOTH_SAM)
    bands.bands[ConfigData.currentBand].sideband = tempSideband;
  bands.bands[ConfigData.currentBand].mode = tempMode;
  ConfigData.centerFreq = TxRxFreq;
  NCOFreq = 0;
  calibrateFlag = 0;                                      // KF5N
  ConfigData.CWOffset = cwFreqOffsetTemp;                 // Return user selected CW offset frequency.
  sineTone(ConfigData.CWOffset + 6);                      // This function takes "number of cycles" which is the offset + 6.
  ConfigData.currentScale = userScale;                    //  Restore vertical scale to user preference.  KF5N
  ConfigData.transmitPowerLevel = transmitPowerLevelTemp; // Restore the user's transmit power level setting.  KF5N August 15, 2023
  if (TxCalibrate::saveToEeprom)
    eeprom.CalDataWrite(); // Save calibration numbers and configuration.  KF5N August 12, 2023
  calOnFlag = false;
  fftOffset = 0; // Some reboots may be caused by large fftOffset values when Auto-Spectrum is on.
  bands.bands[ConfigData.currentBand].sideband = tempSideband;
  lastState = RadioState::NOSTATE; // This is required due to the function deactivating the receiver.  This forces a pass through the receiver set-up code.  KF5N October 16, 2023
  radioState = tempState;
  ConfigData.spectrum_zoom = userZoomIndex;
  button.ButtonZoom(); // Restore the user's zoom setting.  Note that this function also modifies ConfigData.spectrum_zoom.
  powerUp = true;
}

// void TxCalibrate::buttonTasks(bool radioCal, bool refineCal) {  // Does this function need parameters?
void TxCalibrate::buttonTasks()
{
  task = button.readButton();
  switch (task)
  {
  // Activate initial automatic calibration.
  case MenuSelect::ZOOM: // 2nd row, 1st column button
    TxCalibrate::autoCal = true;
    warmup = 0;
    index = 0;
    state = State::warmup;
    break;
  // Automatic calibration using previously stored values.
  case MenuSelect::FILTER: // 3rd row, 1st column button
    TxCalibrate::autoCal = true;
    warmup = 0;
    index = 0;
    state = State::warmup;
    break;
  // Toggle gain and phasei in manual mode.
  case MenuSelect::UNUSED_1:
    if (IQCalType == 0)
    {
      IQCalType = 1;
      // Turn off red indication of active setting.
      if (calTypeFlag == 1)
        GetEncoderValueLive(-1.0, 1.0, amplitude, xmitIncrement);
      if (calTypeFlag == 2)
        GetEncoderValueLive(-1.0, 1.0, iDCoffset, carrIncrement);
    }
    else
    {
      IQCalType = 0;
      // Turn off red indication of active setting.
      if (calTypeFlag == 1)
        GetEncoderValueLive(-1.0, 1.0, phase, xmitIncrement);
      if (calTypeFlag == 2)
        GetEncoderValueLive(-1.0, 1.0, qDCoffset, carrIncrement);
    }
    break;

  case MenuSelect::MENU_OPTION_SELECT: // Save values and exit from manual calibration.
    exitManual = true;
    break;
  default:
    break;
  } // end switch
}

void TxCalibrate::writeToCalData(float ichannel, float qchannel)
{
  if (mode == 0)
  {
    if (bands.bands[ConfigData.currentBand].sideband == Sideband::LOWER)
    {
      CalData.IQCWAmpCorrectionFactorLSB[ConfigData.currentBand] = ichannel;
      CalData.IQCWPhaseCorrectionFactorLSB[ConfigData.currentBand] = qchannel;
    }
    else if (bands.bands[ConfigData.currentBand].sideband == Sideband::UPPER)
    {
      CalData.IQCWAmpCorrectionFactorUSB[ConfigData.currentBand] = ichannel;
      CalData.IQCWPhaseCorrectionFactorUSB[ConfigData.currentBand] = qchannel;
    }
  }
  if (mode == 1)
  {
    if (bands.bands[ConfigData.currentBand].sideband == Sideband::LOWER)
    {
      CalData.IQSSBAmpCorrectionFactorLSB[ConfigData.currentBand] = ichannel;
      CalData.IQSSBPhaseCorrectionFactorLSB[ConfigData.currentBand] = qchannel;
    }
    else if (bands.bands[ConfigData.currentBand].sideband == Sideband::UPPER)
    {
      CalData.IQSSBAmpCorrectionFactorUSB[ConfigData.currentBand] = ichannel;
      CalData.IQSSBPhaseCorrectionFactorUSB[ConfigData.currentBand] = qchannel;
    }
  }

  // Apply amplitude and phase corrections.
  AudioNoInterrupts();
  if (TxCalibrate::mode == 0)
  {
    if (bands.bands[ConfigData.currentBand].sideband == Sideband::LOWER)
      cessb1.setIQCorrections(true, CalData.IQCWAmpCorrectionFactorLSB[ConfigData.currentBandA], CalData.IQCWPhaseCorrectionFactorLSB[ConfigData.currentBandA], 0.0);
    if (bands.bands[ConfigData.currentBand].sideband == Sideband::UPPER)
      cessb1.setIQCorrections(true, CalData.IQCWAmpCorrectionFactorUSB[ConfigData.currentBandA], CalData.IQCWPhaseCorrectionFactorUSB[ConfigData.currentBandA], 0.0);
  }
  if (TxCalibrate::mode == 1)
  {
    if (bands.bands[ConfigData.currentBand].sideband == Sideband::LOWER)
      cessb1.setIQCorrections(true, CalData.IQSSBAmpCorrectionFactorLSB[ConfigData.currentBandA], CalData.IQSSBPhaseCorrectionFactorLSB[ConfigData.currentBandA], 0.0);
    if (bands.bands[ConfigData.currentBand].sideband == Sideband::UPPER)
      cessb1.setIQCorrections(true, CalData.IQSSBAmpCorrectionFactorUSB[ConfigData.currentBandA], CalData.IQSSBPhaseCorrectionFactorUSB[ConfigData.currentBandA], 0.0);
  }
  AudioInterrupts();
}

/*****
  Purpose: Combined input/output for the purpose of calibrating the transmit IQ.

   Parameter List:
      mode, bool radioCal, bool refineCal, bool saveToEeprom

   Return value:
      void
 *****/
void TxCalibrate::DoXmitCalibrate(int calMode, bool radio, bool toEeprom)
{
  int freqOffset = 0; // Calibration tone same as regular modulation tone.
  float maxSweepAmp = 0.1;
  float maxSweepPhase = 0.05;
  float adjdB_old{0};
  float adjdB_min{0};
  xmitIncrement = 0.01; // Coarse increment used during initial amplitude calibration.
  IQCalType = 0;        // Begin with IQ gain optimization.
  std::vector<float32_t> sweepVector(21);
  std::vector<float32_t> sweepVectorValue(21);
  elapsedMillis fiveSeconds;
  int startTimer = 0;
  TxCalibrate::autoCal = false;
  TxCalibrate::mode = calMode;          // CW or SSB.  This is an object state variable.
  TxCalibrate::radioCal = radio;        // Initial calibration of all bands.
  TxCalibrate::saveToEeprom = toEeprom; // Save to EEPROM
  std::vector<float>::iterator result;
  TxCalibrate::CalibratePreamble(2); // Set zoom to 4X.  Sample rate 48ksps.
  calTypeFlag = 1;                   // TX sideband
  int refinePass{0};

  SetFreqCal(freqOffset);
  // Get current values into the amplitude and phase working variables.
  if (mode == 0)
  {
    if (bands.bands[ConfigData.currentBand].sideband == Sideband::LOWER)
    {
      amplitude = CalData.IQCWAmpCorrectionFactorLSB[ConfigData.currentBand];
      phase = CalData.IQCWPhaseCorrectionFactorLSB[ConfigData.currentBand];
    }
    else if (bands.bands[ConfigData.currentBand].sideband == Sideband::UPPER)
    {
      amplitude = CalData.IQCWAmpCorrectionFactorUSB[ConfigData.currentBand];
      phase = CalData.IQCWPhaseCorrectionFactorUSB[ConfigData.currentBand];
    }
  }
  else if (mode == 1)
  {
    if (bands.bands[ConfigData.currentBand].sideband == Sideband::LOWER)
    {
      amplitude = CalData.IQSSBAmpCorrectionFactorLSB[ConfigData.currentBand];
      phase = CalData.IQSSBPhaseCorrectionFactorLSB[ConfigData.currentBand];
    }
    else if (bands.bands[ConfigData.currentBand].sideband == Sideband::UPPER)
    {
      amplitude = CalData.IQSSBAmpCorrectionFactorUSB[ConfigData.currentBand];
      phase = CalData.IQSSBPhaseCorrectionFactorUSB[ConfigData.currentBand];
    }
  }
  // Run this so Phase shows from beginning.  Get the value for the current sideband.
  GetEncoderValueLive(-2.0, 2.0, phase, xmitIncrement);
  warmUpCal();

  if (radioCal)
  {
    autoCal = true;
    warmup = 0;
    index = 0; // Why is index = 1???
    IQCalType = 0;
    std::fill(sweepVectorValue.begin(), sweepVectorValue.end(), 0.0);
    std::fill(sweepVector.begin(), sweepVector.end(), 0.0);
    state = State::warmup;
  }

  // Transmit Calibration Loop
  while (true)
  {
//    fftActive = true;
    computeAdjdB();
    evedisplay.drawTransmitterCalScreen(pixelnew);

    // This function takes care of button presses and resultant control of the rest of the process.
    // The buttons are polled by the while loop.
    TxCalibrate::buttonTasks();

    if (exitManual == true) // Exit from manual calibration by button push.
    {
      TxCalibrate::CalibrateEpilogue();
      return;
    }
    //  Begin automatic calibration state machine.
    if (autoCal or radioCal)
    {
      switch (state)
      {
      case State::warmup:
        autoCal = true;
        std::fill(sweepVectorValue.begin(), sweepVectorValue.end(), 0.0);
        std::fill(sweepVector.begin(), sweepVector.end(), 0.0);
        warmup = warmup + 1;
        phase = 0.0 + maxSweepPhase;   //  Need to use these values during warmup
        amplitude = 1.0 + maxSweepAmp; //  so adjdB and adjdB_avg are forced upwards.
        state = State::warmup;
        if (warmup == 1)         // Was 16.
          state = State::state0; // Proceed with initial calibration.
        break;

      case State::state0:
        // Starting values for initial calibration sweeps.  First sweep is amplitude (gain).
        phase = 0.0;                                          // Hold phase at 0.0 while amplitude sweeps.
        amplitude = 1.0 - maxSweepAmp;                        // Begin sweep at low end and move upwards.
        GetEncoderValueLive(-2.0, 2.0, phase, xmitIncrement); // Display phase value during amplitude sweep.
        adjdB = 0;
        //        adjdB_avg = 0;
        index = 0;
        IQCalType = 0;        // IQ Gain
        xmitIncrement = 0.01; // Reset in case initial cal is run twice.
        adjdB_min = 0.0;
        state = State::initialSweepAmp; // Let this fall through.

      case State::initialSweepAmp:
        sweepVectorValue[index] = amplitude;
        sweepVector[index] = adjdB;
        if ((adjdB < adjdB_min) and (adjdB < -20.0))
          adjdB_min = adjdB;
        if ((adjdB - adjdB_min) > 2)
          amplitude = maxSweepAmp;
        // Increment for next measurement.
        index = index + 1;
        amplitude = amplitude + xmitIncrement; // Next one!
        if (adjdB > -35.0)
          amplitude = amplitude + 2.0 * xmitIncrement;
        // Done with initial sweep, move to initial sweep of phase.
        if (abs(amplitude - 1.0) > maxSweepAmp)
        {                                                                    // Needs to be subtracted from 1.0.
          result = std::min_element(sweepVector.begin(), sweepVector.end()); // Value of the minimum.
          adjdBMinIndex = std::distance(sweepVector.begin(), result);        // Find the index.
          iOptimal = sweepVectorValue[adjdBMinIndex];                        // Set to the discovered minimum.
          amplitude = iOptimal;                                              // Set amplitude to the discovered optimal value.
          // Update display to optimal value and change from red to white.
          GetEncoderValueLive(-2.0, 2.0, amplitude, xmitIncrement);
          IQCalType = 1;          // Prepare for phase.
          phase = -maxSweepPhase; // The starting value for phase.
          index = 0;
          // Clear the vector before moving to phase.
          std::fill(sweepVectorValue.begin(), sweepVectorValue.end(), 0.0);
          std::fill(sweepVector.begin(), sweepVector.end(), 0.0);
          xmitIncrement = 0.01; // The initial increment can be reduced because the sweep range is half.
          adjdB_old = 0.0;
          adjdB_min = 0.0;
          state = State::initialSweepPhase; // Initial sweeps done; proceed to refine phase.
          break;
        }

        state = State::initialSweepAmp;
        break;

      case State::initialSweepPhase:
        sweepVectorValue[index] = phase;
        sweepVector[index] = adjdB;
        if ((adjdB < adjdB_min) and (adjdB < -20.0))
          adjdB_min = adjdB;
        if ((adjdB - adjdB_min) > 2)
          phase = maxSweepPhase;
        index = index + 1;
        // Increment for the next measurement.
        phase = phase + xmitIncrement;
        //        if(adjdB > 35.0) phase = phase + 2.0 * xmitIncrement;
        if (phase > maxSweepPhase)
        {
          result = std::min_element(sweepVector.begin(), sweepVector.end());
          adjdBMinIndex = std::distance(sweepVector.begin(), result);
          qOptimal = sweepVectorValue[adjdBMinIndex]; // Set to the discovered minimum.
          phase = qOptimal;                           // Set to the discovered minimum.
                                                      // Update display to optimal value and change from red to white.
          GetEncoderValueLive(-2.0, 2.0, phase, xmitIncrement);
          IQCalType = 0;
          adjdB = 0.0;
          index = 0;
          xmitIncrement = 0.01;
          refinePass = 0;
          writeToCalData(amplitude, phase); // Optimal values at end of initial amplitude and phase sweeps.
          computeAdjdB();                   // This is to flush out transient from phase last set at extreme.
          amplitude = amplitude + 0.001;    // Now increment amplitude.
          adjdB_old = adjdB;                // Must calculate current best adjdB before entering refineAmpPlus.
          state = State::refineAmpPlus;
          break;
        }
        state = State::initialSweepPhase;
        break;

      case State::refineAmpPlus:

        if (adjdB < adjdB_old)
        {
          adjdB_old = adjdB;
          amplitude = amplitude + 0.001;
          state = State::refineAmpPlus;
          break;
        }
        else // Prepare for refineAmpMinus.
        {
          amplitude = amplitude - 0.001 - 0.001; // Put back last increment and increment in minus direction.
          refinePass = refinePass + 1;
          state = State::refineAmpMinus;
          break;
        }

        break;

      case State::refineAmpMinus:

        if (adjdB < adjdB_old)
        {
          adjdB_old = adjdB;
          amplitude = amplitude - 0.001;
          state = State::refineAmpMinus;
          break;
        }
        else
        {
          amplitude = amplitude + 0.001; // Put back last step.
          phase = phase + 0.001;         // Set for refinePhasePlus.
          state = State::refinePhasePlus;
          break;
        }

        break;

      case State::refinePhasePlus:

        if (adjdB < adjdB_old)
        {
          adjdB_old = adjdB;
          phase = phase + 0.001;
          state = State::refinePhasePlus;
          break;
        }
        else
        {
          phase = phase - 0.001 - 0.001; // Put back last increment and increment in minus direction.
          state = State::refinePhaseMinus;
          break;
        }

        break;

      case State::refinePhaseMinus:

        if (adjdB < adjdB_old)
        {
          adjdB_old = adjdB;
          phase = phase - 0.001;
          state = State::refinePhaseMinus;
          break;
        }
        else
        {
          phase = phase + 0.001; // Put back last increment.
          if (refinePass == 2)
          {
            state = State::setOptimal;
          }
          else
          {
            amplitude = amplitude + 0.001;
            state = State::refineAmpPlus; // Proceed to next pass.
          }
          break;
        }

        break;

      case State::setOptimal:
        // Write the optimal values to the data structure.
        writeToCalData(iOptimal, qOptimal);
        state = State::exit;
        startTimer = static_cast<int>(milliTimer); // Start result view timer.
        break;
      case State::exit:
        // Delay exit if in radio calibration to show calibration results, and then exit.
        if (radioCal)
        {
          if ((static_cast<int>(milliTimer) - startTimer) < 100)
          { // Show calibration result at conclusion during Radio Cal.
            state = State::exit;
            break;
          }
          else
          { // Clean up and return.
            TxCalibrate::CalibrateEpilogue();
            return;
          }
          //  Not in radioCal, but was in autoCal, and now need to return to manual cal.
        }
        else
        {
          autoCal = false; // Don't enter switch, but remain in manual loop.
        }
        break;
      }
    } // end automatic calibration state machine

    task = MenuSelect::DEFAULT; // Reset task after it is used.
                                //  Read encoder and update values.
    if (IQCalType == 0)
      amplitude = GetEncoderValueLive(-2.0, 2.0, amplitude, 0.001);
    if (IQCalType == 1)
      phase = GetEncoderValueLive(-2.0, 2.0, phase, 0.001);
    writeToCalData(amplitude, phase);
  } // end while
} // End Transmit calibration

/*****
  Purpose: Combined input/output for the purpose of calibrating the transmit IQ.

   Parameter List:
      int mode, bool radioCal, bool refineCal, bool saveToEeprom

   Return value:
      void
 *****/

void TxCalibrate::DoXmitCarrierCalibrate(int calMode, bool radio, bool toEeprom)
{
  float32_t maxSweepAmp = 0.1;
  float32_t maxSweepPhase = 0.05;
  float adjdB_old{0};
  float adjdB_min{0};
  carrIncrement = 0.005;       // Initial carrier increment.
  IQCalType = 0;               // Begin with I channel offset.
  TxCalibrate::mode = calMode; // CW or SSB
  TxCalibrate::radioCal = radio;
  TxCalibrate::saveToEeprom = toEeprom;   // Save to EEPROM
  std::vector<float32_t> sweepVector(41); // 0 + 450 * 2 / 5
  std::vector<float32_t> sweepVectorValue(41);
  std::vector<float>::iterator result;
  int startTimer = 0;
  int refinePass{0};
  TxCalibrate::CalibratePreamble(2); // Set zoom to 4X.  Note this is using 48ksps sample rate.
  int freqOffset = 0;                // Calibration tone same as regular modulation tone.
  calTypeFlag = 2;                   // Carrier calibration
  radioState = RadioState::SSB_TRANSMIT_STATE;
  SetFreqCal(freqOffset);
  // Get current values into the iDCoffset and qDCoffset working variables.
  if (mode == 0)
  {
    iDCoffset = CalData.iDCoffsetCW[ConfigData.currentBand];
    qDCoffset = CalData.qDCoffsetCW[ConfigData.currentBand];
  }
  if (mode == 1)
  {
    iDCoffset = CalData.iDCoffsetSSB[ConfigData.currentBand];
    qDCoffset = CalData.qDCoffsetSSB[ConfigData.currentBand];
  }
  // Run this so Q offset shows from begining.
  GetEncoderValueLive(-1.0, 1.0, qDCoffset, carrIncrement);
  warmUpCal();

  if (radioCal)
  {
    autoCal = true;
    warmup = 0;
    index = 0;
    IQCalType = 0;
    std::fill(sweepVectorValue.begin(), sweepVectorValue.end(), 0.0);
    std::fill(sweepVector.begin(), sweepVector.end(), 0.0);
    state = State::warmup;
  }

  // Carrier Calibration Loop
  while (true)
  {
//    fftActive = true;
    computeAdjdB();
    evedisplay.drawTransmitterCalScreen(pixelnew);
    TxCalibrate::buttonTasks(); // This takes care of manual calls to the initial or refinement calibrations.
    // Exit from manual calibration by button push.
    if (exitManual == true)
    {
      TxCalibrate::CalibrateEpilogue();
      return;
    }
    //  Begin automatic calibration state machine.
    if (autoCal or radioCal)
    {
      switch (state)
      {
      case State::warmup:
        autoCal = true;
        std::fill(sweepVectorValue.begin(), sweepVectorValue.end(), 0.0);
        std::fill(sweepVector.begin(), sweepVector.end(), 0.0);
        warmup = warmup + 1;
        qDCoffset = maxSweepPhase; //  Need to use these values during warmup
        iDCoffset = maxSweepAmp;   //  so adjdB and adjdB_avg are forced upwards.
        state = State::warmup;
        if (warmup == 1)
          state = State::state0;
        break;

      case State::state0:
        iDCoffset = -maxSweepAmp; // Begin sweep at low end and move upwards.
        qDCoffset = -maxSweepAmp; // Begin sweep at low end and move upwards.
        GetEncoderValueLive(-1.0, 1.0, qDCoffset, carrIncrement);
        index = 0;
        IQCalType = 0;
        adjdB_min = 0.0;
        carrIncrement = 0.005;          // Reset in case initial cal is run twice.
        state = State::initialSweepAmp; // Let this fall through.

      case State::initialSweepAmp:
        sweepVectorValue[index] = iDCoffset; // Starting at -maxSweepAmp.
        sweepVector[index] = adjdB;          // Already computed in warm-up for index 0.
        if ((adjdB < adjdB_min) and (adjdB < -20.0))
          adjdB_min = adjdB;
        if ((adjdB - adjdB_min) > 2.0)
          iDCoffset = maxSweepAmp;
//        Serial.printf("adjdB = %f adjdB_min = %f\n", adjdB, adjdB_min);
        index = index + 1;
        // Increment for next measurement.
        iDCoffset = iDCoffset + carrIncrement;
        qDCoffset = qDCoffset + carrIncrement;
        if (adjdB > -25.0)
        {
          iDCoffset = iDCoffset + 2.0 * carrIncrement;
          qDCoffset = qDCoffset + 2.0 * carrIncrement;
        }
        // Go to Q channel when I channel sweep is finished.
        if (iDCoffset > maxSweepAmp)
        {
          IQCalType = 1;                                                     // Get ready for phase.
          result = std::min_element(sweepVector.begin(), sweepVector.end()); // Value of the minimum.
          adjdBMinIndex = std::distance(sweepVector.begin(), result);        // Find the index.
          iOptimal = sweepVectorValue[adjdBMinIndex];                        // Set to the discovered minimum.
          iDCoffset = iOptimal;                                              // Reset for next sweep.
          qDCoffset = iOptimal;
          qOptimal = qDCoffset;
          // Update display to optimal value and change from red to white.
          GetEncoderValueLive(-1.0, 1.0, iDCoffset, carrIncrement);
          IQCalType = 1; // Prepare for phase.
          index = 0;
          adjdB = 0;
          // Clear the vector before moving to phase.
          std::fill(sweepVectorValue.begin(), sweepVectorValue.end(), 0.0);
          std::fill(sweepVector.begin(), sweepVector.end(), 0.0);
          qDCoffset = -maxSweepPhase + iDCoffset; // The starting value for phase.
          adjdB_min = 0.0;
          state = State::initialSweepPhase; // Initial I channel sweep done; proceed to initial Q sweep.
          break;
        }
        state = State::initialSweepAmp; // Continue sweeping.
        break;

      case State::initialSweepPhase:
        sweepVectorValue[index] = qDCoffset; // Start at -maxSweepPhase + iDCoffset.
        sweepVector[index] = adjdB;
        if ((adjdB < adjdB_min) and (adjdB < -20.0))
          adjdB_min = adjdB;
        if ((adjdB - adjdB_min) > 2.0)
          qDCoffset = maxSweepAmp;
        index = index + 1;
        // Increment for the next measurement.
        qDCoffset = qDCoffset + carrIncrement;
        //        if(adjdB > -25.0) {
        //        qDCoffset = qDCoffset + 2.0 * carrIncrement;
        //        }
        if (qDCoffset > maxSweepPhase)
        {
          result = std::min_element(sweepVector.begin(), sweepVector.end());
          adjdBMinIndex = std::distance(sweepVector.begin(), result);
          qOptimal = sweepVectorValue[adjdBMinIndex]; // Set to the discovered minimum.
          qDCoffset = qOptimal;                       // Set to the discovered minimum.
          // Update display to optimal value and change from red to white.
          GetEncoderValueLive(-1.0, 1.0, qDCoffset, carrIncrement);
          IQCalType = 0;
          adjdB = 0.0;
          index = 0;
          carrIncrement = 0.0005; // Manual increment.
          refinePass = 0;

          // Set optimal values to CalData.
          if (mode == 0)
          {
            CalData.iDCoffsetCW[ConfigData.currentBand] = iDCoffset;
            CalData.qDCoffsetCW[ConfigData.currentBand] = qDCoffset;
          }
          if (mode == 1)
          {
            CalData.iDCoffsetSSB[ConfigData.currentBand] = iDCoffset;
            CalData.qDCoffsetSSB[ConfigData.currentBand] = qDCoffset;
          }
          computeAdjdB();                 // This is to flush out transient from phase last set at extreme.
          adjdB_old = adjdB;              // Must calculate current best adjdB before entering refineAmpPlus.
          iDCoffset = iDCoffset + 0.0005; // Now increment amplitude.  Now calculate adjdB.
          state = State::refineAmpPlus;   // Proceed to refine the gain channel.
          break;
        }
        state = State::initialSweepPhase;
        break;

      case State::refineAmpPlus:

        if (adjdB < adjdB_old)
        {
          adjdB_old = adjdB;
          iDCoffset = iDCoffset + 0.0005;
          state = State::refineAmpPlus;
          break;
        }
        else
        {
          iDCoffset = iDCoffset - 0.0005 - 0.0005; // Put back last increment and increment in minus direction.
          refinePass = refinePass + 1;
          state = State::refineAmpMinus;
          break;
        }
        break;

      case State::refineAmpMinus:

        if (adjdB < adjdB_old)
        {
          adjdB_old = adjdB;
          iDCoffset = iDCoffset - 0.0005;
          state = State::refineAmpMinus;
          break;
        }
        else
        {

          iDCoffset = iDCoffset + 0.0005; // Put back last increment.
          qDCoffset = qDCoffset + 0.0005; // Increment for refinePhasePlus.
          state = State::refinePhasePlus;
          break;
        }
        break;

      case State::refinePhasePlus:

        if (adjdB < adjdB_old)
        {
          adjdB_old = adjdB;
          qDCoffset = qDCoffset + 0.0005;
          state = State::refinePhasePlus;
          break;
        }
        else
        {
          qDCoffset = qDCoffset - 0.0005 - 0.0005; // Put back last increment, and increment in the negative direction.
          state = State::refinePhaseMinus;
          break;
        }
        break;

      case State::refinePhaseMinus:

        if (adjdB < adjdB_old)
        {
          adjdB_old = adjdB;
          qDCoffset = qDCoffset - 0.0005;
          state = State::refinePhaseMinus;
          break;
        }
        else
        {
          qDCoffset = qDCoffset + 0.0005; // Put back last increment.
          if (refinePass == 2)
          {
            state = State::setOptimal;
          }
          else
          {
            iDCoffset = iDCoffset + 0.0005;
            state = State::refineAmpPlus; // Proceed to next pass.
            break;
          }

          state = State::setOptimal;
          break;
        }
        break;

      case State::setOptimal:

        if (mode == 0)
        {
          CalData.iDCoffsetCW[ConfigData.currentBand] = iDCoffset;
          CalData.qDCoffsetCW[ConfigData.currentBand] = qDCoffset;
        }
        if (mode == 1)
        {
          CalData.iDCoffsetSSB[ConfigData.currentBand] = iDCoffset;
          CalData.qDCoffsetSSB[ConfigData.currentBand] = qDCoffset;
        }
        state = State::exit;
        startTimer = static_cast<int>(milliTimer); // Start result view timer.
        break;
      case State::exit:
        // Delay exit if in radio calibration to show calibration results and then exit.
        if (radioCal)
        {
          if ((static_cast<int>(milliTimer) - startTimer) < 100)
          { // Show calibration result at conclusion during Radio Cal.
            state = State::exit;
            break;
          }
          else
          {
            TxCalibrate::CalibrateEpilogue();
            return;
          }
        }
        else
        {
          autoCal = false; // Go back to manual mode.
        }
        break;
      default:
        break;
      }
    } // end automatic calibration state machine

    task = MenuSelect::DEFAULT; // Reset task after it is used.
    //  Read encoder and update values.
    if (IQCalType == 0)
      iDCoffset = GetEncoderValueLive(-1.0, 1.0, iDCoffset, 0.0005);
    if (IQCalType == 1)
      qDCoffset = GetEncoderValueLive(-1.0, 1.0, qDCoffset, 0.0005);
    if (mode == 0)
    {
      CalData.iDCoffsetCW[ConfigData.currentBand] = iDCoffset;
      CalData.qDCoffsetCW[ConfigData.currentBand] = qDCoffset;
    }
    if (mode == 1)
    {
      CalData.iDCoffsetSSB[ConfigData.currentBand] = iDCoffset;
      CalData.qDCoffsetSSB[ConfigData.currentBand] = qDCoffset;
    }

  } // end while
} // End carrier calibration

// Automatic calibration of all bands.  Greg KF5N June 4, 2024
void TxCalibrate::RadioCal(int mode)
{
  std::vector<int> ham_bands = {BAND_80M, BAND_40M, BAND_20M, BAND_17M, BAND_15M, BAND_12M, BAND_10M};

  uint32_t currentBandTemp = ConfigData.currentBand;
  uint32_t centerFreqTemp = ConfigData.centerFreq;
  Sideband sidebandTemp = bands.bands[ConfigData.currentBand].sideband;
  RadioMode modeTemp = bands.bands[ConfigData.currentBand].mode;

  // Calibrate all bands.
  for (int band : ham_bands)
  {
    button.BandSet(band);
    ConfigData.currentBand = ConfigData.currentBandA = band;
    TxRxFreq = calFrequencies[band][mode]; // [band][cw 0, ssb 1]
    ConfigData.centerFreq = TxRxFreq;
    SetFreq();
    SetBandRelay();
    if (band < 2) // CW and SSB both calibrated LSB.
    {
      bands.bands[ConfigData.currentBand].sideband = Sideband::LOWER; // Calibrate lower sideband for 80M and 40M.
      rxcalibrater.DoReceiveCalibrate(mode, true, false);
      txcalibrater.DoXmitCarrierCalibrate(mode, true, false);
      txcalibrater.DoXmitCalibrate(mode, true, false);
    }

    if (band < 2 and mode == 1) // SSB calibrated USB (for FT8).
    {
      bands.bands[ConfigData.currentBand].sideband = Sideband::UPPER; // Calibrate upper sideband for 80M and 40M.
      rxcalibrater.DoReceiveCalibrate(mode, true, false);
      txcalibrater.DoXmitCarrierCalibrate(mode, true, false);
      txcalibrater.DoXmitCalibrate(mode, true, false);
    }

    if (band > 1)
    {
      bands.bands[ConfigData.currentBand].sideband = Sideband::UPPER;
      rxcalibrater.DoReceiveCalibrate(mode, true, false); // Include 80M and 40M due to FT8.
      txcalibrater.DoXmitCalibrate(mode, true, false);
      txcalibrater.DoXmitCarrierCalibrate(mode, true, false);
    }
  }

  ConfigData.currentBand = currentBandTemp;
  ConfigData.centerFreq = centerFreqTemp;
  bands.bands[ConfigData.currentBand].sideband = sidebandTemp;
  ConfigData.lastSideband[ConfigData.currentBand] = sidebandTemp;
  bands.bands[ConfigData.currentBand].mode = modeTemp;
  SetBandRelay();

  button.BandSet(ConfigData.currentBand);

  // Set flag for initial calibration completed.
  if (mode == 0)
    CalData.CWradioCalComplete = true;
  if (mode == 1)
    CalData.SSBradioCalComplete = true;
}

/*****
  Purpose: Signal processing for the purpose of calibration.  FFT only, no audio!

   Parameter List:


   Return value:
      void
 *****/
void TxCalibrate::MakeFFTData()
{
  float32_t rfGainValue;     // AFP 2-11-23.  Greg KF5N February 13, 2023
  uint32_t dataWidth = 2048; // was 2048
  float32_t powerScale = 0;

  float32_t *iBuffer = nullptr; // I and Q pointers needed for one-time read of record queues.
  float32_t *qBuffer = nullptr;

  // Read incoming I and Q audio blocks from the SSB exciter.
  // Data gatekeeper.  Are there at least N_BLOCKS buffers in each channel available ?
  while (static_cast<uint32_t>(Q_in_L_Ex.available()) < 16 and static_cast<uint32_t>(Q_in_R_Ex.available()) < 16)
  {
    ;
  }

  for (unsigned i = 0; i < 16; i++)
  {

    iBuffer = Q_in_L_Ex.readBuffer();
    qBuffer = Q_in_R_Ex.readBuffer();
    std::copy(iBuffer, iBuffer + 128, &float_buffer_L_EX[128 * i]);
    std::copy(qBuffer, qBuffer + 128, &float_buffer_R_EX[128 * i]);

    Q_in_L_Ex.freeBuffer();
    Q_in_R_Ex.freeBuffer();
  }

  // Set the sideband.
  if (bands.bands[ConfigData.currentBand].sideband == Sideband::LOWER)
    cessb1.setSideband(false);
  if (bands.bands[ConfigData.currentBand].sideband == Sideband::UPPER)
    cessb1.setSideband(true);

  /* Apply amplitude and phase corrections.
  AudioNoInterrupts();
  if (TxCalibrate::mode == 0)
  {
    if (bands.bands[ConfigData.currentBand].sideband == Sideband::LOWER)
      cessb1.setIQCorrections(true, CalData.IQCWAmpCorrectionFactorLSB[ConfigData.currentBandA], CalData.IQCWPhaseCorrectionFactorLSB[ConfigData.currentBandA], 0.0);
    if (bands.bands[ConfigData.currentBand].sideband == Sideband::UPPER)
      cessb1.setIQCorrections(true, CalData.IQCWAmpCorrectionFactorUSB[ConfigData.currentBandA], CalData.IQCWPhaseCorrectionFactorUSB[ConfigData.currentBandA], 0.0);
  }
  if (TxCalibrate::mode == 1)
  {
    if (bands.bands[ConfigData.currentBand].sideband == Sideband::LOWER)
      cessb1.setIQCorrections(true, CalData.IQSSBAmpCorrectionFactorLSB[ConfigData.currentBandA], CalData.IQSSBPhaseCorrectionFactorLSB[ConfigData.currentBandA], 0.0);
    if (bands.bands[ConfigData.currentBand].sideband == Sideband::UPPER)
      cessb1.setIQCorrections(true, CalData.IQSSBAmpCorrectionFactorUSB[ConfigData.currentBandA], CalData.IQSSBPhaseCorrectionFactorUSB[ConfigData.currentBandA], 0.0);
  }
  AudioInterrupts();
  */

  //  This is the correct place in the data stream to inject the scaling for power.
  if (mode == 0)
    powerScale = 2.0 * ConfigData.powerOutCW[ConfigData.currentBand];
  if (mode == 1)
    powerScale = 2.0 * ConfigData.powerOutSSB[ConfigData.currentBand];

  arm_scale_f32(float_buffer_L_EX, powerScale, float_buffer_L_EX, dataWidth);
  arm_scale_f32(float_buffer_R_EX, powerScale, float_buffer_R_EX, dataWidth);

  if (TxCalibrate::mode == 0)
  {
    arm_offset_f32(float_buffer_L_EX, CalData.iDCoffsetCW[ConfigData.currentBand] + CalData.dacOffsetCW, float_buffer_L_EX, dataWidth); // Carrier suppression offset.
    arm_offset_f32(float_buffer_R_EX, CalData.qDCoffsetCW[ConfigData.currentBand] + CalData.dacOffsetCW, float_buffer_R_EX, dataWidth);
  }
  if (TxCalibrate::mode == 1)
  {
    arm_offset_f32(float_buffer_L_EX, CalData.iDCoffsetSSB[ConfigData.currentBand] + CalData.dacOffsetSSB, float_buffer_L_EX, dataWidth); // Carrier suppression offset.
    arm_offset_f32(float_buffer_R_EX, CalData.qDCoffsetSSB[ConfigData.currentBand] + CalData.dacOffsetSSB, float_buffer_R_EX, dataWidth);
  }

  Q_out_L_Ex.play(float_buffer_L_EX, dataWidth); // play it!  This is the I channel from the Audio Adapter line out to QSE I input.
  Q_out_R_Ex.play(float_buffer_R_EX, dataWidth); // play it!  This is the Q channel from the Audio Adapter line out to QSE Q input.

  // End of transmit code.  Begin receive code.

  // Get audio samples from the audio  buffers and convert them to float.
  // Read in 16 blocks of 128 samples in I and Q if available.
  if (static_cast<uint32_t>(ADC_RX_I.available()) > 15 and static_cast<uint32_t>(ADC_RX_Q.available()) > 15)
  {
    for (unsigned i = 0; i < N_BLOCKS; i++)
    {
      /**********************************************************************************  AFP 12-31-20
          Using arm_Math library, convert to float one buffer_size.
          Float_buffer samples are now standardized from > -1.0 to < 1.0
      **********************************************************************************/
      arm_q15_to_float(ADC_RX_Q.readBuffer(), &float_buffer_L[BUFFER_SIZE * i], BUFFER_SIZE); // convert int_buffer to float 32bit
      arm_q15_to_float(ADC_RX_I.readBuffer(), &float_buffer_R[BUFFER_SIZE * i], BUFFER_SIZE); // convert int_buffer to float 32bit
      ADC_RX_I.freeBuffer();
      ADC_RX_Q.freeBuffer();
    }

    rfGainValue = pow(10, static_cast<float32_t>(ConfigData.rfGain[ConfigData.currentBand]) / 20); // AFP 2-11-23
    arm_scale_f32(float_buffer_L, rfGainValue, float_buffer_L, 2048);                              // AFP 2-11-23
    arm_scale_f32(float_buffer_R, rfGainValue, float_buffer_R, 2048);                              // AFP 2-11-23

    // Manual IQ amplitude and phase correction (receive only).
    if (mode == 0)
    {
      if (bands.bands[ConfigData.currentBand].sideband == Sideband::LOWER)
      {
        arm_scale_f32(float_buffer_L, CalData.IQCWRXAmpCorrectionFactorLSB[ConfigData.currentBand], float_buffer_L, 2048); // AFP 04-14-22
        IQPhaseCorrection(float_buffer_L, float_buffer_R, CalData.IQCWRXPhaseCorrectionFactorLSB[ConfigData.currentBand], 2048);
      }
      else
      {
        if (bands.bands[ConfigData.currentBand].sideband == Sideband::UPPER)
        {
          arm_scale_f32(float_buffer_L, CalData.IQCWRXAmpCorrectionFactorUSB[ConfigData.currentBand], float_buffer_L, 2048); // AFP 04-14-22 KF5N changed sign
          IQPhaseCorrection(float_buffer_L, float_buffer_R, CalData.IQCWRXPhaseCorrectionFactorUSB[ConfigData.currentBand], 2048);
        }
      }
    }

    if (mode == 1)
    {
      if (bands.bands[ConfigData.currentBand].sideband == Sideband::LOWER)
      {
        arm_scale_f32(float_buffer_L, CalData.IQSSBRXAmpCorrectionFactorLSB[ConfigData.currentBand], float_buffer_L, 2048); // AFP 04-14-22
        IQPhaseCorrection(float_buffer_L, float_buffer_R, CalData.IQSSBRXPhaseCorrectionFactorLSB[ConfigData.currentBand], 2048);
      }
      else
      {
        if (bands.bands[ConfigData.currentBand].sideband == Sideband::UPPER)
        {
          arm_scale_f32(float_buffer_L, CalData.IQSSBRXAmpCorrectionFactorUSB[ConfigData.currentBand], float_buffer_L, 2048); // AFP 04-14-22 KF5N changed sign
          IQPhaseCorrection(float_buffer_L, float_buffer_R, CalData.IQSSBRXPhaseCorrectionFactorUSB[ConfigData.currentBand], 2048);
        }
      }
    }

    FreqShift1(); // 12 kHz shift

    // This process started because there are 2048 samples available.  Perform FFT.
    updateDisplayFlag = true;
//    if (fftActive)
      ZoomFFTExe(2048);
    fftSuccess = true;
  } // End of receive code
  else
  {
    fftSuccess = false; // Insufficient receive buffers to make FFT.  Do not plot FFT data!
    Serial.printf("FFT failed due to insufficient I and Q receive data!\n");
  }
}

/*****
  Purpose: Show Spectrum display modified for IQ calibration.
           This is similar to the function used for normal reception, however, it has
           been simplified and streamlined for calibration.

  Parameter list:
    void

  Return value;
    void
*****/
void TxCalibrate::ShowSpectrum() // AFP 2-10-23
{
  int x1 = 0;
  int capture_bins = 8; // Sets the number of bins to scan for signal peak.
  int cal_bins[3] = {0, 0, 0};

  if ((calTypeFlag == 1 || calTypeFlag == 2) && bands.bands[ConfigData.currentBand].sideband == Sideband::LOWER)
  {
    cal_bins[0] = 257; // LSB
    cal_bins[1] = 289; // Carrier
    cal_bins[2] = 322; // Undesired sideband
  } // Transmit and Carrier calibration, LSB.  KF5N
  if ((calTypeFlag == 1 || calTypeFlag == 2) && bands.bands[ConfigData.currentBand].sideband == Sideband::UPPER)
  {
    cal_bins[0] = 257; // USB
    cal_bins[1] = 225; // Carrier
    cal_bins[2] = 193; // Undesired sideband
  } // Transmit and Carrier calibration, USB.  KF5N

  // Plot carrier during transmit cal, do not return a dB value:
  if (calTypeFlag == 1)
  { // Transmit cal
    x1 = cal_bins[0] - capture_bins;
    TxCalibrate::PlotCalSpectrum(x1, cal_bins, capture_bins); // Compute adjdB
  }
  if (calTypeFlag == 2)
  {
    x1 = cal_bins[0] - capture_bins; // x1 < cal_bins[0] + capture_bins; x1++)
    TxCalibrate::PlotCalSpectrum(x1, cal_bins, capture_bins);
  }

} // end ShowSpectrum()

/*****
  Purpose:  Plot Calibration Spectrum   //  KF5N 7/2/2023
  Parameter list:
    int x1, where x1 is the FFT bin.
    cal_bins[3], locations of the desired and undesired signals
    capture_bins, width of the bins used to display the signals
  Return value;
    float, returns the adjusted value in dB
*****/
void TxCalibrate::PlotCalSpectrum(int x1, int cal_bins[3], int capture_bins)
{
  int16_t adjAmplitude = 0; // Was float; cast to float in dB calculation.  KF5N
  int16_t refAmplitude = 0; // Was float; cast to float in dB calculation.  KF5N

  uint32_t index_of_max; // This variable is not currently used, but it is required by the ARM max function.  KF5N

  updateDisplayFlag = true; // This flag is used in ZoomFFTExe().  When true the FFT is performed, when false skipped.
                            // This is going to acquire 2048 samples of I and Q from the receiver, and then perform the FFT.
                            // The result will be a 512 wide array of FFT bin levels.
  TxCalibrate::MakeFFTData();

  // Find the maximums of the desired and undesired signals so that dB can be calculated.
  // This is done on sub-arrays of the FFT bins for efficiency.
  if ((bands.bands[ConfigData.currentBand].sideband == Sideband::LOWER) && (calTypeFlag == 1))
  {
    arm_max_q15(&pixelnew[(cal_bins[0] - capture_bins)], capture_bins * 2, &refAmplitude, &index_of_max);
    arm_max_q15(&pixelnew[(cal_bins[2] - capture_bins)], capture_bins * 2, &adjAmplitude, &index_of_max);
  }
  if ((bands.bands[ConfigData.currentBand].sideband == Sideband::UPPER) && (calTypeFlag == 1))
  {
    arm_max_q15(&pixelnew[(cal_bins[0] - capture_bins)], capture_bins * 2, &adjAmplitude, &index_of_max);
    arm_max_q15(&pixelnew[(cal_bins[2] - capture_bins)], capture_bins * 2, &refAmplitude, &index_of_max);
  }

  if ((bands.bands[ConfigData.currentBand].sideband == Sideband::LOWER) && (calTypeFlag == 2))
  {
    arm_max_q15(&pixelnew[(cal_bins[0] - capture_bins)], capture_bins * 2, &refAmplitude, &index_of_max);
    arm_max_q15(&pixelnew[(cal_bins[1] - capture_bins)], capture_bins * 2, &adjAmplitude, &index_of_max);
  }
  if ((bands.bands[ConfigData.currentBand].sideband == Sideband::UPPER) && (calTypeFlag == 2))
  {
    arm_max_q15(&pixelnew[(cal_bins[0] - capture_bins)], capture_bins * 2, &adjAmplitude, &index_of_max);
    arm_max_q15(&pixelnew[(cal_bins[1] - capture_bins)], capture_bins * 2, &refAmplitude, &index_of_max);
  }

  adjdB = (static_cast<float32_t>(adjAmplitude) - static_cast<float32_t>(refAmplitude)) / (1.95 * 2.0); // Cast to float and calculate the dB level.  Needs further refinement for accuracy.  KF5N
  if (bands.bands[ConfigData.currentBand].sideband == Sideband::UPPER && not(calTypeFlag == 0))
    adjdB = -adjdB; // Flip sign for USB only for TX cal.

} // end PlotCalSpectrum(. . .)

void TxCalibrate::computeAdjdB()
{
  float adjdB1{0.0};
  float adjdB2{0.0};
  int equalCounter{0};
  int worseCounter{0};
  bool notComputed{true};
  float epsilon = 0.1;

  // Compute initial value of adjdB.
  ShowSpectrum();

  adjdB1 = adjdB; // Initial value of adjdB1 computed by Showspectrum().

  adjdBstate = computeAdjdB::measureAdjdB2;

  while (notComputed)
  {
    switch (adjdBstate)
    {

    case computeAdjdB::measureAdjdB2:

      ShowSpectrum(); // Compute adjdB2.

      adjdB2 = adjdB;
      adjdBstate = computeAdjdB::computeNextState;
      break;

    case computeAdjdB::computeNextState:

      // Floating point values are equal.
      if (fabs(adjdB2 - adjdB1) < epsilon)
      {
        equalCounter = equalCounter + 1;
        if (equalCounter == 2)
        {
          equalCounter = 0;
          adjdBstate = computeAdjdB::computed;
          break;
        }
        adjdBstate = computeAdjdB::measureAdjdB2;
        adjdB1 = adjdB2;
        break;
      }

      // adjdB2 < adjdB1.  Improvement.  Always re-measure.
      if (adjdB2 - adjdB1 < 0.0)
      {
        adjdBstate = computeAdjdB::measureAdjdB2;
        adjdB1 = adjdB2;
        break;
      }

      // adjdB2 > adjdB1.  Worse.
      if (adjdB2 - adjdB1 > 0.0)
      {
        worseCounter = worseCounter + 1;
        if (worseCounter == 2)
        {
          worseCounter = 0;
          adjdBstate = computeAdjdB::computed;
          break;
        }
        adjdBstate = computeAdjdB::measureAdjdB2;
        adjdB1 = adjdB2;
        break;
      }

      break;

    case computeAdjdB::computed:

      notComputed = false;
      return;

      break;
    }
  } // end while
}
