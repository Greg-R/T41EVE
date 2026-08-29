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

// RxCalibrate class
// Class extensively modified to perform receive calibration only.  Greg KF5N August 2025

#include "SDT.h"

/*****
  Purpose: Load buffers used to modulate the transmitter during calibration.
          The epilogue must restore the buffers for normal operation!

   Parameter List:
      void

   Return value:
      void
 *****/
void RxCalibrate::loadCalToneBuffers(float toneFreq)
{
  float theta;
  // This loop creates the sinusoidal waveform for the tone.
  for (int kf = 0; kf < 256; kf++)
  {
    theta = kf * 2.0 * PI * toneFreq / 24000;
    sinBuffer[kf] = sin(theta);
    cosBuffer[kf] = cos(theta);
  }
}

/*****
  Purpose: Run MakeFFTData() a few times to load and settle out buffers.  KF5N May 22, 2024
           Compute FFT in order to find maximum signal peak prior to beginning calibration.
  Parameter list:
    void

  Return value:
    void
*****/
void RxCalibrate::warmUpCal()
{
  uint32_t index_of_max{0};
  uint32_t count{0};
  uint32_t i;
  // MakeFFTData() has to be called enough times for transients to settle out before computing FFT.
  for (i = 0; i < 32; i = i + 1)
  {
    fftActive = true;
    updateDisplayFlag = true;
    RxCalibrate::MakeFFTData(); // Note, FFT not called if buffers are not sufficiently filled.
    arm_max_q15(pixelnew, 512, &rawSpectrumPeak, &index_of_max);
    if (index_of_max > 380 and index_of_max < 388)
    { // The peak is in the correct bin?
      count = count + 1;
    }
    else
      count = 0; // Reset count in case of failure.
    if (count == 5)
      break; // If five in a row, exit the loop.  Warm-up is complete.
  }
  updateDisplayFlag = true; // This flag is used by the normal receiver process.
  fftActive = true;         // This is a flag local to this class.
  updateDisplayFlag = false;
  fftActive = false;
  // Find peak of spectrum, which is 512 wide.  Use this to adjust spectrum peak to top of spectrum display.
  arm_max_q15(pixelnew, 512, &rawSpectrumPeak, &index_of_max);
  //    Serial.printf("RX rawSpectrumPeak = %d count = %d i = %d\n", rawSpectrumPeak, count, i);
  //    Serial.printf("RX index_of_max = %d\n", index_of_max);
  if (index_of_max < 380 or index_of_max > 388)
    Serial.printf("Problem with RX warmUpCal\n");
  ADC_RX_I.clear();
  ADC_RX_Q.clear();
  Q_in_L_Ex.clear();
  Q_in_R_Ex.clear();
}

/*****
  Purpose: Set up prior to IQ calibrations.  New function.  KF5N August 14, 2023
  These things need to be saved here and restored in the epilogue function:
  Vertical scale in dB  (set to 10 dB during calibration)
  Zoom, set to 1X during receive calibration.
  Transmitter power, set to 5W during both calibrations.
   Parameter List:
      int setZoom   (This parameter should be 0 for receive (1X).

   Return value:
      void
 *****/
void RxCalibrate::CalibratePreamble(int setZoom)
{
  //  cessb1.processorUsageMaxReset();
  controlAudioOut(ConfigData.audioOut, true); // Mute all receiver audio.
  calOnFlag = true;
  exitManual = false;
  transmitPowerLevelTemp = ConfigData.transmitPowerLevel; // AFP 05-11-23
  cwFreqOffsetTemp = ConfigData.CWOffset;
  // Remember the mode and state, and restore in the Epilogue.
  tempMode = bands.bands[ConfigData.currentBand].mode;
  tempState = radioState;
  // Calibrate requires upper or lower sideband.  Change if currently in an AM mode.  Put back in Epilogue.
  if (bands.bands[ConfigData.currentBand].sideband == Sideband::BOTH_AM or bands.bands[ConfigData.currentBand].sideband == Sideband::BOTH_SAM)
  {
    tempSideband = bands.bands[ConfigData.currentBand].sideband;
    // Use the last upper or lower sideband (CW, SSB, or FT8) during calibration.
    bands.bands[ConfigData.currentBand].sideband = ConfigData.lastSideband[ConfigData.currentBand];
  }
  else
    tempSideband = bands.bands[ConfigData.currentBand].sideband;
  ConfigData.CWOffset = 2;                  // 750 Hz for TX calibration.  Epilogue restores user selected offset.
  userZoomIndex = ConfigData.spectrum_zoom; // Save the zoom index so it can be reset at the conclusion.  KF5N August 12, 2023
  ConfigData.spectrum_zoom = setZoom;
  button.ButtonZoom();

  userScale = ConfigData.currentScale; //  Remember user preference so it can be reset when done.  KF5N
  ConfigData.currentScale = 1;         //  Set vertical scale to 10 dB during calibration.  KF5N
  updateDisplayFlag = false;
  ConfigData.centerFreq = TxRxFreq;
  NCOFreq = 0;
  digitalWrite(MUTE, MUTEAUDIO); //  Mute Audio  (HIGH=Mute)
  digitalWrite(RXTX, HIGH);      // Turn on transmitter.
  radioState = RadioState::RECEIVE_CALIBRATE_STATE;
  rawSpectrumPeak = 0;
  SetAudioOperatingState(radioState); // Do this last!  This clears the queues.
}

/*****
  Purpose: Shut down and clean up after IQ calibrations.  New function.  KF5N August 14, 2023

   Parameter List:
      void

   Return value:
      void
 *****/
void RxCalibrate::CalibrateEpilogue(bool radioCal, bool saveToEeprom)
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
  Serial.printf("cessb1 max processor usage = %d\n", cessb1.processorUsageMax());
  */

  digitalWrite(RXTX, LOW); // Turn off the transmitter.
  updateDisplayFlag = false;
  // Clear queues to reduce transient.
  ADC_RX_I.end();
  ADC_RX_I.clear();
  ADC_RX_Q.end();
  ADC_RX_Q.clear();
  ConfigData.centerFreq = TxRxFreq;
  NCOFreq = 0;
  calibrateFlag = 0;                                      // KF5N
  ConfigData.CWOffset = cwFreqOffsetTemp;                 // Return user selected CW offset frequency.
  sineTone(ConfigData.CWOffset + 6);                      // This function takes "number of cycles" which is the offset + 6.
  ConfigData.currentScale = userScale;                    //  Restore vertical scale to user preference.  KF5N
  ConfigData.transmitPowerLevel = transmitPowerLevelTemp; // Restore the user's transmit power level setting.  KF5N August 15, 2023
  if (bands.bands[ConfigData.currentBand].sideband == Sideband::BOTH_AM or bands.bands[ConfigData.currentBand].sideband == Sideband::BOTH_SAM)
    bands.bands[ConfigData.currentBand].sideband = tempSideband;
  bands.bands[ConfigData.currentBand].mode = tempMode;
  if (saveToEeprom)
    eeprom.CalDataWrite(); // Save calibration numbers and configuration.  KF5N August 12, 2023
  calOnFlag = false;
  fftOffset = 0; // Some reboots may be caused by large fftOffset values when Auto-Spectrum is on.
  ResetFlipFlops();
  bands.bands[ConfigData.currentBand].sideband = tempSideband; // Restore the sideband.
  radioState = tempState;
  lastState = RadioState::NOSTATE; // This is required due to the function deactivating the receiver.  This forces a pass through the receiver set-up code.  KF5N October 16, 2023
  SetAudioOperatingState(radioState);
  ConfigData.spectrum_zoom = userZoomIndex;
  button.ButtonZoom(); // Restore the user's zoom setting.
  powerUp = true;      // Clip off transient.
}

/*****
  Purpose: Write the calibration factors to the CalData struct.

   Parameter List:
      float ichannel, float qchannel

   Return value:
      void
 *****/
void RxCalibrate::writeToCalData(float ichannel, float qchannel)
{
  if (mode == 0)
  { // CW
    if (bands.bands[ConfigData.currentBand].sideband == Sideband::LOWER)
    {
      CalData.IQCWRXAmpCorrectionFactorLSB[ConfigData.currentBand] = amplitude;
      CalData.IQCWRXPhaseCorrectionFactorLSB[ConfigData.currentBand] = phase;
    }
    else if (bands.bands[ConfigData.currentBand].sideband == Sideband::UPPER)
    {
      CalData.IQCWRXAmpCorrectionFactorUSB[ConfigData.currentBand] = amplitude;
      CalData.IQCWRXPhaseCorrectionFactorUSB[ConfigData.currentBand] = phase;
    }
  }
  if (mode == 1)
  { // SSB
    if (bands.bands[ConfigData.currentBand].sideband == Sideband::LOWER)
    {
      CalData.IQSSBRXAmpCorrectionFactorLSB[ConfigData.currentBand] = amplitude;
      CalData.IQSSBRXPhaseCorrectionFactorLSB[ConfigData.currentBand] = phase;
    }
    else if (bands.bands[ConfigData.currentBand].sideband == Sideband::UPPER)
    {
      CalData.IQSSBRXAmpCorrectionFactorUSB[ConfigData.currentBand] = amplitude;
      CalData.IQSSBRXPhaseCorrectionFactorUSB[ConfigData.currentBand] = phase;
    }
  }
}

/*****
  Purpose: Combined input/output for the purpose of calibrating the receiver IQ.

   Parameter List:
      mode (0 is CW, 1 is SSB), bool radioCal, bool refineCal, bool saveToEeprom

   Return value:
      void
 *****/
void RxCalibrate::DoReceiveCalibrate(int calMode, bool radio, bool refine, bool toEeprom)
{
  MenuSelect task = MenuSelect::DEFAULT;

  RxCalibrate::mode = calMode;          // CW or SSB.  This is an object state variable.
  RxCalibrate::radioCal = radio;        // Initial calibration of all bands.
  RxCalibrate::refineCal = refine;      // Refinement (using existing values a starting point) calibration for all bands.
  RxCalibrate::saveToEeprom = toEeprom; // Save to EEPROM

  loadCalToneBuffers(750.0);
  CalibratePreamble(0);     // Set zoom to 1X.
  int calFreqShift = 96000; // Transmit frequency to 2 times IF, the image.

  ResetFlipFlops(); // This function has delay.

  SetFreqCal(calFreqShift);
  IQCalType = 0;               // Start with IG Gain calibration.
  warmUpCal();                 // Finds the peak of the FFT to adjust in display.
  State state = State::warmup; // Start calibration state machine in warmup state.
  float maxSweepAmp = 0.1;
  float maxSweepPhase = 0.1;
  increment = 0.01; // Used in initial sweeps.
  int refinePass{0};
  float iOptimal = 1.0;
  float qOptimal = 0.0;
  float adjdB_min{0};
  float adjdB_old{0};
  std::vector<float32_t> sweepVector(21);
  std::vector<float32_t> sweepVectorValue(21);
  int startTimer = 0; // Used to time display of results.
  std::vector<float>::iterator result;
  // Get current values for amplitude and phase.  This is for refinement only.
  if (mode == 0)
  {
    if (bands.bands[ConfigData.currentBand].sideband == Sideband::LOWER)
    {
      iOptimal = amplitude = CalData.IQCWRXAmpCorrectionFactorLSB[ConfigData.currentBand];
      qOptimal = phase = CalData.IQCWRXPhaseCorrectionFactorLSB[ConfigData.currentBand];
    }
    else if (bands.bands[ConfigData.currentBand].sideband == Sideband::UPPER)
    {
      iOptimal = amplitude = CalData.IQCWRXAmpCorrectionFactorUSB[ConfigData.currentBand];
      qOptimal = phase = CalData.IQCWRXPhaseCorrectionFactorUSB[ConfigData.currentBand];
    }
  }
  if (mode == 1)
  {
    if (bands.bands[ConfigData.currentBand].sideband == Sideband::LOWER)
    {
      iOptimal = amplitude = CalData.IQSSBRXAmpCorrectionFactorLSB[ConfigData.currentBand];
      qOptimal = phase = CalData.IQSSBRXPhaseCorrectionFactorLSB[ConfigData.currentBand];
    }
    else if (bands.bands[ConfigData.currentBand].sideband == Sideband::UPPER)
    {
      iOptimal = amplitude = CalData.IQSSBRXAmpCorrectionFactorUSB[ConfigData.currentBand];
      qOptimal = phase = CalData.IQSSBRXPhaseCorrectionFactorUSB[ConfigData.currentBand];
    }
  }

  GetEncoderValueLive(-2.0, 2.0, phase, increment); // Show phase on display.

  if (radioCal)
  {
    autoCal = true;
    warmup = 0;
    index = 1;
    IQCalType = 0;
    std::fill(sweepVectorValue.begin(), sweepVectorValue.end(), 0.0);
    std::fill(sweepVector.begin(), sweepVector.end(), 0.0);
    state = State::warmup;
  }

  // Receive Calibration Loop
  while (true)
  {
    fftActive = true;
    computeAdjdB();

    if ((static_cast<int>(displayTimer) - lastDisplayTime) > 20)
    {
      evedisplay.drawReceiverCalScreen(pixelnew);
      lastDisplayTime = static_cast<int>(displayTimer);
    }

    // Exit from manual calibration by button push.
    if (exitManual == true)
    {
      RxCalibrate::CalibrateEpilogue(radioCal, saveToEeprom);
      return;
    }
    task = button.readButton();

    switch (task)
    {
    // Activate automatic calibration (initial calibration).
    case MenuSelect::ZOOM: // 2nd row, 1st column button
      autoCal = true;
      warmup = 0;
      index = 1;
      IQCalType = 0;
      std::fill(sweepVectorValue.begin(), sweepVectorValue.end(), 0.0);
      std::fill(sweepVector.begin(), sweepVector.end(), 0.0);
      state = State::warmup;
      break;
    // Automatic calibration using previously stored values (refine calibration).
    case MenuSelect::FILTER: // 3rd row, 1st column button
      refineCal = true;
      autoCal = true;
      warmup = 0;
      index = 1;
      IQCalType = 0;
      std::fill(sweepVectorValue.begin(), sweepVectorValue.end(), 0.0);
      std::fill(sweepVector.begin(), sweepVector.end(), 0.0);
      state = State::warmup;
      break;
    // Toggle gain and phase adjustment in manual mode.
    case MenuSelect::UNUSED_1:
      if (IQCalType == 0)
      {
        IQCalType = 1;
        // Switching to phase, make gain white.
        GetEncoderValueLive(-2.0, 2.0, amplitude, increment);
      }
      else
      {
        IQCalType = 0;
        // Switching go gain, make phase white.
        GetEncoderValueLive(-2.0, 2.0, phase, increment);
      }
      break;
    // Toggle increment value
    case MenuSelect::BEARING: // UNUSED_2 is now called BEARING
      corrChange = not corrChange;
      if (corrChange == true)
      { // Toggle increment value
        increment = 0.001;
      }
      else
      {
        increment = 0.002;
      }
      break;
    case MenuSelect::MENU_OPTION_SELECT: // Save values and exit calibration.
      exitManual = true;
      RxCalibrate::CalibrateEpilogue(radioCal, saveToEeprom);
      return;

      break;
    default:
      break;
    } // end switch

    //  Begin automatic calibration state machine.
    if (autoCal || radioCal)
    {
      switch (state)
      {
      case State::warmup:
        autoCal = true;
        index = 0;
        IQCalType = 0;
        std::fill(sweepVectorValue.begin(), sweepVectorValue.end(), 0.0);
        std::fill(sweepVector.begin(), sweepVector.end(), 0.0);
        warmup = warmup + 1;
        if (not refineCal)
        {
          phase = 0.0 + maxSweepPhase;   //  Need to use these values during warmup
          amplitude = 1.0 + maxSweepAmp; //  so adjdB and adjdB_avg are forced upwards.
        }
        state = State::warmup;
        if (warmup == 1)
          state = State::state0;
        break;

      case State::state0:
        // Starting values for sweeps.  First sweep is amplitude (gain).
        phase = 0.0;
        amplitude = 1.0 - maxSweepAmp;                    // Begin sweep at low end and move upwards.
        GetEncoderValueLive(-2.0, 2.0, phase, increment); // Display the phase value.
        adjdB = 0;
        index = 0;
        IQCalType = 0;
        increment = 0.01;               // Reset increment in case initial cal is run twice.
        adjdB_min = 0;
        state = State::initialSweepAmp; // Let this fall through.

      case State::initialSweepAmp:
        sweepVectorValue[index] = amplitude;
        sweepVector[index] = adjdB;
        if((adjdB < adjdB_min) and (adjdB < -20.0)) adjdB_min = adjdB;
        if((adjdB - adjdB_min) > 2.0) amplitude = maxSweepAmp;
        index = index + 1;
        // Increment for next measurement.
        amplitude = amplitude + increment; // Next one!
        // Go to Q channel when I channel sweep is finished.
        if (abs(amplitude - 1.0) > maxSweepAmp)
        {                                                                    // Needs to be subtracted from 1.0.
                                                                             ////            IQCalType = 1;                                                      // Get ready for phase.
          result = std::min_element(sweepVector.begin(), sweepVector.end()); // Value of the minimum.
          adjdBMinIndex = std::distance(sweepVector.begin(), result);        // Find the index.
          iOptimal = sweepVectorValue[adjdBMinIndex];                        // Set to the discovered minimum.
          phase = -maxSweepPhase;                                            // The starting value for phase.
          amplitude = iOptimal;                                              // Reset for next sweep.
          // Update display to optimal value.
          GetEncoderValueLive(-2.0, 2.0, amplitude, increment);
          IQCalType = 1; // Prepare for phase.
          index = 0;
          // Clear the vector before moving to phase.
          std::fill(sweepVectorValue.begin(), sweepVectorValue.end(), 0.0);
          std::fill(sweepVector.begin(), sweepVector.end(), 0.0);
          adjdB_min = 0;
          state = State::initialSweepPhase; // Initial sweeps done; proceed to refine phase.
          break;
        }
        state = State::initialSweepAmp; // This statement is not strictly necessary; leave here for clarity.
        break;

      case State::initialSweepPhase:
        sweepVectorValue[index] = phase;
        sweepVector[index] = adjdB;
        if((adjdB < adjdB_min) and (adjdB < -20.0)) adjdB_min = adjdB;
        if((adjdB - adjdB_min) > 2.0) phase = maxSweepPhase;
        index = index + 1;
        // Increment for the next measurement.
        phase = phase + increment;
        if (phase > maxSweepPhase)
        {
          result = std::min_element(sweepVector.begin(), sweepVector.end());
          adjdBMinIndex = std::distance(sweepVector.begin(), result); // Input 2 is first right channel.
          qOptimal = sweepVectorValue[adjdBMinIndex];                 // Set to the discovered minimum.
          phase = qOptimal;                                           // Set to the discovered minimum.
          GetEncoderValueLive(-2.0, 2.0, phase, increment);
          IQCalType = 0;
          adjdB = 0.0;
          index = 0;
          increment = 0.001; // Set this for manual.

          computeAdjdB();                // This is to flush out transient from phase last set at extreme.
          adjdB_old = adjdB;             // Must calculate current best adjdB before entering refineAmpPlus.
          amplitude = amplitude + 0.001; // Now increment amplitude.
          refinePass = 0;
          state = State::refineAmpPlus; // Proceed to refine the gain channel.

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
          print = false;
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
        else // Prepare for refinePhasePlus.
        {
          amplitude = amplitude + 0.001; // Put back last increment and increment in minus direction.
          phase = phase + 0.001;
          print = false;
          refinePass = refinePass + 1;
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
          print = false;
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
          print = false;
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
        writeToCalData(amplitude, phase);
        state = State::exit;
        startTimer = static_cast<int>(milliTimer); // Start result view timer.
        break;
      case State::exit:
        // Delay exit if in radio calibration to show calibration results for 5 seconds.
        if (radioCal)
        {
          if ((static_cast<int>(milliTimer) - startTimer) < 100)
          { // Show calibration result for 5 seconds at conclusion during Radio Cal.
            state = State::exit;
            break;
          }
          else
          {
            RxCalibrate::CalibrateEpilogue(radioCal, saveToEeprom);
            return;
          }
        }
        else
        {
          autoCal = false; // Don't enter switch, but remain in manual loop.
          refineCal = false;
        }
        break;
      }
    } // end automatic calibration state machine

    task = MenuSelect::DEFAULT; // Reset task after it is used.

    //  Read encoder and update values.  This is manual calibration.
    if (IQCalType == 0)
      amplitude = GetEncoderValueLive(-2.0, 2.0, amplitude, increment);
    if (IQCalType == 1)
      phase = GetEncoderValueLive(-2.0, 2.0, phase, increment);
    writeToCalData(amplitude, phase);
  } // end while
} // End Receive calibration

/*****
  Purpose: Signal processing for the purpose of calibration.
           This operates at 192ksps and is therefore not compatible with TX calibration.

   Parameter List:
      none

   Return value:
      void
 *****/
void RxCalibrate::MakeFFTData()
{
  float rfGainValue, powerScale; // AFP 2-11-23.  Greg KF5N February 13, 2023
                                 //  float recBandFactor[7] = { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 };  // AFP 2-11-23  KF5N uniform values

  /**********************************************************************************  AFP 12-31-20
        Get samples from queue buffers
        Teensy Audio Library stores ADC data in two buffers size=128, Q_in_L and Q_in_R as initiated from the audio lib.
        Then the buffers are read into two arrays sp_L and sp_R in blocks of 128 up to N_BLOCKS.  The arrays are
        of size BUFFER_SIZE * N_BLOCKS.  BUFFER_SIZE is 128.
        N_BLOCKS = FFT_LENGTH / 2 / BUFFER_SIZE * (uint32_t)DF; // should be 16 with DF == 8 and FFT_LENGTH = 512
        BUFFER_SIZE*N_BLOCKS = 2048 samples
     **********************************************************************************/
  // Generate I and Q for receive calibration.  Greg Raven KF5N October 2025
  arm_scale_f32(cosBuffer, 0.20, float_buffer_L_EX, 256); // AFP 2-11-23 Use pre-calculated sin & cos instead of Hilbert
  arm_scale_f32(sinBuffer, 0.20, float_buffer_R_EX, 256);

  // Transmitter calibration parameters.  These will be approximations, and good enough for this purpose.
  if (mode == 0)
  {
    if (bands.bands[ConfigData.currentBand].sideband == Sideband::LOWER)
    {
      arm_scale_f32(float_buffer_L_EX, -CalData.IQCWAmpCorrectionFactorLSB[ConfigData.currentBand], float_buffer_L_EX, 256);      // Adjust level of L buffer // AFP 2-11-23
      IQPhaseCorrection(float_buffer_L_EX, float_buffer_R_EX, CalData.IQCWPhaseCorrectionFactorLSB[ConfigData.currentBand], 256); // Adjust phase
    }
    if (bands.bands[ConfigData.currentBand].sideband == Sideband::UPPER)
    {
      arm_scale_f32(float_buffer_L_EX, CalData.IQCWAmpCorrectionFactorUSB[ConfigData.currentBand], float_buffer_L_EX, 256); // AFP 2-11-23
      IQPhaseCorrection(float_buffer_L_EX, float_buffer_R_EX, CalData.IQCWPhaseCorrectionFactorUSB[ConfigData.currentBand], 256);
    }
  }
  if (mode == 1)
  {
    if (bands.bands[ConfigData.currentBand].sideband == Sideband::LOWER)
    {
      arm_scale_f32(float_buffer_L_EX, -CalData.IQSSBAmpCorrectionFactorLSB[ConfigData.currentBand], float_buffer_L_EX, 256);      // Adjust level of L buffer // AFP 2-11-23
      IQPhaseCorrection(float_buffer_L_EX, float_buffer_R_EX, CalData.IQSSBPhaseCorrectionFactorLSB[ConfigData.currentBand], 256); // Adjust phase
    }
    if (bands.bands[ConfigData.currentBand].sideband == Sideband::UPPER)
    {
      arm_scale_f32(float_buffer_L_EX, CalData.IQSSBAmpCorrectionFactorUSB[ConfigData.currentBand], float_buffer_L_EX, 256); // AFP 2-11-23
      IQPhaseCorrection(float_buffer_L_EX, float_buffer_R_EX, CalData.IQSSBPhaseCorrectionFactorUSB[ConfigData.currentBand], 256);
    }
  }

  // 24KHz effective sample rate here
  arm_fir_interpolate_f32(&FIR_int1_EX_I, float_buffer_L_EX, float_buffer_LTemp, 256);
  arm_fir_interpolate_f32(&FIR_int1_EX_Q, float_buffer_R_EX, float_buffer_RTemp, 256);

  // interpolation-by-4,  48KHz effective sample rate here
  arm_fir_interpolate_f32(&FIR_int2_EX_I, float_buffer_LTemp, float_buffer_L_EX, 512);
  arm_fir_interpolate_f32(&FIR_int2_EX_Q, float_buffer_RTemp, float_buffer_R_EX, 512);

  //  This is the correct place in the data stream to inject the scaling for power.
  powerScale = 40.0 * ConfigData.powerOutCW[ConfigData.currentBand];

  //  192KHz effective sample rate here
  arm_scale_f32(float_buffer_L_EX, powerScale, float_buffer_L_EX, 2048); // Scale to compensate for losses in Interpolation
  arm_scale_f32(float_buffer_R_EX, powerScale, float_buffer_R_EX, 2048);

  if (mode == 0)
  {
    arm_offset_f32(float_buffer_L_EX, CalData.iDCoffsetCW[ConfigData.currentBand] + CalData.dacOffsetCW, float_buffer_L_EX, 2048);
    arm_offset_f32(float_buffer_R_EX, CalData.qDCoffsetCW[ConfigData.currentBand] + CalData.dacOffsetCW, float_buffer_R_EX, 2048);
  }
  if (mode == 1)
  {
    arm_offset_f32(float_buffer_L_EX, CalData.iDCoffsetSSB[ConfigData.currentBand] + CalData.dacOffsetSSB, float_buffer_L_EX, 2048); // Carrier suppression offset.
    arm_offset_f32(float_buffer_R_EX, CalData.qDCoffsetSSB[ConfigData.currentBand] + CalData.dacOffsetSSB, float_buffer_R_EX, 2048);
  }

  Q_out_L_Ex.setBehaviour(AudioPlayQueue_F32::ORIGINAL);
  Q_out_R_Ex.setBehaviour(AudioPlayQueue_F32::ORIGINAL);
  Q_out_L_Ex.play(float_buffer_L_EX, 2048);
  Q_out_R_Ex.play(float_buffer_R_EX, 2048);

  // End of transmit code.  Begin receive code.

  // Get I16 audio blocks from the record queues and convert them to float.
  // Read in 16 blocks of 128 samples in I and Q if available.
  if (static_cast<uint32_t>(ADC_RX_I.available()) > 16 and static_cast<uint32_t>(ADC_RX_Q.available()) > 16)
  {
    for (unsigned i = 0; i < 16; i++)
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

    rfGainValue = pow(10, (float)ConfigData.rfGain[ConfigData.currentBand] / 20);       // AFP 2-11-23
    arm_scale_f32(float_buffer_L, rfGainValue, float_buffer_L, BUFFER_SIZE * N_BLOCKS); // AFP 2-11-23
    arm_scale_f32(float_buffer_R, rfGainValue, float_buffer_R, BUFFER_SIZE * N_BLOCKS); // AFP 2-11-23

    // IQ amplitude and phase correction.  Mode 0 is for CW and Mode 1 is for SSB.
    if (mode == 0)
    {
      if (bands.bands[ConfigData.currentBand].sideband == Sideband::LOWER)
      {
        arm_scale_f32(float_buffer_L, CalData.IQCWRXAmpCorrectionFactorLSB[ConfigData.currentBand], float_buffer_L, BUFFER_SIZE * N_BLOCKS); // AFP 04-14-22
        IQPhaseCorrection(float_buffer_L, float_buffer_R, CalData.IQCWRXPhaseCorrectionFactorLSB[ConfigData.currentBand], BUFFER_SIZE * N_BLOCKS);
      }
      if (bands.bands[ConfigData.currentBand].sideband == Sideband::UPPER)
      {
        arm_scale_f32(float_buffer_L, CalData.IQCWRXAmpCorrectionFactorUSB[ConfigData.currentBand], float_buffer_L, BUFFER_SIZE * N_BLOCKS); // AFP 04-14-22 KF5N changed sign
        IQPhaseCorrection(float_buffer_L, float_buffer_R, CalData.IQCWRXPhaseCorrectionFactorUSB[ConfigData.currentBand], BUFFER_SIZE * N_BLOCKS);
      }
    }
    if (mode == 1)
    {
      if (bands.bands[ConfigData.currentBand].sideband == Sideband::LOWER)
      {
        arm_scale_f32(float_buffer_L, CalData.IQSSBRXAmpCorrectionFactorLSB[ConfigData.currentBand], float_buffer_L, BUFFER_SIZE * N_BLOCKS); // AFP 04-14-22
        IQPhaseCorrection(float_buffer_L, float_buffer_R, CalData.IQSSBRXPhaseCorrectionFactorLSB[ConfigData.currentBand], BUFFER_SIZE * N_BLOCKS);
      }
      if (bands.bands[ConfigData.currentBand].sideband == Sideband::UPPER)
      {
        arm_scale_f32(float_buffer_L, CalData.IQSSBRXAmpCorrectionFactorUSB[ConfigData.currentBand], float_buffer_L, BUFFER_SIZE * N_BLOCKS); // AFP 04-14-22 KF5N changed sign
        IQPhaseCorrection(float_buffer_L, float_buffer_R, CalData.IQSSBRXPhaseCorrectionFactorUSB[ConfigData.currentBand], BUFFER_SIZE * N_BLOCKS);
      }
    }

    // This process started because there are 2048 samples available.  Perform FFT.
    updateDisplayFlag = true;
    if (fftActive)
      CalcZoom1Magn(); // Receiver calibration uses 1X zoom.
    FreqShift1();      // 48 kHz shift
    fftSuccess = true;
  } // End of receive code
  else
  {
    fftSuccess = false; // Insufficient receive buffers to make FFT.  Do not plot FFT data!
    Serial.printf("FFT failed!\n");
  }
} // end MakeFFTData()

/*****
  Purpose: Show Spectrum display modified for IQ calibration.
           This is similar to the function used for normal reception, however, it has
           been simplified and streamlined for calibration.

  Parameter list:
    void

  Return value;
    void
*****/
void RxCalibrate::ShowSpectrum() // AFP 2-10-23
{
  int x1 = 0;

  pixelnew[0] = 0;
  pixelnew[1] = 0;

  int cal_bins[3] = {0, 0, 0};
  if (bands.bands[ConfigData.currentBand].sideband == Sideband::LOWER)
  {
    cal_bins[0] = rx_blue_usb; // was 315
    cal_bins[1] = rx_red_usb;
  } // Receive calibration, LSB.  KF5N
  if (bands.bands[ConfigData.currentBand].sideband == Sideband::UPPER)
  {
    cal_bins[0] = rx_blue_usb; // was 315
    cal_bins[1] = rx_red_usb;
  } // Receive calibration, USB.  KF5N

  x1 = cal_bins[0] - capture_bins / 2;
  PlotCalSpectrum(x1, cal_bins, capture_bins);
}

/*****
  Purpose:  Plot Calibration Spectrum   //  KF5N 7/2/2023
            This function plots a partial spectrum during calibration only.
            This is intended to increase the efficiency and therefore the responsiveness of the calibration encoder.
            This function is called by ShowSpectrum2() in two for-loops.  One for-loop is for the reference signal,
            and the other for-loop is for the undesired sideband.
  Parameter list:
    int x1, where x1 is the FFT bin.
    cal_bins[3], locations of the desired and undesired signals
    capture_bins, width of the bins used to display the signals
  Return value;
    void
*****/
void RxCalibrate::PlotCalSpectrum(int x1, int cal_bins[3], int capture_bins)
{
  int16_t adjAmplitude = 0; // Was float; cast to float in dB calculation.  KF5N
  int16_t refAmplitude = 0; // Was float; cast to float in dB calculation.  KF5N

  uint32_t index_of_max; // This variable is not currently used, but it is required by the ARM max function.  KF5N
  int y_new_plot, y1_new_plot, y_old_plot, y_old2_plot;

  // The FFT should be performed only at the beginning of the sweep, and buffers must be full.
  if (x1 == (cal_bins[0] - capture_bins / 2))
  {                             // Set flag at revised beginning.  KF5N
    updateDisplayFlag = true;   // This flag is used in ZoomFFTExe().
    RxCalibrate::MakeFFTData(); // Compute FFT and draw it on the display.
  }
  else
    updateDisplayFlag = false; //  Do not save the the display data for the remainder of the sweep.

  // Call the Audio process from within the display routine to eliminate conflicts with drawing the spectrum.

  y_new = pixelnew[x1];

  // Find the maximums of the desired and undesired signals.

  if (bands.bands[ConfigData.currentBand].sideband == Sideband::LOWER)
  {
    arm_max_q15(&pixelnew[(cal_bins[0] - capture_bins / 2)], capture_bins, &refAmplitude, &index_of_max);
    arm_max_q15(&pixelnew[(cal_bins[1] - capture_bins / 2)], capture_bins, &adjAmplitude, &index_of_max);
  }
  if (bands.bands[ConfigData.currentBand].sideband == Sideband::UPPER)
  {
    arm_max_q15(&pixelnew[(cal_bins[0] - capture_bins / 2)], capture_bins, &refAmplitude, &index_of_max);
    arm_max_q15(&pixelnew[(cal_bins[1] - capture_bins / 2)], capture_bins, &adjAmplitude, &index_of_max);
  }

  y_old2_plot = 135 + (-y_old2 + rawSpectrumPeak);
  y_old_plot = 135 + (-y_old + rawSpectrumPeak);
  y1_new_plot = 135 + (-y1_new + rawSpectrumPeak);
  y_new_plot = 135 + (-y_new + rawSpectrumPeak);

  // Prevent spectrum from going above the top of the spectrum area.  KF5N
  if (y_new_plot < 120)
    y_new_plot = 120;
  if (y1_new_plot < 120)
    y1_new_plot = 120;
  if (y_old_plot < 120)
    y_old_plot = 120;
  if (y_old2_plot < 120)
    y_old2_plot = 120;

  // The prevents spectrum from going below lower limit.
  if (y_new_plot > base_y)
    y_new_plot = base_y;
  if (y_old_plot > base_y)
    y_old_plot = base_y;
  if (y_old2_plot > base_y)
    y_old2_plot = base_y;
  if (y1_new_plot > base_y)
    y1_new_plot = base_y;

  // Cast to float and calculate the dB level.  Needs further refinement for accuracy.  KF5N
  adjdB = (static_cast<float>(adjAmplitude) - static_cast<float>(refAmplitude)) / (1.95 * 2.0);
}

void RxCalibrate::computeAdjdB()
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
