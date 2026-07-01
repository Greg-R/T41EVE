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

// The SSB exciter sketch code.  Most of the SSB (and CW) exciter is in AudioSignal.h.

#include "SDT.h"

float32_t* iBuffer = nullptr;  // I and Q pointers needed for one-time read of record queues.
float32_t* qBuffer = nullptr;

// This function sets the microphone gain and compressor parameters.  Greg KF5N March 9, 2025.
void updateMic() {

  micGain.setGain_dB(ConfigData.micGain);  // Set the microphone gain.

  struct compressionCurve crv = { -3.0, 0.0,  // margin, offset
                                  { 0.0, -10.0, static_cast<float>(ConfigData.micThreshold), -1000.0f, -1000.0f },
                                  { 10.0, static_cast<float>(ConfigData.micCompRatio), 1.0f, 1.0, 1.0 } };

  int16_t delaySize = 256;                        // Any power of 2, i.e., 256, 128, 64, etc.
  compressor1.setDelayBufferSize(delaySize);      // Improves transient response of compressor.
  compressor1.setAttackReleaseSec(0.005f, 2.0f);  // Same as used in Tiny Ten by Bob W7PUA.
  compressor1.setCompressionCurve(&crv);
  compressor1.begin();
}


/*****
  Purpose: Retrieve I and Q samples from the Open Audio Library CESSB object at 48ksps.
           Apply calibration factors and scale for power.  Push the modified I and Q
           back into the Teensy audio system to drive the Audio Adapter outputs which
           are connected to the QSE.

  Parameter list: none

  Return value;
    void
    Notes:
    There are several actions in this function
    1.  Read in the I and Q data from the CESSB object.  16 blocks of 128 samples each.
    2.  Apply magnitude and phase calibration factors.
    3.  Scale for power.
    4.  Push the blocks back to the Teensy audio system to the Audio Adapter outputs
        at 48ksps.
*****/

void SSB_ExciterIQData() {
  uint32_t N_BLOCKS_EX = N_B_EX;
  float32_t powerScale;

  /**********************************************************************************  AFP 12-31-20
        Get samples from queue buffers
        Teensy Audio Library stores ADC data in two buffers size=128, Q_in_L and Q_in_R as initiated from the audio lib.
        Then the buffers are read into two arrays in blocks of 128 up to N_BLOCKS.  The arrays are
        of size BUFFER_SIZE*N_BLOCKS.  BUFFER_SIZE is 128.
        N_BLOCKS = FFT_L / 2 / BUFFER_SIZE * (uint32_t)DF; // should be 16 with DF == 8 and FFT_L = 512
        BUFFER_SIZE * N_BLOCKS = 2048 samples
     **********************************************************************************/
  // Are there at least N_BLOCKS buffers in each channel available ?
  if (static_cast<uint32_t>(Q_in_L_Ex.available()) < 32 or static_cast<uint32_t>(Q_in_R_Ex.available()) < 32) {
    return;
  }

  // Get audio samples from the audio buffers and convert them to float.
  for (unsigned i = 0; i < N_BLOCKS_EX; i++) {

    iBuffer = Q_in_L_Ex.readBuffer();
    qBuffer = Q_in_R_Ex.readBuffer();
    std::copy(iBuffer, iBuffer + 128, &float_buffer_L_EX[128 * i]);
    std::copy(qBuffer, qBuffer + 128, &float_buffer_R_EX[128 * i]);

    Q_in_L_Ex.freeBuffer();
    Q_in_R_Ex.freeBuffer();  // Right channel not used.  KF5N March 11, 2024
  }

  // Set the sideband.
  if (bands.bands[ConfigData.currentBand].sideband == Sideband::LOWER) cessb1.setSideband(false);
  if (bands.bands[ConfigData.currentBand].sideband == Sideband::UPPER) cessb1.setSideband(true);

  // Apply amplitude and phase corrections.  FT8 uses CW corrections and is always USB.
  if (bands.bands[ConfigData.currentBand].sideband == Sideband::LOWER) {
    if (bands.bands[ConfigData.currentBand].mode == RadioMode::SSB_MODE)
      cessb1.setIQCorrections(true, CalData.IQSSBAmpCorrectionFactorLSB[ConfigData.currentBand], CalData.IQSSBPhaseCorrectionFactorLSB[ConfigData.currentBand], 0.0);
    else if (bands.bands[ConfigData.currentBand].mode == RadioMode::FT8_MODE)
      cessb1.setIQCorrections(true, CalData.IQCWAmpCorrectionFactorUSB[ConfigData.currentBand], CalData.IQCWPhaseCorrectionFactorUSB[ConfigData.currentBand], 0.0);
  } else if (bands.bands[ConfigData.currentBand].sideband == Sideband::UPPER) {
    if (bands.bands[ConfigData.currentBand].mode == RadioMode::SSB_MODE)
      cessb1.setIQCorrections(true, CalData.IQSSBAmpCorrectionFactorUSB[ConfigData.currentBand], CalData.IQSSBPhaseCorrectionFactorUSB[ConfigData.currentBand], 0.0);
    else if (bands.bands[ConfigData.currentBand].mode == RadioMode::FT8_MODE)
      cessb1.setIQCorrections(true, CalData.IQCWAmpCorrectionFactorUSB[ConfigData.currentBand], CalData.IQCWPhaseCorrectionFactorUSB[ConfigData.currentBand], 0.0);
  }

  //  This is the correct place in the data flow to inject the scaling for power.
  powerScale = 2.0 * ConfigData.powerOutSSB[ConfigData.currentBand];

  arm_scale_f32(float_buffer_L_EX, powerScale, float_buffer_L_EX, 2048);  //Scale to compensate for losses in Interpolation
  arm_scale_f32(float_buffer_R_EX, powerScale, float_buffer_R_EX, 2048);

  if (bands.bands[ConfigData.currentBand].mode == RadioMode::SSB_MODE) {
    arm_offset_f32(float_buffer_L_EX, CalData.iDCoffsetSSB[ConfigData.currentBand] + CalData.dacOffsetSSB, float_buffer_L_EX, 2048);  // Carrier suppression offset.
    arm_offset_f32(float_buffer_R_EX, CalData.qDCoffsetSSB[ConfigData.currentBand] + CalData.dacOffsetSSB, float_buffer_R_EX, 2048);
  } else if (bands.bands[ConfigData.currentBand].mode == RadioMode::FT8_MODE) {
    arm_offset_f32(float_buffer_L_EX, CalData.iDCoffsetCW[ConfigData.currentBand] + CalData.dacOffsetCW, float_buffer_L_EX, 2048);  // Carrier suppression offset.
    arm_offset_f32(float_buffer_R_EX, CalData.qDCoffsetCW[ConfigData.currentBand] + CalData.dacOffsetCW, float_buffer_R_EX, 2048);
  }

  Q_out_L_Ex.setBehaviour(AudioPlayQueue_F32::ORIGINAL);
  Q_out_R_Ex.setBehaviour(AudioPlayQueue_F32::ORIGINAL);

  // Play audio.
  Q_out_L_Ex.play(float_buffer_L_EX, 2048);  // play it!  This is the I channel from the Audio Adapter line out to QSE I input.
  Q_out_R_Ex.play(float_buffer_R_EX, 2048);  // play it!  This is the Q channel from the Audio Adapter line out to QSE Q input.
}

/*****
  Purpose: Set the current band relay ON or OFF.  Reduce relay cycling.  Greg KF5N March 24, 2025

  Parameter list:
    void

  Return value;
    void
*****/
void SetBandRelay() {
  // There are 4 physical relays in the case of the V10/V11 LPF board.
  for (int i = 0; i < 5; i = i + 1) {
    if (i == ConfigData.currentBand) {
      digitalWrite(bandswitchPins[ConfigData.currentBand], HIGH);
    } else {
      if (bandswitchPins[i] != bandswitchPins[ConfigData.currentBand])  // Skip if the pins are the same.
        digitalWrite(bandswitchPins[i], LOW);                           // Set band relay low.
    }
  }
}
