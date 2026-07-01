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

// CW Exciter class

#include "SDT.h"

/*****
  Purpose: Load buffers used to modulate the transmitter during calibration.
          The epilogue must restore the buffers for normal operation!

   Parameter List:
      void

   Return value:
      void
 *****/
void CW_Exciter::writeSineBuffer(int numCycles) {
  float32_t theta{ 0.0 };
  float32_t freqSideTone{ 0.0 };
  const float32_t amplitude{ 0.12 };  // Sets the peak amplitude of the sinusoid.
  freqSideTone = static_cast<float>(numCycles) * 48000.0 / 512.0;
  for (int kf = 0, increment = 0; kf < 512; increment += 1, kf++) {  // Calc: numCycles = 8, 750 hz sine wave.
    theta = static_cast<float32_t>(increment) * 2.0 * PI * freqSideTone / 48000.0;
    sineBuffer[kf] = amplitude * sin(theta);  // Create the CW tone waveform.
  }
}


/*****
  Purpose: Create I and Q signals for CW transmission.

  Parameter list:
    int shaping   one of CW_SHAPING_RISE, CW_SHAPING_FALL, or CW_SHAPING_NONE

  Return value;
    void
*****/
void CW_Exciter::CW_ExciterIQData(int shaping)  //AFP 08-20-22
{
  float32_t powerScale = 0.0;

  if (shaping == CW_SHAPING_RISE) {
    arm_mult_f32(sineBuffer, cwRiseBuffer, float_buffer_cw, 512);
  } else if (shaping == CW_SHAPING_FALL) {
    arm_mult_f32(sineBuffer, cwFallBuffer, float_buffer_cw, 512);
  } else if (shaping == CW_SHAPING_ZERO) {
    arm_scale_f32(sineBuffer, 0.0, float_buffer_cw, 512);
  } else {
    arm_scale_f32(sineBuffer, 1.0, float_buffer_cw, 512);
  }

  cwToneData.setBehaviour(AudioPlayQueue_F32::ORIGINAL);
  cwToneData.setMaxBuffers(4);
  cwToneData.play(float_buffer_cw, 512);  // Push CW waveform into SSB transmitter input.

  // Make Q15 data for CW sidetone.
//  arm_float_to_q15(float_buffer_cw, q15_buffer_Sidetone, 512);

  float32_t float_buffer_i[512] = { 0.0 };
  float32_t float_buffer_q[512] = { 0.0 };
  float32_t* iBuffer = nullptr;  // I and Q pointers needed for one-time read of record queues.
  float32_t* qBuffer = nullptr;

  // Read incoming I and Q audio blocks from the SSB exciter.
  if (Q_in_L_Ex.available() > 3 and Q_in_R_Ex.available() > 3) {
    for (int i = 0; i < 4; i = i + 1) {
      iBuffer = Q_in_L_Ex.readBuffer();
      qBuffer = Q_in_R_Ex.readBuffer();
      std::copy(iBuffer, iBuffer + 128, &float_buffer_i[128 * i]);
      std::copy(qBuffer, qBuffer + 128, &float_buffer_q[128 * i]);
      Q_in_L_Ex.freeBuffer();
      Q_in_R_Ex.freeBuffer();
    }
  } else return;

    //  This is the correct place in the data flow to inject the scaling for power.
  powerScale = 2.0 * ConfigData.powerOutCW[ConfigData.currentBand];

  arm_scale_f32(float_buffer_i, powerScale, float_buffer_i, 512);
  arm_scale_f32(float_buffer_q, powerScale, float_buffer_q, 512);

  arm_offset_f32(float_buffer_i, CalData.iDCoffsetCW[ConfigData.currentBand] + CalData.dacOffsetCW, float_buffer_i, 512);  // Carrier suppression offset.
  arm_offset_f32(float_buffer_q, CalData.qDCoffsetCW[ConfigData.currentBand] + CalData.dacOffsetCW, float_buffer_q, 512);

//  Q_out_L_Ex.setBehaviour(AudioPlayQueue_F32::ORIGINAL);
//  Q_out_R_Ex.setBehaviour(AudioPlayQueue_F32::ORIGINAL);

  // Play audio.
  Q_out_L_Ex.play(float_buffer_i, 512);  // play it!  This is the I channel from the Audio Adapter line out to QSE I input.
  Q_out_R_Ex.play(float_buffer_q, 512);  // play it!  This is the Q channel from the Audio Adapter line out to QSE Q input.

//  Q_out_L.play(q15_buffer_Sidetone, 512);
  sidetoneOutQueue.play(float_buffer_cw, 512);
}
