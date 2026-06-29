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

// Demodulation algorithms

#include "SDT.h"


/*****  AFP 11-03-22
  Purpose: AMDecodeSAM()
  Parameter list:
    void
  Return value;
    void
  Notes:  Synchronous AM detection.  Determines the carrier frequency, adjusts freq and replaces the received carrier with a steady signal to prevent fading.
  This algorithm works best of those implemented.
  // taken from Warren Pratt´s WDSP, 2016
  // http://svn.tapr.org/repos_sdr_hpsdr/trunk/W5WC/PowerSDR_HPSDR_mRX_PS/Source/wdsp/
*****/
void Demod::AMDecodeSAM() {
  int zeta_help = 65;
  float32_t zeta = (float32_t)zeta_help / 100.0;  // PLL step response: smaller, slower response 1.0 - 0.1
  float32_t g1 = 1.0 - exp(-2.0 * ConfigData.omegaN * zeta * 1 / 24000);
  float32_t g2 = -g1 + 2.0 * (1 - exp(-ConfigData.omegaN * zeta * 1 / 24000) * cosf(ConfigData.omegaN * 1 / 24000 * sqrtf(1.0 - zeta * zeta)));
  const float32_t omega_min = TWO_PI * -ConfigData.pll_fmax * 1 / 24000;
  const float32_t omega_max = TWO_PI * ConfigData.pll_fmax * 1 / 24000;
  const float32_t tauR = 0.02;  // original 0.02;
  const float32_t tauI = 1.4;   // original 1.4;
  float32_t mtauR = exp(-1 / 24000 * tauR);
  float32_t onem_mtauR = 1.0 - mtauR;
  float32_t mtauI = exp(-1 / 24000 * tauI);
  float32_t onem_mtauI = 1.0 - mtauI;
  uint8_t fade_leveler = 1;
  float32_t Sin, Cos;

  for (unsigned i = 0; i < FFT_length / 2; i++) {
    Sin = arm_sin_f32(phzerror);
    Cos = arm_cos_f32(phzerror);
    ai = Cos * iFFT_buffer[FFT_length + i * 2];
    bi = Sin * iFFT_buffer[FFT_length + i * 2];
    aq = Cos * iFFT_buffer[FFT_length + i * 2 + 1];
    bq = Sin * iFFT_buffer[FFT_length + i * 2 + 1];
    corr[0] = +ai + bq;
    corr[1] = -bi + aq;

    audio = (ai - bi) + (aq + bq);

    if (fade_leveler) {
      dc = mtauR * dc + onem_mtauR * audio;
      dc_insert = mtauI * dc_insert + onem_mtauI * corr[0];
      audio = audio + dc_insert - dc;
    }
    float_buffer_L[i] = audio;

    if (fade_leveler) {
      dcu = mtauR * dcu + onem_mtauR * audiou;
      dc_insertu = mtauI * dc_insertu + onem_mtauI * corr[0];
      audiou = audiou + dc_insertu - dcu;
    }
    float_buffer_R[i] = audiou;

    det = ApproxAtan2(corr[1], corr[0]);

    del_out = fil_out;
    omega2 = omega2 + g2 * det;
    if (omega2 < omega_min) omega2 = omega_min;
    else if (omega2 > omega_max) omega2 = omega_max;
    fil_out = g1 * det + omega2;
    phzerror = phzerror + del_out;

    //Wrap round 2PI, modulus
    while (phzerror >= TWO_PI) phzerror -= TWO_PI;
    while (phzerror < 0.0) phzerror += TWO_PI;
  }

  // In the small frequency display
  // we calculate carrier offset here and the display function is
  // then called in main loop every 100ms.
  // To make this smoother, a simple lowpass/exponential averager here . . .
  SAM_carrier = 0.08 * (omega2 * 24000) / (2 * TWO_PI);
  SAM_carrier = SAM_carrier + 0.92 * SAM_lowpass;
  SAM_carrier_freq_offset = static_cast<int>(10 * SAM_carrier);
  SAM_carrier_freq_offset = 0.9 * SAM_carrier_freq_offsetOld + 0.1 * SAM_carrier_freq_offset;
  SAM_lowpass = SAM_carrier;

  samOffset = 0.20024 * SAM_carrier_freq_offset;
  SAM_carrier_freq_offsetOld = SAM_carrier_freq_offset;
}

/*****  AFP 11-03-22
  Purpose: ApproxAtan2
  Parameter list:
    void
  Return value;
    void
*****/
float Demod::ApproxAtan2(float y, float x) {
  if (x != 0.0f) {
    if (fabsf(x) > fabsf(y)) {
      const float z = y / x;
      if (x > 0.0f) {
        // atan2(y,x) = atan(y/x) if x > 0
        return ApproxAtan(z);
      } else if (y >= 0.0f) {
        // atan2(y,x) = atan(y/x) + PI if x < 0, y >= 0
        return ApproxAtan(z) + PI;
      } else {
        // atan2(y,x) = atan(y/x) - PI if x < 0, y < 0
        return ApproxAtan(z) - PI;
      }
    } else  // Use property atan(y/x) = PI/2 - atan(x/y) if |y/x| > 1.
    {
      const float z = x / y;
      if (y > 0.0f) {
        // atan2(y,x) = PI/2 - atan(x/y) if |y/x| > 1, y > 0
        return -ApproxAtan(z) + TWO_PI;
      } else {
        // atan2(y,x) = -PI/2 - atan(x/y) if |y/x| > 1, y < 0
        return -ApproxAtan(z) - TWO_PI;
      }
    }
  } else {
    if (y > 0.0f)  // x = 0, y > 0
    {
      return TWO_PI;
    } else if (y < 0.0f)  // x = 0, y < 0
    {
      return -TWO_PI;
    }
  }
  return 0.0f;  // x,y = 0. Could return NaN instead.
}
