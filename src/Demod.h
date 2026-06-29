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

// Demod class

class Demod {

public:

float32_t dc = 0.0;
float32_t dc_insert = 0.0;
float32_t dcu = 0.0;
float32_t dc_insertu = 0.0;
float32_t del_out = 0.0;
float32_t omega2 = 0.0;
float32_t phzerror = 0.0;
float32_t det = 0.0;
float32_t fil_out = 0.0;
float32_t SAM_carrier = 0.0;                 //AFP 11-02-22
float32_t SAM_lowpass = 2700.0;              //AFP 11-02-22
float32_t SAM_carrier_freq_offset = 0.0;     //AFP 11-02-22
float32_t SAM_carrier_freq_offsetOld = 0.0;  //AFP 11-02-22
float32_t ai, bi, aq, bq;
float32_t ai_ps, bi_ps, aq_ps, bq_ps;
float32_t audiou;
float32_t Cos = 0.0;
float32_t samOffset{0};

void AMDecodeSAM();
float ApproxAtan2(float y, float x);

};