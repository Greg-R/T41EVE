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

// Display class

#include "SDT.h"


// DB2OO, 30-AUG-23: this variable determines the pixels per S step. In the original code it was 12.2 pixels !?
#ifdef TCVSDR_SMETER
const float pixels_per_s = 12;
#else
const float pixels_per_s = 12.2;
#endif

/*****
  Purpose: Display signal level in dBm.
  Parameter list:
    void
  Return value;
    void
*****/
void Display::DisplaydbM()
{
  float32_t rfGain;
#ifdef TCVSDR_SMETER
  const float32_t slope = 10.0;
  const float32_t cons = -92;
#else
  float32_t audioLogAveSq;
#endif

  // DB2OO, 30-AUG-23: the S-Meter bar and the dBm value were inconsistent, as they were using different base values.
  //  Moreover the bar could go over the limits of the S-meter box, as the map() function, does not constrain the values
  //  with TCVSDR_SMETER defined the S-Meter bar will be consistent with the dBm value and the S-Meter bar will always be restricted to the box
#ifdef TCVSDR_SMETER
  // DB2OO, 9-OCT_23: dbm_calibration set to -22 in SDT.ino; gainCorrection is a value between -2 and +6 to compensate the frequency dependant pre-Amp gain
  //  attenuator is 0 and could be set in a future HW revision; RFgain is initialized to 1 in the bands.bands[] init in SDT.ino; cons=-92; slope=10
  //  if (ConfigData.autoGain) rfGain = ConfigData.rfGainCurrent;
  rfGain = ConfigData.rfGain[ConfigData.currentBand];
  dbm = CalData.dBm_calibration + bands.bands[ConfigData.currentBand].gainCorrection + static_cast<float32_t>(attenuator) + slope * log10f_fast(audioMaxSquaredAve) + cons - static_cast<float32_t>(bands.bands[ConfigData.currentBand].RFgain) * 1.5 - rfGain; // DB2OO, 08-OCT-23; added ConfigData.rfGainAllBands
#else
  // DB2OO, 9-OCT-23: audioMaxSquaredAve is proportional to the input power. With ConfigData.rfGainAllBands=0 it is approx. 40 for -73dBm @ 14074kHz with the V010 boards and the pre-Amp fed by 12V
  //  for audioMaxSquaredAve=40 audioLogAveSq will be 26
  audioLogAveSq = 10 * log10f_fast(audioMaxSquaredAve) + 10; // AFP 09-18-22
  // DB2OO, 9-OCT-23: calculate dBm value from audioLogAveSq and ignore band gain differences and a potential attenuator like in original code
  dbm = audioLogAveSq - 100;

  // DB2OO, 9-OCT-23: this is the orginal code, that will map a 30dB difference (35-5) to 60 (635-575) pixels, i.e. 5 S steps to 5*12 pixels
  //  SMETER_X is 528 --> X=635 would be 107 pixels / 12pixels per S step --> approx. S9
  smeterPad = map(audioLogAveSq, 5, 35, 575, 635); // AFP 09-18-22

#endif
  // Determine length of S-meter bar, limit it to the box and draw it
  smeterPad = map(dbm, -73.0 - 9 * 6.0 /*S1*/, -73.0 /*S9*/, 0, 9 * pixels_per_s);
  // DB2OO; make sure, that it does not extend beyond the field
  smeterPad = max(0, smeterPad);
  smeterPad = min(SMETER_BAR_LENGTH, smeterPad);

  // DB2OO, 17-AUG-23: create PWM analog output signal on the "HW_SMETER" output. This is scaled for a 250uA  S-meter full scale,
  //  connected to HW_SMTER output via a 8.2kOhm resistor and a 4.7kOhm resistor and 10uF capacitor parallel to the S-Meter
#ifdef HW_SMETER
  {
    int hw_s;
    hw_s = map((int)dbm - 3, -73 - (8 * 6), -73 + 60, 0, 228);
    hw_s = max(0, min(hw_s, 255));
    analogWrite(HW_SMETER, hw_s);
  }
#endif

  // DB2OO, 13-OCT-23: if DEBUG_SMETER defined debug messages on S-Meter variables will be put out
  // #define DEBUG_SMETER

#ifdef DEBUG_SMETER
  // added, to debug S-Meter display problems
  Serial.printf("DisplaydbM(): dbm=%.1f, dbm_calibration=%.1f, bands.bands[ConfigData.currentBand].gainCorrection=%.1f, attenuator=%d, bands.bands[ConfigData.currentBand].RFgain=%d, ConfigData.rfGainAllBands=%d\n",
                dbm, dbm_calibration, bands.bands[ConfigData.currentBand].gainCorrection, attenuator, bands.bands[ConfigData.currentBand].RFgain, ConfigData.rfGainAllBands);
  Serial.printf("\taudioMaxSquaredAve=%.4f, audioLogAveSq=%.1f\n", audioMaxSquaredAve, audioLogAveSq);
#endif
}

/*****
  Purpose: Display the current temperature and load figures for Teensy 4.1.

  Parameter list:
    int notchF        the notch to use
    int MODE          the current MODE

  Return value;
    void
*****/
void Display::ShowTempAndLoad()
{
  double block_time;

  elapsed_micros_mean = elapsed_micros_sum / elapsed_micros_idx_t;

  block_time = 128.0 / (double)SR[SampleRate].rate; // one audio block is 128 samples and uses this in seconds
  block_time = block_time * N_BLOCKS;

  block_time *= 1000000.0;                                 // now in µseconds
  processor_load = elapsed_micros_mean / block_time * 100; // take audio processing time divide by block_time, convert to %

  if (processor_load >= 100.0)
  {
    processor_load = 100.0;
  }

  CPU_temperature = TGetTemp();

  elapsed_micros_idx_t = 0;
  elapsed_micros_sum = 0;
  elapsed_micros_mean = 0;
}

/*****
  Purpose: This function redraws the entire display screen.

  Parameter list:
    void

  Return value;
    void
*****/
void Display::RedrawAll()
{
  SetBandRelay();                  // Set LPF relays for current band.
  lastState = RadioState::NOSTATE; // Force an update.
}

/*****
  Purpose: DisplayClock()
  Parameter list:
    void
  Return value;
    void
*****/
void Display::DisplayClock()
{
  char temp[5];

  temp[0] = '\0';
  timeBuffer[0] = '\0';
  strcpy(timeBuffer, MY_TIMEZONE); // e.g., EST
#ifdef TIME_24H
  // DB2OO, 29-AUG-23: use 24h format
  itoa(hour(), temp, DEC);
#else
  itoa(hourFormat12(), temp, DEC);
#endif
  if (strlen(temp) < 2)
  {
    strcat(timeBuffer, "0");
  }
  strcat(timeBuffer, temp);
  strcat(timeBuffer, ":");

  itoa(minute(), temp, DEC);
  if (strlen(temp) < 2)
  {
    strcat(timeBuffer, "0");
  }
  strcat(timeBuffer, temp);
  strcat(timeBuffer, ":");

  itoa(second(), temp, DEC);
  if (strlen(temp) < 2)
  {
    strcat(timeBuffer, "0");
  }
  strcat(timeBuffer, temp);
} // end function Displayclock

