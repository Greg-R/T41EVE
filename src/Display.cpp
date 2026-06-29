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

/*****
  Purpose: Show Spectrum display with auto RF gain.  Harry GM3RVL, January 16, 2024
            Note that this routine calls the Audio process Function during each display cycle,
            for each of the 512 display frequency bins.  This means that the audio is refreshed at the maximum rate
            and does not have to wait for the display to complete drawing the full spectrum.
            However, the display data are only updated ONCE during each full display cycle,
            ensuring consistent data for the erase/draw cycle at each frequency point.

  Parameter list:
    void

  Return value;
    void
*****/
void Display::ShowSpectrum(bool drawSpectrum)
{

  int AudioH_max = 0, AudioH_max_box = 0; // Used to center audio spectrum.
  int audio_hist[256]{0};                 // All values are initialized to zero using this syntax.
  int k;
  int middleSlice = centerLine / 2; // Approximate center element
  int x1 = 0;                       // AFP
              //   int h = SPECTRUM_HEIGHT + 7;
  int y1_new{0};
  int test1;
  updateDisplayCounter = 0;
  updateDisplayFlag = false;
  spectrumErased = false;
  bool plotInProgress = false;
  bool blocksAvailable = false;

  for (x1 = 0; x1 < 512; x1++)
  { // Bins on the ends are junk, don't plot.

    // Quit and start transmitter if key is pressed.
    if (keyPressedOn)
    {
      return;
    }
    // Draws the main Spectrum, Waterfall and Audio displays

    // Is there enough data to calculate an FFT?
    blocksAvailable = (static_cast<uint32_t>(ADC_RX_I.available()) > 15) and (static_cast<uint32_t>(ADC_RX_Q.available()) > 15);

    // 1X zoom.
    if ((ConfigData.spectrum_zoom == 0) and (not plotInProgress) and blocksAvailable)
    {
      x1 = 1; // This forces enough calls of ProcessIQData() to generate enough blocks before x1 > 511.
      updateDisplayFlag = true;
      plotInProgress = true;
    }
    // 2X zoom.
    if ((ConfigData.spectrum_zoom == 1) and (not plotInProgress) and blocksAvailable)
    {
      x1 = 1;
      updateDisplayFlag = true;
      plotInProgress = true;
    }
    // 4X zoom.
    if ((ConfigData.spectrum_zoom == 2) and (not plotInProgress) and blocksAvailable)
    {
      x1 = 1;
      updateDisplayFlag = true;
      plotInProgress = true;
    }
    // 8X zoom.
    if ((ConfigData.spectrum_zoom == 3) and (not plotInProgress) and blocksAvailable)
    {
      updateDisplayCounter = updateDisplayCounter + 1;
      x1 = 1; // This forces enough calls of ProcessIQData() to generate enough blocks before x1 > 511.
      if (updateDisplayCounter == 3)
      {
        updateDisplayFlag = true;
        plotInProgress = true;
      }
    }
    // 16X zoom.
    if ((ConfigData.spectrum_zoom == 4) and (not plotInProgress) and blocksAvailable)
    {
      updateDisplayCounter = updateDisplayCounter + 1;
      x1 = 1; // This forces enough calls of ProcessIQData() to generate enough blocks before x1 > 511.
      if (updateDisplayCounter == 7)
      {
        updateDisplayFlag = true;
        plotInProgress = true;
      }
    }

    if (startRxFlag) updateDisplayFlag = false; // Don't process data the first time after coming out of transmit mode.
    startRxFlag = false;

    // This has to be repeatedly called for continuous audio.
    // This function returns if there are not enough 128 buffers in the queues.
////    process.ProcessIQData();

    // Don't call this function unless the filter bandwidth has been adjusted.
    if (encoderFilterFlag)
    {
      FilterSetSSB();
    }

    //  EncoderCenterTune();  //Moved the tuning encoder to reduce lag times and interference during tuning.

//    if (not plotInProgress)
//    {
//      delayMicroseconds(100); // WOW we have to delay to allow the data to catch up!
//      continue;               // We are not plotting, so go to the next iteration.  Skip the following plotting stuff.
//    }

    y1_new = pixelnew[x1];

    // Collect a histogram of audio spectral values.  This is used to keep the audio spectrum in the viewable area.
    // 247??? is the spectral display bottom.  129 is the audio spectrum display top.
    if ((x1 < 256) and (audioYPixel[x1] > 0)) //  Collect audio frequency distribution to find noise floor.
    {
      k = audioYPixel[x1]; // +40 to get 10 bins below zero - want to straddle zero to make the entire spectrum viewable.
      audio_hist[k] += 1;  // Add (accumulate) to the bin.
                           //       if(x1 == 50) Serial.printf("audio_hist[k] = %d AudioH_max = %d\n", audio_hist[k], AudioH_max);
      if (audio_hist[k] > AudioH_max)
      {                             // FH_max starts at 0.
        AudioH_max = audio_hist[k]; // Reset FH_max to the current bin value.
        AudioH_max_box = k;         // Index of FH_max.  this corresponds to the noise floor.
                                    //        Serial.printf("k = %d AudioH_max_box = %d\n", k, AudioH_max_box);
      }
    } //  HB finish

    // Draw audio spectrum.  The audio spectrum width is smaller than the RF spectrum width.
    // The audio spectrum arrays are generated in ReceiveDSP.cpp by method ProcessIQData().
    if (x1 < 253)
    { // AFP 09-01-22
      //       if (keyPressedOn == true) {                                                                 //AFP 09-01-22
      //                                                                                                   ////        return;
      //       } else {                                                                                    //AFP 09-01-22
      //    if (audioYPixelold[x1] > CLIP_AUDIO_PEAK) audioYPixelold[x1] = CLIP_AUDIO_PEAK;           // audioSpectrumHeight = 118
      if (audioYPixel[x1] != 0)
      {
        if (audioYPixel[x1] > CLIP_AUDIO_PEAK) // audioSpectrumHeight = 118
          audioYPixel[x1] = CLIP_AUDIO_PEAK;
        if (x1 == middleSlice)
        {
          smeterLength = y_new;
        }
        // Draw a vertical line with the audio spectrum magnitude.  AUDIO_SPECTRUM_BOTTOM = 247
      }
    }


    test1 = -y1_new + 236; // Nudged waterfall towards blue.  KF5N July 23, 2023

    if (test1 < 0)
      test1 = 0;
    if (test1 > 116)
      test1 = 116;
    //    waterfall[x1] = convert_rgb565_to_rgb332(gradient[test1]);  // Try to put pixel values in middle of gradient array.  KF5N
    waterfall[x1] = signalToRGB332(static_cast<int32_t>((static_cast<float32_t>(test1) * 2.2)));
  } // End 512 for(...) Draw MAX_WATERFALL_WIDTH spectral points

  // Manage audio spectral display graphics.  Keep the spectrum within the viewable area.
  if (AudioH_max_box > 30)
  { // HB. Adjust rfGainAllBands 15 and 13 to alter to move target base up and down. UPPERPIXTARGET = 15
    audioFFToffset = audioFFToffset - 1;
  }
  if (AudioH_max_box < 28)
  { // LOWERPIXTARGET = 13
    audioFFToffset = audioFFToffset + 1;
  }

    
} // End ShowSpectrum()

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
  //  char buff[10];
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

uint8_t Display::convert_rgb565_to_rgb332(uint16_t rgb565_color)
{
  // 1. Extract the original 5-bit Red, 6-bit Green, and 5-bit Blue components
  // Red: Mask the top 5 bits (0b1111100000000000), then shift right 11 bits
  uint8_t r5 = (rgb565_color & 0xF800) >> 11;
  // Green: Mask the middle 6 bits (0b0000011111100000), then shift right 5 bits
  uint8_t g6 = (rgb565_color & 0x07E0) >> 5;
  // Blue: Mask the bottom 5 bits (0b0000000000011111)
  uint8_t b5 = rgb565_color & 0x001F;

  // 2. Downscale components to the new bit depths (3 bits Red, 3 bits Green, 2 bits Blue)
  // Simple right shift truncates the least significant bits.
  uint8_t r3 = r5 >> 2; // Lose 2 LSBs (5 bits -> 3 bits)
  uint8_t g3 = g6 >> 3; // Lose 3 LSBs (6 bits -> 3 bits)
  uint8_t b2 = b5 >> 3; // Lose 3 LSBs (5 bits -> 2 bits)

  // 3. Combine the new components into a single 8-bit value
  // Shift R3 to the top 3 bits, G3 to the middle 3 bits, B2 to the bottom 2 bits, and combine with OR
  uint8_t rgb332_color = (r3 << 5) | (g3 << 2) | b2;

  return rgb332_color;
}

uint8_t Display::signalToRGB332(uint8_t signal)
{
  uint8_t red = 0;
  uint8_t green = 0;
  uint8_t blue = 0;

  if (signal < 51)
  {
    // black -> blue
    blue = signal * 3 / 51;
  }
  else if (signal < 102)
  {
    // blue -> cyan
    uint8_t t = signal - 51;
    blue = 3;
    green = t * 7 / 51;
  }
  else if (signal < 153)
  {
    // cyan -> green
    uint8_t t = signal - 102;
    blue = (51 - t) * 3 / 51;
    green = 7;
  }
  else if (signal < 204)
  {
    // green -> yellow
    uint8_t t = signal - 153;
    green = 7;
    red = t * 7 / 51;
  }
  else
  {
    // yellow -> red
    uint8_t t = signal - 204;
    green = (51 - t) * 7 / 51;
    red = 7;
  }

  // rrrgggbb
  return (red << 5) | (green << 2) | blue;
}