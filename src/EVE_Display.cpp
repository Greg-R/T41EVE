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


#include "SDT.h"

// Constructor.
EVE_Display::EVE_Display()
{
  /*
  pinMode(10, OUTPUT);   // SPI chip select.
  digitalWrite(10, HIGH);

  SPI.begin(); // sets up the SPI to run in Mode 0 and 1 MHz
  // switch to 8MHz, note, init must be done with <11MHz
  SPI.beginTransaction(SPISettings(4UL * 1000000UL, MSBFIRST, SPI_MODE0));
  tft_init();
  SPI.endTransaction();
  SPI.beginTransaction(SPISettings(8UL * 1000000UL, MSBFIRST, SPI_MODE0));
  */
}

// Initialize the EVE display.
void EVE_Display::initialize()
{

  pinMode(10, OUTPUT);   // SPI chip select.
  digitalWrite(10, HIGH);

    // Configure GPIO 10 for high speed.
  uint32_t iospeed_display = IOMUXC_PAD_DSE(3) | IOMUXC_PAD_SPEED(1);
  *(digital_pin_to_info_PGM + 10)->pad = iospeed_display;

  SPI.begin(); /* sets up the SPI to run in Mode 0 and 1 MHz */
  /* switch to 8MHz, note, init must be done with <11MHz */
  SPI.beginTransaction(SPISettings(4UL * 1000000UL, MSBFIRST, SPI_MODE0));
//  tft_init();

  if (E_OK == EVE_init())
  {
    tft_active = 1;
    EVE_memWrite32(REG_PWM_DUTY, 0x30); /* setup backlight, range is from 0 = off to 0x80 = max */
  }

  // Load RAM_G with static command lists.
  receiverStatic_cmd_list();
  directEntryStatic_cmd_list();
  buttonEntryStatic_cmd_list();
  encoderEntryStatic_cmd_list();
  transmitterCalStatic_cmd_list();
  receiverCalStatic_cmd_list();
  equalizerAdjustStatic_cmd_list();
  switchMatrixCalStatic_cmd_list();
  transmitterStatic_cmd_list();

  SPI.endTransaction();
  SPI.beginTransaction(SPISettings(12UL * 1000000UL, MSBFIRST, SPI_MODE0));
}

// Introductory display.
FLASHMEM void EVE_Display::drawSplash()
{
  EVE_cmd_dl(CMD_DLSTART);
  EVE_cmd_dl(DL_CLEAR_COLOR_RGB | 0xFFFFFF);
  EVE_cmd_dl(DL_CLEAR | CLR_COL | CLR_STN | CLR_TAG);
  EVE_color_rgb(0x000000);
  EVE_cmd_text(200, 100, 31, 0, "T41-3 TRANSCEIVER");
  EVE_cmd_text(100, 150, 31, 0, "T41 EXPERIMENTAL PLATFORM");
  EVE_cmd_text(250, 250, 30, 0, "by Greg Raven KF5N");
  EVE_cmd_text(230, 300, 30, 0, "based on the T41-EP by");
  EVE_cmd_text(110, 350, 30, 0, "Al Peters AC8GY and Jack Purdum W8TEE");
  EVE_color_rgb(0xFF0000);
  // Warn the user that the SD card is not detected.
  if (ConfigData.sdCardPresent == 0)
    EVE_cmd_text(150, 50, 31, 0, "SD CARD NOT DETECTED!");
  EVE_cmd_dl(DL_DISPLAY);
  EVE_cmd_dl(CMD_SWAP);
  while (EVE_busy())
    ;
}

// Primary receiver screen (dynamic);
void EVE_Display::drawReceiverScreen(int16_t *fftArray, uint8_t *waterfall, int16_t *audioArray)
{
  std::string mode{""};
  std::string sideband{""};
  float CWFilterPosition = 0.0;

  // Calculate tuning bar.
  ComputeBandWidthIndicatorBar();

  EVE_start_cmd_burst();
  EVE_cmd_dl_burst(CMD_DLSTART);
  EVE_cmd_dl_burst(DL_CLEAR_COLOR_RGB | 0x000000);
  EVE_cmd_dl_burst(DL_CLEAR | CLR_COL | CLR_STN | CLR_TAG);
  EVE_vertex_format_burst(0);
  EVE_point_size_burst(32);

  // Static items.
  EVE_cmd_calllist_burst(0);

  // Show top menu.
  if (evemenucontrol.top)
  {
    EVE_cmd_text_burst(200, 10, 29, 0, topMenus[mainMenuIndex].c_str());
  }

  // Show sub-menu.
  if (evemenucontrol.subMenuSelect)
  {
    EVE_cmd_text_burst(200, 10, 29, 0, menuProc.subMenuString.c_str());
  }

  // Show Morse decoder sensitivity.  Special case for decoder adjustment.
  // Must be able to see decoded Morse.
  if (morseDecodeAdjustFlag)
  {
    EVE_cmd_text_burst(50, 10, 29, 0, "Morse Decode Sensitivity:");
    EVE_cmd_number_burst(360, 10, 29, 0, ConfigData.morseDecodeSensitivity);
  }

  // Transmitter power adjust.
  if (transmitPowerAdjustFlag)
  {
    EVE_cmd_text_burst(110, 10, 29, 0, "Transmitter Power:");
    EVE_cmd_number_burst(360, 10, 29, 0, ConfigData.transmitPowerLevel);
  }

    // Receiver gain adjust.
  if (receiverGainAdjustFlag)
  {
    EVE_cmd_text_burst(110, 10, 29, 0, "Receiver DSP Gain dB:");
    EVE_cmd_number_burst(360, 10, 29, EVE_OPT_SIGNED, ConfigData.rfGain[ConfigData.currentBand]);
  }

  // Main frequency, VFO A and VFO B.  Should skip this and simply show TxRxFreq???
  if (ConfigData.activeVFO == VFO_A)
  {
    if (ConfigData.currentFreqA < bands.bands[ConfigData.currentBandA].fBandLow or ConfigData.currentFreqA > bands.bands[ConfigData.currentBandA].fBandHigh)
    {
      EVE_color_rgb_burst(0xFF0000); // Red
    }
    else
    {
      EVE_color_rgb_burst(0x0000FF); // Blue
    }
    EVE_cmd_number_burst(0, 37, 31, 0, ConfigData.currentFreqA);   // x, y, font, options, n
    EVE_color_rgb_burst(0x7d898b);                                 // VFOB gray color (not active);
    EVE_cmd_number_burst(300, 47, 30, 0, ConfigData.currentFreqB); // VFOB
  }
  if (ConfigData.activeVFO == VFO_B)
  {
    if (ConfigData.currentFreqB < bands.bands[ConfigData.currentBandB].fBandLow or ConfigData.currentFreqB > bands.bands[ConfigData.currentBandB].fBandHigh)
    {
      EVE_color_rgb_burst(0xFF0000); // Red
    }
    else
    {
      EVE_color_rgb_burst(0x0000FF); // Blue
    }
    EVE_cmd_number_burst(300, 37, 31, 0, ConfigData.currentFreqB); // VFOB
    EVE_color_rgb_burst(0x7d898b);                                 // VFOA gray color (not active);
    EVE_cmd_number_burst(0, 47, 30, 0, ConfigData.currentFreqA);   // VFOA
  }

  // Blue "tuning bar".
  EVE_color_rgb_burst(0x0000ff);
  EVE_color_a_burst(100);
  EVE_begin_burst(EVE_RECTS);
  EVE_vertex2f_burst(x1, 101);
  EVE_vertex2f_burst(x2, 249);
  EVE_end_burst();

  // Tuning alignment line.
  EVE_color_rgb_burst(0xffffff);
  EVE_color_a_burst(100);
  EVE_begin_burst(EVE_LINES);
  EVE_vertex2f_burst(tuneAlignmentLineX, 101);
  EVE_vertex2f_burst(tuneAlignmentLineX, 249);
  EVE_end_burst();

  // Draw the RF spectrum.
  EVE_color_rgb_burst(0xf0f000); // Make spectrum blue.
  EVE_begin_burst(EVE_LINE_STRIP);
  for (int j = 0; j < 512; j = j + 1)
  {
    EVE_vertex2f_burst(j, fftArray[j]);
  }
  EVE_end_burst();

  // Draw the audio spectrum.
  EVE_color_rgb_burst(0xf0f000); // Make spectrum blue.
  EVE_begin_burst(EVE_LINE_STRIP);
  for (int j = 0; j < 256; j = j + 1)
  {
    EVE_vertex2f_burst(j + 532, 245 - audioArray[j]);
  }
  EVE_end_burst();

  // Now add all of the waterfall line bitmaps.
  EVE_color_rgb_burst(0xffffff);
  EVE_color_a_burst(255);
  EVE_cmd_setbitmap_burst(500000, 4, 512, waterfallLines);
  EVE_begin_burst(EVE_BITMAPS);
  EVE_cell_burst(0);
  EVE_vertex2f_burst(1, 290);
  EVE_end_burst();

  EVE_end_cmd_burst();
  EVE_start_cmd_burst();

  // Volume
  EVE_color_rgb_burst(0x0000FF);
  if(ConfigData.audioOut == AudioState::SPEAKER or ConfigData.audioOut == AudioState::BOTH)
  EVE_cmd_number_burst(610, 278, 30, 0, ConfigData.speakerVolume);
  if(ConfigData.audioOut == AudioState::HEADPHONE)
  EVE_cmd_number_burst(610, 278, 30, 0, ConfigData.headphoneVolume);

  // AGC on or off.
  std::string agcMode{"ON"};
  if (ConfigData.AGCMode == false)
    agcMode = "OFF";
  EVE_cmd_text_burst(730, 278, 30, 0, agcMode.c_str());

  // Fine increment
  EVE_cmd_number_burst(597, 312, 27, 0, ConfigData.fineTuneStep); // x, y, font, options, n

  // Coarse increment
  EVE_cmd_number_burst(715, 312, 27, 0, ConfigData.centerTuneStep); // x, y, font, options, n

  // AutoNotch status.
  std::string ANR_notch{"Off"};
  if (ConfigData.ANR_notch == true)
    ANR_notch = "On";
  EVE_cmd_text_burst(630, 331, 27, 0, ANR_notch.c_str());

  // Noise filter selection.
  EVE_cmd_text_burst(630, 351, 27, 0, filter[ConfigData.nrOptionSelect].c_str());
  EVE_cmd_text_burst(630, 371, 27, 0, zoomOptions[ConfigData.spectrum_zoom].c_str());

  // Compressor status.
  std::string compressor{"Off"};
  if (ConfigData.compressorFlag == true)
  {
    compressor = "On";
    EVE_cmd_number_burst(660, 391, 27, 0, ConfigData.micCompRatio);
  }
  EVE_cmd_text_burst(630, 391, 27, 0, compressor.c_str());

  // Key type and keyer L/R and WPM.
  std::string keyType{""};
  if (ConfigData.keyType == 0)
    keyType = "Straight Key";
  if (ConfigData.keyType == 1)
  {
    keyType = "Paddles";
    if (ConfigData.paddleFlip == 0)
    {
      EVE_cmd_text_burst(705, 411, 27, 0, "R");
    }
    else
    {
      EVE_cmd_text_burst(705, 411, 27, 0, "L");
    }
    EVE_cmd_number_burst(725, 411, 27, 0, ConfigData.currentWPM);
  }
  EVE_cmd_text_burst(630, 411, 27, 0, keyType.c_str());

  // Morse decoder
  std::string decoder{"Off"};
  if (ConfigData.decoderFlag)
  {
    decoder = "WPM";
    EVE_cmd_number_burst(680, 431, 27, 0, cwprocess.estimatedWPM);
    EVE_cmd_text_burst(10, 450, 29, 0, cwprocess.decodeBuffer);
    // CW decode indicator rectangle.
    if (cwprocess.cwdecode)
    {
      EVE_begin_burst(EVE_RECTS);
      EVE_vertex2f(710, 433);
      EVE_vertex2f(724, 447);
      EVE_end_burst();
    }
  }
  EVE_cmd_text_burst(630, 431, 27, 0, decoder.c_str()); // Decoder "Off" or "WPM".

  // On Band Information line at top of RF spectrum.
  EVE_cmd_number_burst(90, 80, 26, 0, ConfigData.centerFreq);
  EVE_cmd_text_burst(160, 80, 26, 0, bands.bands[ConfigData.currentBandA].name);
  // Which mode?
  if (bands.bands[ConfigData.currentBand].mode == RadioMode::CW_MODE)
    mode = "CW";
  if (bands.bands[ConfigData.currentBand].mode == RadioMode::SSB_MODE)
  {
    if (ConfigData.cessb == false)
      mode = "SSB";
    else
      mode = "CESSB";
  }
  if (bands.bands[ConfigData.currentBand].mode == RadioMode::FT8_MODE)
  {
    mode = "FT8";
    // Green if enabled, red if disabled.
    if (ft8EnableFlag)
      EVE_color_rgb_burst(0x00FF00);
    else
      EVE_color_rgb_burst(0xFF0000);
  }
  if (bands.bands[ConfigData.currentBand].mode == RadioMode::AM_MODE)
    mode = "AM";
  if (bands.bands[ConfigData.currentBand].mode == RadioMode::SAM_MODE)
  {
    mode = "SAM";
    // If in SAM mode, need to show frequency indicator.
    EVE_color_rgb_burst(0xFF0000); // Red
    //   snprintf(buffer, 10, "%4.1f", demod.samOffset);  // This fails with smallest code.
    dtostrf(demod.samOffset, 4, 1, buffer);
    EVE_cmd_text_burst(325, 80, 26, 0, buffer);
  }

  // The special case of frequency calibration.  Show frequency correction
  // factor while tuning.  This must be done in the main receiver loop.
  if (calibrateFlag and menuProc.subMenuChoice == 0)
  {
    EVE_cmd_text_burst(80, 8, 29, 0, "Frequency Correction Factor:");
    EVE_cmd_number_burst(400, 8, 29, 0, CalData.freqCorrectionFactor);
  }

  // CW PA calibration.
  // Done within main loop.  The transmitter will still work, but inside its own loop.
  if (calibrateFlag and menuProc.subMenuChoice == 1)
  {
    EVE_cmd_text_burst(180, 8, 29, 0, "CW PA Cal:");
    dtostrf(CalData.CWPowerCalibrationFactor[ConfigData.currentBand], 4, 2, buffer);
    EVE_cmd_text_burst(325, 8, 29, 0, buffer);
  }

  // SSB PA calibration.
  // Done within main loop.  The transmitter will still work, but inside its own loop.
  if (calibrateFlag and menuProc.subMenuChoice == 5)
  {
    EVE_cmd_text_burst(180, 8, 29, 0, "SSB PA Cal:");
    dtostrf(CalData.SSBPowerCalibrationFactor[ConfigData.currentBand], 4, 2, buffer);
    EVE_cmd_text_burst(325, 8, 29, 0, buffer);
  }


  // The special case of dBm power calibration.  Show power correction
  // factor while tuning.  This must be done in the main receiver loop.
  if (calibrateFlag and menuProc.subMenuChoice == 13)
  {
    EVE_cmd_text_burst(80, 8, 29, 0, "dBm Correction Factor:");
    EVE_cmd_number_burst(400, 8, 29, 0, CalData.dBm_calibration);
  }

    // The special case of "Button Repeat" adjustment.
  if (calibrateFlag and menuProc.subMenuChoice == 15)
  {
    EVE_cmd_text_burst(80, 8, 29, 0, "Button Repeat Delay:");
    EVE_cmd_number_burst(330, 8, 29, 0, CalData.buttonRepeatDelay);
  }

  EVE_cmd_text_burst(240, 80, 26, 0, mode.c_str());
  EVE_color_rgb_burst(0x0000FF);
  // Which sideband?
  switch (bands.bands[ConfigData.currentBand].sideband)
  {
  case Sideband::LOWER:
    sideband = "LSB";
    break;
  case Sideband::UPPER:
    sideband = "USB";
    break;
  case Sideband::BOTH_AM:
    sideband = "DSB";
    break;
  case Sideband::BOTH_SAM:
    sideband = " DSB";
    break;
  default:
    break;
  }
  EVE_cmd_text_burst(288, 80, 26, 0, sideband.c_str());

  // Show audio state.
  switch (ConfigData.audioOut)
  {
  case AudioState::SPEAKER:
    EVE_cmd_text_burst(672, 360, 27, 0, "Speaker On");
    EVE_cmd_text_burst(667, 376, 27, 0, "Headphone Mute");
    break;
  case AudioState::MUTE_BOTH:
    EVE_cmd_text_burst(667, 360, 27, 0, "Speaker Mute");
    EVE_cmd_text_burst(667, 376, 27, 0, "Headphone Mute");
    break;
  case AudioState::HEADPHONE:
    EVE_cmd_text_burst(667, 360, 27, 0, "Speaker Mute");
    EVE_cmd_text_burst(667, 376, 27, 0, "Headphone On");
    break;
  case AudioState::BOTH:
    EVE_cmd_text_burst(667, 360, 27, 0, "Speaker On");
    EVE_cmd_text_burst(667, 376, 27, 0, "Headphone On");
    break;
  default:
    break;
  }

  // Spectrum x-axis frequency values.  First, Zoom = 1.
  int centerIdx{0};
  float disp_freq{0.0};
  float freq_calc{0.0};
  float grat(0.0);
  const uint32_t graticulePos[7]{2, 87, 174, 258, 343, 427, 515};
  const uint32_t cursorPos[7]{0, 76, 153, 233, 314, 391, 463};
  grat = static_cast<float>(SR[SampleRate].rate) / 6000.0 / static_cast<float>(1 << ConfigData.spectrum_zoom);
  freq_calc = static_cast<float>(ConfigData.centerFreq) / 1000.0; // Current frequency in kHz.
  disp_freq = freq_calc + (centerIdx * grat);
  if (ConfigData.spectrum_zoom != 0)
    centerIdx = 0;
  else
    centerIdx = -2;

  if (ConfigData.spectrum_zoom == 0)
  {
    for (int idx = -3; idx < 4; idx++)
    {
      // Don't print the center frequency, it was printed above.  Also skip the 2nd and 3rd.
      if (idx == -3 or idx >= 0)
      {
        if (idx == -3)
          disp_freq = freq_calc - 48.0; // Farthest left.
        if (idx >= 0)
          disp_freq = freq_calc + (idx * grat) + 48.0;
        // Draw the graticule lines.
        EVE_color_rgb_burst(0xff0000);
        EVE_begin_burst(EVE_LINES);
        EVE_vertex2f_burst(graticulePos[idx + 3] - 2, WATERFALL_TOP_Y - 4);
        EVE_vertex2f_burst(graticulePos[idx + 3] - 2, WATERFALL_TOP_Y - 4 + 7);
        EVE_end_burst();
      }
    }
    EVE_begin_burst(EVE_LINES);
    EVE_vertex2f_burst(128, WATERFALL_TOP_Y - 4); // Center frequency graticule for 1X zoom.
    EVE_vertex2f_burst(128, WATERFALL_TOP_Y - 4 + 7);
    EVE_end_burst();
  }

  // Print the frequencies.
  // Zoom == 1.  This is a special case, and the x-axis is weird.
  if (ConfigData.spectrum_zoom == 0)
  {
    for (int idx = -3; idx < 4; idx++)
    {
      // Don't print the center frequency, it was printed above.  Also skip the 2nd and 3rd.
      if (idx == -3 or idx >= 0)
      {
        if (idx == -3)
          disp_freq = freq_calc - 48.0; // Farthest left.
        if (idx >= 0)
          disp_freq = freq_calc + (idx * grat) + 48.0;

        //     snprintf(buffer, 10, "%4.1f", disp_freq);
        dtostrf(disp_freq, 4, 1, buffer);
        EVE_color_rgb_burst(0xff0000);
        EVE_cmd_text_burst(cursorPos[idx + 3], WATERFALL_TOP_Y + 5, 27, 0, buffer);
        EVE_begin_burst(EVE_LINES);
        EVE_vertex2f_burst(graticulePos[idx + 3] - 2, WATERFALL_TOP_Y - 4);
        EVE_vertex2f_burst(graticulePos[idx + 3] - 2, WATERFALL_TOP_Y - 4 + 7);
        EVE_vertex2f_burst(128, WATERFALL_TOP_Y - 4); // Center frequency graticule for 1X zoom.
        EVE_vertex2f_burst(128, WATERFALL_TOP_Y - 4 + 7);
        EVE_end_burst();
      }
    }
    EVE_color_rgb_burst(0x00ff00);
    snprintf(buffer, 10, "%4.1f", freq_calc);
    EVE_cmd_text_burst(centerLine - 148, WATERFALL_TOP_Y + 5, 27, 0, buffer); // Center frequency
                                                                              // EVE_cmd_number(centerLine - 148,  WATERFALL_TOP_Y + 5, 27, 0, buffer);
  }

  // Zooms not equal to 1:
  if (ConfigData.spectrum_zoom != 0)
  {
    for (int idx = -3; idx < 4; idx++)
    {
      disp_freq = freq_calc + (idx * grat);
      dtostrf(disp_freq, 4, 1, buffer);
      EVE_color_rgb_burst(0x00ff00);
      // Don't print the center frequency, it was printed above.
      if (idx != centerIdx)
        EVE_cmd_text_burst(cursorPos[idx + 3], WATERFALL_TOP_Y + 5, 27, 0, buffer);
      EVE_begin_burst(EVE_LINES);
      EVE_color_rgb_burst(0xff0000);
      EVE_vertex2f_burst(graticulePos[idx + 3] - 2, WATERFALL_TOP_Y - 4);
      EVE_vertex2f_burst(graticulePos[idx + 3] - 2, WATERFALL_TOP_Y - 4 + 7);
      EVE_end_burst();
    }
    // }
    EVE_color_rgb_burst(0x00ff00);
    dtostrf(freq_calc, 4, 1, buffer);
    EVE_cmd_text_burst(centerLine - 20, WATERFALL_TOP_Y + 5, 27, 0, buffer); // Center frequency
  }

  // Low and high pass filter delimiters in audio spectrum box.
  //  Calculate position and draw filter delimiters in audio spectrum box.
  int filterLoPositionMarker{0};
  int filterHiPositionMarker{0};
  if (bands.bands[ConfigData.currentBand].sideband == Sideband::LOWER or bands.bands[ConfigData.currentBand].sideband == Sideband::UPPER)
  {
    filterLoPositionMarker = map(bands.bands[ConfigData.currentBand].FLoCut, 0, 6000, 0, 256);
    filterHiPositionMarker = map(bands.bands[ConfigData.currentBand].FHiCut, 0, 6000, 0, 256);
  }
  else if (bands.bands[ConfigData.currentBand].sideband == Sideband::BOTH_AM or bands.bands[ConfigData.currentBand].sideband == Sideband::BOTH_SAM)
  {
    filterHiPositionMarker = map(bands.bands[ConfigData.currentBand].FAMCut, 0, 6000, 0, 256);
  }

  // Draw Filter indicator lines on audio plot.
  // The encoder should adjust the low side of the filter if the CW filter is on.  This creates a tunable bandpass.  Sort of.
  if (bands.bands[ConfigData.currentBand].mode == RadioMode::CW_MODE and ConfigData.CWFilterIndex != 5)
    switchFilterSideband = true;

  if (bands.bands[ConfigData.currentBand].sideband == Sideband::LOWER or bands.bands[ConfigData.currentBand].sideband == Sideband::UPPER)
  {
    if (not switchFilterSideband)
    {
      EVE_begin_burst(EVE_LINES);
      EVE_color_rgb_burst(0x7d898b);
      EVE_vertex2f_burst(BAND_INDICATOR_X - 6 + abs(filterLoPositionMarker), SPECTRUM_BOTTOM - 2);
      EVE_vertex2f_burst(BAND_INDICATOR_X - 6 + abs(filterLoPositionMarker), SPECTRUM_BOTTOM - 117);
      EVE_color_rgb_burst(0xff0000);  // Red
      EVE_vertex2f_burst(BAND_INDICATOR_X - 7 + abs(filterHiPositionMarker), SPECTRUM_BOTTOM - 2);
      EVE_vertex2f_burst(BAND_INDICATOR_X - 7 + abs(filterHiPositionMarker), SPECTRUM_BOTTOM - 117);
      EVE_end_burst();
    }
    else
    {
      EVE_begin_burst(EVE_LINES);
      EVE_color_rgb_burst(0xff0000);
      EVE_vertex2f_burst(BAND_INDICATOR_X - 6 + abs(filterLoPositionMarker), SPECTRUM_BOTTOM - 2);
      EVE_vertex2f_burst(BAND_INDICATOR_X - 6 + abs(filterLoPositionMarker), SPECTRUM_BOTTOM - 117);
      EVE_color_rgb_burst(0x7d898b);
      EVE_vertex2f_burst(BAND_INDICATOR_X - 7 + abs(filterHiPositionMarker), SPECTRUM_BOTTOM - 2);
      EVE_vertex2f_burst(BAND_INDICATOR_X - 7 + abs(filterHiPositionMarker), SPECTRUM_BOTTOM - 117);
      EVE_end_burst();
    }
  }

  // In AM modes draw high delimiter only and always make it red (active);
  if (bands.bands[ConfigData.currentBand].sideband == Sideband::BOTH_AM or bands.bands[ConfigData.currentBand].sideband == Sideband::BOTH_SAM)
  {
    EVE_begin_burst(EVE_LINES);
    EVE_color_rgb_burst(0xff0000);
    EVE_vertex2f_burst(BAND_INDICATOR_X - 7 + abs(filterHiPositionMarker), SPECTRUM_BOTTOM - 2);
    EVE_vertex2f_burst(BAND_INDICATOR_X - 7 + abs(filterHiPositionMarker), SPECTRUM_BOTTOM - 117);
    EVE_end_burst();
  }

  // Draw CW offset tone delimiters.
  // Select the x-axis position for the CW filter box and delimiter line (on the right side of the box).
  switch (ConfigData.CWFilterIndex)
  {
  case 0:
    CWFilterPosition = 35.7; // 0.84 * 42.5;
    break;
  case 1:
    CWFilterPosition = 42.5;
    break;
  case 2:
    CWFilterPosition = 55.25; // 1.3 * 42.5;
    break;
  case 3:
    CWFilterPosition = 76.5; // 1.8 * 42.5;
    break;
  case 4:
    CWFilterPosition = 85.0; // 2.0 * 42.5;
    break;
  case 5:
    CWFilterPosition = 0.0;
    break;
  }

  // Draw CW filter box if required.  Don't do this if the CW filter is off.
  if (bands.bands[ConfigData.currentBand].mode == RadioMode::CW_MODE and ConfigData.CWFilterIndex != 5)
  {
    EVE_begin_burst(EVE_RECTS);
    EVE_color_rgb_burst(0xff0000);
    EVE_color_a_burst(100);
    EVE_vertex2f_burst(BAND_INDICATOR_X - 6 + abs(filterLoPositionMarker), AUDIO_SPECTRUM_TOP);
    EVE_vertex2f_burst(BAND_INDICATOR_X - 6 + abs(filterLoPositionMarker) + CWFilterPosition - abs(filterLoPositionMarker), AUDIO_SPECTRUM_TOP + 117);
    EVE_end_burst();
    EVE_begin_burst(EVE_LINES);
    EVE_color_rgb_burst(0x7d898b);
    EVE_color_a_burst(255);
    EVE_vertex2f_burst(BAND_INDICATOR_X - 8 + CWFilterPosition, AUDIO_SPECTRUM_TOP);
    EVE_vertex2f_burst(BAND_INDICATOR_X - 8 + CWFilterPosition, AUDIO_SPECTRUM_TOP + 117);
    EVE_end_burst();

    // Draw CW filter bandwidth setting.
    EVE_color_rgb_burst(0xffffff);
    EVE_color_a_burst(255);
    EVE_cmd_text_burst(380, 102, 26, 0, "CW Filter =");
    EVE_cmd_text_burst(451, 102, 26, 0, cwprocess.CWFilter[ConfigData.CWFilterIndex].c_str());
  }
  if (bands.bands[ConfigData.currentBand].mode == RadioMode::CW_MODE and ConfigData.CWFilterIndex == 5)
  {
    EVE_color_rgb_burst(0xffffff);
    EVE_color_a_burst(255);
    EVE_cmd_text_burst(380, 102, 26, 0, "CW Filter = Off");
  }

  // Draw decoder delimiters if in CW mode.  The delimiters are also used for frequency spotting (for same TX frequency as received signal).
  if (bands.bands[ConfigData.currentBand].mode == RadioMode::CW_MODE)
  {
    // Draw delimiter bars for CW offset frequency.  This depends on the user selected offset.
    if (ConfigData.CWOffset == 0)
    { // 562.5
      EVE_begin_burst(EVE_LINES);
      EVE_color_rgb_burst(0x7d898b);
      EVE_vertex2f_burst(BAND_INDICATOR_X + 12, AUDIO_SPECTRUM_BOTTOM - 117);
      EVE_vertex2f_burst(BAND_INDICATOR_X + 12, AUDIO_SPECTRUM_BOTTOM - 1);
      EVE_vertex2f_burst(BAND_INDICATOR_X + 18, AUDIO_SPECTRUM_BOTTOM - 117);
      EVE_vertex2f_burst(BAND_INDICATOR_X + 18, AUDIO_SPECTRUM_BOTTOM - 1);
      EVE_end_burst();
    }
    if (ConfigData.CWOffset == 1)
    { // 656.5
      EVE_begin_burst(EVE_LINES);
      EVE_color_rgb_burst(0x7d898b);
      EVE_vertex2f_burst(BAND_INDICATOR_X + 16, AUDIO_SPECTRUM_BOTTOM - 117);
      EVE_vertex2f_burst(BAND_INDICATOR_X + 16, AUDIO_SPECTRUM_BOTTOM - 1);
      EVE_vertex2f_burst(BAND_INDICATOR_X + 22, AUDIO_SPECTRUM_BOTTOM - 117);
      EVE_vertex2f_burst(BAND_INDICATOR_X + 22, AUDIO_SPECTRUM_BOTTOM - 1);
      EVE_end_burst();
    }
    if (ConfigData.CWOffset == 2)
    {
      EVE_begin_burst(EVE_LINES);
      EVE_color_rgb_burst(0x7d898b);
      EVE_vertex2f_burst(BAND_INDICATOR_X + 20, AUDIO_SPECTRUM_BOTTOM - 117);
      EVE_vertex2f_burst(BAND_INDICATOR_X + 20, AUDIO_SPECTRUM_BOTTOM - 1);
      EVE_vertex2f_burst(BAND_INDICATOR_X + 26, AUDIO_SPECTRUM_BOTTOM - 117);
      EVE_vertex2f_burst(BAND_INDICATOR_X + 26, AUDIO_SPECTRUM_BOTTOM - 1);
      EVE_end_burst();
    }
    if (ConfigData.CWOffset == 3)
    {
      EVE_begin_burst(EVE_LINES);
      EVE_color_rgb_burst(0x7d898b);
      EVE_vertex2f_burst(BAND_INDICATOR_X + 24, AUDIO_SPECTRUM_BOTTOM - 117);
      EVE_vertex2f_burst(BAND_INDICATOR_X + 24, AUDIO_SPECTRUM_BOTTOM - 1);
      EVE_vertex2f_burst(BAND_INDICATOR_X + 30, AUDIO_SPECTRUM_BOTTOM - 117);
      EVE_vertex2f_burst(BAND_INDICATOR_X + 30, AUDIO_SPECTRUM_BOTTOM - 1);
      EVE_end_burst();
    }
  }

  // S-Meter bar.  This is the moving red bar.
  EVE_begin_burst(EVE_RECTS);
  EVE_color_rgb_burst(0xff0000);
  EVE_vertex2f_burst(SMETER_X + 1, SMETER_Y + 2);
  EVE_vertex2f_burst(SMETER_X + 1 + display.smeterPad, SMETER_Y + 2 + SMETER_BAR_HEIGHT - 2);
  EVE_end_burst();
  // Draw dBm level.
  dtostrf(display.dbm, 4, 1, buffer);
  EVE_cmd_text_burst(SMETER_X + 185, SMETER_Y + 3, 26, 0, buffer);

  // Show RF gain.
  EVE_color_rgb_burst(0xffffff);
  EVE_cmd_number_burst(SPECTRUM_LEFT_X + 70, SPECTRUM_TOP_Y + 2, 26, EVE_OPT_SIGNED, ConfigData.rfGain[ConfigData.currentBand]);

  // High pass and low pass filter settings.
  // Don't show HPF filter setting in AM modes, because it doesn't exist yet.
  if (bands.bands[ConfigData.currentBand].mode != RadioMode::SAM_MODE and bands.bands[ConfigData.currentBand].mode != RadioMode::AM_MODE)
  {
    EVE_cmd_text_burst(163, 102, 26, 0, "HPF =");
    EVE_cmd_text_burst(225, 102, 26, 0, "kHz");
    dtostrf(static_cast<float>(bands.bands[ConfigData.currentBand].FLoCut) / 1000.0f, 4, 1, buffer);
    EVE_cmd_text_burst(200, 102, 26, 0, buffer);
  }

  if (bands.bands[ConfigData.currentBand].mode == RadioMode::SAM_MODE or bands.bands[ConfigData.currentBand].mode == RadioMode::AM_MODE)
  {
    EVE_cmd_text_burst(265, 102, 26, 0, "LPF =");
    EVE_cmd_text_burst(325, 102, 26, 0, "kHz");
    dtostrf(static_cast<float>(bands.bands[ConfigData.currentBand].FAMCut) / 1000.0f, 4, 1, buffer);
    EVE_cmd_text_burst(300, 102, 26, 0, buffer);
  }
  else
  {
    EVE_cmd_text_burst(265, 102, 26, 0, "LPF =");
    EVE_cmd_text_burst(325, 102, 26, 0, "kHz");
    dtostrf(static_cast<float>(bands.bands[ConfigData.currentBand].FHiCut) / 1000.0f, 4, 1, buffer);
    EVE_cmd_text_burst(300, 102, 26, 0, buffer);
  }

  // Draw TX power setting above on right side of spectrum window.
  EVE_color_rgb_burst(0xff0000);
  EVE_cmd_number_burst(430, 80, 26, 0, ConfigData.transmitPowerLevel);

  // Receive and transmit equalizer indicators.

  if (ConfigData.receiveEQFlag)
  {
    EVE_color_rgb_burst(0x00ff00);
    EVE_cmd_text_burst(660, 451, 27, 0, "On");
  }
  else
  {
    EVE_color_rgb_burst(0xff0000);
    EVE_cmd_text_burst(655, 451, 27, 0, "Off");
  }
  if (ConfigData.xmitEQFlag)
  {
    EVE_color_rgb_burst(0x00ff00);
    EVE_cmd_text_burst(715, 451, 27, 0, "On");
  }
  else
  {
    EVE_color_rgb_burst(0xff0000);
    EVE_cmd_text_burst(715, 451, 27, 0, "Off");
  }

  // Show the time.
  EVE_color_rgb_burst(0x5d82f3);
  EVE_cmd_text_burst(510, 38, 30, 0, display.timeBuffer);

// Show temperature and load.
#ifdef TEMP_AND_LOAD
  EVE_color_rgb_burst(0xFF0000);
  dtostrf(display.CPU_temperature, 4, 1, buffer);
  EVE_cmd_text_burst(5, 5, 27, 0, "CPU Temperature =");
  EVE_cmd_text_burst(160, 5, 27, 0, buffer);

  dtostrf(display.processor_load, 4, 1, buffer);
  EVE_cmd_text_burst(210, 5, 27, 0, "CPU Load =");
  EVE_cmd_text_burst(301, 5, 27, 0, buffer);
#endif

  EVE_display_burst();
  EVE_cmd_swap_burst();
  EVE_end_cmd_burst();
  while (EVE_busy())
    ;
} // End receiver dynamic.

// Static items in receiver display.
FLASHMEM void EVE_Display::receiverStatic_cmd_list()
{
  EVE_cmd_newlist(0);
  EVE_vertex_format(0);
  EVE_line_width(16);

  // T41-EP "brand"
  EVE_color_rgb(0xffffff);
  EVE_cmd_text(510, 5, 30, 0, "T41-EP SDT");

  // Current version.
  EVE_color_rgb(0x00ff00);
  EVE_cmd_text(705, 5, 27, 0, "T41EVE.01");

  // Spectrum center line.
  EVE_begin(EVE_LINES);
  EVE_line_width(10);
  EVE_color_a(110);        // Make center line transparent.
  EVE_color_rgb(0xffffff); // White
  EVE_vertex2f(256, 100);
  EVE_vertex2f(256, 249);
  EVE_end();
  EVE_color_a(255);

  // RF spectrum display container.
  EVE_color_rgb(0xff0000); // Red
  EVE_line_width(16);
  EVE_begin(EVE_LINE_STRIP);
  EVE_vertex2f(0, 100);
  EVE_vertex2f(513, 100);
  EVE_vertex2f(513, 250);
  EVE_vertex2f(0, 250);
  EVE_vertex2f(0, 100);
  EVE_end();

  // Audio spectrum display container.
  EVE_color_rgb(0xff0000); // Red
  EVE_begin(EVE_LINE_STRIP);
  EVE_vertex2f(531, 129);
  EVE_vertex2f(788, 129);
  EVE_vertex2f(788, 247);
  EVE_vertex2f(531, 247);
  EVE_vertex2f(531, 129);
  EVE_end();

  // Information window container.
  EVE_color_rgb(0xff0000); // Red
  EVE_begin(EVE_LINE_STRIP);
  EVE_vertex2f(530, 280);
  EVE_vertex2f(799, 280);
  EVE_vertex2f(799, 475);
  EVE_vertex2f(530, 475);
  EVE_vertex2f(530, 280);
  EVE_end();

  uint32_t line1y{311};

  // Information window static text.
  EVE_color_rgb(0xffffff);
  EVE_cmd_text(533, 278, 30, 0, "VOL:");
  EVE_cmd_text(660, 278, 30, 0, "AGC");
  EVE_cmd_text(533, line1y, 27, 0, "Fine Inc:");
  EVE_cmd_text(630, line1y, 27, 0, "Coarse Inc:");
  EVE_cmd_text(533, line1y + 20, 27, 0, "AutoNotch:");
  EVE_cmd_text(573, line1y + 40, 27, 0, "Noise:");
  EVE_cmd_text(570, line1y + 60, 27, 0, "Zoom:");
  EVE_cmd_text(538, line1y + 80, 27, 0, "Compress:");
  EVE_cmd_text(572, line1y + 100, 27, 0, "Keyer:");
  EVE_cmd_text(552, line1y + 120, 27, 0, "Decoder:");
  EVE_cmd_text(547, line1y + 140, 27, 0, "Equalizer:");
  EVE_cmd_text(630, line1y + 140, 27, 0, "Rx");
  EVE_cmd_text(687, line1y + 140, 27, 0, "Tx");

  // Band and mode information line.
  EVE_color_rgb(0xffffff); // White
  EVE_cmd_text(0, 80, 26, 0, "Center Freq");
  EVE_cmd_text(200, 80, 26, 0, "Mode");

  // Draw audio spectrum frequency axis.
  std::string audioMarkers[6] = {"0k", "1k", "2k", "3k", "4k", "5k"};

  for (int k = 0; k < 6; k++)
  {
    EVE_color_rgb(0x00ff00); // Green
    EVE_cmd_text(BAND_INDICATOR_X - 14 + k * 43.8, SPECTRUM_BOTTOM + 10, 27, 0, audioMarkers[k].c_str());
    EVE_color_rgb(0xff0000); // Red
    EVE_begin(EVE_LINES);    // Tick marks.
    EVE_vertex2f(BAND_INDICATOR_X - 9 + k * 43.8, AUDIO_SPECTRUM_BOTTOM);
    EVE_vertex2f(BAND_INDICATOR_X - 9 + k * 43.8, AUDIO_SPECTRUM_BOTTOM + 7);
    EVE_end();
  }

  // Signal strength indicator.
  int i;
  // DB2OO, 30-AUG-23: the white line must only go till S9
  // S-Meter container lines.
  EVE_color_rgb(0xffffff);
  EVE_begin(EVE_LINES);
  EVE_vertex2f(SMETER_X, SMETER_Y - 1);
  EVE_vertex2f(SMETER_X + 9 * pixels_per_s, SMETER_Y - 1);

  EVE_vertex2f(SMETER_X, SMETER_Y + SMETER_BAR_HEIGHT + 2);
  EVE_vertex2f(SMETER_X + 9 * pixels_per_s, SMETER_Y + SMETER_BAR_HEIGHT + 2);
  EVE_end();

  for (i = 0; i < 10; i++) // Draw tick marks for S-values.
  {
#ifdef TCVSDR_SMETER
    // DB2OO, 30-AUG-23: draw wider tick marks in the style of the Teensy Convolution SDR
    EVE_color_rgb(0xffffff);
    EVE_begin(EVE_LINES);
    EVE_vertex2f(SMETER_X + i * pixels_per_s, SMETER_Y - 7 - (i % 2) * 2);
    EVE_vertex2f(SMETER_X + i * pixels_per_s, SMETER_Y - 7 - (i % 2) * 2 + 6 + (i % 2) * 2);
    EVE_end();
#else
//    tft.drawFastVLine(SMETER_X + i * 12.2, SMETER_Y - 6, 7, RA8875_WHITE);
#endif
  }

  // DB2OO, 30-AUG-23: the green line must start at S9
  // Green part of S-Meter rectangle.
  EVE_color_rgb(0x00ff00); // Green
  EVE_begin(EVE_LINES);
  EVE_vertex2f(SMETER_X + 9 * pixels_per_s, SMETER_Y - 1);
  EVE_vertex2f(SMETER_X + 9 * pixels_per_s + SMETER_BAR_LENGTH + 2 - 9 * pixels_per_s, SMETER_Y - 1);
  EVE_vertex2f(SMETER_X + 9 * pixels_per_s, SMETER_Y + SMETER_BAR_HEIGHT + 2);
  EVE_vertex2f(SMETER_X + 9 * pixels_per_s + SMETER_BAR_LENGTH + 2 - 9 * pixels_per_s, SMETER_Y + SMETER_BAR_HEIGHT + 2);
  EVE_end();

  for (i = 1; i <= 3; i++) // Draw tick marks for s9+ values in 10dB steps.
  {
#ifdef TCVSDR_SMETER
    EVE_color_rgb(0x00ff00);
    EVE_begin(EVE_LINES);
    EVE_vertex2f(SMETER_X + 9 * pixels_per_s + i * pixels_per_s * 10.0 / 6.0, SMETER_Y - 8 + (i % 2) * 2);
    EVE_vertex2f(SMETER_X + 9 * pixels_per_s + i * pixels_per_s * 10.0 / 6.0, SMETER_Y - 8 + (i % 2) * 2 + 8 - (i % 2) * 2);
    EVE_end();
#else
//    tft.drawFastVLine(SMETER_X + 9 * pixels_per_s + i * pixels_per_s * 10.0 / 6.0, SMETER_Y - 6, 7, RA8875_GREEN);
#endif
  }
  // Draw ends of the S-Meter rectangle.
  EVE_color_rgb(0xffffff);
  EVE_begin(EVE_LINES);
  EVE_vertex2f(SMETER_X, SMETER_Y - 1);
  EVE_vertex2f(SMETER_X, SMETER_Y - 1 + SMETER_BAR_HEIGHT + 3);
  EVE_color_rgb(0xffffff);
  EVE_vertex2f(SMETER_X + SMETER_BAR_LENGTH + 2, SMETER_Y - 1);
  EVE_vertex2f(SMETER_X + SMETER_BAR_LENGTH + 2, SMETER_Y - 1 + SMETER_BAR_HEIGHT + 3);
  EVE_end();

  // Draw the S-Meter scale.
  EVE_cmd_text(SMETER_X - 8, SMETER_Y - 30, 27, 0, "S");
  EVE_cmd_text(SMETER_X + 8, SMETER_Y - 30, 27, 0, "1");
  EVE_cmd_text(SMETER_X + 32, SMETER_Y - 30, 27, 0, "3");
  EVE_cmd_text(SMETER_X + 56, SMETER_Y - 30, 27, 0, "5");
  EVE_cmd_text(SMETER_X + 80, SMETER_Y - 30, 27, 0, "7");
  EVE_cmd_text(SMETER_X + 104, SMETER_Y - 30, 27, 0, "9");
  EVE_cmd_text(SMETER_X + 133, SMETER_Y - 30, 27, 0, "+20dB");

  //  Draw dBm at end of S-Meter.
  EVE_color_rgb(0xffffff);
  EVE_cmd_text(SMETER_X + 230, SMETER_Y + 3, 26, 0, "dBm");

  // Show transmit/receive status.
  EVE_begin(EVE_RECTS);
  EVE_color_rgb(0x00ff00);
  EVE_color_a(100); // Set transparency.
  EVE_vertex2f(X_R_STATUS_X, X_R_STATUS_Y);
  EVE_vertex2f(X_R_STATUS_X + 55, X_R_STATUS_Y + 25);
  EVE_end();
  EVE_color_a(255);
  EVE_color_rgb(0xffffff);
  EVE_cmd_text(X_R_STATUS_X + 8, X_R_STATUS_Y + 0, 28, 0, "REC");

  // Draw "RF GAIN" in spectrum window.
  EVE_cmd_text(SPECTRUM_LEFT_X + 10, SPECTRUM_TOP_Y + 2, 26, 0, "RF GAIN");

  // Draw "Watts" above on right side of spectrum window.
  EVE_color_rgb(0xff0000);
  EVE_cmd_text(450, 80, 26, 0, "Watts");

  EVE_cmd_endlist();
} // End of receiver static.

// This needs to generate 2 x variables.  The left x coordinate of the tuning bar, and the right x coordinate.
void EVE_Display::ComputeBandWidthIndicatorBar() // AFP 10-30-22
{
  int32_t Zoom1Offset = 0.0;
  int32_t cwOffsetPixels = 0;
  int32_t cwOffsets[4]{563, 657, 750, 844}; // Rounded to nearest Hz.
  float hz_per_pixel = 0.0;
  int32_t hpf_offset{0};
  int32_t cwFilterBW[5]{800, 1000, 1300, 1800, 2000}; // CW filter bandwidths.

  switch (ConfigData.spectrum_zoom)
  {
  case 0: // 1X
    hz_per_pixel = 375.0;
    Zoom1Offset = -128; // The "center" is halfway to the left of the spectrum window center.  // KF5N April 23, 2024
    break;

  case 1: // 2X
    hz_per_pixel = 187.5;
    Zoom1Offset = 0;
    break;

  case 2: // 4X
    hz_per_pixel = 93.75;
    Zoom1Offset = 0;
    break;

  case 3: //  8X
    hz_per_pixel = 46.875;
    Zoom1Offset = 0;
    break;

  case 4: // 16X
    hz_per_pixel = 23.4375;
    Zoom1Offset = 0;
    break;
  }

  // Calculate the offset due to the high-pass filter.
  hpf_offset = static_cast<int32_t>(static_cast<float>(bands.bands[ConfigData.currentBand].FLoCut) / hz_per_pixel);

  // newCursorPosition must take into account the high-pass filter setting.
  if (bands.bands[ConfigData.currentBand].sideband == Sideband::UPPER)
    newCursorPosition = static_cast<int>(NCOFreq / hz_per_pixel) + Zoom1Offset + hpf_offset; // More accurate tuning bar position.  KF5N May 17, 2024
  if (bands.bands[ConfigData.currentBand].sideband == Sideband::LOWER or bands.bands[ConfigData.currentBand].sideband == Sideband::BOTH_AM or bands.bands[ConfigData.currentBand].sideband == Sideband::BOTH_SAM)
    newCursorPosition = static_cast<int>(NCOFreq / hz_per_pixel) + Zoom1Offset;

  if (bands.bands[ConfigData.currentBand].sideband == Sideband::LOWER or bands.bands[ConfigData.currentBand].sideband == Sideband::UPPER)
    filterWidth = static_cast<int32_t>((bands.bands[ConfigData.currentBand].FHiCut - bands.bands[ConfigData.currentBand].FLoCut) / hz_per_pixel); // AFP 10-30-22
  else if (bands.bands[ConfigData.currentBand].sideband == Sideband::BOTH_AM or bands.bands[ConfigData.currentBand].sideband == Sideband::BOTH_SAM)
    filterWidth = static_cast<int32_t>((static_cast<float32_t>(bands.bands[ConfigData.currentBand].FAMCut) * 2.0) / hz_per_pixel); // AFP 10-30-22

  // Draw the bar, except for CW.
  if (radioState != RadioState::CW_RECEIVE_STATE)
  {
    switch (bands.bands[ConfigData.currentBand].sideband)
    {
    case Sideband::LOWER:
      x1 = centerLine - oldFilterWidth + oldCursorPosition;
      x2 = x1 + oldFilterWidth + 1;

      break;

    case Sideband::UPPER:
      x1 = centerLine + newCursorPosition;
      x2 = x1 + filterWidth;

      break;

    case Sideband::BOTH_AM:
    case Sideband::BOTH_SAM:
      x1 = centerLine - filterWidth / 2 + newCursorPosition;
      x2 = x1 + filterWidth;
      break;

    default:
      break;
    }
  }

  // Draw the offset bar for CW.
  if (radioState == RadioState::CW_RECEIVE_STATE)
  {
    // If a CW filter is on, filterWidth needs to take this into account.
    if (ConfigData.CWFilterIndex != 5)
    {
      filterWidth = static_cast<int>((cwFilterBW[ConfigData.CWFilterIndex] - bands.bands[ConfigData.currentBand].FLoCut) / hz_per_pixel);
    }

    // Calculate the number of bins to offset based on CW offset, which can be 562.5, 656.5, 750.0 or 843.75 Hz.
    cwOffsetPixels = cwOffsets[ConfigData.CWOffset] / hz_per_pixel;
    switch (bands.bands[ConfigData.currentBand].sideband)
    {
    case Sideband::LOWER:
      x1 = centerLine + newCursorPosition + cwOffsetPixels - (filterWidth + hpf_offset);
      x2 = x1 + filterWidth;
      break;

    case Sideband::UPPER:
      x1 = centerLine + newCursorPosition - cwOffsetPixels;
      x2 = x1 + filterWidth;
      break;

    default:
      break;
    }
  }

  // Draw tuning alignment line in blue bandwidth bar.  Need to ignore the high-pass filter offset.
  if (bands.bands[ConfigData.currentBand].sideband == Sideband::LOWER)
  {
    tuneAlignmentLineX = centerLine + newCursorPosition;
  }
  if (bands.bands[ConfigData.currentBand].sideband == Sideband::UPPER)
  {
    tuneAlignmentLineX = centerLine + newCursorPosition - hpf_offset;
  }
  if (bands.bands[ConfigData.currentBand].sideband == Sideband::BOTH_AM or bands.bands[ConfigData.currentBand].sideband == Sideband::BOTH_SAM)
  {
    tuneAlignmentLineX = centerLine + newCursorPosition;
  }

  oldFilterWidth = filterWidth;
  oldCursorPosition = newCursorPosition;
  old_hpf_offset = hpf_offset;
}

// Static items in Direct Frequency Entry screen.
FLASHMEM void EVE_Display::directEntryStatic_cmd_list()
{
  EVE_cmd_newlist(5000);
  EVE_vertex_format(0);
  EVE_line_width(16);

  // Entered frequency indicator.
  EVE_cmd_text(20, 5, 30, 0, "Enter Frequency kHz or MHz:");

  // Rectangle enclosing guide and button graphics.
  EVE_color_rgb(0xff0000);
  EVE_begin(EVE_LINE_STRIP);
  EVE_vertex2f(50, 100);
  EVE_vertex2f(750, 100);
  EVE_vertex2f(750, 470);
  EVE_vertex2f(50, 470);
  EVE_vertex2f(50, 100);
  EVE_end();

  // Keypad background.  Needs to be in the back.
  EVE_color_rgb(0x7d898b); // Oslo gray
  EVE_begin(EVE_RECTS);
  EVE_vertex2f(450, 110);
  EVE_vertex2f(720, 460);
  EVE_end();

  // Button circles
  EVE_vertex_format(0);
  EVE_point_size(20 * 16);
  EVE_begin(EVE_POINTS);
  // Top row
  EVE_color_rgb(0xe5ea22);
  EVE_vertex2f(500, 145);
  EVE_color_rgb(0xff0000);
  EVE_vertex2f(586, 145);
  EVE_color_rgb(0xff0000);
  EVE_vertex2f(670, 145);
  // The next 3 rows are all the same color, so a loop will work.
  EVE_color_rgb(0x0000ff);
  uint32_t y_spacing = 55;
  for (int x = 0; x < 3; x = x + 1)
  {
    for (int y = 0; y < 3; y = y + 1)
    {
      EVE_color_rgb(0x0000ff);
      EVE_vertex2f(500 + x * 85, 200 + y * y_spacing);
    }
  }
  // Last 2 rows.
  EVE_color_rgb(0x0000ff);
  EVE_vertex2f(500 + 0 * 85, 200 + 3 * y_spacing);
  EVE_color_rgb(0x2227ea);
  EVE_vertex2f(500 + 1 * 85, 200 + 3 * y_spacing);
  EVE_color_rgb(0x2227ea);
  EVE_vertex2f(500 + 2 * 85, 200 + 3 * y_spacing);
  EVE_color_rgb(0xe5ea22);
  EVE_vertex2f(500 + 0 * 85, 200 + 4 * y_spacing);
  EVE_color_rgb(0xe5ea22);
  EVE_vertex2f(500 + 1 * 85, 200 + 4 * y_spacing);
  EVE_color_rgb(0x2227ea);
  EVE_vertex2f(500 + 2 * 85, 200 + 4 * y_spacing);
  EVE_end();

  // Characters over the top of the points.  Same coordinates.
  // Row 1
  EVE_color_rgb(0x000000);
  EVE_cmd_text(490, 118, 31, 0, "<");
  EVE_color_rgb(0xFFFFFF);
  EVE_cmd_text(661, 128, 30, 0, "X");
  // Rows 2-4
  EVE_cmd_number(492, 182, 30, 0, 7);
  EVE_cmd_number(577, 182, 30, 0, 8);
  EVE_cmd_number(662, 182, 30, 0, 9);
  EVE_cmd_number(492, 237, 30, 0, 4);
  EVE_cmd_number(577, 237, 30, 0, 5);
  EVE_cmd_number(662, 237, 30, 0, 6);
  EVE_cmd_number(492, 292, 30, 0, 1);
  EVE_cmd_number(577, 292, 30, 0, 2);
  EVE_cmd_number(662, 292, 30, 0, 3);
  // Row 5
  EVE_cmd_number(491, 347, 30, 0, 0);
  EVE_cmd_text(575, 347, 30, 0, "D");
  // Row 6
  EVE_cmd_text(660, 402, 30, 0, "S");

  uint32_t line1y{150};
  uint32_t linex{70};

  // Entry guide static text.
  EVE_color_rgb(0xffffff);
  EVE_cmd_text(linex, line1y, 28, 0, "Direct Frequency Entry");
  EVE_cmd_text(linex, line1y + 50, 28, 0, "< Apply entered frequency");
  EVE_cmd_text(linex, line1y + 80, 28, 0, "X Exit without changing frequency");
  EVE_cmd_text(linex, line1y + 110, 28, 0, "D Delete last digit");
  EVE_cmd_text(linex, line1y + 140, 28, 0, "S Save direct to last frequency");

  EVE_cmd_endlist();
} // End direct entry static list.

// Direct Entry screen (dynamic).
void EVE_Display::drawDirectEntryScreen()
{
  EVE_start_cmd_burst();
  EVE_cmd_dl(CMD_DLSTART);
  EVE_cmd_dl(DL_CLEAR_COLOR_RGB | 0x000000);
  EVE_cmd_dl(DL_CLEAR | CLR_COL | CLR_STN | CLR_TAG);
  EVE_vertex_format_burst(0);
  EVE_point_size_burst(32);

  // Static items.
  EVE_cmd_calllist_burst(5000);
  // Show the entered frequency.
  EVE_cmd_text(500, 5, 30, 0, button.freqString.c_str());

  // Write status of save_last_frequency.
  if (button.save_last_frequency)
  {
    EVE_color_rgb(0x00FF00);
    EVE_cmd_text(380, 150 + 140, 28, 0, "On");
  }
  else
  {
    EVE_color_rgb(0xff0000);
    EVE_cmd_text(380, 150 + 140, 28, 0, "Off");
  }

  EVE_display_burst();
  EVE_cmd_swap_burst();
  EVE_end_cmd_burst();
  while (EVE_busy())
    ;
}

// Button entry screen (dynamic).
void EVE_Display::drawButtonEntryScreen()
{
  EVE_start_cmd_burst();
  EVE_cmd_dl(CMD_DLSTART);
  EVE_cmd_dl(DL_CLEAR_COLOR_RGB | 0x000000);
  EVE_cmd_dl(DL_CLEAR | CLR_COL | CLR_STN | CLR_TAG);
  EVE_vertex_format_burst(0);
  EVE_point_size_burst(32);

  // Static items.
  EVE_cmd_calllist_burst(10000);
  // Show the parameter name.
  EVE_cmd_text(250, 40, 30, 0, button.buttonParameterName.c_str());
  // Show the parameter.
  EVE_cmd_text(500, 40, 30, 0, button.buttonSelection.c_str());

  EVE_display_burst();
  EVE_cmd_swap_burst();
  EVE_end_cmd_burst();
  while (EVE_busy())
    ;
}

// Direct Entry screen (dynamic).
void EVE_Display::drawEncoderEntryScreen(bool typeFloat)
{
  EVE_start_cmd_burst();
  EVE_cmd_dl(CMD_DLSTART);
  EVE_cmd_dl(DL_CLEAR_COLOR_RGB | 0x000000);
  EVE_cmd_dl(DL_CLEAR | CLR_COL | CLR_STN | CLR_TAG);
  EVE_vertex_format_burst(0);
  EVE_point_size_burst(32);

  // Static items.
  EVE_cmd_calllist_burst(15000);
  // Show the parameter name.
  EVE_cmd_text_burst(200, 40, 30, 0, button.buttonParameterName.c_str());
  // Show the parameter value.
  if (typeFloat)
  {
    dtostrf(button.encoderAdjustFloat, 4, 2, buffer);
    EVE_cmd_text_burst(460, 40, 30, 0, buffer); // Write float.
  }
  else
    EVE_cmd_number_burst(460, 40, 30, EVE_OPT_SIGNED, button.encoderAdjustInt); // Write integer.

  EVE_display_burst();
  EVE_cmd_swap_burst();
  EVE_end_cmd_burst();
  while (EVE_busy())
    ;
}

// Static items in button parameter entry.
FLASHMEM void EVE_Display::buttonEntryStatic_cmd_list()
{

  EVE_cmd_newlist(10000);
  EVE_vertex_format(0);
  EVE_line_width(16);

  // Rectangle enclosing guide and button graphics.
  EVE_color_rgb(0xff0000);
  EVE_begin(EVE_LINE_STRIP);
  EVE_vertex2f(50, 100);
  EVE_vertex2f(750, 100);
  EVE_vertex2f(750, 470);
  EVE_vertex2f(50, 470);
  EVE_vertex2f(50, 100);
  EVE_end();

  // Keypad background.  Needs to be in the back.
  EVE_color_rgb(0x7d898b); // Oslo gray
  EVE_begin(EVE_RECTS);
  EVE_vertex2f(450, 110);
  EVE_vertex2f(720, 460);
  EVE_end();

  // Button circles
  EVE_vertex_format(0);
  EVE_point_size(20 * 16);
  EVE_begin(EVE_POINTS);
  // Top row
  EVE_color_rgb(0xe5ea22);
  EVE_vertex2f(500, 145);
  EVE_color_rgb(0x11aa10); // La Palma green
  EVE_vertex2f(586, 145);
  EVE_color_rgb(0xff0000);
  EVE_vertex2f(670, 145);
  // 2nd row, red, green, red.
  EVE_color_rgb(0xff0000);
  EVE_vertex2f(500 + 0 * 85, 200);
  EVE_color_rgb(0x11aa10);
  EVE_vertex2f(586, 200);
  EVE_color_rgb(0xff0000);
  EVE_vertex2f(670, 200);

  // The last 4 rows are all the same color, so a loop will work.
  EVE_color_rgb(0xff0000);
  uint32_t y_spacing = 55;
  for (int x = 0; x < 3; x = x + 1)
  {
    for (int y = 0; y < 4; y = y + 1)
    {
      EVE_color_rgb(0xff0000);
      EVE_vertex2f(500 + x * 85, 255 + y * y_spacing);
    }
  }
  EVE_end();

  // Characters over the top of the points.  Same coordinates.
  // Row 1
  EVE_color_rgb(0x000000);
  EVE_cmd_text(490, 118, 31, 0, "<");
//  EVE_color_rgb(0xFFFFFF);
//  EVE_cmd_text(661, 128, 30, 0, "X");
  // Rows 2-4
//  EVE_cmd_text(578, 128, 31, 0, "^");

  uint32_t line1y{150};
  uint32_t linex{70};

//  std::string test12 = "↑";

  // Entry guide static text.
  EVE_color_rgb(0xffffff);
  EVE_cmd_text(linex, line1y, 29, 0, "Button Parameter Entry");
  EVE_cmd_text(linex, line1y + 50, 29, 0, "Up/Down arrows to select");
//  EVE_cmd_text(linex, line1y + 80, 29, 0, " Down to next");
  EVE_cmd_text(linex, line1y + 110, 29, 0, "< Apply entry");
//  EVE_cmd_text(linex, line1y + 140, 29, 0, "X Exit no change");

//  EVE_cmd_text(578, 182, 31, 0, "^");

  // Up and down arrows.
  EVE_line_width(24);
  EVE_begin(EVE_LINES);
  EVE_color_rgb(0xFFFFFF); // White
  // Up arrow
  EVE_vertex2f(576, 147);
  EVE_vertex2f(586, 132);

  EVE_vertex2f(586, 132);
  EVE_vertex2f(596, 147);

  EVE_vertex2f(586, 134);
  EVE_vertex2f(586, 157);

  // Down arrow
  EVE_vertex2f(576, 197);
  EVE_vertex2f(586, 212);

  EVE_vertex2f(586, 212);
  EVE_vertex2f(596, 197);

  EVE_vertex2f(586, 189);
  EVE_vertex2f(586, 212);

  EVE_end();


//  EVE_cmd_text(578, 182, 31, 0, "^");

  EVE_cmd_endlist();
} // End button entry static list.

// Static items in button parameter entry.
FLASHMEM void EVE_Display::encoderEntryStatic_cmd_list()
{

  EVE_cmd_newlist(15000);
  EVE_vertex_format(0);
  EVE_line_width(16);

  // Rectangle enclosing guide and button graphics.
  EVE_color_rgb(0xff0000);
  EVE_begin(EVE_LINE_STRIP);
  EVE_vertex2f(50, 100);
  EVE_vertex2f(750, 100);
  EVE_vertex2f(750, 470);
  EVE_vertex2f(50, 470);
  EVE_vertex2f(50, 100);
  EVE_end();

  // Keypad background.  Needs to be in the back.
  EVE_color_rgb(0x7d898b); // Oslo gray
  EVE_begin(EVE_RECTS);
  EVE_vertex2f(450, 110);
  EVE_vertex2f(720, 460);
  EVE_end();

  // Button circles
  EVE_vertex_format(0);
  EVE_point_size(20 * 16);
  EVE_begin(EVE_POINTS);
  // Top row
  EVE_color_rgb(0xe5ea22);
  EVE_vertex2f(500, 145);
  EVE_color_rgb(0xff0000); // La Palma green
  EVE_vertex2f(586, 145);
  EVE_color_rgb(0xff0000);
  EVE_vertex2f(670, 145);
  // 2nd row, red, green, red.
  EVE_color_rgb(0xff0000);
  EVE_vertex2f(500 + 0 * 85, 200);
  EVE_color_rgb(0xff0000);
  EVE_vertex2f(586, 200);
  EVE_color_rgb(0xff0000);
  EVE_vertex2f(670, 200);

  // The last 4 rows are all the same color, so a loop will work.
  EVE_color_rgb(0xff0000);
  uint32_t y_spacing = 55;
  for (int x = 0; x < 3; x = x + 1)
  {
    for (int y = 0; y < 4; y = y + 1)
    {
      EVE_color_rgb(0xff0000);
      EVE_vertex2f(500 + x * 85, 255 + y * y_spacing);
    }
  }
  EVE_end();

  // Characters over the top of the points.  Same coordinates.
  // Row 1
  EVE_color_rgb(0x000000);
  EVE_cmd_text(490, 118, 31, 0, "<");
  EVE_color_rgb(0xFFFFFF);
//  EVE_cmd_text(661, 128, 30, 0, "X");
  // Rows 2-4
  //    EVE_cmd_number(492, 182, 30, 0, 7);
  //EVE_cmd_text(578, 128, 31, 0, "^");
  //EVE_cmd_text(578, 182, 31, 0, "^");

  uint32_t line1y{150};
  uint32_t linex{70};

  // Entry guide static text.
  EVE_color_rgb(0xffffff);
  EVE_cmd_text(linex, line1y, 29, 0, "Encoder Parameter Entry");
  EVE_cmd_text(linex, line1y + 50, 29, 0, "Use encoder to adjust");
  //  EVE_cmd_text(linex, line1y + 80, 29, 0, " Down to next");
  EVE_cmd_text(linex, line1y + 110, 29, 0, "< Apply entry");
//  EVE_cmd_text(linex, line1y + 140, 29, 0, "X Exit no change");

  EVE_cmd_endlist();
} // End static items in button parameter entry.

// This function writes the waterfall array from the Teensy to RAM_G in the display module.
void EVE_Display::writeWaterFalltoRAM_G(uint8_t *waterfall)
{
  // EVE_memWrite_sram_buffer(500000, waterfall, 512);

  EVE_start_cmd_burst();
  EVE_cmd_memwrite_burst(500000, 512, waterfall);
  EVE_end_cmd_burst();

  while (EVE_busy())
    ;
}

// void EVE_cmd_memcpy(uint32_t dest, uint32_t src, uint32_t num)
// This function moves bit map cells in RAM_G.
// This is done from the "bottom up".
void EVE_Display::moveBitmapCells()
{
  uint32_t source{0};

  for (int32_t i = waterfallLines; i > -1; i = i - 1)
  {
    source = 500000 + (512 * i);
    EVE_cmd_memcpy((source + 512), source, 512); // Overwrite next bitmap.
    // while (EVE_busy());
  }
}

// Static items in transmit and carrier calibration screen.
FLASHMEM void EVE_Display::transmitterCalStatic_cmd_list()
{
  EVE_cmd_newlist(20000);
  EVE_vertex_format(0);
  EVE_line_width(16);

  // T41-EP "brand"
  EVE_color_rgb(0xffffff);
  EVE_cmd_text(510, 10, 30, 0, "T41-3 SDT");

  // RF spectrum display container.
  EVE_color_rgb(0xff0000); // Red
  EVE_line_width(16);
  EVE_begin(EVE_LINE_STRIP);
  EVE_vertex2f(0, 100);
  EVE_vertex2f(513, 100);
  EVE_vertex2f(513, 470);
  EVE_vertex2f(0, 470);
  EVE_vertex2f(0, 100);
  EVE_end();

  // Band and mode information line.
  EVE_color_rgb(0xffffff); // White
  EVE_cmd_text(0, 80, 26, 0, "Center Freq");
  EVE_cmd_text(200, 80, 26, 0, "Mode");

  // Show transmit/receive status.
  EVE_begin(EVE_RECTS);
  EVE_color_rgb(0xff0000);
  EVE_color_a(100); // Set transparency.
  EVE_vertex2f(X_R_STATUS_X, X_R_STATUS_Y);
  EVE_vertex2f(X_R_STATUS_X + 55, X_R_STATUS_Y + 25);
  EVE_end();
  EVE_color_a(255);
  EVE_color_rgb(0xffffff);
  EVE_cmd_text(X_R_STATUS_X + 12, X_R_STATUS_Y + 0, 28, 0, "TX");

  // Draw "Watts" above on right side of spectrum window.
  EVE_color_rgb(0xff0000);
  EVE_cmd_text(450, 80, 26, 0, "Watts");

  // Draw the button push guides.
  EVE_color_rgb(0x00ff00);
  EVE_cmd_text(550, 125, 28, 0, "Incr =");
  EVE_cmd_text(640, 150, 30, 0, "dBC");
  EVE_cmd_text(550, 185, 28, 0, "User1: Gain/Phase");
  EVE_cmd_text(550, 215, 28, 0, "User2: Increment");
  EVE_cmd_text(550, 245, 28, 0, "Zoom: Auto-Cal");
  EVE_cmd_text(550, 275, 28, 0, "Filter: Refine-Cal");

  EVE_cmd_endlist();
} // End of transmit/carrier calibration static.

// Transmitter calibration screen (dynamic);
void EVE_Display::drawTransmitterCalScreen(int16_t *fftArray)
{
  std::string mode{""};
  std::string sideband{""};
  int32_t fftPlot{0};

  EVE_start_cmd_burst();
  EVE_cmd_dl_burst(CMD_DLSTART);
  EVE_cmd_dl_burst(DL_CLEAR_COLOR_RGB | 0x000000);
  EVE_cmd_dl_burst(DL_CLEAR | CLR_COL | CLR_STN | CLR_TAG);
  EVE_vertex_format_burst(0);
  EVE_point_size_burst(32);

  // Static items.
  EVE_cmd_calllist_burst(20000);

  // Draw the calibration type and calibration mode.
  int textX{0};
  if (bands.bands[ConfigData.currentBand].sideband == Sideband::LOWER)
    textX = 30;
  if (bands.bands[ConfigData.currentBand].sideband == Sideband::UPPER)
    textX = 300;
  EVE_color_rgb(0xff0000);

  if (txcalibrater.mode == 0)
    EVE_cmd_text(textX, 200, 30, 0, "Transmit CW");
  if (txcalibrater.mode == 1)
    EVE_cmd_text(textX, 200, 30, 0, "Transmit SSB");

  EVE_cmd_text(textX, 240, 30, 0, "Calibrate");
  if (txcalibrater.calTypeFlag == 1)
    EVE_cmd_text(textX, 280, 30, 0, "Sideband");
  if (txcalibrater.calTypeFlag == 2)
    EVE_cmd_text(textX, 280, 30, 0, "Carrier");

  if (txcalibrater.autoCal and not txcalibrater.refineCal)
    EVE_cmd_text(textX, 320, 30, 0, "Auto Mode");
  if (txcalibrater.refineCal)
    EVE_cmd_text(textX, 320, 30, 0, "Refine Mode");
  if (not txcalibrater.autoCal and not txcalibrater.refineCal)
    EVE_cmd_text(textX, 320, 30, 0, "Manual Mode");

  // Draw the red and blue boxes.  These have to be dynamic because
  // they depend on sideband and type of calibration.
  // Sideband calibration.
  if (bands.bands[ConfigData.currentBand].sideband == Sideband::LOWER and txcalibrater.calTypeFlag == 1)
  {
    EVE_begin(EVE_RECTS);
    EVE_color_a(100);        // Set transparency.
    EVE_color_rgb(0xff0000); // Red
    EVE_vertex2f(311, 101);
    EVE_vertex2f(331, 469);
    EVE_color_rgb(0x0000ff); // Blue
    EVE_vertex2f(246, 101);
    EVE_vertex2f(266, 469);
    EVE_end();
  }
  if (bands.bands[ConfigData.currentBand].sideband == Sideband::UPPER and txcalibrater.calTypeFlag == 1)
  {
    EVE_begin(EVE_RECTS);
    EVE_color_a(100);        // Set transparency.
    EVE_color_rgb(0xff0000); // Red
    EVE_vertex2f(181, 101);
    EVE_vertex2f(201, 469);
    EVE_color_rgb(0x0000ff); // Blue
    EVE_vertex2f(246, 101);
    EVE_vertex2f(266, 469);
    EVE_end();
  }

  // Carrier calibration.
  if (bands.bands[ConfigData.currentBand].sideband == Sideband::LOWER and txcalibrater.calTypeFlag == 2)
  {
    EVE_begin(EVE_RECTS);
    EVE_color_a(100);        // Set transparency.
    EVE_color_rgb(0xff0000); // Red
    EVE_vertex2f(279, 101);
    EVE_vertex2f(299, 469);
    EVE_color_rgb(0x0000ff); // Blue
    EVE_vertex2f(247, 101);
    EVE_vertex2f(267, 469);
    EVE_end();
  }
  if (bands.bands[ConfigData.currentBand].sideband == Sideband::UPPER and txcalibrater.calTypeFlag == 2)
  {
    EVE_begin(EVE_RECTS);
    EVE_color_a(100);        // Set transparency.
    EVE_color_rgb(0xff0000); // Red
    EVE_vertex2f(215, 101);
    EVE_vertex2f(235, 469);
    EVE_color_rgb(0x0000ff); // Blue
    EVE_vertex2f(247, 101);
    EVE_vertex2f(267, 469);
    EVE_end();
  }

  // Main frequency.
  EVE_color_rgb_burst(0x0000FF);
  EVE_color_a(255);
  EVE_cmd_number_burst(0, 37, 31, 0, TxRxFreq); // x, y, font, options, n

  // Draw the RF spectrum.
  EVE_color_rgb_burst(0xf0f000); // Make spectrum blue.
  EVE_begin_burst(EVE_LINE_STRIP);
  for (int j = 0; j < 512; j = j + 1)
  {
    // Invert and offset for the display.
    fftPlot = -fftArray[j] + txcalibrater.rawSpectrumPeak + 110;
    if (fftPlot > 469)
      fftPlot = 469;
    EVE_vertex2f_burst(j, fftPlot);
  }
  EVE_end_burst();

  // On Band Information line at top of RF spectrum.
  EVE_cmd_number_burst(90, 80, 26, 0, ConfigData.centerFreq);
  EVE_cmd_text_burst(160, 80, 26, 0, bands.bands[ConfigData.currentBandA].name);

  // Which sideband?
  switch (bands.bands[ConfigData.currentBand].sideband)
  {
  case Sideband::LOWER:
    sideband = "LSB";
    break;
  case Sideband::UPPER:
    sideband = "USB";
    break;
  case Sideband::BOTH_AM:
    sideband = "DSB";
    break;
  case Sideband::BOTH_SAM:
    sideband = " DSB";
    break;
  default:
    break;
  }
  EVE_cmd_text_burst(270, 80, 26, 0, sideband.c_str());

  // Show RF gain.
  // EVE_color_rgb_burst(0xffffff);
  //  EVE_cmd_number_burst(SPECTRUM_LEFT_X + 120, SPECTRUM_TOP_Y + 2, 26, 0, ConfigData.rfGain[ConfigData.currentBand]);

  // Draw TX power setting above on right side of spectrum window.
  EVE_color_rgb_burst(0xff0000);
  EVE_color_a(255);
  EVE_cmd_number_burst(430, 80, 26, 0, ConfigData.transmitPowerLevel);

  // Draw current suppression in dBC.
  dtostrf(txcalibrater.adjdB_avg, 4, 1, buffer);
  EVE_color_a(255);
  EVE_cmd_text_burst(550, 150, 30, 0, buffer);

  // Show IQ Gain and Phase for transmit calibration.
  EVE_color_rgb(0xffffff);
  if (txcalibrater.calTypeFlag == 1)
  {
    EVE_cmd_text_burst(10, 5, 29, 0, "IQ Gain");
    EVE_cmd_text_burst(210, 5, 29, 0, "IQ Phase");
  }
  if (txcalibrater.calTypeFlag == 2)
  {
    EVE_cmd_text_burst(10, 5, 29, 0, "I Offset");
    EVE_cmd_text_burst(210, 5, 29, 0, "Q Offset");
  }

  // Draw amplitude and phase near top of display.
  if (txcalibrater.IQCalType == 0)
    EVE_color_rgb(0xff0000);
  else
    EVE_color_rgb(0xffffff);

  if (txcalibrater.calTypeFlag == 1)
    dtostrf(txcalibrater.amplitude, 4, 3, buffer);

  if (txcalibrater.calTypeFlag == 2)
    dtostrf(txcalibrater.iDCoffset, 5, 4, buffer);

  EVE_cmd_text_burst(100, 5, 29, 0, buffer);

  if (txcalibrater.IQCalType == 1)
    EVE_color_rgb(0xff0000);
  else
    EVE_color_rgb(0xffffff);

  if (txcalibrater.calTypeFlag == 1)
    dtostrf(txcalibrater.phase, 4, 3, buffer);

  if (txcalibrater.calTypeFlag == 2)
    dtostrf(txcalibrater.qDCoffset, 5, 4, buffer);

  EVE_cmd_text_burst(320, 5, 29, 0, buffer);

  if (txcalibrater.calTypeFlag == 1)
  {
    dtostrf(txcalibrater.xmitIncrement, 4, 3, buffer);
    EVE_color_rgb(0xff0000);
    EVE_cmd_text(608, 125, 28, 0, buffer);
  }

  if (txcalibrater.calTypeFlag == 2)
  {
    dtostrf(txcalibrater.carrIncrement, 5, 4, buffer);
    EVE_color_rgb(0xff0000);
    EVE_cmd_text(608, 125, 28, 0, buffer);
  }

  EVE_display_burst();
  EVE_cmd_swap_burst();
  EVE_end_cmd_burst();
  while (EVE_busy())
    ;
} // End transmit calibration dynamic.

// Static items in receiver calibration screen.
FLASHMEM void EVE_Display::receiverCalStatic_cmd_list()
{
  EVE_cmd_newlist(50000);
  EVE_vertex_format(0);
  EVE_line_width(16);

  // T41-EP "brand"
  EVE_color_rgb(0xffffff);
  EVE_cmd_text(510, 10, 30, 0, "T41-3 SDT");

  // RF spectrum display container.
  EVE_color_rgb(0xff0000); // Red
  EVE_line_width(16);
  EVE_begin(EVE_LINE_STRIP);
  EVE_vertex2f(0, 100);
  EVE_vertex2f(513, 100);
  EVE_vertex2f(513, 470);
  EVE_vertex2f(0, 470);
  EVE_vertex2f(0, 100);
  EVE_end();

  // Band and mode information line.
  EVE_color_rgb(0xffffff); // White
  EVE_cmd_text(0, 80, 26, 0, "Center Freq");
  EVE_cmd_text(200, 80, 26, 0, "Mode");

  // Show transmit/receive status.
  EVE_begin(EVE_RECTS);
  EVE_color_rgb(0xff0000);
  EVE_color_a(100); // Set transparency.
  EVE_vertex2f(X_R_STATUS_X, X_R_STATUS_Y);
  EVE_vertex2f(X_R_STATUS_X + 55, X_R_STATUS_Y + 25);
  EVE_end();
  EVE_color_a(255);
  EVE_color_rgb(0xffffff);
  EVE_cmd_text(X_R_STATUS_X + 12, X_R_STATUS_Y + 0, 28, 0, "TX");

  // Draw "Watts" above on right side of spectrum window.
  EVE_color_rgb(0xff0000);
  EVE_cmd_text(450, 80, 26, 0, "Watts");

  // Draw the button push guides.
  EVE_color_rgb(0x00ff00);
  EVE_cmd_text(550, 125, 28, 0, "Incr =");
  EVE_cmd_text(640, 150, 30, 0, "dBC");
  EVE_cmd_text(550, 185, 28, 0, "User1: Gain/Phase");
  EVE_cmd_text(550, 215, 28, 0, "User2: Increment");
  EVE_cmd_text(550, 245, 28, 0, "Zoom: Auto-Cal");
  EVE_cmd_text(550, 275, 28, 0, "Filter: Refine-Cal");

  EVE_cmd_endlist();
} // End of receiver calibration static.

// Receiver calibration screen (dynamic);
void EVE_Display::drawReceiverCalScreen(int16_t *fftArray)
{
  std::string mode{""};
  std::string sideband{""};
  int32_t fftPlot{0};

  EVE_start_cmd_burst();
  EVE_cmd_dl_burst(CMD_DLSTART);
  EVE_cmd_dl_burst(DL_CLEAR_COLOR_RGB | 0x000000);
  EVE_cmd_dl_burst(DL_CLEAR | CLR_COL | CLR_STN | CLR_TAG);
  EVE_vertex_format_burst(0);
  EVE_point_size_burst(32);

  // Static items.
  EVE_cmd_calllist_burst(50000);

  // Draw the calibration type and calibration mode.
  EVE_color_rgb(0xff0000);
  if (rxcalibrater.mode == 0)
    EVE_cmd_text(170, 200, 30, 0, "Receive CW");
  if (rxcalibrater.mode == 1)
    EVE_cmd_text(170, 200, 30, 0, "Receive SSB");
  EVE_cmd_text(170, 240, 30, 0, "Calibrate");
  if (rxcalibrater.autoCal or rxcalibrater.refineCal)
    EVE_cmd_text(170, 280, 30, 0, "Auto Mode");
  else
    EVE_cmd_text(170, 280, 30, 0, "Manual Mode");

  // Draw the red and blue boxes.  These have to be dynamic because
  // they depend on sideband and type of calibration.  True for receieve???
  EVE_begin(EVE_RECTS);
  EVE_color_a(100);        // Set transparency.
  EVE_color_rgb(0xff0000); // Red
  EVE_vertex2f(113, 101);
  EVE_vertex2f(145, 469);
  EVE_color_rgb(0x0000ff); // Blue
  EVE_vertex2f(369, 101);
  EVE_vertex2f(401, 469);
  EVE_end();

  // Main frequency.
  EVE_color_rgb_burst(0x0000FF);
  EVE_color_a(255);
  EVE_cmd_number_burst(0, 37, 31, 0, TxRxFreq); // x, y, font, options, n

  // Draw the RF spectrum.
  EVE_color_rgb_burst(0xf0f000); // Make spectrum blue.
  EVE_begin_burst(EVE_LINE_STRIP);
  for (int j = 0; j < 512; j = j + 1)
  {
    // Invert and offset for the display.
    fftPlot = -fftArray[j] + rxcalibrater.rawSpectrumPeak + 110;
    if (fftPlot > 469)
      fftPlot = 469;
    EVE_vertex2f_burst(j, fftPlot);
  }
  EVE_end_burst();

  // On Band Information line at top of RF spectrum.
  EVE_cmd_number_burst(90, 80, 26, 0, ConfigData.centerFreq);
  EVE_cmd_text_burst(160, 80, 26, 0, bands.bands[ConfigData.currentBandA].name);

  // Which sideband?
  switch (bands.bands[ConfigData.currentBand].sideband)
  {
  case Sideband::LOWER:
    sideband = "LSB";
    break;
  case Sideband::UPPER:
    sideband = "USB";
    break;
  case Sideband::BOTH_AM:
    sideband = "DSB";
    break;
  case Sideband::BOTH_SAM:
    sideband = " DSB";
    break;
  default:
    break;
  }
  EVE_cmd_text_burst(270, 80, 26, 0, sideband.c_str());

  // Draw TX power setting above on right side of spectrum window.
  EVE_color_rgb_burst(0xff0000);
  EVE_color_a(255);
  EVE_cmd_number_burst(430, 80, 26, 0, ConfigData.transmitPowerLevel);

  // Draw current suppression in dBC.
  dtostrf(rxcalibrater.adjdB_avg, 4, 1, buffer);
  EVE_color_a(255);
  EVE_cmd_text_burst(550, 150, 30, 0, buffer);

  // Show IQ Gain and Phase for transmit calibration.
  EVE_color_rgb(0xffffff);
  EVE_cmd_text_burst(10, 5, 29, 0, "IQ Gain");
  EVE_cmd_text_burst(210, 5, 29, 0, "IQ Phase");

  // Draw amplitude and phase near top of display.
  if (rxcalibrater.IQCalType == 0)
    EVE_color_rgb(0xff0000);
  else
    EVE_color_rgb(0xffffff);
  dtostrf(rxcalibrater.amplitude, 4, 3, buffer);
  EVE_cmd_text_burst(100, 5, 29, 0, buffer);
  if (rxcalibrater.IQCalType == 1)
    EVE_color_rgb(0xff0000);
  else
    EVE_color_rgb(0xffffff);
  dtostrf(rxcalibrater.phase, 4, 3, buffer);
  EVE_cmd_text_burst(320, 5, 29, 0, buffer);

  dtostrf(rxcalibrater.increment, 4, 3, buffer);
  EVE_color_rgb(0xff0000);
  EVE_cmd_text(608, 125, 28, 0, buffer);

  EVE_display_burst();
  EVE_cmd_swap_burst();
  EVE_end_cmd_burst();
  while (EVE_busy())
    ;
} // End receiver calibration dynamic.

// The only thing which this is required to do is draw rectangles with height
// according to an array.
void EVE_Display::drawEqualizerAdjustScreen(int EQType)
{
  std::string rXeqFreq[14]{" 200", " 250", " 315", " 400", " 500", " 630", " 800", "1000", "1250", "1600", "2000", "2500", "3150", "4000"};
  std::string tXeqFreq[14]{"  50", "  71", " 100", " 141", " 200", " 283", " 400", " 566", " 800", "1131", "1600", "2263", "3200", "4526"};
  int barWidth = 49;

  EVE_start_cmd_burst();
  EVE_cmd_dl_burst(CMD_DLSTART);
  EVE_cmd_dl_burst(DL_CLEAR_COLOR_RGB | 0x000000);
  EVE_cmd_dl_burst(DL_CLEAR | CLR_COL | CLR_STN | CLR_TAG);
  EVE_vertex_format_burst(0);

  EVE_cmd_calllist_burst(60000);
  EVE_color_rgb_burst(0x00FF00); // Green
  if (EQType == 0)
    EVE_cmd_text_burst(130, 5, 30, 0, "Receive Equalizer (Nominal value is 100)");
  if (EQType == 1)
    EVE_cmd_text_burst(140, 5, 30, 0, "Transmit Equalizer (Nominal value is 0)");

  // Draw bars.
  for (uint32_t iFreq = 0; iFreq < EQUALIZER_CELL_COUNT; iFreq++)
  {
    EVE_begin_burst(EVE_RECTS);
    EVE_color_a(100);              // Set transparency.
    EVE_color_rgb_burst(0xff0000); // Red
    if (iFreq == menuProc.columnIndex)
      EVE_color_rgb_burst(0x00FF00); // The active bar is green.
    // Translate so that bar is halfway at 0 dB.
    if (EQType == 0)
      EVE_vertex2f_burst(60 + iFreq * barWidth, 240 - ConfigData.equalizerRec[iFreq]);
    if (EQType == 1)
      EVE_vertex2f_burst(60 + iFreq * barWidth, 240 - ConfigData.equalizerXmt[iFreq]);
    EVE_vertex2f_burst(60 + iFreq * barWidth + (barWidth - 5), 399);
    EVE_end_burst();
  }

  // Write the frequencies and bar values on the X-axis.
  EVE_color_a(255); // Set transparency.
  for (uint32_t iFreq = 0; iFreq < EQUALIZER_CELL_COUNT; iFreq++)
  {
    EVE_color_rgb_burst(0xff0000); // Red
    if (EQType == 0)
      EVE_cmd_text_burst(64 + iFreq * barWidth, 410, 27, 0, rXeqFreq[iFreq].c_str());
    if (EQType == 1)
      EVE_cmd_text_burst(64 + iFreq * barWidth, 410, 27, 0, tXeqFreq[iFreq].c_str());
    if (iFreq == menuProc.columnIndex)
      EVE_color_rgb_burst(0x00FF00); // The active bar is green.
    if (EQType == 0)
      EVE_cmd_number_burst(66 + iFreq * barWidth, 440, 27, EVE_OPT_SIGNED, ConfigData.equalizerRec[iFreq]);
    if (EQType == 1)
      EVE_cmd_number_burst(66 + iFreq * barWidth, 440, 27, EVE_OPT_SIGNED, ConfigData.equalizerXmt[iFreq]);
  }

  EVE_display_burst();
  EVE_cmd_swap_burst();
  EVE_end_cmd_burst();
  while (EVE_busy())
    ;
}

FLASHMEM void EVE_Display::equalizerAdjustStatic_cmd_list()
{
  EVE_cmd_newlist(60000);
  EVE_vertex_format(0);
  EVE_line_width(16);

  // Blue boundary box.
  EVE_begin(EVE_LINE_STRIP);
  EVE_color_rgb(0x0000FF); // Blue
  EVE_vertex2f(50, 50);
  EVE_vertex2f(750, 50);
  EVE_vertex2f(750, 400);
  EVE_vertex2f(50, 400);
  EVE_vertex2f(50, 51);
  EVE_end();

  // Draw center line.
  EVE_begin(EVE_LINES);
  EVE_color_rgb(0xFFFFFF); // Red
  EVE_vertex2f(51, 240);
  EVE_vertex2f(749, 240);
  EVE_end();

  EVE_color_rgb(0xFFFFFF); // White
  // Write vertical scale numbers.
  EVE_cmd_text_burst(20, 40, 28, 0, "dB");
  EVE_cmd_text_burst(20, 230, 28, 0, "0");
  // Units for X-axis.
  EVE_cmd_text(755, 409, 27, 0, "Hz");
  EVE_cmd_text(755, 440, 27, 0, "dB");

  EVE_cmd_endlist();
}

// Static items in Switch Matrix Calibration screen.
FLASHMEM void EVE_Display::switchMatrixCalStatic_cmd_list()
{
  std::string buttonNames[18]{
      "SELECT",
      "MAIN MENU_UP",
      "BAND UP",
      "ZOOM",
      "MAIN MENU DOWN",
      "BAND DOWN",
      "FILTER",
      "SIDEBAND",
      "MODE",
      "NOISE REDUCTION",
      "NOTCH FILTER",
      "MUTE AUDIO",
      "FINE INCREMENT",
      "DECODER TOGGLE",
      "COARSE INCREMENT",
      "RESET TUNING",
      "UNUSED 1",
      "BEARING"};
  EVE_cmd_newlist(70000);
  EVE_vertex_format(0);
  EVE_line_width(16);

  EVE_cmd_text(320, 5, 29, 0, "Switch Matrix Calibration");

  // Keypad background.  Needs to be in the back.
  EVE_color_rgb(0x7d898b); // Oslo gray
  EVE_color_a(90);
  EVE_begin(EVE_RECTS);
  EVE_vertex2f(215, 40);
  EVE_vertex2f(720, 470);
  EVE_end();
  EVE_color_a(255);

  // Button circles
  EVE_vertex_format(0);
  EVE_point_size(15 * 16);
  EVE_begin(EVE_POINTS);
  EVE_color_a(100);        // Transparency so the number can be seen.
  EVE_color_rgb(0xFF0000); // Red indicating uncalibrated.
  uint32_t rowSpacing = 72;
  uint32_t columnSpacing = 160;
  uint32_t xOrigin = 300;
  uint32_t yOrigin = 60;
  for (int x = 0; x < 6; x = x + 1)
  {
    for (int y = 0; y < 3; y = y + 1)
    {
      EVE_vertex2f(xOrigin + columnSpacing * y, yOrigin + rowSpacing * x);
    }
  }
  EVE_end();

  // Characters over the top of the points.
  uint32_t xTextOrigin = 295;
  uint32_t yTextOrigin = 48;
  uint32_t bigNumberOffset = 0;
  uint32_t textOffset = 0;

  uint32_t counter = 1;
  for (uint32_t row = 0; row < 6; row = row + 1)
  {
    for (uint32_t column = 0; column < 3; column = column + 1)
    {
      if (counter > 9)
        bigNumberOffset = 6;
      EVE_color_rgb(0xFFFFFF);
      EVE_cmd_number(xTextOrigin + columnSpacing * column - bigNumberOffset, yTextOrigin + rowSpacing * row, 28, 0, counter);

      textOffset = buttonNames[counter - 1].length() * 4;
      EVE_cmd_text(xTextOrigin + columnSpacing * column - textOffset, yTextOrigin + rowSpacing * row + 40, 27, 0, buttonNames[counter - 1].c_str());
      counter = counter + 1;
    }
  }

  uint32_t line1y{30};
  uint32_t linex{30};

  // Entry guide static text.
  EVE_color_rgb(0xff0000);
  EVE_cmd_text(linex, line1y + 30, 28, 0, "Push buttons\nin ascending\norder.");

  EVE_cmd_endlist();
} // End switch matrix cal static list.


// Switch Matric release button notification.
// This is used with #define DEBUG_SWITCH_CAL  in MyConfigurationFile.h.
void EVE_Display::drawReleaseButtonScreen()
{
  uint32_t line1y{30};
  uint32_t linex{30};

  EVE_cmd_dl(CMD_DLSTART);
  EVE_cmd_dl(DL_CLEAR_COLOR_RGB | 0x000000);
  EVE_cmd_dl(DL_CLEAR | CLR_COL | CLR_STN | CLR_TAG);
  EVE_vertex_format(0);
  EVE_point_size(32);

  EVE_cmd_text(linex, line1y + 60, 28, 0, "Release button within 5 seconds to start Switch Matrix calibration.");

  EVE_display();
  EVE_cmd_swap();
  while (EVE_busy());
}


// Switch Matric calibration (dynamic).
void EVE_Display::drawSwitchMatrixCalScreen()
{
  EVE_cmd_dl(CMD_DLSTART);
  EVE_cmd_dl(DL_CLEAR_COLOR_RGB | 0x000000);
  EVE_cmd_dl(DL_CLEAR | CLR_COL | CLR_STN | CLR_TAG);
  EVE_vertex_format(0);
  EVE_point_size(32);

  // Static items.
  EVE_cmd_calllist(70000);

  // Characters over the top of the points.
  uint32_t xOrigin = 300;
  uint32_t yOrigin = 60;
  uint32_t xTextOrigin = 295;
  uint32_t yTextOrigin = 48;
  uint32_t rowSpacing = 72;
  uint32_t columnSpacing = 160;
  uint32_t index = 0;
  EVE_color_rgb(0xFFFFFF);

  for (uint32_t row = 0; row < 6; row = row + 1)
  {
    for (uint32_t column = 0; column < 3; column = column + 1)
    {
      EVE_color_rgb(0xFFFFFF); // White
      EVE_cmd_number(xTextOrigin + columnSpacing * column + 50, yTextOrigin + rowSpacing * row, 28, 0, CalData.switchValues[index]);
      // Turn the circle green since it has been pushed.
      if (buttonFinished[index])
      {
//        Serial.printf("EVE buttonFinished[%d] = %d row = %d column = %d\n", index, buttonFinished[index], row, column);
        EVE_vertex_format(0);
        EVE_point_size(15 * 16);
        EVE_begin(EVE_POINTS);
        EVE_color_a(100);        // Transparency so the number can be seen.
        EVE_color_rgb(0x00FF00); // Green
        EVE_vertex2f(xOrigin + columnSpacing * column, yOrigin + rowSpacing * row);
        EVE_end();
      }
      index = index + 1;
    }
  }

  EVE_display();
  EVE_cmd_swap();
//  EVE_end_cmd_burst();
  while (EVE_busy())
    ;
}

// Static items in receiver display.
FLASHMEM void EVE_Display::transmitterStatic_cmd_list()
{
  EVE_cmd_newlist(80000);
  EVE_vertex_format(0);
  EVE_line_width(16);

  // T41-EP "brand"
  EVE_color_rgb(0xffffff);
  EVE_cmd_text(510, 5, 30, 0, "T41-EP SDT");

  // Current version.
  EVE_color_rgb(0x00ff00);
  EVE_cmd_text(705, 5, 27, 0, "T41EVE.01");

  // Spectrum center line.
  EVE_begin(EVE_LINES);
  EVE_line_width(10);
  EVE_color_a(110);        // Make center line transparent.
  EVE_color_rgb(0xffffff); // White
  EVE_vertex2f(256, 100);
  EVE_vertex2f(256, 249);
  EVE_end();
  EVE_color_a(255);

  // RF spectrum display container.
  EVE_color_rgb(0xff0000); // Red
  EVE_line_width(16);
  EVE_begin(EVE_LINE_STRIP);
  EVE_vertex2f(0, 100);
  EVE_vertex2f(513, 100);
  EVE_vertex2f(513, 250);
  EVE_vertex2f(0, 250);
  EVE_vertex2f(0, 100);
  EVE_end();

  // Audio spectrum display container.
  EVE_color_rgb(0xff0000); // Red
  EVE_begin(EVE_LINE_STRIP);
  EVE_vertex2f(531, 129);
  EVE_vertex2f(788, 129);
  EVE_vertex2f(788, 247);
  EVE_vertex2f(531, 247);
  EVE_vertex2f(531, 129);
  EVE_end();

  // Information window container.
  EVE_color_rgb(0xff0000); // Red
  EVE_begin(EVE_LINE_STRIP);
  EVE_vertex2f(530, 280);
  EVE_vertex2f(799, 280);
  EVE_vertex2f(799, 475);
  EVE_vertex2f(530, 475);
  EVE_vertex2f(530, 280);
  EVE_end();

  uint32_t line1y{311};

  // Information window static text.
  EVE_color_rgb(0xffffff);
  EVE_cmd_text(533, 278, 30, 0, "VOL:");
  EVE_cmd_text(660, 278, 30, 0, "AGC");
  EVE_cmd_text(533, line1y, 27, 0, "Fine Inc:");
  EVE_cmd_text(630, line1y, 27, 0, "Coarse Inc:");
  EVE_cmd_text(533, line1y + 20, 27, 0, "AutoNotch:");
  EVE_cmd_text(573, line1y + 40, 27, 0, "Noise:");
  EVE_cmd_text(570, line1y + 60, 27, 0, "Zoom:");
  EVE_cmd_text(538, line1y + 80, 27, 0, "Compress:");
  EVE_cmd_text(572, line1y + 100, 27, 0, "Keyer:");
  EVE_cmd_text(552, line1y + 120, 27, 0, "Decoder:");
  EVE_cmd_text(547, line1y + 140, 27, 0, "Equalizer:");
  EVE_cmd_text(630, line1y + 140, 27, 0, "Rx");
  EVE_cmd_text(687, line1y + 140, 27, 0, "Tx");

  // Band and mode information line.
  EVE_color_rgb(0xffffff); // White
  EVE_cmd_text(0, 80, 26, 0, "Center Freq");
  EVE_cmd_text(200, 80, 26, 0, "Mode");

  // Draw audio spectrum frequency axis.
  std::string audioMarkers[6] = {"0k", "1k", "2k", "3k", "4k", "5k"};

  for (int k = 0; k < 6; k++)
  {
    EVE_color_rgb(0x00ff00); // Green
    EVE_cmd_text(BAND_INDICATOR_X - 14 + k * 43.8, SPECTRUM_BOTTOM + 10, 27, 0, audioMarkers[k].c_str());
    EVE_color_rgb(0xff0000); // Red
    EVE_begin(EVE_LINES);    // Tick marks.
    EVE_vertex2f(BAND_INDICATOR_X - 9 + k * 43.8, AUDIO_SPECTRUM_BOTTOM);
    EVE_vertex2f(BAND_INDICATOR_X - 9 + k * 43.8, AUDIO_SPECTRUM_BOTTOM + 7);
    EVE_end();
  }

  // Signal strength indicator.
  int i;
  // DB2OO, 30-AUG-23: the white line must only go till S9
  // S-Meter container lines.
  EVE_color_rgb(0xffffff);
  EVE_begin(EVE_LINES);
  EVE_vertex2f(SMETER_X, SMETER_Y - 1);
  EVE_vertex2f(SMETER_X + 9 * pixels_per_s, SMETER_Y - 1);

  EVE_vertex2f(SMETER_X, SMETER_Y + SMETER_BAR_HEIGHT + 2);
  EVE_vertex2f(SMETER_X + 9 * pixels_per_s, SMETER_Y + SMETER_BAR_HEIGHT + 2);
  EVE_end();

  for (i = 0; i < 10; i++) // Draw tick marks for S-values.
  {
#ifdef TCVSDR_SMETER
    // DB2OO, 30-AUG-23: draw wider tick marks in the style of the Teensy Convolution SDR
    EVE_color_rgb(0xffffff);
    EVE_begin(EVE_LINES);
    EVE_vertex2f(SMETER_X + i * pixels_per_s, SMETER_Y - 7 - (i % 2) * 2);
    EVE_vertex2f(SMETER_X + i * pixels_per_s, SMETER_Y - 7 - (i % 2) * 2 + 6 + (i % 2) * 2);
    EVE_end();
#else
//    tft.drawFastVLine(SMETER_X + i * 12.2, SMETER_Y - 6, 7, RA8875_WHITE);
#endif
  }

  // DB2OO, 30-AUG-23: the green line must start at S9
  // Green part of S-Meter rectangle.
  EVE_color_rgb(0x00ff00); // Green
  EVE_begin(EVE_LINES);
  EVE_vertex2f(SMETER_X + 9 * pixels_per_s, SMETER_Y - 1);
  EVE_vertex2f(SMETER_X + 9 * pixels_per_s + SMETER_BAR_LENGTH + 2 - 9 * pixels_per_s, SMETER_Y - 1);
  EVE_vertex2f(SMETER_X + 9 * pixels_per_s, SMETER_Y + SMETER_BAR_HEIGHT + 2);
  EVE_vertex2f(SMETER_X + 9 * pixels_per_s + SMETER_BAR_LENGTH + 2 - 9 * pixels_per_s, SMETER_Y + SMETER_BAR_HEIGHT + 2);
  EVE_end();

  for (i = 1; i <= 3; i++) // Draw tick marks for s9+ values in 10dB steps.
  {
#ifdef TCVSDR_SMETER
    EVE_color_rgb(0x00ff00);
    EVE_begin(EVE_LINES);
    EVE_vertex2f(SMETER_X + 9 * pixels_per_s + i * pixels_per_s * 10.0 / 6.0, SMETER_Y - 8 + (i % 2) * 2);
    EVE_vertex2f(SMETER_X + 9 * pixels_per_s + i * pixels_per_s * 10.0 / 6.0, SMETER_Y - 8 + (i % 2) * 2 + 8 - (i % 2) * 2);
    EVE_end();
#else
//    tft.drawFastVLine(SMETER_X + 9 * pixels_per_s + i * pixels_per_s * 10.0 / 6.0, SMETER_Y - 6, 7, RA8875_GREEN);
#endif
  }
  // Draw ends of the S-Meter rectangle.
  EVE_color_rgb(0xffffff);
  EVE_begin(EVE_LINES);
  EVE_vertex2f(SMETER_X, SMETER_Y - 1);
  EVE_vertex2f(SMETER_X, SMETER_Y - 1 + SMETER_BAR_HEIGHT + 3);
  EVE_color_rgb(0xffffff);
  EVE_vertex2f(SMETER_X + SMETER_BAR_LENGTH + 2, SMETER_Y - 1);
  EVE_vertex2f(SMETER_X + SMETER_BAR_LENGTH + 2, SMETER_Y - 1 + SMETER_BAR_HEIGHT + 3);
  EVE_end();

  // Draw the S-Meter scale.
  EVE_cmd_text(SMETER_X - 8, SMETER_Y - 30, 27, 0, "S");
  EVE_cmd_text(SMETER_X + 8, SMETER_Y - 30, 27, 0, "1");
  EVE_cmd_text(SMETER_X + 32, SMETER_Y - 30, 27, 0, "3");
  EVE_cmd_text(SMETER_X + 56, SMETER_Y - 30, 27, 0, "5");
  EVE_cmd_text(SMETER_X + 80, SMETER_Y - 30, 27, 0, "7");
  EVE_cmd_text(SMETER_X + 104, SMETER_Y - 30, 27, 0, "9");
  EVE_cmd_text(SMETER_X + 133, SMETER_Y - 30, 27, 0, "+20dB");

  //  Draw dBm at end of S-Meter.
  EVE_color_rgb(0xffffff);
  EVE_cmd_text(SMETER_X + 230, SMETER_Y + 3, 26, 0, "dBm");

  // Show transmit/receive status.
  EVE_begin(EVE_RECTS);
  EVE_color_rgb(0xFF0000);
  EVE_color_a(100); // Set transparency.
  EVE_vertex2f(X_R_STATUS_X, X_R_STATUS_Y);
  EVE_vertex2f(X_R_STATUS_X + 55, X_R_STATUS_Y + 25);
  EVE_end();
  EVE_color_a(255);
  EVE_color_rgb(0xffffff);
  EVE_cmd_text(X_R_STATUS_X + 8, X_R_STATUS_Y + 0, 28, 0, "TX");

  // Draw "RF GAIN" in spectrum window.
  EVE_cmd_text(SPECTRUM_LEFT_X + 61, SPECTRUM_TOP_Y + 2, 26, 0, "RF GAIN");

  // Draw "Watts" above on right side of spectrum window.
  EVE_color_rgb(0xff0000);
  EVE_cmd_text(450, 80, 26, 0, "Watts");

  EVE_cmd_endlist();
} // End of transmitter static.

// Primary transmitter screen (dynamic);
void EVE_Display::drawTransmitterScreen()
{
  std::string mode{""};
  std::string sideband{""};

  EVE_start_cmd_burst();
  EVE_cmd_dl_burst(CMD_DLSTART);
  EVE_cmd_dl_burst(DL_CLEAR_COLOR_RGB | 0x000000);
  EVE_cmd_dl_burst(DL_CLEAR | CLR_COL | CLR_STN | CLR_TAG);
  EVE_vertex_format_burst(0);
  EVE_point_size_burst(32);

  // Static items.
  EVE_cmd_calllist_burst(80000);

  // Main frequency, VFO A and VFO B.  Should skip this and simply show TxRxFreq???
  if (ConfigData.activeVFO == VFO_A)
  {
    if (ConfigData.currentFreqA < bands.bands[ConfigData.currentBandA].fBandLow or ConfigData.currentFreqA > bands.bands[ConfigData.currentBandA].fBandHigh)
    {
      EVE_color_rgb_burst(0xFF0000); // Red
    }
    else
    {
      EVE_color_rgb_burst(0x0000FF); // Blue
    }
    EVE_cmd_number_burst(0, 37, 31, 0, ConfigData.currentFreqA);   // x, y, font, options, n
    EVE_color_rgb_burst(0x7d898b);                                 // VFOB gray color (not active);
    EVE_cmd_number_burst(300, 47, 30, 0, ConfigData.currentFreqB); // VFOB
  }
  if (ConfigData.activeVFO == VFO_B)
  {
    if (ConfigData.currentFreqB < bands.bands[ConfigData.currentBandB].fBandLow or ConfigData.currentFreqB > bands.bands[ConfigData.currentBandB].fBandHigh)
    {
      EVE_color_rgb_burst(0xFF0000); // Red
    }
    else
    {
      EVE_color_rgb_burst(0x0000FF); // Blue
    }
    EVE_cmd_number_burst(300, 37, 31, 0, ConfigData.currentFreqB); // VFOB
    EVE_color_rgb_burst(0x7d898b);                                 // VFOA gray color (not active);
    EVE_cmd_number_burst(0, 47, 30, 0, ConfigData.currentFreqA);   // VFOA
  }

  // Compressor status.
  std::string compressor{"Off"};
  if (ConfigData.compressorFlag == true)
  {
    compressor = "On";
    EVE_cmd_number_burst(660, 391, 27, 0, ConfigData.micCompRatio);
  }
  EVE_cmd_text_burst(630, 391, 27, 0, compressor.c_str());

  // Key type and keyer L/R and WPM.
  std::string keyType{""};
  if (ConfigData.keyType == 0)
    keyType = "Straight Key";
  if (ConfigData.keyType == 1)
  {
    keyType = "Paddles";
    if (ConfigData.paddleFlip == 0)
    {
      EVE_cmd_text_burst(705, 411, 27, 0, "R");
    }
    else
    {
      EVE_cmd_text_burst(705, 411, 27, 0, "L");
    }
    EVE_cmd_number_burst(725, 411, 27, 0, ConfigData.currentWPM);
  }
  EVE_cmd_text_burst(630, 411, 27, 0, keyType.c_str());

  // On Band Information line at top of RF spectrum.
  EVE_cmd_number_burst(90, 80, 26, 0, ConfigData.centerFreq);
  EVE_cmd_text_burst(160, 80, 26, 0, bands.bands[ConfigData.currentBandA].name);
  // Which mode?
  if (bands.bands[ConfigData.currentBand].mode == RadioMode::CW_MODE)
    mode = "CW";
  if (bands.bands[ConfigData.currentBand].mode == RadioMode::SSB_MODE)
  {
    if (ConfigData.cessb == false)
      mode = "SSB";
    else
      mode = "CESSB";
  }
  if (bands.bands[ConfigData.currentBand].mode == RadioMode::FT8_MODE)
  {
    mode = "FT8";
    // Green if enabled, red if disabled.
    if (ft8EnableFlag)
      EVE_color_rgb_burst(0x00FF00);
    else
      EVE_color_rgb_burst(0xFF0000);
  }

  EVE_cmd_text_burst(240, 80, 26, 0, mode.c_str());
  EVE_color_rgb_burst(0x0000FF);
  // Which sideband?
  switch (bands.bands[ConfigData.currentBand].sideband)
  {
  case Sideband::LOWER:
    sideband = "LSB";
    break;
  case Sideband::UPPER:
    sideband = "USB";
    break;
  case Sideband::BOTH_AM:
    sideband = "DSB";
    break;
  case Sideband::BOTH_SAM:
    sideband = " DSB";
    break;
  default:
    break;
  }
  EVE_cmd_text_burst(288, 80, 26, 0, sideband.c_str());

  // S-Meter bar.  This is the moving red bar.
  EVE_begin_burst(EVE_RECTS);
  EVE_color_rgb_burst(0xff0000);
  EVE_vertex2f_burst(SMETER_X + 1, SMETER_Y + 2);
  EVE_vertex2f_burst(SMETER_X + 1 + display.smeterPad, SMETER_Y + 2 + SMETER_BAR_HEIGHT - 2);
  EVE_end_burst();
  // Draw dBm level.
  //  snprintf(buffer, 10, "%4.1f", display.dbm);
  dtostrf(display.dbm, 4, 1, buffer);
  EVE_cmd_text_burst(SMETER_X + 185, SMETER_Y + 3, 26, 0, buffer);

  // Draw TX power setting above on right side of spectrum window.
  EVE_color_rgb_burst(0xff0000);
  EVE_cmd_number_burst(430, 80, 26, 0, ConfigData.transmitPowerLevel);

  // Receive and transmit equalizer indicators.

  if (ConfigData.receiveEQFlag)
  {
    EVE_color_rgb_burst(0x00ff00);
    EVE_cmd_text_burst(660, 451, 27, 0, "On");
  }
  else
  {
    EVE_color_rgb_burst(0xff0000);
    EVE_cmd_text_burst(655, 451, 27, 0, "Off");
  }
  if (ConfigData.xmitEQFlag)
  {
    EVE_color_rgb_burst(0x00ff00);
    EVE_cmd_text_burst(715, 451, 27, 0, "On");
  }
  else
  {
    EVE_color_rgb_burst(0xff0000);
    EVE_cmd_text_burst(715, 451, 27, 0, "Off");
  }

  // Show the time.
  EVE_color_rgb_burst(0x5d82f3);
  EVE_cmd_text_burst(510, 38, 30, 0, display.timeBuffer);

// Show temperature and load.
#ifdef TEMP_AND_LOAD
  EVE_color_rgb_burst(0xFF0000);
  dtostrf(display.CPU_temperature, 4, 1, buffer);
  EVE_cmd_text_burst(5, 5, 27, 0, "CPU Temperature =");
  EVE_cmd_text_burst(160, 5, 27, 0, buffer);

  dtostrf(display.processor_load, 4, 1, buffer);
  EVE_cmd_text_burst(210, 5, 27, 0, "CPU Load =");
  EVE_cmd_text_burst(301, 5, 27, 0, buffer);
#endif

// IMD Test in SSB menu.
  if (radioState == RadioState::SSB_IM3TEST_STATE)
  {
    EVE_cmd_text_burst(50, 10, 29, 0, "IMD Adjust Level:");
    EVE_cmd_number_burst(360, 10, 29, 0, menuProc.imdAmplitudedB);
  }

  EVE_display_burst();
  EVE_cmd_swap_burst();
  EVE_end_cmd_burst();
  while (EVE_busy())
    ;
} // End transmitter dynamic.

// Introductory display.
FLASHMEM void EVE_Display::drawTransmitterAlarmScreen(std::string warningMessage)
{
//  EVE_start_cmd_burst();
  EVE_cmd_dl(CMD_DLSTART);
  EVE_cmd_dl(DL_CLEAR_COLOR_RGB | 0xFFFFFF);
  EVE_cmd_dl(DL_CLEAR | CLR_COL | CLR_STN | CLR_TAG);
  EVE_color_rgb(0xFF0000);
  EVE_cmd_text(80, 100, 31, 0, warningMessage.c_str());
  EVE_cmd_text(80, 150, 31, 0, "Correct Problem and restart radio.");

  EVE_cmd_dl(DL_DISPLAY);
  EVE_cmd_dl(CMD_SWAP);
//  EVE_end_cmd_burst();
  while (EVE_busy())
    ;
}

void EVE_Display::Example1() {
EVE_cmd_dl(CMD_DLSTART); // tells EVE to start a new display-list
delay(1);
EVE_cmd_dl(DL_CLEAR_COLOR_RGB | WHITE); // sets the background color
delay(1);
EVE_cmd_dl(DL_CLEAR | CLR_COL | CLR_STN | CLR_TAG);
delay(1);
EVE_color_rgb(BLACK);
delay(1);
EVE_cmd_text(5, 15, 28, 0, "Hello there!");
delay(1);
EVE_cmd_dl(DL_DISPLAY); // put in the display list to mark its end
delay(1);
EVE_cmd_dl(CMD_SWAP); // tell EVE to use the new display list
delay(1);
while (EVE_busy());
}

void EVE_Display::Example2() {
EVE_cmd_dl(CMD_DLSTART); // tells EVE to start a new display-list
delay(1);
EVE_cmd_dl(DL_CLEAR_COLOR_RGB | WHITE); // sets the background color
delay(1);
EVE_cmd_dl(DL_CLEAR | CLR_COL | CLR_STN | CLR_TAG);
delay(1);
EVE_color_rgb(BLACK);
delay(1);
EVE_cmd_text(5, 15, 28, 0, "Hola aqui!");
delay(1);
EVE_cmd_dl(DL_DISPLAY); // put in the display list to mark its end
delay(1);
EVE_cmd_dl(CMD_SWAP); // tell EVE to use the new display list
while (EVE_busy());
}