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

// Receive DSP.  ProcessIQData is the primary DSP function of the receiver.

#include "SDT.h"


/*****
  Purpose: Read audio from Teensy Audio Library
             Calculate FFT for display
             Process audio into SSB signal
             Output audio to amplifier

   Parameter List:
      void

   Return value:
      bool true if audio blocks were processed.
 *****/
bool ReceiveDSP::ProcessIQData() {
  /**********************************************************************************  AFP 12-31-20
        Get samples from queue buffers
        Teensy Audio Library stores ADC data in two buffers size=128, Q_in_L and Q_in_R as initiated from the audio lib.
        Then the buffers are read into two arrays sp_L and sp_R in blocks of 128 up to N_BLOCKS.  The arrarys are
        of size BUFFER_SIZE * N_BLOCKS.  BUFFER_SIZE is 128.
        N_BLOCKS = FFT_LENGTH / 2 / BUFFER_SIZE * (uint32_t)DF; // should be 16 with DF == 8 and FFT_LENGTH = 512
        BUFFER_SIZE*N_BLOCKS = 2024 samples
     **********************************************************************************/
  float32_t audioMaxSquared;
  uint32_t AudioMaxIndex;
  float rfGainValue;
  int rfGain;

  // Are there at least N_BLOCKS buffers in each channel available ?  N_BLOCKS should be 16.  Fill float_buffer_L/R[2048].
//  if (static_cast<uint32_t>(ADC_RX_I.available()) > 15 && static_cast<uint32_t>(ADC_RX_Q.available()) > 15) {
    usec = 0;
    // Get audio samples from the audio  buffers and convert them to float.
    // Read in 16 blocks and 128 samples in I and Q.  16 * 128 = 2048  (N_BLOCKS = 16)
    for (unsigned i = 0; i < N_BLOCKS; i++) {
      /**********************************************************************************  AFP 12-31-20
          Using arm_Math library, convert to float one buffer_size.
          Float_buffer samples are now standardized from > -1.0 to < 1.0
      **********************************************************************************/
      arm_q15_to_float(ADC_RX_Q.readBuffer(), &float_buffer_L[BUFFER_SIZE * i], BUFFER_SIZE);  // convert int_buffer to float 32bit.  BUFFER_SIZE = 128.
      arm_q15_to_float(ADC_RX_I.readBuffer(), &float_buffer_R[BUFFER_SIZE * i], BUFFER_SIZE);  // convert int_buffer to float 32bit
      ADC_RX_I.freeBuffer();
      ADC_RX_Q.freeBuffer();
    }  // end for loop

    // Set frequency here only to minimize interruption to signal stream during tuning.
    // This code was unnecessary in the revised tuning scheme.  KF5N July 22, 2023
////    if (centerTuneFlag == 1) {  //  This flag is set by EncoderFineTune() and also by Direct Freq Entry.
////      display.DrawBandWidthIndicatorBar();
////      display.ShowFrequency();
////    }                    //AFP 10-04-22
////    centerTuneFlag = 0;  //AFP 10-04-22
    if (resetTuningFlag == 1) {
      ResetTuning();
    }
    resetTuningFlag = 0;

    // What is the correct RF gain value for maximum dynamic range?

    //  Set RFGain for all bands.
////    if (ConfigData.autoGain) rfGain = ConfigData.rfGainCurrent - 25;  // Auto-gain.  Note the constant must be the same as below!
    rfGain = ConfigData.rfGain[ConfigData.currentBand] - 25;     // Manual gain adjust.  The constant is a critical determinant of dynamic range.
    rfGainValue = pow(10, static_cast<float32_t>(rfGain) / 20.0);     // DSPGAINSCALE removed in T41EEE.9.  Greg KF5N February 24, 2024

    rfGainValue = rfGainValue * audioGainCompensate;

    arm_scale_f32(float_buffer_L, rfGainValue, float_buffer_L, BUFFER_SIZE * N_BLOCKS);  //AFP 09-27-22
    arm_scale_f32(float_buffer_R, rfGainValue, float_buffer_R, BUFFER_SIZE * N_BLOCKS);  //AFP 09-27-22

    /**********************************************************************************  AFP 12-31-20
        Remove DC offset to reduce central spike.  First read the Mean value of
        left and right channels.  Then fill L and R correction arrays with those Means
        and subtract the Means from the float L and R buffer data arrays.  Again use Arm_Math functions
        to manipulate the arrays.  Arrays are all BUFFER_SIZE * N_BLOCKS long
    **********************************************************************************/

    /*    arm_mean_f32(float_buffer_L, BUFFER_SIZE * N_BLOCKS, &sample_meanL);
    arm_mean_f32(float_buffer_R, BUFFER_SIZE * N_BLOCKS, &sample_meanR);

    for (uint32_t j = 0; j < BUFFER_SIZE * N_BLOCKS  ; j++) {
      L_BufferOffset [j] = -sample_meanL;
      R_BufferOffset [j] = -sample_meanR;
    }
    arm_add_f32(float_buffer_L , L_BufferOffset, float_buffer_L2 , BUFFER_SIZE * N_BLOCKS ) ;
    arm_add_f32(float_buffer_R , R_BufferOffset, float_buffer_R2 , BUFFER_SIZE * N_BLOCKS ) ;

    arm_biquad_cascade_df2T_f32(&s1_Receive2, float_buffer_L, float_buffer_L, 2048);  //AFP 11-03-22
    arm_biquad_cascade_df2T_f32(&s1_Receive2, float_buffer_R, float_buffer_R, 2048);  //AFP 11-03-22
    */

    /**********************************************************************************  AFP 12-31-20
      Clear Buffers
      This is to prevent overfilled queue buffers during each switching event.
      (band change, mode change, frequency change, the audio chain runs and fills the buffers.
      If the buffers are full, the Teensy needs much more time.
      In that case, we clear the buffers to keep the whole audio chain running smoothly.
      **********************************************************************************/
        if (ADC_RX_I.available() > 50) {
          ADC_RX_I.clear();
         ADC_RX_Q.clear();
        }

    /**********************************************************************************  AFP 12-31-20
      IQ amplitude and phase correction.  For this scaled down version the I an Q channels are
      equalized and phase corrected manually. This is done by applying a correction, which is the difference, to
      the L channel only.  The phase is corrected in the IQPhaseCorrection() function.
    ***********************************************************************************************/

    // IQ amplitude and phase correction
    // Correct using CW calibration if using CW or FT8.
    if (radioState == RadioState::CW_RECEIVE_STATE or radioState == RadioState::FT8_RECEIVE_STATE) {
      if (bands.bands[ConfigData.currentBand].sideband == Sideband::LOWER) {
        arm_scale_f32(float_buffer_L, CalData.IQCWRXAmpCorrectionFactorLSB[ConfigData.currentBand], float_buffer_L, BUFFER_SIZE * N_BLOCKS);  //AFP 04-14-22
        IQPhaseCorrection(float_buffer_L, float_buffer_R, CalData.IQCWRXPhaseCorrectionFactorLSB[ConfigData.currentBand], BUFFER_SIZE * N_BLOCKS);
      } else {
        if (bands.bands[ConfigData.currentBand].sideband == Sideband::UPPER) {
          arm_scale_f32(float_buffer_L, CalData.IQCWRXAmpCorrectionFactorUSB[ConfigData.currentBand], float_buffer_L, BUFFER_SIZE * N_BLOCKS);  //AFP 04-14-22
          IQPhaseCorrection(float_buffer_L, float_buffer_R, CalData.IQCWRXPhaseCorrectionFactorUSB[ConfigData.currentBand], BUFFER_SIZE * N_BLOCKS);
        }
      }
      // Correct using SSB calibration if using SSB or AM demodulation modes.
    } else if (radioState == RadioState::SSB_RECEIVE_STATE or radioState == RadioState::AM_RECEIVE_STATE or radioState == RadioState::SAM_RECEIVE_STATE) {
      if (bands.bands[ConfigData.currentBand].sideband == Sideband::LOWER or bands.bands[ConfigData.currentBand].sideband == Sideband::BOTH_AM or bands.bands[ConfigData.currentBand].sideband == Sideband::BOTH_SAM) {
        arm_scale_f32(float_buffer_L, CalData.IQSSBRXAmpCorrectionFactorLSB[ConfigData.currentBand], float_buffer_L, BUFFER_SIZE * N_BLOCKS);  //AFP 04-14-22
        IQPhaseCorrection(float_buffer_L, float_buffer_R, CalData.IQSSBRXPhaseCorrectionFactorLSB[ConfigData.currentBand], BUFFER_SIZE * N_BLOCKS);
      } else {
        if (bands.bands[ConfigData.currentBand].sideband == Sideband::UPPER or bands.bands[ConfigData.currentBand].sideband == Sideband::BOTH_AM or bands.bands[ConfigData.currentBand].sideband == Sideband::BOTH_SAM) {
          arm_scale_f32(float_buffer_L, CalData.IQSSBRXAmpCorrectionFactorUSB[ConfigData.currentBand], float_buffer_L, BUFFER_SIZE * N_BLOCKS);  //AFP 04-14-22
          IQPhaseCorrection(float_buffer_L, float_buffer_R, CalData.IQSSBRXPhaseCorrectionFactorUSB[ConfigData.currentBand], BUFFER_SIZE * N_BLOCKS);
        }
      }
    }

    /**********************************************************************************  AFP 12-31-20
        Frequency translation by Fs/4 without multiplication from Lyons (2011): chapter 13.1.2 page 646
        together with the savings of not having to shift/rotate the FFT_buffer, this saves
        about 1% of processor use.

        This is for +Fs/4 [moves receive frequency to the left in the spectrum display]
           float_buffer_L contains I = real values
           float_buffer_R contains Q = imaginary values
           xnew(0) =  xreal(0) + jximag(0)
               leave first value (DC component) as it is!
           xnew(1) =  - ximag(1) + jxreal(1)
    **********************************************************************************/
    // X1 zoom must be done before the frequency shift!
    if ((ConfigData.spectrum_zoom == 0) and updateDisplayFlag) {
      CalcZoom1Magn();
    }

    FreqShift1();

    /**********************************************************************************  AFP 12-31-20
        ConfigData.spectrum_zoom_2 and larger here after frequency conversion!
        Spectrum zoom displays a magnified display of the data around the translated receive frequency.
        Processing is done in the ZoomFFTExe(BUFFER_SIZE * N_BLOCKS) function.  For magnifications of 2x to 16X
        Larger magnifications are possible.

        Spectrum Zoom uses the shifted spectrum, so the center "hump" around DC is shifted by fs/4
    **********************************************************************************/
    // Zooms 2 and 4.  These need only 16 blocks, so can be done in a single pass.
    if (updateDisplayFlag and (ConfigData.spectrum_zoom == 1 or ConfigData.spectrum_zoom == 2)) {
      ZoomFFTExe(BUFFER_SIZE * N_BLOCKS);
    }
    // Zooms 8 and 16.  These have to be called repeatedly to accumulate data.
    if (ConfigData.spectrum_zoom == 3 or ConfigData.spectrum_zoom == 4) {
      ZoomFFTExe(BUFFER_SIZE * N_BLOCKS);
    }

//    if (calibrateFlag == true) {  // This is required for frequency calibration as it runs with the receiver active.
//      menuProc.CalibrateOptions();
//    }
    // This handles functions selected from the second level of menus.
    if (evemenucontrol.runInDSP == true) {  // This is required for Morse decode sensitivity adjustment with receiver active.
      functionPtr[mainMenuIndex]();  // The top menu item to run.
    }

    /*************************************************************************************************
        freq_conv2()

        FREQUENCY CONVERSION USING A SOFTWARE QUADRATURE OSCILLATOR
        Creates a new IF frequency to allow the tuning window to be moved anywhere in the current display.
        THIS VERSION calculates the COS AND SIN WAVE on the fly - uses double precision float

        MAJOR ADVANTAGE: frequency conversion can be done for any frequency !

        large parts of the code taken from the mcHF code by Clint, KA7OEI, thank you!
          see here for more info on quadrature oscillators:
        Wheatley, M. (2011): CuteSDR Technical Manual Ver. 1.01. - http://sourceforge.net/projects/cutesdr/
        Lyons, R.G. (2011): Understanding Digital Processing. – Pearson, 3rd edition.
     *************************************************************************************************/

    FreqShift2();  //AFP 12-14-21

    /**********************************************************************************  AFP 12-31-20
        Decimation
        Resample (Decimate) the shifted time signal, first by 4, then by 2.  Each time the
        signal is decimated by an even number, the spectrum is reversed.  Resampling twice
        returns the spectrum to the correct orientation.
        Signal has now been shifted to base band, leaving aliases at higher frequencies,
        which are removed at each decimation step using the Arm combined decimate/filter function.
        If the starting sample rate is 192K SPS after the combined decimation, the sample rate is
        now 192K/8 = 24K SPS.  The array size is also reduced by 8, making FFT calculations much faster.
        The effective bandwidth (up to Nyquist frequency) is 12KHz.
     **********************************************************************************/
    // decimation-by-4 in-place!
    arm_fir_decimate_f32(&FIR_dec1_I, float_buffer_L, float_buffer_L, BUFFER_SIZE * N_BLOCKS);
    arm_fir_decimate_f32(&FIR_dec1_Q, float_buffer_R, float_buffer_R, BUFFER_SIZE * N_BLOCKS);

    // decimation-by-2 in-place
    arm_fir_decimate_f32(&FIR_dec2_I, float_buffer_L, float_buffer_L, BUFFER_SIZE * N_BLOCKS / (uint32_t)DF1);
    arm_fir_decimate_f32(&FIR_dec2_Q, float_buffer_R, float_buffer_R, BUFFER_SIZE * N_BLOCKS / (uint32_t)DF1);

    /**********************************************************************************  AFP 12-31-20
        Digital FFT convolution
        Filtering is accomplished by combining (multiplying) spectra in the frequency domain.
         Basis for this was Lyons, R. (2011): Understanding Digital Processing.
         "Fast FIR Filtering using the FFT", pages 688 - 694.
         Method used here: overlap-and-save.

        First, Create Complex time signal for CFFT routine.
        Fill first block with Zeros.
        Then interleave RE and IM parts to create signal for FFT.
     **********************************************************************************/
    // Prepare the audio signal buffers:
    // ONLY FOR the VERY FIRST FFT: fill first samples with zeros

    if (first_block) {  // Fill real & imaginaries with zeros for the first BLOCKSIZE samples.
      for (unsigned i = 0; i < BUFFER_SIZE * N_BLOCKS / (uint32_t)(DF / 2.0); i++) {
        FFT_buffer[i] = 0.0;
      }
      first_block = 0;
    } else  // All other FFTs

      // Fill FFT_buffer with last events audio samples for all other FFT instances.
      for (unsigned i = 0; i < BUFFER_SIZE * N_BLOCKS / (uint32_t)(DF); i++) {
        FFT_buffer[i * 2] = last_sample_buffer_L[i];      // real
        FFT_buffer[i * 2 + 1] = last_sample_buffer_R[i];  // imaginary
      }

    for (unsigned i = 0; i < BUFFER_SIZE * N_BLOCKS / (uint32_t)(DF); i++) {  // Copy recent samples to last_sample_buffer for next time!
      last_sample_buffer_L[i] = float_buffer_L[i];
      last_sample_buffer_R[i] = float_buffer_R[i];
    }

    //------------------------------ now fill recent audio samples into FFT_buffer (left channel: re, right channel: im)
    for (unsigned i = 0; i < BUFFER_SIZE * N_BLOCKS / (uint32_t)(DF); i++) {
      FFT_buffer[FFT_length + i * 2] = float_buffer_L[i];      // real
      FFT_buffer[FFT_length + i * 2 + 1] = float_buffer_R[i];  // imaginary
    }

    /**********************************************************************************  AFP 12-31-20
       Perform complex FFT on the audio time signals
       calculation is performed in-place the FFT_buffer [re, im, re, im, re, im . . .]
     **********************************************************************************/
    arm_cfft_f32(S, FFT_buffer, 0, 1);

    /**********************************************************************************  AFP 12-31-20
      Continuing FFT Convolution
          Next, prepare the filter mask (done in the Filter.cpp file).  Only need to do this once for each filter setting.
          Allows efficient real-time variable LP and HP audio filters, without the overhead of time-domain convolution filtering.

          After the Filter mask in the frequency domain is created, complex multiply  filter mask with the frequency domain audio data.
          Filter mask previously calculated in setup Array of filter mask coefficients:
          FIR_filter_mask[]
     **********************************************************************************/

    arm_cmplx_mult_cmplx_f32(FFT_buffer, FIR_filter_mask, iFFT_buffer, FFT_length);

    // Create audio spectrum.
    if (updateDisplayFlag == true) {
      for (int k = 0; k < 1024; k++) {
        audioSpectBuffer[1023 - k] = (iFFT_buffer[k] * iFFT_buffer[k]);  // iFFT_buffer[1025]?
      }
      for (int k = 0; k < 256; k++) {
//        audioYPixelold[k] = audioYPixelcurrent[k];  // Store the existing audio spectrum so it can be erased.
        if (bands.bands[ConfigData.currentBand].sideband == Sideband::UPPER || bands.bands[ConfigData.currentBand].sideband == Sideband::BOTH_AM || bands.bands[ConfigData.currentBand].sideband == Sideband::BOTH_SAM) {
          //       audioYPixel[k] = 65 + map(15 * log10f((audioSpectBuffer[1023 - k] + audioSpectBuffer[1023 - k + 1] + audioSpectBuffer[1023 - k + 2]) / 3.0), 0, 100, 0, 120) + audioFFToffset;
          audioYPixel[k] = 65 + static_cast<int16_t>(15.0 * log10f(audioSpectBuffer[1023 - k])) + audioFFToffset;  // Simplified for T41EEE.91.
        } else if (bands.bands[ConfigData.currentBand].sideband == Sideband::LOWER) {
          //       audioYPixel[k] = 65 + map(15 * log10f((audioSpectBuffer[k] + audioSpectBuffer[k + 1] + audioSpectBuffer[k + 2]) / 3.0), 0, 100, 0, 120) + audioFFToffset;
          audioYPixel[k] = 65 + static_cast<int16_t>(15.0 * log10f(audioSpectBuffer[k])) + audioFFToffset;  // Simplified for T41EEE.91.
        }
        if (audioYPixel[k] < 0)
          audioYPixel[k] = 0;
      }
      arm_max_f32(audioSpectBuffer, 1024, &audioMaxSquared, &AudioMaxIndex);  // AFP 09-18-22 Max value of squared abin magnitude in audio
      audioMaxSquaredAve = .5 * audioMaxSquared + .5 * audioMaxSquaredAve;    // AFP 09-18-22 Running averaged values
      display.DisplaydbM();
      updateDisplayFlag = false;
    }

    /**********************************************************************************
          Additional Convolution Processes:
              // filter by just deleting bins - principle of Linrad
      only works properly when we have the right window function!

        (automatic) notch filter = Tone killer --> the name is stolen from SNR ;-)
        first test, we set a notch filter at 1kHz
        which bin is that?
        positive & negative frequency -1kHz and +1kHz --> delete 2 bins
        we are not deleting one bin, but five bins for the test
        1024 bins in 12ksps = 11.71Hz per bin
        SR[SampleRate].rate / 8.0 / 1024 = bin BW
        1000Hz / 11.71Hz = bin 85.333

     **********************************************************************************/

    /**********************************************************************************  AFP 12-31-20
      After the frequency domain filter mask and other processes are complete, do a
      complex inverse FFT to return to the time domain
        (if sample rate = 192kHz, we are in 24ksps now, because we decimated by 8)
        perform iFFT (in-place)  IFFT is selected by the IFFT flag=1 in the Arm CFFT function.
     **********************************************************************************/

    arm_cfft_f32(iS, iFFT_buffer, 1, 1);

    // Adjust for level alteration because of filters.
    //  Need to scale iFFT_buffer[] here.

    for (unsigned i = 0; i < FFT_length / 2; i++) {
      iFFT_buffer[FFT_length + 2 * i + 0] = RFGAINSCALE * iFFT_buffer[FFT_length + 2 * i + 0];
      iFFT_buffer[FFT_length + 2 * i + 1] = RFGAINSCALE * iFFT_buffer[FFT_length + 2 * i + 1];
    }

    /**********************************************************************************
          Demodulation
            our time domain output is a combination of the real part (left channel) AND the imaginary part (right channel) of the second half of the FFT_buffer
            The demod mode is accomplished by selecting/combining the real and imaginary parts of the output of the IFFT process.
       **********************************************************************************/
    //===================== AFP 10-27-22  =========

    switch (bands.bands[ConfigData.currentBand].sideband) {
      case Sideband::LOWER:
        for (unsigned i = 0; i < FFT_length / 2; i++) {
          //if (bands.bands[ConfigData.currentBand].mode == DEMOD_USB || bands.bands[ConfigData.currentBand].mode == DEMOD_LSB ) {  // for SSB copy real part in both outputs
          float_buffer_L[i] = iFFT_buffer[FFT_length + (i * 2)];
          float_buffer_R[i] = float_buffer_L[i];
          //}
        }
        break;
      case Sideband::UPPER:
        for (unsigned i = 0; i < FFT_length / 2; i++) {
          float_buffer_L[i] = iFFT_buffer[FFT_length + (i * 2)];
          float_buffer_R[i] = float_buffer_L[i];
          audiotmp = AlphaBetaMag(iFFT_buffer[FFT_length + (i * 2)], iFFT_buffer[FFT_length + (i * 2) + 1]);
        }
        break;
      case Sideband::BOTH_AM:
        for (unsigned i = 0; i < FFT_length / 2; i++) {  // Magnitude estimation Lyons (2011): page 652 / libcsdr
          audiotmp = AlphaBetaMag(iFFT_buffer[FFT_length + (i * 2)], iFFT_buffer[FFT_length + (i * 2) + 1]);
          // DC removal filter -----------------------
          w = audiotmp + wold * 0.99f;  // Response to below 200Hz AFP 10-30-22
          float_buffer_L[i] = w - wold;
          wold = w;
        }
        arm_biquad_cascade_df1_f32(&biquad_lowpass1, float_buffer_L, float_buffer_R, FFT_length / 2);
        arm_copy_f32(float_buffer_R, float_buffer_L, FFT_length / 2);
        break;
      case Sideband::BOTH_SAM:  //AFP 11-03-22
        demod.AMDecodeSAM();
        break;
      default:
        break;
    }


    //============================  Receive EQ  ========================  AFP 08-08-22
    if (ConfigData.receiveEQFlag) {
      DoReceiveEQ();
      arm_copy_f32(float_buffer_L, float_buffer_R, FFT_length / 2);
    }
    //============================ End Receive EQ

    /**********************************************************************************
      Noise Reduction
      3 algorithms working 3-15-22
      NR_Kim
      Spectral NR
      LMS variable leak NR
    **********************************************************************************/
    switch (ConfigData.nrOptionSelect) {
      case 0:  // NR Off
        break;
      case 1:  // Kim NR
        Kim1_NR();
        arm_scale_f32(float_buffer_L, 2.0, float_buffer_L, FFT_length / 2);  // Scaling factor reduced; was blasting speaker.  KF5N February 20, 2024.
        arm_scale_f32(float_buffer_R, 2.0, float_buffer_R, FFT_length / 2);
        break;
      case 2:  // Spectral NR
        SpectralNoiseReduction();
        arm_scale_f32(float_buffer_L, 2.0, float_buffer_L, FFT_length / 2);  // Scaling factor reduced; was blasting speaker.  KF5N February 20, 2024.
        arm_scale_f32(float_buffer_R, 2.0, float_buffer_R, FFT_length / 2);
        break;
      case 3:  // LMS NR.  KF5N March 2, 2024.
        Xanr();
        //        arm_scale_f32 (float_buffer_L, 1.5, float_buffer_L, FFT_length / 2);  // Why is scaling different???
        arm_scale_f32(float_buffer_R, 4.0, float_buffer_R, FFT_length / 2);  // Attempt to equalize gains for all NR algorithms.  Greg KF5N June 24, 2024.
        arm_copy_f32(float_buffer_R, float_buffer_L, FFT_length / 2);        //  This is apparently required by the algorithm; it works on right channel only.
        break;
    }
    //==================  End NR ============================
    // ===========================Automatic Notch ==================
    if (ConfigData.ANR_notch) {  // KF5N March 2, 2024.
      Xanr();
      arm_copy_f32(float_buffer_R, float_buffer_L, FFT_length / 2);  //  This is apparently required by the algorithm; it works on right channel only.
    }
    // ====================End notch =================================
    /**********************************************************************************
      EXPERIMENTAL: noise blanker
      by Michael Wild
    **********************************************************************************
    if (NB_on != 0) {
      NoiseBlanker(float_buffer_L, float_buffer_R);
      arm_copy_f32(float_buffer_R, float_buffer_L, FFT_length / 2);
    }
*/

    if (bands.bands[ConfigData.currentBand].mode == RadioMode::CW_MODE) {
    if (ConfigData.decoderFlag) cwprocess.DoCWReceiveProcessing();  // Run Morse decoder.

      // ----------------------  CW Narrow band filters  AFP 10-18-22 -------------------------
      if (ConfigData.CWFilterIndex != 5) {
        switch (ConfigData.CWFilterIndex) {
          case 0:                                                                                           // 0.8 KHz
            arm_biquad_cascade_df2T_f32(&S1_CW_AudioFilter1, float_buffer_L, float_buffer_L_AudioCW, 256);  //AFP 10-18-22
            arm_copy_f32(float_buffer_L_AudioCW, float_buffer_L, FFT_length / 2);                           //AFP 10-18-22
            arm_copy_f32(float_buffer_L_AudioCW, float_buffer_R, FFT_length / 2);
            break;
          case 1:                                                                                           // 1.0 KHz
            arm_biquad_cascade_df2T_f32(&S1_CW_AudioFilter2, float_buffer_L, float_buffer_L_AudioCW, 256);  //AFP 10-18-22
            arm_copy_f32(float_buffer_L_AudioCW, float_buffer_L, FFT_length / 2);                           //AFP 10-18-22
            arm_copy_f32(float_buffer_L_AudioCW, float_buffer_R, FFT_length / 2);
            break;
          case 2:                                                                                           // 1.3 KHz
            arm_biquad_cascade_df2T_f32(&S1_CW_AudioFilter3, float_buffer_L, float_buffer_L_AudioCW, 256);  //AFP 10-18-22
            arm_copy_f32(float_buffer_L_AudioCW, float_buffer_L, FFT_length / 2);                           //AFP 10-18-22
            arm_copy_f32(float_buffer_L_AudioCW, float_buffer_R, FFT_length / 2);
            break;
          case 3:                                                                                           // 1.8 KHz
            arm_biquad_cascade_df2T_f32(&S1_CW_AudioFilter4, float_buffer_L, float_buffer_L_AudioCW, 256);  //AFP 10-18-22
            arm_copy_f32(float_buffer_L_AudioCW, float_buffer_L, FFT_length / 2);                           //AFP 10-18-22
            arm_copy_f32(float_buffer_L_AudioCW, float_buffer_R, FFT_length / 2);
            break;
          case 4:                                                                                           // 2.0 KHz
            arm_biquad_cascade_df2T_f32(&S1_CW_AudioFilter5, float_buffer_L, float_buffer_L_AudioCW, 256);  //AFP 10-18-22
            arm_copy_f32(float_buffer_L_AudioCW, float_buffer_L, FFT_length / 2);                           //AFP 10-18-22
            arm_copy_f32(float_buffer_L_AudioCW, float_buffer_R, FFT_length / 2);
            break;
          case 5:  //Off
            break;
        }
      }
    }

    // =================Interpolation up to 192ksps  ================
    //  Right channel audio deactivated.  KF5N March 11, 2024
    arm_fir_interpolate_f32(&FIR_int1_I, float_buffer_L, iFFT_buffer, BUFFER_SIZE * N_BLOCKS / (uint32_t)(DF));
    // interpolation-by-4
    arm_fir_interpolate_f32(&FIR_int2_I, iFFT_buffer, float_buffer_L, BUFFER_SIZE * N_BLOCKS / (uint32_t)(DF1));

    // Scale by 8 to compensate for interpolation.
    arm_scale_f32(float_buffer_L, 8.0, float_buffer_L, BUFFER_SIZE * N_BLOCKS);

    /**********************************************************************************  AFP 12-31-20
      CONVERT TO INTEGER AND PLAY AUDIO
    **********************************************************************************/
////    q15_t q15_buffer_LTemp[2048];  //KF5N
////    arm_float_to_q15(float_buffer_L, q15_buffer_LTemp, 2048);
////    Q_out_L.play(q15_buffer_LTemp, 2048);
audioOutQueue.play(float_buffer_L, 2048);

    elapsed_micros_sum = elapsed_micros_sum + usec;
    elapsed_micros_idx_t++;

    return true;  // Audio blocks were processed.
//  }               // end of if(audio blocks available)
//  return false;   // Audio blocks were NOT processed!
}


/*****
Note September 7, 2026.  This function is semi-obsolete.  It needs to be pared down!!!  Greg KF5N
This function is still used in the case of 8 and 16 zooms which require more than 2048 samples.

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
void ReceiveDSP::StreamAudioMakeSpectrums()
{
  int AudioH_max = 0, AudioH_max_box = 0; // Used to center audio spectrum.
  int audio_hist[256]{0};                 // All values are initialized to zero using this syntax.
  int k;
  int middleSlice = centerLine / 2; // Approximate center element
  int wfall{0};

  updateDisplayCounter = 0;
  updateDisplayFlag = false;

  //  Zoom is tricky.  1X, 2X, and 4X can compute FFT with 2048 samples.
  //  8X and 16X need more samples, and thus require multiple passes of the DSP code.

  // 1X zoom.
  if (ConfigData.spectrum_zoom == 0)
  {
    updateDisplayFlag = true;
  }
  // 2X zoom.
  if (ConfigData.spectrum_zoom == 1)
  {
    updateDisplayFlag = true;
  }
  // 4X zoom.
  if (ConfigData.spectrum_zoom == 2)
  {
    updateDisplayFlag = true;
  }
  // 8X zoom.
  if (ConfigData.spectrum_zoom == 3)
  {
    updateDisplayCounter = updateDisplayCounter + 1;
    if (updateDisplayCounter == 3)
    {
      updateDisplayFlag = true;
    }
  }
  // 16X zoom.
  if (ConfigData.spectrum_zoom == 4)
  {
    updateDisplayCounter = updateDisplayCounter + 1;
    if (updateDisplayCounter == 7)
    {
      updateDisplayFlag = true;
    }
  }

  if (startRxFlag)
    updateDisplayFlag = false; // Don't process data the first time after coming out of transmit mode.
  startRxFlag = false;

  // Collect a histogram of audio spectral values.  This is used to keep the audio spectrum in the viewable area.
  // 247??? is the spectral display bottom.  129 is the audio spectrum display top.
  for (int x1 = 0; x1 < 256; x1 = x1 + 1)
  {
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
      if (audioYPixel[x1] != 0)
      {
        if (audioYPixel[x1] > CLIP_AUDIO_PEAK) // audioSpectrumHeight = 118
          audioYPixel[x1] = CLIP_AUDIO_PEAK;
        if (x1 == middleSlice)
        {
          smeterLength = y_new;
        }
      }
    }
  }

  for (int x = 0; x < 512; x = x + 1)
  {
    wfall = -pixelnew[x] + 236;
    if (wfall < 0)
      wfall = 0;
    if (wfall > 116)
      wfall = 116;
    waterfall[x] = signalToRGB332(static_cast<int32_t>((static_cast<float32_t>(wfall) * 2.2)));
  }

  // Manage audio spectral display graphics.  Keep the spectrum within the viewable area.
  if (AudioH_max_box > 30)
  { // HB. Adjust rfGainAllBands 15 and 13 to alter to move target base up and down. UPPERPIXTARGET = 15
    audioFFToffset = audioFFToffset - 1;
  }
  if (AudioH_max_box < 28)
  { // LOWERPIXTARGET = 13
    audioFFToffset = audioFFToffset + 1;
  }

} // End 


uint8_t ReceiveDSP::convert_rgb565_to_rgb332(uint16_t rgb565_color)
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

uint8_t ReceiveDSP::signalToRGB332(uint8_t signal)
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
