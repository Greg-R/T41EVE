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

// EEPROM class

#pragma once

const int MAX_FAVORITES = 13;  // Max number of favorite frequencies stored in EEPROM

class Eeprom {
public:

  void ConfigDataWrite();
  void ConfigDataRead();
  void ConfigDataRead(config_t &configStruct);
  void ConfigDataWriteSize(uint32_t structSize);
  void CalDataWrite();
  void CalDataRead();
  void CalDataRead(calibration_t &calStruct);
  void CalDataWriteSize(uint32_t structSize);
  void BandsWrite();
  void BandsRead();
  void BandsWriteSize(uint32_t structSize);

  int EEPROMReadSize(uint32_t address);
  void EEPROMStuffFavorites(uint32_t current[]);
//  void SetFavoriteFrequency();  // Get and Set favorite frequency temporarily removed.
//  void GetFavoriteFrequency();
  void CalDataDefaults();
  void ConfigDataDefaults();
  void EEPROMStartup();

  // SD card
  int InitializeSDCard();
  int SDPresentCheck();

private:
  const uint32_t EEPROM_BASE_ADDRESS = 0;
  const uint32_t CAL_BASE_ADDRESS = 1024;
  const uint32_t BANDS_BASE_ADDRESS = 2048;
};