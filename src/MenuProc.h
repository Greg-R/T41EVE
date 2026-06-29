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

// Menu Processing class.

// #include "SDT.h"

class MenuProc
{

public:
    int micChoice = 0;
    int splitOn = 0;
    int IQChoice = 0;
    int32_t subMenuChoice{0};  // This can be incremented to -1.  Must be signed.
    uint32_t columnIndex{0};
    std::string subMenuString{""};
    std::string subMenuName{""};
    std::string commandOptionsName{""};
    std::string commandName{""};
    float32_t encoderValueLive{0};
    bool selectString{false};
    float imdAmplitudedB = 5;  // This needs to be public so EVE_Display can show it during test.

#ifdef QSE2
    std::vector<std::string> IQOptions{"Freq Cal", "CW PA Cal", "CW Rec Cal", "CW Carrier Cal", "CW Xmit Cal", "SSB PA Cal",
                                       "SSB Rec Cal", "SSB Carrier Cal", "SSB Transmit Cal", "CW Radio Cal", "CW Refine Cal", "SSB Radio Cal", "SSB Refine Cal",
                                       "dBm Level Cal", "Switch Matrix Cal", "Button Repeat", "Cancel"};
#else
    std::vector<std::string> IQOptions = {"Freq Cal", "CW PA Cal", "CW Rec Cal", "CW Xmit Cal", "SSB PA Cal",
                                          "SSB Rec Cal", "SSB Transmit Cal", "CW Radio Cal", "CW Refine Cal", "SSB Radio Cal", "SSB Refine Cal",
                                          "dBm Level Cal", "Btn Cal", "Btn Repeat", "Cancel"};
#endif

    void CalibrateOptions();
    void CWOptions();
    void AGCOptions();
    void ProcessEqualizerChoices(int EQType);
    void EqualizerRecOptions();
    void EqualizerXmtOptions();
    void SSBOptions();
    void RFOptions();
    void VFOSelect();
    void ConfigDataOptions();
    void CalDataOptions();
    void SubmenuSelectString(std::vector<std::string> options);
};