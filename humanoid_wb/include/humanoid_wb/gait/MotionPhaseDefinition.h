/******************************************************************************
Copyright (c) 2023, Ioannis Dadiotis <ioannis.dadiotis@iit.it>. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#pragma once

#include <iostream>
#include <map>
#include <string>
#include <vector>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/misc/LoadData.h>

#include "humanoid_wb/common/Types.h"

namespace ocs2 {
namespace humanoid_wb {

enum ModeNumber {  // {LF - Left Foot, RF - Right Foot, LH - Left Hand, RH - Right Hand}
  FLY = 0,
  RF = 1,
  LF = 2,
  STANCE = 3,

  RH_FLY = 4,
  RH_RF = 5,
  RH_LF = 6,
  RH_STANCE = 7,

  LH_FLY = 8,
  LH_RF = 9,
  LH_LF = 10,
  LH_STANCE = 11,

  RH_LH_FLY = 12,
  RH_LH_RF = 13,
  RH_LH_LF = 14,
  RH_LH_STANCE = 15,

};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
inline contact_flag_t modeNumber2StanceLeg(const size_t& modeNumber) {
  contact_flag_t stanceLegs;  // {RF, LF}

  const int legModeOffset = 4;

  switch (modeNumber) {
    case 0: case (0 + legModeOffset): case (0 + 2 * legModeOffset): case (0 + 3 * legModeOffset):
      stanceLegs = contact_flag_t{false, false};
      break;  // 0:  0-leg-stance
    case 1: case (1 + legModeOffset): case (1 + 2 * legModeOffset): case (1 + 3 * legModeOffset):
      stanceLegs = contact_flag_t{true, false};
      break;  // 1:  RH
    case 2: case (2 + legModeOffset): case (2 + 2 * legModeOffset): case (2 + 3 * legModeOffset):
      stanceLegs = contact_flag_t{false, true};
      break;  // 2:  LH
    case 3: case (3 + legModeOffset): case (3 + 2 * legModeOffset): case (3 + 3 * legModeOffset):
      stanceLegs = contact_flag_t{true, true};
      break;  // 3: stance
  }

  return stanceLegs;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
inline arm_contact_flag_t modeNumber2StanceArm(const size_t& modeNumber) {
  arm_contact_flag_t stanceArms;  // {RH, LH}
  const int legModeOffset = 4;

  if (modeNumber < legModeOffset) {
      stanceArms = arm_contact_flag_t{false, false};
//      std::cout << "[modeNumber2StanceArm] modeNumber = " << modeNumber << ", stanceArms = " << stanceArms[0] << " " << stanceArms[1] << std::endl;
  }
  else if (legModeOffset - 1 <  modeNumber && modeNumber < 2 * legModeOffset) {
      stanceArms = arm_contact_flag_t{true, false};
//      std::cout << "[modeNumber2StanceArm] modeNumber = " << modeNumber << ", stanceArms = " << stanceArms[0] << " " << stanceArms[1] << std::endl;
  }
  else if (2 * legModeOffset - 1 < modeNumber && modeNumber < 3 * legModeOffset) {
      stanceArms = arm_contact_flag_t{false, true};
//      std::cout << "[modeNumber2StanceArm] modeNumber = " << modeNumber << ", stanceArms = " << stanceArms[0] << " " << stanceArms[1] << std::endl;
  }
  else if (3 * legModeOffset - 1 < modeNumber) {
      stanceArms = arm_contact_flag_t{true, true};
//      std::cout << "[modeNumber2StanceArm] modeNumber = " << modeNumber << ", stanceArms = " << stanceArms[0] << " " << stanceArms[1] << std::endl;
  }
  return stanceArms;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
inline locoma_contact_flag_t modeNumber2ActiveContacts(const size_t& modeNumber) {
  locoma_contact_flag_t contacts;  // {RF, LF}

  arm_contact_flag_t contactArmFlags = modeNumber2StanceArm(modeNumber);
  contact_flag_t contactLegFlags = modeNumber2StanceLeg(modeNumber);
  contacts = locoma_contact_flag_t{contactLegFlags[0], contactLegFlags[1], contactArmFlags[0], contactArmFlags[1]};
  return contacts;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
inline size_t stanceLeg2ModeNumber(const contact_flag_t& stanceLegs, const arm_contact_flag_t& stanceArms) {
  return static_cast<size_t>(stanceLegs[0]) + 2 * static_cast<size_t>(stanceLegs[1]) + 4 * static_cast<size_t>(stanceArms[0]) + 8 * static_cast<size_t>(stanceArms[1]);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
inline std::string modeNumber2String(const size_t& modeNumber) {

  // build the map from mode number to name
  std::map<size_t, std::string> modeToName;
  modeToName[FLY] = "FLY";
  modeToName[RF] = "RF";
  modeToName[LF] = "LF";
  modeToName[STANCE] = "STANCE";

  modeToName[RH_FLY] = "RH_FLY";
  modeToName[RH_RF] = "RH_RF";
  modeToName[RH_LF] = "RH_LF";
  modeToName[RH_STANCE] = "RH_STANCE";

  modeToName[LH_FLY] = "LH_FLY";
  modeToName[LH_RF] = "LH_RF";
  modeToName[LH_LF] = "LH_LF";
  modeToName[LH_STANCE] = "LH_STANCE";

  modeToName[RH_LH_FLY] = "RH_LH_FLY";
  modeToName[RH_LH_RF] = "RH_LH_RF";
  modeToName[RH_LH_LF] = "RH_LH_LF";
  modeToName[RH_LH_STANCE] = "RH_LH_STANCE";


  return modeToName[modeNumber];
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
inline size_t string2ModeNumber(const std::string& modeString) {

  // build the map from name to mode number
  std::map<std::string, size_t> nameToMode;
  nameToMode["FLY"] = FLY;
  nameToMode["RF"] = RF;
  nameToMode["LF"] = LF;
  nameToMode["STANCE"] = STANCE;

  nameToMode["RH_FLY"] = RH_FLY;
  nameToMode["RH_RF"] = RH_RF;
  nameToMode["RH_LF"] = RH_LF;
  nameToMode["RH_STANCE"] = RH_STANCE;

  nameToMode["LH_FLY"] = LH_FLY;
  nameToMode["LH_RF"] = LH_RF;
  nameToMode["LH_LF"] = LH_LF;
  nameToMode["LH_STANCE"] = LH_STANCE;

  nameToMode["RH_LH_FLY"] = RH_LH_FLY;
  nameToMode["RH_LH_RF"] = RH_LH_RF;
  nameToMode["RH_LH_LF"] = RH_LH_LF;
  nameToMode["RH_LH_STANCE"] = RH_LH_STANCE;


  return nameToMode[modeString];
}

}  // namespace humanoid_wb
}  // end of namespace ocs2
