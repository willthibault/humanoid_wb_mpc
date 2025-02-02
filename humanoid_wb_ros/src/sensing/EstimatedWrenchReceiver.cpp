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

#include "humanoid_wb_ros/sensing/EstimatedWrenchReceiver.h"

namespace ocs2 {
namespace humanoid_wb {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
EstimatedWrenchReceiver::EstimatedWrenchReceiver(ros::NodeHandle nodeHandle,
                                                 std::shared_ptr<ForceTorqueSensing> forceTorqueSensingPtr)
    : receivedWrenches_(EeEstimatedWrenches()), wrenchesUpdated_(false),
      forceTorqueSensingPtr_(forceTorqueSensingPtr) {
  eeEstimatedWrenchSubscriber_ = nodeHandle.subscribe("/humanoid_wb_estimation/contacts/wrench", 1, // this needs adjustment and further consideration
                                                      &EstimatedWrenchReceiver::eeEstimatedWrenchCallback, this,
                                                      ::ros::TransportHints().tcpNoDelay());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void EstimatedWrenchReceiver::preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& currentState,
                                const ReferenceManagerInterface& referenceManager) {
  if (wrenchesUpdated_) {
    std::lock_guard<std::mutex> lock(receivedWrenchesMutex_);
//    std::cerr << "[EstimatedWrenchReceiver]: Received new estimated wrenches at " << initTime << ".\n";
//    std::cerr << receivedWrenches_.forceNorms.transpose() << "\n";
    forceTorqueSensingPtr_->setEeEstimatedWrenches(receivedWrenches_);
    wrenchesUpdated_ = false;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void EstimatedWrenchReceiver::eeEstimatedWrenchCallback(const base_estimation::ContactsWrenchConstPtr& msg) {
    std::lock_guard<std::mutex> lock(receivedWrenchesMutex_);
    receivedWrenches_ = readEstimatedWrenchMsg(*msg);
    wrenchesUpdated_ = true;
}

}  // namespace humanoid_wb
}  // namespace ocs2
