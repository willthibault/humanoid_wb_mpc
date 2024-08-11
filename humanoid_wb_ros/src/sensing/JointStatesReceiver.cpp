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

#include "humanoid_wb_ros/sensing/JointStatesReceiver.h"

namespace ocs2 {
namespace humanoid_wb {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
JointStatesReceiver::JointStatesReceiver(ros::NodeHandle nodeHandle,
                                         std::shared_ptr<ForceTorqueSensing> forceTorqueSensingPtr)
    : receivedJointStates_(JointStates()), jointStatesUpdated_(false),
      forceTorqueSensingPtr_(forceTorqueSensingPtr) {
  jointStatesSubscriber_ = nodeHandle.subscribe("/xbotcore/joint_states", 1, // want to move away from xbot
                                                &JointStatesReceiver::jointStatesCallback, this,
                                                ::ros::TransportHints().tcpNoDelay());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void JointStatesReceiver::preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& currentState,
                                       const ReferenceManagerInterface& referenceManager) {
  if (jointStatesUpdated_) {
    std::lock_guard<std::mutex> lock(receivedJointStatesMutex_);
//    std::cerr << "[JointStatesReceiver]: Received new joint states at " << initTime << ".\n";
    forceTorqueSensingPtr_->setJointStates(receivedJointStates_);
    jointStatesUpdated_ = false;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void JointStatesReceiver::jointStatesCallback(const xbot_msgs::JointStateConstPtr& msg) {
    std::lock_guard<std::mutex> lock(receivedJointStatesMutex_);
    readJointStatesMsg(*msg, receivedJointStates_);
    jointStatesUpdated_ = true;
}

}  // namespace humanoid_wb
}  // namespace ocs2
