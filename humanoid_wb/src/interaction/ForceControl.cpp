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

#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/model.hpp>

#include <humanoid_wb/interaction/ForceControl.h>
#include <ocs2_centroidal_model/AccessHelperFunctions.h>

namespace ocs2 {
namespace interaction {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ForceControl::ForceControl(const PinocchioInterface& pinocchioInterface, const CentroidalModelInfo& centroidalModelInfo,
                           Config config)
    : pinocchioInterfacePtr_(new PinocchioInterface(pinocchioInterface)), centroidalModelInfo_(centroidalModelInfo),
      config_(config), eeEstimatedWrenches_(vector_t::Zero(6 * config.eeIndices.size()))
{}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t ForceControl::mapWrenchErrorToArmJointTorques(const vector_t& mpcState) const {

    const size_t wrenchesTotalDim = config_.eeIndices.size() * 6;
    const size_t armJointsNum = 7;
    const size_t armsJointsTotalDim = config_.eeIndices.size() * armJointsNum;

    // get jacobian from torso to upper body
    const auto& model = pinocchioInterfacePtr_->getModel();
    auto& data = pinocchioInterfacePtr_->getData();
    const auto q = centroidal_model::getGeneralizedCoordinates(mpcState, centroidalModelInfo_);
    pinocchio::computeJointJacobians(model, data, q);
    pinocchio::updateFramePlacements(model, data);
    pinocchio::forwardKinematics(model, data, q);

    // jacobian
    matrix_t torsoToArmEeJacobians = matrix_t::Zero(wrenchesTotalDim, armsJointsTotalDim);
    for (size_t i = 0; i < config_.eeFrameNames.size(); i++) {

      matrix_t jacobianWorldToContactPointInWorldFrame = matrix_t::Zero(6, centroidalModelInfo_.generalizedCoordinatesNum);
      pinocchio::getFrameJacobian(model, data, model.getBodyId(config_.eeFrameNames[i]), pinocchio::LOCAL_WORLD_ALIGNED,
                                  jacobianWorldToContactPointInWorldFrame);
      // remove 2 for universe and root and add 6 for floating base
      size_t armJointsIndexInGeneralized = model.getJointId("arm_right_link_1") - 2 + 6 + i * armJointsNum; //confirm this line

      torsoToArmEeJacobians.block(6 * i, armJointsNum * i, 6, armJointsNum) =
          jacobianWorldToContactPointInWorldFrame.block(0, armJointsIndexInGeneralized, 6, armJointsNum);
    }
    auto wrenchError = getWrenchError();    // volumn vector of size 6 * nee

    // TODO: add gain
    return torsoToArmEeJacobians.transpose() * wrenchError;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t ForceControl::getWrenchError() const {
    // TODO: check sign of wrenches and forces
    return config_.eeRefWrenches - eeEstimatedWrenches_;
}


}
}
