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

#include "humanoid_wb_ros/torque_mapping/JointImpedanceMappedPolicyPublisher.h"

// OCS2
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <gazebo_ocs2_ros_interfaces/visualization/VisualizationHelpers.h>
#include "humanoid_wb/gait/MotionPhaseDefinition.h"

namespace ocs2 {
namespace humanoid_wb {

/*
 * joints belonging to lower body
 */
std::vector<std::string> lowerBodyJoints = {
    "leg_right_1_joint", "leg_right_2_joint", "leg_right_3_joint", "leg_right_4_joint", "leg_right_5_joint", "leg_right_6_joint",
    "leg_left_1_joint", "leg_left_2_joint", "leg_left_3_joint", "leg_left_4_joint", "leg_left_5_joint", "leg_left_6_joint",
};

/*
 * Map from joint name to an index characterizing motor size
 * 0 -> small, 1 -> medium, 2 -> big
 */
std::map<std::string, int> mapJointNameToMotorSizeIndex = {
    {"leg_right_1_joint", 1}, {"leg_right_2_joint", 1}, {"leg_right_3_joint", 1}, {"leg_right_4_joint", 2}, {"leg_right_5_joint", 2}, {"leg_right_6_joint", 1},
    {"leg_left_1_joint", 1}, {"leg_left_2_joint", 1}, {"leg_left_3_joint", 1}, {"leg_left_4_joint", 2}, {"leg_left_5_joint", 2}, {"leg_left_6_joint", 1},

    {"torso_1_joint", 2}, {"torso_2_joint", 2},
    {"head_1_joint", 0}, {"head_2_joint", 0},

    {"arm_right_1_joint", 1}, {"arm_right_2_joint", 1}, {"arm_right_3_joint", 1}, {"arm_right_4_joint", 1}, {"arm_right_5_joint", 0}, {"arm_right_6_joint", 0}, {"arm_right_7_joint", 0},
    {"arm_left_1_joint", 1}, {"arm_left_2_joint", 1}, {"arm_left_3_joint", 1}, {"arm_left_4_joint", 1}, {"arm_left_5_joint", 0}, {"arm_left_6_joint", 0}, {"arm_left_7_joint", 0}
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
JointImpedanceMappedPolicyPublisher::JointImpedanceMappedPolicyPublisher(PinocchioInterface pinocchioInterface,
                                                                         CentroidalModelInfo centroidalModelInfo,
                                                                         ros::NodeHandle& nodeHandle,
                                                                         scalar_t maxUpdateFrequency)
    : centroidalModelInfo_(centroidalModelInfo),
      lastTime_(std::numeric_limits<scalar_t>::lowest()),
      minPublishTimeDifference_(1.0 / maxUpdateFrequency),
      nodeHandle_(nodeHandle),
      jointNames_{"leg_right_1_joint", "leg_right_2_joint", "leg_right_3_joint", "leg_right_4_joint", "leg_right_5_joint", "leg_right_6_joint",
                  "leg_left_1_joint", "leg_left_2_joint", "leg_left_3_joint", "leg_left_4_joint", "leg_left_5_joint", "leg_left_6_joint",
                  "torso_1_joint", "torso_2_joint",
                  "head_1_joint", "head_2_joint",
                  "arm_right_1_joint", "arm_right_2_joint", "arm_right_3_joint", "arm_right_4_joint", "arm_right_5_joint", "arm_right_6_joint", "arm_right_7_joint",
                  "arm_left_1_joint", "arm_left_2_joint", "arm_left_3_joint", "arm_left_4_joint", "arm_left_5_joint", "arm_left_6_joint", "arm_left_7_joint"
                 },
      centroidalModelRbdConversions_(pinocchioInterface, centroidalModelInfo),
      wheelIndices_(4, 0)
{
  if (jointNames_.size() != centroidalModelInfo_.actuatedDofNum) {
      throw std::runtime_error("[JointImpedanceMappedPolicyPublisher] Hardcoded joint names do not match with CentroidalModelInfo!");
  }
  jointImpedanceMappedPolicyPublisher_ = nodeHandle.advertise<ocs2_msgs::joint_impedance_mapped_policy>(publishTopicName_, 1);
  getParamsFromServer(nodeHandle);
  // wheel joint indeces
  // wheelIndices_.reserve(4);
  // for (int wheel_i = 1; wheel_i < 5; wheel_i++) {
  //     wheelIndices_[wheel_i - 1] = std::distance(jointNames_.begin(), std::find(jointNames_.begin(), jointNames_.end(), "j_wheel_" + std::to_string(wheel_i)));
  // }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void JointImpedanceMappedPolicyPublisher::getParamsFromServer(ros::NodeHandle& nodeHandle) {

  // load PD gains for control
  std::vector<float> lowerBodyPGains(3), lowerBodyDGains(3), upperBodyPGains(3), upperBodyDGains(3);
  std::string jointImpedanceGainsPrefix("mrt_joint_impedance_gains");
  nodeHandle_.getParamCached(jointImpedanceGainsPrefix + "/lower_body/p_gains", lowerBodyPGains);
  nodeHandle_.getParamCached(jointImpedanceGainsPrefix + "/lower_body/d_gains", lowerBodyDGains);
  nodeHandle_.getParamCached(jointImpedanceGainsPrefix + "/upper_body/p_gains", upperBodyPGains);
  nodeHandle_.getParamCached(jointImpedanceGainsPrefix + "/upper_body/d_gains", upperBodyDGains);
  const auto lowerBodyJointImpedanceGains = std::make_pair(lowerBodyPGains, lowerBodyDGains);
  const auto upperBodyJointImpedanceGains = std::make_pair(upperBodyPGains, upperBodyDGains);
  orderedJointImpedanceGains_ = getJointImpedanceGains(lowerBodyJointImpedanceGains, upperBodyJointImpedanceGains);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void JointImpedanceMappedPolicyPublisher::update(const SystemObservation& observation, const PrimalSolution& primalSolution, const CommandData& command) {
  if (observation.time - lastTime_ > minPublishTimeDifference_) {
    jointImpedanceMappedPolicyPublisher_.publish(createJointImpedanceMappedPolicyMsg(primalSolution));
    lastTime_ = observation.time;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ocs2_msgs::joint_impedance_mapped_policy
JointImpedanceMappedPolicyPublisher::createJointImpedanceMappedPolicyMsg(const PrimalSolution& primalSolution) {
    ocs2_msgs::joint_impedance_mapped_policy impedancePolicyMsg;
    int jointNumber = centroidalModelInfo_.actuatedDofNum;
    impedancePolicyMsg.name = jointNames_;
    impedancePolicyMsg.timeTrajectory = primalSolution.timeTrajectory_;
    for (auto& i: wheelIndices_) {
        orderedJointImpedanceGains_.first[i] = 0.0;
        orderedJointImpedanceGains_.second[i] = 10.0;
    }
    impedancePolicyMsg.stiffness.assign(orderedJointImpedanceGains_.first.data(),
                                        orderedJointImpedanceGains_.first.data() + orderedJointImpedanceGains_.first.size());
    impedancePolicyMsg.damping.assign(orderedJointImpedanceGains_.second.data(),
                                      orderedJointImpedanceGains_.second.data() + orderedJointImpedanceGains_.second.size());

    impedancePolicyMsg.ff_torque.clear();
    impedancePolicyMsg.ff_torque.reserve(primalSolution.timeTrajectory_.size());
    for (int trjPoint = 0; trjPoint < primalSolution.stateTrajectory_.size(); trjPoint++) {
        // position & velocity
        vector_t qJoints = centroidal_model::getJointAngles(primalSolution.stateTrajectory_[trjPoint], centroidalModelInfo_);
        vector_t dqJoints = centroidal_model::getJointVelocities(primalSolution.inputTrajectory_[trjPoint], centroidalModelInfo_);
        // compute torques with inverse dynamics, discard unactuaded part
        vector_t ffJointTorque = centroidalModelRbdConversions_.computeRbdTorqueFromCentroidalModel(
                    primalSolution.stateTrajectory_[trjPoint],
                    primalSolution.inputTrajectory_[trjPoint],
                    vector_t::Zero(jointNumber)).tail(jointNumber);

        // clamp joint commands if true from task.info file
//        clampReferences(ffJointTorque, qJoints, dqJoints);

        // fill message
        ocs2_msgs::mpc_state effortTrjPoint, positionRefTrjPoint, velocityRefTrjPoint;
        effortTrjPoint.value.assign(ffJointTorque.data(), ffJointTorque.data() + ffJointTorque.size());
        positionRefTrjPoint.value.assign(qJoints.data(), qJoints.data() + qJoints.size());
        velocityRefTrjPoint.value.assign(dqJoints.data(), dqJoints.data() + dqJoints.size());

        // keep default control for the wheels
        for (auto& i: wheelIndices_) {
            effortTrjPoint.value.at(i) = 0.0;
        }
        // assign to impedance policy msg
        impedancePolicyMsg.ff_torque.emplace_back(effortTrjPoint);
        impedancePolicyMsg.position_reference.emplace_back(positionRefTrjPoint);
        impedancePolicyMsg.velocity_reference.emplace_back(velocityRefTrjPoint);

    }
    return impedancePolicyMsg;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::pair<vector_t, vector_t> JointImpedanceMappedPolicyPublisher::getJointImpedanceGains(const std::pair<std::vector<float>, std::vector<float>>& lowerBodyJointImpedanceGains,
                                                                    const std::pair<std::vector<float>, std::vector<float>>& upperBodyJointImpedanceGains) const {
    vector_t stiffness = vector_t::Zero(jointNames_.size());
    vector_t damping = vector_t::Zero(jointNames_.size());

    for (int i = 0; i < jointNames_.size(); i++) {
        // if joint is in lower body
        if (std::find(lowerBodyJoints.begin(), lowerBodyJoints.end(), jointNames_[i]) != lowerBodyJoints.end()) {
            stiffness[i] = lowerBodyJointImpedanceGains.first[mapJointNameToMotorSizeIndex[jointNames_[i]]];
            damping[i] = lowerBodyJointImpedanceGains.second[mapJointNameToMotorSizeIndex[jointNames_[i]]];
        } else {
            stiffness[i] = upperBodyJointImpedanceGains.first[mapJointNameToMotorSizeIndex[jointNames_[i]]];
            damping[i] = upperBodyJointImpedanceGains.second[mapJointNameToMotorSizeIndex[jointNames_[i]]];
        }
    }
    auto jointGains = std::make_pair(stiffness, damping);
    return jointGains;
}

}  // namespace humanoid_wb
}  // namespace ocs2
