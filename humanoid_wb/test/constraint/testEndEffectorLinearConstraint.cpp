/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

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

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics-derivatives.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <gtest/gtest.h>

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>

#include "humanoid_wb/common/ModelSettings.h"
#include "humanoid_wb/constraint/EndEffectorLinearConstraint.h"
#include "humanoid_wb/test/HumanoidWbFactoryFunctions.h"

using namespace ocs2;
using namespace humanoid_wb;

class testEndEffectorLinearConstraint : public ::testing::Test {
 public:
  testEndEffectorLinearConstraint() {
    const ModelSettings modelSettings;  // default constructor just to get contactNames3DoF

    pinocchioMappingPtr.reset(new CentroidalModelPinocchioMapping(centroidalModelInfo));
    pinocchioMappingAdPtr.reset(new CentroidalModelPinocchioMappingCppAd(centroidalModelInfo.toCppAd()));

    eeKinematicsPtr.reset(
        new PinocchioEndEffectorKinematics(*pinocchioInterfacePtr, *pinocchioMappingPtr, {modelSettings.contactNames3DoF[0]}));

    auto velocityUpdateCallback = [&](ad_vector_t state, PinocchioInterfaceTpl<ad_scalar_t>& pinocchioInterfaceAd) {
      const ad_vector_t& q = state.tail(centroidalModelInfo.generalizedCoordinatesNum);
      updateCentroidalDynamics(pinocchioInterfaceAd, centroidalModelInfo.toCppAd(), q);
    };
    eeKinematicsAdPtr.reset(new PinocchioEndEffectorKinematicsCppAd(
        *pinocchioInterfacePtr, *pinocchioMappingAdPtr, {modelSettings.contactNames3DoF[0]}, centroidalModelInfo.stateDim,
        centroidalModelInfo.inputDim, velocityUpdateCallback, "arm_right_link_7", "/tmp/ocs2", true, true));

    x.resize(centroidalModelInfo.stateDim);
    x(0) = 0.0;  // vcom_x
    x(1) = 0.0;  // vcom_y
    x(2) = 0.0;  // vcom_z
    x(3) = 0.0;  // L_x / robotMass
    x(4) = 0.0;  // L_y / robotMass
    x(5) = 0.0;  // L_z / robotMass

    x(6) = 0.0;   // p_base_x
    x(7) = 0.0;   // p_base_y
    x(8) = 0.82;  // p_base_z
    x(9) = 0.0;   // theta_base_z
    x(10) = 0.0;  // theta_base_y
    x(11) = 0.0;  // theta_base_x

    x(12) = 0.0;  // leg_right_1_joint
    x(13) = 0.0;  // leg_right_2_joint
    x(14) = -0.31;  // leg_right_3_joint
    x(15) = 0.64;  // leg_right_4_joint
    x(16) = -0.32;  // leg_right_5_joint
    x(17) = 0.0;  // leg_right_6_joint

    x(18) = 0.0;  // leg_left_1_joint
    x(19) = 0.0;  // leg_left_2_joint
    x(20) = -0.31;  // leg_left_3_joint
    x(21) = 0.64;  // leg_left_4_joint
    x(22) = -0.32;  // leg_left_5_joint
    x(23) = 0.0;  // leg_left_6_joint

    x(24) = 0.0;  // torso_1_joint
    x(25) = 0.0;  // torso_2_joint

    x(26) = 0.0;  // head_1_joint
    x(27) = 0.0;  // head_2_joint

    x(28) = -0.44;  // arm_right_1_joint
    x(29) = 0.26;  // arm_right_2_joint
    x(30) = 0.0;  // arm_right_3_joint
    x(31) = 0.88;  // arm_right_4_joint
    x(32) = 0.0;  // arm_right_5_joint
    x(33) = 0.0;  // arm_right_6_joint
    x(34) = 0.0;  // arm_right_7_joint

    x(35) = -0.44;  // arm_right_1_joint
    x(36) = 0.26;  // arm_right_2_joint
    x(37) = 0.0;  // arm_right_3_joint
    x(38) = 0.88;  // arm_right_4_joint
    x(39) = 0.0;  // arm_right_5_joint
    x(40) = 0.0;  // arm_right_6_joint
    x(41) = 0.0;  // arm_right_7_joint

    u = vector_t::Random(centroidalModelInfo.inputDim);

    config.b = vector_t::Random(3);
    config.Ax = matrix_t::Random(3, 3);
    config.Av = matrix_t::Random(3, 3);
  }

  const CentroidalModelType centroidalModelType = CentroidalModelType::SingleRigidBodyDynamics;
  std::unique_ptr<PinocchioInterface> pinocchioInterfacePtr = createHumanoidWbPinocchioInterface();
  const CentroidalModelInfo centroidalModelInfo = createHumanoidWbCentroidalModelInfo(*pinocchioInterfacePtr, centroidalModelType);
  PreComputation preComputation;

  std::unique_ptr<CentroidalModelPinocchioMapping> pinocchioMappingPtr;
  std::unique_ptr<CentroidalModelPinocchioMappingCppAd> pinocchioMappingAdPtr;
  std::unique_ptr<PinocchioEndEffectorKinematics> eeKinematicsPtr;
  std::unique_ptr<PinocchioEndEffectorKinematicsCppAd> eeKinematicsAdPtr;
  EndEffectorLinearConstraint::Config config;
  vector_t x, u;
};

TEST_F(testEndEffectorLinearConstraint, testValue) {
  auto eeVelConstraintPtr = std::unique_ptr<EndEffectorLinearConstraint>(new EndEffectorLinearConstraint(*eeKinematicsPtr, 3));
  eeVelConstraintPtr->configure(config);
  auto eeVelConstraintAdPtr = std::unique_ptr<EndEffectorLinearConstraint>(new EndEffectorLinearConstraint(*eeKinematicsAdPtr, 3));
  eeVelConstraintAdPtr->configure(config);

  dynamic_cast<PinocchioEndEffectorKinematics&>(eeVelConstraintPtr->getEndEffectorKinematics())
      .setPinocchioInterface(*pinocchioInterfacePtr);
  pinocchioMappingPtr->setPinocchioInterface(*pinocchioInterfacePtr);

  const auto& model = pinocchioInterfacePtr->getModel();
  auto& data = pinocchioInterfacePtr->getData();

  const auto q = pinocchioMappingPtr->getPinocchioJointPosition(x);
  updateCentroidalDynamics(*pinocchioInterfacePtr, centroidalModelInfo, q);
  const auto v = pinocchioMappingPtr->getPinocchioJointVelocity(x, u);

  // For getPosition() & getVelocity() of PinocchioEndEffectorKinematics
  pinocchio::forwardKinematics(model, data, q, v);
  pinocchio::updateFramePlacements(model, data);

  const auto value = eeVelConstraintPtr->getValue(0.0, x, u, preComputation);
  const auto valueAd = eeVelConstraintAdPtr->getValue(0.0, x, u, preComputation);

  EXPECT_TRUE(value.isApprox(valueAd));
}

TEST_F(testEndEffectorLinearConstraint, testLinearApproximation) {
  auto eeVelConstraintPtr = std::unique_ptr<EndEffectorLinearConstraint>(new EndEffectorLinearConstraint(*eeKinematicsPtr, 3));
  eeVelConstraintPtr->configure(config);
  auto eeVelConstraintAdPtr = std::unique_ptr<EndEffectorLinearConstraint>(new EndEffectorLinearConstraint(*eeKinematicsAdPtr, 3));
  eeVelConstraintAdPtr->configure(config);

  dynamic_cast<PinocchioEndEffectorKinematics&>(eeVelConstraintPtr->getEndEffectorKinematics())
      .setPinocchioInterface(*pinocchioInterfacePtr);
  pinocchioMappingPtr->setPinocchioInterface(*pinocchioInterfacePtr);

  const auto& model = pinocchioInterfacePtr->getModel();
  auto& data = pinocchioInterfacePtr->getData();

  // PinocchioInterface update for the analytical EndEffectorVelocityConstraint
  const auto q = pinocchioMappingPtr->getPinocchioJointPosition(x);
  updateCentroidalDynamics(*pinocchioInterfacePtr, centroidalModelInfo, q);
  const auto v = pinocchioMappingPtr->getPinocchioJointVelocity(x, u);
  const auto a = vector_t::Zero(q.size());

  // For getPositionLinearApproximation of PinocchioEndEffectorKinematics
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);
  pinocchio::computeJointJacobians(model, data);
  // For getVelocityLinearApproximation of PinocchioEndEffectorKinematics
  pinocchio::computeForwardKinematicsDerivatives(model, data, q, v, a);
  // For getOcs2Jacobian of CentroidalModelPinocchioMapping
  updateCentroidalDynamicsDerivatives(*pinocchioInterfacePtr, centroidalModelInfo, q, v);

  const auto linApprox = eeVelConstraintPtr->getLinearApproximation(0.0, x, u, preComputation);
  const auto linApproxAd = eeVelConstraintAdPtr->getLinearApproximation(0.0, x, u, preComputation);

  EXPECT_TRUE(linApprox.f.isApprox(linApproxAd.f));
  EXPECT_TRUE(linApprox.dfdx.isApprox(linApproxAd.dfdx, 1e-14));
  EXPECT_TRUE(linApprox.dfdu.isApprox(linApproxAd.dfdu));
}
