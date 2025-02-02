
#pragma once

// clang-format off
/* Centauro URDF string */
static constexpr auto humanoidWbUrdf = R"(
<robot name="reemc" xmlns:controller="http://playerstage.sourceforge.net/gazebo/xmlschema/#controller" xmlns:interface="http://playerstage.sourceforge.net/gazebo/xmlschema/#interface" xmlns:sensor="http://playerstage.sourceforge.net/gazebo/xmlschema/#sensor">
  <link name="base_link">
    <inertial>
      <mass value="14.89"/>
      <origin rpy="0 0 0" xyz="-0.029021 -0.0018945 0.018427"/>
      <inertia ixx="0.026081" ixy="0.00013572" ixz="0.0011164" iyy="0.022992" iyz="-0.00010555" izz="0.038521"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://reemc_description/meshes/base/base_hcrmi.dae"/>
      </geometry>
      <material name="LightGrey"/>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="-0.022 0 0.01"/>
      <geometry>
        <box size="0.3 0.35 0.14"/>
      </geometry>
    </collision>
  </link>
  <!-- TODO: Update dynamic models -->
  <!-- TODO: Update dynamic models -->
  <gazebo>
    <plugin filename="libgazebo_ros_control.so" name="gazebo_ros_control">
      <ns/>
      <!-- <robotSimType>reemc_hardware_gazebo/ReemcHardwareGazebo</robotSimType> -->
      <robotSimType>pal_hardware_gazebo/PalHardwareGazebo</robotSimType>
      <robotNamespace/>
      <controlPeriod>0.001</controlPeriod>
    </plugin>
    <plugin filename="libgazebo_ros_p3d.so" name="gazebo_ros_p3d">
      <bodyName>base_link</bodyName>
      <topicName>ground_truth_odom</topicName>
      <alwaysOn>true</alwaysOn>
      <updateRate>100.0</updateRate>
    </plugin>
  </gazebo>
  <gazebo>
    <plugin filename="libgazebo_ros_force.so" name="gazebo_ros_force">
      <bodyName>base_link</bodyName>
      <topicName>base_link_wrench</topicName>
    </plugin>
  </gazebo>
  <gazebo>
    <plugin filename="libgazebo_world_odometry.so" name="gazebo_ros_odometry">
      <frameName>base_link</frameName>
      <topicName>floating_base_pose_simulated</topicName>
    </plugin>
  </gazebo>
  <gazebo reference="imu_link">
    <!-- this is expected to be reparented to pelvis with appropriate offset
         when imu_link is reduced by fixed joint reduction -->
    <!-- todo: this is working with gazebo 1.4, need to write a unit test -->
    <sensor name="imu_sensor" type="imu">
      <always_on>1</always_on>
      <update_rate>1000.0</update_rate>
      <imu>
        <noise>
          <type>gaussian</type>
          <!-- Noise parameters from Boston Dynamics
               (http://gazebosim.org/wiki/Sensor_noise):
                 rates (rad/s): mean=0, stddev=2e-4
                 accels (m/s/s): mean=0, stddev=1.7e-2
                 rate bias (rad/s): 5e-6 - 1e-5
                 accel bias (m/s/s): 1e-1
               Experimentally, simulation provide rates with noise of
               about 1e-3 rad/s and accels with noise of about 1e-1 m/s/s.
               So we don't expect to see the noise unless number of inner iterations
               are increased.

               We will add bias.  In this model, bias is sampled once for rates
               and once for accels at startup; the sign (negative or positive)
               of each bias is then switched with equal probability.  Thereafter,
               the biases are fixed additive offsets.  We choose
               bias means and stddevs to produce biases close to the provided
               data. -->
          <rate>
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
            <bias_mean>0.0000075</bias_mean>
            <bias_stddev>0.0000008</bias_stddev>
          </rate>
          <accel>
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
            <bias_mean>0.1</bias_mean>
            <bias_stddev>0.001</bias_stddev>
          </accel>
        </noise>
      </imu>
    </sensor>
  </gazebo>
  <gazebo reference="r_foot">
    <mu1>0.9</mu1>
    <mu2>0.9</mu2>
    <fdir1>1 0 0</fdir1>
    <maxVel>1.0</maxVel>
    <minDepth>0.003</minDepth>
  </gazebo>
  <gazebo reference="l_foot">
    <mu1>0.9</mu1>
    <mu2>0.9</mu2>
    <fdir1>1 0 0</fdir1>
    <maxVel>1.0</maxVel>
    <minDepth>0.003</minDepth>
  </gazebo>
  <material name="Blue">
    <color rgba="0.0 0.0 0.8 1.0"/>
  </material>
  <material name="Green">
    <color rgba="0.0 0.8 0.0 1.0"/>
  </material>
  <material name="Grey">
    <color rgba="0.7 0.7 0.7 1.0"/>
  </material>
  <material name="LightGrey">
    <color rgba="0.9 0.9 0.9 1.0"/>
  </material>
  <material name="DarkGrey">
    <color rgba="0.2 0.2 0.2 1.0"/>
  </material>
  <material name="Red">
    <color rgba="0.8 0.0 0.0 1.0"/>
  </material>
  <material name="White">
    <color rgba="1.0 1.0 1.0 1.0"/>
  </material>
  <!-- Now we can start using the macros included above to define the actual robot -->
  <!-- The reflect parameter takes the value 1 or -1 to respectively distinguish right from left side in expressions -->
  <link name="leg_right_1_link">
    <inertial>
      <mass value="1.02901"/>
      <origin rpy="0 0 0" xyz="-0.02571 0 0.02557"/>
      <inertia ixx="0.00132187534" ixy="0.00000001514" ixz="0.00040022268" iyy="0.0018528617" iyz="0.00000050713" izz="0.00084231334"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://reemc_description/meshes/leg/leg_1.dae" scale="1 1 1"/>
      </geometry>
      <material name="LightGrey"/>
    </visual>
  </link>
  <joint name="leg_right_1_joint" type="revolute">
    <origin rpy="0 0 0" xyz="0  -0.075 -0.14353"/>
    <axis xyz="0 0 1"/>
    <parent link="base_link"/>
    <child link="leg_right_1_link"/>
    <dynamics damping="1.0" friction="1.0"/>
    <limit effort="50.69" lower="-0.523598775598" upper="0.785398163397" velocity="2.74"/>
    <safety_controller k_position="100" k_velocity="100" soft_lower_limit="-0.488692190558" soft_upper_limit="0.750491578358"/>
  </joint>
  <link name="leg_right_2_link">
    <inertial>
      <mass value="0.69621"/>
      <origin rpy="0.0 0.0 0.0" xyz="0.00057 -0.00881 -0.01125"/>
      <inertia ixx="0.0008416476" ixy="-0.00000268743" ixz="0.00000667199" iyy="0.00039794844" iyz="0.00002668971" izz="0.00084196694"/>
    </inertial>
    <visual>
      <origin rpy="1.57079632679 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="0.10" radius="0.04"/>
      </geometry>
      <material name="LightGrey"/>
    </visual>
  </link>
  <joint name="leg_right_2_joint" type="revolute">
    <origin rpy="0 1.57079632679 0" xyz="0 0 0.0"/>
    <axis xyz="0 0 1"/>
    <parent link="leg_right_1_link"/>
    <child link="leg_right_2_link"/>
    <dynamics damping="1.0" friction="1.0"/>
    <limit effort="71.4" lower="-0.523598775598" upper="0.261799387799" velocity="3.94"/>
    <safety_controller k_position="100" k_velocity="100" soft_lower_limit="-0.488692190558" soft_upper_limit="0.226892802759"/>
  </joint>
  <link name="leg_right_3_link">
    <inertial>
      <mass value="4.82"/>
      <origin rpy="0.0 0 0.0" xyz="0.1521 -0.0094 -0.0152"/>
      <inertia ixx="0.00507885174" ixy="-0.00034076421" ixz="-0.00225952731" iyy="0.02850097402" iyz="0.00003047874" izz="0.02632392239"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://reemc_description/meshes/leg/leg_3_hcrmi.dae" scale="1 1 -1.0"/>
      </geometry>
      <material name="LightGrey"/>
    </visual>
    <collision>
      <origin rpy="0 1.57079632679 0" xyz="0.15 0 -0.015"/>
      <geometry>
        <box size="0.15 0.10 0.20"/>
      </geometry>
    </collision>
  </link>
  <joint name="leg_right_3_joint" type="revolute">
    <origin rpy="-1.57079632679 0 0" xyz="0 0 0.0"/>
    <axis xyz="0 0 1"/>
    <parent link="leg_right_2_link"/>
    <child link="leg_right_3_link"/>
    <dynamics damping="1.0" friction="1.0"/>
    <limit effort="62.9" lower="-1.74532925199" upper="0.785398163397" velocity="6.0"/>
    <safety_controller k_position="100" k_velocity="100" soft_lower_limit="-1.71042266695" soft_upper_limit="0.750491578358"/>
  </joint>
  <link name="leg_right_4_link">
    <inertial>
      <mass value="3.35"/>
      <origin rpy="0.0 0.0 0.0" xyz="0.1368 0.0099 -0.0173"/>
      <inertia ixx="0.00361006492" ixy="0.00012218208" ixz="0.00160208242" iyy="0.02470649045" iyz="0.00008129755" izz="0.02272322206"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://reemc_description/meshes/leg/leg_4_hcrmi.dae" scale="1 1 -1.0"/>
      </geometry>
      <material name="LightGrey"/>
    </visual>
    <collision>
      <origin rpy="0 -1.57079632679 0" xyz="0.15 0 -0.015"/>
      <geometry>
        <box size="0.15 0.10 0.20"/>
      </geometry>
    </collision>
  </link>
  <joint name="leg_right_4_joint" type="revolute">
    <origin rpy="0 0 0" xyz="0.300 0 0.0"/>
    <axis xyz="0 0 1"/>
    <parent link="leg_right_3_link"/>
    <child link="leg_right_4_link"/>
    <dynamics damping="0.1" friction="0"/>
    <dynamics damping="1.0" friction="1.0"/>
    <limit effort="140.4" lower="0.0" upper="2.61799387799" velocity="4.25"/>
    <safety_controller k_position="100" k_velocity="100" soft_lower_limit="0.0349065850399" soft_upper_limit="2.58308729295"/>
  </joint>
  <link name="leg_right_5_link">
    <inertial>
      <mass value="0.68213"/>
      <origin rpy="0.0 0.0 0.0" xyz="-0.00028 0.01119 -0.00859"/>
      <inertia ixx="0.00082035023" ixy="0.00000048534" ixz="0.00000226301" iyy="0.00082074105" iyz="-0.00002892247" izz="0.00037614067"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="0.104" radius="0.04"/>
      </geometry>
      <material name="LightGrey"/>
    </visual>
  </link>
  <joint name="leg_right_5_joint" type="revolute">
    <origin rpy="0 0 0" xyz="0.30 0 0.0"/>
    <axis xyz="0 0 1"/>
    <parent link="leg_right_4_link"/>
    <child link="leg_right_5_link"/>
    <dynamics damping="1.0" friction="1.0"/>
    <limit effort="91.6" lower="-1.308996939" upper="0.785398163397" velocity="4.0"/>
    <safety_controller k_position="100" k_velocity="100" soft_lower_limit="-1.27409035396" soft_upper_limit="0.750491578358"/>
  </joint>
  <link name="leg_right_6_link">
    <inertial>
      <mass value="2.00843"/>
      <origin rpy="0.0 0.0 0.0" xyz="0.06111 -0.00336 -0.00497"/>
      <inertia ixx="0.00366502559" ixy="0.00020479295" ixz="0.00112439297" iyy="0.00532618026" iyz="0.00012133755" izz="0.00353091463"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://reemc_description/meshes/leg/leg_6_hcrmi.dae" scale="1 -1.0 1"/>
      </geometry>
      <material name="LightGrey"/>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0.1 -0.01 0.0195 "/>
      <geometry>
        <box size="0.03 0.14 0.210"/>
      </geometry>
    </collision>
  </link>
  <joint name="leg_right_6_joint" type="revolute">
    <origin rpy="1.57079632679 0 0" xyz="0 0 0"/>
    <axis xyz="0 0 1"/>
    <parent link="leg_right_5_link"/>
    <child link="leg_right_6_link"/>
    <dynamics damping="1.0" friction="1.0"/>
    <limit effort="71.48" lower="-0.261799387799" upper="0.523598775598" velocity="3.94"/>
    <safety_controller k_position="100" k_velocity="100" soft_lower_limit="-0.226892802759" soft_upper_limit="0.488692190558"/>
  </joint>
  <link name="right_sole_link">
    <inertial>
      <mass value="0.001"/>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <inertia ixx="0.0" ixy="0.0" ixz="0.0" iyy="0.0" iyz="0.0" izz="0.0"/>
    </inertial>
  </link>
  <joint name="right_sole_joint" type="fixed">
    <!-- origin x taken from link_${side}_6_link/visual/origin/x above -->
    <origin rpy="0.0 -1.57079632679 0.0" xyz="0.117 0.0 0.0"/>
    <axis xyz="0 0 1"/>
    <parent link="leg_right_6_link"/>
    <child link="right_sole_link"/>
    <dynamics damping="1.0" friction="1.0"/>
    <limit effort="60" lower="0.0" upper="0.0" velocity="0"/>
  </joint>
  <gazebo reference="leg_right_1_link">
    <mu1>0.9</mu1>
    <mu2>0.9</mu2>
  </gazebo>
  <gazebo reference="leg_right_2_link">
    <mu1>0.9</mu1>
    <mu2>0.9</mu2>
  </gazebo>
  <gazebo reference="leg_right_3_link">
    <mu1>0.9</mu1>
    <mu2>0.9</mu2>
  </gazebo>
  <gazebo reference="leg_right_4_link">
    <mu1>0.9</mu1>
    <mu2>0.9</mu2>
  </gazebo>
  <gazebo reference="leg_right_5_link">
    <mu1>0.9</mu1>
    <mu2>0.9</mu2>
  </gazebo>
  <!-- contact model for foot surface -->
  <gazebo reference="leg_right_6_link">
    <kp>1000000.0</kp>
    <kd>100.0</kd>
    <mu1>1.0</mu1>
    <mu2>1.0</mu2>
    <fdir1>0 0 1</fdir1>
    <maxVel>1.0</maxVel>
    <minDepth>0.00</minDepth>
    <implicitSpringDamper>1</implicitSpringDamper>
  </gazebo>
  <!--force torque sensor -->
  <!--xacro:reemc_force_torque_sensor name="leg_${side}_6_link"  update_rate="100.0"/-->
  <gazebo reference="leg_right_1_joint">
    <implicitSpringDamper>1</implicitSpringDamper>
  </gazebo>
  <gazebo reference="leg_right_2_joint">
    <implicitSpringDamper>1</implicitSpringDamper>
  </gazebo>
  <gazebo reference="leg_right_3_joint">
    <implicitSpringDamper>1</implicitSpringDamper>
  </gazebo>
  <gazebo reference="leg_right_4_joint">
    <implicitSpringDamper>1</implicitSpringDamper>
  </gazebo>
  <gazebo reference="leg_right_5_joint">
    <implicitSpringDamper>1</implicitSpringDamper>
  </gazebo>
  <gazebo reference="leg_right_6_joint">
    <implicitSpringDamper>1</implicitSpringDamper>
    <provideFeedback>1</provideFeedback>
  </gazebo>
  <transmission name="leg_right_1_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <actuator name="leg_right_1_motor">
      <mechanicalReduction>1.0</mechanicalReduction>
    </actuator>
    <joint name="leg_right_1_joint">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
      <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    </joint>
  </transmission>
  <transmission name="leg_right_2_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <actuator name="leg_right_2_motor">
      <mechanicalReduction>1.0</mechanicalReduction>
    </actuator>
    <joint name="leg_right_2_joint">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
      <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    </joint>
  </transmission>
  <transmission name="leg_right_3_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <actuator name="leg_right_3_motor">
      <mechanicalReduction>1.0</mechanicalReduction>
    </actuator>
    <joint name="leg_right_3_joint">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
      <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    </joint>
  </transmission>
  <transmission name="leg_right_4_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <actuator name="leg_right_4_motor">
      <mechanicalReduction>1.0</mechanicalReduction>
    </actuator>
    <joint name="leg_right_4_joint">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
      <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    </joint>
  </transmission>
  <transmission name="leg_right_5_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <actuator name="leg_right_5_motor">
      <mechanicalReduction>1.0</mechanicalReduction>
    </actuator>
    <joint name="leg_right_5_joint">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
      <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    </joint>
  </transmission>
  <transmission name="leg_right_6_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <actuator name="leg_right_6_motor">
      <mechanicalReduction>1.0</mechanicalReduction>
    </actuator>
    <joint name="leg_right_6_joint">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
      <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    </joint>
  </transmission>
  <link name="right_laser_link">
    <inertial>
      <origin rpy="0 0 0" xyz="0.0 0.0 0.0"/>
      <mass value="0.001"/>
      <inertia ixx="3.7174E-05" ixy="4.9927E-08" ixz="1.1015E-05" iyy="4.2412E-05" iyz="-9.8165E-09" izz="4.167E-05"/>
    </inertial>
    <!--    <visual>
         <origin xyz="0.0 0 0.05" rpy="0 0 ${90.0*deg_to_rad}" />
        <geometry>
          <mesh filename="package://reemc_description/meshes/sensors/urg-04lx-ug01.stl" scale="0.001 0.001 0.001"/>
        </geometry>
      <material name="DarkGrey" />
      </visual>
  -->
  </link>
  <joint name="right_laser_joint" type="fixed">
    <axis xyz="0 1 0"/>
    <origin rpy="0.0 0.0 -0.523598775598" xyz="0.085 -0.01 0.073"/>
    <parent link="right_sole_link"/>
    <child link="right_laser_link"/>
    <dynamics damping="1" friction="1.0"/>
    <limit effort="60" lower="0" upper="0" velocity="20"/>
  </joint>
  <gazebo reference="right_laser_link">
    <sensor name="right_laser_sensor" type="ray">
      <pose>0 0 0 0 0 0</pose>
      <visualize>false</visualize>
      <update_rate>10.0</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>360</samples>
            <resolution>1</resolution>
            <min_angle>-1.57079632679</min_angle>
            <max_angle>1.57079632679</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.02</min>
          <max>5.6</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <plugin filename="libgazebo_ros_laser.so" name="gazebo_ros_laser">
        <frameName>/right_laser_link</frameName>
        <topicName>right_scan</topicName>
        <gaussianNoise>0.03</gaussianNoise>
        <hokuyoMinIntensity>101</hokuyoMinIntensity>
        <updateRate>10.0</updateRate>
        <alwaysOn>true</alwaysOn>
      </plugin>
    </sensor>
  </gazebo>
  <link name="leg_left_1_link">
    <inertial>
      <mass value="1.02901"/>
      <origin rpy="0 0 0" xyz="-0.02571 0 0.02557"/>
      <inertia ixx="0.00132187534" ixy="0.00000001514" ixz="0.00040022268" iyy="0.0018528617" iyz="0.00000050713" izz="0.00084231334"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://reemc_description/meshes/leg/leg_1.dae" scale="1 1 1"/>
      </geometry>
      <material name="LightGrey"/>
    </visual>
  </link>
  <joint name="leg_left_1_joint" type="revolute">
    <origin rpy="0 0 0" xyz="0  0.075 -0.14353"/>
    <axis xyz="0 0 1"/>
    <parent link="base_link"/>
    <child link="leg_left_1_link"/>
    <dynamics damping="1.0" friction="1.0"/>
    <limit effort="50.69" lower="-0.785398163397" upper="0.523598775598" velocity="2.74"/>
    <safety_controller k_position="100" k_velocity="100" soft_lower_limit="-0.750491578358" soft_upper_limit="0.488692190558"/>
  </joint>
  <link name="leg_left_2_link">
    <inertial>
      <mass value="0.69621"/>
      <origin rpy="0.0 0.0 0.0" xyz="0.00057 0.00881 -0.01125"/>
      <inertia ixx="0.0008416476" ixy="-0.00000268743" ixz="0.00000667199" iyy="0.00039794844" iyz="0.00002668971" izz="0.00084196694"/>
    </inertial>
    <visual>
      <origin rpy="1.57079632679 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="0.10" radius="0.04"/>
      </geometry>
      <material name="LightGrey"/>
    </visual>
  </link>
  <joint name="leg_left_2_joint" type="revolute">
    <origin rpy="0 1.57079632679 0" xyz="0 0 0.0"/>
    <axis xyz="0 0 1"/>
    <parent link="leg_left_1_link"/>
    <child link="leg_left_2_link"/>
    <dynamics damping="1.0" friction="1.0"/>
    <limit effort="71.4" lower="-0.261799387799" upper="0.523598775598" velocity="3.94"/>
    <safety_controller k_position="100" k_velocity="100" soft_lower_limit="-0.226892802759" soft_upper_limit="0.488692190558"/>
  </joint>
  <link name="leg_left_3_link">
    <inertial>
      <mass value="4.82"/>
      <origin rpy="0.0 0 0.0" xyz="0.1521 -0.0094 0.0152"/>
      <inertia ixx="0.00507885174" ixy="-0.00034076421" ixz="-0.00225952731" iyy="0.02850097402" iyz="0.00003047874" izz="0.02632392239"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://reemc_description/meshes/leg/leg_3_hcrmi.dae" scale="1 1 1.0"/>
      </geometry>
      <material name="LightGrey"/>
    </visual>
    <collision>
      <origin rpy="0 1.57079632679 0" xyz="0.15 0 0.015"/>
      <geometry>
        <box size="0.15 0.10 0.20"/>
      </geometry>
    </collision>
  </link>
  <joint name="leg_left_3_joint" type="revolute">
    <origin rpy="-1.57079632679 0 0" xyz="0 0 0.0"/>
    <axis xyz="0 0 1"/>
    <parent link="leg_left_2_link"/>
    <child link="leg_left_3_link"/>
    <dynamics damping="1.0" friction="1.0"/>
    <limit effort="62.9" lower="-1.74532925199" upper="0.785398163397" velocity="6.0"/>
    <safety_controller k_position="100" k_velocity="100" soft_lower_limit="-1.71042266695" soft_upper_limit="0.750491578358"/>
  </joint>
  <link name="leg_left_4_link">
    <inertial>
      <mass value="3.35"/>
      <origin rpy="0.0 0.0 0.0" xyz="0.1368 0.0099 0.0173"/>
      <inertia ixx="0.00361006492" ixy="0.00012218208" ixz="0.00160208242" iyy="0.02470649045" iyz="0.00008129755" izz="0.02272322206"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://reemc_description/meshes/leg/leg_4_hcrmi.dae" scale="1 1 1.0"/>
      </geometry>
      <material name="LightGrey"/>
    </visual>
    <collision>
      <origin rpy="0 1.57079632679 0" xyz="0.15 0 0.015"/>
      <geometry>
        <box size="0.15 0.10 0.20"/>
      </geometry>
    </collision>
  </link>
  <joint name="leg_left_4_joint" type="revolute">
    <origin rpy="0 0 0" xyz="0.300 0 0.0"/>
    <axis xyz="0 0 1"/>
    <parent link="leg_left_3_link"/>
    <child link="leg_left_4_link"/>
    <dynamics damping="0.1" friction="0"/>
    <dynamics damping="1.0" friction="1.0"/>
    <limit effort="140.4" lower="0.0" upper="2.61799387799" velocity="4.25"/>
    <safety_controller k_position="100" k_velocity="100" soft_lower_limit="0.0349065850399" soft_upper_limit="2.58308729295"/>
  </joint>
  <link name="leg_left_5_link">
    <inertial>
      <mass value="0.68213"/>
      <origin rpy="0.0 0.0 0.0" xyz="-0.00028 0.01119 0.00859"/>
      <inertia ixx="0.00082035023" ixy="0.00000048534" ixz="0.00000226301" iyy="0.00082074105" iyz="-0.00002892247" izz="0.00037614067"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="0.104" radius="0.04"/>
      </geometry>
      <material name="LightGrey"/>
    </visual>
  </link>
  <joint name="leg_left_5_joint" type="revolute">
    <origin rpy="0 0 0" xyz="0.30 0 0.0"/>
    <axis xyz="0 0 1"/>
    <parent link="leg_left_4_link"/>
    <child link="leg_left_5_link"/>
    <dynamics damping="1.0" friction="1.0"/>
    <limit effort="91.6" lower="-1.308996939" upper="0.785398163397" velocity="4.0"/>
    <safety_controller k_position="100" k_velocity="100" soft_lower_limit="-1.27409035396" soft_upper_limit="0.750491578358"/>
  </joint>
  <link name="leg_left_6_link">
    <inertial>
      <mass value="2.00843"/>
      <origin rpy="0.0 0.0 0.0" xyz="0.06111 0.00336 -0.00497"/>
      <inertia ixx="0.00366502559" ixy="0.00020479295" ixz="0.00112439297" iyy="0.00532618026" iyz="0.00012133755" izz="0.00353091463"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://reemc_description/meshes/leg/leg_6_hcrmi.dae" scale="1 1.0 1"/>
      </geometry>
      <material name="LightGrey"/>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0.1 0.01 0.0195 "/>
      <geometry>
        <box size="0.03 0.14 0.210"/>
      </geometry>
    </collision>
  </link>
  <joint name="leg_left_6_joint" type="revolute">
    <origin rpy="1.57079632679 0 0" xyz="0 0 0"/>
    <axis xyz="0 0 1"/>
    <parent link="leg_left_5_link"/>
    <child link="leg_left_6_link"/>
    <dynamics damping="1.0" friction="1.0"/>
    <limit effort="71.48" lower="-0.523598775598" upper="0.261799387799" velocity="3.94"/>
    <safety_controller k_position="100" k_velocity="100" soft_lower_limit="-0.488692190558" soft_upper_limit="0.226892802759"/>
  </joint>
  <link name="left_sole_link">
    <inertial>
      <mass value="0.001"/>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <inertia ixx="0.0" ixy="0.0" ixz="0.0" iyy="0.0" iyz="0.0" izz="0.0"/>
    </inertial>
  </link>
  <joint name="left_sole_joint" type="fixed">
    <!-- origin x taken from link_${side}_6_link/visual/origin/x above -->
    <origin rpy="0.0 -1.57079632679 0.0" xyz="0.117 0.0 0.0"/>
    <axis xyz="0 0 1"/>
    <parent link="leg_left_6_link"/>
    <child link="left_sole_link"/>
    <dynamics damping="1.0" friction="1.0"/>
    <limit effort="60" lower="0.0" upper="0.0" velocity="0"/>
  </joint>
  <gazebo reference="leg_left_1_link">
    <mu1>0.9</mu1>
    <mu2>0.9</mu2>
  </gazebo>
  <gazebo reference="leg_left_2_link">
    <mu1>0.9</mu1>
    <mu2>0.9</mu2>
  </gazebo>
  <gazebo reference="leg_left_3_link">
    <mu1>0.9</mu1>
    <mu2>0.9</mu2>
  </gazebo>
  <gazebo reference="leg_left_4_link">
    <mu1>0.9</mu1>
    <mu2>0.9</mu2>
  </gazebo>
  <gazebo reference="leg_left_5_link">
    <mu1>0.9</mu1>
    <mu2>0.9</mu2>
  </gazebo>
  <!-- contact model for foot surface -->
  <gazebo reference="leg_left_6_link">
    <kp>1000000.0</kp>
    <kd>100.0</kd>
    <mu1>1.0</mu1>
    <mu2>1.0</mu2>
    <fdir1>0 0 1</fdir1>
    <maxVel>1.0</maxVel>
    <minDepth>0.00</minDepth>
    <implicitSpringDamper>1</implicitSpringDamper>
  </gazebo>
  <!--force torque sensor -->
  <!--xacro:reemc_force_torque_sensor name="leg_${side}_6_link"  update_rate="100.0"/-->
  <gazebo reference="leg_left_1_joint">
    <implicitSpringDamper>1</implicitSpringDamper>
  </gazebo>
  <gazebo reference="leg_left_2_joint">
    <implicitSpringDamper>1</implicitSpringDamper>
  </gazebo>
  <gazebo reference="leg_left_3_joint">
    <implicitSpringDamper>1</implicitSpringDamper>
  </gazebo>
  <gazebo reference="leg_left_4_joint">
    <implicitSpringDamper>1</implicitSpringDamper>
  </gazebo>
  <gazebo reference="leg_left_5_joint">
    <implicitSpringDamper>1</implicitSpringDamper>
  </gazebo>
  <gazebo reference="leg_left_6_joint">
    <implicitSpringDamper>1</implicitSpringDamper>
    <provideFeedback>1</provideFeedback>
  </gazebo>
  <transmission name="leg_left_1_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <actuator name="leg_left_1_motor">
      <mechanicalReduction>1.0</mechanicalReduction>
    </actuator>
    <joint name="leg_left_1_joint">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
      <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    </joint>
  </transmission>
  <transmission name="leg_left_2_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <actuator name="leg_left_2_motor">
      <mechanicalReduction>1.0</mechanicalReduction>
    </actuator>
    <joint name="leg_left_2_joint">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
      <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    </joint>
  </transmission>
  <transmission name="leg_left_3_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <actuator name="leg_left_3_motor">
      <mechanicalReduction>1.0</mechanicalReduction>
    </actuator>
    <joint name="leg_left_3_joint">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
      <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    </joint>
  </transmission>
  <transmission name="leg_left_4_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <actuator name="leg_left_4_motor">
      <mechanicalReduction>1.0</mechanicalReduction>
    </actuator>
    <joint name="leg_left_4_joint">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
      <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    </joint>
  </transmission>
  <transmission name="leg_left_5_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <actuator name="leg_left_5_motor">
      <mechanicalReduction>1.0</mechanicalReduction>
    </actuator>
    <joint name="leg_left_5_joint">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
      <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    </joint>
  </transmission>
  <transmission name="leg_left_6_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <actuator name="leg_left_6_motor">
      <mechanicalReduction>1.0</mechanicalReduction>
    </actuator>
    <joint name="leg_left_6_joint">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
      <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    </joint>
  </transmission>
  <link name="left_laser_link">
    <inertial>
      <origin rpy="0 0 0" xyz="0.0 0.0 0.0"/>
      <mass value="0.001"/>
      <inertia ixx="3.7174E-05" ixy="4.9927E-08" ixz="1.1015E-05" iyy="4.2412E-05" iyz="-9.8165E-09" izz="4.167E-05"/>
    </inertial>
    <!--    <visual>
         <origin xyz="0.0 0 0.05" rpy="0 0 ${90.0*deg_to_rad}" />
        <geometry>
          <mesh filename="package://reemc_description/meshes/sensors/urg-04lx-ug01.stl" scale="0.001 0.001 0.001"/>
        </geometry>
      <material name="DarkGrey" />
      </visual>
  -->
  </link>
  <joint name="left_laser_joint" type="fixed">
    <axis xyz="0 1 0"/>
    <origin rpy="0.0 0.0 0.523598775598" xyz="0.085 0.01 0.073"/>
    <parent link="left_sole_link"/>
    <child link="left_laser_link"/>
    <dynamics damping="1" friction="1.0"/>
    <limit effort="60" lower="0" upper="0" velocity="20"/>
  </joint>
  <gazebo reference="left_laser_link">
    <sensor name="left_laser_sensor" type="ray">
      <pose>0 0 0 0 0 0</pose>
      <visualize>false</visualize>
      <update_rate>10.0</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>360</samples>
            <resolution>1</resolution>
            <min_angle>-1.57079632679</min_angle>
            <max_angle>1.57079632679</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.02</min>
          <max>5.6</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <plugin filename="libgazebo_ros_laser.so" name="gazebo_ros_laser">
        <frameName>/left_laser_link</frameName>
        <topicName>left_scan</topicName>
        <gaussianNoise>0.03</gaussianNoise>
        <hokuyoMinIntensity>101</hokuyoMinIntensity>
        <updateRate>10.0</updateRate>
        <alwaysOn>true</alwaysOn>
      </plugin>
    </sensor>
  </gazebo>
  <!--************************-->
  <!--        TORSO_1         -->
  <!--************************-->
  <link name="torso_1_link">
    <inertial>
      <origin rpy="0 0 0" xyz="-0.00011 -0.00002 -0.00669"/>
      <mass value="1.58341"/>
      <!-- NOTE: Assuming cylinder with length="0.295" radius="0.0465-->
      <inertia ixx="0.00071606" ixy="-1.6247E-07" ixz="4.1281E-07" iyy="0.00082439" iyz="-1.4599E-10" izz="0.00090588"/>
    </inertial>
    <visual>
      <origin rpy="1.57079632679 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://reemc_description/meshes/torso/torso_1_hcrmi.dae"/>
      </geometry>
      <material name="DarkGrey"/>
    </visual>
  </link>
  <joint name="torso_1_joint" type="revolute">
    <parent link="base_link"/>
    <child link="torso_1_link"/>
    <origin rpy="0 0 0" xyz="0 0 0.14353"/>
    <axis xyz="0 0 1"/>
    <limit effort="78.0" lower="-1.308996939" upper="1.308996939" velocity="5.4"/>
    <dynamics damping="1.0" friction="1.0"/>
    <safety_controller k_position="20" k_velocity="20" soft_lower_limit="-1.25409035396" soft_upper_limit="1.25409035396"/>
  </joint>
  <!--************************-->
  <!--        TORSO_2         -->
  <!--************************-->
  <link name="torso_2_link">
    <inertial>
      <origin rpy="0 0 0" xyz="-0.0183 -0.1498 0.0016"/>
      <mass value="14.62"/>
      <!-- NOTE: Assuming box with d=0.2, w=0.295, h=0.254 -->
      <inertia ixx="0.025688" ixy="0.00030269" ixz="-9.89E-05" iyy="0.023995" iyz="0.00011751" izz="0.01786"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://reemc_description/meshes/torso/torso_2_hcrmi.dae"/>
      </geometry>
      <material name="LightGrey"/>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://reemc_description/meshes/torso/torso_2_collision.stl"/>
      </geometry>
    </collision>
  </link>
  <joint name="torso_2_joint" type="revolute">
    <parent link="torso_1_link"/>
    <child link="torso_2_link"/>
    <origin rpy="-1.57079632679 0 0" xyz="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit effort="78.0" lower="-0.261799387799" upper="0.785398163397" velocity="5.4"/>
    <dynamics damping="1.0" friction="1.0"/>
    <safety_controller k_position="20" k_velocity="20" soft_lower_limit="-0.206892802759" soft_upper_limit="0.730491578358"/>
  </joint>
  <gazebo reference="torso_1_link">
    <mu1>0.9</mu1>
    <mu2>0.9</mu2>
  </gazebo>
  <gazebo reference="torso_2_link">
    <mu1>0.9</mu1>
    <mu2>0.9</mu2>
  </gazebo>
  <gazebo reference="torso_1_joint">
    <implicitSpringDamper>1</implicitSpringDamper>
  </gazebo>
  <gazebo reference="torso_2_joint">
    <implicitSpringDamper>1</implicitSpringDamper>
    <provideFeedback>1</provideFeedback>
  </gazebo>
  <link name="torso_sonar_01_link">
    <inertial>
      <mass value="0.001"/>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.000001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>
  <joint name="torso_sonar_01_joint" type="fixed">
    <origin rpy="0.0 0.0 0.349065850399" xyz="0.10 -0.029 0"/>
    <axis xyz="0 0 1"/>
    <parent link="torso_2_link"/>
    <child link="torso_sonar_01_link"/>
  </joint>
  <gazebo reference="torso_sonar_01_link">
    <sensor name="torso_sonar_01" type="ray">
      <pose>0 0 0 0 0 0</pose>
      <update_rate>5</update_rate>
      <visualize>false</visualize>
      <ray>
        <scan>
          <horizontal>
            <samples>5</samples>
            <resolution>1</resolution>
            <min_angle>-0.17455</min_angle>
            <max_angle>0.17455</max_angle>
          </horizontal>
          <vertical>
            <samples>5</samples>
            <resolution>1</resolution>
            <min_angle>-0.17455</min_angle>
            <max_angle>0.17455</max_angle>
          </vertical>
        </scan>
        <range>
          <min>0.005</min>
          <max>0.7</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <plugin filename="libgazebo_ros_range.so" name="gazebo_ros_range">
        <gaussianNoise>0.005</gaussianNoise>
        <alwaysOn>true</alwaysOn>
        <updateRate>5</updateRate>
        <topicName>sonar_torso</topicName>
        <frameName>torso_sonar_01_link</frameName>
        <minRange>0.005</minRange>
        <maxRange>0.7</maxRange>
        <fov>0.3491</fov>
        <radiation>ultrasound</radiation>
      </plugin>
    </sensor>
  </gazebo>
  <link name="torso_sonar_02_link">
    <inertial>
      <mass value="0.001"/>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.000001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>
  <joint name="torso_sonar_02_joint" type="fixed">
    <origin rpy="0.0 0.0 0.0" xyz="0.15 -0.18 0"/>
    <axis xyz="0 0 1"/>
    <parent link="torso_2_link"/>
    <child link="torso_sonar_02_link"/>
  </joint>
  <gazebo reference="torso_sonar_02_link">
    <sensor name="torso_sonar_02" type="ray">
      <pose>0 0 0 0 0 0</pose>
      <update_rate>5</update_rate>
      <visualize>false</visualize>
      <ray>
        <scan>
          <horizontal>
            <samples>5</samples>
            <resolution>1</resolution>
            <min_angle>-0.17455</min_angle>
            <max_angle>0.17455</max_angle>
          </horizontal>
          <vertical>
            <samples>5</samples>
            <resolution>1</resolution>
            <min_angle>-0.17455</min_angle>
            <max_angle>0.17455</max_angle>
          </vertical>
        </scan>
        <range>
          <min>0.005</min>
          <max>0.7</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <plugin filename="libgazebo_ros_range.so" name="gazebo_ros_range">
        <gaussianNoise>0.005</gaussianNoise>
        <alwaysOn>true</alwaysOn>
        <updateRate>5</updateRate>
        <topicName>sonar_torso</topicName>
        <frameName>torso_sonar_02_link</frameName>
        <minRange>0.005</minRange>
        <maxRange>0.7</maxRange>
        <fov>0.3491</fov>
        <radiation>ultrasound</radiation>
      </plugin>
    </sensor>
  </gazebo>
  <link name="torso_sonar_04_link">
    <inertial>
      <mass value="0.001"/>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.000001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>
  <joint name="torso_sonar_04_joint" type="fixed">
    <origin rpy="0.0 3.14159265359 0.0" xyz="-0.17 -0.029 0.0"/>
    <axis xyz="0 0 1"/>
    <parent link="torso_2_link"/>
    <child link="torso_sonar_04_link"/>
  </joint>
  <gazebo reference="torso_sonar_04_link">
    <sensor name="torso_sonar_04" type="ray">
      <pose>0 0 0 0 0 0</pose>
      <update_rate>5</update_rate>
      <visualize>false</visualize>
      <ray>
        <scan>
          <horizontal>
            <samples>5</samples>
            <resolution>1</resolution>
            <min_angle>-0.17455</min_angle>
            <max_angle>0.17455</max_angle>
          </horizontal>
          <vertical>
            <samples>5</samples>
            <resolution>1</resolution>
            <min_angle>-0.17455</min_angle>
            <max_angle>0.17455</max_angle>
          </vertical>
        </scan>
        <range>
          <min>0.005</min>
          <max>0.7</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <plugin filename="libgazebo_ros_range.so" name="gazebo_ros_range">
        <gaussianNoise>0.005</gaussianNoise>
        <alwaysOn>true</alwaysOn>
        <updateRate>5</updateRate>
        <topicName>sonar_torso</topicName>
        <frameName>torso_sonar_04_link</frameName>
        <minRange>0.005</minRange>
        <maxRange>0.7</maxRange>
        <fov>0.3491</fov>
        <radiation>ultrasound</radiation>
      </plugin>
    </sensor>
  </gazebo>
  <transmission name="torso_trans">
    <type>transmission_interface/DifferentialTransmission</type>
    <actuator name="torso_1_motor">
      <role>actuator1</role>
      <mechanicalReduction>1.0</mechanicalReduction>
    </actuator>
    <actuator name="torso_2_motor">
      <role>actuator2</role>
      <mechanicalReduction>1.0</mechanicalReduction>
    </actuator>
    <joint name="torso_1_joint">
      <role>joint1</role>
      <offset>0.0</offset>
      <mechanicalReduction>1.0</mechanicalReduction>
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
      <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    </joint>
    <joint name="torso_2_joint">
      <role>joint2</role>
      <offset>0.0</offset>
      <mechanicalReduction>1.0</mechanicalReduction>
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
      <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    </joint>
  </transmission>
  <link name="head_1_link">
    <inertial>
      <origin rpy="0 0 0" xyz="0.014823 -0.000253 -0.008383"/>
      <mass value="0.646763"/>
      <!-- NOTE: Assuming cylinder with r=0.06, h=0.08 -->
      <!--         <inertia ixx="0.0014125"  ixy="0"  ixz="0" iyy="0.0014125" iyz="0" izz="0.0017738" /> -->
      <inertia ixx="0.001175" ixy="-0.000003" ixz="0.000012" iyy="0.001273" iyz="-0.000008" izz="0.001132"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="-0.03 0 0"/>
      <!--TODO: Remove offset when good meshes arrive-->
      <geometry>
        <mesh filename="package://reemc_description/meshes/head/head_1.dae"/>
      </geometry>
      <material name="DarkGrey"/>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="-0.03 0 0"/>
      <geometry>
        <mesh filename="package://reemc_description/meshes/head/head_1_collision.stl"/>
      </geometry>
    </collision>
  </link>
  <joint name="head_1_joint" type="revolute">
    <parent link="torso_2_link"/>
    <child link="head_1_link"/>
    <origin rpy="1.57079632679 0 0" xyz="0.0 -0.42316 0"/>
    <axis xyz="0 0 1"/>
    <limit effort="3.4" lower="-1.308996939" upper="1.308996939" velocity="3.0"/>
    <dynamics damping="0.5" friction="1.0"/>
    <safety_controller k_position="20" k_velocity="20" soft_lower_limit="-1.25409035396" soft_upper_limit="1.25409035396"/>
  </joint>
  <gazebo reference="head_1_link">
    <mu1>0.9</mu1>
    <mu2>0.9</mu2>
  </gazebo>
  <gazebo reference="head_1_joint">
    <implicitSpringDamper>1</implicitSpringDamper>
  </gazebo>
  <link name="head_2_link">
    <inertial>
      <origin rpy="0 0 0" xyz="-0.023981 -0.100591 0.000294"/>
      <mass value="0.920999"/>
      <!-- NOTE: Assuming box with d=0.28433, w=0.20302, h=0.26741 -->
      <!--         <inertia ixx="0.011353"  ixy="0"  ixz="0" iyy="0.012294" iyz="0" izz="0.015345" /> -->
      <!--NOTE: Low (0.1 or lower) diagonal inertia values make Gazebo 1.0.2 simulation unstable when head sensors are added.-->
      <inertia ixx="0.005972" ixy="-0.000046" ixz="0.000006" iyy="0.006327" iyz="0.000030" izz="0.006213"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://reemc_description/meshes/head/head_2_v1_hcrmi.dae"/>
      </geometry>
      <material name="LightGrey"/>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="-0.03 0 0"/>
      <geometry>
        <mesh filename="package://reemc_description/meshes/head/head_2_collision.stl"/>
      </geometry>
    </collision>
  </link>
  <joint name="head_2_joint" type="revolute">
    <parent link="head_1_link"/>
    <child link="head_2_link"/>
    <origin rpy="-1.57079632679 0.0  0" xyz="0.0445 0 0"/>
    <axis xyz="0 0 1"/>
    <limit effort="3.26" lower="-0.261799387799" upper="0.785398163397" velocity="3.0"/>
    <dynamics damping="0.5" friction="1.0"/>
    <safety_controller k_position="20" k_velocity="20" soft_lower_limit="-0.206892802759" soft_upper_limit="0.750491578358"/>
  </joint>
  <gazebo reference="head_2_link">
    <mu1>0.9</mu1>
    <mu2>0.9</mu2>
  </gazebo>
  <gazebo reference="head_2_joint">
    <implicitSpringDamper>1</implicitSpringDamper>
  </gazebo>
  <link name="head_sonar_03_link">
    <inertial>
      <mass value="0.001"/>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.000001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>
  <joint name="head_sonar_03_joint" type="fixed">
    <origin rpy="1.57 0.0 0.0" xyz="0.11 -0.195 0.0"/>
    <axis xyz="0 0 1"/>
    <parent link="head_2_link"/>
    <child link="head_sonar_03_link"/>
  </joint>
  <gazebo reference="head_sonar_03_link">
    <sensor name="head_sonar_03" type="ray">
      <pose>0 0 0 0 0 0</pose>
      <update_rate>5</update_rate>
      <visualize>false</visualize>
      <ray>
        <scan>
          <horizontal>
            <samples>5</samples>
            <resolution>1</resolution>
            <min_angle>-0.17455</min_angle>
            <max_angle>0.17455</max_angle>
          </horizontal>
          <vertical>
            <samples>5</samples>
            <resolution>1</resolution>
            <min_angle>-0.17455</min_angle>
            <max_angle>0.17455</max_angle>
          </vertical>
        </scan>
        <range>
          <min>0.0</min>
          <max>0.6</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <plugin filename="libgazebo_ros_range.so" name="gazebo_ros_range">
        <gaussianNoise>0.005</gaussianNoise>
        <alwaysOn>true</alwaysOn>
        <updateRate>5</updateRate>
        <topicName>sonar_torso</topicName>
        <frameName>head_sonar_03_link</frameName>
        <minRange>0.0</minRange>
        <maxRange>0.6</maxRange>
        <fov>0.3491</fov>
        <radiation>ultrasound</radiation>
      </plugin>
    </sensor>
  </gazebo>
  <transmission name="head">
    <type>pal_transmissions/HeadTransmission</type>
    <actuator name="head_1_motor">
      <role>independent</role>
      <mechanicalReduction>1.0</mechanicalReduction>
    </actuator>
    <actuator name="head_2_motor">
      <role>dependent</role>
      <mechanicalReduction>1.0</mechanicalReduction>
    </actuator>
    <joint name="head_1_joint">
      <role>independent</role>
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </joint>
    <joint name="head_2_joint">
      <role>dependent</role>
      <limits>
        <key> -0.872664625997 </key>
        <max> 0.244346095279 </max>
      </limits>
      <limits>
        <key> -0.698131700798 </key>
        <max> 0.314159265359 </max>
      </limits>
      <limits>
        <key> -0.523598775598 </key>
        <max> 0.471238898038 </max>
      </limits>
      <limits>
        <key> -0.349065850399 </key>
        <max> 0.680678408278 </max>
      </limits>
      <limits>
        <key> -0.174532925199 </key>
        <max> 0.785398163397 </max>
      </limits>
      <limits>
        <key> 0.174532925199 </key>
        <max> 0.785398163397 </max>
      </limits>
      <limits>
        <key> 0.349065850399 </key>
        <max> 0.680678408278 </max>
      </limits>
      <limits>
        <key> 0.523598775598 </key>
        <max> 0.471238898038 </max>
      </limits>
      <limits>
        <key> 0.698131700798 </key>
        <max> 0.314159265359 </max>
      </limits>
      <limits>
        <key> 0.872664625997 </key>
        <max> 0.244346095279 </max>
      </limits>
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </joint>
  </transmission>
  <!--************************-->
  <!--        SHOULDER        -->
  <!--************************-->
  <link name="arm_right_1_link">
    <inertial>
      <origin rpy="0 0 0" xyz="0.00701 -0.00011 0.18627"/>
      <mass value="1.53614"/>
      <!--        <inertia ixx="0"  ixy="0"  ixz="0" iyy="0" iyz="0" izz="0" /> -->
      <!--         NOTE: Assuming cylinder centered at CoM with r=0.05, h=0.05 with axis parallel to x coordinate -->
      <!--        <inertia ixx="0.0026850"  ixy="0"  ixz="0" iyy="0.0017900" iyz="0" izz="0.0017900" /> -->
      <inertia ixx="0.0021367" ixy="2.7772E-06" ixz="0.00016462" iyy="0.0015748" iyz="-3.7464E-06" izz="0.0015138"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://reemc_description/meshes/arm/arm_1.dae" scale="1 1 -1.0"/>
      </geometry>
      <material name="DarkGrey"/>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://reemc_description/meshes/arm/arm_1_collision.stl" scale="1 1 -1.0"/>
      </geometry>
    </collision>
  </link>
  <joint name="arm_right_1_joint" type="revolute">
    <parent link="torso_2_link"/>
    <child link="arm_right_1_link"/>
    <origin rpy="-2.87979326579 0 0" xyz="0 -0.31827 0"/>
    <axis xyz="0 0 1"/>
    <limit effort="44.64" lower="-0.785398163397" upper="3.14159265359" velocity="2.7"/>
    <dynamics damping="1.0" friction="1.0"/>
    <safety_controller k_position="20" k_velocity="20" soft_lower_limit="-0.730491578358" soft_upper_limit="3.08668606855"/>
  </joint>
  <link name="arm_right_2_link">
    <inertial>
      <origin rpy="0 0 0" xyz="0.0252 -0.0627 0.0175"/>
      <mass value="1.66"/>
      <!--<inertia ixx="0"  ixy="0"  ixz="0" iyy="0" iyz="0" izz="0" />-->
      <!-- NOTE: Assuming cylinder centered at CoM with r=0.05, h=0.05 with axis parallel to y coordinate -->
      <!-- <inertia ixx="0.0013417"  ixy="0"  ixz="0" iyy="0.0020125" iyz="0" izz="0.0013417" /> -->
      <inertia ixx="0.001836" ixy="0.00012127" ixz="-6.919E-05" iyy="0.0017584" iyz="0.00017005" izz="0.002477"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://reemc_description/meshes/arm/arm_2.dae" scale="1 1 -1.0"/>
      </geometry>
      <material name="DarkGrey"/>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://reemc_description/meshes/arm/arm_2_collision.stl" scale="1 1 -1.0"/>
      </geometry>
    </collision>
  </link>
  <joint name="arm_right_2_joint" type="revolute">
    <parent link="arm_right_1_link"/>
    <child link="arm_right_2_link"/>
    <origin rpy="-1.57079632679 -1.308996939 1.57079632679" xyz="0 0 0.21749"/>
    <axis xyz="0 0 1"/>
    <limit effort="22.32" lower="-0.261799387799" upper="2.09439510239" velocity="3.66"/>
    <dynamics damping="1.0" friction="1.0"/>
    <safety_controller k_position="20" k_velocity="20" soft_lower_limit="-0.206892802759" soft_upper_limit="2.03948851735"/>
  </joint>
  <link name="arm_right_3_link">
    <inertial>
      <origin rpy="0 0 0" xyz="0.0024 -0.0245 0.0573"/>
      <mass value="1.71"/>
      <!--<inertia ixx="0"  ixy="0"  ixz="0" iyy="0" iyz="0" izz="0" />-->
      <!-- NOTE: Assuming cylinder centered at CoM with r=0.05, h=0.05 with axis parallel to x coordinate -->
      <inertia ixx="0.0026597" ixy="6.9102E-05" ixz="0.00017249" iyy="0.0020207" iyz="-0.00011657" izz="0.0020875"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://reemc_description/meshes/arm/arm_3.dae" scale="1 1 -1.0"/>
      </geometry>
      <material name="DarkGrey"/>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://reemc_description/meshes/arm/arm_3_collision.stl" scale="1 1 -1.0"/>
      </geometry>
    </collision>
  </link>
  <joint name="arm_right_3_joint" type="revolute">
    <parent link="arm_right_2_link"/>
    <child link="arm_right_3_link"/>
    <origin rpy="1.57079632679 0 0" xyz="0.02 -0.142 0"/>
    <axis xyz="0 0 1"/>
    <limit effort="17.86" lower="-2.35619449019" upper="2.74889357189" velocity="4.58"/>
    <dynamics damping="1.0" friction="1.0"/>
    <safety_controller k_position="20" k_velocity="20" soft_lower_limit="-2.30128790515" soft_upper_limit="2.69398698685"/>
  </joint>
  <!--************************-->
  <!--        ELBOW           -->
  <!--************************-->
  <link name="arm_right_4_link">
    <inertial>
      <origin rpy="0 0 0" xyz="-0.0342 0.0241 -0.0165"/>
      <mass value="1.27"/>
      <!--<inertia ixx="0"  ixy="0"  ixz="0" iyy="0" iyz="0" izz="0" />-->
      <!-- NOTE: Assuming cylinder centered at CoM with r=0.05, h=0.05 with axis parallel to x coordinate -->
      <!-- <inertia ixx="0.0012663"  ixy="0"  ixz="0" iyy="0.00084417" iyz="0" izz="0.00084417" /> -->
      <inertia ixx="0.0012786" ixy="0.00011984" ixz="-0.00017046" iyy="0.0015715" iyz="6.8669E-05" izz="0.0022145"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://reemc_description/meshes/arm/arm_4.dae" scale="1 1 -1.0"/>
      </geometry>
      <material name="DarkGrey"/>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://reemc_description/meshes/arm/arm_4_collision.stl" scale="1 1 -1.0"/>
      </geometry>
    </collision>
  </link>
  <joint name="arm_right_4_joint" type="revolute">
    <parent link="arm_right_3_link"/>
    <child link="arm_right_4_link"/>
    <origin rpy="0 1.57079632679 0" xyz="0 -0.02 0.088"/>
    <axis xyz="0 0 1"/>
    <limit effort="17.86" lower="0" upper="2.26892802759" velocity="4.58"/>
    <dynamics damping="1.0" friction="1.0"/>
    <safety_controller k_position="20" k_velocity="20" soft_lower_limit="0.0549065850399" soft_upper_limit="2.21402144255"/>
  </joint>
  <gazebo reference="arm_right_1_link">
    <mu1>0.9</mu1>
    <mu2>0.9</mu2>
  </gazebo>
  <gazebo reference="arm_right_2_link">
    <mu1>0.9</mu1>
    <mu2>0.9</mu2>
  </gazebo>
  <gazebo reference="arm_right_3_link">
    <mu1>0.9</mu1>
    <mu2>0.9</mu2>
  </gazebo>
  <gazebo reference="arm_right_4_link">
    <mu1>0.9</mu1>
    <mu2>0.9</mu2>
  </gazebo>
  <gazebo reference="arm_right_1_joint">
    <implicitSpringDamper>1</implicitSpringDamper>
  </gazebo>
  <gazebo reference="arm_right_2_joint">
    <implicitSpringDamper>1</implicitSpringDamper>
  </gazebo>
  <gazebo reference="arm_right_3_joint">
    <implicitSpringDamper>1</implicitSpringDamper>
  </gazebo>
  <gazebo reference="arm_right_4_joint">
    <implicitSpringDamper>1</implicitSpringDamper>
  </gazebo>
  <!--************************-->
  <!--        WRIST           -->
  <!--************************-->
  <link name="arm_right_5_link">
    <inertial>
      <origin rpy="0 0 0" xyz="0.0067 0.0071 0.0705"/>
      <mass value="1.23"/>
      <!-- <inertia ixx="0"  ixy="0"  ixz="0" iyy="0" iyz="0" izz="0" /> -->
      <!-- NOTE: Assuming cylinder centered at CoM with r=0.05, h=0.1 with axis parallel to z coordinate -->
      <!--         <inertia ixx="0.0027096"  ixy="0"  ixz="0" iyy="0.0027096" iyz="0" izz="0.0023225" /> -->
      <inertia ixx="0.0011556" ixy="-1.8058E-07" ixz="2.2399E-06" iyy="0.00091494" iyz="2.2113E-07" izz="0.00065799"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://reemc_description/meshes/arm/arm_5.dae" scale="1 1 -1.0"/>
      </geometry>
      <material name="LightGrey"/>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://reemc_description/meshes/arm/arm_5_collision.stl" scale="1 1 -1.0"/>
      </geometry>
    </collision>
  </link>
  <joint name="arm_right_5_joint" type="revolute">
    <parent link="arm_right_4_link"/>
    <child link="arm_right_5_link"/>
    <origin rpy="0 -1.57079632679 0" xyz="-0.088 0.02 0"/>
    <axis xyz="0 0 1"/>
    <limit effort="3" lower="-2.09439510239" upper="2.09439510239" velocity="1.95"/>
    <!--TODO: Check effort value!-->
    <dynamics damping="1.0" friction="1.0"/>
    <safety_controller k_position="20" k_velocity="20" soft_lower_limit="-2.03948851735" soft_upper_limit="2.03948851735"/>
  </joint>
  <link name="arm_right_6_link">
    <inertial>
      <origin rpy="0 0 0" xyz="-0.00005 -0.00252 7e-05"/>
      <mass value="0.41597"/>
      <!-- <inertia ixx="0"  ixy="0"  ixz="0" iyy="0" iyz="0" izz="0" /> -->
      <!-- NOTE: Assuming cylinder centered at CoM with r=0.02, h=0.08 with axis parallel to z coordinate -->
      <!--         <inertia ixx="0.0001615"  ixy="0"  ixz="0" iyy="0.0001615" iyz="0" izz="0.000051" /> -->
      <inertia ixx="3.6511E-05" ixy="-2.3773E-09" ixz="1.33E-07" iyy="2.6707E-05" iyz="-1.6779E-07" izz="3.4209E-05"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://reemc_description/meshes/arm/arm_6.dae" scale="1 1 -1.0"/>
      </geometry>
      <material name="LightGrey"/>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="0.0385" radius="0.04"/>
      </geometry>
    </collision>
  </link>
  <joint name="arm_right_6_joint" type="revolute">
    <parent link="arm_right_5_link"/>
    <child link="arm_right_6_link"/>
    <origin rpy="-1.57079632679 0 0" xyz="0 0 0.15"/>
    <axis xyz="0 0 1"/>
    <limit effort="6.6" lower="-1.41371669412" upper="1.41371669412" velocity="1.76"/>
    <dynamics damping="1.0" friction="1.0"/>
    <safety_controller k_position="20" k_velocity="20" soft_lower_limit="-1.42862327916" soft_upper_limit="1.35881010908"/>
  </joint>
  <link name="arm_right_7_link">
    <inertial>
      <!--TODO: Verify these inertial parameters-->
      <origin rpy="0 0 0" xyz="6.7314E-06 -0.00013277 -0.029334"/>
      <mass value="0.10051"/>
      <!-- <inertia ixx="0"  ixy="0"  ixz="0" iyy="0" iyz="0" izz="0" /> -->
      <!-- NOTE: Assuming point mass -->
      <!--         <inertia ixx="0"  ixy="0"  ixz="0" iyy="0" iyz="0" izz="0" /> -->
      <inertia ixx="0.0000123718" ixy="0.00000000002" ixz="0.00000002403" iyy="0.00001242132" iyz="0.00000000001" izz="0.0000059514"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0.046"/>
      <geometry>
        <cylinder length="0.005" radius="0.02"/>
      </geometry>
      <material name="DarkGrey"/>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0.0345"/>
      <geometry>
        <cylinder length="0.023" radius="0.02"/>
      </geometry>
    </collision>
  </link>
  <joint name="arm_right_7_joint" type="revolute">
    <parent link="arm_right_6_link"/>
    <child link="arm_right_7_link"/>
    <origin rpy="1.57079632679 0 0" xyz="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit effort="6.6" lower="-2.09439510239" upper="2.09439510239" velocity="1.76"/>
    <dynamics damping="1.0" friction="1.0"/>
    <safety_controller k_position="20" k_velocity="20" soft_lower_limit="-2.03948851735" soft_upper_limit="2.03948851735"/>
  </joint>
  <gazebo reference="arm_right_5_link">
    <mu1>0.9</mu1>
    <mu2>0.9</mu2>
  </gazebo>
  <gazebo reference="arm_right_6_link">
    <mu1>0.9</mu1>
    <mu2>0.9</mu2>
  </gazebo>
  <gazebo reference="arm_right_7_link">
    <mu1>0.9</mu1>
    <mu2>0.9</mu2>
  </gazebo>
  <gazebo reference="arm_right_5_joint">
    <implicitSpringDamper>1</implicitSpringDamper>
  </gazebo>
  <gazebo reference="arm_right_6_joint">
    <implicitSpringDamper>1</implicitSpringDamper>
  </gazebo>
  <gazebo reference="arm_right_7_joint">
    <implicitSpringDamper>1</implicitSpringDamper>
    <provideFeedback>1</provideFeedback>
  </gazebo>
  <transmission name="arm_right_5_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <actuator name="arm_right_5_motor">
      <mechanicalReduction>1.0</mechanicalReduction>
    </actuator>
    <joint name="arm_right_5_joint">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </joint>
  </transmission>
  <transmission name="wrist_right_trans">
    <type>transmission_interface/DifferentialTransmission</type>
    <actuator name="arm_right_6_motor">
      <role>actuator1</role>
      <mechanicalReduction>-1.0</mechanicalReduction>
    </actuator>
    <actuator name="arm_right_7_motor">
      <role>actuator2</role>
      <mechanicalReduction>1.0</mechanicalReduction>
    </actuator>
    <joint name="arm_right_6_joint">
      <role>joint1</role>
      <offset>0.0</offset>
      <mechanicalReduction>-1.0</mechanicalReduction>
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </joint>
    <joint name="arm_right_7_joint">
      <role>joint2</role>
      <offset>0.0</offset>
      <mechanicalReduction>-1.0</mechanicalReduction>
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </joint>
  </transmission>
  <!--***********************-->
  <!--        TOOL           -->
  <!--***********************-->
  <link name="arm_right_tool_link">
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0" ixy="0.0" ixz="0.0" iyy="0.0" iyz="0.0" izz="0.0"/>
    </inertial>
    <visual>
      <origin rpy="0 1.57079632679 0" xyz="0.001 0 0"/>
      <geometry>
        <cylinder length="0.005" radius="0.005"/>
      </geometry>
      <material name="LightGrey"/>
    </visual>
    <collision>
      <origin rpy="0 1.57079632679 0" xyz="0.001 0 0"/>
      <geometry>
        <cylinder length="0.005" radius="0.005"/>
      </geometry>
    </collision>
  </link>
  <joint name="arm_right_tool_joint" type="fixed">
    <parent link="arm_right_7_link"/>
    <child link="arm_right_tool_link"/>
    <origin rpy="1.57079632679 -1.57079632679 0" xyz="0 0 0.046"/>
  </joint>
  <transmission name="arm_right_1_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <actuator name="arm_right_1_motor">
      <mechanicalReduction>1.0</mechanicalReduction>
    </actuator>
    <joint name="arm_right_1_joint">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
      <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    </joint>
  </transmission>
  <transmission name="arm_right_2_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <actuator name="arm_right_2_motor">
      <mechanicalReduction>1.0</mechanicalReduction>
    </actuator>
    <joint name="arm_right_2_joint">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
      <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    </joint>
  </transmission>
  <transmission name="arm_right_3_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <actuator name="arm_right_3_motor">
      <mechanicalReduction>1.0</mechanicalReduction>
    </actuator>
    <joint name="arm_right_3_joint">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
      <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    </joint>
  </transmission>
  <transmission name="arm_right_4_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <actuator name="arm_right_4_motor">
      <mechanicalReduction>1.0</mechanicalReduction>
    </actuator>
    <joint name="arm_right_4_joint">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
      <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    </joint>
  </transmission>
  <!--************************-->
  <!--        SHOULDER        -->
  <!--************************-->
  <link name="arm_left_1_link">
    <inertial>
      <origin rpy="0 0 0" xyz="0.00701 -0.00011 -0.18627"/>
      <mass value="1.53614"/>
      <!--        <inertia ixx="0"  ixy="0"  ixz="0" iyy="0" iyz="0" izz="0" /> -->
      <!--         NOTE: Assuming cylinder centered at CoM with r=0.05, h=0.05 with axis parallel to x coordinate -->
      <!--        <inertia ixx="0.0026850"  ixy="0"  ixz="0" iyy="0.0017900" iyz="0" izz="0.0017900" /> -->
      <inertia ixx="0.0021367" ixy="2.7772E-06" ixz="0.00016462" iyy="0.0015748" iyz="-3.7464E-06" izz="0.0015138"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://reemc_description/meshes/arm/arm_1.dae" scale="1 1 1.0"/>
      </geometry>
      <material name="DarkGrey"/>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://reemc_description/meshes/arm/arm_1_collision.stl" scale="1 1 1.0"/>
      </geometry>
    </collision>
  </link>
  <joint name="arm_left_1_joint" type="revolute">
    <parent link="torso_2_link"/>
    <child link="arm_left_1_link"/>
    <origin rpy="2.87979326579 0 0" xyz="0 -0.31827 0"/>
    <axis xyz="0 0 1"/>
    <limit effort="44.64" lower="-0.785398163397" upper="3.14159265359" velocity="2.7"/>
    <dynamics damping="1.0" friction="1.0"/>
    <safety_controller k_position="20" k_velocity="20" soft_lower_limit="-0.730491578358" soft_upper_limit="3.08668606855"/>
  </joint>
  <link name="arm_left_2_link">
    <inertial>
      <origin rpy="0 0 0" xyz="0.0252 -0.0627 -0.0175"/>
      <mass value="1.66"/>
      <!--<inertia ixx="0"  ixy="0"  ixz="0" iyy="0" iyz="0" izz="0" />-->
      <!-- NOTE: Assuming cylinder centered at CoM with r=0.05, h=0.05 with axis parallel to y coordinate -->
      <!-- <inertia ixx="0.0013417"  ixy="0"  ixz="0" iyy="0.0020125" iyz="0" izz="0.0013417" /> -->
      <inertia ixx="0.001836" ixy="0.00012127" ixz="-6.919E-05" iyy="0.0017584" iyz="0.00017005" izz="0.002477"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://reemc_description/meshes/arm/arm_2.dae" scale="1 1 1.0"/>
      </geometry>
      <material name="DarkGrey"/>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://reemc_description/meshes/arm/arm_2_collision.stl" scale="1 1 1.0"/>
      </geometry>
    </collision>
  </link>
  <joint name="arm_left_2_joint" type="revolute">
    <parent link="arm_left_1_link"/>
    <child link="arm_left_2_link"/>
    <origin rpy="1.57079632679 1.308996939 1.57079632679" xyz="0 0 -0.21749"/>
    <axis xyz="0 0 1"/>
    <limit effort="22.32" lower="-0.261799387799" upper="2.09439510239" velocity="3.66"/>
    <dynamics damping="1.0" friction="1.0"/>
    <safety_controller k_position="20" k_velocity="20" soft_lower_limit="-0.206892802759" soft_upper_limit="2.03948851735"/>
  </joint>
  <link name="arm_left_3_link">
    <inertial>
      <origin rpy="0 0 0" xyz="0.0024 -0.0245 -0.0573"/>
      <mass value="1.71"/>
      <!--<inertia ixx="0"  ixy="0"  ixz="0" iyy="0" iyz="0" izz="0" />-->
      <!-- NOTE: Assuming cylinder centered at CoM with r=0.05, h=0.05 with axis parallel to x coordinate -->
      <inertia ixx="0.0026597" ixy="6.9102E-05" ixz="0.00017249" iyy="0.0020207" iyz="-0.00011657" izz="0.0020875"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://reemc_description/meshes/arm/arm_3.dae" scale="1 1 1.0"/>
      </geometry>
      <material name="DarkGrey"/>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://reemc_description/meshes/arm/arm_3_collision.stl" scale="1 1 1.0"/>
      </geometry>
    </collision>
  </link>
  <joint name="arm_left_3_joint" type="revolute">
    <parent link="arm_left_2_link"/>
    <child link="arm_left_3_link"/>
    <origin rpy="-1.57079632679 0 0" xyz="0.02 -0.142 0"/>
    <axis xyz="0 0 1"/>
    <limit effort="17.86" lower="-2.35619449019" upper="2.74889357189" velocity="4.58"/>
    <dynamics damping="1.0" friction="1.0"/>
    <safety_controller k_position="20" k_velocity="20" soft_lower_limit="-2.30128790515" soft_upper_limit="2.69398698685"/>
  </joint>
  <!--************************-->
  <!--        ELBOW           -->
  <!--************************-->
  <link name="arm_left_4_link">
    <inertial>
      <origin rpy="0 0 0" xyz="-0.0342 0.0241 0.0165"/>
      <mass value="1.27"/>
      <!--<inertia ixx="0"  ixy="0"  ixz="0" iyy="0" iyz="0" izz="0" />-->
      <!-- NOTE: Assuming cylinder centered at CoM with r=0.05, h=0.05 with axis parallel to x coordinate -->
      <!-- <inertia ixx="0.0012663"  ixy="0"  ixz="0" iyy="0.00084417" iyz="0" izz="0.00084417" /> -->
      <inertia ixx="0.0012786" ixy="0.00011984" ixz="-0.00017046" iyy="0.0015715" iyz="6.8669E-05" izz="0.0022145"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://reemc_description/meshes/arm/arm_4.dae" scale="1 1 1.0"/>
      </geometry>
      <material name="DarkGrey"/>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://reemc_description/meshes/arm/arm_4_collision.stl" scale="1 1 1.0"/>
      </geometry>
    </collision>
  </link>
  <joint name="arm_left_4_joint" type="revolute">
    <parent link="arm_left_3_link"/>
    <child link="arm_left_4_link"/>
    <origin rpy="0 -1.57079632679 0" xyz="0 -0.02 -0.088"/>
    <axis xyz="0 0 1"/>
    <limit effort="17.86" lower="0" upper="2.26892802759" velocity="4.58"/>
    <dynamics damping="1.0" friction="1.0"/>
    <safety_controller k_position="20" k_velocity="20" soft_lower_limit="0.0549065850399" soft_upper_limit="2.21402144255"/>
  </joint>
  <gazebo reference="arm_left_1_link">
    <mu1>0.9</mu1>
    <mu2>0.9</mu2>
  </gazebo>
  <gazebo reference="arm_left_2_link">
    <mu1>0.9</mu1>
    <mu2>0.9</mu2>
  </gazebo>
  <gazebo reference="arm_left_3_link">
    <mu1>0.9</mu1>
    <mu2>0.9</mu2>
  </gazebo>
  <gazebo reference="arm_left_4_link">
    <mu1>0.9</mu1>
    <mu2>0.9</mu2>
  </gazebo>
  <gazebo reference="arm_left_1_joint">
    <implicitSpringDamper>1</implicitSpringDamper>
  </gazebo>
  <gazebo reference="arm_left_2_joint">
    <implicitSpringDamper>1</implicitSpringDamper>
  </gazebo>
  <gazebo reference="arm_left_3_joint">
    <implicitSpringDamper>1</implicitSpringDamper>
  </gazebo>
  <gazebo reference="arm_left_4_joint">
    <implicitSpringDamper>1</implicitSpringDamper>
  </gazebo>
  <!--************************-->
  <!--        WRIST           -->
  <!--************************-->
  <link name="arm_left_5_link">
    <inertial>
      <origin rpy="0 0 0" xyz="0.0067 0.0071 -0.0705"/>
      <mass value="1.23"/>
      <!-- <inertia ixx="0"  ixy="0"  ixz="0" iyy="0" iyz="0" izz="0" /> -->
      <!-- NOTE: Assuming cylinder centered at CoM with r=0.05, h=0.1 with axis parallel to z coordinate -->
      <!--         <inertia ixx="0.0027096"  ixy="0"  ixz="0" iyy="0.0027096" iyz="0" izz="0.0023225" /> -->
      <inertia ixx="0.0011556" ixy="-1.8058E-07" ixz="2.2399E-06" iyy="0.00091494" iyz="2.2113E-07" izz="0.00065799"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://reemc_description/meshes/arm/arm_5.dae" scale="1 1 1.0"/>
      </geometry>
      <material name="LightGrey"/>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://reemc_description/meshes/arm/arm_5_collision.stl" scale="1 1 1.0"/>
      </geometry>
    </collision>
  </link>
  <joint name="arm_left_5_joint" type="revolute">
    <parent link="arm_left_4_link"/>
    <child link="arm_left_5_link"/>
    <origin rpy="0 1.57079632679 0" xyz="-0.088 0.02 0"/>
    <axis xyz="0 0 1"/>
    <limit effort="3" lower="-2.09439510239" upper="2.09439510239" velocity="1.95"/>
    <!--TODO: Check effort value!-->
    <dynamics damping="1.0" friction="1.0"/>
    <safety_controller k_position="20" k_velocity="20" soft_lower_limit="-2.03948851735" soft_upper_limit="2.03948851735"/>
  </joint>
  <link name="arm_left_6_link">
    <inertial>
      <origin rpy="0 0 0" xyz="-0.00005 -0.00252 -7e-05"/>
      <mass value="0.41597"/>
      <!-- <inertia ixx="0"  ixy="0"  ixz="0" iyy="0" iyz="0" izz="0" /> -->
      <!-- NOTE: Assuming cylinder centered at CoM with r=0.02, h=0.08 with axis parallel to z coordinate -->
      <!--         <inertia ixx="0.0001615"  ixy="0"  ixz="0" iyy="0.0001615" iyz="0" izz="0.000051" /> -->
      <inertia ixx="3.6511E-05" ixy="-2.3773E-09" ixz="1.33E-07" iyy="2.6707E-05" iyz="-1.6779E-07" izz="3.4209E-05"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://reemc_description/meshes/arm/arm_6.dae" scale="1 1 1.0"/>
      </geometry>
      <material name="LightGrey"/>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="0.0385" radius="0.04"/>
      </geometry>
    </collision>
  </link>
  <joint name="arm_left_6_joint" type="revolute">
    <parent link="arm_left_5_link"/>
    <child link="arm_left_6_link"/>
    <origin rpy="1.57079632679 0 0" xyz="0 0 -0.15"/>
    <axis xyz="0 0 1"/>
    <limit effort="6.6" lower="-1.41371669412" upper="1.41371669412" velocity="1.76"/>
    <dynamics damping="1.0" friction="1.0"/>
    <safety_controller k_position="20" k_velocity="20" soft_lower_limit="-1.42862327916" soft_upper_limit="1.35881010908"/>
  </joint>
  <link name="arm_left_7_link">
    <inertial>
      <!--TODO: Verify these inertial parameters-->
      <origin rpy="0 0 0" xyz="6.7314E-06 -0.00013277 0.029334"/>
      <mass value="0.10051"/>
      <!-- <inertia ixx="0"  ixy="0"  ixz="0" iyy="0" iyz="0" izz="0" /> -->
      <!-- NOTE: Assuming point mass -->
      <!--         <inertia ixx="0"  ixy="0"  ixz="0" iyy="0" iyz="0" izz="0" /> -->
      <inertia ixx="0.0000123718" ixy="0.00000000002" ixz="0.00000002403" iyy="0.00001242132" iyz="0.00000000001" izz="0.0000059514"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 -0.046"/>
      <geometry>
        <cylinder length="0.005" radius="0.02"/>
      </geometry>
      <material name="DarkGrey"/>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 -0.0345"/>
      <geometry>
        <cylinder length="0.023" radius="0.02"/>
      </geometry>
    </collision>
  </link>
  <joint name="arm_left_7_joint" type="revolute">
    <parent link="arm_left_6_link"/>
    <child link="arm_left_7_link"/>
    <origin rpy="-1.57079632679 0 0" xyz="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit effort="6.6" lower="-2.09439510239" upper="2.09439510239" velocity="1.76"/>
    <dynamics damping="1.0" friction="1.0"/>
    <safety_controller k_position="20" k_velocity="20" soft_lower_limit="-2.03948851735" soft_upper_limit="2.03948851735"/>
  </joint>
  <gazebo reference="arm_left_5_link">
    <mu1>0.9</mu1>
    <mu2>0.9</mu2>
  </gazebo>
  <gazebo reference="arm_left_6_link">
    <mu1>0.9</mu1>
    <mu2>0.9</mu2>
  </gazebo>
  <gazebo reference="arm_left_7_link">
    <mu1>0.9</mu1>
    <mu2>0.9</mu2>
  </gazebo>
  <gazebo reference="arm_left_5_joint">
    <implicitSpringDamper>1</implicitSpringDamper>
  </gazebo>
  <gazebo reference="arm_left_6_joint">
    <implicitSpringDamper>1</implicitSpringDamper>
  </gazebo>
  <gazebo reference="arm_left_7_joint">
    <implicitSpringDamper>1</implicitSpringDamper>
    <provideFeedback>1</provideFeedback>
  </gazebo>
  <transmission name="arm_left_5_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <actuator name="arm_left_5_motor">
      <mechanicalReduction>1.0</mechanicalReduction>
    </actuator>
    <joint name="arm_left_5_joint">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </joint>
  </transmission>
  <transmission name="wrist_left_trans">
    <type>transmission_interface/DifferentialTransmission</type>
    <actuator name="arm_left_6_motor">
      <role>actuator1</role>
      <mechanicalReduction>-1.0</mechanicalReduction>
    </actuator>
    <actuator name="arm_left_7_motor">
      <role>actuator2</role>
      <mechanicalReduction>1.0</mechanicalReduction>
    </actuator>
    <joint name="arm_left_6_joint">
      <role>joint1</role>
      <offset>0.0</offset>
      <mechanicalReduction>-1.0</mechanicalReduction>
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </joint>
    <joint name="arm_left_7_joint">
      <role>joint2</role>
      <offset>0.0</offset>
      <mechanicalReduction>1.0</mechanicalReduction>
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </joint>
  </transmission>
  <!--***********************-->
  <!--        TOOL           -->
  <!--***********************-->
  <link name="arm_left_tool_link">
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0" ixy="0.0" ixz="0.0" iyy="0.0" iyz="0.0" izz="0.0"/>
    </inertial>
    <visual>
      <origin rpy="0 1.57079632679 0" xyz="0.001 0 0"/>
      <geometry>
        <cylinder length="0.005" radius="0.005"/>
      </geometry>
      <material name="LightGrey"/>
    </visual>
    <collision>
      <origin rpy="0 1.57079632679 0" xyz="0.001 0 0"/>
      <geometry>
        <cylinder length="0.005" radius="0.005"/>
      </geometry>
    </collision>
  </link>
  <joint name="arm_left_tool_joint" type="fixed">
    <parent link="arm_left_7_link"/>
    <child link="arm_left_tool_link"/>
    <origin rpy="1.57079632679 1.57079632679 0" xyz="0 0 -0.046"/>
  </joint>
  <transmission name="arm_left_1_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <actuator name="arm_left_1_motor">
      <mechanicalReduction>1.0</mechanicalReduction>
    </actuator>
    <joint name="arm_left_1_joint">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
      <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    </joint>
  </transmission>
  <transmission name="arm_left_2_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <actuator name="arm_left_2_motor">
      <mechanicalReduction>1.0</mechanicalReduction>
    </actuator>
    <joint name="arm_left_2_joint">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
      <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    </joint>
  </transmission>
  <transmission name="arm_left_3_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <actuator name="arm_left_3_motor">
      <mechanicalReduction>1.0</mechanicalReduction>
    </actuator>
    <joint name="arm_left_3_joint">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
      <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    </joint>
  </transmission>
  <transmission name="arm_left_4_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <actuator name="arm_left_4_motor">
      <mechanicalReduction>1.0</mechanicalReduction>
    </actuator>
    <joint name="arm_left_4_joint">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
      <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    </joint>
  </transmission>
  <!--************************-->
  <!--        ft sensor       -->
  <!--************************-->
  <link name="wrist_right_ft_link">
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0" ixy="0.0" ixz="0.0" iyy="0.0" iyz="0.0" izz="0.0"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="0.0157" radius="0.0225"/>
      </geometry>
      <material name="LightGrey"/>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="0.0157" radius="0.0225"/>
      </geometry>
    </collision>
  </link>
  <joint name="wrist_right_ft_joint" type="fixed">
    <parent link="arm_right_tool_link"/>
    <child link="wrist_right_ft_link"/>
    <origin rpy="1.57079632679 0 1.57079632679" xyz="0.00785 0 0"/>
  </joint>
  <!--***********************-->
  <!--       FT TOOL         -->
  <!--***********************-->
  <link name="wrist_right_ft_tool_link">
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0" ixy="0.0" ixz="0.0" iyy="0.0" iyz="0.0" izz="0.0"/>
    </inertial>
    <visual>
      <origin rpy="0 1.57079632679 0" xyz="0.0 0 0"/>
      <geometry>
        <cylinder length="0.00975" radius="0.025"/>
      </geometry>
      <material name="FlatBlack"/>
    </visual>
    <collision>
      <origin rpy="0 1.57079632679 0" xyz="0.0 0 0"/>
      <geometry>
        <cylinder length="0.00975" radius="0.025"/>
      </geometry>
    </collision>
  </link>
  <joint name="wrist_right_tool_joint" type="fixed">
    <parent link="wrist_right_ft_link"/>
    <child link="wrist_right_ft_tool_link"/>
    <origin rpy="-1.57079632679 -1.57079632679 0" xyz="0 0 0.012725"/>
  </joint>
  <!--************************-->
  <!--        ft sensor       -->
  <!--************************-->
  <link name="wrist_left_ft_link">
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0" ixy="0.0" ixz="0.0" iyy="0.0" iyz="0.0" izz="0.0"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="0.0157" radius="0.0225"/>
      </geometry>
      <material name="LightGrey"/>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="0.0157" radius="0.0225"/>
      </geometry>
    </collision>
  </link>
  <joint name="wrist_left_ft_joint" type="fixed">
    <parent link="arm_left_tool_link"/>
    <child link="wrist_left_ft_link"/>
    <origin rpy="1.57079632679 0 1.57079632679" xyz="0.00785 0 0"/>
  </joint>
  <!--***********************-->
  <!--       FT TOOL         -->
  <!--***********************-->
  <link name="wrist_left_ft_tool_link">
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0" ixy="0.0" ixz="0.0" iyy="0.0" iyz="0.0" izz="0.0"/>
    </inertial>
    <visual>
      <origin rpy="0 1.57079632679 0" xyz="0.0 0 0"/>
      <geometry>
        <cylinder length="0.00975" radius="0.025"/>
      </geometry>
      <material name="FlatBlack"/>
    </visual>
    <collision>
      <origin rpy="0 1.57079632679 0" xyz="0.0 0 0"/>
      <geometry>
        <cylinder length="0.00975" radius="0.025"/>
      </geometry>
    </collision>
  </link>
  <joint name="wrist_left_tool_joint" type="fixed">
    <parent link="wrist_left_ft_link"/>
    <child link="wrist_left_ft_tool_link"/>
    <origin rpy="-1.57079632679 -1.57079632679 0" xyz="0 0 0.012725"/>
  </joint>
  <joint name="hand_right_palm_joint" type="fixed">
    <parent link="wrist_right_ft_tool_link"/>
    <child link="hand_right_palm_link"/>
    <origin rpy="0 0 0" xyz="0 0 0"/>
  </joint>
  <link name="hand_right_palm_link">
    <inertial>
      <origin rpy="0 0 0" xyz="0.0641 -0.0022 0.0021"/>
      <!-- NOTE: Less than total palm mass of 0.598kg because actuator masses are specified separately -->
      <mass value="0.68275"/>
      <inertia ixx="0.000305100" ixy="0.000005037" ixz="0.000015302" iyy="0.000811920" iyz="0.000007622" izz="0.000655851"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://reemc_description/meshes/hey5_hand/right_hand_hey5_orientation.dae"/>
      </geometry>
      <material name="Hey5DarkGrey"/>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <!-- <geometry>
          <xacro:if value="${reflect == 1}">
            <mesh filename="package://hey5_description/meshes/palm_collision.stl" scale="1 1 ${reflect}" />
          </xacro:if>
          <xacro:if value="${reflect == -1}">
            <mesh filename="package://hey5_description/meshes/palm_collision.stl" scale="1 1 ${reflect}" />
          </xacro:if>
        </geometry> -->
      <geometry>
        <mesh filename="package://reemc_description/meshes/hey5_hand/right_hand_hey5_orientation.dae"/>
      </geometry>
    </collision>
  </link>
  <link name="hand_right_grasping_frame">
    <inertial>
      <mass value="0"/>
      <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0.0" izz="0"/>
      <origin rpy="0 0 0" xyz="0 0 0"/>
    </inertial>
  </link>
  <joint name="hand_right_grasping_fixed_joint" type="fixed">
    <parent link="hand_right_palm_link"/>
    <child link="hand_right_grasping_frame"/>
    <axis xyz="0 0 1"/>
    <origin rpy="0 0 0" xyz="0.13 0.02 0"/>
  </joint>
  <gazebo reference="hand_link">
    <material>Gazebo/Black</material>
  </gazebo>
  <joint name="hand_left_palm_joint" type="fixed">
    <parent link="wrist_left_ft_tool_link"/>
    <child link="hand_left_palm_link"/>
    <origin rpy="3.14159265359 0 0" xyz="0 0 0"/>
  </joint>
  <link name="hand_left_palm_link">
    <inertial>
      <origin rpy="0 0 0" xyz="0.0641 -0.0022 0.0021"/>
      <!-- NOTE: Less than total palm mass of 0.598kg because actuator masses are specified separately -->
      <mass value="0.68275"/>
      <inertia ixx="0.000305100" ixy="0.000005037" ixz="0.000015302" iyy="0.000811920" iyz="0.000007622" izz="0.000655851"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://reemc_description/meshes/hey5_hand/left_hand_hey5_orientation.dae"/>
      </geometry>
      <material name="Hey5DarkGrey"/>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <!-- <geometry>
          <xacro:if value="${reflect == 1}">
            <mesh filename="package://hey5_description/meshes/palm_collision.stl" scale="1 1 ${reflect}" />
          </xacro:if>
          <xacro:if value="${reflect == -1}">
            <mesh filename="package://hey5_description/meshes/palm_collision.stl" scale="1 1 ${reflect}" />
          </xacro:if>
        </geometry> -->
      <geometry>
        <mesh filename="package://reemc_description/meshes/hey5_hand/left_hand_hey5_orientation.dae"/>
      </geometry>
    </collision>
  </link>
  <link name="hand_left_grasping_frame">
    <inertial>
      <mass value="0"/>
      <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0.0" izz="0"/>
      <origin rpy="0 0 0" xyz="0 0 0"/>
    </inertial>
  </link>
  <joint name="hand_left_grasping_fixed_joint" type="fixed">
    <parent link="hand_left_palm_link"/>
    <child link="hand_left_grasping_frame"/>
    <axis xyz="0 0 1"/>
    <origin rpy="0 0 0" xyz="0.13 0.02 0"/>
  </joint>
  <gazebo reference="hand_link">
    <material>Gazebo/Black</material>
  </gazebo>
  <link name="imu_link">
    <inertial>
      <mass value="0.01"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.000001" ixy="0" ixz="0" iyy="0.000001" iyz="0" izz="0.000001"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <box size="0.01 0.01 0.01"/>
      </geometry>
    </visual>
  </link>
  <joint name="imu_joint" type="fixed">
    <origin rpy="3.14159265359 0 0" xyz="0 0 0"/>
    <parent link="base_link"/>
    <child link="imu_link"/>
  </joint>
</robot>


)";  // clang-format on
