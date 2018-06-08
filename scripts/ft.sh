rosservice call /yumi/hiqp_joint_velocity_controller/set_primitives \
"primitives:
- name: 'knife_middle_point'
  type: 'point'
  frame_id: 'gripper_r_base'
  visible: true
  color: [1.0, 0.0, 1.0, 1.0]
  parameters: [0.0, 0.0, 0.0]

- name: 'x_plane'
  type: 'plane'
  frame_id: 'yumi_body'
  visible: false
  color: [1.0, 1.0, 1.0, 0.6]
  parameters: [1.0, 0.0, 0.0, 0.4]

- name: 'y_plane'
  type: 'plane'
  frame_id: 'yumi_body'
  visible: false
  color: [1.0, 3.0, 1.0, 0.6]
  parameters: [0.0, 1.0, 0.0, -0.25]

- name: 'tabletop_plane'
  type: 'plane'
  frame_id: 'yumi_body'
  visible: true
  color: [1.0, 0.0, 1.0, 0.6]
  parameters: [0.0, 0.0, 1.0, 0.3]
  "


rosservice call /yumi/hiqp_joint_velocity_controller/set_tasks \
"tasks:
- name: 'ft'
  priority: 2
  visible: 1
  active: 1
  monitored: 1
  # def_params: ['TDefGeomProj', 'point', 'plane', 'knife_middle_point = tabletop_plane']
  def_params: ['TDefFT', 'gripper_r_base', 'point', 'plane', 'knife_middle_point = tabletop_plane' ]
  dyn_params: ['TDynImpedance', '2.0', '3.0', '1.0']
  # dyn_params: ['TDynPD', '5.0', '8.0']

- name: 'full_pose'
  priority: 3
  visible: 1
  active: 1
  monitored: 1
  def_params: ['TDefFullPose', '-1.0', '-1.4', '1.7', '-0.2', '0.0', '1.0', '1.0', '1.0', '-1.4', '-1.7', '-0.2', '0.0', '1.0', '-1.0']
  dyn_params: ['TDynPD', '0.5 ', '2.0']


  "
  # - name: 'jnt7'
  #   priority: 3
  #   visible: 1
  #   active: 1
  #   monitored: 1
  #   def_params: ['TDefJntConfig', 'yumi_link_7_r', '-0.71']
  #   dyn_params: ['TDynPD', '1.0', '2.0']
  #
  # - name: 'jnt6'
  #   priority: 3
  #   visible: 1
  #   active: 1
  #   monitored: 1
  #   def_params: ['TDefJntConfig', 'yumi_link_6_r', '0.0']
  #   dyn_params: ['TDynPD', '1.0', '2.0']
  #
  # - name: 'jnt5'
  #   priority: 3
  #   visible: 1
  #   active: 1
  #   monitored: 1
  #   def_params: ['TDefJntConfig', 'yumi_link_5_r', '0.0']
  #   dyn_params: ['TDynPD', '1.0', '2.0']
  #
  #
  # - name: 'jnt4'
  #   priority: 3
  #   visible: 1
  #   active: 1
  #   monitored: 1
  #   def_params: ['TDefJntConfig', 'yumi_link_4_r', '0.0004']
  #   dyn_params: ['TDynPD', '1.0', '2.0']
  #
  # - name: 'jnt3'
  #   priority: 3
  #   visible: 1
  #   active: 1
  #   monitored: 1
  #   def_params: ['TDefJntConfig', 'yumi_link_3_r', '0.3']
  #   dyn_params: ['TDynPD', '1.0', '2.0']
  #
  # - name: 'jnt2'
  #   priority: 3
  #   visible: 1
  #   active: 1
  #   monitored: 1
  #   def_params: ['TDefJntConfig', 'yumi_link_2_r', '-2.1']
  #   dyn_params: ['TDynPD', '1.0', '2.0']
  #
  #
  # - name: 'jnt1'
  #   priority: 3
  #   visible: 1
  #   active: 1
  #   monitored: 1
  #   def_params: ['TDefJntConfig', 'yumi_link_1_r', '1.41']
  #   dyn_params: ['TDynPD', '1.0', '2.0']
