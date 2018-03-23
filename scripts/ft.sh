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
  parameters: [0.0, 0.0, 1.0, 0.07]
  "


rosservice call /yumi/hiqp_joint_velocity_controller/set_tasks \
"tasks:
- name: 'ft'
  priority: 2
  visible: 1
  active: 1
  monitored: 1
  #def_params: ['TDefGeomProj', 'point', 'plane', 'knife_middle_point = tabletop_plane']
  def_params: ['TDefFT', 'gripper_r_base', 'point', 'plane', 'knife_middle_point = tabletop_plane' ]
  # dyn_params: ['TDynImpedance', '5.0', '8.0', '1.0']
  dyn_params: ['TDynPD', '5.0', '8.0']

  "
