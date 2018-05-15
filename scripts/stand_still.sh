rosservice call /yumi/hiqp_joint_velocity_controller/set_primitives \
"primitives:

- name: 'wrist_frame'
  type: 'frame'
  frame_id: 'gripper_r_base'
  visible: true
  color: [1.0, 0.0, 0.0, 1.0]
  parameters: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

- name: 'init_frame2'
  type: 'frame'
  frame_id: 'yumi_body'
  visible: true
  color: [1.0, 0.0, 1.0, 1.0]
  parameters: [0.4, -0.25 , 0.3 , 0.0  , 1.57, 1.57]

- name: 'knife_middle_point'
  type: 'point'
  frame_id: 'gripper_r_base'
  visible: true
  color: [1.0, 0.0, 1.0, 1.0]
  # parameters: [0.0, 0.0, 0.08]
  parameters: [0.0, 0.0, 0.0]

- name: 'tabletop_plane'
  type: 'plane'
  frame_id: 'yumi_body'
  visible: true
  color: [1.0, 0.0, 1.0, 0.6]
  parameters: [0.0, 0.0, 1.0, 0.3]
   "



  rosservice call /yumi/hiqp_joint_velocity_controller/set_tasks \
  "tasks:
  - name: 'frame_tracking'
    priority: 2
    visible: 1
    active: 1
    monitored: 1
    def_params: ['TDefTracking', 'frame', 'frame', 'init_frame2 = wrist_frame']
    dyn_params: ['TDynPD', '4.0', '2.0']

  - name: 'table_projection'
    priority: 2
    visible: 0
    active: 1
    monitored: 1
    def_params: ['TDefFT', 'gripper_r_base', 'point', 'plane', 'knife_middle_point = tabletop_plane' ]
    dyn_params: ['TDynImpedance', '30.0', '10.0', '5.0']

  - name: 'full_pose'
    priority: 3
    visible: 1
    active: 1
    monitored: 1
    def_params: ['TDefFullPose', '-1.0', '-1.4', '1.7', '-0.2', '0.0', '1.0', '1.0', '1.0', '-1.4', '-1.7', '-0.2', '0.0', '1.0', '-1.0']
    dyn_params: ['TDynPD', '0.5 ', '2.0']


   "
