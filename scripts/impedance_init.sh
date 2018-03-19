rosservice call /yumi/hiqp_joint_velocity_controller/set_primitives \
"primitives:

- name: 'wrist_frame'
  type: 'frame'
  frame_id: 'gripper_r_base'
  visible: true
  color: [1.0, 0.0, 0.0, 1.0]
  parameters: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# - name: 'init_frame'
#   type: 'frame'
#   frame_id: 'yumi_body'
#   visible: true
#   color: [1.0, 0.0, 1.0, 1.0]
#   parameters: [0.35, -0.2 , 0.15 , 1.57  , 3.14 , 0.0]

- name: 'init_frame2'
  type: 'frame'
  frame_id: 'yumi_body'
  visible: true
  color: [1.0, 0.0, 1.0, 1.0]
  parameters: [0.4, -0.25 , 0.15 , 0.0  , 1.57, 1.57]

  "


  rosservice call /yumi/hiqp_joint_velocity_controller/set_tasks \
  "tasks:
  - name: 'frame_tracking'
    priority: 2
    visible: 1
    active: 1
    monitored: 1
    def_params: ['TDefTracking', 'frame', 'frame', 'init_frame2 = wrist_frame']
    dyn_params: ['TDynPD', '1.0', '2.0']
   "
