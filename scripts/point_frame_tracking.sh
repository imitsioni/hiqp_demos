rosservice call /yumi/hiqp_joint_velocity_controller/set_primitives \
"primitives:
- name: 'ee_point'
  type: 'point'
  frame_id: 'gripper_r_base'
  visible: true
  color: [1.0, 0.0, 0.0, 1.0]   
  parameters: [0.0, 0.0, 0.15]
  
- name: 'target_frame'
  type: 'frame'
  frame_id: 'yumi_base_link'
  visible: true
  color: [1.0, 0.0, 1.0, 1.0]   
  parameters: [0.55, -0.25 , 0.3, 0.0, 1.47, 1.47]" 

rosservice call /yumi/hiqp_joint_velocity_controller/set_tasks \
"tasks:  
- name: 'point_point_tracking'
  priority: 2
  visible: 1
  active: 1
  monitored: 1
  def_params: ['TDefTracking', 'point', 'frame', 'ee_point = target_frame', '0.04']
  dyn_params: ['TDynImpedance', '20.0', '5.0', '1.0' ] 
- name: 'full_pose'
  priority: 3
  visible: 1
  active: 1
  monitored: 1
  def_params: ['TDefFullPose']
  dyn_params: ['TDynPD', '1.0', '2.0'] "


