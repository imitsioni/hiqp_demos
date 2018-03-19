#Aligns the z-axis of a target frame with a given cylinder's axis while intersecting the x-axis of the target frame with the cylinder's axis
rosservice call /yumi/hiqp_joint_velocity_controller/set_primitives \
"primitives:
- name: 'test_frame'
  type: 'frame'
  frame_id: 'gripper_r_base'
  visible: true
  color: [0.0, 0.0, 1.0, 1.0]   
  parameters: [0.0, 0.0, 0.1, -1.55, 0, -1.55] 
  
- name: 'target_cylinder'
  type: 'cylinder'
  frame_id: 'yumi_base_link'
  visible: true
  color: [0.0, 0.0, 1.0, 1.0]   
  parameters: [0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.1, 0.5]
  
- name: 'ee_point'
  type: 'point'
  frame_id: 'gripper_r_base'
  visible: true
  color: [1.0, 0.0, 0.0, 1.0]   
  parameters: [0.0, 0.0, 0.15]
  
- name: 'target_plane'
  type: 'plane'
  frame_id: 'yumi_body'
  visible: true
  color: [1.0, 0.0, 1.0, 0.6]   
  parameters: [0.0, 0.0, 1.0, 0.1]
  "

rosservice call /yumi/hiqp_joint_velocity_controller/set_tasks \
"tasks:
- name: 'fca'
  priority: 2
  visible: 1
  active: 1
  monitored: 1
  def_params: ['TDefGeomAlign', 'frame', 'cylinder', 'test_frame = target_cylinder', '0.5']
  dyn_params: ['TDynImpedance', '2.0', '3.0', '1.0']
- name: 'neutral_pose'
  priority: 3
  visible: 1
  active: 1
  monitored: 1
  def_params: ['TDefFullPose']
  dyn_params: ['TDynPD', '2.0', '3.0'] "

echo $'----- Sleeping for 10...'
sleep 10

echo $'----- Changing task...'

rosservice call /yumi/hiqp_joint_velocity_controller/remove_tasks "names:
- 'fca'"

rosservice call /yumi/hiqp_joint_velocity_controller/set_tasks \
"tasks:
- name: 'ppp'
  priority: 2
  visible: 1
  active: 1
  monitored: 1
  def_params: ['TDefGeomProj', 'point', 'plane', 'ee_point = target_plane']
  dyn_params: ['TDynPD', '10.0', '8.0']"

