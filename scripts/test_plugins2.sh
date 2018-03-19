rosservice call /yumi/hiqp_joint_velocity_controller/set_primitives \
"primitives:
- name: 'ee_point'
  type: 'point'
  frame_id: 'gripper_r_finger_r'
  visible: true
  color: [1.0, 0.0, 0.0, 1.0]   
  parameters: [0.0, 0.0, 0.0]
- name: 'target_plane'
  type: 'plane'
  frame_id: 'yumi_body'
  visible: true
  color: [1.0, 0.0, 1.0, 0.6]   
  parameters: [1.0, 0.0, 1.0, 0.1]" 

rosservice call /yumi/hiqp_joint_velocity_controller/set_tasks \
   "tasks:
   - name: 'full_pose'
     priority: 3
     visible: 1
     active: 1
     monitored: 1
     def_params: ['TDefFullPose']
     dyn_params: ['TDynPD', '1.0', '1.0']

   - name: 'task2'
     priority: 2
     visible: 1
     active: 1
     monitored: 1
     def_params: ['TDefGeomProj', 'point', 'plane', 'ee_point = target_plane']
     dyn_params: ['TDynImpedance', '2.0', '6.0', '1.0']

   "
  


