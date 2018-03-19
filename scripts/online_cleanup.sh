# =============================================================================
#                           ONLINE SAFE IMPLEMENTATION
# =============================================================================

rosservice call /yumi/hiqp_joint_velocity_controller/remove_tasks "names:
- 'task1'"
rosservice call /yumi/hiqp_joint_velocity_controller/remove_tasks "names:
- 'task2'"
rosservice call /yumi/hiqp_joint_velocity_controller/remove_tasks "names:
- 'task3'"
# rosservice call /yumi/hiqp_joint_velocity_controller/remove_tasks "names:
# - 'task4'"
# rosservice call /yumi/hiqp_joint_velocity_controller/remove_tasks "names:
# - 'task5'"
rosservice call /yumi/hiqp_joint_velocity_controller/remove_tasks "names:
- 'full_pose'"

rosservice call /yumi/hiqp_joint_velocity_controller/remove_all_primitives \


rosservice call /yumi/hiqp_joint_velocity_controller/set_tasks \
"tasks:
- name: 'full_pose'
  priority: 3
  visible: 1
  active: 1
  monitored: 1
  def_params: ['TDefFullPose', '-1.0', '-1.4', '1.7', '-1', '0.0', '1.0', '1.0', '1.0', '-1.4', '-1.7', '-0.2', '0.0', '1.0', '-1.0']
  dyn_params: ['TDynPD', '1.0', '2.0'] "

  sleep 10


  rosservice call /yumi/hiqp_joint_velocity_controller/remove_tasks "names:
  - 'full_pose'"
# removing all primitives isnt very dangerous because they have dependancies and wont be obliterated
  # rosservice call /yumi/hiqp_joint_velocity_controller/remove_all_primitives \
  # rosservice call /yumi/hiqp_joint_velocity_controller/list_all_primitives \
