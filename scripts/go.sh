rosservice call /yumi/hiqp_joint_velocity_controller/remove_tasks \
"names:
- 'frame_tracking'
- 'table_projection'"

rosservice call /yumi/hiqp_joint_velocity_controller/remove_primitives \
"names:
- 'init_frame2'
- 'tabletop_plane'"

rosservice call /yumi/hiqp_joint_velocity_controller/list_all_tasks \
