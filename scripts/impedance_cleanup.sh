#remove tasks
rosservice call /yumi/hiqp_joint_velocity_controller/remove_tasks \
"names:
- 'table_projection'
- 'x_projection'
- 'y_projection'
- 'frame_tracking'
"
# -'full_pose'
#remove primitives
# rosservice call /yumi/hiqp_joint_velocity_controller/remove_primitives \
# "names:
# - 'knife_middle_point'
# - 'x_plane'
# - 'y_plane'
# - 'tabletop_plane'
#
# "

rosservice call /yumi/hiqp_joint_velocity_controller/remove_all_primitives "{}"
