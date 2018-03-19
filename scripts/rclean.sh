#remove tasks
rosservice call /yumi/hiqp_joint_velocity_controller/remove_tasks \
"names:
- 'jnt1'
- 'jnt2'
- 'jnt3'
- 'jnt4'
- 'jnt5'
- 'jnt6'
- 'jnt7'

"

#remove primitives
# rosservice call /yumi/hiqp_joint_velocity_controller/remove_primitives \
# "names:
# - 'knife_middle_point'
# - 'x_plane'
# - 'y_plane'
# - 'tabletop_plane'
#
# "

# rosservice call /yumi/hiqp_joint_velocity_controller/remove_all_primitives "{}"
