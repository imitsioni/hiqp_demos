# Online chopping attempt with the impedance Controller
# using points and planes since it's the easier kind of force
# projection and also seems to be already implemented
# ========================================================================
# Task as Geometric primitives:
# Downward motion > Follow a line which is defined as the intersection of 2
#                    planes until you reach the tabletop plane
# The two planes represent the constraints for the "incision", one is parallel to
# x and one is parallel to y



# the problem  right now is that because the downards motion is specified
# through the knife point, the wrist tends to bend and is not horizontal to the
# cutting plane


rosservice call /yumi/hiqp_joint_velocity_controller/set_primitives \
"primitives:
- name: 'knife_middle_point'
  type: 'point'
  frame_id: 'gripper_r_base'
  visible: true
  color: [1.0, 0.0, 1.0, 1.0]
  parameters: [0.0, 0.0, 0.08]

- name: 'x_plane'
  type: 'plane'
  frame_id: 'yumi_body'
  visible: false
  color: [1.0, 1.0, 1.0, 0.6]
  parameters: [1.0, 0.0, 0.0, 0.35]

- name: 'y_plane'
  type: 'plane'
  frame_id: 'yumi_body'
  visible: false
  color: [1.0, 3.0, 1.0, 0.6]
  parameters: [0.0, 1.0, 0.0, -0.2]

- name: 'tabletop_plane'
  type: 'plane'
  frame_id: 'yumi_body'
  visible: true
  color: [1.0, 0.0, 1.0, 0.6]
  parameters: [0.0, 0.0, 1.0, 0.07]
  "

  #source joint_configuration.sh


  rosservice call /yumi/hiqp_joint_velocity_controller/set_tasks \
  "tasks:
  - name: 'x_projection'
    priority: 2
    visible: 0
    active: 1
    monitored: 1
    def_params: ['TDefGeomProj', 'point', 'plane', 'knife_middle_point = x_plane']
    dyn_params: ['TDynPD', '2.0', '3.0']

  - name: 'y_projection'
    priority: 2
    visible: 0
    active: 1
    monitored: 1
    def_params: ['TDefGeomProj', 'point', 'plane', 'knife_middle_point = y_plane']
    dyn_params: ['TDynPD', '2.0', '3.0']

  - name: 'table_projection'
    priority: 2
    visible: 0
    active: 1
    monitored: 1
    def_params: ['TDefGeomProj', 'point', 'plane', 'knife_middle_point = tabletop_plane']
    dyn_params: ['TDynImpedance', '2.0', '3.0', '1.0']
    # dyn_params: ['TDynPD', '2.0', '3.0']
    "




#  remove the initialization
  rosservice call /yumi/hiqp_joint_velocity_controller/remove_tasks \
  "names:
   - 'frame_tracking'
    "
