# =============================================================================
#                           ONLINE SAFE IMPLEMENTATION
# =============================================================================


# # Modelling the chopping motion through geometric primitives and Stacks of tasks
# # Description (short) :
#
# Initial pose :          Align the EE frame with the frame that corresponds to the
# Switch controllers:     Activate admittance controller
# Chop once:              Get the EE to the table's surface



# Comments: It seems that by removing the "full pose" task, task1 and 2 are achieved
#           much more strictly. Unfortunately, that leaves the elbow in weird configurations
#           When more of the movement is specified, do try to remove it and see what happens
#
#           The second hand is also controlled by the gains provided.

#           For the time being, the object is assumed to be at a higher position than the
#           table's surface


# ========== init =========
echo $'----- Cleanup and initialization starting...'

# # COMMENT IN ONLY WHEN SURE THE INITIAL POSITIONS ARE FINE (THEY SEEM TO BE HITTING THE TABLE)
source online_cleanup.sh


rosservice call /yumi/hiqp_joint_velocity_controller/set_primitives \
"primitives:

- name: 'ee_frame'
  type: 'frame'
  frame_id: 'gripper_r_base'
  visible: true
  color: [1.0, 0.0, 0.0, 1.0]
  parameters: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

- name: 'init_frame'
  type: 'frame'
  frame_id: 'yumi_body'
  visible: true
  color: [1.0, 0.0, 1.0, 1.0]
  parameters: [0.22, -0.08 , 0.1 , 0.0, 1.57, 1.57]

- name: 'ee_point'
  type: 'point'
  frame_id: 'gripper_r_base'
  visible: false
  color: [3.0, 1.0, 1.0, 1.0]
  parameters: [0.0, 0.0 , 0.0]

- name: 'target_point'
  type: 'point'
  frame_id: 'yumi_body'
  visible: false
  color: [3.0, 1.0, 1.0, 1.0]
  parameters: [0.22, -0.08 , 2.0]

- name: 'final_frame'
  type: 'frame'
  frame_id: 'yumi_body'
  visible: false
  color: [3.0, 1.0, 1.0, 1.0]
  parameters: [0.22, -0.08 , -1.0,  0.0, 1.57, 1.57]

  "

# Assuming the object is at the z = 1 plane.

   rosservice call /yumi/hiqp_joint_velocity_controller/set_tasks \
   "tasks:


   - name: 'task1'
     priority: 2
     visible: 1
     active: 1
     monitored: 1
     def_params: ['TDefTracking', 'frame', 'frame', 'init_frame = ee_frame', '0.05', '0.5']
     dyn_params: ['TDynPD', '30.0', '30.0', '30.0', '3.0', '3.0', '3.0', '12.0', '12.0', '12.0', '4.0', '4.0', '4.0']

   - name: 'task2'
     priority: 2
     visible: 1
     active: 1
     monitored: 1
     def_params: ['TDefTracking', 'point', 'point', 'ee_point = target_point', '0.04']
     dyn_params: ['TDynPD', '4.0', '1.0']

   - name: 'full_pose'
     priority: 3
     visible: 1
     active: 1
     monitored: 1
     def_params: ['TDefFullPose']
     dyn_params: ['TDynPD', '1.0', '1.0']

   "
   # dyn_params: ['TDynPD', '11.0', '12.0', '10.0', '20.0', '20.0', '20.0', '4.0', '4.0', '4.0', '1.0', '1.0', '1.0']

    echo $'----- Moving to initial pose...'

    sleep 10

    echo $'----- Pose initialized...'

    echo $'----- Moving downwards...'

    # ==================== Phase 2  ========================
    # rosservice call /yumi/hiqp_joint_velocity_controller/remove_tasks "names:
    # - 'task1'"
    ## NOTE_TO_SELF: I commented this out cause removing it seemed to result in a hiccup


    rosservice call /yumi/hiqp_joint_velocity_controller/remove_tasks "names:
    - 'task2'"


    rosservice call /yumi/hiqp_joint_velocity_controller/set_tasks \
    "tasks:

    - name: 'task3'
      priority: 2
      visible: 1
      active: 1
      monitored: 1
      def_params: ['TDefTracking', 'frame', 'frame', 'ee_frame = final_frame', '0.05', '0.5']
      dyn_params: ['TDynPD', '20.0', '20.0', '20.0', '5.0', '5.0', '3.0', '12.0', '12.0', '12.0', '4.0', '4.0', '4.0']

      "


      # dyn_params: ['TDynPD', '30.0', '30.0', '30.0', '3.0', '3.0', '3.0', '12.0', '12.0', '12.0', '4.0', '4.0', '4.0']


     # ==================== Primitives' graveyard ========================
     # - name: 'ee2_point'
     #   type: 'point'
     #   frame_id: 'yumi_body'
     #   visible: false
     #   color: [3.0, 1.0, 1.0, 1.0]
     #   parameters: [0.22, -0.08 , -1.0]
     # - name: 'elbow_axis'
     #   type: 'line'
     #   frame_id: 'yumi_link_4_r'
     #   visible: true
     #   color: [8.0, 1.0, 0.0, 1.0]
     #   parameters: [0.0, 0.0 , 0.0 , 0.0 , 0, 0.1]
     #
     # - name: 'body_axis'
     #   type: 'line'
     #   frame_id: 'yumi_body'
     #   visible: true
     #   color: [12.0, 10.0, 0.0, 1.0]
     #   parameters: [0.0, 0.0 , 1.0 , 0.0 , 0, 0.1]

     # - name: 'wrist_axis'
     #   type: 'line'
     #   frame_id: 'yumi_link_7_r'
     #   visible: true
     #   color: [2.0, 10.0, 9.0, 1.0]
     #   parameters: [0.0, 0.0 , 1.0 , 0.0 , 0, 0.1]
     #
     # - name: 'almost_object_axis'
     #   type: 'line'
     #   frame_id: 'yumi_body'
     #   visible: true
     #   color: [2.0, 1.0, 0.0, 1.0]
     #   parameters: [0, 1.0 , 0, 0.25 , 0, 0.1]

     # - name: 'knife_axis'
     #   type: 'line'
     #   frame_id: 'gripper_r_base'
     #   visible: false
     #   color: [0.0, 0.0, 0.0, 1.0]
     #   parameters: [0, 0, 1, 0, 0, 0.1]
     #

     #
     # - name: 'object_axis'
     #   type: 'line'
     #   frame_id: 'yumi_body'
     #   visible: true
     #   color: [4.0, 1.0, 0.0, 1.0]
     #   parameters: [0, 1.0 , 0, 0.35 , 0, 0.1]
     #

     #

     #
     #   - name: 'target_point'
     #     type: 'point'
     #     frame_id: 'yumi_body'
     #     visible: false
     #     color: [0.0, 1.0, 0.0, 0.6]
     #     parameters: [1.0, 0.0, 0.0]

   # ==============================================================================
# ==================== Tasks' graveyard ========================
# - name: 'task2'
#   priority: 2
#   visible: 1
#   active: 1
#   monitored: 1
#   def_params: ['TDefTracking', 'point', 'point', 'ee2_point = target_point', '0.04']
#   dyn_params: ['TDynPD', '4.0', '1.0']


   # - name: 'task1'
   #   priority: 2
   #   visible: 1
   #   active: 1
   #   monitored: 1
   #   def_params: ['TDefGeomAlign', 'line', 'line', 'elbow_axis = body_axis']
   #   dyn_params: ['TDynPD', '1.0', '3.0']


   # - name: 'task3'
   #   priority: 3
   #   visible: 1
   #   active: 1
   #   monitored: 1
   #   def_params: ['TDefGeomAlign', 'line', 'line', 'wrist_axis = almost_object_axis']
   #   dyn_params: ['TDynPD', '1.8', '4.0']


   # - name: 'task3'
   #   priority: 3
   #   visible: 1
   #   active: 1
   #   monitored: 1
   #   def_params: ['TDefFullPose', '-1.0', '-1.4', '1.7', '-0.2', '0.0', '1.0', '1.0', '1.0', '-1.4', '-1.7', '-0.2', '0.0', '1.0', '-1.0']
   #   dyn_params: ['TDynPD', '1.0', '2.0']



  # # - name: 'task1'
  # #   priority: 2
  # #   visible: 1
  # #   active: 1
  # #   monitored: 1
  # #   def_params: ['TDefGeomAlign', 'line', 'line', 'knife_axis = object_axis']
  # #   dyn_params: ['TDynPD', '1.8', '4.0']
  # #
  # #
  # #
  # # - name: 'elbow_ffs'
  # #   priority: 3
  # #   visible: 1
  # #   active: 1
  # #   monitored: 1
  # #   def_params: ['TDefJntConfig', 'yumi_link_4_r', '-0.1']
  # #   dyn_params: ['TDynPD', '1.0', '2.0']
  # #
  # #   "
# ==============================================================================
