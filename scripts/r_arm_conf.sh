rosservice call /yumi/hiqp_joint_velocity_controller/set_tasks \
"tasks:
- name: 'jnt7'
  priority: 3
  visible: 1
  active: 1
  monitored: 1
  def_params: ['TDefJntConfig', 'yumi_link_7_r', '-0.71']
  dyn_params: ['TDynPD', '1.0', '2.0']

- name: 'jnt6'
  priority: 3
  visible: 1
  active: 1
  monitored: 1
  def_params: ['TDefJntConfig', 'yumi_link_6_r', '0.0']
  dyn_params: ['TDynPD', '1.0', '2.0']

- name: 'jnt5'
  priority: 3
  visible: 1
  active: 1
  monitored: 1
  def_params: ['TDefJntConfig', 'yumi_link_5_r', '0.0']
  dyn_params: ['TDynPD', '1.0', '2.0']


- name: 'jnt4'
  priority: 3
  visible: 1
  active: 1
  monitored: 1
  def_params: ['TDefJntConfig', 'yumi_link_4_r', '0.0004']
  dyn_params: ['TDynPD', '1.0', '2.0']

- name: 'jnt3'
  priority: 3
  visible: 1
  active: 1
  monitored: 1
  def_params: ['TDefJntConfig', 'yumi_link_3_r', '0.3']
  dyn_params: ['TDynPD', '1.0', '2.0']

- name: 'jnt2'
  priority: 3
  visible: 1
  active: 1
  monitored: 1
  def_params: ['TDefJntConfig', 'yumi_link_2_r', '-2.1']
  dyn_params: ['TDynPD', '1.0', '2.0']


- name: 'jnt1'
  priority: 3
  visible: 1
  active: 1
  monitored: 1
  def_params: ['TDefJntConfig', 'yumi_link_1_r', '1.41']
  dyn_params: ['TDynPD', '1.0', '2.0']

  "
