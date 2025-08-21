# hana_violet
Modified Version For Pink Violet

## hana/Gazebo
250627 add gz_frame_id for IMU sensor --KSH  
250627 add Python launch file for simulation --KSH  
250627 remove /odom topic from simulation (gazebo's yaml file and xacro file) --KSH (lots of odom...)
250628 modify sim_cartographer.lua (Not using IMU version) --KSH  


## hana/Cartographer
250627 add IMU option to initial_pose.lua --KSH  
250627 add Python launch file for simulation --KSH  
250628 modify gazebo_control.xacro : TF (odom - base_link) publishing   from Gazebo --KSH  

## hana/Navigation
250628 add some navigation files for Simulation and make setup --KSH
250628 add my_map.yaml and change default map to my_map --KSH
250628 add slam and navigation for Gazebo --KSH


#### todo
- launch.py File 코드 리뷰  
- **IMU publish on/off 되게 launch file 수정**  
- multi-robot launch파일 만들기  


#### 간단 GZ Sim 메뉴얼
- gazebo : `ros2 launch hana_gazebo sim.launch.py`  
- slam : `ros2 launch hana_cartographer sim_cartographer.launch.xml`  
- navigation : `ros2 launch hana_navigation initialpose_bringup_launch.xml  use_sim_time:=True`  
- slam&nav : `ros2 launch hana_navigation sim_slam_nav.xml` << on going  

#### AMCL Navigation (250630 KSH)
- USING AMCL
  - ros2 launch hana_gazebo launch_sim.launch.xml  
  - ros2 launch hana_navigation amcl_initialpose_bringup_launch.xml  use_sim_time:=True  
  - ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py  
  - ros2 launch hana_navigation nav2_view.launch.xml  