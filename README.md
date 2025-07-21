# Turtlebottest
실제로 폴더에 다운이 되고 빌드 되는지 확인 용 (확인 완_ROS2_Humble 기준)

터미널로 아래와 같은 코드 작성하기:
* cd ~/turtlebot3_ws/src/
* git clone -b main https://github.com/WISET-F19/Turtlebottest.git
* cd ~/turtlebot3_ws && colcon build --symlink-install

새로 코드 업데이트 된 뒤 패키지 없뎃 하고 싶을때:
* cd Turtlebottest/
* git pull origin main

시뮬레이션 돌리고 싶을때 예시 코드:
* ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
* ros2 run turtlebot3_gazebotest turtlebot3_drivetest

실제 상황에서 돌리고 싶을 때:
* ros2 launch turtlebot3_bringup robot.launch.py
* ros2 run turtlebot3_gazebotest turtlebot3_drivetest

깃허브 통해서 코드 업뎃 후 패키지 업뎃 하고 싶을 때 - 터틀봇 버전:
* cd ~/turtlebot3_ws/src/Turtlebottest/
* git pull origin main
* colcon build --packages-select turtlebot3_gazebotest
* source install/setup.bash