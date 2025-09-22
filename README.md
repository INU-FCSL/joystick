# Package for control manipulator with Joystick(xbox)

## 1. Need joystick package

    #workspace에서
    cd <ws_path>/src
    git clone https://github.com/ros-drivers/joystick_drivers/tree/ros2/joy
    cd ..
    colcon build --symlink-install

then, 조이스틱을 연결(유,무선 상관 없음)

    ros2 run joy joy_node
    ros2 topic echo /joy 
    
axes, button의 토픽이 잘 나오는지 확인

1번이 잘 나오면 2번 혹은 3번을 선택해서 실행 (joy_node는 항상 실행!)

## 2. Using Moveit- 미완성

    ros2 run my_controller_pkg xbox_full_control.py


## 3. Controlling Joint
    ros2 run my_controller_pkg direct_arm_control.py

아직 수정중! -박정우 25/09/22


