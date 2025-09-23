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

    ros2 run my_controller_pkg xbox_controller


## 3. Controlling Joint
    ros2 run my_controller_pkg direct_controller

# Patch note
- 25/09/22(박정우) - 초안 작성!
- 25/09/23(박정우) - direct_controller HOME, START 버튼 Way Point 추가, Joint3 증가 감소를 Y,A로 할당
