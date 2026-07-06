빌드
hyeonsik@angg88890:~$ cd ws_cobot_pjt/ws_dsr
hyeonsik@angg88890:~/ws_cobot_pjt/ws_dsr$ colcon build --symlink-install --packages-select brick_pick_place 
Starting >>> brick_pick_place
Finished <<< brick_pick_place [1.46s]          

Summary: 1 package finished [1.89s]
hyeonsik@angg88890:~/ws_cobot_pjt/ws_dsr$ source install/setup.bash
hyeonsik@angg88890:~/ws_cobot_pjt/ws_dsr$ ros2 run brick_pick_place initial 

