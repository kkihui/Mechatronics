# Midterm Project
PID control for motor

### Midterm_25.c
1. Enter total number of iterations & target number of rotations for each iteration
2. Wait untill 1st pulse is received
3. When 1st pulse is received, Do 1st iteration using PID control to roatate motor by the target number of rotation
4. If next pulse is received, it move on to the next iteration & rotates the motor by the next target number of rotation
5. Repeat until all iterations are completed.<br><br>
![KakaoTalk_20231114_213806830](https://github.com/kkihui/Mechatronics/assets/121797755/238928d2-dc60-45ba-adaa-180c22281bbf)
6 iteration: \[5 -3 -2 -2 4 0\]
<br><br>

### PID_control_plot.c
Code for using ziggler nichols method.<br>
It plots magnitude obtained by PID control over time.
