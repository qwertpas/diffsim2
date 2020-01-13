# diffsim2
This is a 2D simulation of a [differential swerve robot](https://www.youtube.com/watch?v=aETaRclTDDo) that you test drive functions on. It features:
- simulated encoder values
- position and velocity graphs
- easy functions to get joystick axis and button values, or automatically use the coordinates of your mouse cursor when no joystick is connected
- similar format as the real FTC program (init and execute functions, 50Hz loop time)
- mostly accurate physics and adjustable constant values (coefficients of friction, gearing, etc.)
- functions for 1D motion profiling, PID, closed loop driving
- functions for generating your own graphs for debugging
- maven for easier library adding

Warnings:
Still a work in progress, performance may vary on different computers, only tested with Logitech Extreme 3D joystick.

# How to use:
1. Clone this repo from git or [download as a .zip file](https://github.com/qwertpas/diffsim2/archive/master.zip) and unzip
2. In VSCode, File > Open then select the joysim folder to open this as a VSCode project (make sure you have java extensions installed)
2. Go to diffsim2/src/main/java/org/chis/sim and open UserCode.java
3. Code something to make the robot go (default makes the robot go straight forever)
4. Run by either:
  - Select debug on the left side and click the green triangle
  - Or open Main.java, find ```public static void main(String[] args)...``` and click run
5. A JFrame window should open. Click resume to start the robot.

