# Warmup Project

## Drive in Square

### Problem
Command the turtlebot to drive in a perfect square.

### Approach
Driving in a square could be broken down into:
1. Commanding the robot to go straight.
2. Commanding the robot to turn 90 degrees.

If we could implement the above two steps, repeating them four times would theoretically give us a robot that drives in squares.

### Code Explanation
The code is object-oriented, and the overall structure was borrowed from Lab A "Spin in Circles" exercises. Instead of circles, it is now "spinning in squares". The class `DriveSquare` contains everything needed to get the robot driving in squares. In the `__init__` function, a robot node is initialized, and a publisher topic 'cmd_vel' is set up for controlling the robot to go straight or turn later. The function `go_straight` sets the forward velocity; the function `turn_right` sets the turning velocity; and the function `drive_square` combines the two with the intervention of `rospy.sleep` to ensure each function runs for enough time before the next function executes. `run` is just an encapsulation of `drive_square`.

### Challenges
The real environment is not always ideal, and calculated theoretical values do not always translate perfectly into reality. I have had to test out various angular velocities to find out the correct value to make the robot turn almost like a 90 degree.

### Future Work
If I had more time, I would have tried to incorporate LiDAR to help the robot turn perfectly 90 degrees by memorizing the environmental data changes.