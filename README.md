# Warmup Project

## Drive in Square

### High-level Description
Command the turtlebot to drive in a perfect square.

### Approach
Driving in a square could be broken down into:
1. Commanding the robot to go straight.
2. Commanding the robot to turn 90 degrees.

If I could implement the above two steps, repeating them four times would theoretically give us a robot that drives in squares.

### Code Explanation
The code is object-oriented, and the overall structure was borrowed from Lab A's "Spin in Circles" exercises. Instead of circles, it is now "spinning in squares". The class `DriveSquare` contains everything needed to get the robot to drive in squares. In the `__init__` function, a robot node is initialized, and a publisher topic 'cmd_vel' is set up for controlling the robot to go straight or turn later. The function `go_straight` sets the forward velocity; the function `turn_right` sets the turning velocity; and the function `drive_square` combines the two with the intervention of `rospy.sleep` to ensure each function runs for enough time before the next function executes. `run` is just an encapsulation of `drive_square`.

### Video
![drive_square](https://github.com/Intro-Robotics-UChicago-Spring-2024/warmup-project-JihuiTanUchicago/assets/91858809/d4fab9f3-53b0-433c-82c1-3cf519763170)

## Person Follower

### High-level Description
Command the robot to follow the "person"(closest object) no matter where the person is.

### Approach
Following a person could be broken down into:
1. Scanning the environment using `lds` and determine which direction has the closest object.
2. Command the robot to make a turn until facing the closest object and moving towards the object. The direction should be dynamically adjusted since the object is expected to be moving randomly.
3. If the robot is very close to the closest object, it should stop moving forward.

### Code Explanation
1. First, I need to subscribe to `/scan` and make a publisher to `/cmd_vel` in init. I also set a `safe_distance` value for later uses when I need to command the robot to stop moving forward if the distance is too close.
2. In `scan_callback`, I first obtained the range data from the lds. I need to clean the zeros in the data because they are not useful for determining the closest object.
3. Then I find the minimum distance value and the corresponding index(`min_index`) within the range data. I need to use the `min_index` to determine whether the robot should turn right or left. This is as simple as determining whether the `min_index` is in the first half(left direction) or the second half(right direction).
4.  then publish a `twist` message based on the direction I need to turn with appropriate linear & angular velocities. In case the closest object is less than the `safe_distance`, I would set the linear velocity to -0.1 to make the robot back up a little bit.

### Videos
![person_folloower](https://github.com/Intro-Robotics-UChicago-Spring-2024/warmup-project-JihuiTanUchicago/assets/91858809/e900d572-7e6b-4955-b433-ff9a8db45f24)

## Wall Follower

### High-level Description
Command the robot to start following a wall no matter what the shape of the wall is or where the robot is.

### Approach
1.  Divide the range data from `lds` into 4 parts: front, left, right, back. We will need to use this division mechanism to determine what the robot should do. Specifically, we need to use the average distance data from the front, left, right, and back directions.
2.  Design an algorithm based on average distance data from the front, left, right, and back directions to make the robot go straight, turn left, or turn right. Details could be found in `code explanation` section.

### Code Explanation
1. Define a `wall_distance` in addition to a `safe_distance`. If the robot's smallest distance value is larger than `wall_distance`, it is said that the robot is away from the wall and needs to go back to the wall first. In this case, the program we wrote for `person_follower` will be used.
2. Define some functions `go_straight`, `go_right`, and `go_left`, as they will be frequently used.
3. `safe_distance` is used to prevent collision, as well as to tell the robot when to make a turn. Here is a breakdown of the algorithm I use:
```
we split the robot into 4 regions: front, back, left, right
if the front's distance average is less than the safe distance:
    if the left's distance average < right's distance average:
        turn right until left and back's distance averages are minimum
    else if
        turn left until right and back's distance averages are minimum
    else
        go straight safely
else
    if the left side is about to hit the wall but the front is farther than safe distance:
        turn right
    else if the right side is about to hit the wall but the front is farther than safe distance:
        turn left
    else
        go straight safely

to determine a tricky case where the left or right is about to hit the wall,
but the front is still larger than the safe distance,
we estimate the distance from the corner of the robot to the wall:
left corner to wall distance estimate 1: cos(45) * left_distance_avg
left corner to wall distance estimate 2: cos(45) * front_distance_avg
similarly, calculate right corner distance estimates 1 & 2

The principle is that if both estimates for the same corner are less than safe_distance,
it is very likely that the robot needs to make a turn in the opposite direction immediately to prevent a collision
```

### Challenges
For `drive_in_square`: The real environment is not always ideal, and calculated theoretical values do not always translate perfectly into a reality. I have had to test out various angular velocities to find out the correct value to make the robot turn perfectly 90 degrees.

For `person_follower`: 1) Not familiar with /scan and lds, so needed to take a little bit of time to search what kind of information could be available and useful from lds; 2) Had to try many different angular velocities to find the ideal value; 3) Had to figure out whether positive angular velocity values would make robot turn left or right; 4) Had to figure out whether the robot should turn right or left based on the information available from lds; 5) Had to figure out what's an ideal safe_distance value.

For `wall_follwer`: 1) The design of a smart algorithm took a lot of trials and errors, as well as some contemplation about how to best use the data from `lds`. It's hard to cover all the cases.


### Future Work
- For `drive_in_square`: If I had more time, I would have tried to incorporate LiDAR to help the robot turn perfectly 90 degrees by memorizing the environmental data changes.
- For `person_follower`: I could leverage open_cv or other computer vision packages to help identify a person in addition to using lidar data.
- For `wall_follower`: an even smarter algorithm to follow the wall more smoothly. If the enclosed space is too compact, the algorithm now might not be correct.

## Take-aways
- Robotics brought physical constraints to the programming environment. Something that works in theory does not mean it works in reality! A lot of values have to be determined(like how fast should the robot move or turn).
- With smart use of limited data, robots could be very intelligent. Algorithm designs could also make a simple robot with lidar and wheels to do a lot of things that involve solving complex tasks.
