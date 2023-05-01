# General Coding Information

### Arm Control
The arm is largely controlled the [SetArm.java](blob/main/src\main\java\frc\robot\commands\setArm.java) command, which fully defines the pose (positions) of the arm. This includes the arm angle, extension, wrist angle, and the intake's direction. 

Using sequences of this command allows more complex multi-step operations. 

Properly defined poses and sequences are then used for all scoring and motion routines. 

### Drivetrain Control
Drivetrain is mostly human controlled, with some code assists. 

[ChassisBalance.java](blob/main/src\main\java\frc\robot\commands\ChassisBalance.java) is a utility command that assists in balancing the Charge Station. This rotates the robot to be perpindicular to the charge station, and drives uphill relatively slowly to work well with other robots. 

[ChassisTurnGyro.java](blob/main/src\main\java\frc\robot\commands\ChassisTurnGyro.java) is a utility function associated with our vision + chassis pose code. It simply turns the robot to a target facing angle. It's used in conjunction with the vision system to ensure that we're facing the targets when we'd expect to see one. 

[ChassisRetroVision.java](blob/main/src\main\java\frc\robot\commands\ChassisVisionRetro.java) uses the Limelight and the retroreflective targets to align the chassis. This combines with the poses from [SetArm.java](blob/main/src\main\java\frc\robot\commands\setArm.java) to quickly and accurately position the robot and place game pieces.

[ChassisTurnToTargetPose.java](blob/main/src\main\java\frc\robot\commands\VisionTurnToTargetPose.java) aligns the robot to arbitrary field positions, such as the cube nodes or pickup stations. This relies on the chassis's pose estimation and limelight's april tags, as well as pre-defined points of interest inside [FieldPosition.java](blob/main/src\main\java\frc\robot\FieldPosition.java).


# Arm Control Logic
The arm is mostly driven by the tuned Feed Forwards that neutralize the force of gravity. This ensures that when the robot is not being driven, it simply stays put. This greatly reduces the amount of precision tuning needed for a good control loop, and offers consistent performance across the full range of motion.

The side effect is that it can be manipulated by hand to help generate desired poses. This greatly simplifies getting certain positions, since we can just set it, read back encoder values, and then code the bot to return to that position.

### Setup and initialization
The Arm pivot and Wrist pivot both contain absolute encoders: This allows the bootup sequence to read a definitive known position, and configure the motor encoders.

The arm uses a chain drive, and thus slip is extremely unlikely: As a result, the arm is driven directly off of the motor feedback to minimizes oscillation due to any backlash. In this case, the absolute encoder is only read at the start of the match, or if the driver manually resyncs with the panic button.

The wrist is driven with belts, and will slip when driven quickly, or when driven against obstacles. As a result, the wrist is continuously resynced to the absolute encoder. This slip was the result of design changes, and is the reason it's not driven directly off the absolute encoder.

The retraction is _not_ detected using absolute states, and _must_ be retracted fully to contain accurate encoder readings. However, if the arm detects a negative extension, then it will reset the position to zero. This allows the arm to be retracted after being powered on, although only by manually turning the crank when disabled.

### Arm pivot control
These revolve around two critical criteria: The arm angle, and the extension. These both drive the generated feed-forwards that neutralize the force of gravity, and keep the arm in a stable position at all times. 

The arm angle serves as the parameter for the angular feed forward: This is greatest when the arm is fully extended horizontally, and zero when vertical.

The arm extension serves as the parameter for the magnitude of the feed forward: The feed forward is greatest when extended, and minimal when retracted. 

Because the torque changes linearly with extension, we can interpolate between the required feed-forward parameters at fully extendend and fully retracted. This yields the rough math of `cos(angle)*interp(extension,retractedFF,extendedff)`. The fully implemented function is in [Arm.java :: DriveArm()](blob/main/src/main/java/frc/robot/subsystems/Arm.java#L241)

### Extension controls

Normal operation relies on the same parameters as the arm pivot: Extension and angle, with code in [Arm.java :: DriveRetract()](blob/main/src/main/java/frc/robot/subsystems/Arm.java#L191).

The arm extension is designed with constant-force springs that extend it, and a belt to retract it. This means that the feed forward is always negative, pulling the arm inward to balance those forces. 

The extension also employs a pnuematic brake to solve a spring-related issue. The springs always apply force, even when the motors cannot, such as when disabled or powered off. This pnuematic brake is disengaged by code when the robot is operating, and can apply power. By use of the PCM's solenoid defaults, the brake is engaged when the robot is disabled, ensuring that the arm cannot extend.

### Wrist control
The wrist control relies on two parameters: The arm pivot angle and the wrist angle. The sum of these angles provides the parameter for a cosine feed forward, `cos(armangle+wristangle)*wristFFGain`. 

The wrist motor control loop PIDs to the arm's relative angle, but control inputs are absolute angles relative to horizon. This facilitates trying to plan motions, and simplifies thinking about the wrist independently of the system.


# The Human Inputs

### The Driver
The driver is responsible for steering and drive train. They have a few additional buttons for shifting, engaging vision steering, and balancing, but all very simple. 

### The Operator
Actual operation is fairly complex [so we have a PDF guide on some of the buttons and processes](blob/main/OperatorHandbook.pdf)

The conceptual operation, however, in is fairly straightforward. 
- The operator selects the active game piece. This sets the intake solenoid, as well as the lights. The lights serve to provide clear feedback, as well as inform the Human Player at the loading station.
- The operator picks up a game piece pickup position. The specific position is determined by the active game piece flag in code. 
- With the game piece aquired, the operator returns to the "holding" position for traversal across the field. 
- The operator puts the arm at the "scoring preperation" pose. This is above the post for a cone, or above the cube node for the cube.
- The operator confirms the placement with another button, which scores the game piece. This ejects a cube, or moves the arm down for a cone. 
- For the cone, there's one additional step: Open the intake to release the cone. 
- With the game piece scored, driver returns to the Carry position, and the process restarts. 

With no buttons pressed, the robot sits in Manual mode. In this case, the operator joystick controls the arm. At the joystick neutral position, no motion is applied. While the driver _can_ use this for some custom actions, it's generally only a fallback for special adjustments.

Operator controls lean on SetArm.java, stepping through sequences of pre-defined Poses. Because these buttons operate on a "hold to move" basis, they can be cancelled out at any time by releasing the button. This releases the bot to Manual mode, and 

### Carry Pose + Safety Features
The Carry position is special pose: This is inside the frame, with the game piece in a stable, optimal position, intended for field travel. This puts the arm almost vertical, with the wrist tilted slightly backwards, and the intake running inward at a light stall: This ensures the game piece does not bounce outward. 

However, getting _to_ this pose is also an important consideration, as we would return here from almost any position: Fully extended after placing a cone, from floor pickup, backwards, or manually adjusted positions for unusual actions. 

As such, this button includes some sequencing to mitigate issues from these poses. 

The initial motion retracts the arm toward the bot, and has different effects from different poses:
- From a floor pickup position, this ensures that the game piece is off the floor
- From a placing position, it ensures that the pivot doesn't try to move while it's fully extended
- From a driver station pickup, this is a potential issue, as it slightly pushes downward on the drive station. However, as the wrist also pivots upward, this is negated, and for cones, this simply helps seat them as intended.

The high cone placement _can_ also generate some unexpected consequences: If the human makes a mistake, the arm can be low enough that the wrist (or wiring) may contact the middle post. To combat this, the initial motion _also_ sets a control bound: It will move the arm to at least ~40 degrees, which serves to clear the arm from the middle post. This shouldn't happen in normal operations. 

The secondary motion simply goes to the final end position. 

Code-wise, this button also serves to reset the multi-step state machine used for placing game pieces. This ensures that the operator always clears it between game pieces without having to add an extra manual step to do so. 

### The panic reset button
This button aims to mitigate the effects of hardware and setup faults, such as temporary power loss to some or all of the robot, belts/drive slips, or drive team setup issues.

This button
- resets all hardware sticky faults
- Resyncs the motors to the absolute encoders
- While held, slowly retracts arm, ignoring soft limits.

Ideally, this button never needs pressing. However, having it ensures that errors on the field are recoverable, and don't cause match loss. 

