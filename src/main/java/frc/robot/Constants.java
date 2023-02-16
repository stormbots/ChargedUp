// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static boolean isCompBot;

  public static class OperatorConstants {
    // organize constants in subclasses as needed
    // public static final int kDriverControllerPort = 0;
  }

  public static class HardwareID{
    // Drive train stuff
    public static int kChassisMotorLeft=1;
    public static int kChassisMotorLeftFollower=2;
    public static int kChassisMotorRight=3;
    public static int kChassisMotorRightFollower=4;
    public static int kShifterSolenoid=1;
    // public static int kChassisBrakeSolenoid=3; //Potential hardware, undecided
    
    // Arm inputs
    public static int kArmMotor=6;
    public static int kRetractMotor=7;
    public static int kRetractBrakeSolenoid=0;//freaking out
    public static int kArmAnalogEncoderChannel=0;

    //Hand+wrist
    public static int kWristServoChannel=8;
    public static int kIntakeSolenoid=3;
    public static int kIntakeMotor=9;

  
    //Vision and driver stuff
    public static int kCameraFrontID=0;
    public static int kCameraRearID=1;
    public static String kLimelightIP="192.28.11.100";

  }

  public  static class ChassisConstants{
    public static double kGeartrainHigh=16.36;
    public static double kGeartrainLow=4.49;
    public static double kWheelSpacing=28;//TODO estimated
    public static double kWheelDiameter=6;//TODO estimated, and possibly not even useful

    public static boolean kShiftHigh=false; //TODO
    public static boolean kShiftLow=!kShiftHigh; //TODO

    public static boolean kLeftInverted = true;
    public static boolean kRightInverted = !kLeftInverted;
    
  }

  public  static class VisionConstants{
    public static double kLimelightAngle=15;
    public static double kLimelightOffsetX=4; 
    public static double kLimelightOffsetY=2;
  }

  public static class ArmConstants{
    public static double kGeartrain = 1/140.0;
    public static double kAbsoluteAngleOffset=0;
    public static double kMinAngle = 0;
    public static double kMaxAngle = 90;

    //All FFs are in volts
    public static double kCosFFNear = 0; //feed forward to cause no motion as the arm is rotated around
    public static double kCosFFFar = 0.33;//In voltage
    public static double ksFFNear = 0; //FF that causes it to move again; Will probably be small
    public static double ksFFFar = 0;
    public static double kvFFNear = 0;
    public static double kvFFFar = 0;
    public static double kaFFNear = 0;
    public static double kaFFFar = 0;
  }

  public static class RetractConstants{
    /** Arm extension, in inches from center of rotation axis*/
    public static double kMinRetraction=0;
    /** Arm extension, in inches from center of rotation axis*/
    public static double kMaxRetraction=30;
    public static boolean ENGAGED = true;
    public static boolean DISENGAGED = !ENGAGED;
    public static double kGeartrain=5.56;
    public static double kStrapWidth=0.03;
    public static double kInnerDiameter=0.5;
    public static double kMaxOuterDiameter=1.5;

    public static double ksFFNear = -0.65; //volts at 0 rotatons, changes based on whether we are extending or retracting
    public static double ksFFFar = -.05; //volts at 40.880 rotations
    public static double kvFFNear = 0;
    public static double kvFFFar = 0;
    public static double kaFFNear = 0;
    public static double kaFFFar = 0;
  }

  public static class WristConstants{
  }

  public static class IntakeConstants{
    public static boolean kClosedBoolean=true;
    public static boolean kOpenBoolean=!kClosedBoolean;
    public static double kCurrentLimitFree=4;
    public static double kCurrentLimitStall=1;
  }

  public static class FunnelConstants{
    public static boolean kDeployedBooleanLeft=true;
    public static boolean kRetractedBooleanLeft=!kDeployedBooleanLeft;
    public static boolean kDeployedBooleanRight=true;
    public static boolean kRetractedBooleanRight=!kDeployedBooleanRight;
    public static double kCurrentLimitFree=8;
    public static double kCurrentLimitStall=2;
  }


  /** Contains tuned paramaters for Practice Bot, if they differ from comp
   * 
   */
  public static void SetPracticebotValues(){
    ArmConstants.kAbsoluteAngleOffset=32.645;


  }

}
