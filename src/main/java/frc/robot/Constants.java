// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.stormbots.Lerp;

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
    
    
    // Arm inputs
    public static int kArmMotor=5;
    public static int kRetractMotor=7;
    public static int kRetractBrakeSolenoid=6;//freaking out
    public static int kArmAnalogEncoderChannel=9;

    //Hand+wrist
    public static int kWristMotorID=13;
    // public static int kWristServoChannel=8;
    public static int kIntakeSolenoid=4;
    public static int kIntakeMotor=9;
    public static int kWristAnalogEncoderChannel=0;

  
    //Vision and driver stuff
    public static int kCameraFrontID=0;
    public static int kCameraRearID=1;
    public static String kLimelightIP="10.28.11.100";

  }

  public  static class ChassisConstants{
    public static double kGeartrainHigh=16.36;
    public static double kGeartrainLow=4.49;
    public static double kWheelSpacing=28;//TODO estimated
    public static double kWheelDiameter=6;//TODO estimated, and possibly not even useful

    public static boolean kShiftHigh=true; //TODO
    public static boolean kShiftLow=!kShiftHigh; //TODO

    public static boolean kLeftInverted = false;
    public static boolean kRightInverted = !kLeftInverted;
  }


  public  static class VisionConstants{
    public static double kLimelightAngle=15;
    public static double kLimelightOffsetX=4; 
    public static double kLimelightOffsetY=2;
  }

  public static class ArmConstants{
    public static double kGeartrain = 1/140.0;
    public static double kAbsoluteAngleOffset=-90.25;
    public static double kAbsoluteAngleDistancePerRotation=360;
    public static double kMotorEncoderConversionFactor = 90/(71.12105560302734-1.4);

    //All FFs are in volts
    public static double kCosFFNear = 0; //feed forward to cause no motion as the arm is rotated around
    public static double kCosFFFar = 0.33;//In voltage
    public static double ksFFNear = 0; //FF that causes it to move again; Will probably be small
    public static double ksFFFar = 0;
    public static double kvFFNear = 0;
    public static double kvFFFar = 0;
    public static double kaFFNear = 0;
    public static double kaFFFar = 0;

    public static double kPNear = 0.03;
    public static double kINear = 0; 
    public static double kDNear = 0;

    public static double kPFar = kPNear; //TODO
    public static double kIFar = 0; 
    public static double kDFar = 0;

    public static float kSoftLimitReverseNear = -50;
    public static float kSoftLimitForwardNear = 90;
    public static float kSoftLimitReverseFar = -10;
    public static float kSoftLimitForwardFar = 90;
    
  }

  public static class RetractConstants{
    //Measured from center of turret mounting to center of wrist axis
    public static double kMinRetractionRotations=0;
    public static double kMaxRetractionRotations= 57;
    public static double kMinRetractionInches=0;
    public static double kMaxRetractionInches=46;
    public static float kRetractSoftLimitReverse = 0;
    public static float kRetractSoftLimitForward = 57;
  
    public static boolean ENGAGED = false;
    public static boolean DISENGAGED = !ENGAGED;
    public static double kGeartrain=5.56;
    public static double kStrapWidth=0.03;
    public static double kInnerDiameter=0.5;
    public static double kMaxOuterDiameter=1.5;

    public static double ksFFNear = -0.65; //volts at 0 rotatons, changes based on whether we are extending or retracting
    public static double ksFFFar = -.05; //volts at 40.880 rotations
    public static double kvFFNear = 0;
    public static double kvFFFar = 0; //-1.9 -.25
    public static double kaFFNear = 0;
    public static double kaFFFar = 0;

    public static double kPNear = 0.2;
    public static double kINear = 0; 
    public static double kDNear = 0;

    public static double kPFar = kPNear; //TODO
    public static double kIFar = 0; 
    public static double kDFar = 0;

  }

  public static class WristConstants{
    public static float kMinAngle=-75; 
    public static float kMaxAngle=43; 
    public static double kMinRotations=-75; 
    public static double kMaxRotations=10;
    public static double kConversionFactor= 360/(63/1 * 5/4.0);
    public static boolean kReverseMotor=false;
    public static double kAbsoluteAngleOffset=-140.493235;
    public static double kFFCos=0.042*12;
    public static double kP=1/90.0;
    public static double kI=0;
    public static double kD=0;
    public static double kAbsoluteAngleDistancePerRotation=360;

  }

  public static class IntakeConstants{
    public static boolean kClosedBoolean=false;
    public static boolean kOpenBoolean=!kClosedBoolean;
    public static int kCurrentLimitFree=25;
    public static int kCurrentLimitStall=18;
    public static boolean kIntakeMotorInverted=true;
  }

  /** Contains tuned paramaters for Practice Bot, if they differ from comp
   * 
   */
  public static void SetPracticebotValues(){

    //Hardware IDs
    HardwareID.kChassisMotorLeft=1;
    HardwareID.kChassisMotorLeftFollower=2;
    HardwareID.kChassisMotorRight=3;
    HardwareID.kChassisMotorRightFollower=4;
    HardwareID.kShifterSolenoid=1;
    HardwareID.kArmMotor=6;
    HardwareID.kRetractMotor=7;
    HardwareID.kRetractBrakeSolenoid=0;//freaking out
    HardwareID.kArmAnalogEncoderChannel=0;
    // HardwareID.kWristServoChannel=8;
    HardwareID.kWristMotorID=13;
    HardwareID.kIntakeSolenoid=3;
    HardwareID.kIntakeMotor=9;



    //Arm parameters
    ArmConstants.kAbsoluteAngleDistancePerRotation=90/(0.34-0.09);
    ArmConstants.kCosFFNear = 0; //feed forward to cause no motion as the arm is rotated around
    ArmConstants.kCosFFFar = 0.33;//In voltage
    ArmConstants.kPNear = 0.05;
    ArmConstants.kPFar = ArmConstants.kPNear; //TODO

    ArmConstants.kAbsoluteAngleOffset=32.645;
    ArmConstants.kSoftLimitReverseNear = -30;
    ArmConstants.kSoftLimitForwardNear = 90;
    ArmConstants.kSoftLimitReverseFar = -10;
    ArmConstants.kSoftLimitForwardFar = 90;


    //Retraction constants
    RetractConstants.kMinRetractionRotations=40;
    RetractConstants.kMaxRetractionRotations=40;
    RetractConstants.kMinRetractionInches=0;
    RetractConstants.kMaxRetractionInches=30;

    RetractConstants.kGeartrain=5.56;
    RetractConstants.kStrapWidth=0.03;
    RetractConstants.kInnerDiameter=0.5;
    RetractConstants.kMaxOuterDiameter=1.5;

    RetractConstants.ksFFNear = -0.65; //volts at 0 rotatons, changes based on whether we are extending or retracting
    RetractConstants.ksFFFar = -.05; //volts at 40.880 rotations

    RetractConstants.kPNear =0.05;

    //Wrist Constants
    IntakeConstants.kCurrentLimitFree=25;
    IntakeConstants.kCurrentLimitStall=18;


    // WristConstants.kMinAngle=0; 
    // WristConstants.kMaxAngle=0; 
    // WristConstants.kMinRotations=0; 
    // WristConstants.kMaxRotations=10;
    // WristConstants.kConversionFactor=(WristConstants.kMaxAngle-WristConstants.kMinAngle)/(WristConstants.kMaxRotations-WristConstants.kMinRotations);
    // WristConstants.kReverseMotor=false;


  }

}
