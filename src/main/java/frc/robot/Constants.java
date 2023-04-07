// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

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
    public static int kShifterSolenoid = 1;
    public static int kShifterSolenoidb = 14;
    
    
    // Arm inputs
    public static int kArmMotor=5;
    public static int kRetractMotor=7;
    public static int kRetractBrakeSolenoid=4;
    public static int kArmAnalogEncoderChannel=9;

    //Hand+wrist
    public static int kWristMotorID=13;
    // public static int kWristServoChannel=8;
    public static int kIntakeForwardSolenoid=5;
    public static int kIntakeReverseSolenoid=15;
    public static int kIntakeMotor=9;
    public static int kWristAnalogEncoderChannel=0;

  
    //Vision and driver stuff
    public static int kCameraFrontID=0;
    public static int kCameraRearID=1;
    public static String kLimelightIP="10.28.11.31"; //31, for pi. get it? Heh. 
  }

  public  static class ChassisConstants{
    public static double kGeartrainHigh=4.74;
    public static double kGeartrainLow=13.85;
    public static double kWheelSpacing=Units.inchesToMeters(22.695);//TODO estimated
    public static double kWheelDiameter=Units.inchesToMeters(6.1);
    // public static double kEncoderConversionFactorLow=1/ChassisConstants.kGeartrainLow*Math.PI*ChassisConstants.kWheelDiameter;
    public static double kEncoderConversionFactorLow=Units.inchesToMeters(100/66.66);
    public static double kEncoderConversionFactorHigh=1/ChassisConstants.kGeartrainHigh*Math.PI*ChassisConstants.kWheelDiameter;

    public static Value kShiftHigh=Value.kForward;
    public static Value kShiftLow= (kShiftHigh==Value.kForward ? Value.kReverse : Value.kForward);

    public static double kLowGearSlewRate = 1/0.35;
    public static double kHighGearSlewRate = 1/0.7;

    public static boolean kLeftInverted = false;
    public static boolean kRightInverted = !kLeftInverted;

    //basic chassis constants for levelling and proportional turning
    public static double kDriveLowKSLevel = 0.12;
    public static double kDriveLowKSTilted = 0.22+0.0; //for use only when driving up a tilted ramp
    public static double kDriveHighKSLevel = 0.2;
    public static double kDriveHighKSTilted = 0.0; //for use only when driving up a tilted ramp
    
    public static double kDriveLowKPTilt = 0.12/12.0; //proportional
    public static double kDriveLowKDTilt = 0;//0.15/12.0; //proportional
        
    public static double kTurnLowKS = 0.23-0.01-0.0; //proportional
    public static double kTurnHighKS = 0.31; //proportional
    public static double kTurnLowKP = 0.18/10.0; //proportional
    public static double kTurnHighKP = 0.03; //proportional 0.14/90/.25*0.02*kp 
    //ouput = g*= .14/ ( (90/.25) *0.02t )
    //.14*.25/90/0.02-> gain of 0.019

    //Constants for use with distance measures/pathfinding
    public static double kDriveLowKP = 0.0;
    public static double kDriveLowKI = 0.0;
    public static double kDriveLowKD = 0.0;

    public static double kDriveHighKP = 0.0;
    public static double kDriveHighKI = 0.0;
    public static double kDriveHighKD = 0.0;


    public static double kNavxRollPositive=-1;
  }


  public  static class VisionConstants{
    public static double kLimelightAngle=15;
    public static double kLimelightOffsetX=4; 
    public static double kLimelightOffsetY=2;
  }

  public static class ArmConstants{
    public static double kGeartrain = 1/140.0;
    public static double kAbsoluteAngleOffset=270-63+129-2;
    public static double kAbsoluteAngleDistancePerRotation=360;
    public static double kMotorEncoderConversionFactor = 90/(30.7-(-4.1));

    //All FFs are in volts
    public static double kCosFFNear = 0; //feed forward to cause no motion as the arm is rotated around
    public static double kCosFFFar = 0.7;//In voltage
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
    public static double kMaxRetractionRotations= 55;
    public static double kMinRetractionInches=0;
    public static double kMaxRetractionInches=46;
    public static float kRetractSoftLimitReverse = 0;
    public static float kRetractSoftLimitForward = 55;

    public static Boolean ENGAGED = false;
    public static Boolean DISENGAGED = true;
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
    public static float kMinAngle=-95; 
    public static float kMaxAngle=43; 
    public static double kMinRotations=-75; 
    public static double kMaxRotations=10;
    public static double kMaxRangeOfMotion = kMaxRotations-kMinRotations;
    public static double kConversionFactor= 360/(63/1 * 5/4.0);
    public static boolean kReverseMotor=false;
    public static double kAbsoluteAngleOffset=98+8;
    public static double kFFCos=0.042*12;
    public static double kP=1/70.0;
    public static double kI=0;
    public static double kD=0;
    public static double kAbsoluteAngleDistancePerRotation=360;

  }

  public static class IntakeConstants{
    public static Value kClosed=Value.kReverse;
    public static Value kOpen=Value.kForward;
    public static int kCurrentLimitFree=25;
    public static int kCurrentLimitStall=18;
    public static boolean kIntakeMotorInverted=true;
  }

  /** Contains tuned paramaters for Practice Bot, if they differ from comp
   * 
   */
  public static void SetPracticebotValues(){

    ChassisConstants.kLeftInverted = false;
    ChassisConstants.kRightInverted = !ChassisConstants.kLeftInverted;    
    ChassisConstants.kNavxRollPositive= -1;

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
    //HardwareID.kIntakeForwarSolenoid=3;
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
