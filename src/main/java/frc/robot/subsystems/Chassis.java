// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ChassisConstants;
import frc.robot.Constants.HardwareID;

public class Chassis extends SubsystemBase {
  
  public static enum Gear{
    LOW,HIGH
  }

  //CAN ID's are placeholders
  public CANSparkMax leftLeader = new CANSparkMax(Constants.HardwareID.kChassisMotorLeft,MotorType.kBrushless);
  public CANSparkMax leftFollower = new CANSparkMax(Constants.HardwareID.kChassisMotorLeftFollower,MotorType.kBrushless);
  public CANSparkMax rightLeader = new CANSparkMax(Constants.HardwareID.kChassisMotorRight,MotorType.kBrushless);
  public CANSparkMax rightFollower = new CANSparkMax(Constants.HardwareID.kChassisMotorRightFollower,MotorType.kBrushless);
  
  public RelativeEncoder leftEncoder = leftLeader.getEncoder();
  public RelativeEncoder rightEncoder = rightLeader.getEncoder();

  Solenoid shifter = new Solenoid(PneumaticsModuleType.REVPH, HardwareID.kShifterSolenoid);
  public Gear gearPosition = Gear.LOW;

  public AHRS navx;

  DifferentialDrive driveTrain = new DifferentialDrive(leftLeader, rightLeader);
  private final DifferentialDriveOdometry m_odometry;


  /** Creates a new Chassis. */
  public Chassis(AHRS navx) {
    //Reset Encoders
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);

    for(CANSparkMax m : new CANSparkMax[]{leftLeader,rightLeader,leftFollower,rightFollower}){
      //Set limits for motors
      m.setOpenLoopRampRate(0.08);

      m.setIdleMode(IdleMode.kBrake);
      m.clearFaults();
      //Restricts each motor to a max of 60 amps
      m.setSmartCurrentLimit(200/4, 200/4);//240 is sensible current limit to chassis, but causing brownouts
    }
    //Set motors to follow the same side
    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);

    leftLeader.setInverted(ChassisConstants.kLeftInverted);
    rightLeader.setInverted(ChassisConstants.kRightInverted);

    this.navx = navx;
    setShifter(Gear.LOW); //Robot.teleopInit() will set this high for drivers

    m_odometry = new DifferentialDriveOdometry(navx.getRotation2d(), Units.inchesToMeters(leftEncoder.getPosition()), Units.inchesToMeters(rightEncoder.getPosition()));
  }

  public void setShifter(Gear gear){
    this.gearPosition = gear;
    var positionLeft = leftEncoder.getPosition();
    var positionRight = rightEncoder.getPosition();
    if(gear == Gear.HIGH){
      shifter.set(ChassisConstants.kShiftHigh);
      leftEncoder.setPositionConversionFactor(ChassisConstants.kEncoderConversionFactorHigh);
      rightEncoder.setPositionConversionFactor(ChassisConstants.kEncoderConversionFactorHigh);
    }
    else{
      leftEncoder.setPositionConversionFactor(ChassisConstants.kEncoderConversionFactorLow);
      rightEncoder.setPositionConversionFactor(ChassisConstants.kEncoderConversionFactorLow);
      shifter.set(ChassisConstants.kShiftLow);
    }
    leftEncoder.setPosition(positionLeft);
    rightEncoder.setPosition(positionRight);
  }

  public Gear getShifterPosition(){
    return this.gearPosition;
  }

  public void arcadeDrive(double power, double turn) {
    driveTrain.arcadeDrive(power,turn);
  }

 
  @Override
  public void periodic() {

    m_odometry.update(navx.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());

    // This method will be called once per scheduler run
    SmartDashboard.putString("ShifterPosition", getShifterPosition().toString());
    SmartDashboard.putNumber("BusVoltage",  rightLeader.getBusVoltage());
    SmartDashboard.putNumber("LeftAmps", leftLeader.getOutputCurrent());
    SmartDashboard.putNumber("RightAmps", rightLeader.getOutputCurrent());
    SmartDashboard.putNumber("LeftFAmps", leftFollower.getOutputCurrent());
    SmartDashboard.putNumber("RightFAmps", rightFollower.getOutputCurrent());
    // SmartDashboard.putNumber("BusVoltage",  rightLeader.getBusVoltage());
    SmartDashboard.putNumber("chassis/voltLeftOutput", leftLeader.getAppliedOutput()/leftLeader.getBusVoltage());
    SmartDashboard.putNumber("chassis/voltRightOutput", rightLeader.getAppliedOutput()/rightLeader.getBusVoltage());
    SmartDashboard.putNumber("chassis/metersLeft", leftEncoder.getPosition());
    SmartDashboard.putNumber("chassis/inchesLeft", Units.metersToInches(leftEncoder.getPosition()));
    SmartDashboard.putNumber("chassis/rotationsLeft", leftEncoder.getPosition()/leftEncoder.getPositionConversionFactor());
  }

  public Pose2d getPose(){
    return m_odometry.getPoseMeters();
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftLeader.setVoltage(leftVolts);
    rightLeader.setVoltage(rightVolts);
    driveTrain.feed();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(leftEncoder.getPosition()*0.305, rightEncoder.getPosition()*0.305); //i would like to know if there is a feet to meters method but i am too lazy to do that right now
  }

  public void resetEncoders(){
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(
        navx.getRotation2d(), leftEncoder.getPosition()*0.305, rightEncoder.getPosition()*0.305, pose);
  }
}
