// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.stormbots.Lerp;
import com.stormbots.closedloop.MiniPID;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
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

  public static double ramprateHighGear = 0.13*2;
  public static double ramprateLowGear = 0.13;

  int maxChassisCurrentAllowance = 45;

  DoubleSolenoid shifter = new DoubleSolenoid(PneumaticsModuleType.REVPH, HardwareID.kShifterSolenoid,HardwareID.kShifterSolenoidb);
  public Gear gearPosition = Gear.LOW;
  DifferentialDrive driveTrain = new DifferentialDrive(leftLeader, rightLeader);
  private Field2d field;
  private AHRS navx;
  public DifferentialDrivePoseEstimator pe;

      //this should be in chassis, but here for simplicity temporarily
  public MiniPID pidTurn = new MiniPID(0,0,0)
  .setSetpointRange(15)
  .setP(ChassisConstants.kTurnLowKP) // GOAL/ACTUAL or 
  .setF((s,a,e)->{return Math.signum(e)*ChassisConstants.kTurnLowKS; })
  .setContinuousMode(-180,180)
  ;
  

  /** Creates a new Chassis. */
  public Chassis(DifferentialDrivePoseEstimator poseEstimator, AHRS navx, Field2d field) {
    this.field = field;
    this.navx = navx;
    this.pe = poseEstimator;

    //Reset Encoders
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);

    for(CANSparkMax m : new CANSparkMax[]{leftLeader,rightLeader,leftFollower,rightFollower}){
      // m.restoreFactoryDefaults();
      
      //Set limits for motors
      m.setOpenLoopRampRate(ramprateHighGear);
      
      m.setIdleMode(IdleMode.kBrake);
      m.clearFaults();
      //Restricts each motor to a max of 60 amps
      m.setSmartCurrentLimit(maxChassisCurrentAllowance);//240 is sensible current limit to chassis, but causing brownouts
    }
    //Set motors to follow the same side
    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);

    leftLeader.setInverted(ChassisConstants.kLeftInverted);
    rightLeader.setInverted(ChassisConstants.kRightInverted);

    setShifter(Gear.LOW); //Robot.teleopInit() will set this high for drivers
  }

  public void setShifter(Gear gear){
    //gear = gear.LOW;
    this.gearPosition = gear;
    var positionLeft = leftEncoder.getPosition();
    var positionRight = rightEncoder.getPosition();
    if(gear == Gear.HIGH){
      for(CANSparkMax m : new CANSparkMax[]{leftLeader,rightLeader,leftFollower,rightFollower}){
        m.setOpenLoopRampRate(ramprateHighGear);
      }
      shifter.set(ChassisConstants.kShiftHigh);
      leftEncoder.setPositionConversionFactor(ChassisConstants.kEncoderConversionFactorHigh);
      rightEncoder.setPositionConversionFactor(ChassisConstants.kEncoderConversionFactorHigh);
    }
    else{
      for(CANSparkMax m : new CANSparkMax[]{leftLeader,rightLeader,leftFollower,rightFollower}){
        m.setOpenLoopRampRate(ramprateLowGear);
      }
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

    SmartDashboard.putNumber("chassis/angle", navx.getRotation2d().getDegrees());
    //pe.update(rot, left.getEncoder().getPosition(), right.getEncoder().getPosition());
    pe.update(navx.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
    field.setRobotPose(pe.getEstimatedPosition());


    //MOTOR SAFETY PRECAUTION. Drop output if motors are too hot.
    for(CANSparkMax m : new CANSparkMax[]{leftLeader,rightLeader,leftFollower,rightFollower}){
      if(DriverStation.isFMSAttached())break;
      var temp = m.getMotorTemperature();
      if(temp<55)continue;
      var maxCurrent = Lerp.lerp(temp, 55, 65, maxChassisCurrentAllowance, maxChassisCurrentAllowance/4);
      // m.setSmartCurrentLimit((int)maxCurrent);
      var other = SmartDashboard.getNumber("chassis/Motor Overtemp!", 0);
      SmartDashboard.putNumber("chassis/Motor Overtemp!", Math.max(temp, other));
    }

    // This method will be called once per scheduler run
    SmartDashboard.putString("ShifterPosition", getShifterPosition().toString());
    // SmartDashboard.putNumber("BusVoltage",  rightLeader.getBusVoltage());
    // SmartDashboard.putNumber("chassis/voltLeftOutput", leftLeader.getAppliedOutput()/leftLeader.getBusVoltage());
    // SmartDashboard.putNumber("chassis/voltRightOutput", rightLeader.getAppliedOutput()/rightLeader.getBusVoltage());
    // SmartDashboard.putNumber("chassis/metersLeft", leftEncoder.getPosition());
    // SmartDashboard.putNumber("chassis/inchesLeft", Units.metersToInches(leftEncoder.getPosition()));
    // SmartDashboard.putNumber("chassis/rotationsLeft", leftEncoder.getPosition()/leftEncoder.getPositionConversionFactor());

    // SmartDashboard.putNumber("chassis/1 amp", leftLeader.getOutputCurrent());
    // SmartDashboard.putNumber("chassis/2 amp", leftFollower.getOutputCurrent());
    // SmartDashboard.putNumber("chassis/3 amp", rightLeader.getOutputCurrent());
    // SmartDashboard.putNumber("chassis/4 amp", rightFollower.getOutputCurrent());

    // SmartDashboard.putNumber("chassis/temp 1", leftLeader.getMotorTemperature());
    // SmartDashboard.putNumber("chassis/temp 2", leftFollower.getMotorTemperature());
    // SmartDashboard.putNumber("chassis/temp 3", rightLeader.getMotorTemperature());
    // SmartDashboard.putNumber("chassis/temp 4", rightFollower.getMotorTemperature());

  }
}