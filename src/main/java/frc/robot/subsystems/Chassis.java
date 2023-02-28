// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

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
  private CANSparkMax leftLeader = new CANSparkMax(Constants.HardwareID.kChassisMotorLeft,MotorType.kBrushless);
  private CANSparkMax leftFollower = new CANSparkMax(Constants.HardwareID.kChassisMotorLeftFollower,MotorType.kBrushless);
  private CANSparkMax rightLeader = new CANSparkMax(Constants.HardwareID.kChassisMotorRight,MotorType.kBrushless);
  private CANSparkMax rightFollower = new CANSparkMax(Constants.HardwareID.kChassisMotorRightFollower,MotorType.kBrushless);
  
  public RelativeEncoder leftEncoder = leftLeader.getEncoder();
  public RelativeEncoder rightEncoder = rightLeader.getEncoder();

  Solenoid shifter = new Solenoid(PneumaticsModuleType.REVPH, HardwareID.kShifterSolenoid);
  
  DifferentialDrive driveTrain = new DifferentialDrive(leftLeader, rightLeader);

  /** Creates a new Chassis. */
  public Chassis() {
    //Reset Encoders
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);

    for(CANSparkMax m : new CANSparkMax[]{leftLeader,rightLeader,leftFollower,rightFollower}){
      //Set limits for motors

      //m.setOpenLoopRampRate(0.2);

      m.setIdleMode(IdleMode.kBrake);
      m.clearFaults();
      //Restricts each motor to a max of 60 amps
      m.setSmartCurrentLimit(240/4, 240/4);//240 is sensible current limit to chassis
    }
    //Set motors to follow the same side
    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);

    leftLeader.setInverted(ChassisConstants.kLeftInverted);
    rightLeader.setInverted(ChassisConstants.kRightInverted);

    leftEncoder.setPositionConversionFactor(ChassisConstants.kEncoderConversionFactorLow);
    rightEncoder.setPositionConversionFactor(ChassisConstants.kEncoderConversionFactorLow);

    //Hardware defaults to low; Make sure encoders/hardware matches
    //shift high is done in robot.java
    setShifter(Gear.LOW);

  }

  public void setShifter(Gear gear){
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
  public void arcadeDrive(double power, double turn) {
    driveTrain.arcadeDrive(power,turn);
  }

 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("BusVoltage",  rightLeader.getBusVoltage());
    SmartDashboard.putNumber("chassis/voltLeftOutput", leftLeader.getAppliedOutput()/leftLeader.getBusVoltage());
    SmartDashboard.putNumber("chassis/voltRightOutput", leftLeader.getAppliedOutput()/rightLeader.getBusVoltage());
    SmartDashboard.putNumber("chassis/metersLeft", leftEncoder.getPosition());
    SmartDashboard.putNumber("chassis/rotationsLeft", leftEncoder.getPosition()/leftEncoder.getPositionConversionFactor());

  }
}
