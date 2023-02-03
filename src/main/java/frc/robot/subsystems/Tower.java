// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Tower extends SubsystemBase {

  CANSparkMax motor = new CANSparkMax(Constants.HardwareID.kTurretMotor, MotorType.kBrushless);
  RelativeEncoder encoder = motor.getEncoder();
  SparkMaxPIDController pid = motor.getPIDController();
  double ks = 0; //Volts

  /** Creates a new Tower. */
  public Tower() {
    
    encoder.setPositionConversionFactor(10);
    encoder.setPosition(0); //TODO read current position from absdolute encoder

    motor.setSoftLimit(SoftLimitDirection.kForward, 20);
    motor.enableSoftLimit(SoftLimitDirection.kForward, true);
    //TODO set limit for kreverse

    pid.setP(0);
    pid.setI(0);
    pid.setD(0);
    pid.setFF(0);

    setAngle(0);
  }

  public void setAngle(double angle){
    pid.setReference(0, ControlType.kPosition, 0, ks, ArbFFUnits.kVoltage);
  }

  public void getAngle(){
    encoder.getPosition();
  }
  
  public boolean isOnTarget(){
    return isOnTarget(3);
  }
  public boolean isOnTarget(double degrees){
    return true;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //print useful development values
  }

}
