// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.RetractConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.Arm;

public class setArm extends CommandBase {
  private final Arm arm;
  private DoubleSupplier angle;
  private DoubleSupplier extension;
  private DoubleSupplier intakeSpeed;
  private DoubleSupplier wristAngle;
  SlewRateLimiter retractRateLimiter = new SlewRateLimiter(
    RetractConstants.kMaxRetractionRotations*1.5,
    -RetractConstants.kMaxRetractionRotations*1.5, 0);
  SlewRateLimiter wristRateLimiter = new SlewRateLimiter(
    WristConstants.kMaxRangeOfMotion*2.0, 
    -WristConstants.kMaxRangeOfMotion*2.0, 0);


  /** Moves arm to a pose. */
  public setArm(double armAngle, double extension, double wristAngle, double intakeSpeed, Arm arm) {
    this.arm = arm;
    this.angle = ()->armAngle;
    this.extension = ()->extension;
    this.intakeSpeed = ()->intakeSpeed;
    this.wristAngle = ()->wristAngle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  public setArm(
      DoubleSupplier armAngle, 
      DoubleSupplier extension,
      DoubleSupplier wristAngle, 
      DoubleSupplier intakeSpeed, 
      Arm arm) {
    this.arm = arm;
    this.angle = armAngle;
    this.extension = extension;
    this.intakeSpeed = intakeSpeed;
    this.wristAngle = wristAngle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wristRateLimiter.reset(arm.getWristAngle());
    retractRateLimiter.reset(arm.getRetractRotations());
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //set arm angle
    //check to see if arm angle is where we want
    // if so set arm extension
    var extension = this.extension.getAsDouble();
    var angle = this.angle.getAsDouble();
    var intakeSpeed = this.intakeSpeed.getAsDouble();
    var wristAngle = this.wristAngle.getAsDouble();


    //TODO: We need to be mindful of extend poses
    // but current set up does not work
    // Only execute extending poses from carry
    arm.setRetractPID(retractRateLimiter.calculate(extension));
    arm.setWristPID(wristRateLimiter.calculate(wristAngle));
    arm.intakeMotor.set(intakeSpeed);
    arm.setArmPID(angle);


    if(true)return;
    //Check if arm is extended, if extended, retract before moving up/down
    //Might need to change to inches
    if(arm.getRetractRotations() >= RetractConstants.kMaxRetractionRotations/8.0){
      arm.setRetractPID(extension);
      arm.setWristPID(wristAngle);
      arm.intakeMotor.set(intakeSpeed);
      if (arm.getRetractRotations() -5 <= extension && arm.getRetractRotations() +5 >= extension){
        arm.setArmPID(angle);
      }
    }
    //If arm is not extended rotate before extending
    else{
      arm.setArmPID(angle);
      arm.setWristPID(wristAngle);
      arm.intakeMotor.set(intakeSpeed);
      if (arm.getArmAngle() -5 <= angle && arm.getArmAngle() +5 >= angle){
        arm.setRetractPID(extension);
      }
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.armMotor.set(0.2);
    arm.retractMotor.set(0.0);
    arm.intakeMotor.set(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
