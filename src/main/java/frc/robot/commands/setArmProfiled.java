// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.RetractConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class setArmProfiled extends CommandBase {
  private final Arm arm;
  private DoubleSupplier angle;
  private DoubleSupplier extension;
  private DoubleSupplier intakeSpeed;
  private DoubleSupplier wristAngle;
  SlewRateLimiter retractRateLimiter = new SlewRateLimiter(
    RetractConstants.kMaxRetractionRotations*1.5,
    -RetractConstants.kMaxRetractionRotations*1.5, 0);
  SlewRateLimiter wristRateLimiter = new SlewRateLimiter(
    WristConstants.kMaxRangeOfMotion*1.5, 
    -WristConstants.kMaxRangeOfMotion*1.5, 0);
  // SlewRateLimiter armRateLimiter =new SlewRateLimiter(
  //   180, 180, 0);


  TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
    WristConstants.kMaxRangeOfMotion*1.5*4, //degrees/s
    720 // degrees/s/s
    );
  TrapezoidProfile.State goal = new TrapezoidProfile.State(0, 0);
  TrapezoidProfile armProfile = new TrapezoidProfile(constraints, goal);
  double startTimer = 0;

  /** Moves arm to a pose. */
  public setArmProfiled(double armAngle, double extension, double wristAngle, double intakeSpeed, Arm arm, Intake intake) {
    this.arm = arm;
    this.angle = ()->armAngle;
    this.extension = ()->extension;
    this.intakeSpeed = ()->intakeSpeed;
    this.wristAngle = ()->wristAngle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
    addRequirements(intake);
    goal = new TrapezoidProfile.State(armAngle, 0);
  }

  public setArmProfiled(
      DoubleSupplier armAngle, 
      DoubleSupplier extension,
      DoubleSupplier wristAngle, 
      DoubleSupplier intakeSpeed, 
      Arm arm,
      Intake intake) {
    this.arm = arm;
    this.angle = armAngle;
    this.extension = extension;
    this.intakeSpeed = intakeSpeed;
    this.wristAngle = wristAngle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
    addRequirements(intake);
    goal = new TrapezoidProfile.State(armAngle.getAsDouble(), 0);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wristRateLimiter.reset(arm.getWristAngle());
    retractRateLimiter.reset(arm.getRetractRotations());

    startTimer = Timer.getFPGATimestamp();
    var initial = new TrapezoidProfile.State(arm.getArmAngle(), arm.armMotor.getEncoder().getVelocity());
    armProfile = new TrapezoidProfile(constraints, goal, initial);
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var extension = this.extension.getAsDouble();
    var intakeSpeed = this.intakeSpeed.getAsDouble();
    var wristAngle = this.wristAngle.getAsDouble();

    var targetPosition = armProfile.calculate(Timer.getFPGATimestamp()-startTimer).position;


    //TODO: We need to be mindful of extend poses
    // but current set up does not work
    // Only execute extending poses from carry
    arm.setRetractPID(retractRateLimiter.calculate(extension));
    arm.setWristPID(wristRateLimiter.calculate(wristAngle));
    arm.intakeMotor.set(intakeSpeed);
    arm.setArmPID(targetPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.armMotor.set(0.0);
    arm.retractMotor.set(0.0);
    arm.intakeMotor.set(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
