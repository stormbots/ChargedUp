// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;
import com.stormbots.closedloop.MiniPID;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ChassisConstants;
import frc.robot.FieldPosition.TargetType;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Chassis.Gear;

public class VisionTurnToTargetPose extends CommandBase {
  MiniPID turnpid;
  private TargetType targetType;
  DoubleSupplier fwdPower;
  DoubleSupplier turnPower;
  private Vision vision;
  private Chassis chassis;
  private AHRS navx;
  Field2d field;



  /** Creates a new VisionTurnToTargetPose. */
  public VisionTurnToTargetPose(DoubleSupplier fwdPower, DoubleSupplier turnPower, TargetType targetType, Field2d field, double tolerance, Chassis chassis, Vision vision, AHRS navx) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.fwdPower = fwdPower;
    this.turnPower = turnPower;

    this.targetType = targetType;
    this.vision = vision;
    this.chassis = chassis;
    this.navx = navx;
    turnpid = chassis.pidTurn;
    this.field =  field;

    addRequirements(chassis);
    addRequirements(vision);

  }
  //
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    chassis.pidTurn.reset();
    turnpid.reset();
    chassis.setShifter(Gear.LOW);
    vision.setPipeline(Vision.LimelightPipeline.kAprilTag);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    var target = vision.setTarget(targetType);
    double angle = vision.getAngleToTargetPose(target);
    SmartDashboard.putNumber("visionpose/Angle to target", angle);
    field.getObject("targetpose").setPose(target.toPose2d());

    //get feedforwards and pids
    // var kp = chassis.getShifterPosition()==Gear.HIGH ? ChassisConstants.kTurnHighKP : ChassisConstants.kTurnLowKP;
    // var kff = chassis.getShifterPosition()==Gear.HIGH ? ChassisConstants.kTurnHighKS : ChassisConstants.kTurnLowKS;

    //turning and driving
    // double turnOutput = angle*kp;
    // turnOutput += kff*Math.signum(turnOutput);

    var turnOutput = turnpid.getOutput(0.0, angle);
    SmartDashboard.putNumber("visionpose/turnoutput", turnOutput);


    //driver input
    var forward = fwdPower.getAsDouble();
    turnOutput += turnPower.getAsDouble();
    chassis.arcadeDrive(forward, -turnOutput);


    // double turnpower = turnpid.getOutput(angle, 0);
    // chassis.arcadeDrive(0, turnpower);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.arcadeDrive(0, 0);
    // chassis.setShifter(Gear.HIGH);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
