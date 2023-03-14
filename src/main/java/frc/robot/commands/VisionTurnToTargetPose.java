// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.stormbots.closedloop.MiniPID;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Chassis.Gear;

public class VisionTurnToTargetPose extends CommandBase {
  private Chassis drivetrain;
  MiniPID turnpid;
  private Vision vision;

  /** Creates a new VisionTurnToTargetPose. */
  public VisionTurnToTargetPose(Chassis chassis,Vision vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = chassis;
    this.vision = vision;
    turnpid = chassis.pidTurn;

    addRequirements(chassis);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turnpid.reset();
    drivetrain.setShifter(Gear.LOW);
    vision.setPipeline(Vision.LimelightPipeline.kAprilTag);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("testangletotarget", vision.testGetAngleToTargetTestPose());
    double turnpower = turnpid.getOutput(vision.testGetAngleToTargetTestPose(), 0);
    drivetrain.arcadeDrive(0, turnpower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}