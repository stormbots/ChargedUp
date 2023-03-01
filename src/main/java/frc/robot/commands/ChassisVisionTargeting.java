// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Vision;

public class ChassisVisionTargeting extends CommandBase {
  Chassis chassis;
  Vision vision;
  AHRS gyro;
  double x;
  double y;
  double fwdPower;
  double turnpower;
  /** Creates a new ChassisVisionTargeting. */
  public ChassisVisionTargeting(double fwdPower, double turnpower, Chassis chassis, Vision vision, AHRS gyro) {
    this.fwdPower = fwdPower;
    this.turnpower = turnpower;
    this.chassis = chassis;
    this.vision = vision;
    this.gyro = gyro;
    x = vision.getX();
    y = vision.getY();
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double forward = fwdPower;
    double outputTurn = turnpower;

    if (vision.hasValidTarget() == false){
      chassis.arcadeDrive(forward,outputTurn);
      return;
    }
    else{
      outputTurn = vision.pidTurn.getOutput(0, vision.getX());
      chassis.arcadeDrive(forward, outputTurn);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
