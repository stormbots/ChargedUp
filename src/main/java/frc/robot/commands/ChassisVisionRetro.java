// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Chassis.Gear;
import frc.robot.subsystems.Vision.LimelightPipeline;

public class ChassisVisionRetro extends CommandBase {
  Chassis chassis;
  Vision vision;
  AHRS gyro;
  double x;
  double y;
  DoubleSupplier fwdPower;
  DoubleSupplier turnpower;
  LimelightPipeline pipeline;
  
  /** Creates a new ChassisVisionTargeting. */
  public ChassisVisionRetro(DoubleSupplier fwdPower, DoubleSupplier turnpower, LimelightPipeline pipeline, Chassis chassis, Vision vision, AHRS gyro) {
    this.fwdPower = fwdPower;
    this.turnpower = turnpower;
    this.chassis = chassis;
    this.vision = vision;
    this.gyro = gyro;
    this.pipeline = pipeline;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(chassis);
    addRequirements(vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    chassis.pidTurn.reset();
    chassis.setShifter(Gear.LOW);
    vision.setPipeline(pipeline);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double forward = fwdPower.getAsDouble();
    double outputTurn = turnpower.getAsDouble();

    if (vision.hasValidTarget() == false){
      chassis.arcadeDrive(forward,outputTurn);
      return;
    }
    else{
      outputTurn += chassis.pidTurn.getOutput(0, -vision.getX());
      chassis.arcadeDrive(forward, outputTurn);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    vision.setPipeline(LimelightPipeline.kNoVision);
    chassis.arcadeDrive(0, 0);
    chassis.setShifter(Gear.HIGH);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
