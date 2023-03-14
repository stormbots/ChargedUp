// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;
import com.stormbots.Clamp;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ChassisConstants;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Chassis.Gear;

public class ChassisTurnGyro extends CommandBase {
  /** Creates a new ChassisTurnGyro. */
  AHRS navx;
  private Chassis chassis;
  private DoubleSupplier driverTurn;
  private DoubleSupplier driverForward;
  private double tolerance;
  private double angleError;

  public ChassisTurnGyro(DoubleSupplier forward, DoubleSupplier turn, double tolerance, Chassis chassis, AHRS navx) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.navx = navx;
    this.chassis = chassis;
    this.driverForward = forward;
    this.driverTurn = turn;
    this.tolerance = tolerance; 

    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        //Optionally, restrict our system response to avoid excess power 
    
    
        //turning left/right stuff
        double targetAngle = -180;// navx.getRotation2d().getDegrees()  < 0 ? 180 : -180;
        double angle =  navx.getRotation2d().getDegrees(); // negative to account for navx rotation relative to chassis

        SmartDashboard.putNumber("chassis/angle", angle);
        if( angle <-0){
          targetAngle = -180;
        }
        else if( angle > 0){
          targetAngle = 180;
        }    
        angleError = targetAngle - angle;
        angleError= MathUtil.clamp(angleError, -50, 50);
        double turnoutput = angleError*ChassisConstants.kTurnLowKP;
        turnoutput += ChassisConstants.kTurnLowKS*Math.signum(turnoutput);

        turnoutput += driverTurn.getAsDouble();

        chassis.arcadeDrive(driverForward.getAsDouble(), turnoutput);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.arcadeDrive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Clamp.bounded(angleError, -tolerance, tolerance);
  }
}
