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
  private double targetAngle;

  public ChassisTurnGyro(DoubleSupplier forward, DoubleSupplier turn, double targetAngle, double tolerance, Chassis chassis, AHRS navx) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.navx = navx;
    this.chassis = chassis;
    this.targetAngle = targetAngle;
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
        //turning left/right stuff
        double angle =  navx.getRotation2d().getRadians(); // negative to account for navx rotation relative to chassis
        angle = MathUtil.angleModulus(angle);
        angle = Math.toDegrees(angle);

        angleError = targetAngle - angle;

        var continuousMin=-180;
        var continuousMax=180;
        if(continuousMin != continuousMax){
          var continuousHalfRange = (continuousMax-continuousMin)/2.0;
          angleError %= (continuousHalfRange*2);
          if(angleError>continuousHalfRange) angleError-=2*continuousHalfRange;
          if(angleError<-continuousHalfRange) angleError+=2*continuousHalfRange;
        }
    
        angleError= MathUtil.clamp(angleError, -25, 25);

        double turnoutput = angleError*ChassisConstants.kTurnLowKP;
        turnoutput += ChassisConstants.kTurnLowKS*Math.signum(turnoutput);

        turnoutput += driverTurn.getAsDouble();
        turnoutput += navx.getRate() * .05/4.0;

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
    
    return Clamp.bounded(angleError, -tolerance, tolerance) && Math.abs(navx.getRate()) < 15 ;
  }
}
