// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;
import com.stormbots.Lerp;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ChassisConstants;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Chassis.Gear;

public class ChassisBalance extends CommandBase {
  AHRS navx;
  private Chassis chassis;
  
  LinearFilter filter = LinearFilter.singlePoleIIR(0.1, 0.02);
  //measured practice bot values
  // Lerp driveFFLerpHigh = new Lerp(0, 12.5,  0.03, 0.24);//just %out
  // Lerp driveFFLerpLow = new Lerp(0, 12.5, 0.11, 0); //not done

  //comp bot values
    // fflow = 0.09
  // fftlow = .2
  // ramplow = .17

  // Lerp driveFFLerpHigh = new Lerp(0, 12.5,  0.03, 0.24);
  Lerp driveFFLerpLow = new Lerp(0, 12.5,  .02, 0.17);
  private DoubleSupplier driverForward;
  private DoubleSupplier driverTurn;

  /** Creates a new ChassisBalance. */
  public ChassisBalance(DoubleSupplier forward, DoubleSupplier turn, Chassis chassis, AHRS navx) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.navx = navx;
    this.chassis = chassis;
    this.driverForward = forward;
    this.driverTurn = turn;

    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    chassis.setShifter(Gear.LOW);
  }

  // Called every time the scheduler runs while the command is scheduled.

  @Override
  public void execute() {
    double drivekp = 0.14/12.0; //proportional 
    double drivekd = 0.3/24.0; //derivatve 
    double turnkp = (0.15/10.0); // turn proportional 
    double ki = 0; //integral
  
    double turnksff = 0.003;
    

    //Balance stuff
    double error = ChassisConstants.kNavxRollInvert*navx.getRoll();
    double feedforward = driveFFLerpHigh.get(Math.abs(error))*Math.signum(error);

    double delta = navx.getRawGyroX();
    delta = filter.calculate(delta);
    double doutput = delta * drivekd;
    doutput *= Math.signum(error);
    double output = error*drivekp + feedforward - doutput; 

    //turning left/right stuff
    double targetAngle = 0;
    double angle =  -navx.getAngle() % 180; // negative to account for navx rotation relative to chassis
    if( angle <-90){
      targetAngle = -180;
    }
    else if( angle > 90){
      targetAngle = 180;
    }
    double angleError = targetAngle - angle;
    SmartDashboard.putNumber("balance/error%", angleError);
    SmartDashboard.putNumber("balance/error%adjusted", angleError);
    double turnoutput = angleError * turnkp + turnksff*Math.signum(angleError); 


    output += driverForward.getAsDouble();
    turnoutput += driverTurn.getAsDouble();
    output = driverForward.getAsDouble()*.5;
    turnoutput = driverTurn.getAsDouble()*.5;
    chassis.arcadeDrive(output,turnoutput);

    SmartDashboard.putNumber("balance/angle", -navx.getAngle());
    SmartDashboard.putNumber("balance/turnoutput(noninverted)", turnoutput);
    SmartDashboard.putNumber("balance/output", output);
    SmartDashboard.putNumber("balance/modangleE", angleError);
    SmartDashboard.putNumber("balance/targetAngle", targetAngle);
    SmartDashboard.putNumber("balance/rollangle", error);
    SmartDashboard.putNumber("balance/driveksff", feedforward);
  

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.arcadeDrive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
