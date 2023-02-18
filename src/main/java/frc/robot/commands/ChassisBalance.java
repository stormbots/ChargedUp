// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

public class ChassisBalance extends CommandBase {
  AHRS navx;
  private Chassis chassis;
  
  LinearFilter filter = LinearFilter.singlePoleIIR(0.1, 0.02);
  
  DoubleSupplier driverForward;
  DoubleSupplier driverTurn;
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

  }

  // Called every time the scheduler runs while the command is scheduled.

  @Override
  public void execute() {
//highff = .0027
//lowff = .0017
//hightff = .003
//lowtff = .0015
//highrampff = .005
//LOWRAMPFF = .OO15
    double drivekp = 0.028/12.0; //proportional 
    double drivekd = 0.06/24.0; //derivatve 
    double turnkp = (0.01/10.0); // turn proportional 
    double ki = 0; //integral
  
    double driveksff = 0.0027; //feed forward
    double turnksff = 0.003;
    

    double error = (0- navx.getRoll());
    double feedforward = driveksff*Math.signum(error);
    double delta = navx.getRawGyroX();
    delta = filter.calculate(delta);
    double doutput = delta * drivekd;
    doutput *= Math.signum(error);
    double output = error*drivekp + feedforward - doutput; 

    
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
  

    chassis.arcadeDrive(output,-turnoutput);

    SmartDashboard.putNumber("balance/angle", -navx.getAngle());
    SmartDashboard.putNumber("balance/turnoutput(noninverted)", turnoutput);
    SmartDashboard.putNumber("balance/output", output);
    SmartDashboard.putNumber("balance/modangleE", angleError);
    SmartDashboard.putNumber("balance/targetAngle", targetAngle);
  

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
