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
  
  double lastTilt = 0;
  LinearFilter tiltDeltaFilter = LinearFilter.singlePoleIIR(0.2, 0.02);

  Lerp driveFFLerpLow = new Lerp(0, 12.5,  ChassisConstants.kDriveLowKSLevel, ChassisConstants.kDriveLowKSLevel);
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
    tiltDeltaFilter.reset();
    lastTilt = ChassisConstants.kNavxRollPositive*navx.getRoll();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //Get our Tilt system inputs
    var targetTilt = 0.0;
    var tilt = ChassisConstants.kNavxRollPositive*navx.getRoll();
    var tiltError = targetTilt - tilt;
    tiltError*=-1; //Unusual step of inverting error; this accounts for a positive system response (forward) correcting a negative error (tilted upward).
    var tiltDelta = tiltDeltaFilter.calculate(tilt-lastTilt);
    lastTilt = tilt;
    //Generate closed loop outputs
    var tiltOutP = tiltError*ChassisConstants.kDriveLowKPTilt;
    var tiltoutD = tiltDelta*ChassisConstants.kDriveLowKDTilt;
    var tiltOutput =  tiltOutP + tiltoutD;

    //Apply our FF in the direction our system wants to move to make the correction
    var tiltOutFF = driveFFLerpLow.get(Math.abs(tilt))*Math.signum(tiltOutput);
    tiltOutput += tiltOutFF;

    //Optionally, restrict our system response to avoid excess power
    var max=0.5;
    MathUtil.clamp(tiltoutD, -max, max); 


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
    double turnoutput = angleError*ChassisConstants.kTurnLowKP;
    turnoutput += ChassisConstants.kTurnLowKS*Math.signum(turnoutput); 

    // Add manual, generate output
    tiltOutput += driverForward.getAsDouble();
    turnoutput += driverTurn.getAsDouble();
    // tiltOutput = driverForward.getAsDouble()*.5;
    // turnoutput = driverTurn.getAsDouble()*.5;
    // chassis.arcadeDrive(output,turnoutput);
    chassis.arcadeDrive(0,0);

    // SmartDashboard.putNumber("balance/angle", -navx.getAngle());
    // SmartDashboard.putNumber("balance/turnoutput(noninverted)", turnoutput);
    // SmartDashboard.putNumber("balance/output", output);
    // SmartDashboard.putNumber("balance/angelError", angleError);
    // SmartDashboard.putNumber("balance/targetAngle", targetAngle);
    SmartDashboard.putNumber("balance/Roll", navx.getRoll());
    SmartDashboard.putNumber("balance/tilt", tilt);
    SmartDashboard.putNumber("balance/tiltError", tiltError);
    SmartDashboard.putNumber("balance/tiltDelta", tiltDelta);
    SmartDashboard.putNumber("balance/tiltOutP", tiltOutP);
    SmartDashboard.putNumber("balance/tiltOutD", tiltoutD);
    SmartDashboard.putNumber("balance/tiltOutput", tiltOutput);
    // SmartDashboard.putNumber("balance/driveksff", feedforward);
  

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
