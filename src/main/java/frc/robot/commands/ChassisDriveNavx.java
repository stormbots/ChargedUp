/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;


import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;
import com.stormbots.Clamp;
import com.stormbots.closedloop.FB;
import com.stormbots.closedloop.MiniPID;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ChassisConstants;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Chassis.Gear;

public class ChassisDriveNavx extends CommandBase {
  private final Chassis chassis;
  private AHRS gyro;
  private double targetDistance;
  private DoubleSupplier targetBearingSupplier;
  private double targetBearing;
  // private double initialBearing;

  private double angleTolerance;
  private double distanceTolerance;
  private double distance=0;

  
  SlewRateLimiter distanceSlew;
  // MiniPID pid = new MiniPID(0.1/12, 0, 0);
  MiniPID pid = new MiniPID(ChassisConstants.kTurnLowKP, 0, 0);

  /**
   * Creates a new ChassisDriveManual.
   * All distance units should be in Meters
   */
  public ChassisDriveNavx(
    double targetDistance, 
    DoubleSupplier targetBearingSupplier, double angleTolerance, double distanceTolerance, AHRS gyro, Chassis chassis) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(chassis);
    this.chassis = chassis;
    this.gyro = gyro;
    this.targetDistance = targetDistance;
    this.targetBearingSupplier = targetBearingSupplier;
    this.angleTolerance = angleTolerance;
    this.distanceTolerance = distanceTolerance;
    distanceSlew = new SlewRateLimiter(1.5, -1.5, 0);
    pid.setI(0.018/10.0);
    pid.setMaxIOutput(0.1);
    pid.setContinuousMode(-180, 180);
  }

  public ChassisDriveNavx(
    double targetDistance, 
    DoubleSupplier targetBearingSupplier, double angleTolerance, double distanceTolerance,
    double velocity,
    AHRS gyro, Chassis chassis) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(chassis);
    this.chassis = chassis;
    this.gyro = gyro;
    this.targetDistance = targetDistance;
    this.targetBearingSupplier = targetBearingSupplier;
    this.angleTolerance = angleTolerance;
    this.distanceTolerance = distanceTolerance;
    distanceSlew = new SlewRateLimiter(velocity, -velocity, 0);
    pid.setI(0.018/10.0);
    pid.setMaxIOutput(0.1);
    pid.setContinuousMode(-180, 180);
  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    pid.reset();

    chassis.leftEncoder.setPosition(0);
    chassis.rightEncoder.setPosition(0);
    Timer.delay(0.04);    
    
    targetBearing = targetBearingSupplier.getAsDouble();

    distanceSlew.reset(0);
    // initialBearing = gyro.getAngle();
    pid.setSetpoint(targetBearing);

    chassis.setShifter(Gear.LOW);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // SmartDashboard.putNumber("Chassis/targetBearing", targetBearing);
    // SmartDashboard.putNumber("Chassis/targetBearingSupplier", targetBearingSupplier.getAsDouble());

    distance = (chassis.leftEncoder.getPosition()+ chassis.rightEncoder.getPosition() )/2;

    double currentAngle =  gyro.getRotation2d().getDegrees();
    //Uses PID to create a motion controlled turn value
    double turn = pid.getOutput(currentAngle);
    

    double targetDistance = distanceSlew.calculate(this.targetDistance);

    double forwardSpeed = FB.fb(targetDistance, distance, 0.6); // TABI 0.4 || PRACTICE 0.4


    turn+= Math.signum(turn)*ChassisConstants.kTurnLowKS*.5; //*.3 = no wobble, but some drift allowed
    // turn+= Math.signum(turn)*0.02; //TODO: fIXME WHEN MINIPID WORKS PROPERLY // TABIIIIIIIIIIIIIIIIIIIII


    chassis.arcadeDrive(
        forwardSpeed, 
        turn
      );


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
    chassis.arcadeDrive(0,0);
    // chassis.setShifter(Gear.HIGH); Probably don't want this

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    // angleTolerance = Math.copySign(angleTolerance, targetBearing.getAsDouble());

    // SmartDashboard.putNumber("Chassis/angle Error", gyro.getAngle() - (initialBearing + targetBearing) );
    // SmartDashboard.putNumber("Chassis/position Error", chassis.getAverageDistance() - targetDistance);

    var distanceOnTarget=Clamp.bounded(distance, targetDistance-distanceTolerance, targetDistance+distanceTolerance);
    var angleOnTarget = Clamp.bounded(gyro.getAngle(), targetBearing-angleTolerance, targetBearing+angleTolerance);

    // SmartDashboard.putBoolean("autos/drivenavx/atDistance", distanceOnTarget);
    // SmartDashboard.putBoolean("autos/drivenavx/atAngle", angleOnTarget);
    // SmartDashboard.putBoolean("autos/drivenavx/atBoth", angleOnTarget && distanceOnTarget);

    // SmartDashboard.putBoolean("Chassis/Exit Angle", Clamp.bounded(gyro.getAngle(), initialBearing+targetBearing-angleTolerance, initialBearing+targetBearing+angleTolerance));

    return angleOnTarget && distanceOnTarget;

    // return false; 
  }
}