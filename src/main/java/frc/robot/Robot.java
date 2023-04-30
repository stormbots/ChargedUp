// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Chassis.Gear;
import frc.robot.subsystems.Vision.LimelightPipeline;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command autonomousCommand;
  UsbCamera camera1;
  // UsbCamera camera2;
  private RobotContainer robotContainer;
  private boolean autoHasRun = false;

  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    SmartDashboard.putData(CommandScheduler.getInstance());
    LiveWindow.enableAllTelemetry();
    camera1 = CameraServer.startAutomaticCapture();
    String key = "/bot/isCompBot";
    Constants.isCompBot = Preferences.getBoolean(key,false);
    if(! Preferences.containsKey(key)){
      System.err.println("ROBOT COMP STATE NOT DEFINED:");
      System.err.println("Assuming COMP for safety: View Preferences to change");
      Preferences.setBoolean(key, true);
    }
    if(Constants.isCompBot==true){
      System.err.println("Comp Bot Detected!");
    }else{
      System.err.println("Practice Bot Detected! Using practice bot values");
      Constants.SetPracticebotValues();
    }
    
    robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    // SmartDashboard.putNumber("navx/angle", robotContainer.navx.getAngle());

    // SmartDashboard.putNumber("chassis/systemvoltage", robotContainer.pdp.getVoltage());
    // SmartDashboard.putBoolean("chassis/isHighGear", robotContainer.chassis.gearPosition==Gear.HIGH);
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit(){
    robotContainer.vision.setPipeline(LimelightPipeline.kNoVision); 
    robotContainer.chassis.setShifter(Gear.LOW);
   
    robotContainer.field.getObject("MeasuringTool").setPose(FieldPosition.kOriginOffsetX,FieldPosition.kOriginOffsetY,new Rotation2d());
    robotContainer.vision.setPipeline(LimelightPipeline.kAprilTag);
  }

  @Override
  public void disabledPeriodic(){
    robotContainer.arm.armMotor.setIdleMode(IdleMode.kBrake);

    if(autoHasRun == false){
      robotContainer.navx.reset();
    }
   
    //Put some useful simulation tools on the dashboard.
    robotContainer.field.getObject("tags").setPoses(
      FieldPosition.AprilTags.stream().map(Pose3d::toPose2d).toList()
    );
    robotContainer.field.getObject("bluepickup").setPoses(
      FieldPosition.BluePickupDouble.stream().map(Pose3d::toPose2d).toList()
    );
    robotContainer.field.getObject("redpickup").setPoses(
      FieldPosition.RedPickupDouble.stream().map(Pose3d::toPose2d).toList()
    );
    //done in chassis, but here just in case that one's not working
    // robotContainer.field.setRobotPose(robotContainer.chassis.pe.getEstimatedPosition());
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    robotContainer.navx.reset();
    Timer.delay(0.04);

    while(robotContainer.navx.isCalibrating()){
      Timer.delay(0.01);
      System.err.println("NAVX CALIBRATING");
      robotContainer.chassis.arcadeDrive(0, 0);
    }

    autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }

    autoHasRun = true;
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic(){
    //robotContainer.arm.setRetractBrake(RetractSolenoidPosition.DISENGAGED);
  }

  @Override
  public void teleopInit(){
    // if(camera1 == null) {camera1 = CameraServer.startAutomaticCapture(0);}
    // if(camera2 ==null)  {camera2 = CameraServer.startAutomaticCapture(2);}
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }

    robotContainer.chassis.setShifter(Gear.HIGH);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //robotContainer.arm.setRetractBrake(RetractSolenoidPosition.DISENGAGED);
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    // robotContainer.lighting.setColor(LedPattern.RED);
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    SmartDashboard.putNumber("navx/degrees", robotContainer.navx.getRotation2d().getDegrees());
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
