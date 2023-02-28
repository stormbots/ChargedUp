// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DrivetrainVisionTargeting;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.VisionTurnToTargetPose;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public Field2d field = new Field2d();
  public AHRS navx = new AHRS(Port.kMXP);
  public DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.3048);;
  public DifferentialDrivePoseEstimator pe =  new DifferentialDrivePoseEstimator(
    kinematics, navx.getRotation2d(),
    0, 0,
    new Pose2d(0,0, new Rotation2d())
  );

  public Drivetrain drivetrain = new Drivetrain(pe, navx, field);
  public Vision vision = new Vision(pe, navx, field);

  public Joystick driver = new Joystick(0);
  JoystickButton aimButton = new JoystickButton(driver, 8);

  
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    SmartDashboard.putData(field);

    // Configure the button bindings
    configureButtonBindings();
    aimButton.whileTrue(new VisionTurnToTargetPose(drivetrain, vision));
    //.whileHeld(new DrivetrainVisionTargeting(driver.getRawAxis(1),driver.getRawAxis(2),drivetrain, vision, navx));
    

    var cube1mid = field.getObject("cube1mid");
    cube1mid.setPose(14.73, 3, new Rotation2d(Math.PI));

    drivetrain.setDefaultCommand(new RunCommand(
      ()->{
        drivetrain.arcadeDrive(-driver.getRawAxis(1)/2.0, driver.getRawAxis(2)/2.0);
      },drivetrain));

  }

  public AHRS getNavx() {
    return navx;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // drivetrain.setDefaultCommand(
    //   new DrivetrainVisionTargeting(-driver.getRawAxis(1),driver.getRawAxis(2),drivetrain, vision, navx)
    //   );
  
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
