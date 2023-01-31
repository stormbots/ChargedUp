// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  //NavX Gyroscope and Accellerometer
  public AHRS navx = new AHRS(Port.kMXP);
  Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);

  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
  public Chassis chassis = new Chassis();
  public Arm arm = new Arm();
  public Vision vision = new Vision();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandJoystick driver = new CommandJoystick(0);
  private final CommandJoystick operator = new CommandJoystick(1);

  //Commands
  ExampleCommand exampleCommand = new ExampleCommand(exampleSubsystem);
  SendableChooser<Command> autoChooser = new SendableChooser<>();



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(){
    navx.reset();

    chassis.setDefaultCommand(
      // this one's really basic, but needed to get systems moving right away.
      new RunCommand(
        ()->{chassis.arcadeDrive( -driver.getRawAxis(1), driver.getRawAxis(2) );}
        ,chassis)
      );
      //These values for the controller, these is joystick and will have to be adjusted
      arm.setDefaultCommand(new RunCommand(
        ()->{
          arm.armMotor.set(driver.getRawAxis(1)); 
          arm.retractMotor.set(driver.getRawAxis(2));
        }
      , arm));

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings(){
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    driver.button(0).whileTrue(exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(){
    // An example command will be run in autonomous

    // TODO: Move autos to a dedicated holder class and fetch it from there, don't clutter RobotContainer.
    // return Autos.exampleAuto(exampleSubsystem);
    return autoChooser.getSelected();
  }
}
