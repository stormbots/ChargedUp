// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ChassisBalance;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.IntakeSolenoidPosition;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Chassis.Gear;
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
  Compressor compressor = new Compressor(1, PneumaticsModuleType.REVPH);
  public PowerDistribution pdp = new PowerDistribution(21,ModuleType.kRev);

  // The robot's subsystems and commands are defined here...
  public Chassis chassis = new Chassis();
  public Arm arm = new Arm();
  public Vision vision = new Vision();
  

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandJoystick driver = new CommandJoystick(0);
  private final CommandJoystick operator = new CommandJoystick(1);
  

  //Commands
  SendableChooser<Command> autoChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(){
   
   
    //compressor.clearStickyFaults();
    navx.reset();
    //SmartDashboard.putNumber("PCH #", compressor.getModuleNumber());

    // Configure the trigger bindings
    configureDefaultCommands();
    configureDriverBindings();
    configureOperatorBindings();
  }


  private void configureDefaultCommands(){
    chassis.setDefaultCommand(
      // this one's really basic, but needed to get systems moving right away.
       new RunCommand(
        ()->{chassis.arcadeDrive( -driver.getRawAxis(1), driver.getRawAxis(2) *.75);}
        ,chassis)
       );
      
      //These values for the controller, these is joystick and will have to be adjusted
      arm.setDefaultCommand(new RunCommand(
        ()->{
          arm.driveArm(-operator.getRawAxis(1));
          arm.driveRetract(operator.getRawAxis(0));
          arm.wristServo.set(operator.getRawAxis(2));
          //arm.setWristAngle(Lerp.lerp(operator.getRawAxis(3), -1, 1, 45,10));
        },arm
      ));
  }


  private void configureDriverBindings(){
    //DRIVER
    driver.button(6)
    .whileTrue(new RunCommand(()->{
      chassis.setShifter(Gear.LOW);
    }))
    .onFalse(new RunCommand (()->{
      chassis.setShifter(Gear.HIGH);
    }));
    driver.button(3).whileTrue(
      new ChassisBalance(()->-driver.getRawAxis(1)/2.0, ()->driver.getRawAxis(2)/2.0, chassis, navx)
    );
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
  private void configureOperatorBindings(){
    operator.button(7).whileTrue(new RunCommand(()->{
      //Pickup from double substation
      arm.testRetractPID(8.2);
      arm.testArmPID(61.0);
    })); 
    operator.button(5).whileTrue(new RunCommand(()->{
      //Score Mid
      arm.testArmPID(40.1);
      if (arm.armMotor.getEncoder().getPosition() >34){
        arm.testRetractPID(27);
      }
    })); 
    operator.button(6).whileTrue(new RunCommand (()->{
      //Score Top
      arm.testArmPID(44.0);
      if (arm.armMotor.getEncoder().getPosition() >40){
        arm.testRetractPID(48);
      }
      
    }));
    operator.button(8).whileTrue(new RunCommand (()->{
      //PickupGround
      arm.testRetractPID(0);
      if (arm.retractMotor.getEncoder().getPosition() < 5){
        arm.testArmPID(-30);
      }
    }));
    operator.button(3).onFalse(new RunCommand (()->{

      //Set to mid platform
      arm.intakeMotor.set(0.0);
    }));
    operator.button(4).whileTrue(new RunCommand (()->{
      //Set to mid platform
      arm.intakeMotor.set(-0.1);
    }));
    operator.button(4).onFalse(new RunCommand (()->{
      //Set to mid platform
      arm.intakeMotor.set(0.0);
    }));

    //INTAKE MOTORS DRIVE INWARDS
    operator.povCenter().whileFalse(new InstantCommand(()->arm.intakeMotor.set(1.0)));
    //CUBE/CONE SELECTOR
    operator.button(1).onTrue(new ConditionalCommand(
      new InstantCommand(()->arm.setIntake(IntakeSolenoidPosition.OPEN)), 
      new InstantCommand(()->arm.setIntake(IntakeSolenoidPosition.CLOSED)),
      ()->arm.getIntakePosition()==IntakeSolenoidPosition.CLOSED
    ));
   
    //Debug for boom encoder
    // operator.button(11).whileTrue(new InstantCommand(()->{
    //   arm.setRetractBrake(retractSolenoidPosition.DISENGAGED);
    //   arm.retractMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
    // })); 
    // operator.button(11).onFalse(new InstantCommand (()->{
    //   arm.retractMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    // }));
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(){
    // An example command will be run in autonomous

    // TODO: Move autos to a dedicated holder class and fetch it from there, don't clutter RobotContainer.f
    // return Autos.exampleAuto(exampleSubsystem);
    return autoChooser.getSelected();
  }
}
