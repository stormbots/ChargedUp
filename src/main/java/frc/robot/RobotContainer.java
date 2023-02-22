// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.setArm;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.IntakeSolenoidPosition;
import frc.robot.subsystems.Arm.PlaceOrExecute;
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
  PneumaticHub PCH = new PneumaticHub(1);
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
   
    // boolean placing = false ;
    // Trigger PlacingTrigger = new Trigger(()->driver.button(2).getAsBoolean());
    // driver.button(1)
    // .and(PlacingTrigger)
    // .whileTrue(()->{})
    // .onFalse(()->{});
    PCH.clearStickyFaults();
    pdp.clearStickyFaults();
    if (Constants.isCompBot){
      PCH.enableCompressorAnalog(80, 110);
    }
    else{
      
    }

    
   
    
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
        ()->{chassis.arcadeDrive( driver.getRawAxis(1), driver.getRawAxis(2) *.75);}
        ,chassis)
      );

      //These values for the controller, these is joystick and will have to be adjusted
      arm.setDefaultCommand(new RunCommand(
        ()->{
          arm.driveArm(-operator.getRawAxis(1));
          arm.driveRetract(operator.getRawAxis(0));
          arm.wristServo.set(operator.getRawAxis(2));
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
  }


  private void configureOperatorBindings(){
 
    //CUBE/CONE SELECTOR
    operator.button(1).onTrue(new ConditionalCommand(
      new InstantCommand(()->arm.setIntake(IntakeSolenoidPosition.OPEN)), 
      new InstantCommand(()->arm.setIntake(IntakeSolenoidPosition.CLOSED)),
      ()->arm.getIntakePosition()==IntakeSolenoidPosition.CLOSED
    ));
     //PLACE/EXECUTE SELECTOR
    operator.button(3).onTrue(new ConditionalCommand(
      new InstantCommand(()->arm.setPlaceOrExecute(PlaceOrExecute.EXECUTE)), 
      new InstantCommand(()->arm.setPlaceOrExecute(PlaceOrExecute.PLACE)), 
      ()->arm.getPlaceOrExecute()==PlaceOrExecute.PLACE
    ));
    
    //INTAKE MOTORS DRIVE INWARDS
    operator.povCenter().whileFalse((new RunCommand(()->arm.intakeMotor.set(1.0))));
    
    operator.povCenter().onTrue(new RunCommand(()->arm.intakeMotor.set(0.1)));
   
    //INTAKE MOTOR MANUAL EJECT/
    operator.button(4).whileTrue(new RunCommand (()->{
      arm.intakeMotor.set(-0.1);
    }));
    operator.button(4).onFalse(new RunCommand (()->{
      arm.intakeMotor.set(0.0);
    }));

    //POSITION MID LEVEL
    operator.button(5).whileTrue(new ConditionalCommand(
      new ConditionalCommand(
        new setArm(42, 27, 0, 0.1, arm), 
        new setArm(35, 27, 0, -0.1, arm),
        ()->arm.getPlaceOrExecute()==PlaceOrExecute.PLACE) 
      ,
      new ConditionalCommand(
        new setArm(21, 12, 0, 0.1, arm), 
        new setArm(21, 12, 0, -0.1, arm),
        ()->arm.getPlaceOrExecute()==PlaceOrExecute.PLACE)
      ,
      ()->arm.getIntakePosition()==IntakeSolenoidPosition.CLOSED));

    
    
    //POSITION TOP LEVEL
    operator.button(5).whileTrue(new ConditionalCommand(
      new ConditionalCommand(
        new setArm(44.0, 46.0, 0, 0.1, arm), 
        new setArm(33.0, 46.0, 0, -0.1, arm),
        ()->arm.getPlaceOrExecute()==PlaceOrExecute.PLACE) 
      ,
      new ConditionalCommand(
        new setArm(44.0, 46.0, 0, 0.1, arm), 
        new setArm(44.0, 46.0, 0, -0.1, arm),
        ()->arm.getPlaceOrExecute()==PlaceOrExecute.PLACE)
      ,
      ()->arm.getIntakePosition()==IntakeSolenoidPosition.CLOSED));

    

    //PICKUP DOUBLE SUBSTATION
    operator.button(7).whileTrue(new setArm(65, 11, 1.0, 1.0, arm));
     
    //PICKUP FROM GROUND/SCORE LOW
    operator.button(8).whileTrue(new ConditionalCommand(
      new setArm(-35, 0.0, 0, 1.0, arm), 
      new setArm(-30, 5.0, 0.0, 1.0, arm),
      ()->arm.getIntakePosition()==IntakeSolenoidPosition.CLOSED)
    );
    
    //MOVE TO CARRY POSITION
    operator.button(2).whileTrue(new setArm(72, 0, 0, 0.1, arm));
  }


  
  public Command getAutonomousCommand(){
    
    // TODO: Move autos to a dedicated holder class and fetch it from there, don't clutter RobotContainer.f
    // return Autos.exampleAuto(exampleSubsystem);
    return autoChooser.getSelected();
  }
}
