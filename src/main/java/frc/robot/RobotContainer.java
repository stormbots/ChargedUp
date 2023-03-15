// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.ChassisConstants;
import frc.robot.FieldPosition.TargetType;
import frc.robot.commands.ChassisBalance;
import frc.robot.commands.ChassisDriveNavx;
import frc.robot.commands.ChassisTurnGyro;
import frc.robot.commands.ChassisVisionRetro;
import frc.robot.commands.setArm;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.IntakeSolenoidPosition;
import frc.robot.subsystems.Arm.PrepareOrExecute;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Chassis.Gear;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lighting;
import frc.robot.subsystems.Lighting.LedPattern;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.LimelightPipeline;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  //NavX Gyroscope and Accellerometer
  public Field2d field = new Field2d();
  public AHRS navx = new AHRS(Port.kMXP);
  public DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(ChassisConstants.kWheelSpacing);
  public DifferentialDrivePoseEstimator pe =  new DifferentialDrivePoseEstimator(
    kinematics, navx.getRotation2d(),
    0, 0,
    new Pose2d(0,0, new Rotation2d())
  );
  PneumaticHub PCH = new PneumaticHub(1);
  public PowerDistribution pdp = new PowerDistribution(21,ModuleType.kRev);

  // The robot's subsystems and commands are defined here...
  public Chassis chassis = new Chassis(pe, navx, field);
  public Arm arm = new Arm();
  public Intake intake = new Intake();
  public Vision vision = new Vision(pe, navx, field);
  public final Lighting lighting = new Lighting();

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
      PCH.enableCompressorAnalog(80, 120);
    }
    else{
    }

    
   
    
    //compressor.clearStickyFaults();
    SmartDashboard.putData("chassis/Reset Navx",new InstantCommand( ()-> navx.reset()));
    navx.reset();
    //SmartDashboard.putNumber("PCH #", compressor.getModuleNumber());
    SmartDashboard.putData(field);

    // Configure the button bindings
    //driver.button(8).whileTrue(new VisionTurnToTargetPose(chassis, vision));
    //.whileHeld(new DrivetrainVisionTargeting(driver.getRawAxis(1),driver.getRawAxis(2),chassis, vision, navx));
    

    var cube1mid = field.getObject("cube1mid");
    cube1mid.setPose(14.73, 3, new Rotation2d(Math.PI));

    
   



    configureAutos();

    // Configure the trigger bindings
    configureDefaultCommands();
    configureDriverBindings();
    configureOperatorBindings();
  }


  private void configureDefaultCommands(){
    chassis.setDefaultCommand(
      // this one's really basic, but needed to get systems moving right away.
       new RunCommand(
        ()->{
          SmartDashboard.putNumber("chassis/turnvalue",-driver.getRawAxis(2));
          chassis.arcadeDrive( -driver.getRawAxis(1), -driver.getRawAxis(2));}
        ,chassis)
       );

      //These values for the controller, these is joystick and will have to be adjusted
      arm.setDefaultCommand(new RunCommand(
        ()->{
          arm.driveArm(-operator.getRawAxis(1));
          arm.driveRetract(operator.getRawAxis(0));
          arm.driveWrist(operator.getRawAxis(2));
        },arm
      ));

      intake.setDefaultCommand(new InstantCommand(
        ()->{
        arm.intakeMotor.set(.20);
        },intake
      ));

      lighting.setDefaultCommand(new RunCommand(()->{
        if(arm.prepareOrExecute==PrepareOrExecute.EXECUTE){
          lighting.setColor(LedPattern.RED);
        }
        else if(arm.intakeSolenoidPosition==IntakeSolenoidPosition.OPEN){
          lighting.setColor(LedPattern.NEED_CUBE);
        }
        else{
          lighting.setColor(LedPattern.NEED_CONE);
        }  
      }, lighting));

  }


  private void configureDriverBindings(){
    //DRIVER
    driver.button(8)
    .onTrue(new InstantCommand(()->{
      chassis.setShifter(Gear.LOW);
    }))
    .onFalse(new InstantCommand(()->{
      chassis.setShifter(Gear.HIGH);
    }));
    driver.button(7).whileTrue(
      new ChassisBalance(()->-driver.getRawAxis(1)/2.0, ()-> driver.getRawAxis(2)/2.0, chassis, navx)
    );

    driver.button(2).whileTrue(new InstantCommand()
      .andThen(()->chassis.setShifter(Gear.LOW))
      .andThen(new ChassisTurnGyro(() -> -driver.getRawAxis(1),()-> driver.getRawAxis(2), 10, chassis, navx))
      .andThen( new ChassisVisionRetro( ()-> -driver.getRawAxis(1),()-> -driver.getRawAxis(2), LimelightPipeline.kMidCone, chassis, vision, navx) )
      .finallyDo((cancelled)->chassis.setShifter(Gear.HIGH))
    );

    driver.button(4).whileTrue(new InstantCommand()
      .andThen(()->chassis.setShifter(Gear.LOW))
      .andThen(new ChassisTurnGyro(() -> -driver.getRawAxis(1),()-> driver.getRawAxis(2), 20, chassis, navx))
      .andThen(new ChassisVisionRetro(()-> -driver.getRawAxis(1),()-> -driver.getRawAxis(2), LimelightPipeline.kHighCone, chassis, vision, navx))
      .finallyDo((cancelled)->chassis.setShifter(Gear.HIGH))
    );

  }


  private void configureOperatorBindings(){
 
    //CUBE/CONE SELECTOR
    operator.button(1).onTrue(new ConditionalCommand(
      new InstantCommand(()->arm.setIntake(IntakeSolenoidPosition.OPEN)), 
      new InstantCommand(()->arm.setIntake(IntakeSolenoidPosition.CLOSED)),
      ()->arm.getIntakePosition()==IntakeSolenoidPosition.CLOSED
    ));
    operator.button(3).onTrue(new InstantCommand(()->arm.setPrepareOrExecute(PrepareOrExecute.EXECUTE)));
    operator.button(3).onFalse(new InstantCommand(()->arm.setPrepareOrExecute(PrepareOrExecute.PREPARE)));
    //INTAKE MOTORS DRIVE INWARDS
    operator.povCenter().whileFalse((new RunCommand(()->arm.intakeMotor.set(1.0),intake)));
    operator.povCenter().onTrue(new RunCommand(()->arm.intakeMotor.set(0.2),intake) );
   
    //INTAKE MOTOR MANUAL EJECT/
    operator.button(4).whileTrue(new RunCommand (()->{
      arm.intakeMotor.set(-0.2);
    },intake));
    operator.button(4).onFalse(new InstantCommand (()->{
      arm.intakeMotor.set(0.0);
    },intake));

    //POSITION TOP LEVEL
    operator.button(5).whileTrue(new ConditionalCommand(
      new ConditionalCommand(
        //Place cones
        new setArm(48, 50, 48, 0.2, arm, intake), 
        //Execute cones
        new setArm(34.5, 50, 35, 0.2, arm, intake)
          .withTimeout(0.25)
          .andThen(()->arm.setIntake(IntakeSolenoidPosition.OPEN))
          .withTimeout(0.25)
          .andThen(new setArm(48, 50, 90, 0.2, arm, intake)),
        ()->arm.getPrepareOrExecute()==PrepareOrExecute.PREPARE)
      ,
      new ConditionalCommand(
        //Place cubes
        new setArm(35, 48, 6, 0.2, arm, intake), 
        //Execute cubes
        new setArm(35, 48, 6, -0.2, arm, intake),

        ()->arm.getPrepareOrExecute()==PrepareOrExecute.PREPARE)
      ,
      ()->arm.getIntakePosition()==IntakeSolenoidPosition.CLOSED));

    
    //POSITION MID LEVEL
    operator.button(6).whileTrue(new ConditionalCommand(
      new ConditionalCommand(
        //Place Cones
        new setArm(45.0, 25.0, 20, 0.2, arm, intake), 
        //Execute Cones
        new setArm(28.0, 25.0, 20, 0.2, arm, intake)
          .withTimeout(.25)
          .andThen(()->arm.setIntake(IntakeSolenoidPosition.OPEN)),
        ()->arm.getPrepareOrExecute()==PrepareOrExecute.PREPARE)
      ,
      new ConditionalCommand(
        //Get cube values
        //Place cubes 
        new setArm(29.0, 11.0, 4.0, 0.2, arm, intake),
        //Execute cubes 
        new setArm(29.0, 11.0, 4.0, -0.25, arm, intake),
        ()->arm.getPrepareOrExecute()==PrepareOrExecute.PREPARE)
      ,
      ()->arm.getIntakePosition()==IntakeSolenoidPosition.CLOSED));

    

    //PICKUP DOUBLE SUBSTATION
    operator.button(9).whileTrue(new setArm(50, 21, 11, 1.0, arm, intake));

    //PICKUP TIPPED CONE
    // operator.button(7).whileTrue(new InstantCommand()
    // .andThen(new setArm(45,0,-75,0,arm,intake).withTimeout(1))
    // .andThen(new setArm(10,0,-75,0,arm,intake).until(()->arm.isRobotOnTarget(3, 1, 1)))
    // .andThen(new setArm(-15,0,-100,1.0,arm,intake).withTimeout(1))
    // .andThen(new setArm(10,0, -75, 1.0, arm, intake))
    // );
    operator.button(7).whileTrue(new ConditionalCommand(
      new setArm(10,0,-75,0,arm,intake), 
      new setArm(-15,0,-100,1.0,arm,intake).withTimeout(0.5)
      .andThen(new setArm(10,0,-75,0,arm,intake)), 
      ()->arm.getPrepareOrExecute()==PrepareOrExecute.PREPARE))
      ;
  
    //PICKUP FROM GROUND/SCORE LOW
    operator.button(8).whileTrue(new InstantCommand()
      .andThen(new setArm(()->-25 ,()->6,()->0,()->0.2,arm,intake)
      .until(()->arm.isRobotOnTarget(20, 50, 7)).withTimeout(0.25))
      .andThen(commandBuilder(CommandSelect.kArmToPickupPosition))
    );
    
    //MOVE TO CARRY POSITION
    operator.button(2).whileTrue(new InstantCommand()
    .andThen(
      new setArm(()->arm.getArmAngle(),()->0,()->arm.getWristAngle(),()->0.2,arm,intake).withTimeout(0.3)
      //.until(()->arm.isRobotOnTarget(90, 10, 90))
    )
    .andThen(commandBuilder(CommandSelect.kArmToCarryPosition))
    );
    
    operator.button(2).onTrue(new InstantCommand(()->arm.setPrepareOrExecute(PrepareOrExecute.PREPARE)));

    // new InstantCommand()
    // .andThen()
    // ;
    ;
    //TEST SHOOTING CUBES
    operator.button(12).whileTrue(new RunCommand (()->{
      arm.intakeMotor.set(-1.0);
    },intake));
    operator.button(12).onFalse(new RunCommand (()->{
      arm.intakeMotor.set(0.0);
    },intake));
    //Sync Encoders & Clear Stickies
    operator.button(11).onTrue(new InstantCommand(()->{
      arm.armMotor.clearFaults();
      arm.retractMotor.clearFaults();
      arm.wristMotor.clearFaults();
      arm.intakeMotor.clearFaults();
      arm.armMotor.getEncoder().setPosition(arm.getArmAngleAbsolute());
      arm.wristMotor.getEncoder().setPosition(arm.getWristAngleAbsolute());
      chassis.leftLeader.clearFaults();
      chassis.leftFollower.clearFaults();
      chassis.rightLeader.clearFaults();
      chassis.rightFollower.clearFaults();
    }));
  }

  public enum CommandSelect{
    kLevelArmAndResetEncoder,
    kPlaceConeMidBackwards,
    kPlaceCubeMidBackwards,
    kPlaceCubeHighBackwards,
    kDriveToGamePiece,
    kDriveToChargerAndBalance,
    kArmToCarryPosition,
    kArmToPickupPosition,
  }

  /** Generate useful common auto parts that we can fit together */
  public Command commandBuilder(CommandSelect commandSnippet){
    switch(commandSnippet){
      case kArmToPickupPosition:
      return new ConditionalCommand(
      new setArm(-46, 0, -4, 1.0, arm, intake), //cone
      new setArm(-38, 6, -5, 1.0, arm, intake), //cube
      ()->arm.getIntakePosition()==IntakeSolenoidPosition.CLOSED)
      ;

      case kArmToCarryPosition:
      return new setArm(90, 0, 150, 0.3, arm, intake);

      case kDriveToGamePiece: 
      return new ChassisDriveNavx(Units.inchesToMeters(156+24), ()->0, 5 , Units.inchesToMeters(1), navx, chassis);

      case kDriveToChargerAndBalance:
      return new ChassisDriveNavx(Units.inchesToMeters(100), ()->0, 5 , Units.inchesToMeters(20), navx, chassis)
      .withTimeout(4)
      // .andThen( commandBuilder(CommandSelect.kArmToCarryPosition).until(()->arm.isRobotOnTarget(3, 1, 3)).withTimeout(1) )
      // .andThen(new WaitCommand(0.1))
      .andThen(new ChassisBalance(()->0, ()->0, chassis, navx))
      ;

      case kLevelArmAndResetEncoder:
      return new InstantCommand()
      .andThen(new setArm(80, 0, 90, 0.2, arm, intake).until(()->arm.isRobotOnTarget(5, 1, 10)).withTimeout(.5) )
      .andThen(new WaitCommand(0.1))
      //go to cube pickup
      //.andThen( commandBuilder(CommandSelect.kArmToPickupPosition).until(()->arm.isRobotOnTarget(3, 1, 3)).withTimeout(1) )
      .andThen(new InstantCommand(()->arm.armMotor.enableSoftLimit(SoftLimitDirection.kForward, true)))
      .andThen(new InstantCommand(()->arm.armMotor.enableSoftLimit(SoftLimitDirection.kReverse, true)))
      .andThen(new InstantCommand(()->{
          //reject jank; Only run this if the arm successfully got where it should have gone
          if( arm.getArmAngle()<90 ){ arm.armMotor.getEncoder().setPosition(arm.getArmAngleAbsolute() ); }
        }))
      ;

     case kPlaceConeMidBackwards:
     return new InstantCommand()
     .andThen(new InstantCommand(()->arm.setIntake(IntakeSolenoidPosition.CLOSED)))
     .andThen(new InstantCommand(()->arm.armMotor.enableSoftLimit(SoftLimitDirection.kForward, false)))
     .andThen(new InstantCommand(()->arm.armMotor.enableSoftLimit(SoftLimitDirection.kReverse, false)))
     //place 
     .andThen(new setArm(145, 27, 157, 0.2, arm, intake).withTimeout(.75))
     .andThen(new WaitCommand(0.1))
     //execute 
     .andThen(new setArm(166, 27, 172, 0.2, arm, intake).until(()->arm.isRobotOnTarget(3, 1, 3)).withTimeout(.75))
     .andThen(new InstantCommand(()->arm.setIntake(IntakeSolenoidPosition.OPEN)))
     .andThen(new WaitCommand(0.1))
     //put arm up somewhere away from posts
     .andThen(commandBuilder(CommandSelect.kLevelArmAndResetEncoder))
     ;

     case kPlaceCubeHighBackwards:
     return new InstantCommand()
     .andThen(new InstantCommand(()->arm.setIntake(IntakeSolenoidPosition.OPEN)))
     .andThen(new InstantCommand(()->arm.armMotor.enableSoftLimit(SoftLimitDirection.kForward, false)))
     .andThen(new InstantCommand(()->arm.armMotor.enableSoftLimit(SoftLimitDirection.kReverse, false)))
     //place 
     .andThen(new setArm(0, 0, 0, 0.2, arm, intake).withTimeout(.75))
     .andThen(new WaitCommand(0.1))
     //execute 
     .andThen(new setArm(0, 0, 0, -0.1, arm, intake).until(()->arm.isRobotOnTarget(3, 1, 3)).withTimeout(.75))
     .andThen(new WaitCommand(0.1))
     //put arm up somewhere away from posts
     .andThen(commandBuilder(CommandSelect.kLevelArmAndResetEncoder))
     ;

     case kPlaceCubeMidBackwards:
     return new InstantCommand()
     .andThen(new InstantCommand(()->arm.setIntake(IntakeSolenoidPosition.OPEN)))
     .andThen(new InstantCommand(()->arm.armMotor.enableSoftLimit(SoftLimitDirection.kForward, false)))
     .andThen(new InstantCommand(()->arm.armMotor.enableSoftLimit(SoftLimitDirection.kReverse, false)))
     //place get values
     .andThen(new setArm(0, 0, 0, 0.2, arm, intake).withTimeout(.75))
     .andThen(new WaitCommand(0.1))
     //execute get values
     .andThen(new setArm(0, 0, 0, -0.1, arm, intake).until(()->arm.isRobotOnTarget(3, 1, 3)).withTimeout(.75))
     .andThen(new WaitCommand(0.1))
     //put arm up somewhere away from posts
     .andThen(commandBuilder(CommandSelect.kLevelArmAndResetEncoder))
     ;
    }
    
    //Anything that's not fully defined or commented out, return nothing and skip it in any sequences
    return new InstantCommand();
  }

  public void configureAutos(){

    SmartDashboard.putData("autos/TurnOffLimit", new InstantCommand( 
      ()-> {
        arm.armMotor.enableSoftLimit(SoftLimitDirection.kForward, false) ;
        // arm.armMotor.enableSoftLimit(SoftLimitDirection.kReverse, false) ;        
      } 
    ));

    SmartDashboard.putData("autos/Level+Reset Arm", new InstantCommand().andThen(commandBuilder(CommandSelect.kLevelArmAndResetEncoder)));


    var blueLeftConePlaceMid = commandBuilder(CommandSelect.kPlaceConeMidBackwards)
    .andThen(commandBuilder(CommandSelect.kDriveToGamePiece));

    // var blueMiddleConeBalance =commandBuilder(CommandSelect.kPlaceConeMidBackwards)
    // .andThen(commandBuilder(CommandSelect.kDriveToChargerAndBalance));

    var blueRightConePlaceMid = commandBuilder(CommandSelect.kPlaceConeMidBackwards)
    .andThen(commandBuilder(CommandSelect.kDriveToGamePiece));
    
    var redRightConePlaceMid = commandBuilder(CommandSelect.kPlaceConeMidBackwards)
    .andThen(commandBuilder(CommandSelect.kDriveToGamePiece));

    // var redMiddleConeBalance = commandBuilder(CommandSelect.kPlaceConeMidBackwards)
    // .andThen(commandBuilder(CommandSelect.kDriveToChargerAndBalance));

    var redLeftConePlaceMid = commandBuilder(CommandSelect.kPlaceConeMidBackwards)
    .andThen(commandBuilder(CommandSelect.kDriveToGamePiece));

    var redLeftCubePlaceMid = commandBuilder(CommandSelect.kPlaceCubeMidBackwards)
    .andThen(commandBuilder(CommandSelect.kDriveToGamePiece));

    var redRightCubePlaceMid= commandBuilder(CommandSelect.kPlaceCubeMidBackwards)
    .andThen(commandBuilder(CommandSelect.kDriveToGamePiece));

    var blueLeftCubePlaceMid = commandBuilder(CommandSelect.kPlaceCubeMidBackwards)
    .andThen(commandBuilder(CommandSelect.kDriveToGamePiece));

    var blueRightCubePlaceMid = commandBuilder(CommandSelect.kPlaceCubeMidBackwards)
    .andThen(commandBuilder(CommandSelect.kDriveToGamePiece));
    
    var redRightCubePlaceHigh = commandBuilder(CommandSelect.kPlaceCubeHighBackwards)
    .andThen(commandBuilder(CommandSelect.kDriveToGamePiece));

    var redLeftCubePlaceHigh  = commandBuilder(CommandSelect.kPlaceCubeHighBackwards)
    .andThen(commandBuilder(CommandSelect.kDriveToGamePiece));

    var blueLeftCubePlaceHigh = commandBuilder(CommandSelect.kPlaceCubeHighBackwards)
    .andThen(commandBuilder(CommandSelect.kDriveToGamePiece));

    var blueRightCubePlaceHigh = commandBuilder(CommandSelect.kPlaceCubeHighBackwards)
    .andThen(commandBuilder(CommandSelect.kDriveToGamePiece));
  

    var balanceCommunityScoreConeMid = new InstantCommand()
    .andThen( commandBuilder(CommandSelect.kPlaceConeMidBackwards))
    .andThen(new ChassisDriveNavx(Units.inchesToMeters(160), ()->0, 10, Units.inchesToMeters(20), navx, chassis))
    .andThen(new ChassisDriveNavx(Units.inchesToMeters(-65), ()->0, 10, Units.inchesToMeters(15), navx, chassis))
    .andThen(new ChassisBalance(()->0, ()->0, chassis, navx));

    var balanceCommunityScoreCubeMid = new InstantCommand()
    .andThen( commandBuilder(CommandSelect.kPlaceCubeMidBackwards))
    .andThen(new ChassisDriveNavx(Units.inchesToMeters(160), ()->0, 10, Units.inchesToMeters(20), navx, chassis))
    .andThen(new ChassisDriveNavx(Units.inchesToMeters(-65), ()->0, 10, Units.inchesToMeters(15), navx, chassis))
    .andThen(new ChassisBalance(()->0, ()->0, chassis, navx));

    var balanceCommunityScoreCubeHigh = new InstantCommand()
    .andThen( commandBuilder(CommandSelect.kPlaceCubeHighBackwards))
    .andThen(new ChassisDriveNavx(Units.inchesToMeters(160), ()->0, 10, Units.inchesToMeters(20), navx, chassis))
    .andThen(new ChassisDriveNavx(Units.inchesToMeters(-65), ()->0, 10, Units.inchesToMeters(15), navx, chassis))
    .andThen(new ChassisBalance(()->0, ()->0, chassis, navx));

    // .andThen(

    var targetAngle = 90;
    var jankyFollowAembotOutOfCommunity= new InstantCommand()
    .andThen( commandBuilder(CommandSelect.kPlaceConeMidBackwards))
      //left turn 90
      .andThen(new ChassisDriveNavx(0, ()->targetAngle, 5, 10, navx, chassis))
      //go straight some distance
      .andThen(new ChassisDriveNavx(Units.inchesToMeters(48), ()->targetAngle, 5, 10, navx, chassis))
      //turn right 90
      .andThen(new ChassisDriveNavx(0, ()->0, 5, 10, navx, chassis))
      // turn some DynamicConstantDesc
      .andThen(new ChassisDriveNavx(Units.inchesToMeters(193), ()->0, 5, 10, navx, chassis))


    ;
      // new ChassisDriveNavx(targetDistance, targetBearingSupplier, angleTolerance, distanceTolerance, gyro, chassis)

    autoChooser.setDefaultOption("Do Nothing", new InstantCommand());
    autoChooser.addOption("Blue Left Cone",blueLeftConePlaceMid);
    autoChooser.addOption("Blue Left Cube Mid",blueLeftCubePlaceMid);
    autoChooser.addOption("Blue Left Cube High",blueLeftCubePlaceHigh);
    autoChooser.addOption("Blue Right Cone",blueRightConePlaceMid);
    autoChooser.addOption("Blue Right Cube Mid",blueRightCubePlaceMid);
    autoChooser.addOption("Blue Right Cube High", blueRightCubePlaceHigh);
    autoChooser.addOption("Red Left Cone",redLeftConePlaceMid);
    autoChooser.addOption("Red Left Cube Mid",redLeftCubePlaceMid);
    autoChooser.addOption("Red Left Cube High",redLeftCubePlaceHigh);
    autoChooser.addOption("Red Right Cone",redRightConePlaceMid);
    autoChooser.addOption("Red Right Cube Mid",redRightCubePlaceMid);
    autoChooser.addOption("Red Right Cube High", redRightCubePlaceHigh);
    autoChooser.addOption("Drive+Balance Only",commandBuilder(CommandSelect.kDriveToChargerAndBalance));
    autoChooser.addOption("Score then do nothing",commandBuilder(CommandSelect.kPlaceConeMidBackwards));
    autoChooser.addOption("Drive Only",commandBuilder(CommandSelect.kDriveToChargerAndBalance));
    autoChooser.addOption("Mobility+Balance+Score Cone Mid",balanceCommunityScoreConeMid);
    autoChooser.addOption("Mobility + Balance + Score Cube Mid", balanceCommunityScoreCubeMid);
    autoChooser.addOption("Mobility + Balance + Score Cube High", balanceCommunityScoreCubeHigh);
    autoChooser.addOption("UNTESTED Follow Aembot",jankyFollowAembotOutOfCommunity);



    SmartDashboard.putData("autos/Auto Chooser",autoChooser);
  }
  
  public Command getAutonomousCommand(){
    // return new RunCommand( ()->chassis.arcadeDrive(0.05, 0), chassis);
    // return new ChassisDriveNavx(1, ()->0, 5 , 0.01, navx, chassis);
    return autoChooser.getSelected();

  }
}
