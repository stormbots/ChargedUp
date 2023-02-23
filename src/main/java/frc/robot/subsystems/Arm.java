// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import com.stormbots.Lerp;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.HardwareID;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.RetractConstants;
import frc.robot.Constants.WristConstants;

public class Arm extends SubsystemBase {
  
  public static enum RetractSolenoidPosition{
    ENGAGED,DISENGAGED
  }
  
  public static enum IntakeSolenoidPosition{
    OPEN,CLOSED
  }

  public static enum PlaceOrExecute{
    PLACE,
    EXECUTE
  }
  private RetractSolenoidPosition retractSolenoidPosition = RetractSolenoidPosition.DISENGAGED;
  private IntakeSolenoidPosition intakeSolenoidPosition = IntakeSolenoidPosition.CLOSED;
  private PlaceOrExecute placeOrExecute  = PlaceOrExecute.PLACE;

  
    //-Motors to move the arm up and down
    public CANSparkMax armMotor = new CANSparkMax(Constants.HardwareID.kArmMotor, MotorType.kBrushless);
    private SparkMaxPIDController armPID = armMotor.getPIDController();

    //-Motor to move the arm in and out
    public CANSparkMax retractMotor = new CANSparkMax(Constants.HardwareID.kRetractMotor, MotorType.kBrushless);
    private SparkMaxPIDController retractPID = retractMotor.getPIDController();
    //Analog Encoder
    public DutyCycleEncoder armAbsEncoder = new DutyCycleEncoder(Constants.HardwareID.kArmAnalogEncoderChannel);//This is the DIO port on the roborio this is plugged into

    public Lerp armAnalogLerp = new Lerp(0, 93, -90, 90); //GET GEAR RATIO!!, this for 54:18 gear ratio
    Lerp armksFFLerp = new Lerp(RetractConstants.kMinRetractionInches, RetractConstants.kMaxRetractionInches, ArmConstants.ksFFNear, ArmConstants.ksFFFar); //arm extension to ff output
    Lerp armkCosFFLerp = new Lerp(RetractConstants.kMinRetractionInches, RetractConstants.kMaxRetractionInches, ArmConstants.kCosFFNear, ArmConstants.kCosFFFar); //arm ext to ff output

    /** Motor + Solenoidfor the intake**/
    public CANSparkMax intakeMotor = new CANSparkMax(Constants.HardwareID.kIntakeMotor, MotorType.kBrushless);
    public Solenoid intakeSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.HardwareID.kIntakeSolenoid); 
    public Solenoid brakeSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.HardwareID.kRetractBrakeSolenoid);//temp value
   
    
    //Wrist Servo for up/down
    // public Servo wristServo = new Servo(Constants.HardwareID.kWristMotorID);
    public CANSparkMax wristMotor = new CANSparkMax(HardwareID.kWristMotorID, MotorType.kBrushless);
    public DutyCycleEncoder wristAbsEncoder = new DutyCycleEncoder(HardwareID.kWristAnalogEncoderChannel);

    //Wrist variables
    SlewRateLimiter wristAngleEstimate = new SlewRateLimiter(2);
    double wristAngleTarget = 0.0;


    /** Variables for the arm */
    double targetArmPos = 0.0;
    double currentArmPos = 0.0;
    double armPower = 0.0;
    Lerp retractFFLerp = new Lerp(0, 30, RetractConstants.ksFFNear, RetractConstants.ksFFFar); //inches/rotations to ff output
    
    /** Variables for the hand */
    public boolean isOpen;

    /** Arm Max and Min values for angles */
    public static final double MAX_ANGLE = 90; //temp values
    public static final double MIN_ANGLE = 0; //temp values
    
    //Constants for PIDS
    // public double kRetractP =0.0;
    // public double kRetractI =0.0;
    // public double kRetractD =0.0;


    //Angle from analog encoder
    public double armAngle =0.0;
    
    //integer for intake toggle
    public boolean intakeToggle = false;

    /**
     * Parameters for L16-R Actuonix Linear Actuators
     *
     * @param channel PWM channel used to control the servo
     * @param length max length of the servo [mm]
     * @param speed max speed of the servo [mm/second]
    */
    public Arm() {
      /** Note: this JUST FOR PRACTICE BOT!! */
      // armMotor.restoreFactoryDefaults();
      
      //Invert the intake
      intakeMotor.setInverted(IntakeConstants.kIntakeMotorInverted);
      
      armMotor.setIdleMode(IdleMode.kBrake);
      retractMotor.setIdleMode(IdleMode.kBrake);
      intakeMotor.setIdleMode(IdleMode.kCoast);

      //Current Limits, Arbitrary
      armMotor.setSmartCurrentLimit(80);
      //armMotorA.setSmartCurrentLimit(10);

      retractMotor.setSmartCurrentLimit(80);

      intakeMotor.setSmartCurrentLimit(IntakeConstants.kCurrentLimitStall, IntakeConstants.kCurrentLimitFree);

      //PIDS
      armPID.setP(ArmConstants.kPNear);
      armPID.setI(ArmConstants.kINear);
      armPID.setD(ArmConstants.kDNear);
      armPID.setFF(0);

      retractPID.setP(RetractConstants.kPNear);
      retractPID.setI(RetractConstants.kINear);
      retractPID.setD(RetractConstants.kDNear);
      retractPID.setFF(0);

      wristMotor.getPIDController().setP(WristConstants.kP);
      wristMotor.getPIDController().setI(WristConstants.kI);
      wristMotor.getPIDController().setD(WristConstants.kD);
      wristMotor.getPIDController().setFF(0);

      /** Set the bounds for thwe wrist */
      // wristServo.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
      armMotor.setSoftLimit(SoftLimitDirection.kReverse, ArmConstants.kSoftLimitReverseNear);
      armMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

      armMotor.setSoftLimit(SoftLimitDirection.kForward, ArmConstants.kSoftLimitForwardNear);
      armMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
      armMotor.setInverted(true);
      armMotor.getEncoder().setPositionConversionFactor(ArmConstants.kMotorEncoderConversionFactor);
      armMotor.getEncoder().setPosition(getArmAngleAbsolute());

      
      retractMotor.getEncoder().setPositionConversionFactor(1); //Set to use rotations for base unit
      retractMotor.setSoftLimit(SoftLimitDirection.kForward, RetractConstants.kRetractSoftLimitForward);
      retractMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
      retractMotor.setSoftLimit(SoftLimitDirection.kReverse, RetractConstants.kRetractSoftLimitReverse);
      retractMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
      
      
      wristMotor.setIdleMode(IdleMode.kCoast);
      wristMotor.setSmartCurrentLimit(30,30);
      wristMotor.setInverted(WristConstants.kReverseMotor);
      wristMotor.getEncoder().setPositionConversionFactor(WristConstants.kConversionFactor);
      wristMotor.setSoftLimit(SoftLimitDirection.kForward, 90); 
      wristMotor.setSoftLimit(SoftLimitDirection.kReverse, -90); 
      wristMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
      wristMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
      wristMotor.getEncoder().setPosition(getWristAngleAbsolute());

      
      setRetractBrake(RetractSolenoidPosition.DISENGAGED);
      setIntake(IntakeSolenoidPosition.CLOSED);

      //NOTE: We also re-set arm and wrist angles from the abs encoder in Robot.disabledPeriodic()
      // This is due to alleged boot up issues where the values from angle encoder are not 
      // properly scaled, resulting in incorrect angle readings.
    }


    public void setRetractBrake(RetractSolenoidPosition mode){
      this.retractSolenoidPosition = mode;
      if(mode == RetractSolenoidPosition.ENGAGED){
        brakeSolenoid.set(RetractConstants.ENGAGED);
      }
      else{
        brakeSolenoid.set(RetractConstants.DISENGAGED);
      }
    }

    public double getArmAngle() {
        return armMotor.getEncoder().getPosition();
    }
     
    // public double getRetractLength(){
    //   double rotations = retractMotor.getEncoder().getPosition()/retractMotor.getEncoder().getPositionConversionFactor();
    //   var id = RetractConstants.kInnerDiameter;
    //   var strapwidth = RetractConstants.kStrapWidth;
    //   var od = 2*rotations*strapwidth + id ; 
    //   var area = Math.PI/4.0*od*od - Math.PI/4.0*id*id;
    //   var length = area/strapwidth;
    //   return length;
    // }

    public double getRetractRotations(){
      return retractMotor.getEncoder().getPosition()/retractMotor.getEncoder().getPositionConversionFactor();
    }

    public void setRetractPID(double setpoint){
      retractPID.setP(RetractConstants.kPNear);
      retractPID.setIZone( 0.1 / retractPID.getI());
      var retractFF = Lerp.lerp(getRetractRotations(), 
        RetractConstants.kRetractSoftLimitReverse, RetractConstants.kRetractSoftLimitForward, 
        RetractConstants.ksFFNear, RetractConstants.ksFFFar)/12.0;
      retractPID.setReference(setpoint, ControlType.kPosition, 0, retractFF, ArbFFUnits.kVoltage);
    }

    public void setArmPID(double setpoint){
      armPID.setP(ArmConstants.kPNear);
      var armFF =Lerp.lerp(getRetractRotations(),
        RetractConstants.kRetractSoftLimitReverse, RetractConstants.kRetractSoftLimitForward, 
        ArmConstants.kCosFFNear, ArmConstants.kCosFFFar)/12.0;
      armPID.setReference(setpoint, ControlType.kPosition, 0, armFF, ArbFFUnits.kVoltage);
    }

    public void driveRetract(double power){
      //FF
      var retractFF = Lerp.lerp(getRetractRotations(), RetractConstants.kRetractSoftLimitReverse, 
      RetractConstants.kRetractSoftLimitForward, RetractConstants.ksFFNear, RetractConstants.ksFFFar)/12.0;
      retractMotor.set(power + retractFF);
    }

    public void driveArm(double power){
      var armFF = Lerp.lerp(getRetractRotations(), 
        RetractConstants.kRetractSoftLimitReverse, RetractConstants.kRetractSoftLimitForward,
        ArmConstants.kCosFFNear, ArmConstants.kCosFFFar) /12.0;
      armMotor.set(power/2.0 + armFF);
    }
    
    public void driveWrist(double power){
      //wristServo.set(Lerp.lerp(power,-1,1,0,1));
      var ff = WristConstants.kFFCos/12.0;
      ff *= Math.cos(Math.toRadians(getWristAngle()));
      wristMotor.set(power + ff );
    }
       
    public double getArmAngleAbsolute(){
      var angle = armAbsEncoder.getAbsolutePosition()*360-Constants.ArmConstants.kAbsoluteAngleOffset;
      if( angle >180 ){ 
        return angle -360; 
      }
      return angle;
    }

    /** Set wrist angle relative to ground/horizon*/
    public void setWristPID(double angle){
      //wristServo.set(Lerp.lerp(power,-1,1,0,1));
      angle -= getArmAngle();
      var ff = WristConstants.kFFCos/12.0;
      ff *= Math.cos(Math.toRadians(getWristAngle()));
      wristMotor.getPIDController().setReference(angle, ControlType.kPosition, 0, ff, ArbFFUnits.kVoltage);
    }

    /** Returns wrist angle relative to the arm using absolute encoder */
    public double getWristAngleAbsolute(){
      var angle = wristAbsEncoder.getAbsolutePosition()*360-WristConstants.kAbsoluteAngleOffset;
      if( angle >180 ){
        return angle -360;
      }
      return angle;
      }

    /**returns wrist angle relative to ground/horizon*/
    public double getWristAngle(){
      var angle = getArmAngle()+wristMotor.getEncoder().getPosition();
      return angle;
    }

  
    public void setIntake(IntakeSolenoidPosition handPosition){
      this.intakeSolenoidPosition=handPosition;
      if(handPosition == IntakeSolenoidPosition.CLOSED){
        intakeSolenoid.set(IntakeConstants.kClosedBoolean);
      }
      else{
        intakeSolenoid.set(IntakeConstants.kOpenBoolean);
      }
    }

    public IntakeSolenoidPosition getIntakePosition(){
      return this.intakeSolenoidPosition;
    }

    public void setPlaceOrExecute(PlaceOrExecute selection){
      this.placeOrExecute=selection;
    }
    
    public PlaceOrExecute getPlaceOrExecute(){
      return this.placeOrExecute;
    }
    

    @Override
    public void periodic() {
      //setWristAngle(wristAngleTarget);

      SmartDashboard.putNumber("arm/poseData/angleSparkEncoder", armMotor.getEncoder().getPosition());
      SmartDashboard.putNumber("arm/armangle/angleArmRotations", armMotor.getEncoder().getPosition()/armMotor.getEncoder().getPositionConversionFactor());
      SmartDashboard.putNumber("arm/armangle/absoluteAdjusted", getArmAngleAbsolute()); //??
      SmartDashboard.putNumber("arm/armangle/absoluteRaw", armAbsEncoder.getAbsolutePosition()); //??

      SmartDashboard.putNumber("arm/intake/intakeVolts", intakeMotor.getAppliedOutput()*intakeMotor.getBusVoltage());
      SmartDashboard.putNumber("arm/intake/intakeAmps", intakeMotor.getOutputCurrent());

      SmartDashboard.putNumber("arm/poseData/retractEncoderRotations", retractMotor.getEncoder().getPosition()/retractMotor.getEncoder().getPositionConversionFactor());
      SmartDashboard.putNumber("arm/retract/retractOutput", retractMotor.getAppliedOutput()*retractMotor.getBusVoltage());
      SmartDashboard.putNumber("arm/retract/retractamps", retractMotor.getOutputCurrent());
     
      SmartDashboard.putNumber("arm/armangle/outputVoltage", armMotor.getAppliedOutput()*armMotor.getBusVoltage());

      SmartDashboard.putNumber("arm/wrist/angleAbs", getWristAngleAbsolute());
      SmartDashboard.putNumber("arm/wrist/angleMotorEnc", wristMotor.getEncoder().getPosition());
      SmartDashboard.putNumber("arm/wrist/angleHorizon", getWristAngle());
      SmartDashboard.putNumber("arm/wrist/outputPow", wristMotor.getAppliedOutput());
      SmartDashboard.putNumber("arm/wrist/outputVolt", wristMotor.getAppliedOutput()/wristMotor.getBusVoltage());

      // SmartDashboard.putNumber("arm/poseData/wristAngle", Lerp.lerp(wristServo.getAngle(), 0,1,WristConstants.kMinAngle,WristConstants.kMaxAngle));
      // SmartDashboard.putNumber("arm/poseData/wristAngle", wristServo.get());
      SmartDashboard.putString("intakeState",getIntakePosition().toString());
      SmartDashboard.putString("executeToggle", getPlaceOrExecute().toString());
    }
    
    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}