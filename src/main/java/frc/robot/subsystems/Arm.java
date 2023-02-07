// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.nio.channels.ClosedByInterruptException;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import com.stormbots.Clamp;
import com.stormbots.Lerp;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.AngleStatistics;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class Arm extends SubsystemBase {
  public static enum IntakePosition{
    /** Utility enum to provide named values for intake solenoid states 
    * This helps keep consistent types and umambiguous functions
    */
    OPEN(true, true),
    CLOSED(false, false);
    private boolean compbot,practicebot;
    IntakePosition(boolean compbot, boolean practicebot){
      this.compbot = compbot;
      this.practicebot = practicebot;
    }
    public boolean bool(){return Constants.isCompBot ? this.compbot : this.practicebot;};
  }

  
    //-Motors to move the arm up and down
    public CANSparkMax armMotor = new CANSparkMax(5, MotorType.kBrushless);
    public CANSparkMax armMotorA = new CANSparkMax(6, MotorType.kBrushless);
    private SparkMaxPIDController armPID;

    //-Motor to move the arm in and out
    public CANSparkMax boomMotor = new CANSparkMax(7, MotorType.kBrushless);
    private SparkMaxPIDController boomPID;
    //Analog Encoder
    public DutyCycleEncoder shaftEncoder = new DutyCycleEncoder(0);//This is the DIO port on the roborio this is plugged into

    public Lerp armAnalogLerp = new Lerp(0, 93, -90, 90); //GET GEAR RATIO!!, this for 54:18 gear ratio

    /** Motor + Solenoidfor the intake**/
    public CANSparkMax handMotor = new CANSparkMax(9, MotorType.kBrushless);
    Solenoid wristSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 3); //temp value
    // public RelativeEncoder encoderRotateMotor = rotateMotor.getEncoder();
    
    //Wrist Servo for up/down
    Servo wristServo = new Servo(8);
   
    //Wrist variables
    SlewRateLimiter wristAngleEstimate = new SlewRateLimiter(2);
    double wristAngleTarget = 0.0;


    /** Variables for the arm */
    double targetArmPos = 0.0;
    double currentArmPos = 0.0;
    double armPower = 0.0;

    
    /** Variables for the hand */
    public double intakeSpeed = 0.3;
    public boolean isOpen;

    /** Arm Max and Min values for angles */
    public static final double MAX_ANGLE = 90; //temp values
    public static final double MIN_ANGLE = 0; //temp values
    
    //Constants for PIDS
    public double kBoomP =0.0;
    public double kBoomI =0.0;
    public double kBoomD =0.0;


    public double kArmP =0.0;
    public double kArmI =0.0;
    public double kArmD =0.0;


    //Angle from analog encoder
    public double armAngle =0.0;
    
    /**
     * Parameters for L16-R Actuonix Linear Actuators
     *
     * @param channel PWM channel used to control the servo
     * @param length max length of the servo [mm]
     * @param speed max speed of the servo [mm/second]
    */
    public Arm() {
      /** Note: this JUST FOR PRACTICE BOT!! */
      handMotor.setInverted(true); //Double check this!

      //Current Limits, Arbitrary
      armMotor.setSmartCurrentLimit(10);
      armMotorA.setSmartCurrentLimit(10);

      boomMotor.setSmartCurrentLimit(10);

      handMotor.setSmartCurrentLimit(10);

      armMotorA.follow(armMotor,true);
      
      //PIDS
      armMotor.getPIDController();

      armPID.setP(kArmP);
      armPID.setI(kArmI);
      armPID.setD(kArmD);
      armPID.setFF(0);

      boomMotor.getPIDController();

      boomPID.setP(kBoomP);
      boomPID.setI(kBoomI);
      boomPID.setD(kBoomD);
      boomPID.setFF(0);

      /** Set the bounds for thwe wrist */
      wristServo.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
      
      shaftEncoder.reset();
      /**  */
    }

    public Arm setAngle(double pos) {
        targetArmPos = pos;
        return this;
    }
    public double getArmAngle() {
        return armMotor.getEncoder().getPosition();
    }
    public boolean isOnTarget(double tolerance) {
        return Clamp.bounded(getArmAngle(), targetArmPos - tolerance, targetArmPos + tolerance);
    }
    public void configSetArmEncoderPosition(double angle) {
        armMotor.getEncoder().setPosition((int)armAnalogLerp.getReverse(angle));
    }


    public void setBoomLength(double length) { //Do this later
        boomPID.setReference(0, ControlType.kPosition, 0, 0, ArbFFUnits.kVoltage);
    }

    public void driveBoom(double power){
      boomMotor.set(power/4);
    }
    public void driveArm(double power){
      armMotor.set(power/4);
    }


    /**Hand Methods*/
    public Arm setHand(IntakePosition intake){
      wristSolenoid.set(intake.bool());
      return this;
    }
    public Arm setIntake(double power){
      handMotor.set(power);
      return this;
    }

    /** This for the wrist */
    // Run this method in any periodic function to update the position estimation of your servo
    public Arm setWristAngle(double angle) {
      angle = angle-getArmAngle();
      wristAngleTarget = angle;
      wristAngleEstimate.calculate(angle);
      double servoOut = Lerp.lerp(angle,0,40,0,1);
      wristServo.set(MathUtil.clamp(servoOut, 0, 1));
      return this;
    }

    //* Current position of the servo, must be calling {@link #updateCurPos() updateCurPos()} periodically 
    public double getWristAngle() {
      return wristAngleEstimate.calculate(wristAngleTarget);
    }
    //Checks if the servo is at its target position, must be calling {@link #updateCurPos() updateCurPos()} periodically @return true when servo is at its target
    // public boolean isFinished() {
    //   return curPos == setPos;
    // }

    
    @Override
    public void periodic() {

      
        // This method will be called once per scheduler run
        /** SmartDashboard for the arm */
        SmartDashboard.getNumber("arm/ArmAngle", armMotor.getEncoder().getPosition());

        //arm/open/wrist SmartDashboard stuff
        SmartDashboard.getNumber("arm/openwrist/targetWristPos", targetArmPos);
        // SmartDashboard.getBoolean("arm/wrist/OpenOrClosed", actuateHand(Intake.CLOSED));
        // SmartDashboard.getNumber("arm/wrist/voltage", );

        // // aropem/wrist/
        //   // targetArmPos
        //   open or close 
        //   output voltage
        // arm/boom 
        //   rotations
        //   targetlen
        //   actuallengt
        //   utput voltage
        // arm/ angle
        //   targetangle
        //   measured arm Angle
        //   measured abs encoder angle 
        //   output volts


    }
    
    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}