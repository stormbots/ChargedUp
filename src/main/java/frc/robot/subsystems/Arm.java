// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.stormbots.Clamp;
import com.stormbots.Lerp;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    /** Motors for the Tower */
    public CANSparkMax armMotor = new CANSparkMax(5, MotorType.kBrushless);
    public CANSparkMax armMotorA = new CANSparkMax(6, MotorType.kBrushless);
    public CANSparkMax retractMotor = new CANSparkMax(7, MotorType.kBrushless);
    public RelativeEncoder encoderArmMotor = armMotor.getEncoder();
    public Lerp armAnalogLerp = new Lerp(0, 93, -90, 90); //GET GEAR RATIO!!, this for 54:18 gear ratio
    DutyCycleEncoder armAbsEncoder = new DutyCycleEncoder(5);

    /** Motors for the Arm*/
    public CANSparkMax intakeMotor = new CANSparkMax(9, MotorType.kBrushless);
    // public RelativeEncoder encoderRotateMotor = rotateMotor.getEncoder();
    
    /** Solenod for the hand Variables */
    Servo wristServo = new Servo(8);
    Solenoid wristSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 3); //temp value
    public final boolean HAND_OPEN = true;
    SlewRateLimiter wristAngleEstimate = new SlewRateLimiter(2);
    double wristAngleTarget = 0.0;


    /** Variables for the arm */
    double targetArmPos = 0.0;
    double currentArmPos = 0.0;
    double armPower = 0.0;
    double armAngle = 0.0;
    
    /** Variables for the hand */
    double intakePower = 0.0;

    /** Arm Feet Forward */ 
    double kArmFF;

    /** Arm Max and Min values for angles */
    public static final double MAX_ANGLE = 90; //temp values
    public static final double MIN_ANGLE = 0; //temp values
    
    /** Wrist Linear Actuator Variables */
    double m_speed = 10.0; //temp value 10 mm/sec
    double m_length = 50.0;
    double setPos;
    double curPos;

    /**
     * Parameters for L16-R Actuonix Linear Actuators
     *
     * @param channel PWM channel used to control the servo
     * @param length max length of the servo [mm]
     * @param speed max speed of the servo [mm/second]
    */
    public Arm() {
      /** Note: this JUST FOR PRACTICE BOT!! */
      intakeMotor.setInverted(true); //Double check this!

      //SmartDashboard stuff
      //SmartDashboard.putData("Arm/Position", getArmAngle());

      armMotor.setInverted(false);
      armMotorA.follow(armMotor,true);
      
      /** Set the bounds for thwe wrist */
      wristServo.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);

      /**  */
    }


    /** Solenoid stuff */
    //rotateMotor.setSmartCurrentLimit(30); //Broke fix later for safety
    

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        /** SmartDashboard for the arm */
        SmartDashboard.getNumber("arm/ArmAngle", encoderArmMotor.getPosition());

    }
    
    // private Mode mode = Mode.CLOSEDLOOP;

    /** Boom Code */
    public Arm setAngle(double pos) {
        targetArmPos = pos;
        return this;
    }
    public double getArmAngle() {
        return encoderArmMotor.getPosition();
    }
    public boolean isOnTarget(double tolerance) {
        return Clamp.bounded(getArmAngle(), targetArmPos - tolerance, targetArmPos + tolerance);
    }
    public void configSetArmEncoderPosition(double angle) {
        encoderArmMotor.setPosition((int)armAnalogLerp.getReverse(angle));
    }

    public Arm setArmLength(double length) { //Do this later
         
      return this;
    }

    /** This for the Hand */
    public void openHand(){
        wristSolenoid.set(HAND_OPEN);
    }
    public void closeHand(){
        wristSolenoid.set(!HAND_OPEN);
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
    public boolean isFinished() {
      return curPos == setPos;
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}