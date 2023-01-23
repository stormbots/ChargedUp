// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Chassis extends SubsystemBase {
  
  public static enum Gear{
    /** Utility enum to provide named values for shifted gear states 
    * This helps keep consistent types and umanbiguous functions
    */

    HIGH(true, true),
    LOW(false, false);
    private boolean compbot,practicebot;
    Gear(boolean compbot, boolean practicebot){
      this.compbot = compbot;
      this.practicebot = practicebot;
    }
    public boolean bool(){return Constants.isCompBot ? this.compbot : this.practicebot;};
  }

  private CANSparkMax leftLeader;
  private CANSparkMax rightLeader;
  private CANSparkMax leftFollower;
  private CANSparkMax rightFollower;

  public RelativeEncoder leftEncoder;
  public RelativeEncoder rightEncoder;

  Solenoid shifter;

  DifferentialDrive chassis;

  /** Creates a new Chassis. */
  public Chassis() {


    //CAN ID's are placeholders
    leftLeader = new CANSparkMax(1,MotorType.kBrushless);
    leftFollower = new CANSparkMax(2,MotorType.kBrushless);
    rightLeader = new CANSparkMax(3,MotorType.kBrushless);
    rightFollower = new CANSparkMax(4,MotorType.kBrushless);
    
    //Encoders for each side
    leftEncoder = leftLeader.getEncoder();
    rightEncoder = rightLeader.getEncoder();
    //Reset Encoders
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);

    for(CANSparkMax m : new CANSparkMax[]{leftLeader,rightLeader,leftFollower,rightFollower}){
      //Set limits for motors

      //m.setOpenLoopRampRate(0.2);

      m.setIdleMode(IdleMode.kBrake);

      //Restricts each motor to a max of 60 amps
      m.setSmartCurrentLimit(240/4, 240/4);//240 is sensible current limit to chassis
    }
    //Set motors to follow the same side
    leftLeader.follow(leftFollower);
    rightLeader.follow(rightFollower);

    leftLeader.setInverted(true);
    rightLeader.setInverted(!leftLeader.getInverted());

    //Declare drivetrain
    shifter = new Solenoid(PneumaticsModuleType.REVPH, Constants.isCompBot?1:1);

    shiftManual(Gear.LOW);
  }
  
  public void arcadeDrive(double power, double turn) {
    chassis.arcadeDrive(power,turn);
  }

  public void shiftManual(Gear gear){
    shifter.set(gear.bool());
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
