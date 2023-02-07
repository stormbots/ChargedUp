// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Funnel extends SubsystemBase {
  public CANSparkMax leftFunnelMotor = new CANSparkMax(10, MotorType.kBrushless);
  public CANSparkMax rightFunnelMotor = new CANSparkMax(11, MotorType.kBrushless);
  public Solenoid rightSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 6);
  public Solenoid leftSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 5);
  
  /** Creates a new Funnel. */
  public Funnel() {
    leftFunnelMotor.follow(rightFunnelMotor, true);
    rightSolenoid.set(true);
    leftSolenoid.set(true);
  
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void down(){
    rightSolenoid.set(false);
    leftSolenoid.set(false);
  }
  public void up(){
    rightSolenoid.set(true);
    leftSolenoid.set(true);
  }
  public void rotateConesForward(){
    rightFunnelMotor.set(0.2);
  }
  public void rotateConesBackward(){
    rightFunnelMotor.set(0.2);
  }
  
}
