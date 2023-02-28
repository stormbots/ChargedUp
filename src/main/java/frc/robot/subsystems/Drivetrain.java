// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Timer;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

  Field2d field;
  private AHRS navx;
  
  CANSparkMax left = new CANSparkMax(4,MotorType.kBrushless);
  CANSparkMax right = new CANSparkMax(1,MotorType.kBrushless);
  DifferentialDrive drive = new DifferentialDrive(left, right);
  Timer timer = new Timer();
  
  //DifferentialDriveKinematics kinematics;
  //DifferentialDriveOdometry odom = new DifferentialDriveOdometry(new Rotation2d(0), 0, 0);
  public DifferentialDrivePoseEstimator pe;
  // DifferentialDrivePoseEstimator poseEstimator; 

  
  /** Creates a new Drivetrain. */
  public Drivetrain(DifferentialDrivePoseEstimator poseEstimator, AHRS navx, Field2d field) {
    this.field = field;
    this.navx = navx;
    this.pe = poseEstimator;
    navx.reset();
    
    // DifferentialDrivePoseEstimator pe = new DifferentialDrivePoseEstimator(kinematics, navx.getRotation2d(), 0, 0, new Pose2d(0,0, navx.getRotation2d()));
    // DifferentialDriveKinematics ki= new DifferentialDriveKinematics(0.3048);
    left.setInverted(true);
    right.setInverted(false);

    var factor = 14/72.0*13*2.54/100.0;
    left.getEncoder().setPositionConversionFactor(factor);
    right.getEncoder().setPositionConversionFactor(factor);

  }

  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("gyro angle", -(navx.getAngle()%360));
    //pe.update(rot, left.getEncoder().getPosition(), right.getEncoder().getPosition());
    pe.update(navx.getRotation2d(), left.getEncoder().getPosition(), right.getEncoder().getPosition());
    field.setRobotPose(pe.getEstimatedPosition());
  }
  
  public void arcadeDrive(double power, double turn) {
    drive.arcadeDrive(power,turn);
  }

}
