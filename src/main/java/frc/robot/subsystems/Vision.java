// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.reflect.Array;
import java.util.List;

import com.kauailabs.navx.frc.AHRS;
import com.stormbots.closedloop.MiniPID;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldPosition;
import frc.robot.FieldPosition.TargetType;


public class Vision extends SubsystemBase {


  private AHRS gyro;

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  
  NetworkTableEntry tx = table.getEntry("tx"); // gets horizontal offset from crosshair
  NetworkTableEntry ty = table.getEntry("ty"); // gets vertical offset from crosshair
  NetworkTableEntry ta = table.getEntry("ta"); // gets target area; how much its taking from the camera screen
  NetworkTableEntry tv = table.getEntry("tv"); //valid target = 1, if target not valid, its equal to 0
  NetworkTableEntry bptable = table.getEntry("botpose"); //gets translation (x, y, z) and rotation (x, y, z) for bot pose
  
  public enum LimelightPipeline{
    kNoVision, kMidCone, kHighCone, kAprilTag
  }

  public double camAngle = 45.0; 
  public double camHeight = 7.0; 
  public double targetHeight = 37; 


  public double bpDefault [] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  public Rotation2d rot = new Rotation2d(0,0);
  public Pose2d botPose = new Pose2d(0, 0, new Rotation2d(0));
  private DifferentialDrivePoseEstimator poseEstimator;
  Field2d field;
  public Pose3d target = new Pose3d();



  /** Creates a new Vision. */
  public Vision(DifferentialDrivePoseEstimator poseEstimator, AHRS gyro, Field2d field) {
    this.poseEstimator = poseEstimator;
    this.gyro = gyro;
    this.field = field;
    



    // driverPipeline();
    setPipeline(LimelightPipeline.kNoVision);

  }

  public double getTargetHeading() {
    return gyro.getAngle() + tx.getDouble(0.0);
  }
  


  @Override
  public void periodic() {

    SmartDashboard.putNumber("vision/navxangle", gyro.getAngle());
    SmartDashboard.putNumber("vision/poseangle", poseEstimator.getEstimatedPosition().getRotation().getDegrees());
    SmartDashboard.putNumber("vision/navxangle", gyro.getAngle());


    //read values periodically
    boolean hasTargets = tv.getDouble(0) == 1 ? true : false;
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0); 

    SmartDashboard.putBoolean("vision/TargetValid", hasTargets);
    SmartDashboard.putNumber("vision/X", x);
    SmartDashboard.putNumber("vision/Y", y);

    double[] bp = bptable.getDoubleArray(bpDefault);
    if(Array.getLength(bp)<6) return;
        
    //post to smart dashboard periodically
    // SmartDashboard.putNumber("vision/Area", targetArea);
    SmartDashboard.putNumberArray("vision/Botpose", bp);
    SmartDashboard.putNumber("vision/TargetDistance", distance());
    SmartDashboard.putNumber("vision/DegFromTarget", testGetAngleToTargetTestPose());

    rot = new Rotation2d( Math.toRadians( bp[5]) );
    // botPose =  new Pose2d(bp[0]+15.980/2.0, bp[1]+8.210/2.0, rot);

    botPose =  new Pose2d(bp[0]+15.980/2.0-0.6, bp[1]+(8.210/2.0)+0.02, rot); //the -0.6 is just because trial and error

    poseEstimator.addVisionMeasurement(botPose, Timer.getFPGATimestamp());

    
  }

  public double distance() {

    double y = ty.getDouble(0.0);
    double distance = (targetHeight-camHeight)/Math.tan(y+camAngle);

    if (distance < 0) {
      return 0;
    }

    return distance;
  }
  

  public Pose2d getPose(){
    return botPose;
  }

  public double getX () {
    return tx.getDouble(0.0);
  }

  public double getY () {
    return ty.getDouble(0.0);
  }

  public boolean hasValidTarget() {
    return tv.getDouble(0) == 1 ? true : false;
  }

  public double testGetAngleToTargetTestPose(){
    Pose2d botpose = poseEstimator.getEstimatedPosition();
    Pose2d targetpose = field.getObject("cube1mid").getPose();

    double dx =  targetpose.getX() - botpose.getX();
    double dy = targetpose.getY() - botpose.getY();
    //trig
    SmartDashboard.putNumber("dx", dx);
    SmartDashboard.putNumber("dy", dy);
    double angle = Math.toDegrees(Math.atan2(dy,dx));
    // angle += botpose.getRotation().getDegrees();

    // var delta = targetpose.minus(botpose);
    // angle = delta.getRotation().getDegrees();
    // SmartDashboard.putNumber("vision/deltax", delta.getX());
    // SmartDashboard.putNumber("vision/deltay", delta.getY());
    // SmartDashboard.putNumber("vision/deltaAngle", delta.getRotation().getDegrees());

    double botposeAngle = (botpose.getRotation().getDegrees() % 360); //might be pose estimator not pose2d type
    SmartDashboard.putNumber("botposeAngle",botposeAngle);
    angle = botposeAngle - angle;
    return angle;
    // var delta = botpose.relativeTo(targetpose); //TODO possibly useful

  }



  public double getArmAngleToTarget(){
    //Do the math for arm angle here
    return 10;
  }

  public double getArmExtensionToTarget(){
    //Do the math for arm extension here
    return 20;
  }


  public void setTarget(TargetType targetType){
    //TODO: allow passing in offsets for the target, and then add them to the distance and heights to the target
    List<Pose3d> targetList=FieldPosition.GetTargetList(targetType);
    Pose2d botPose = poseEstimator.getEstimatedPosition();
    //Do the appropriate sorting type
    switch(targetType){
    case ConeHigh:
      target = FieldPosition.GetNearestToBearing(botPose,targetList);
      break; 
    case PickupSlide:
      target = targetList.get(0); //nothing to sort for this case
      break;
    case PickupDouble:
      target = FieldPosition.GetNearestByX(botPose,targetList);
      break;
    default:
      target = FieldPosition.GetNearestByY(botPose,targetList); //what we usually want
    }
  }

  public void setPipeline(LimelightPipeline pipeline){
    switch(pipeline){
      case kNoVision:
      table.getEntry("pipeline").setNumber(0);
      break;
      case kMidCone:
      table.getEntry("pipeline").setNumber(1);
      break;
      case kHighCone:
      table.getEntry("pipeline").setNumber(2);
      break;
      }
  }
}

