// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.reflect.Array;
import java.util.List;

import com.kauailabs.navx.frc.AHRS;
import com.stormbots.closedloop.MiniPID;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Vision extends SubsystemBase {

  public enum Color{RED,BLUE}

  //example of an enum
  public enum VisionTargets{
    k1(Color.BLUE,0,0,0),
    k2(Color.BLUE, 0,0,0),
    k3(Color.BLUE, 0,0,0),
    ;

    public  double x;
    public double y;
    public double z;
    public Color color;

    VisionTargets(Color color, double x, double y, double z){
      this.color = color;
      this.x = x; 
      this.y = y; 
      this.z = z;
    }
  }


  /** Set field position in meters of various objects */
  public enum ScoringTarget{
    k1LeftConeMid(Color.RED,14.73,2,0),
    k1LeftConeHigh(Color.RED, 15.26,2,0),
    k1CubeMid(Color.RED, 14.73,1.5,0),
    k1CubeHigh(Color.RED, 15.26,1.5,0),
    k1RightConeMid(Color.RED, 14.73,0.945,0),
    k1RightConeHigh(Color.RED, 15.26,0.945,0),
    k2LeftConeMid(Color.RED,14.73,3.463,0),
    k2LeftConeHigh(Color.RED, 15.26,3.463,0),
    k2CubeMid(Color.RED, 14.73,3,0),
    k2CubeHigh(Color.RED, 15.26,3,0),
    k2RightConeMid(Color.RED, 14.73,2.463,0),
    k2RightConeHigh(Color.RED, 15.26,2.463,0),
    k3LeftConeMid(Color.RED,14.73,5,0),
    k3LeftConeHigh(Color.RED, 15.26,5,0),
    k3CubeMid(Color.RED, 14.73,4.481,0),
    k3CubeHigh(Color.RED, 15.26,4.481,0),
    k3RightConeMid(Color.RED, 14.73,4,0),
    k3RightConeHigh(Color.RED, 15.26,4,0),
    k4LoadingZone(Color.RED, 15.23,6.573,0),




    k5LeftConeMid(Color.BLUE,1.289,0.926,0),
    k5LeftConeHigh(Color.BLUE, 0.955,0.926,0),
    k5CubeMid(Color.BLUE, 1.289,1.445,0),
    k5CubeHigh(Color.BLUE, 0.955,1.445,0),
    k5RightConeMid(Color.BLUE, 1.289,1.985,0),
    k5RightConeHigh(Color.BLUE, 0.955,1.985,0),
    k6LeftConeMid(Color.BLUE,1.289,0,0),
    k6LeftConeHigh(Color.BLUE, 0.955,0,0),
    k6CubeMid(Color.BLUE, 1.289,0,0),
    k6CubeHigh(Color.BLUE, 0.955,0,0),
    k6RightConeMid(Color.BLUE, 1.289,0,0),
    k6RightConeHigh(Color.BLUE, 0.955,0,0),
    k7LeftConeMid(Color.BLUE,1.289,0,0),
    k7LeftConeHigh(Color.BLUE, 0.955,0,0),
    k7CubeMid(Color.BLUE, 1.289,0,0),
    k7CubeHigh(Color.BLUE, 0.955,0,0),
    k7RightConeMid(Color.BLUE, 1.289,0,0),
    k7RightConeHigh(Color.BLUE, 0.955,0,0),
    k8LoadingZone(Color.BLUE, .77,6.573,0),
    ;

    public Color color;
    public double x;
    public double y;
    public double z;


    ScoringTarget(Color color, double x, double y, double z){
      this.color = color;
      this.x = x; 
      this.y = y; 
      this.z = z;
    }

    

  }

  

  private AHRS gyro;
  public MiniPID pidTurn;

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  
  NetworkTableEntry tx = table.getEntry("tx"); // gets horizontal offset from crosshair
  NetworkTableEntry ty = table.getEntry("ty"); // gets vertical offset from crosshair
  NetworkTableEntry ta = table.getEntry("ta"); // gets target area; how much its taking from the camera screen
  NetworkTableEntry tv = table.getEntry("tv"); //valid target = 1, if target not valid, its equal to 0
  NetworkTableEntry bptable = table.getEntry("botpose"); //gets translation (x, y, z) and rotation (x, y, z) for bot pose
  

  public double camAngle = 45.0; 
  public double camHeight = 7.0; 
  public double targetHeight = 37; 


  public double bpDefault [] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  public Rotation2d rot = new Rotation2d(0,0);
  public Pose2d botPose = new Pose2d(0, 0, new Rotation2d(0));
  private DifferentialDrivePoseEstimator poseEstimator;
  Field2d field;

  /** Creates a new Vision. */
  public Vision(DifferentialDrivePoseEstimator poseEstimator, AHRS gyro, Field2d field) {
    this.poseEstimator = poseEstimator;
    this.gyro = gyro;
    this.field = field;

    //this should be in chassis, but here for simplicity temporarily
    pidTurn = new MiniPID(0,0,0);
    pidTurn.setSetpointRange(15); 
    pidTurn.setP(0.003/.2); // GOAL/ACTUAL or 
    pidTurn.setF((s,a,e)->{return Math.signum(e)*0.05;/*static FeedForward*/ });


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
    double targetArea = ta.getDouble(0.0);
    double[] bp = bptable.getDoubleArray(bpDefault);


    if(Array.getLength(bp)<6) return;
        
    //post to smart dashboard periodically
    SmartDashboard.putBoolean("vision/TargetValid", hasTargets);
    // SmartDashboard.putNumber("vision/X", x);
    // SmartDashboard.putNumber("vision/Y", y);
    SmartDashboard.putNumber("vision/Area", targetArea);
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



  // public double rpm() {
    
  //   return 0.0;
  // }

//  public void armToPole() {

//   //gets the angle to the pole
//   double distanceX = ; //distance from 
//   double distanceY = 0; //unknown rn
//   double angle = Math.arctan(y/x+distance());  
//   }
}

