// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

/** 
 * Robot Field Space
 *   +-----------+-----------+
 *  |5R         |         4B|
 *  |           |           |
 *  |B 6        x        3 R|
 *  |B 7        |        2 R|
 *  |B 8        |        1 R|
 *  +-----------+----------+
 *  LL Origin at X
 *  Field CAD origin at x
 *  Field Pose/Glass origin at bottom-left
 * 
 */
public class FieldPosition {
    public final static double kOriginOffsetX=15.98/2.0;
    public final static double kOriginOffsetY=8.120/2.0;

    public final static double[] scoringCols = {.936, 1.37 , 14.64 , 15.05};
    public final static double[] scoringRows = {
        //staring from bottom (origin) and increasing
        //1 and 8
        .959, 
        1.47,
        1.951,
        //2 and 7
        2.46,
        2.969,
        3.477,
        //3 and 6
        3.961,
        4.495,
        4.953
        };
    public final static double[] pickupRows = {
        5.996,
        7.191,
        7.726
    };
    public final static double[] pickupCols={
        .918,  2.69,  13.24, 15.07
    };

    public final static double kConeHeightMid = .5;
    public final static double kConeHeightHigh = 1;
    public final static double kCubeHeightMid = kConeHeightMid;
    public final static double kCubeHeightHigh = kConeHeightHigh;
    public final static double kPickupHeightSlide = 0.6;
    public final static double kPickupHeightDouble = 0.7;


    public final static ArrayList<Pose3d> RedConesHigh= new ArrayList<Pose3d>(){{
        add( new Pose3d(scoringCols[3], scoringRows[0], kConeHeightHigh, new Rotation3d()) );
        //cube
        add( new Pose3d(scoringCols[3], scoringRows[2], kConeHeightHigh, new Rotation3d()) );
        add( new Pose3d(scoringCols[3], scoringRows[3], kConeHeightHigh, new Rotation3d()) );
        //cube
        add( new Pose3d(scoringCols[3], scoringRows[5], kConeHeightHigh, new Rotation3d()) );
        add( new Pose3d(scoringCols[3], scoringRows[6], kConeHeightHigh, new Rotation3d()) );
        //cube
        add( new Pose3d(scoringCols[3], scoringRows[8], kConeHeightHigh, new Rotation3d()) );
    }};

    public final static ArrayList<Pose3d> RedCubesHigh= new ArrayList<Pose3d>(){{
        add( new Pose3d(scoringCols[3], scoringRows[1], kCubeHeightHigh, new Rotation3d()) );
        add( new Pose3d(scoringCols[3], scoringRows[4], kCubeHeightHigh, new Rotation3d()) );
        add( new Pose3d(scoringCols[3], scoringRows[7], kCubeHeightHigh, new Rotation3d()) );
    }};


    public final static ArrayList<Pose3d> RedConesMid= new ArrayList<Pose3d>(){{
        add( new Pose3d(scoringCols[2], scoringRows[0], kConeHeightMid, new Rotation3d()) );
        //cube
        add( new Pose3d(scoringCols[2], scoringRows[2], kConeHeightMid, new Rotation3d()) );
        add( new Pose3d(scoringCols[2], scoringRows[3], kConeHeightMid, new Rotation3d()) );
        //cube
        add( new Pose3d(scoringCols[2], scoringRows[5], kConeHeightMid, new Rotation3d()) );
        add( new Pose3d(scoringCols[2], scoringRows[6], kConeHeightMid, new Rotation3d()) );
        //cube
        add( new Pose3d(scoringCols[2], scoringRows[8], kConeHeightMid, new Rotation3d()) );
    }};

    public final static ArrayList<Pose3d> RedCubesMid= new ArrayList<Pose3d>(){{
        add( new Pose3d(scoringCols[2], scoringRows[1], kCubeHeightMid, new Rotation3d()) );
        add( new Pose3d(scoringCols[2], scoringRows[4], kCubeHeightMid, new Rotation3d()) );
        add( new Pose3d(scoringCols[2], scoringRows[7], kCubeHeightMid, new Rotation3d()) );
    }};

    public final static ArrayList<Pose3d> RedPickupSlide= new ArrayList<Pose3d>(){{
        add( new Pose3d(pickupCols[1], pickupRows[2], kPickupHeightSlide, new Rotation3d()) );
    }};

    public final static ArrayList<Pose3d> RedPickupDouble= new ArrayList<Pose3d>(){{
        add( new Pose3d(Units.inchesToMeters(14.25-12), Units.inchesToMeters(265.74-30), Units.inchesToMeters(27.38), new Rotation3d(0,0,0)) );
        add( new Pose3d(Units.inchesToMeters(14.25-12), Units.inchesToMeters(265.74+30), Units.inchesToMeters(27.38), new Rotation3d(0,0,0)) );
        // add( new Pose3d(pickupCols[0], pickupRows[0], kPickupHeightDouble, new Rotation3d()) );
        // add( new Pose3d(pickupCols[0], pickupRows[1], kPickupHeightDouble, new Rotation3d()) );
    }};

    //******************************
    //* Blue! 
    //******************************

    public final static ArrayList<Pose3d> BlueConesHigh= new ArrayList<Pose3d>(){{
        add( new Pose3d(scoringCols[0], scoringRows[0], kConeHeightHigh, new Rotation3d()) );
        //cube
        add( new Pose3d(scoringCols[0], scoringRows[2], kConeHeightHigh, new Rotation3d()) );
        add( new Pose3d(scoringCols[0], scoringRows[3], kConeHeightHigh, new Rotation3d()) );
        //cube
        add( new Pose3d(scoringCols[0], scoringRows[5], kConeHeightHigh, new Rotation3d()) );
        add( new Pose3d(scoringCols[0], scoringRows[6], kConeHeightHigh, new Rotation3d()) );
        //cube
        add( new Pose3d(scoringCols[0], scoringRows[8], kConeHeightHigh, new Rotation3d()) );
    }};

    public final static ArrayList<Pose3d> BlueCubesHigh = new ArrayList<Pose3d>(){{
        add( new Pose3d(scoringCols[0], scoringRows[1], kCubeHeightHigh, new Rotation3d()) );
        add( new Pose3d(scoringCols[0], scoringRows[4], kCubeHeightHigh, new Rotation3d()) );
        add( new Pose3d(scoringCols[0], scoringRows[7], kCubeHeightHigh, new Rotation3d()) );
    }};


    public final static ArrayList<Pose3d> BlueConesMid = new ArrayList<Pose3d>(){{
        add( new Pose3d(scoringCols[1], scoringRows[0], kConeHeightMid, new Rotation3d()) );
        //cube
        add( new Pose3d(scoringCols[1], scoringRows[2], kConeHeightMid, new Rotation3d()) );
        add( new Pose3d(scoringCols[1], scoringRows[3], kConeHeightMid, new Rotation3d()) );
        //cube
        add( new Pose3d(scoringCols[1], scoringRows[5], kConeHeightMid, new Rotation3d()) );
        add( new Pose3d(scoringCols[1], scoringRows[6], kConeHeightMid, new Rotation3d()) );
        //cube
        add( new Pose3d(scoringCols[1], scoringRows[8], kConeHeightMid, new Rotation3d()) );
    }};

    public final static ArrayList<Pose3d> BlueCubesMid = new ArrayList<Pose3d>(){{

        add( new Pose3d(scoringCols[1], scoringRows[1], kCubeHeightMid, new Rotation3d()) );
        add( new Pose3d(scoringCols[1], scoringRows[4], kCubeHeightMid, new Rotation3d()) );
        add( new Pose3d(scoringCols[1], scoringRows[7], kCubeHeightMid, new Rotation3d()) );
    }};

    public final static ArrayList<Pose3d> BluePickupSlide = new ArrayList<Pose3d>(){{
        add( new Pose3d(pickupCols[2], pickupRows[2], kPickupHeightSlide, new Rotation3d()) );
    }};

    public final static ArrayList<Pose3d> BluePickupDouble = new ArrayList<Pose3d>(){{
        add( new Pose3d(Units.inchesToMeters(636.77+12), Units.inchesToMeters(265.74-30), Units.inchesToMeters(27.38), new Rotation3d(0,0,Math.PI)) );
        add( new Pose3d(Units.inchesToMeters(636.77+12), Units.inchesToMeters(265.74+30), Units.inchesToMeters(27.38), new Rotation3d(0,0,Math.PI)) );
        // add( new Pose3d(pickupCols[3], pickupRows[0], kPickupHeightDouble, new Rotation3d()) );
        // add( new Pose3d(pickupCols[3], pickupRows[1], kPickupHeightDouble, new Rotation3d()) );
    }};

    /**
     * From https://firstfrc.blob.core.windows.net/frc2023/FieldAssets/2023LayoutMarkingDiagram.pdf page 4
     * Distances are to center of field tag
     * */
    public final static ArrayList<Pose3d> AprilTags = new ArrayList<Pose3d>(){{
        add( new Pose3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation3d()) );
        //1,2,3 : Red alliance grid
        add( new Pose3d(Units.inchesToMeters(610.77), Units.inchesToMeters(42.19), Units.inchesToMeters(18.22), new Rotation3d(0,0,Math.PI)) );
        add( new Pose3d(Units.inchesToMeters(610.77), Units.inchesToMeters(108.19), Units.inchesToMeters(18.22), new Rotation3d(0,0,Math.PI)) );
        add( new Pose3d(Units.inchesToMeters(610.77), Units.inchesToMeters(174.19), Units.inchesToMeters(18.22), new Rotation3d(0,0,Math.PI)) );
        //4 Blue alliance loading station
        add( new Pose3d(Units.inchesToMeters(636.77), Units.inchesToMeters(265.74), Units.inchesToMeters(27.38), new Rotation3d(0,0,Math.PI)) );
        //5 Red Loading station
        add( new Pose3d(Units.inchesToMeters(14.25), Units.inchesToMeters(265.74), Units.inchesToMeters(27.38), new Rotation3d(0,0,0)) );
        //6,7,8 blue alliance grid
        add( new Pose3d(Units.inchesToMeters(40.45), Units.inchesToMeters(174.19), Units.inchesToMeters(18.22), new Rotation3d(0,0,0)) );
        add( new Pose3d(Units.inchesToMeters(40.45), Units.inchesToMeters(108.19), Units.inchesToMeters(18.22), new Rotation3d(0,0,0)) );
        add( new Pose3d(Units.inchesToMeters(40.45), Units.inchesToMeters(42.19), Units.inchesToMeters(18.22), new Rotation3d(0,0,0)) );
    }};

    ///////////////////////////
    //Utility Functions
    ///////////////////////////
    public static enum TargetType{CubeMid,CubeHigh,ConeMid,ConeHigh,PickupSlide,PickupDouble}

    public static List<Pose3d> GetTargetList(TargetType targetType){
      Alliance color = DriverStation.getAlliance();
      switch(targetType){
      case CubeMid:   return color==Alliance.Red ? FieldPosition.RedConesMid : FieldPosition.BlueConesMid;
      case CubeHigh:  return color==Alliance.Red ? FieldPosition.RedCubesHigh : FieldPosition.BlueCubesHigh;
      case ConeMid:   return color==Alliance.Red ? FieldPosition.RedConesMid : FieldPosition.BlueConesMid;
      case ConeHigh:  return color==Alliance.Red ? FieldPosition.RedConesHigh : FieldPosition.BlueConesHigh;
      case PickupSlide:  return color==Alliance.Red ? FieldPosition.RedPickupSlide : FieldPosition.BluePickupSlide;
      case PickupDouble: return color==Alliance.Red ? FieldPosition.RedPickupDouble : FieldPosition.BluePickupDouble;
      }
      return new ArrayList<>();
    }

    public static Pose3d GetNearestToBearing(Pose2d botPose, List<Pose3d> poses){
        var filteredList = poses.stream()
        .sorted((a,b)-> Double.compare(
            Math.abs(botPose.getY() - a.getY()),
            Math.abs(botPose.getY() - b.getY())
            )
        )
        .limit(2)
        .min((a,b)-> Double.compare(
            Math.abs(Math.atan2(a.getY(), a.getX())-botPose.getRotation().getDegrees()),
            Math.abs(Math.atan2(b.getY(), b.getX())-botPose.getRotation().getDegrees())
        ))
        ;
        return filteredList.get();
    }

    public static Pose3d GetNearestByX(Pose2d botPose, List<Pose3d> poses){
        var filteredList = poses.stream()
        .min((a,b)-> Double.compare(
            Math.abs(botPose.getX()- a.getX()),
            Math.abs(botPose.getX()- b.getX())
            )
        )
        ;
        return filteredList.get();
    }

    public static Pose3d GetNearestByY(Pose2d botPose, List<Pose3d> poses){
        var filteredList = poses.stream()
        .min((a,b)-> Double.compare(
            Math.abs(botPose.getY()-a.getY()),
            Math.abs(botPose.getY()-b.getY())
            )
        )
        ;
        return filteredList.get();
    }
    
    public static double GetArmAngletoPoint(Pose2d botPose, Pose3d targetObject){
        var armDistance=GetArmDistancetoPoint(botPose,targetObject);
        double armAngle = Math.acos(Math.sqrt(Math.pow(targetObject.getX(), 2.0) + Math.pow(targetObject.getY(), 2.0))/ armDistance);
        return armAngle;
    }

    public static double GetArmDistancetoPoint(Pose2d botPose, Pose3d targetObject){
        double armDistance = Math.sqrt(Math.pow(targetObject.getX(), 2.0) + Math.pow(targetObject.getY(), 2.0) + Math.pow(targetObject.getZ(), 2.0));
        return armDistance;
    }

    public static double GetChassisRotationToPoint(Pose2d botPose, Pose3d targetObject){
        //TODO
        return 0;
    }
    

    public static List<Pose2d> GetPose2dList(ArrayList<Pose3d> poses){
        return poses.stream().map(Pose3d::toPose2d).collect(Collectors.toList());
    }

    public static void ShowOnGlassDashboard(Field2d field){
        field.getObject("Red Cones High").setPoses(GetPose2dList(RedConesHigh));
        field.getObject("Red Cones Mid").setPoses(GetPose2dList(RedConesMid));

        field.getObject("Red Cubes High").setPoses(GetPose2dList(RedCubesHigh));
        field.getObject("Red Cubes Mid").setPoses(GetPose2dList(RedCubesMid));

        field.getObject("Red Pickup Double").setPoses(GetPose2dList(RedPickupDouble));
        field.getObject("Red Pickup Slide").setPoses(GetPose2dList(RedPickupSlide));


        field.getObject("Blue Cones High").setPoses(GetPose2dList(BlueConesHigh));
        field.getObject("Blue Cones Mid").setPoses(GetPose2dList(BlueConesMid));
        
        field.getObject("Blue Cubes High").setPoses(GetPose2dList(BlueCubesHigh));
        field.getObject("Blue Cubes Mid").setPoses(GetPose2dList(BlueCubesMid));

        field.getObject("Blue Pickup Double").setPoses(GetPose2dList(BluePickupDouble));
        field.getObject("Blue Pickup Slide").setPoses(GetPose2dList(BluePickupSlide));
    }
}