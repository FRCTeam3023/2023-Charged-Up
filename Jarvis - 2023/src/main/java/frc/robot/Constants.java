// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final boolean testCode = true;

    public static final double DRIVE_TOLERANCE_PERCENT = 0.15;
    public static final double ROTATION_TOLERANCE_PERCENT = 0.15;

    public static final double JOG_SPEED = 0.4;

    //Max radians/sec that the bot will rotate at
    public static final double MAX_ANGULAR_SPEED = 2;
    public static final double SLOW_MAX_ANGULAR_SPEED = 3;
    //acceleration of rotation, rad/sec^2
    public static final double MAX_ANGULAR_ACCELERATION = 16;

    public static final double ROBOT_WHEEL_BASE = Units.inchesToMeters(21.5);

    public static final double FALCON_UNITS_PER_REV = 2048;
    public static final double QUAD_ENCODER_UNITS_PER_REV = 4096;
    public static final double CANCODER_UNITS_PER_REV = 4096;

    
    

    public static final int PRIMARY_PID_LOOP_IDX = 0;

    public static final int TIMEOUT_MS = 30;



    public final class ModuleConstants {
        public static final double DRIVE_GEARING = 8; 
        public static final double WHEEL_DIA = 3.875; //in inches
        public static final double TURN_GEARING = 2.89 * 2.89 * 6;
        public static final double MAX_SPEED = 4;
        public static final double SLOW_MAX_SPEED = 1.5; 

        //module offsets for the zero position for each wheel, in this order
        //Front Left - Front Right - Back Left - Back Right
        public static final double MODULE1_OFFSET = -Math.PI/4;
        public static final double MODULE2_OFFSET = Math.PI/4;
        public static final double MODULE3_OFFSET = Math.PI/4;
        public static final double MODULE4_OFFSET = -Math.PI/4;



    }


    public final class AutoConstants {
        public static final double MAX_ACCELERATION = 1;
        public static final double MAX_SPEED = 2;

        public static final double X_CONTROLLER = 2;
        public static final double Y_CONTROLLER = 2;
        public static final double THETA_CONTROLLER = 2.5;
        public final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS = 
            new TrapezoidProfile.Constraints(
                MAX_ANGULAR_SPEED, 
                MAX_ANGULAR_ACCELERATION);
    }

    public static class PhotonConstants {
        public final static double CAM_PITCH = 15; //degrees

        public final static Transform3d CAMERA_TO_ROBOT = 
            new Transform3d(
                new Translation3d(0, Units.inchesToMeters(5.5), -Units.inchesToMeters(11.75)), 
                new Rotation3d(Units.degreesToRadians(CAM_PITCH),0,0)
            );

        public static final Transform3d ROBOT_TO_CAMERA = CAMERA_TO_ROBOT.inverse();

        
        public static final Pose3d TARGET_1_POSE = new Pose3d(15.54 , 1.06, Units.inchesToMeters(18.13), new Rotation3d(0,0,Math.PI));
        public static final Pose3d TARGET_2_POSE = new Pose3d(15.54 , 2.75, Units.inchesToMeters(18.13), new Rotation3d(0,0,Math.PI));
        public static final Pose3d TARGET_3_POSE = new Pose3d(15.54 , 4.42, Units.inchesToMeters(18.13), new Rotation3d(0,0,Math.PI));

        public static final Pose3d TARGET_4_POSE = new Pose3d(16.21 , 6.72, Units.inchesToMeters(27.38), new Rotation3d(0,0,Math.PI));

        

        public static final Pose3d TARGET_5_POSE = new Pose3d(0.33 , 6.75, Units.inchesToMeters(27.38), new Rotation3d());

        public static final Pose3d TARGET_6_POSE = new Pose3d(1 , 4.42, Units.inchesToMeters(18.13), new Rotation3d());
        public static final Pose3d TARGET_7_POSE = new Pose3d(1,2.75, Units.inchesToMeters(18.13), new Rotation3d());
        public static final Pose3d TARGET_8_POSE = new Pose3d(1 , 1.06, Units.inchesToMeters(18.13), new Rotation3d());
    }   

    public static class ArmConstants {
        public static final double BASE_GEAR_RATIO = 30 * (72/20) * (50/20);
        public static final double ELBOW_GEAR_RATIO = 100*2; 
        public static final double WRIST_GEAR_RATIO = 100;
        public static final double WRIST_PULLEY_RATIO = 1;
        public static final double CLAW_GEAR_RATIO = 100;
        public static final double CLAW_PULLEY_RATIO = 1;

        public static final double CLAW_CLOSING_OUTPUT = 0.3;
        public static final double CLAW_CLOSE_THRESHOLD = 20;
        public static final double CLAW_CLOSE_LIMIT = 300;
        public static final double CUBE_CLAW_OFFSET = 100;
        public static final double CONE_CLAW_OFFSET = 250;


        public static final ArmState HOME_STATE = new ArmState(Rotation2d.fromDegrees(-22), new Rotation2d(), Rotation2d.fromDegrees(80), true);
        // public static final ArmState NEUTRAL_STATE = new ArmState(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(30), Rotation2d.fromDegrees(75), true);
        public static final ArmState PRE_MID_SCORE_STATE = new ArmState(Rotation2d.fromDegrees(-20), Rotation2d.fromDegrees(93), Rotation2d.fromDegrees(80), true);
        public static final ArmState MID_SCORE_STATE = new ArmState(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(93), Rotation2d.fromDegrees(20), true);
        public static final ArmState HIGH_SCORE_STATE = new ArmState(Rotation2d.fromDegrees(20),Rotation2d.fromDegrees(125),Rotation2d.fromDegrees(0),true);
        public static final ArmState PICKUP_STATE = new ArmState(Rotation2d.fromDegrees(-20),Rotation2d.fromDegrees(85),Rotation2d.fromDegrees( 13), false);
        public static final ArmState CLEARANCE_STATE = new ArmState(Rotation2d.fromDegrees(-22), Rotation2d.fromDegrees(50), Rotation2d.fromDegrees(80), true);
        
        public static final double BASE_HOLDING_POWER = -0.06;


        public static final double PULLEY_DIA = 0.5; //in
        public static final double PULLEY_CIRCUMFERENCE = PULLEY_DIA * Math.PI;

        public static final double WRIST_PULLEY_DIA = 0.625; //in
        public static final double WRIST_PULLEY_CIRCUMFERENCE = WRIST_PULLEY_DIA * Math.PI;

        public static final double CLAW_CABLE_LENGTH_OPEN_TO_CLOSE = 1.375;

        public static final double BASE_JOINT_OFFSET = 20;

        public static final double WRIST_MAGNET_OFFSET = 207;
        public static final double ELBOW_MAGNET_OFFSET = 180;

        public static final double HUMOROUS_LENGTH = 37;
        public static final double FOREARM_LENGTH = 37;
        public static final double CLAW_LENGTH = 6;


      }



      public static class ScoringPositions {
        public static final double SCORING_DEPTH_BLUE = 1.8;
        public static final double SCORING_DEPTH_RED = 14.74;

        public static final Pose2d BLUE1 = new Pose2d(SCORING_DEPTH_BLUE, 4.97, new Rotation2d(Math.PI));
        public static final Pose2d BLUE2 = new Pose2d(SCORING_DEPTH_BLUE, 4.42, new Rotation2d(Math.PI));
        public static final Pose2d BLUE3 = new Pose2d(SCORING_DEPTH_BLUE, 3.86, new Rotation2d(Math.PI));
        public static final Pose2d BLUE4 = new Pose2d(SCORING_DEPTH_BLUE, 3.31, new Rotation2d(Math.PI));
        public static final Pose2d BLUE5 = new Pose2d(SCORING_DEPTH_BLUE, 2.75, new Rotation2d(Math.PI));
        public static final Pose2d BLUE6 = new Pose2d(SCORING_DEPTH_BLUE, 2.19, new Rotation2d(Math.PI));
        public static final Pose2d BLUE7 = new Pose2d(SCORING_DEPTH_BLUE, 1.62, new Rotation2d(Math.PI));
        public static final Pose2d BLUE8 = new Pose2d(SCORING_DEPTH_BLUE, 1.07, new Rotation2d(Math.PI));
        public static final Pose2d BLUE9 = new Pose2d(SCORING_DEPTH_BLUE, 0.51, new Rotation2d(Math.PI));

        public static final Pose2d Red1 = new Pose2d(SCORING_DEPTH_RED, 0.51, new Rotation2d());
        public static final Pose2d Red2 = new Pose2d(SCORING_DEPTH_RED, 1.07, new Rotation2d());
        public static final Pose2d Red3 = new Pose2d(SCORING_DEPTH_RED, 1.62, new Rotation2d());
        public static final Pose2d Red4 = new Pose2d(SCORING_DEPTH_RED, 2.19, new Rotation2d());
        public static final Pose2d Red5 = new Pose2d(SCORING_DEPTH_RED, 2.75, new Rotation2d());
        public static final Pose2d Red6 = new Pose2d(SCORING_DEPTH_RED, 3.31, new Rotation2d());
        public static final Pose2d Red7 = new Pose2d(SCORING_DEPTH_RED, 3.86, new Rotation2d());
        public static final Pose2d Red8 = new Pose2d(SCORING_DEPTH_RED, 4.42, new Rotation2d());
        public static final Pose2d Red9 = new Pose2d(SCORING_DEPTH_RED, 4.97, new Rotation2d());



        public static final double PICKUP_DEPTH_RED = 1;
        public static final double PICKUP_DEPTH_BLUE = 15.6;


        public static final Pose2d RED_PICKUP_RIGHT = new Pose2d(PICKUP_DEPTH_RED, 7.45, new Rotation2d(Math.PI));
        public static final Pose2d RED_PICKUP_LEFT = new Pose2d(PICKUP_DEPTH_RED, 6.14, new Rotation2d(Math.PI));


        public static final Pose2d BLUE_PICKUP_RIGHT = new Pose2d(PICKUP_DEPTH_BLUE, 6.14, new Rotation2d(Math.PI));
        public static final Pose2d BLUE_PICKUP_LEFT = new Pose2d(PICKUP_DEPTH_BLUE, 7.4, new Rotation2d(Math.PI));

        


        public static final int SCORING_BUTTON_1 = 8;
        public static final int SCORING_BUTTON_2 = 9;
        public static final int SCORING_BUTTON_3 = 10;
        public static final int SCORING_BUTTON_4 = 11;
        public static final int SCORING_BUTTON_5 = 5;
        public static final int SCORING_BUTTON_6 = 4;
        public static final int SCORING_BUTTON_7 = 3;
        public static final int SCORING_BUTTON_8 = 2;
        public static final int SCORING_BUTTON_9 = 1;



      }
    

}
