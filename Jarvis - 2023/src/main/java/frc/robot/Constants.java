// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
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

    //Max radians/sec that the bot will rotate at
    public static final double MAX_ANGULAR_SPEED = 2;
    //acceleration of rotation, rad/sec^2
    public static final double MAX_ANGULAR_ACCELERATION = 16;

    public static final double ROBOT_WHEEL_BASE = Units.inchesToMeters(21.5);

    public static final double FALCON_UNITS_PER_REV = 2048;
    public static final double QUAD_ENCODER_UNITS_PER_REV = 4096;

    
    

    public static final int PRIMARY_PID_LOOP_IDX = 0;

    public static final int TIMEOUT_MS = 30;


    public final class ModuleConstants {
        public static final double DRIVE_GEARING = 7; 
        public static final double WHEEL_DIA = 4; //in inches
        public static final double TURN_GEARING = 2.89 * 2.89 * 6;
        public static final double MAX_SPEED = 3; 

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
                new Translation3d(-Units.inchesToMeters(21/2), 0, -Units.inchesToMeters(6)), 
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
      }
    

}
