// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;


import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.PhotonConstants;

public class Drivetrain extends SubsystemBase {

  
  
  private PhotonCamera photonCamera;
  private double previousPipelineTimestamp = 0;
  
  ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0,0,0);


  // public static boolean aprilTagOn

  // private final ADXRS450_Gyro gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);

  // ADIS16470 plugged into the MXP port
  ADIS16470_IMU gyro = new ADIS16470_IMU();

  //module objects
  private SwerveModule frontLeft = new SwerveModule(1, 1, 5, ModuleConstants.MODULE1_OFFSET, true, 0); // Module 1
  private SwerveModule frontRight = new SwerveModule(2, 2, 6, ModuleConstants.MODULE2_OFFSET, false, 1); // Module 2
  private SwerveModule backLeft = new SwerveModule(3, 3, 7, ModuleConstants.MODULE3_OFFSET, true, 2); // Module 3
  private SwerveModule backRight = new SwerveModule(4, 4, 8, ModuleConstants.MODULE4_OFFSET, false, 3); // Module 4

  //module positions
  public final Translation2d frontLeftLocation = new Translation2d(Constants.ROBOT_WHEEL_BASE/2 , Constants.ROBOT_WHEEL_BASE/2);
  public final Translation2d frontRightLocation = new Translation2d(Constants.ROBOT_WHEEL_BASE/2, -Constants.ROBOT_WHEEL_BASE/2);
  public final Translation2d backLeftLocation = new Translation2d(-Constants.ROBOT_WHEEL_BASE/2,Constants.ROBOT_WHEEL_BASE/2);
  public final Translation2d backRightLocation = new Translation2d(-Constants.ROBOT_WHEEL_BASE/2, -Constants.ROBOT_WHEEL_BASE/2);


  public final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

  //Swerve Drive Pose Estimator, uses the combination of encoder data and vision to estimate pose on the field
  private SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
    kinematics, 
    getChassisAngle(), 
    new SwerveModulePosition[] {frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()},
    new Pose2d(0,0, new Rotation2d()),
    new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.05,0.05,0.05), //Standard deviations for state estimate, (m,m,rad). Increase to trust less
    new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.9,0.9,0.9) //Standard deviations for vision estimate, (m,m,rad). Increase to trust less
    );
     

  public boolean allModuleHomeStatus = false;


  public Drivetrain(PhotonCamera photonCamera) {
    calibrateGyro();

    this.photonCamera = photonCamera;

    setCurrentPose(new Pose2d(new Translation2d(1, 0), Rotation2d.fromDegrees(180)));


  }

  @Override
  public void periodic() {

    
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("1 Homed", frontLeft.homeFinished);
    SmartDashboard.putBoolean("2 Homed", frontRight.homeFinished);
    SmartDashboard.putBoolean("3 Homed", backLeft.homeFinished);
    SmartDashboard.putBoolean("4 Homed", backRight.homeFinished);

    SmartDashboard.putNumber("Bot Heading", getChassisAngle().getDegrees());
    SmartDashboard.putNumber("Bot Pitch", getPitch().getDegrees());

    var pipelineResult = photonCamera.getLatestResult();
    var resultTimestamp = pipelineResult.getTimestampSeconds();

    if (resultTimestamp != previousPipelineTimestamp && pipelineResult.hasTargets()) {
      previousPipelineTimestamp = resultTimestamp;
      var target = pipelineResult.getBestTarget();

      if (target.getPoseAmbiguity() <= .05) {
        Transform3d camToTarget = target.getBestCameraToTarget();
        Transform3d targetToCamera = camToTarget.inverse();

        Pose3d targetPose = getSelectedTargetPose(target.getFiducialId());
        Pose3d camPose = targetPose.transformBy(targetToCamera);

        Pose2d visionMeasurement = camPose.transformBy(PhotonConstants.CAMERA_TO_ROBOT).toPose2d();

        

        poseEstimator.addVisionMeasurement(visionMeasurement, resultTimestamp);
        SmartDashboard.putString("Vision Pose", visionMeasurement.toString());


      }
    }

    
    poseEstimator.update(getChassisAngle(), getModulePositions());

    SmartDashboard.putString("Robot Pose", getRobotPose().toString());


  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  /**
   * Drive the bot using joystick input.
   * Positive x is forward, y - left, rot - CC rotation
   * 
   * 
   * @param xSpeed m/s speed positive away from drive station
   * @param ySpeed m/s speed positive left of drive station
   * @param rot angular rate of bot CCW in radians
   * @param isFieldRelative if bot should be in field relative control
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean isFieldRelative){

    Alliance alliance = DriverStation.getAlliance();

    Rotation2d heading =  getRobotPose().getRotation();


    if(alliance == Alliance.Red){
      heading = getRobotPose().getRotation().plus(Rotation2d.fromDegrees(180));
    } 

    //field relative or not, forward always away from drive station
    if(isFieldRelative){
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, heading);
    } else {
      chassisSpeeds = new ChassisSpeeds(xSpeed,ySpeed,rot);
    }

    
    // Convert to module states
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);

    setModuleStates(moduleStates);

  }

  public void drive(ChassisSpeeds speeds){
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
    setModuleStates(moduleStates);
  }

  /**
   * Set module states for all of the modules. In an array of states in same order as kinimatics
   * @param moduleStates list of states
   */
  public void setModuleStates(SwerveModuleState[] moduleStates){
    //desaturate wheel speeds to under max
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, ModuleConstants.MAX_SPEED);

    //assign states to the modules
    frontLeft.setDesiredState(moduleStates[0]);
    frontRight.setDesiredState(moduleStates[1]);
    backLeft.setDesiredState(moduleStates[2]);
    backRight.setDesiredState(moduleStates[3]);
  }

  public SwerveModuleState[] getModuleStates(){
    SwerveModuleState[] states = {
      frontLeft.getState(),
      frontRight.getState(),
      backLeft.getState(),
      backRight.getState()
    };
    return states;
  }

  public SwerveModulePosition[] getModulePositions(){
    return new SwerveModulePosition[] {
      frontLeft.getPosition(),
      frontRight.getPosition(),
      backLeft.getPosition(),
      backRight.getPosition()
    };
  }

  public Pose2d getRobotPose(){
    return poseEstimator.getEstimatedPosition();
  }

  public void setCurrentPose(Pose2d newPose) {
    poseEstimator.resetPosition(getChassisAngle(), getModulePositions(), newPose);
  }


  /**
   * Homes all the swerve modules. 
   * Call resetHomeStatus() before to initialize the process
   */
  public void homeAllModules(){
    frontLeft.home();
    frontRight.home();
    backLeft.home();
    backRight.home();

    //if all modules are done then change variable state to true
    allModuleHomeStatus = frontLeft.homeFinished && frontRight.homeFinished && backLeft.homeFinished && backRight.homeFinished;

  }

  /**
   * Resets all the modules' home status to false
   */
  public void resetHomeStatus(){
    frontLeft.setHomeStatus(false);
    frontRight.setHomeStatus(false);
    backLeft.setHomeStatus(false);
    backRight.setHomeStatus(false);

    allModuleHomeStatus = false;
  }

  /**
   * @return current chassis angle
   */
  public Rotation2d getChassisAngle(){
    // return gyro.getRotation2d();
    return Rotation2d.fromDegrees(gyro.getAngle());
  }

  /**
   * Stops all modules
   */
  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  /**
   * Resets encoders for all modules
   */
  public void zeroEncoders() {
    frontLeft.zeroEncoders();
    frontRight.zeroEncoders();
    backLeft.zeroEncoders();
    backRight.zeroEncoders();
  }

  /**
   * recalibrates the gyroscope. This takes a significant amount of time so make sure bot is stationary
   */
  public void calibrateGyro(){
    gyro.calibrate();
  }

  /**
   * test code for front left module
   * @param angle target angle radians
   */
  public void setSteerAngle(double angle){
    frontLeft.setSteerAngle(angle);
  }



  public Pose3d getSelectedTargetPose(int ID){
    Pose3d targetPose;

    switch (ID) {
      case 1:
        targetPose = PhotonConstants.TARGET_1_POSE;
        break;
      case 2:
        targetPose = PhotonConstants.TARGET_2_POSE;
        break;
      case 3:
        targetPose = PhotonConstants.TARGET_3_POSE;
        break;
      case 4:
        targetPose = PhotonConstants.TARGET_4_POSE;
        break;
      case 5:
        targetPose = PhotonConstants.TARGET_5_POSE;
        break;
      case 6:
        targetPose = PhotonConstants.TARGET_6_POSE;
        break;
      case 7:
        targetPose = PhotonConstants.TARGET_7_POSE;
        break;
      case 8:
        targetPose = PhotonConstants.TARGET_8_POSE;
        break;
    
      default:
        targetPose = new Pose3d();
        break;
    }
    
    return targetPose;
  }

  public Rotation2d getPitch(){
    return Rotation2d.fromDegrees(gyro.getYComplementaryAngle());
  }

  public ChassisSpeeds getRobotRelativeSpeeds(){
    return kinematics.toChassisSpeeds(getModuleStates());
  }


}
