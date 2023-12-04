// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ScoringPositions;
import frc.robot.Util.Gains;
import frc.robot.subsystems.Drivetrain;

public class MoveToFieldPosition extends CommandBase {
  /** Creates a new MoveToFieldPosition. */
  Drivetrain drivetrain;
  Pose2d targetFieldPosition;
  // private static final ShuffleboardTab PIDTab = Shuffleboard.getTab("PID Tuning");
  // private static GenericEntry xyP;
  // private static GenericEntry tP;

  Gains translationGains = new Gains(7, 0, 0, 0, 0, 1);
  Gains rotationGains = new Gains(7, 0, 0.1, 0, 0, 0);

  ProfiledPIDController xController = new ProfiledPIDController(translationGains.P, 0, 0, new TrapezoidProfile.Constraints(3,1.5));
  ProfiledPIDController yController = new ProfiledPIDController(translationGains.P, 0, 0, new TrapezoidProfile.Constraints(3,1.5));
  ProfiledPIDController thetaController = new ProfiledPIDController(rotationGains.P, 0, 0, new TrapezoidProfile.Constraints(4,3));


  public MoveToFieldPosition(Pose2d targetFieldPosition, Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    this.targetFieldPosition = targetFieldPosition;
    addRequirements(drivetrain);

  }

  public MoveToFieldPosition(int buttonID, Drivetrain drivetrain){
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);

    if(DriverStation.getAlliance() == Alliance.Blue){
      switch (buttonID) {
        case 1:
          targetFieldPosition = ScoringPositions.BLUE1;
          break;

        case 2:
          targetFieldPosition = ScoringPositions.BLUE2;
          break;

        case 3:
          targetFieldPosition = ScoringPositions.BLUE3;
          break;

        case 4:
          targetFieldPosition = ScoringPositions.BLUE4;
          break;

        case 5:
          targetFieldPosition = ScoringPositions.BLUE5;
          break;

        case 6:
          targetFieldPosition = ScoringPositions.BLUE6;
          break;

        case 7:
          targetFieldPosition = ScoringPositions.BLUE7;
          break;

        case 8:
          targetFieldPosition = ScoringPositions.BLUE8;
          break;

        case 9:
          targetFieldPosition = ScoringPositions.BLUE9;
          break;

        case 10:
          targetFieldPosition = ScoringPositions.BLUE_PICKUP_LEFT;
          break;

        case 11:
          targetFieldPosition = ScoringPositions.BLUE_PICKUP_RIGHT;
          break;
      
        default:
          targetFieldPosition = drivetrain.getRobotPose();
          break;
      }
    }else{
      switch (buttonID) {
        case 1:
          targetFieldPosition = ScoringPositions.Red1;
          break;

        case 2:
          targetFieldPosition = ScoringPositions.Red2;
          break;

        case 3:
          targetFieldPosition = ScoringPositions.Red3;
          break;

        case 4:
          targetFieldPosition = ScoringPositions.Red4;
          break;

        case 5:
          targetFieldPosition = ScoringPositions.Red5;
          break;

        case 6:
          targetFieldPosition = ScoringPositions.Red6;
          break;

        case 7:
          targetFieldPosition = ScoringPositions.Red7;
          break;

        case 8:
          targetFieldPosition = ScoringPositions.Red8;
          break;

        case 9:
          targetFieldPosition = ScoringPositions.Red9;
          break;

        case 10:
          targetFieldPosition = ScoringPositions.RED_PICKUP_LEFT;
          break;

        case 11:
          targetFieldPosition = ScoringPositions.RED_PICKUP_RIGHT;
          break;
      
        default:
          targetFieldPosition = drivetrain.getRobotPose();
          break;
      }
    }
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    xController.reset(drivetrain.getRobotPose().getX());
    yController.reset(drivetrain.getRobotPose().getY());
    thetaController.reset(drivetrain.getRobotPose().getRotation().getRadians());

    xController.setTolerance(0.05);
    yController.setTolerance(0.05);
    thetaController.setTolerance(0.05);
    checkShuffleboard();

    SmartDashboard.putNumber("Translation P", translationGains.P);
    SmartDashboard.putNumber("Rotation P", rotationGains.P);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = -xController.calculate(drivetrain.getRobotPose().getX(),targetFieldPosition.getX());
    double ySpeed = -yController.calculate(drivetrain.getRobotPose().getY(),targetFieldPosition.getY());
    double thetaSpeed = thetaController.calculate(drivetrain.getRobotPose().getRotation().getRadians(), targetFieldPosition.getRotation().getRadians());

    drivetrain.drive(xSpeed, ySpeed, thetaSpeed, true);

    checkShuffleboard();

    SmartDashboard.putNumber("target Rotation", thetaSpeed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return xController.atGoal() && yController.atGoal() && thetaController.atGoal();
    return false;
  }


  private void checkShuffleboard(){

    if(SmartDashboard.getNumber("Translation P", translationGains.P) != xController.getP()){
      xController.setP(SmartDashboard.getNumber("Translation P", translationGains.P));
      yController.setP(SmartDashboard.getNumber("Translation P", translationGains.P));
    }

    if(SmartDashboard.getNumber("Rotation P", translationGains.P) != thetaController.getP()){
      thetaController.setP(SmartDashboard.getNumber("Rotation P", translationGains.P));
    }




  }
}
