// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Gains;
import frc.robot.subsystems.Drivetrain;

public class MoveToFieldPosition extends CommandBase {
  /** Creates a new MoveToFieldPosition. */
  Drivetrain drivetrain;
  Pose2d targetFieldPosition;
  // private static final ShuffleboardTab PIDTab = Shuffleboard.getTab("PID Tuning");
  // private static GenericEntry xyP;
  // private static GenericEntry tP;

  Gains translationGains = new Gains(3, 0, 0, 0, 0, 1);
  Gains rotationGains = new Gains(5, 0, 0.1, 0, 0, 0);

  ProfiledPIDController xController = new ProfiledPIDController(translationGains.P, 0, 0, new TrapezoidProfile.Constraints(3,3));
  ProfiledPIDController yController = new ProfiledPIDController(translationGains.P, 0, 0, new TrapezoidProfile.Constraints(3,3));
  ProfiledPIDController thetaController = new ProfiledPIDController(rotationGains.P, 0, 0, new TrapezoidProfile.Constraints(4,3));


  public MoveToFieldPosition(Pose2d targetFieldPosition, Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    this.targetFieldPosition = targetFieldPosition;
    addRequirements(drivetrain);

    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SmartDashboard.putNumber("Translation P", translationGains.P);
    SmartDashboard.putNumber("Rotation P", rotationGains.P);


    // xyP = PIDTab.add("Translation P",translationGains.P).getEntry();
    // tP = PIDTab.add("Rotation P", rotationGains.P).getEntry();

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xController.reset(drivetrain.getRobotPose().getX());
    yController.reset(drivetrain.getRobotPose().getY());
    thetaController.reset(drivetrain.getRobotPose().getRotation().getRadians());

    xController.setTolerance(0.1);
    yController.setTolerance(0.1);
    thetaController.setTolerance(0.05);
    checkShuffleboard();



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
    return xController.atGoal() && yController.atGoal() && thetaController.atGoal();
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
