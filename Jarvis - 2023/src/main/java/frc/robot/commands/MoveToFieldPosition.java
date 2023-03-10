// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class MoveToFieldPosition extends CommandBase {
  /** Creates a new MoveToFieldPosition. */
  Drivetrain drivetrain;
  Pose2d targetFieldPosition;
  ShuffleboardTab PIDTab;

  ProfiledPIDController xController = new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(3,2));
  ProfiledPIDController yController = new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(3,2));
  ProfiledPIDController thetaController = new ProfiledPIDController(0.1, 0, 0.01, new TrapezoidProfile.Constraints(4,4));

  public MoveToFieldPosition(Pose2d targetFieldPosition, Drivetrain drivetrain, ShuffleboardTab PIDTab) {
    this.drivetrain = drivetrain;
    this.targetFieldPosition = targetFieldPosition;
    addRequirements(drivetrain);

    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    PIDTab.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xController.reset(drivetrain.getRobotPose().getX());
    yController.reset(drivetrain.getRobotPose().getY());
    thetaController.reset(drivetrain.getRobotPose().getRotation().getRadians());

    xController.setTolerance(0.1);
    yController.setTolerance(0.1);
    thetaController.setTolerance(0.2);


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = -xController.calculate(drivetrain.getRobotPose().getX(),targetFieldPosition.getX());
    double ySpeed = -yController.calculate(drivetrain.getRobotPose().getY(),targetFieldPosition.getY());
    double thetaSpeed = -thetaController.calculate(drivetrain.getRobotPose().getRotation().getRadians(), targetFieldPosition.getRotation().getRadians());

    drivetrain.drive(xSpeed, ySpeed, thetaSpeed, true);

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
}
