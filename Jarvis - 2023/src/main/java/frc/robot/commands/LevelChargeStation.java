// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class LevelChargeStation extends CommandBase {
  /** Creates a new LevelChargeStation. */
  Drivetrain drivetrain;

  ProfiledPIDController controller = new ProfiledPIDController(0.022, 0, 0, new TrapezoidProfile.Constraints(1,1));
  public LevelChargeStation(Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    controller.setTolerance(2);
    this.drivetrain = drivetrain;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller.setGoal(0);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = controller.calculate(drivetrain.getPitch().getDegrees());

    if(!controller.atGoal()){
      drivetrain.drive(speed, 0, 0, false);
    }else{
      drivetrain.stopModules();
    }

    SmartDashboard.putNumber("PID output", speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;
  }
}
