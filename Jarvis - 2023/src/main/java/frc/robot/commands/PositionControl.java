// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;

public class PositionControl extends CommandBase {
  /** Creates a new PositionControl. */

  private Arm arm;
  private Joystick joystick;

  private Rotation2d startPos = new Rotation2d();



  public PositionControl(Drivetrain drivetrain, Arm arm, Joystick joystick) {
    this.arm = arm;
    this.joystick = joystick;
    addRequirements(arm);
    addRequirements(drivetrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startPos = arm.getBaseJointPos();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Rotation2d joystickPosition = Rotation2d.fromDegrees(45*joystick.getY());
    arm.setBaseMotorPosition(startPos.plus(joystickPosition));

    SmartDashboard.putNumber("Target Pos", startPos.plus(joystickPosition).getDegrees());
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.setBaseMotorOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
