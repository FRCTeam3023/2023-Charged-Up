// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class SetArmState extends CommandBase {
  /** Creates a new SetArmState. */
  private Arm arm;
  private Rotation2d basePos;
  private Rotation2d elbowPos;
  private Rotation2d wristPos;
  private boolean clawClosed;

  public SetArmState(Arm arm, Rotation2d basePos, Rotation2d elbowPos, Rotation2d wristPos, boolean clawClosed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.basePos = basePos;
    this.elbowPos = elbowPos;
    this.wristPos = wristPos;
    this.clawClosed = clawClosed;

    addRequirements(arm);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.setArmState(basePos, elbowPos, wristPos, clawClosed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.setArmState(basePos, elbowPos, wristPos, clawClosed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.setArmState(
      arm.getBaseJointPos(), 
      arm.getElbowMotorPos(), 
      arm.getWristMotorPos(), 
      arm.getClawPosition()
    );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(
    Math.abs(arm.getBaseJointPos().minus(basePos).getDegrees()) < 5 &&
    Math.abs(arm.getElbowMotorPos().minus(elbowPos).getDegrees()) < 5 &&
    Math.abs(arm.getWristMotorPos().minus(wristPos).getDegrees()) < 5
    ){
      return true;
    }
    return false;
  }
}
