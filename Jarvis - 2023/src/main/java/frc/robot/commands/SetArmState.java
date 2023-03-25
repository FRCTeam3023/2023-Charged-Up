// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ArmState;
import frc.robot.subsystems.Arm;

public class SetArmState extends CommandBase {
  /** Creates a new SetArmState. */
  private Arm arm;
  private ArmState targetState;

  public SetArmState(Arm arm, ArmState targetState) {
    this.arm = arm;
    this.targetState = targetState;
    addRequirements(arm);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.setArmState(targetState);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.stopAllMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(
    Math.abs(arm.getBaseJointPosition().minus(targetState.baseJointPosition).getDegrees()) < 5 &&
    Math.abs(arm.getElbowJointPosition().minus(targetState.elbowJointPosition).getDegrees()) < 5 &&
    Math.abs(arm.getWristJointPosition().minus(targetState.wristJointPosition).getDegrees()) < 10
    ){
      return true;
    }
    return false;
  }
}
