// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class JoystickControl extends CommandBase {
  private Arm arm;
  private Joystick joystick;
  /** Creates a new JoystickControl. */
  public JoystickControl(Arm arm, Joystick joystick) {
    this.arm = arm;
    this.joystick = joystick;

    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.setBaseMotorOutput(joystick.getY() * Arm.baseSpeedLimit);

    if(joystick.getRawButtonPressed(5)){
      Arm.baseSpeedLimit = Arm.baseSpeedLimit + 0.05;
    }
    if(joystick.getRawButtonPressed(6)){
      Arm.baseSpeedLimit = Arm.baseSpeedLimit - 0.05;
    }
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
