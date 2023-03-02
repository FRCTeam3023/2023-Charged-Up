// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;

public class PercentageControl extends CommandBase {
  private Arm arm;
  private Joystick joystick;
  private Joystick secondaryJoystick;
  /** Creates a new JoystickControl. */
  public PercentageControl(Drivetrain drivetrain, Arm arm, Joystick joystick, Joystick secondaryJoystick) {
    this.arm = arm;
    this.joystick = joystick;
    this.secondaryJoystick = secondaryJoystick;

    addRequirements(arm);
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.setBaseMotorOutput(-joystick.getY() * Arm.baseSpeedLimit);

    arm.setElbowMotorOutput(-secondaryJoystick.getY() * Arm.elbowSpeedLimit);

    if((secondaryJoystick.getPOV() < 90 || secondaryJoystick.getPOV() > 270) && secondaryJoystick.getPOV() >= 0){
      arm.setWristMotorOutput(0.15);

    }else if(secondaryJoystick.getPOV() > 90 && secondaryJoystick.getPOV() < 270){
      arm.setWristMotorOutput(-0.15);

    }else{
      arm.setWristMotorOutput(0);
    }



    // if(joystick.getRawButton(8)){
    //   arm.setWristMotorOutput(0.2);
    // }else if(joystick.getRawButton(10)){
    //   arm.setWristMotorOutput(-0.2);
    // }else{
    //   arm.setWristMotorOutput(0);
    // }

    if(secondaryJoystick.getRawButton(2)){
      arm.setClawMotorOutput(0.3);

    }else if(secondaryJoystick.getRawButton(1)){
      arm.setClawMotorOutput(-.3);
    }else{
      arm.setClawMotorOutput(0);
    }


    // if(joystick.getRawButtonPressed(5)){
    //   Arm.baseSpeedLimit = Arm.baseSpeedLimit + 0.05;
    // }
    // if(joystick.getRawButtonPressed(6)){
    //   Arm.baseSpeedLimit = Arm.baseSpeedLimit - 0.05;
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.stopAllMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
