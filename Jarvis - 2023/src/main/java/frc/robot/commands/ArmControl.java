// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ArmState;
import frc.robot.subsystems.Arm;

public class ArmControl extends CommandBase {
  /** Creates a new ArmControl. */
  Arm arm;
  Joystick joystick;
  ArmState stateToHold;
  public ArmControl(Arm arm, Joystick joystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.joystick = joystick;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stateToHold = arm.currentState;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Math.abs(joystick.getY()) > 0.25){
      arm.setBaseMotorOutput(-joystick.getY()* 0.2);
      stateToHold.baseJointPosition = arm.currentState.baseJointPosition;
    }else{
      arm.setBaseJointPosition(stateToHold.baseJointPosition);
    }


    if(joystick.getPOV() == 0){
      arm.setElbowMotorOutput(0.15);
      stateToHold.elbowJointPosition = arm.currentState.elbowJointPosition;
    }else if(joystick.getPOV() == 180){
      arm.setElbowMotorOutput(-.15);
      stateToHold.elbowJointPosition = arm.currentState.elbowJointPosition;
    }else{
      arm.setElbowJointPosition(stateToHold.elbowJointPosition);
    }

    if(joystick.getRawButton(9)){
      arm.setWristMotorOutput(0.15);
      stateToHold.wristJointPosition = arm.currentState.wristJointPosition;
    }else if(joystick.getRawButton(11)){
      arm.setWristMotorOutput(-.15);
      stateToHold.wristJointPosition = arm.currentState.wristJointPosition;
    }else{
      arm.setWristJointPosition(stateToHold.wristJointPosition);
    }

    if(joystick.getRawButton(1)){
      arm.setClawState(true);
    }
    if(joystick.getRawButton(2)){
      arm.setClawState(false);
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
