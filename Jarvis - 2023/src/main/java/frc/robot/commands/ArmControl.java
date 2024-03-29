// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Util.ArmState;
import frc.robot.subsystems.Arm;

public class ArmControl extends CommandBase {
  /** Creates a new ArmControl. */
  Arm arm;
  Joystick joystick;

  boolean limitSwitch = false;
  boolean lastLimitSwitch = false;
  private static ArmState stateToHold = new ArmState();
  public ArmControl(Arm arm, Joystick joystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.joystick = joystick;
    addRequirements(arm);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stateToHold.setPostion(arm.currentState);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Math.abs(joystick.getY()) > 0.25){
      if(Constants.testCode){
        if(arm.currentState.baseJointPosition.getDegrees() > 20){
          arm.setBaseMotorOutput(Math.min(-joystick.getY()* 0.1, 0));
        }else{
          arm.setBaseMotorOutput(-joystick.getY()* 0.1);
        }
      }else{
        arm.setBaseMotorOutput(-joystick.getY()* 0.1);

      }
      stateToHold.baseJointPosition = arm.currentState.baseJointPosition;
    }else{
      arm.setBaseJointPosition(stateToHold.baseJointPosition);
    }


    if(joystick.getPOV() == 0){
      if(Constants.testCode){
        if(arm.currentState.baseJointPosition.getDegrees() > 10 && arm.currentState.elbowJointPosition.getDegrees() > 130){
          // arm.setElbowJointPosition(Rotation2d.fromDegrees(100));
          arm.setElbowMotorOutput(0);
          
        }else if(arm.currentState.baseJointPosition.getDegrees() <= 10 && arm.currentState.elbowJointPosition.getDegrees() >= 100){
          if(arm.currentState.elbowJointPosition.getDegrees() > 93){
            arm.setElbowJointPosition(Rotation2d.fromDegrees(90));
          }else{
            arm.setElbowMotorOutput(0);
          }
          // arm.setElbowJointPosition(Rotation2d.fromDegrees(70));
        }else{
          arm.setElbowMotorOutput(0.25);
        }
      }else{
        arm.setElbowMotorOutput(0.25);
      }
      stateToHold.elbowJointPosition = arm.currentState.elbowJointPosition;
    }else if(joystick.getPOV() == 180){
      arm.setElbowMotorOutput(-.08);
      stateToHold.elbowJointPosition = arm.currentState.elbowJointPosition;

    }else{
      arm.setElbowJointPosition(stateToHold.elbowJointPosition);
    }

    if(joystick.getRawButton(9)){
      arm.setWristMotorOutput(0.2);
      stateToHold.wristJointPosition = arm.currentState.wristJointPosition;
    }else if(joystick.getRawButton(11)){
      arm.setWristMotorOutput(-.2);
      stateToHold.wristJointPosition = arm.currentState.wristJointPosition;
    }else{
      arm.setWristJointPosition(stateToHold.wristJointPosition);
    }

    if(joystick.getRawButton(1)){
      arm.setClawState(true);
      stateToHold.clawClosed = arm.currentState.clawClosed;
    }else if(joystick.getRawButton(2)){
      arm.setClawState(false);
      stateToHold.clawClosed = arm.currentState.clawClosed;
    }else if(joystick.getRawButton(10)){
      arm.setClawMotorOutput(-0.2);
      stateToHold.clawClosed = arm.currentState.clawClosed;
    }else if(joystick.getRawButton(12)){
      arm.setClawMotorOutput(ArmConstants.CLAW_CLOSING_OUTPUT);
      stateToHold.clawClosed = arm.currentState.clawClosed;
    }else{
      // arm.setClawPosition(arm.getClawPosition());
      arm.setClawState(arm.currentState.clawClosed);
    }

    if(joystick.getRawButton(8)){
      arm.resetClawPos(0);
    }


    limitSwitch = arm.getClawLimitSwitch();
    if(limitSwitch != lastLimitSwitch && limitSwitch){
      arm.resetClawPos(0);
    }

    lastLimitSwitch = arm.getClawLimitSwitch();

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
