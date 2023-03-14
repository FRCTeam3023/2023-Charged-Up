// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Gains;
import frc.robot.Constants.ArmConstants;

public class Claw extends SubsystemBase {
  /** Creates a new Claw. */
  private TalonSRX clawMotor = new TalonSRX(12);
  public Gains clawJointGains = new Gains(20, 0, 0, 0, 0, 0.5);


  public Claw() {
    
    clawMotor.configFactoryDefault();
    clawMotor.setNeutralMode(NeutralMode.Brake); 
    clawMotor.set(ControlMode.PercentOutput,0);
    clawMotor.setInverted(true);

    clawMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    clawMotor.configSelectedFeedbackCoefficient((1/Constants.QUAD_ENCODER_UNITS_PER_REV) * (1/ArmConstants.CLAW_PULLEY_RATIO) * 360);

    clawMotor.config_kP(Constants.PRIMARY_PID_LOOP_IDX, clawJointGains.P);
    clawMotor.config_kI(Constants.PRIMARY_PID_LOOP_IDX, clawJointGains.I);
    clawMotor.config_kD(Constants.PRIMARY_PID_LOOP_IDX, clawJointGains.D);
    clawMotor.config_kF(Constants.PRIMARY_PID_LOOP_IDX, clawJointGains.F);

    clawMotor.configClosedLoopPeakOutput(Constants.PRIMARY_PID_LOOP_IDX, clawJointGains.PeakOutput);
    resetPosition(0);


  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Claw Position", getPosition());
  }




  public void setState(boolean isClosed){
    if(isClosed){
      if(getPosition() > ArmConstants.CLAW_CLOSE_THRESHOLD){
        setPosition(ArmConstants.CLAW_CLOSE_LIMIT);
      }else{
        setOutputPercent(ArmConstants.CLAW_CLOSING_OUTPUT);
      }
    }else{
      setPosition(0);
    }
  }


  public void setOutputPercent (double percentOutput){
    clawMotor.set(ControlMode.PercentOutput, percentOutput);
  }

  public void setPosition(double closePercent){
    clawMotor.set(ControlMode.Position, closePercent);

  }

  public double getPosition(){
    return clawMotor.getSelectedSensorPosition();
  }

  public boolean getState(){
    if(clawMotor.getSelectedSensorPosition() > 0.1){
      return true;
    }

    return false;
  }

  public void resetPosition(double position){
    clawMotor.setSelectedSensorPosition(position);
  }
}
