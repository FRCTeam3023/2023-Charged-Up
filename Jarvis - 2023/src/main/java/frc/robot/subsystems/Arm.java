// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenixpro.hardware.CANcoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ArmState;
import frc.robot.Constants;
import frc.robot.Gains;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {

  private TalonFX baseMotor = new TalonFX(10);
  private TalonFX elbowMotor = new TalonFX(11);
  private TalonSRX wristMotor = new TalonSRX(13);
  private TalonSRX clawMotor = new TalonSRX(12);

  private CANCoder wristEncoder = new CANCoder(20);


  public static double elbowSpeedLimit = 0.5;
  public static double baseSpeedLimit = 0.5;

  public Gains baseJointGains = new Gains(20, 0, 0, 0,0, 0.25);

  public Gains elbowJointGains = new Gains(25, 0, 10, 0, 0, 0.30);

  public Gains wristJointGains = new Gains(15, 0, 0, 0, 0, 0.25);
  public Gains clawJointGains = new Gains(20, 0, 0, 0, 0, 0.5);

  public ArmState currentState = new ArmState();




  public Arm() {
    baseMotor.configFactoryDefault();
    baseMotor.setNeutralMode(NeutralMode.Brake);
    baseMotor.set(ControlMode.PercentOutput, 0);
    baseMotor.setInverted(false);

    //select internal sensor
    baseMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.PRIMARY_PID_LOOP_IDX, Constants.TIMEOUT_MS);
    //change the feedback units to degrees of output
    baseMotor.configSelectedFeedbackCoefficient( (1/Constants.FALCON_UNITS_PER_REV) * (1/ArmConstants.BASE_GEAR_RATIO) * 360);
    

    baseMotor.config_kP(Constants.PRIMARY_PID_LOOP_IDX, baseJointGains.P);
    baseMotor.config_kI(Constants.PRIMARY_PID_LOOP_IDX, baseJointGains.I);
    baseMotor.config_kD(Constants.PRIMARY_PID_LOOP_IDX, baseJointGains.D);
    baseMotor.config_kF(Constants.PRIMARY_PID_LOOP_IDX, baseJointGains.F);

    baseMotor.configClosedLoopPeakOutput(Constants.PRIMARY_PID_LOOP_IDX, baseJointGains.PeakOutput);
    baseMotor.configClosedloopRamp(1.5);

    baseMotor.setSelectedSensorPosition(0);

    //-----------------------------------------------------------

    elbowMotor.configFactoryDefault();
    elbowMotor.setNeutralMode(NeutralMode.Brake);
    elbowMotor.set(ControlMode.PercentOutput, 0);
    elbowMotor.setInverted(false);

    elbowMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, Constants.PRIMARY_PID_LOOP_IDX, Constants.TIMEOUT_MS);

    //returns degrees of motion instead of sensor units
    elbowMotor.configSelectedFeedbackCoefficient( (1/Constants.FALCON_UNITS_PER_REV) * (1/ArmConstants.ELBOW_GEAR_RATIO) * 360);


    elbowMotor.config_kP(Constants.PRIMARY_PID_LOOP_IDX, elbowJointGains.P);
    elbowMotor.config_kI(Constants.PRIMARY_PID_LOOP_IDX, elbowJointGains.I);
    elbowMotor.config_kD(Constants.PRIMARY_PID_LOOP_IDX, elbowJointGains.D);
    elbowMotor.config_kF(Constants.PRIMARY_PID_LOOP_IDX, elbowJointGains.F);

    elbowMotor.configClosedLoopPeakOutput(Constants.PRIMARY_PID_LOOP_IDX, elbowJointGains.PeakOutput);
    elbowMotor.configClosedloopRamp(1.5);

    elbowMotor.setSelectedSensorPosition(0);

    //----------------------------------------------------------------

    wristMotor.configFactoryDefault();
    wristMotor.setNeutralMode(NeutralMode.Brake);
    wristMotor.set(ControlMode.PercentOutput, 0);
    wristMotor.setInverted(false);

    wristMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder );
    wristMotor.configSelectedFeedbackCoefficient((1/Constants.QUAD_ENCODER_UNITS_PER_REV) * 360);

    wristMotor.config_kP(Constants.PRIMARY_PID_LOOP_IDX, wristJointGains.P);
    wristMotor.config_kI(Constants.PRIMARY_PID_LOOP_IDX, wristJointGains.I);
    wristMotor.config_kD(Constants.PRIMARY_PID_LOOP_IDX, wristJointGains.D);
    wristMotor.config_kF(Constants.PRIMARY_PID_LOOP_IDX, wristJointGains.F);

    wristMotor.configClosedLoopPeakOutput(Constants.PRIMARY_PID_LOOP_IDX, wristJointGains.PeakOutput);
    wristMotor.configClosedloopRamp(1.5);

    //------------------------------------------------------------------

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
    resetClawPos(0);



    currentState.setPostion(getBaseJointPosition(), getElbowJointPosition(), getWristJointPosition(), getClawPosition());

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Angle", getBaseJointPosition().getDegrees());
    SmartDashboard.putNumber("Elbow Angle", getElbowJointPosition().getDegrees());
    SmartDashboard.putNumber("Wrist Angle", getWristJointPosition().getDegrees());
    SmartDashboard.putNumber("Claw Position", getClawPosition());

    System.out.println("Encoder Pos: " + wristEncoder.getPosition());

    currentState.setPostion(getBaseJointPosition(), getElbowJointPosition(), getWristJointPosition(), getClawPosition());

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }


  public void setArmState(ArmState state){
    setBaseJointPosition(state.baseJointPosition);
    setElbowJointPosition(state.elbowJointPosition);
    setWristJointPosition(state.wristJointPosition);
    setClawState(state.clawClosed);
  }

  public void setClawState(boolean isClosed){
    if(isClosed){
      if(getClawPosition() > ArmConstants.CLAW_CLOSE_THRESHOLD){
        setClawPosition(ArmConstants.CLAW_CLOSE_LIMIT);
      }else{
        setClawMotorOutput(ArmConstants.CLAW_CLOSING_OUTPUT);
      }
    }else{
      setClawPosition(0);
    }
  }



  /**
   * Sets the base joint to a specific percent output
   * @param percentOutput %
   */
  public void setBaseMotorOutput(double percentOutput){
    baseMotor.set(ControlMode.PercentOutput, percentOutput);
    System.out.println(percentOutput);
  }

  /**
   * Set the target angle for the base joint. 
   * 0 degrees is vertical, pos forward
   * @param angle target angle
   */
  public void setBaseJointPosition(Rotation2d angle){
    double FF = Math.sin(getBaseJointPosition().getRadians() * ArmConstants.BASE_HOLDING_POWER);

    baseMotor.set(ControlMode.Position, angle.getDegrees(), DemandType.ArbitraryFeedForward, FF);
  }

  /**
   * 
   * @return the position of the base joint
   */
  public Rotation2d getBaseJointPosition(){
    return Rotation2d.fromDegrees(baseMotor.getSelectedSensorPosition());
  }


  /**
   * Set output percent of elbow motor
   * @param percentOutput
   */
  public void setElbowMotorOutput(double percentOutput){
    elbowMotor.set(ControlMode.PercentOutput, percentOutput);

  }

  /**
   * set the target angle for the elbow joint.
   * 0 degrees is straight down.
   * @param angle
   */
  public void setElbowJointPosition(Rotation2d angle){
    elbowMotor.set(ControlMode.Position, angle.getDegrees());
  }

  /**
   * get Elbow position
   * @return
   */
  public Rotation2d getElbowJointPosition(){
    return Rotation2d.fromDegrees(elbowMotor.getSelectedSensorPosition());
  }


  public void setWristMotorOutput(double percentOutput){
    wristMotor.set(ControlMode.PercentOutput, percentOutput);
  }

  public void setWristJointPosition(Rotation2d angle){
    wristMotor.set(ControlMode.Position, angle.getDegrees());
  }

  public Rotation2d getWristJointPosition() {
    return Rotation2d.fromDegrees(wristMotor.getSelectedSensorPosition());
  }



  
  public void setClawMotorOutput (double percentOutput){
    clawMotor.set(ControlMode.PercentOutput, percentOutput);
  }

  public void setClawPosition(double closePercent){
    clawMotor.set(ControlMode.Position, closePercent);

  }

  public double getClawPosition(){
    return clawMotor.getSelectedSensorPosition();
  }

  public boolean getClawClosed(){
    if(clawMotor.getSelectedSensorPosition() > 0.1){
      return true;
    }

    return false;
  }

  public double getBaseError(){
    return baseMotor.getClosedLoopError();
  }

  public void stopAllMotors(){
    baseMotor.set(ControlMode.PercentOutput, 0);
    elbowMotor.set(ControlMode.PercentOutput, 0);
    wristMotor.set(ControlMode.PercentOutput, 0);
    clawMotor.set(ControlMode.PercentOutput, 0);
  }

  public void zeroEncoders(){
    baseMotor.setSelectedSensorPosition(0);
    elbowMotor.setSelectedSensorPosition(0);

  }

  public void zeroClawPos(){
    clawMotor.setSelectedSensorPosition(0);
  }

  public void resetClawPos(double position){
    clawMotor.setSelectedSensorPosition(position);
  }
}
