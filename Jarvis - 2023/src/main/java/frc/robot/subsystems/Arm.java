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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalInput;
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

  private CANCoder elbowEncoder = new CANCoder(13);
  private CANCoder wristEncoder = new CANCoder(14);


  public static double elbowSpeedLimit = 0.5;
  public static double baseSpeedLimit = 0.5;

  public Gains baseJointGains = new Gains(20, 0, 0, 0,0, 0.25);

  public Gains elbowJointGains = new Gains(25, 0, 10, 0, 0, 0.30);

  public Gains wristJointGains = new Gains(5, 0, 0, 0, 0, 0.5);
  public Gains clawJointGains = new Gains(10, 0, 0, 0, 0, 0.5);

  public ArmState currentState = new ArmState();

  private final DigitalInput clawLimitSwitch = new DigitalInput(5);


  private static double wristJointOffset; 
  private static double elbowJointOffset;

  public static boolean isCube = true;

  public static double cableLengthOffset = 0;





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

    // elbowMotor.configRemoteFeedbackFilter(elbowEncoder, 0);
    // elbowMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
    // elbowMotor.configSelectedFeedbackCoefficient((1/Constants.CANCODER_UNITS_PER_REV) * 360);



    elbowMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    elbowMotor.configSelectedFeedbackCoefficient((1/ArmConstants.ELBOW_GEAR_RATIO) * (1/Constants.FALCON_UNITS_PER_REV) * 360);

    elbowJointOffset = elbowEncoder.getAbsolutePosition() - elbowMotor.getSelectedSensorPosition();




    elbowMotor.config_kP(Constants.PRIMARY_PID_LOOP_IDX, elbowJointGains.P);
    elbowMotor.config_kI(Constants.PRIMARY_PID_LOOP_IDX, elbowJointGains.I);
    elbowMotor.config_kD(Constants.PRIMARY_PID_LOOP_IDX, elbowJointGains.D);
    elbowMotor.config_kF(Constants.PRIMARY_PID_LOOP_IDX, elbowJointGains.F);

    elbowMotor.configClosedLoopPeakOutput(Constants.PRIMARY_PID_LOOP_IDX, elbowJointGains.PeakOutput);
    elbowMotor.configClosedloopRamp(1.5);


    //----------------------------------------------------------------


    wristMotor.configFactoryDefault();
    wristMotor.setNeutralMode(NeutralMode.Brake);
    wristMotor.set(ControlMode.PercentOutput, 0);
    wristMotor.setInverted(false);

    wristMotor.configRemoteFeedbackFilter(wristEncoder, 0);
    wristMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
    wristMotor.configSelectedFeedbackCoefficient((1/Constants.CANCODER_UNITS_PER_REV) * 360);
    // wristMotor.setSelectedSensorPosition(wristEncoder.g);

    wristJointOffset = wristEncoder.getAbsolutePosition() - wristMotor.getSelectedSensorPosition();

    wristMotor.config_kP(Constants.PRIMARY_PID_LOOP_IDX, wristJointGains.P);
    wristMotor.config_kI(Constants.PRIMARY_PID_LOOP_IDX, wristJointGains.I);
    wristMotor.config_kD(Constants.PRIMARY_PID_LOOP_IDX, wristJointGains.D);
    wristMotor.config_kF(Constants.PRIMARY_PID_LOOP_IDX, wristJointGains.F);

    wristMotor.configClosedLoopPeakOutput(Constants.PRIMARY_PID_LOOP_IDX, wristJointGains.PeakOutput);
    wristMotor.configClosedloopRamp(1);

    //------------------------------------------------------------------

    clawMotor.configFactoryDefault();
    clawMotor.setNeutralMode(NeutralMode.Brake); 
    clawMotor.set(ControlMode.PercentOutput,0);
    clawMotor.setInverted(true);

    clawMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    clawMotor.configSelectedFeedbackCoefficient((1/Constants.QUAD_ENCODER_UNITS_PER_REV)* 360);

    clawMotor.config_kP(Constants.PRIMARY_PID_LOOP_IDX, clawJointGains.P);
    clawMotor.config_kI(Constants.PRIMARY_PID_LOOP_IDX, clawJointGains.I);
    clawMotor.config_kD(Constants.PRIMARY_PID_LOOP_IDX, clawJointGains.D);
    clawMotor.config_kF(Constants.PRIMARY_PID_LOOP_IDX, clawJointGains.F);

    clawMotor.configClosedLoopPeakOutput(Constants.PRIMARY_PID_LOOP_IDX, clawJointGains.PeakOutput);
    resetClawPos(0);


    wristMotor.setSelectedSensorPosition(wristEncoder.getAbsolutePosition());


    // elbowMotor.setSelectedSensorPosition(elbowEncoder.getAbsolutePosition());
    currentState.setPostion(getBaseJointPosition(), getElbowJointPosition(), getWristJointPosition(), getClawPosition());


  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Base Angle", getBaseJointPosition().getDegrees());
    SmartDashboard.putNumber("Elbow Angle", getElbowJointPosition().getDegrees());
    SmartDashboard.putNumber("Wrist Angle", getWristJointPosition().getDegrees());
    SmartDashboard.putNumber("Claw Position", getClawPosition());
    SmartDashboard.putBoolean("Claw Limit", getClawLimitSwitch());

    // SmartDashboard.putNumber("Elbow Absolute", elbowEncoder.getAbsolutePosition());
    // SmartDashboard.putNumber("Wrist Absolute", wristEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("Claw Offset", getRelativeCableOffset());

    SmartDashboard.putString("Arm End", getArmEndPosition().toString());

    SmartDashboard.putBoolean("isCube", isCube);



    currentState.setPostion(getBaseJointPosition(), getElbowJointPosition(), getWristJointPosition(), getClawPosition());

    // isCube = SmartDashboard.getBoolean("isCube", isCube);
    

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
    System.out.println(isClosed);
    if(isClosed){
      if(!Constants.testCode){
        setClawMotorOutput(ArmConstants.CLAW_CLOSING_OUTPUT);
      }else{
        if(isCube){
          setClawPosition(ArmConstants.CUBE_CLAW_OFFSET);
        }else{
          setClawPosition(ArmConstants.CONE_CLAW_OFFSET);
        }
      }

     
    }else{
      setClawPosition(-20);
    }
  }

  public Translation2d getArmEndPosition(){
    return new Pose2d(new Translation2d(0,6), Rotation2d.fromDegrees(90 - getBaseJointPosition().getDegrees()))
        .plus(new Transform2d(new Translation2d(ArmConstants.HUMOROUS_LENGTH, 0), Rotation2d.fromDegrees(getElbowJointPosition().getDegrees() - 180)))
        .plus(new Transform2d(new Translation2d(ArmConstants.FOREARM_LENGTH, 0), Rotation2d.fromDegrees(getWristJointPosition().getDegrees())))
        .plus(new Transform2d(new Translation2d(ArmConstants.CLAW_LENGTH, 0), new Rotation2d())).getTranslation();

  }

  /**
   * Sets the base joint to a specific percent output
   * @param percentOutput %
   */
  public void setBaseMotorOutput(double percentOutput){
    baseMotor.set(ControlMode.PercentOutput, percentOutput);
  }

  /**
   * Set the target angle for the base joint. 
   * 0 degrees is vertical, pos forward
   * @param angle target angle
   */
  public void setBaseJointPosition(Rotation2d angle){
    double FF = Math.sin(getBaseJointPosition().getRadians() * ArmConstants.BASE_HOLDING_POWER);

    baseMotor.set(ControlMode.Position, angle.getDegrees() + ArmConstants.BASE_JOINT_OFFSET, DemandType.ArbitraryFeedForward, FF);
  }

  /**
   * 
   * @return the position of the base joint
   */
  public Rotation2d getBaseJointPosition(){
    return Rotation2d.fromDegrees(baseMotor.getSelectedSensorPosition() - ArmConstants.BASE_JOINT_OFFSET);
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
    elbowMotor.set(ControlMode.Position, angle.getDegrees() - elbowJointOffset + ArmConstants.ELBOW_MAGNET_OFFSET);
  }

  /**
   * get Elbow position
   * @return
   */
  public Rotation2d getElbowJointPosition(){
    return Rotation2d.fromDegrees(elbowMotor.getSelectedSensorPosition() + elbowJointOffset - ArmConstants.ELBOW_MAGNET_OFFSET);
  }


  public void setWristMotorOutput(double percentOutput){
    wristMotor.set(ControlMode.PercentOutput, percentOutput);
  }

  public void setWristJointPosition(Rotation2d angle){
    wristMotor.set(ControlMode.Position, angle.getDegrees() - wristJointOffset + ArmConstants.WRIST_MAGNET_OFFSET);
  }

  public Rotation2d getWristJointPosition() {
    return Rotation2d.fromDegrees(wristMotor.getSelectedSensorPosition() + wristJointOffset - ArmConstants.WRIST_MAGNET_OFFSET);
  }

  
  public void setClawMotorOutput (double percentOutput){
    clawMotor.set(ControlMode.PercentOutput, percentOutput);
  }

  public void setClawPosition(double closePercent){
    if(Constants.testCode){
      clawMotor.set(ControlMode.Position, closePercent - getRelativeCableOffset());
    }else{}
      clawMotor.set(ControlMode.Position, closePercent);
    }


  public double getClawPosition(){
    return clawMotor.getSelectedSensorPosition() + getRelativeCableOffset();
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
    if(position == 0 && getClawLimitSwitch()){
      cableLengthOffset = getClawCableLengthOffset();
    }
  }

  public boolean getClawLimitSwitch(){
    return !clawLimitSwitch.get();
  }

  public double getClawCableLengthOffset(){
    double lengthOffset =  -1*(getBaseJointPosition().getDegrees()/360 * ArmConstants.PULLEY_CIRCUMFERENCE) 
    - (getElbowJointPosition().getDegrees()/360 * ArmConstants.PULLEY_CIRCUMFERENCE)
    - (getWristJointPosition().getDegrees()/360 * ArmConstants.WRIST_PULLEY_CIRCUMFERENCE);

    return lengthOffset/ArmConstants.CLAW_CABLE_LENGTH_OPEN_TO_CLOSE * ArmConstants.CLAW_CLOSE_LIMIT;
  }

  public double getRelativeCableOffset(){
    return getClawCableLengthOffset() - cableLengthOffset;
  }

  public void toggleCloseType(){
    isCube = !isCube;
  }

  public boolean getCloseType(){
    return isCube;
  }

}
