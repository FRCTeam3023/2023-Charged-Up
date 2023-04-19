// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.ArmConstants;

/** Add your docs here. */
public class ArmState {

    public Rotation2d baseJointPosition;
    public Rotation2d elbowJointPosition;
    public Rotation2d wristJointPosition;
    public double clawPos;
    public boolean clawClosed;

    /**
     * constructs a new arm state with 0s for angles
     */
    public ArmState(){
        baseJointPosition = new Rotation2d();
        elbowJointPosition = new Rotation2d();
        wristJointPosition = new Rotation2d();
        clawPos = 0;
        clawClosed = false;
    }

    // public ArmState(Rotation2d baseJointPosition,Rotation2d elbowJointPos,Rotation2d wristJointPos, double clawPos){
    //     this.baseJointPosition = baseJointPosition;
    //     this.elbowJointPosition = elbowJointPos;
    //     this.wristJointPosition = wristJointPos;
    //     this.clawPos = clawPos;
    //     if(clawPos > ArmConstants.CLAW_CLOSE_THRESHOLD){
    //         clawClosed = true;
    //     }
    // }

    public ArmState(Rotation2d baseJointPosition,Rotation2d elbowJointPos,Rotation2d wristJointPos, boolean clawClosed){
        this.baseJointPosition = baseJointPosition;
        this.elbowJointPosition = elbowJointPos;
        this.wristJointPosition = wristJointPos;
        this.clawClosed = true;
    }


    public void setPostion(Rotation2d baseJointPosition,Rotation2d elbowJointPos,Rotation2d wristJointPos, double clawPos){
        this.baseJointPosition = baseJointPosition;
        this.elbowJointPosition = elbowJointPos;
        this.wristJointPosition = wristJointPos;
        this.clawPos = clawPos;
        clawClosed = clawPos > ArmConstants.CLAW_CLOSE_THRESHOLD;
        // if(clawPos > ArmConstants.CLAW_CLOSE_THRESHOLD){
        //     clawClosed = true;
        // }
    }

    public void setPostion(ArmState state){
        this.baseJointPosition = state.baseJointPosition;
        this.elbowJointPosition = state.elbowJointPosition;
        this.wristJointPosition = state.wristJointPosition;
        this.clawPos = state.clawPos;
        clawClosed = state.clawPos > ArmConstants.CLAW_CLOSE_THRESHOLD;
        // if(clawPos > ArmConstants.CLAW_CLOSE_THRESHOLD){
        //     clawClosed = true;
        // }
    }

    
}
