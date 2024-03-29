// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */

public class PIDDisplay extends SubsystemBase{

    //shuffeboard stuff

    public static final SendableChooser<PIDSetter> PIDList = new SendableChooser<>();
    public static final WPILibSetter defaultPID = new WPILibSetter(new PIDController(0, 0, 0));

    private static final ShuffleboardTab PIDTab = Shuffleboard.getTab("PID");

    private static GenericEntry PEntry = PIDTab.add("P", 0).withPosition(2, 2).getEntry();
    private static GenericEntry IEntry = PIDTab.add("I", 0).getEntry();
    private static GenericEntry DEntry = PIDTab.add("D", 0).getEntry();
    private static GenericEntry FEntry = PIDTab.add("F", 0).getEntry();



    private static PIDSetter selectedPID;
    private static PIDSetter lastSelected;

    private static double PValue;
    private static double IValue;
    private static double DValue;
    private static double FValue;


    
    public PIDDisplay(){
        PIDList.setDefaultOption("Default PID", defaultPID);
        PIDTab.add(PIDList).withPosition(0, 0).withSize(4,1);
    }

    @Override
    public void periodic(){



        //Check if the selected PID config is different. If so update the Dashboard with new numbers
        selectedPID = PIDList.getSelected();
        double[] codeCurrentPID = selectedPID.getPID();
        if(selectedPID != lastSelected){
            PEntry.setDouble(codeCurrentPID[0]);
            IEntry.setDouble(codeCurrentPID[1]);
            DEntry.setDouble(codeCurrentPID[2]);
            FEntry.setDouble(codeCurrentPID[3]);
        }
        lastSelected = selectedPID;


        
        PValue = PEntry.getDouble(codeCurrentPID[0]);
        IValue = IEntry.getDouble(codeCurrentPID[1]);
        DValue = DEntry.getDouble(codeCurrentPID[2]);
        FValue = FEntry.getDouble(codeCurrentPID[3]);


        //Check if any of the dashboard values are different than the PID values in code, write updated values
        if(PValue != codeCurrentPID[0] || IValue != codeCurrentPID[1] || DValue != codeCurrentPID[2] || FValue != codeCurrentPID[3]){
            selectedPID.setPID(PValue, IValue, DValue, FValue);
        }



    }


    public interface PIDSetter {

        public void setPID(double p, double i, double d, double f);

        public double[] getPID();
        
    }
}
