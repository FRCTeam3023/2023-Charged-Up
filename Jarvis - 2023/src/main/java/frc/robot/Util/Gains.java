package frc.robot.Util;

public class Gains {
    public final double P;
	public final double I;
	public final double D;
	public final double F;
	public final int Izone;
	public final double PeakOutput;

	/** 
     * Class for PID gains
     * @param P P gain
     * @param I I gain
     * @param D D gain
     * @param F FF gain
     * @param Izone I zone
     * @param PeakOutput Peak Output
     */
	public Gains(double P, double I, double D, double F, int Izone, double PeakOutput){
		this.P = P;
		this.I = I;
		this.D = D;
		this.F = F;
		this.Izone = Izone;
		this.PeakOutput = PeakOutput;
	}
}
