package org.firstinspires.ftc.teamcode;

public class PID {

    private static final double DT = 0.01;

    private double kP, kI, kD;
    private double lastError = 0;
    private double integral = 0, derivative = 0;

    public PID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public double distance(double current, double setPoint, double tolerance) {
        double error = (setPoint - current) / setPoint;
        if (Math.abs(error) > tolerance) {
            derivative = (error - lastError) / DT;
            integral += error * DT;
            lastError = error;
            return kP * (error) + kI * (integral) + kD * (derivative);
        } else {
            return 0;
        }
    }

    public double angle(double current, double setPoint, double tolerance) {
        double error = (setPoint - current);
        if (Math.abs(error) > tolerance) {
            derivative = (error - lastError) / DT;
            integral += error * DT;
            lastError = error;
            return kP * (error) + kI * (integral) + kD * (derivative);
        } else {
            return 0;
        }
    }
}