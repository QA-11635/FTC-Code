package org.firstinspires.ftc.teamcode.robot.actions;

import org.firstinspires.ftc.teamcode.autonomous.Action;
import org.firstinspires.ftc.teamcode.robot.Robot;

public class AutoHex implements Action {

    private Robot robot;
    private double initial = 0;

    private double armDegrees;

    public AutoHex(double degrees) {
        this.armDegrees = degrees;
    }

    @Override
    public void setup(Robot robot) {
        this.robot = robot;
        this.initial = robot.getHexMotor().getCurrentPosition();
    }

    @Override
    public boolean loop() {
        if (robot.getTouchSensorLmin().isPressed()){
            robot.getLiftMotor().setPower(0);
            new AutoHex(-20);
        }

        double power = robot.getDrivePID().distance(robot.getHexMotor().getCurrentPosition(), initial + Robot.DEGREES_PER_HEX * armDegrees, 1);
        robot.drive(power, 0);
        return Math.abs(power) == 0;
    }
}