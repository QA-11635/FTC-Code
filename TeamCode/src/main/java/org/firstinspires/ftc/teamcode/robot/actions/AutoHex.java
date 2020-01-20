package org.firstinspires.ftc.teamcode.robot.actions;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.autonomous.Action;
import org.firstinspires.ftc.teamcode.robot.Robot;

public class AutoHex implements Action {

    private Robot robot;
//    private double initial = 0;

    private int setPointH;

    public AutoHex(int encoder) {
        this.setPointH = encoder;
    }

    @Override
    public void setup(Robot robot) {
        this.robot = robot;
//        this.initial = robot.getHexMotor().getCurrentPosition();
    }

    @Override
    public boolean loop() {
        if (robot.getTouchSensorLmin().isPressed()){
            robot.getLiftMotor().setPower(0);
            new AutoHex(-100);
        }

//        double power = robot.getDrivePID().distance(robot.getHexMotor().getCurrentPosition(), initial + Robot.DEGREES_PER_HEX * armDegrees, 1);
//        robot.drive(power, 0);
//        robot.getHexMotor().setPower(power);
//        return Math.abs(power) == 0;
        robot.getHexMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.getHexMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.getHexMotor().setTargetPosition(setPointH);

        robot.getHexMotor().setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.getHexMotor().setPower(0.7);

        if (setPointH >= 0){
            robot.getHexMotor().setPower(0.5);
        }

        while (robot.getHexMotor().isBusy()){

        }

        robot.getHexMotor().setPower(0);

        robot.getHexMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

        return true;
    }
}