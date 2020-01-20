package org.firstinspires.ftc.teamcode.robot.actions;

        import com.qualcomm.robotcore.hardware.DcMotor;

        import org.firstinspires.ftc.teamcode.autonomous.Action;
        import org.firstinspires.ftc.teamcode.robot.Robot;

public class AutoLift implements Action {

    private Robot robot;
    private double initial = 0;

    private int setPoint;

    public AutoLift(int encoder) {
        this.setPoint = encoder;

        // meters has to be in range between 0.12 to -0.12
        //0.12 >= setPointMetersLift >= -0.12
    }

    @Override
    public void setup(Robot robot) {
        this.robot = robot;
        this.initial = robot.getLiftMotor().getCurrentPosition();
    }

    @Override
    public boolean loop() {
//        if (robot.getTouchSensorLmin().isPressed()){
//            robot.getLiftMotor().setPower(0);
//            new AutoLift(0.02);
//        }
//        if (robot.getTouchSensorLmax().isPressed()){
//            robot.getLiftMotor().setPower(0);
//            new AutoLift(-0.02);
//        }
//        double power = robot.getDrivePID().distance(robot.getLiftMotor().getCurrentPosition(), initial + Robot.TICK_PER_METER_LIFT * setPointMetersLift, 0.1);
//        robot.drive(power, 0);
//        robot.getLiftMotor().setPower(power);
//        return Math.abs(power) == 0;

        robot.getLiftMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.getLiftMotor().setTargetPosition(setPoint);

        robot.getLiftMotor().setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.getLiftMotor().setPower(0.7);

        while (robot.getLiftMotor().isBusy()){

        }

        robot.getLiftMotor().setPower(0);

        robot.getLiftMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

        return true;
    }
}