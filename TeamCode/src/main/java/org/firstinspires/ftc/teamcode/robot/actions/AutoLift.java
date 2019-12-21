package org.firstinspires.ftc.teamcode.robot.actions;

        import org.firstinspires.ftc.teamcode.autonomous.Action;
        import org.firstinspires.ftc.teamcode.robot.Robot;

public class AutoLift implements Action {

    private Robot robot;
    private double initial = 0;

    private double setPointMetersLift;

    public AutoLift(double meters) {
        this.setPointMetersLift = meters;

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
        if (robot.getTouchSensorLmin().isPressed()){
            robot.getLiftMotor().setPower(0);
        }
        if (robot.getTouchSensorLmax().isPressed()){
            robot.getLiftMotor().setPower(0);
        }
        double power = robot.getDrivePID().distance(robot.getLiftMotor().getCurrentPosition(), initial + Robot.TICK_PER_METER_LIFT * setPointMetersLift, 1);
        robot.drive(power, 0);
        return Math.abs(power) == 0;
    }
}