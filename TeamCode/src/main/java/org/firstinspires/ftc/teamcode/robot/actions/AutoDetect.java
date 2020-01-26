package org.firstinspires.ftc.teamcode.robot.actions;


import org.firstinspires.ftc.teamcode.autonomous.Action;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.actions.AutoTurn;


public class AutoDetect implements Action {

    private Robot robot;
    private boolean on;

    public AutoDetect(boolean on) {
        this.on = on;

    }

    @Override
    public void setup(Robot robot) {
        this.robot = robot;

    }

    @Override
    public boolean loop() {
        if (on) {
            if (robot.getColorSensorS().alpha() <= 110) {
                robot.getCollectR().setPower(1);
                robot.getCollectL().setPower(1);
            }else{
                robot.getCollectR().setPower(0);
                robot.getCollectL().setPower(0);
            }
        }
        return true;
    }
}