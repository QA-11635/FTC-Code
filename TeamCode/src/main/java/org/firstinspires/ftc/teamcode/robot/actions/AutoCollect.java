package org.firstinspires.ftc.teamcode.robot.actions;

import org.firstinspires.ftc.teamcode.autonomous.Action;
import org.firstinspires.ftc.teamcode.robot.Robot;


public class AutoCollect implements Action {

    private Robot robot;
    private boolean open;

    public AutoCollect(boolean open) {
        this.open = open;
    }

    @Override
    public void setup(Robot robot) {
        this.robot = robot;
    }
    @Override
    public boolean loop() {
        if (open){
            robot.getCollectL().setPower(1);
            robot.getCollectR().setPower(1);
            if (robot.getTouchSensorC().isPressed()){
                robot.getCollectL().setPower(0);
                robot.getCollectR().setPower(0);
            }else {
                robot.getCollectL().setPower(1);
                robot.getCollectR().setPower(1);
            }
        }
        else{
            robot.getCollectL().setPower(0);
            robot.getCollectR().setPower(0);
        }
        return open;

    }
}

