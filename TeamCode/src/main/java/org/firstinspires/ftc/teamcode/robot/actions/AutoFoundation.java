package org.firstinspires.ftc.teamcode.robot.actions;

import org.firstinspires.ftc.teamcode.autonomous.Action;
import org.firstinspires.ftc.teamcode.robot.Robot;


public class AutoFoundation implements Action {

    private Robot robot;
    private int hold = 0;

    public AutoFoundation(int hold) {
        this.hold = hold;
    }

    @Override
    public void setup(Robot robot) {
        this.robot = robot;
    }
    @Override
    public boolean loop() {
        if (hold == 0){
            robot.getServoFoundationL().setPosition(0.7);
            robot.getServoFoundationR().setPosition(0.7);
        }else if (hold == 1) {
            robot.getServoFoundationL().setPosition(1);
            robot.getServoFoundationR().setPosition(1);
        }
        return true;
    }
}

