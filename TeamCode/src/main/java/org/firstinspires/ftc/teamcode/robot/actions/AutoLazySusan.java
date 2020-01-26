package org.firstinspires.ftc.teamcode.robot.actions;

import org.firstinspires.ftc.teamcode.autonomous.Action;
import org.firstinspires.ftc.teamcode.robot.Robot;


public class AutoLazySusan implements Action {

    private Robot robot;
    private int spin = 0;

    public AutoLazySusan(int spin) {
        this.spin = spin;
    }

    @Override
    public void setup(Robot robot) {
        this.robot = robot;
    }
    @Override
    public boolean loop() {
        if (spin == 1){
            robot.getLazySusan().setPosition(0.45);
        }else if (spin == 0) {
            robot.getLazySusan().setPosition(0.15);
        }
        return true;
    }
}

