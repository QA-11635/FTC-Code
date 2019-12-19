package org.firstinspires.ftc.teamcode.robot.actions;


import org.firstinspires.ftc.teamcode.autonomous.Action;
import org.firstinspires.ftc.teamcode.robot.Robot;

public class AutoLine implements Action {

    private Robot robot;
    private boolean work;

    public AutoLine(boolean work) {
        this.work = work;
    }

    @Override
    public void setup(Robot robot) {
        this.robot = robot;
    }
    @Override
    public boolean loop() {
        if (work = true){
            while (robot.getColorSensor().alpha() <= 3){
                new AutoDrive(0);
            }
        }else {
            return work;
        }
        return work;
    }
}