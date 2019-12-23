package org.firstinspires.ftc.teamcode.robot.actions;


import org.firstinspires.ftc.teamcode.autonomous.Action;
import org.firstinspires.ftc.teamcode.robot.Robot;

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
        if (on){
            while (robot.getColorSensorS().alpha() <= 9){
                new AutoDrive(0);
            }
        }else {
            return robot.getColorSensorS().alpha() <= 9;
        }
        return robot.getColorSensorS().alpha() >= 9;
    }
}