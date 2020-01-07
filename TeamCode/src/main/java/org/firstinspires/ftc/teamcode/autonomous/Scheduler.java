package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.robot.Robot;

import java.util.ArrayList;
import java.util.Arrays;

public class Scheduler {

    private Robot robot;
    private ArrayList<Action> actions = new ArrayList<>();
    private boolean isSetup = false;

    public Scheduler(Robot robot, Action[] actions) {
        this.robot = robot;
        this.actions.add(null);
        this.actions.addAll(Arrays.asList(actions));
    }

    public void loop() {
        if (actions.size() > 0) {
            if (actions.get(0) != null) {
                robot.print("Action", actions.get(0).getClass().getSimpleName());
                if (!isSetup) {
                    actions.get(0).setup(robot);
                    isSetup = true;
                }
                if (actions.get(0).loop()) {
                    actions.remove(0);
                    isSetup = false;
                }
            } else {
                actions.remove(0);
                isSetup = false;
            }
        }
    }
}