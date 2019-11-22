package org.firstinspires.ftc.teamcode.robot.actions;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.teamcode.autonomous.Action;
import org.firstinspires.ftc.teamcode.robot.Robot;

public class AutoDetect implements Action {

    private Robot robot;

    @Override
    public void setup(Robot robot) {
        this.robot = robot;
    }

    @Override
    public boolean loop() {
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(robot.getTemplate());
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) robot.getTemplate().getListener()).getPose();
            if (pose != null) {
                VectorF trans = pose.getTranslation();
                Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                double tX = trans.get(0);
                double tY = trans.get(1);
                double tZ = trans.get(2);
                double rX = rot.firstAngle;
                double rY = rot.secondAngle;
                double rZ = rot.thirdAngle;
                return true;
            }
        }
        return false;
    }
}
