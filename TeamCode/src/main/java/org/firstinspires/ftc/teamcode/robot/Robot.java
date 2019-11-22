package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.PID;
import org.firstinspires.ftc.teamcode.autonomous.Action;
import org.firstinspires.ftc.teamcode.autonomous.Scheduler;

public class Robot {
    private static final int TICK_PER_REVOLUTION = 537;
    private static final double WHEEL_RADIUS = 0.05;
    private static final double WHEEL_CIRCUMFERENCE = 2 * Math.PI * WHEEL_RADIUS;
    public static final double TICK_PER_METER = (1 / WHEEL_CIRCUMFERENCE) * TICK_PER_REVOLUTION;

    private DcMotor leftF = null;
    private DcMotor leftB = null;
    private DcMotor rightF = null;
    private DcMotor rightB = null;

    private PID turnPID;
    private PID drivePID;

    private BNO055IMU imu;
    private Orientation lastAngles = new Orientation();
    private double globalAngle, power = 0.30, correction;

    private Scheduler scheduler;

    private Telemetry telemetry;
    private HardwareMap hardwareMap;

    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia;
    private VuforiaTrackables trackables;
    private VuforiaTrackable template;


    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        leftF = hardwareMap.get(DcMotor.class, "leftA");
        leftB = hardwareMap.get(DcMotor.class, "leftB");
        rightF = hardwareMap.get(DcMotor.class, "rightA");
        rightB = hardwareMap.get(DcMotor.class, "rightB");

        turnPID = new PID(0.067, 0, 0.001);
//        goodPID = new PID(0.067, 0, 0.001);
        drivePID = new PID(1, 0, 0);
//        goodPID = new PID(1, 0.2, 0.05);

        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.mode = BNO055IMU.SensorMode.IMU;
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.loggingEnabled = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(imuParameters);

        leftF.setDirection(DcMotor.Direction.FORWARD);
        leftB.setDirection(DcMotor.Direction.FORWARD);
        rightF.setDirection(DcMotor.Direction.REVERSE);
        rightB.setDirection(DcMotor.Direction.REVERSE);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AeNJe/D/////AAABma3zpeBqhUccl7pXtQuCkBhDy8UlUV9/L3rf6F5bdEBfUEunLrchGEFvQcZtw0IdjOgQR4Tjq47JbgzYMX0s9Nt9a/V3eWDZ3KdpEHP4+MBjy+3f96l1z89VIL8UCbfaC1vgnwb5TRrWESErL2KHLJ0sz24w4iSKjKd+gHsAcQbzhOIBeiNeGxR6/M/E8aUyeoQ1AdnyOQLwsbMEnPrJXkLsY+2+jV0Xj1xqWTC2jPMN13ryFqBk/dkq0z/sEgD0DO3ldBObc3ay36a9nEbtvaVN1pPX2YwKVQLHkjsK/Ymgb7RSK5bI1hpIWOu4swFUVOlrkA7cqUEWDdM4U48lXejr1YMwAj9FnZzz/xjxnata";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        trackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        template = trackables.get(0);
    }

    public void runAutonomous(Action[] actions) {
        scheduler = new Scheduler(this, actions);
    }

    public Scheduler getScheduler() {
        return scheduler;
    }

    public Orientation getLastAngles() {
        return lastAngles;
    }

    public void setLastAngles(Orientation lastAngles) {
        this.lastAngles = lastAngles;
    }

    public BNO055IMU getIMU() {
        return imu;
    }

    public Orientation update() {
        return getIMU().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    public void print(String s, String s1) {
        telemetry.addData(s, s1);
    }

    public void print(String s, double d) {
        print(s, "" + d);
    }

    public void drive(double drive, double turn) {
        double leftPower = Range.clip(drive + turn, -1.0, 1.0);
        double rightPower = Range.clip(drive - turn, -1.0, 1.0);

        leftPower /= 2;
        rightPower /= 2;

        leftF.setPower(leftPower);
        leftB.setPower(leftPower);
        rightF.setPower(rightPower);
        rightB.setPower(rightPower);
    }

    public PID getTurnPID() {
        return turnPID;
    }

    public PID getDrivePID() {
        return drivePID;
    }

    public DcMotor getLeftF() {
        return leftF;
    }

    public DcMotor getLeftB() {
        return leftB;
    }

    public DcMotor getRightF() {
        return rightF;
    }

    public DcMotor getRightB() {
        return rightB;
    }

    public VuforiaTrackables getTrackables() {
        return trackables;
    }

    public VuforiaTrackable getTemplate() {
        return template;
    }
}