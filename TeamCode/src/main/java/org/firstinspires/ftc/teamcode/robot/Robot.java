package org.firstinspires.ftc.teamcode.robot;

import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.skystone.StoneDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
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
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

public class Robot {
    private static final int TICK_PER_REVOLUTION = 537;

    private static final double WHEEL_RADIUS_DRIVE = 0.045;
    private static final double WHEEL_CIRCUMFERENCE_DRIVE = (2 * Math.PI * WHEEL_RADIUS_DRIVE);
    public static final double TICK_PER_METER_DRIVE = (1 / WHEEL_CIRCUMFERENCE_DRIVE) * TICK_PER_REVOLUTION;

    private static final double WHEEL_RADIUS_LIFT = 0.01325;
    private static final double WHEEL_CIRCUMFERENCE_LIFT = (2 * Math.PI * WHEEL_RADIUS_LIFT);
    public static final double TICK_PER_METER_LIFT = (1 / WHEEL_CIRCUMFERENCE_LIFT) * TICK_PER_REVOLUTION;

    private static final int DEGREES_PER_REVOLUTION_HEX = 360;
    public static final double DEGREES_PER_HEX = (DEGREES_PER_REVOLUTION_HEX * 2.13);

    private DcMotor leftF = null;
    private DcMotor leftB = null;
    private DcMotor rightF = null;
    private DcMotor rightB = null;

    private Servo servoFoundationL = null;
    private Servo servoFoundationR = null;

    private Servo lazySusan = null;
    private Servo grabServo = null;

    private DcMotor liftMotor = null;
    private DcMotor hexMotor = null;

    private DcMotor collectL = null;
    private DcMotor collectR = null;

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

    private StoneDetector detector;

    private OpenCvCamera phoneCam;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        leftF = hardwareMap.get(DcMotor.class, "leftA");
        leftB = hardwareMap.get(DcMotor.class, "leftB");
        rightF = hardwareMap.get(DcMotor.class, "rightA");
        rightB = hardwareMap.get(DcMotor.class, "rightB");

        servoFoundationL = hardwareMap.get(Servo.class, " Left foundation Servo");
        servoFoundationR = hardwareMap.get(Servo.class, " Right foundation Servo");
        lazySusan = hardwareMap.get(Servo.class, "lazy Susan");

        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        hexMotor = hardwareMap.get(DcMotor.class, "hexMotor");

        collectL = hardwareMap.get(DcMotor.class, "leftCollect");
        collectR = hardwareMap.get(DcMotor.class, "RightCollect");


        turnPID = new PID(0.067, 0, 0.001);
//        goodPID = new PID(0.067, 0, 0.001);
        drivePID = new PID(1, 0.2, 0.05);
//        goodPID = new PID(1, 0.2, 0.05);


//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);


        // OR...  Do Not Activate the Camera Monitor View
        //phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);

        /*
         * Open the connection to the camera device
         */
//        phoneCam.openCameraDevice();


//        detector = new StoneDetector(); // Create detector
//        phoneCam.setPipeline(detector);
//        detector.useDefaults(); // Set detector to use default settings

        // Optional tuning

//        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
//        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
//        detector.maxAreaScorer.weight = 0.005; //
//
//        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

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

        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        hexMotor.setDirection(DcMotor.Direction.FORWARD);

        collectL.setDirection(DcMotor.Direction.REVERSE);
        collectR.setDirection(DcMotor.Direction.FORWARD );

        servoFoundationL.setDirection(Servo.Direction.FORWARD);
        servoFoundationR.setDirection(Servo.Direction.FORWARD);

        lazySusan.setDirection(Servo.Direction.FORWARD);
        grabServo.setDirection(Servo.Direction.FORWARD);

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

    public DcMotor getCollectL() {
        return collectL;
    }

    public DcMotor getCollectR() {
        return collectR;
    }

    public DcMotor getLiftMotor(){
        return liftMotor;
    }

    public DcMotor getHexMotor(){
        return hexMotor;
    }

    public Servo getServoFoundationL(){
        return servoFoundationL;
    }

    public Servo getServoFoundationR(){
        return servoFoundationR;
    }

    public Servo getGrabServo(){
        return grabServo;
    }

    public Servo getLazySusan(){
        return lazySusan;
    }

    public VuforiaTrackables getTrackables() {
        return trackables;
    }

    public VuforiaTrackable getTemplate() {
        return template;
    }

    public StoneDetector getDetector() {
        return detector;
    }

    public HardwareMap getHardwareMap() {
        return hardwareMap;
    }
}