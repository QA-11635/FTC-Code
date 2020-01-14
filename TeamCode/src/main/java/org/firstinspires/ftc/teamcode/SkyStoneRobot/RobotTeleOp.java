package org.firstinspires.ftc.teamcode.SkyStoneRobot;

import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.android.dx.util.Hex;
import org.firstinspires.ftc.teamcode.PID;

import  com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.hardware.bosch.BNO055IMU;

@TeleOp(name = "RobotTeleOp", group = "Iterative Opmode")
public class RobotTeleOp extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftA = null;
    private DcMotor leftB = null;
    private DcMotor rightA = null;
    private DcMotor rightB = null;

//    private Servo servoFoundationL = null;
//    private Servo servoFoundationR = null;

    private Servo lazySusan = null;
    private Servo grabServo = null;

    private DcMotor liftMotor = null;
    private DcMotor hexMotor = null;

    private DcMotor collectL = null;
    private DcMotor collectR = null;

    private TouchSensor touchSensorC = null;
//    private TouchSensor touchSensorLmin = null;
//    private TouchSensor touchSensorLmax = null;
//    private TouchSensor touchSensorHmin = null;
    private ColorSensor colorSensor = null;

    @Override
    public void init() {
        telemetry.addData("Status", "Quack Attack!!!");

        leftA = hardwareMap.get(DcMotor.class, "leftA");
        leftB = hardwareMap.get(DcMotor.class, "leftB");
        rightA = hardwareMap.get(DcMotor.class, "rightA");
        rightB = hardwareMap.get(DcMotor.class, "rightB");

//        servoFoundationL = hardwareMap.get(Servo.class, " LeftFServo");
//        servoFoundationR = hardwareMap.get(Servo.class, " RightFServo");

        lazySusan = hardwareMap.get(Servo.class, "lazySusan");
        grabServo = hardwareMap.get(Servo.class, "grabServo");

        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        hexMotor = hardwareMap.get(DcMotor.class, "hexMotor");

        collectL = hardwareMap.get(DcMotor.class, "leftCollect");
        collectR = hardwareMap.get(DcMotor.class, "rightCollect");

        touchSensorC = hardwareMap.get(TouchSensor.class, "touchCollect");

        colorSensor = hardwareMap.get(ColorSensor.class,"colorSensor");

//        touchSensorLmin = hardwareMap.get(TouchSensor.class, "touchLiftMin");
//        touchSensorLmax = hardwareMap.get(TouchSensor.class, "touchLiftMax");
//
//        touchSensorHmin = hardwareMap.get(TouchSensor.class, "touchHexMin");

        leftA.setDirection(DcMotor.Direction.FORWARD);
        leftB.setDirection(DcMotor.Direction.FORWARD);
        rightA.setDirection(DcMotor.Direction.REVERSE);
        rightB.setDirection(DcMotor.Direction.REVERSE);

        liftMotor.setDirection(DcMotor.Direction.REVERSE);
        hexMotor.setDirection(DcMotor.Direction.FORWARD);

        collectL.setDirection(DcMotor.Direction.REVERSE);
        collectR.setDirection(DcMotor.Direction.FORWARD);

//        servoFoundationL.setDirection(Servo.Direction.FORWARD);
//        servoFoundationR.setDirection(Servo.Direction.FORWARD);

        lazySusan.setDirection(Servo.Direction.FORWARD);
        grabServo.setDirection(Servo.Direction.FORWARD);

        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hexMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {

        double leftPower;
        double rightPower;

        double liftPower;
        double hexPower;

        double drive = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        telemetry.addData("Joystick", "Drive: " + drive + " Turn: " + turn);

        turn /= 2;

        leftPower = Range.clip(drive + turn, -1.0, 1.0);
        rightPower = Range.clip(drive - turn, -1.0, 1.0);

        leftPower /=2;
        rightPower /=2;

        double lift = -gamepad2.left_stick_y;
        double hex = -gamepad2.right_stick_y;
        liftPower = Range.clip(lift + 0, -1.0, 1.0);
        hexPower = Range.clip(hex + 0, -1.0, 1.0);

        liftPower *= 0.8;
        hexPower *= 0.7;

        if (-gamepad2.right_stick_y == 0){
            hexMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        if (-gamepad2.left_stick_y == 0){
            hexMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

//       Driver Controller:
        leftA.setPower(leftPower);
        leftB.setPower(leftPower);
        rightA.setPower(rightPower);
        rightB.setPower(rightPower);

//       Operator controller:

        //elevator
        liftMotor.setPower(liftPower);
        hexMotor.setPower(hexPower);

//        //foundation servo
//        if (gamepad2.dpad_up) {
//            servoFoundationL.setPosition(0);
//            servoFoundationR.setPosition(0);
//        } else if (gamepad2.dpad_down) {
//            servoFoundationL.setPosition(0.5);
//            servoFoundationR.setPosition(0.5);
//        }

//        //collection
        if (gamepad2.a) {
            collectL.setPower(0.75);
            collectR.setPower(0.75);
        } else if (gamepad2.b) {
            collectL.setPower(0);
            collectR.setPower(0);
        }   else if (gamepad2.dpad_left){
            collectL.setPower(-1);
            collectR.setPower(-1);
        }
        if (touchSensorC.isPressed()){
                collectL.setPower(0);
                collectR.setPower(0);
        }


//        //spin
        if (gamepad2.left_bumper) {
            lazySusan.setPosition(1);
        } else if (gamepad2.right_bumper) {
            lazySusan.setPosition(0.45);
        }

        //grab
        if (gamepad2.x) {
            grabServo.setPosition(1);
        } else if (gamepad2.y) {
            grabServo.setPosition(0);
        }

//        telemetry.addData("touch", "touch:" + touchSensorC.isPressed());
//
//        telemetry.addData("color", "color alpha:" + colorSensor.alpha());
//        telemetry.addData("red","red:" + colorSensor.red());
//        telemetry.addData("blue","blue:" + colorSensor.blue());
//        telemetry.addData("green","green:" + colorSensor.green());
//
//        telemetry.addData("encoder", leftA.getCurrentPosition());



        //grab triggers
//        double right_grab = -gamepad2.right_trigger;
//        double left_grab = gamepad2.left_trigger;
//
//        double leftTPower = Range.clip(right_grab + left_grab, -1.0, 1.0);
//        double rightTPower = Range.clip(right_grab - left_grab, -1.0, 1.0);
//
//        grabServo.setPosition(leftTPower);
//        grabServo.setPosition(rightTPower);

//        if (gamepad2.right_trigger > 0){
//            grabServo.setPosition(right_grab);
//        }
//        if(gamepad2.x){
//            grabServo.setPosition(1);
//        }
//        if (gamepad2.left_trigger < 0){
//            grabServo.setPosition(left_grab);
//
//        }
//        if (gamepad2.y){
//            grabServo.setPosition(0);
//        }
//       Switches:

//        grabbing the cube automatic
//        if (touchSensorC.isPressed()){
//            liftMotor.setPower(-1);
//            if (touchSensorLmin.isPressed()){
//                liftMotor.setPower(0);
//
//            }
//            hexMotor.setPower(-1);
//            if (touchSensorHmin.isPressed()) {
//                hexMotor.setPower(0);
//            }
//            lazySusan.setPosition(0);
//            grabServo.setPosition(0);
//            grabServo.setPosition(0.4);
        }

        //turning off collection
//        if (touchSensorC.isPressed()){
//            collectL.setPower(0);
//            collectR.setPower(0);
//        }
//
//        //turning off lift
//        if (touchSensorLmin.isPressed()){
//            liftMotor.setPower(0);
//        }
//        if (touchSensorLmax.isPressed()){
//            liftMotor.setPower(0);
//        }
//
//        //turning off hex
//        if (touchSensorHmin.isPressed()){
//            hexMotor.setPower(0);
//        }
//    }

    @Override
    public void stop() {
        }
}
