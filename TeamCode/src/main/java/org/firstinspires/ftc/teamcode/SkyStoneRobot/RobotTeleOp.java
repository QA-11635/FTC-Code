package org.firstinspires.ftc.teamcode.SkyStoneRobot;

import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
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

    private Servo servoFoundationL = null;
    private Servo servoFoundationR = null;

    private Servo lazySusan = null;
    private Servo grabServo = null;

    private DcMotor liftMotor = null;
    private DcMotor hexMotor = null;

    private DcMotor collectL = null;
    private DcMotor collectR = null;

    private PID leftPID;
    private PID rightPID;

    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, correction;

    @Override
    public void init() {
        telemetry.addData("Status", "Quack Attack!!!");

        leftA = hardwareMap.get(DcMotor.class, "leftA");
        leftB = hardwareMap.get(DcMotor.class, "leftB");
        rightA = hardwareMap.get(DcMotor.class, "rightA");
        rightB = hardwareMap.get(DcMotor.class, "rightB");

        servoFoundationL = hardwareMap.get(Servo.class, " Left foundation Servo");
        servoFoundationR = hardwareMap.get(Servo.class, " Right foundation Servo");

        lazySusan = hardwareMap.get(Servo.class, "lazy Susan");
        grabServo = hardwareMap.get(Servo.class, "grab servo");

        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        hexMotor = hardwareMap.get(DcMotor.class, "hexMotor");

        collectL = hardwareMap.get(DcMotor.class, "leftCollect");
        collectR = hardwareMap.get(DcMotor.class, "RightCollect");

        leftPID = new PID(1, 0, 0);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        leftA.setDirection(DcMotor.Direction.FORWARD);
        leftB.setDirection(DcMotor.Direction.FORWARD);
        rightA.setDirection(DcMotor.Direction.REVERSE);
        rightB.setDirection(DcMotor.Direction.REVERSE);

        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        hexMotor.setDirection(DcMotor.Direction.FORWARD);

        collectL.setDirection(DcMotor.Direction.REVERSE);
        collectR.setDirection(DcMotor.Direction.FORWARD );

        servoFoundationL.setDirection(Servo.Direction.FORWARD);
        servoFoundationR.setDirection(Servo.Direction.FORWARD);

        lazySusan.setDirection(Servo.Direction.FORWARD);
        grabServo.setDirection(Servo.Direction.FORWARD);

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

        leftPower /= 2;
        rightPower /= 2;

        double lift = -gamepad2.left_stick_y;
        double hex = gamepad2.right_stick_x;
        liftPower = Range.clip(lift + 0, -1.0, 1.0);
        hexPower = Range.clip(hex + 0, -1.0, 1.0);

        liftPower /= 4;
        hexPower /= 2;

//       Driver Controller:
        leftA.setPower(leftPower);
        leftB.setPower(leftPower);
        rightA.setPower(rightPower);
        rightB.setPower(rightPower);

//       operator controller:

        //elevator
        liftMotor.setPower(liftPower);
        hexMotor.setPower(hexPower);

        //foundation servo
        if (gamepad2.dpad_up) {
            servoFoundationL.setPosition(0);
            servoFoundationR.setPosition(0);
        } else if (gamepad2.dpad_down) {
            servoFoundationL.setPosition(0.35);
            servoFoundationR.setPosition(0.35);
        }

        //collection
        if (gamepad2.a) {
            collectL.setPower(1);
            collectR.setPower(1);
        } else if (gamepad2.b) {
            collectL.setPower(0);
            collectR.setPower(0);
        }

        //grip
        if (gamepad2.left_bumper) {
            lazySusan.setPosition(0);
        } else if (gamepad2.right_bumper) {
            lazySusan.setPosition(0.35);
        }

        //grab
        if (gamepad2.x) {
            grabServo.setPosition(0.4);
        } else if (gamepad2.y) {
            grabServo.setPosition(0);
        }


        //limit switches

    }

    @Override
    public void stop() {
    }

}
