package org.firstinspires.ftc.teamcode.binarybot;

import static androidx.core.math.MathUtils.clamp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class BinaryBot {
    private DcMotor motorFL;
    private DcMotor motorBL;
    private DcMotor motorFR;
    private DcMotor motorBR;

    // claw member variables
    private DcMotor shoulder;
    private DcMotor slide;
    private Servo claw;
    private Servo bucket;
    private Servo wrist;
    private Servo tilt;
    private int shouldpos = 0;
    private Servo elbow;

    public BinaryBot(HardwareMap hardwareMap) {
        initHardware(hardwareMap);
    }
    public void initHardware(HardwareMap hardwareMap) {
        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");

        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);

        // claw
        shoulder = hardwareMap.dcMotor.get("shoulder");
        slide = hardwareMap.dcMotor.get("slide");
        claw = hardwareMap.servo.get("claw");
        bucket = hardwareMap.servo.get("bucket");
        wrist = hardwareMap.servo.get("wrist");
        tilt = hardwareMap.servo.get("tilt");
        elbow = hardwareMap.servo.get("elbow");

        shoulder.setTargetPosition(shoulder.getCurrentPosition());
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoulder.setPower(.75);

        slide.setTargetPosition(slide.getCurrentPosition());
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(1);
    }

    public void drive(double drive,  double strafe, double twist) {
        double powerFL = drive + strafe + twist;
        double powerBL = drive - strafe + twist;
        double powerFR = drive - strafe - twist;
        double powerBR = drive + strafe - twist;

        powerFL = clamp(1, -1, powerFL);
        powerBL = clamp(1, -1, powerBL);
        powerFR = clamp(1, -1, powerFR);
        powerBR = clamp(1, -1, powerBR);

        motorFL.setPower(powerFL);
        motorBL.setPower(powerBL);
        motorFR.setPower(powerFR);
        motorBR.setPower(powerBR);
    }

    public void stop() {
        drive(0, 0, 0);
    }

    public void tuck() {
        bucket.setPosition(.55);
        shoulder.setTargetPosition(0);
        tilt.setPosition(.16);
        wrist.setPosition(0);
        elbow.setPosition(0);
        claw.setPosition(.77);
    }

    public void bucketUp() {
        bucket.setPosition(1);
    }

    public void halfFold() {
        shoulder.setTargetPosition(-2900);
    }

    public void fullFold() {
        shoulder.setTargetPosition(-5800);
        elbow.setPosition(1);
        tilt.setPosition(.75);
        wrist.setPosition(0);
    }

    public void transfer() {
        bucket.setPosition(.55);
        tilt.setPosition(0.25);
        wrist.setPosition(.25);
        shoulder.setTargetPosition(-2325);
    }

    boolean clawOpen = false;
    public void toggleClaw() {
        claw.setPosition((clawOpen) ? .77 : 1);
        clawOpen = !clawOpen;
    }

    boolean wristRotated = false;
    public void toggleWrist() {
        wrist.setPosition((wristRotated) ? 0 : .75);
        wristRotated = !wristRotated;
    }
}
