package org.firstinspires.ftc.teamcode.binarybot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Manipulator {
    // constamnts.
    public static final int SLIDE_EXTENDED_POSITION = -6509;
    public static final int SLIDE_RETRACTED_POSITION = 0;

    public static final double SHOULDER_POWER = 0.75;
    public static final double SLIDE_POWER = 1.0;

    // private member variables.
    private DcMotor shoulder;
    private DcMotor slide;
    private Servo claw;
    private Servo bucket;
    private Servo wrist;
    private Servo tilt;
    private int shouldpos = 0;
    private Servo elbow;

    // construction
    public Manipulator(HardwareMap hardwareMap) {
        // claw
        shoulder = hardwareMap.dcMotor.get("shoulder");
        slide = hardwareMap.dcMotor.get("slide");
        claw = hardwareMap.servo.get("claw");
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

        // bucket.
        bucket = hardwareMap.servo.get("bucket");

        // initialize positions.
        bucketUp();
        halfFold();
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

    public void extendSlide() {
        slide.setTargetPosition(SLIDE_EXTENDED_POSITION);
    }

    public void retractSlide() {
        slide.setTargetPosition(SLIDE_RETRACTED_POSITION);
    }

    public void trimSlide(float input) {
        int slidePos = slide.getTargetPosition();
        slidePos = (int)(slidePos + input * 5);
        slide.setTargetPosition(slidePos);
    }

    public void trimShoulder(float input) {
        int shoulderPos = slide.getTargetPosition();
        shoulderPos = (int)(shoulderPos + input * 5);
        shoulder.setTargetPosition(shoulderPos);
    }

    public void tiltLeft() {
        tilt.setPosition(.8);
    }

    public void tiltRight() {
        tilt.setPosition(1);
    }

    public void activateShoulder() {
        // should this method be named something else???
        shoulder.setPower(SHOULDER_POWER);
        slide.setPower(SLIDE_POWER);
    }

    // DO WE NEED A WAY TO TURN OFF SHOULDER?
}
