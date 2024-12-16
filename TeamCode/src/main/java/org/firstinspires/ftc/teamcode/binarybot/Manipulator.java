package org.firstinspires.ftc.teamcode.binarybot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Manipulator {
    // constamnts.
    public static final int MAX_SLIDE_EXTENDED_POSITION = 4600;
    public static final int MIN_SLIDE_RETRACTED_POSITION = 0;
    public static final int SLIDE_EXTENDED_POSITION = 4250;
    public static final int SLIDE_RETRACTED_POSITION = 0;

    public static final double SHOULDER_POWER = 0.5;
    public static final double SLIDE_POWER = 0.85;

    public static final int SLIDE_DELTA = 10;

    // private member variables.
   // public DcMotor shoulder;
    public DcMotor slide;
    private Servo claw;
    private Servo bucket;
    private Servo wrist;
    private Servo tilt;
    private int shouldpos = 0;
    private Servo elbow;

    // construction
    public Manipulator(HardwareMap hardwareMap) {
        // claw
       // shoulder = hardwareMap.dcMotor.get("shoulder");
        slide = hardwareMap.dcMotor.get("slide");
        claw = hardwareMap.servo.get("claw");
        wrist = hardwareMap.servo.get("wrist");
        tilt = hardwareMap.servo.get("tilt");
        elbow = hardwareMap.servo.get("elbow");

//        shoulder.setTargetPosition(shoulder.getCurrentPosition());
//        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        shoulder.setPower(SHOULDER_POWER);

        // reverse the slide motor so positive position corresponds to elevated slide.
        slide.setDirection(DcMotorSimple.Direction.REVERSE);

        // TODO: team should develop proper calibration procedure.
        // slide must be pushed all of the way down when initialized.
        resetSlide();

        // bucket.
        bucket = hardwareMap.servo.get("bucket");

//        // initialize positions.
//        bucketUp();
//        halfFold();
    }

    // reset the slide encoder to zero.
    private void resetSlide() {
        // reset encoder to zero.
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // put into run to position mode.
        slide.setTargetPosition(slide.getCurrentPosition());
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(SLIDE_POWER);
    }
    public void tuck() {
        bucket.setPosition(.55);
        //shoulder.setTargetPosition(0);
        tilt.setPosition(.16);
        wrist.setPosition(0);
        elbow.setPosition(0);
        claw.setPosition(.77);
    }

    public void bucketUp() {
        bucket.setPosition(1);
    }

    public void halfFold() {
        //shoulder.setTargetPosition(-2900);
    }

    public void fullFold() {
        //shoulder.setTargetPosition(-5800);
        elbow.setPosition(1);
        tilt.setPosition(.75);
        wrist.setPosition(0);
    }

    public void transfer() {
        bucket.setPosition(.55);
        tilt.setPosition(0.25);
        wrist.setPosition(.25);
        //shoulder.setTargetPosition(-2325);
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
        // get current target position.
        int pos = slide.getTargetPosition();

        if (Math.abs(input) < 0.25) {
            // dead zone. ignore input.
            return;
        }

        if (input >= 0.75) {
            pos = pos + 3 * SLIDE_DELTA;
        } else if (input >= 0.5) {
            pos = pos + 2 * SLIDE_DELTA;
        } else if (input >= 0.25) {
            pos = pos + SLIDE_DELTA;
        } else if (input <= -0.75) {
            pos = pos - 3 * SLIDE_DELTA;
        } else if (input <= -0.5) {
            pos = pos - 2 * SLIDE_DELTA;
        } else if (input <= -0.25) {
            pos = pos - SLIDE_DELTA;
        }

        // don't exceed limits.
        if (pos > MAX_SLIDE_EXTENDED_POSITION) {
            pos = MAX_SLIDE_EXTENDED_POSITION;
        } else if (pos < MIN_SLIDE_RETRACTED_POSITION) {
            pos = MIN_SLIDE_RETRACTED_POSITION;
        }

        // adjust target position.
        slide.setTargetPosition(pos);
    }

    public void trimShoulder(float input) {
        int shoulderPos = slide.getTargetPosition();
        shoulderPos = (int)(shoulderPos + input * 5);
        //shoulder.setTargetPosition(shoulderPos);
    }

    public void tiltLeft() {
        tilt.setPosition(.8);
    }

    public void tiltRight() {
        tilt.setPosition(1);
    }

    public void activateShoulder() {
        // should this method be named something else???
        //shoulder.setPower(SHOULDER_POWER);
        slide.setPower(SLIDE_POWER);
    }

    // DO WE NEED A WAY TO TURN OFF SHOULDER?
}
