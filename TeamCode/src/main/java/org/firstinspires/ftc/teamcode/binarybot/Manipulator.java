package org.firstinspires.ftc.teamcode.binarybot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Manipulator {
    // constants.
    // ******************************************************************
    // slide-related constants
    // ******************************************************************
    // motor power
    public static final double SLIDE_POWER = 0.95;

    // slide motor positions.
    public static final int MAX_SLIDE_EXTENDED_POSITION = 4700;
    public static final int MIN_SLIDE_RETRACTED_POSITION = 0;
    public static final int SLIDE_EXTENDED_POSITION = 4250;
    public static final int SLIDE_RETRACTED_POSITION = 0;
    public static int SLIDE_TRANSFER_POSITION = 400;

    // step size for adjusting position.
    public static final int SLIDE_DELTA = 10;

    // ******************************************************************
    // shoulder-related constants
    // ******************************************************************
    // shoulder motor power
    public static final double SHOULDER_POWER = 0.75;

    // shoulder motor positions
    public static int SHOULDER_ELBOW_BOUNDARY = 4200;
    public static int SHOULDER_CLEAR_BAR = 200;

    public static final int MAX_SHOULDER_POSITION = 6150;
    public static final int MIN_SHOULDER_POSITION = 0;
    public static int SHOULDER_TILT_BOUNDARY = 4200;
    public static int SHOULDER_TRANSFER = 2600;
    public static int SHOULDER_AFTER_TRANSFER = 2900;

    // step size for adjusting positioN
    public static final int SHOULDER_DELTA = 10;

    // ******************************************************************
    // bucket-related constants
    // ******************************************************************
    public static final double BUCKET_NEUTRAL_POSITION = 0.55;

    // ******************************************************************
    // elbow-related constants
    // ******************************************************************
    public static double ELBOW_DEPLOYED = 1.0;
    public static double ELBOW_TRANSFER = 0.85;
    public static double ELBOW_CLEAR_BAR = 0.55;
    public static double ELBOW_RETRACTED = 0.0;

    // ******************************************************************
    // tilt-related constants
    // ******************************************************************
    public static double TILT_DEPLOYED = 0;
    public static double TILT_RETRACTED = 0.75;

    // ******************************************************************
    // claw-related constants
    // ******************************************************************
    public static double CLAW_OPENED = 1;
    public static double CLAW_CLOSED = 0.77;

    // ******************************************************************
    // wrist-related constants
    // ******************************************************************
    public static double WRIST_ROTATED_POSITION = 0.75;
    public static double WRIST_UNROTATED_POSITION = 0;

    // ******************************************************************
    // timing-related constants
    // ******************************************************************

    // time (msec) before moving the shoulder after the block has been released.
    public static int TRANSFER_DELAY = 500;

    // ******************************************************************
    // private member variables.
    // ******************************************************************
    public DcMotor shoulder;
    public DcMotor slide;
    private Servo claw;
    private Servo bucket;
    private Servo wrist;
    public Servo tilt;
    public Servo elbow;

    private OpMode opMode;
    private HardwareMap hardwareMap;

    // state variables.
    private boolean  clawOpen = false;

    // construction
    public Manipulator(HardwareMap hardwareMap, OpMode opMode) {
        this.opMode = opMode;
        this.hardwareMap = hardwareMap;

        // get references to the manipulator hardware.
        shoulder = hardwareMap.dcMotor.get("shoulder");
        slide = hardwareMap.dcMotor.get("slide");
        claw = hardwareMap.servo.get("claw");
        wrist = hardwareMap.servo.get("wrist");
        tilt = hardwareMap.servo.get("tilt");
        elbow = hardwareMap.servo.get("elbow");

        // set zero power brake mode for motors.
        shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // reverse the shoulder motor so positive position corresponds to deployed arm.
        shoulder.setDirection(DcMotorSimple.Direction.REVERSE);

        // zero encoder.
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // put into run to position mode.
        shoulder.setTargetPosition(shoulder.getCurrentPosition());
        shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoulder.setPower(SHOULDER_POWER);

        // reverse the slide motor so positive position corresponds to elevated slide.
        slide.setDirection(DcMotorSimple.Direction.REVERSE);

        // TODO: team should develop proper calibration procedure.
        // slide must be pushed all of the way down when initialized.
        resetSlide();

        // bucket.
        bucket = hardwareMap.servo.get("bucket");

        // test servo.
        bucket.setPosition(BUCKET_NEUTRAL_POSITION);

        // retract elbow.
        elbow.setPosition(ELBOW_RETRACTED);

        // retract tilt
        tilt.setPosition(TILT_RETRACTED);
    }

    // ******************************************************************
    // private/helper methods
    // ******************************************************************
    // reset the slide encoder to zero.
    private void resetSlide() {
        // reset encoder to zero.
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // put into run to position mode.
        slide.setTargetPosition(slide.getCurrentPosition());
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(SLIDE_POWER);
    }

    // update the tilt position depending on current shoulder position.
    private void updateTilt() {
        // the elbow position depends on shoulder position.
        int pos = shoulder.getCurrentPosition();

        if (pos > SHOULDER_TILT_BOUNDARY) {
            tilt.setPosition(TILT_DEPLOYED);
        } else {
            tilt.setPosition(TILT_RETRACTED);
        }
    }

    public void tiltBucket (float input) {
        bucket.setPosition(input*0.45 + BUCKET_NEUTRAL_POSITION);
    }

    public void updateElbow() {
        // the elbow position depends on shoulder position.
        int pos = shoulder.getCurrentPosition();


        if (pos > SHOULDER_ELBOW_BOUNDARY) {
            elbow.setPosition(ELBOW_DEPLOYED);
        } else if (pos > SHOULDER_CLEAR_BAR) {
            elbow.setPosition(ELBOW_CLEAR_BAR);
        } else {
            elbow.setPosition(ELBOW_RETRACTED);
        }
    }

    public void transfer() {
        // make sure shoulder is not too close to the slide.
        if (shoulder.getCurrentPosition() < SHOULDER_TRANSFER - 25) {
            return;
        }

        // lower slide.
        slide.setTargetPosition(SLIDE_TRANSFER_POSITION);

        // tilt claw.
        tilt.setPosition(TILT_RETRACTED);

        // make sure elbow extended.
        elbow.setPosition(ELBOW_DEPLOYED);

        // move wrist to unrotated position.
        unrotateWrist();

        // delay until slide is in place
        LinearOpMode linear_op_mode = (LinearOpMode)opMode;
        long sleepTime = 40;
        while (slide.isBusy()) {
            if (linear_op_mode.opModeIsActive() == false) {
                return;
            }
            linear_op_mode.sleep(sleepTime);
        }

        // move shoulder.
        shoulder.setTargetPosition(SHOULDER_TRANSFER);

        // wait until shoulder is in place
        int shoulderPos = shoulder.getCurrentPosition();
        while (shoulderPos > SHOULDER_TRANSFER) {
            shoulderPos = shoulder.getCurrentPosition();
            if (linear_op_mode.opModeIsActive() == false) {
                // quit.
                return;
            }
            opMode.telemetry.addData("shoulder curr pos", shoulderPos);
            opMode.telemetry.addData("shoulder tgt pos", shoulder.getTargetPosition());
            opMode.telemetry.update();
            linear_op_mode.sleep(sleepTime);
        }

        opMode.telemetry.addData("shoulder curr pos", shoulderPos);
        opMode.telemetry.addData("shoulder tgt pos", shoulder.getTargetPosition());
        opMode.telemetry.update();
        linear_op_mode.sleep(sleepTime);

        // release element.
        openClaw();

        // move shoulder out of the way.
        linear_op_mode.sleep(TRANSFER_DELAY);
        shoulder.setTargetPosition(SHOULDER_AFTER_TRANSFER);
    }

    public void toggleClaw() {
        claw.setPosition((clawOpen) ? CLAW_CLOSED : CLAW_OPENED);
        clawOpen = !clawOpen;
    }

    public void openClaw() {
        clawOpen = true;
        claw.setPosition(CLAW_OPENED);
    }

    public void closeClaw() {
        clawOpen = false;
        claw.setPosition(CLAW_CLOSED);
    }

    boolean wristRotated = true;
    public void toggleWrist() {
        wrist.setPosition((wristRotated) ? WRIST_ROTATED_POSITION : WRIST_UNROTATED_POSITION);
        wristRotated = !wristRotated;
    }

    public void rotateWrist() {
        wrist.setPosition(WRIST_ROTATED_POSITION);
        wristRotated = true;
    }

    public void unrotateWrist() {
        wrist.setPosition(WRIST_UNROTATED_POSITION);
        wristRotated = false;
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
        // get current target position.
        int pos = shoulder.getTargetPosition();

        if (Math.abs(input) < 0.25) {
            // dead zone. ignore input.
            return;
        }

        if (input >= 0.75) {
            pos = pos + 3 * SHOULDER_DELTA;
        } else if (input >= 0.5) {
            pos = pos + 2 * SHOULDER_DELTA;
        } else if (input >= 0.25) {
            pos = pos + SHOULDER_DELTA;
        } else if (input <= -0.75) {
            pos = pos - 3 * SHOULDER_DELTA;
        } else if (input <= -0.5) {
            pos = pos - 2 * SHOULDER_DELTA;
        } else if (input <= -0.25) {
            pos = pos - SHOULDER_DELTA;
        }

        // don't exceed limits.
        if (pos > MAX_SHOULDER_POSITION) {
            pos = MAX_SHOULDER_POSITION;
        } else if (pos < MIN_SHOULDER_POSITION) {
            pos = MIN_SHOULDER_POSITION;
        }

        // adjust target position.
        shoulder.setTargetPosition(pos);

        // automatically update elbow (based on shoulder position).
        updateElbow();

        // automatically update tilt (based on shoulder position).
        updateTilt();
    }

    public void tiltLeft() {
        tilt.setPosition(.8);
    }

    public void tiltRight() {
        tilt.setPosition(1);
    }
}