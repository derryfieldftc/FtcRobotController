package org.firstinspires.ftc.teamcode.ballbot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Launcher {
    public DcMotor spinner;
    public DcMotor harvester;
    public DcMotor firstIntake;
    public DcMotor liftMotor;
    public Servo tilt;
    public Servo ballRelease;

    private final HardwareMap hardwareMap;
    private final OpMode opMode;
    //Doubles for tilt
    public final double TILT_MIN = .4;
    public final double TILT_MAX = 0;
    public double currentTilt;
    private double variableTilt = 0.5;
    private final double TILT_INCREMENT = .1;
    // Values for releasing ball
    private  final double BALL_RELEASE_CLOSED = .5;
    private final double BALL_RELEASE_OPEN = .25;
    private final long BALL_RELEASE_DELAY = 90;
    //Variables for the spinner

    private final double SPINNER_INCREMENT = .1;
    private final double SPINNER_MAX_SPEED = 1;
    private double spinnerSpeed = .5;
    public double spinnerTargetSpeed;
    private boolean spinnerOn = false;
    private double liftPower = .4;
    private double intakePower = .5;

    // construction.
    public Launcher(HardwareMap hardwareMap, OpMode opMode) {

        this.hardwareMap = hardwareMap;
        this.opMode = opMode;
    }
    public void resetPositions() {
        tilt.setPosition(TILT_MIN);
        ballRelease.setPosition(BALL_RELEASE_CLOSED);
    }
    public void initHardware() {
        spinner = hardwareMap.get(DcMotor.class, "spinner");
        harvester = hardwareMap.get(DcMotor.class, "harvester");
        firstIntake = hardwareMap.get(DcMotor.class, "firstIntake");
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        tilt = hardwareMap.get(Servo.class, "tilt");
        ballRelease = hardwareMap.get(Servo.class, "ballRelease");
    }
    public void tiltUp() {
        if (currentTilt - TILT_INCREMENT >= TILT_MAX) {
            variableTilt = variableTilt - TILT_INCREMENT;
            tilt.setPosition(variableTilt);
            currentTilt = variableTilt;
        }
    }
    public void tiltDown() {
        if (currentTilt + TILT_INCREMENT <= TILT_MIN) {
            variableTilt = variableTilt + TILT_INCREMENT;
            tilt.setPosition(variableTilt);
            currentTilt = variableTilt;
        }
    }
    public void releaseBall() {
        LinearOpMode linear_op_mode = (LinearOpMode)opMode;
        ballRelease.setPosition(BALL_RELEASE_OPEN);
        linear_op_mode.sleep(BALL_RELEASE_DELAY);
        ballRelease.setPosition(BALL_RELEASE_CLOSED);
    }
    public void speedUp() {
        if (spinnerSpeed < SPINNER_MAX_SPEED - SPINNER_INCREMENT) {
            spinnerTargetSpeed = spinnerSpeed + SPINNER_INCREMENT;
        }

    }
    public void speedDown() {
            spinnerTargetSpeed = spinnerSpeed - SPINNER_INCREMENT;
            if (spinnerTargetSpeed < 0) {
                spinnerTargetSpeed = 0;
            }


    }
    public boolean getSpinnerState() {
        return spinnerOn;
    }
    public double getSpinnerSpeed() {
        return spinnerTargetSpeed;
    }
    public void turnSpinnerOn() {
        spinnerOn = true;
        spinner.setPower(spinnerTargetSpeed);
    }
    public void turnSpinnerOff() {
        spinnerOn = false;
        spinner.setPower(0);
    }
    public void updateSpinner() {
        if (spinnerOn) {
            spinner.setPower(spinnerTargetSpeed);
        } else {
            spinner.setPower(0);
        }
        spinnerSpeed = spinnerTargetSpeed;
    }
    public void intakeOn() {
        harvester.setPower(intakePower);
        firstIntake.setPower(-intakePower);
    }
    public void intakeOff() {
        harvester.setPower(0);
        firstIntake.setPower(0);
    }
    public void toggleIntake() {
        if (harvester.getPower() < .1) {
            intakeOn();
        } else {
            intakeOff();
        }
    }
    public void liftOn() {
        liftMotor.setPower(liftPower);
    }
    public void liftOff() {
        liftMotor.setPower(0);
    }
    public void toggleLift() {
        if (liftMotor.getPower() <.1) {
            liftOn();
        } else {
            liftOff();
        }
    }
    private double noBalls = 8.6;
    private double oneBall = 7.6;
    private double twoBalls = 7.2;
    private double threeBalls = 5.86;
    private double fourBalls = 3.8;
    private double fiveBalls = 2.2;
    public int calculateBalls(double distance) {
        if (distance > noBalls) {
            return 0;
        } else if (distance > oneBall) {
            return 1;
        } else if (distance > twoBalls) {
            return 2;
        } else if (distance > threeBalls) {
            return 3;
        } else if (distance > fourBalls) {
            return  4;
        } else {
            return 5;
        }
    }
}
