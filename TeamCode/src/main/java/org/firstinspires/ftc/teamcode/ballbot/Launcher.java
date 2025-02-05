package org.firstinspires.ftc.teamcode.ballbot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Launcher {
    public DcMotor spinner;
    public DcMotor harvester;

    public Servo tilt;
    public Servo ballRelease;

    private HardwareMap hardwareMap;
    private OpMode opMode;
    //Doubles for tilt
    public final double TILT_MIN = .5;
    public final double TILT_MAX = 0;
    public double currentTilt;
    // Values for releasing ball
    private  final double BALL_RELEASE_CLOSED = .5;
    private final double BALL_RELEASE_OPEN = .25;
    private final long BALL_RELEASE_DELAY = 150;


    // construction.
    public Launcher(HardwareMap hardwareMap, OpMode opMode) {

        this.hardwareMap = hardwareMap;
        this.opMode = opMode;
    }

    public void initHardware() {
        spinner = hardwareMap.get(DcMotor.class, "spinner");
        harvester = hardwareMap.get(DcMotor.class, "harvester");

        tilt = hardwareMap.get(Servo.class, "tilt");
        ballRelease = hardwareMap.get(Servo.class, "ballRelease");
    }

    public void tilt(double tgtPosition) {
        tilt.setPosition(tgtPosition);
        currentTilt = tgtPosition;
    }
    public void releaseBall() {
        LinearOpMode linear_op_mode = (LinearOpMode)opMode;
        ballRelease.setPosition(BALL_RELEASE_OPEN);
        linear_op_mode.sleep(BALL_RELEASE_DELAY);
        ballRelease.setPosition(BALL_RELEASE_CLOSED);
    }

}
