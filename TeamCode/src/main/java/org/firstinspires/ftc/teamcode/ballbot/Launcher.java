package org.firstinspires.ftc.teamcode.ballbot;

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

    private final float TILT_MIN = 0;
    private final float TILT_MAX = 0.5f;

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

    public void tilt(float tgtPosition) {
        tilt.setPosition(tgtPosition);
    }
}
