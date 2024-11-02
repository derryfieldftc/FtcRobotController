package org.firstinspires.ftc.teamcode.plugins;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.GamepadManager;
import org.firstinspires.ftc.teamcode.RobotPlugin;

public class ITDClaw extends RobotPlugin {

    HardwareMap hardwareMap;
    Gamepad gamepad;
    DcMotor pitch, slide;
    Telemetry telemetry;

    public ITDClaw(OpMode opMode, Gamepad gamepad) {
        this.gamepad = gamepad;
        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;
    }

    @Override
    public void init() {
        pitch = hardwareMap.dcMotor.get("pitch");
        pitch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pitch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slide = hardwareMap.dcMotor.get("slide");
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        double slidePower = gamepad.right_stick_y;

        slide.setPower(slidePower);

        telemetry.addData("slidePower", slidePower);
        telemetry.update();
    }

}
