package org.firstinspires.ftc.teamcode.plugins;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.GamepadManager;
import org.firstinspires.ftc.teamcode.RobotPlugin;
import org.firstinspires.ftc.teamcode.ServoStateMachine;

import java.io.FileWriter;
import java.io.IOException;

public class ITDClaw extends RobotPlugin {

    HardwareMap hardwareMap;
    GamepadManager gamepad;
    DcMotor pitch, slide;
    Telemetry telemetry;
    ServoStateMachine claw;
    ServoStateMachine arm;
    OpMode opMode;

    public ITDClaw(OpMode opMode, Gamepad gamepad) {
        this.gamepad = new GamepadManager(gamepad);
        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;
        this.opMode = opMode;
    }

    @Override
    public void init() {
        claw = new ServoStateMachine.Builder()
                .addServo("claw", Servo::resetDeviceConfigurationForOpMode)
                .addServo("rotate", Servo::resetDeviceConfigurationForOpMode)
                .addServo("clawP", Servo::resetDeviceConfigurationForOpMode)

                .addState("ready", new String[] { "claw", "rotate", "clawP" }, new float[] { 0.8f, 0.0f, 1.0f })
                .addState("grab", new String[] { "claw", "rotate", "clawP" }, new float[] { 0.5f, 0.0f, 0.9f })
                .addState("grabbed", new String[] { "claw", "rotate", "clawP" }, new float[] { 0.5f, 1.0f, 1.0f })
                .build(this.opMode);

        pitch = hardwareMap.dcMotor.get("pitch");
        pitch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pitch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slide = hardwareMap.dcMotor.get("slide");
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        gamepad.poll();
        double pitchPower = gamepad.axis(GamepadManager.Axis.RIGHT_STICK_Y);
        int stateResult = 0;

        if (gamepad.justPressed(GamepadManager.Button.A)) {
            String current = claw.getCurrentState();
            if (current == null) claw.setCurrentState("ready");
            else if (current.equals("ready")) claw.setCurrentState("grab");
            else if (current.equals("grab")) claw.setCurrentState("grabbed");
            else if (current.equals("grabbed")) claw.setCurrentState("ready");
            // else if (current.equals("up")) claw.setCurrentState("ready");
        }

        pitch.setPower(pitchPower);

        telemetry.addData("pitchPower", pitchPower);
        telemetry.addData("state", claw.getCurrentState());
        telemetry.addData("result", stateResult);
        telemetry.update();
    }

}
