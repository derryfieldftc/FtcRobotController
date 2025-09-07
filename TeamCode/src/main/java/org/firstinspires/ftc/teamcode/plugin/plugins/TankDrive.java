package org.firstinspires.ftc.teamcode.plugin.plugins;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.plugin.RobotPlugin;

public class TankDrive extends RobotPlugin {

    Telemetry telemetry;
    HardwareMap hardwareMap;
    Gamepad gamepad1;
    DcMotor motorFR;
    DcMotor motorBR;
    DcMotor motorFL;
    DcMotor motorBL;

    public TankDrive(OpMode opMode) {
        this.hardwareMap = opMode.hardwareMap;
        this.gamepad1 = opMode.gamepad1;
        this.telemetry = opMode.telemetry;
    }

    @Override
    public void init() {
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorBL = hardwareMap.dcMotor.get("motorBL");

        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void loop() {
        double left = -gamepad1.left_stick_y;
        double right = -gamepad1.right_stick_y;
        motorFL.setPower(left);
        motorBL.setPower(left);
        motorFR.setPower(right);
        motorBR.setPower(right);
    }
}
