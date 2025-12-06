package org.firstinspires.ftc.teamcode.opmodes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TurretMotorTest")
@Disabled
public class TurretMotorTest extends OpMode {
	DcMotor spinner1, spinner0;

	@Override
	public void init() {
		spinner0 = hardwareMap.dcMotor.get("spinny0");
		spinner1 = hardwareMap.dcMotor.get("spinny1");

		spinner0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		spinner1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		spinner0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		spinner1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		spinner0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		spinner1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

	}

	@Override
	public void loop() {
		spinner0.setPower(gamepad1.right_stick_y);
		spinner1.setPower(gamepad1.left_stick_y);

		telemetry.addData("0", spinner0.getCurrentPosition());
		telemetry.addData("1", spinner1.getCurrentPosition());
		telemetry.update();

	}
}
