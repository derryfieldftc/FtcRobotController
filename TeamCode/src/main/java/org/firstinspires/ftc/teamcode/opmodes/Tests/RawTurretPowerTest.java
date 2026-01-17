package org.firstinspires.ftc.teamcode.opmodes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class RawTurretPowerTest extends OpMode {
	DcMotor motor;

	@Override
	public void init() {
		motor = hardwareMap.dcMotor.get("spinny0");
		motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
	}

	@Override
	public void loop() {
		motor.setPower(-gamepad1.right_stick_y);
		telemetry.addData("power", motor.getPower());
		telemetry.update();
	}
}
