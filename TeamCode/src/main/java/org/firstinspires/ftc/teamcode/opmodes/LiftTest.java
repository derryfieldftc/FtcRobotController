package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.GamepadManager;

@TeleOp
public class LiftTest extends OpMode {
	DcMotor lift;
	int target;
	double power;
	GamepadManager mgamepad1;

	@Override
	public void init() {
		lift = hardwareMap.dcMotor.get("lift");
		lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		lift.setTargetPosition(0);
		lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		mgamepad1 = new GamepadManager(gamepad1);
		lift.setPower(0);
		lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
	}

	@Override
	public void loop() {
		mgamepad1.poll();
		telemetry.update();
		if (gamepad1.b) {
			power = 0;
		}

		if (mgamepad1.justPressed(GamepadManager.Button.DPAD_UP))
			power += .01;

		if (mgamepad1.justPressed(GamepadManager.Button.DPAD_DOWN))
			power -= .01;

		telemetry.addData("target", target);
		telemetry.addData("power", power);

		telemetry.addData("motorPos", lift.getCurrentPosition());
		lift.setPower(power);
	}
}
