package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.GamepadManager;

@TeleOp
public class LiftTest extends OpMode {
	DcMotor lift;
	Servo slift; // d.5 u.1
	int target;
	double power;
	GamepadManager mgamepad1;

	@Override
	public void init() {
		mgamepad1 = new GamepadManager(gamepad1);
		slift = hardwareMap.servo.get("slift");
		if (false) {
			lift = hardwareMap.dcMotor.get("lift");
			lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
			lift.setTargetPosition(0);
			lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
			lift.setPower(0);
			lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		}
	}

	@Override
	public void loop() {
		mgamepad1.poll();
		telemetry.update();
		double angle =(-gamepad1.left_stick_y + 1) / 2;
		slift.setPosition(angle);
		telemetry.addData("angle", angle);

		if (false) {
			if (gamepad1.b) {
				power = 0;
			}

			if (gamepad1.a)
				target = 0;
			if (gamepad1.x)
				target = 60;

			if (mgamepad1.justPressed(GamepadManager.Button.DPAD_UP))
				power += .01;

			if (mgamepad1.justPressed(GamepadManager.Button.DPAD_DOWN))
				power -= .01;

			telemetry.addData("target", target);
			telemetry.addData("power", power);

			telemetry.addData("motorPos", lift.getCurrentPosition());
			if (lift.getCurrentPosition() < target)
				lift.setPower(power);
			if (lift.getCurrentPosition() > target)
				lift.setPower(-power);
		}
	}
}
