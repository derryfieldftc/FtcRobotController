package org.firstinspires.ftc.teamcode.opmodes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Turret;

@TeleOp
public class TurretRotationTest extends OpMode {
	Turret turret;
	int target = 0;

	@Override
	public void init() {
		turret = new Turret(this);
		turret.setRotationPower(.5);
	}

	@Override
	public void loop() {
		turret.rotator.setTargetPosition(target);
		target += (gamepad1.dpad_up ? 1 : gamepad1.dpad_down ? -1 : 0);
		telemetry.addData("target", target);
		telemetry.addData("", turret.rotator.getCurrentPosition());
		telemetry.update();
	}
}
