package org.firstinspires.ftc.teamcode.opmodes;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GamepadManager;
import org.firstinspires.ftc.teamcode.plugin.plugins.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Tag;

@TeleOp(name = "RobotTest")
public class RobotTest extends OpMode {
	Robot bot;
	MecanumDrive mecanumDrive;
	GamepadManager mgamepad;

	@Override
	public void init() {
		bot = new Robot(this).enableIntake().enableTurret().enableCamera();
		mecanumDrive = new MecanumDrive(this);
		mecanumDrive.init();
		bot.init();
		bot.camera.setTargetTag(Tag.PGP);
		bot.turret.useCamera();

		mgamepad = new GamepadManager(gamepad1);
	}

	@Override
	public void loop() {
		mecanumDrive.loop();
		bot.loop();

		bot.camera.telemetry();
		bot.intake.setSpeed(gamepad2.right_trigger);

		if (mgamepad.justPressed(GamepadManager.Button.A)) {
			bot.turretEnabled = !bot.turretEnabled;
		}

		telemetry.update();
		mgamepad.poll();
	}
}
