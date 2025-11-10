package org.firstinspires.ftc.teamcode.opmodes;

import static com.pedropathing.math.MathFunctions.clamp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GamepadManager;
import org.firstinspires.ftc.teamcode.robot.HandsOfGod;
import org.firstinspires.ftc.teamcode.robot.Robot;

@TeleOp(name="TurretPowerTest")
public class TurretPowerTest extends OpMode {
	Robot bot;
	double speed;
	double angle;
	boolean handsUp;
	GamepadManager mgamepad2;

	@Override
	public void init() {
		bot = new Robot(this).enableTurret().enableHandsOfGod().enableIntake().enablePalmsOfGod();
		bot.init();
		mgamepad2 = new GamepadManager(gamepad2);
	}

	@Override
	public void loop() {
		bot.loop();
		mgamepad2.poll();

		if (mgamepad2.justPressed(GamepadManager.Button.DPAD_DOWN))
			speed = clamp(speed - .05, -1, 1);
		if (mgamepad2.justPressed(GamepadManager.Button.DPAD_UP))
			speed = clamp(speed + .05, -1, 1);

		if (mgamepad2.justPressed(GamepadManager.Button.DPAD_LEFT))
			angle = clamp(angle - .05, 0, 1);
		if (mgamepad2.justPressed(GamepadManager.Button.DPAD_RIGHT))
			angle = clamp(angle + .05, 0, 1);

		if (mgamepad2.justPressed(GamepadManager.Button.X)) {
			handsUp = !handsUp;
		}

		bot.intake.setSpeed(gamepad2.right_trigger);

		bot.handsOfGod.setPosition((handsUp) ? HandsOfGod.Position.Up : HandsOfGod.Position.Down);

		bot.turret.setSpeed(speed);
		bot.turret.setAngle(angle);

		telemetry.addData("speed", speed);
		telemetry.addData("angle", angle);
		telemetry.update();
	}
}
