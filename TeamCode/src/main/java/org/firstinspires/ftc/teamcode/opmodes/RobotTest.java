package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GamepadManager;
import org.firstinspires.ftc.teamcode.plugin.plugins.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.Robot;

@TeleOp(name = "RobotTest")
public class RobotTest extends OpMode {
	Robot bot;
	MecanumDrive mecanumDrive;
	GamepadManager mgamepad;
	boolean handsUp = false;
	boolean shootRight, shootHands, shootLeft;
	boolean leftPalmOpen = false, rightPalmOpen = false;

	@Override
	public void init() {
		bot = new Robot(this).enableIntake().enableTurret();
		mecanumDrive = new MecanumDrive(this);
		mecanumDrive.init();
		bot.init();
//		bot.turret.useGamepad();


		mgamepad = new GamepadManager(gamepad2);
	}

	@Override
	public void loop() {
		// enable or disable parts of the robot, this does not fully shut parts off, intake.setSpeed() will still change the motor speed, this is an okay sacrifice as imo disabling parts like this are niche
		if (gamepad2.dpad_up)
			bot.turretEnabled = !bot.turretEnabled;
		if (gamepad2.dpad_right)
			bot.intakeEnabled = !bot.intakeEnabled;

		mecanumDrive.loop();
		bot.loop();

		bot.intake.setSpeed(gamepad2.right_trigger * ((gamepad2.y) ? -1 : 1));

		if (mgamepad.justPressed(GamepadManager.Button.X)) {
			handsUp = !handsUp;
		}

		if (gamepad1.a) {
			shootHands = true;
		}

		if (gamepad1.b) {
			shootRight = true;
		}

		if (gamepad1.x) {
			shootLeft = true;
		}

		bot.intake.setHeight(gamepad1.right_trigger);

		bot.turret.setSpeed(-gamepad2.left_stick_y);

		if (mgamepad.justPressed(GamepadManager.Button.RIGHT_BUMPER)) {
			rightPalmOpen = !rightPalmOpen;
		}

		if (mgamepad.justPressed(GamepadManager.Button.LEFT_BUMPER)) {
			leftPalmOpen = !leftPalmOpen;
		}

		telemetry.update();
		mgamepad.poll();
	}
}
