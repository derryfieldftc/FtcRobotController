package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GamepadManager;
import org.firstinspires.ftc.teamcode.plugin.plugins.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.Field;
import org.firstinspires.ftc.teamcode.robot.HandsOfGod;
import org.firstinspires.ftc.teamcode.robot.PalmsOfGod;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Tag;

@TeleOp(name = "RobotTest")
public class RobotTest extends OpMode {
	Robot bot;
	MecanumDrive mecanumDrive;
	GamepadManager mgamepad;
	boolean handsUp = false;
	boolean leftPalmOpen = false, rightPalmOpen = false;

	@Override
	public void init() {
		bot = new Robot(this).enableIntake().enableHandsOfGod().enablePalmsOfGod().enableTurret();
		mecanumDrive = new MecanumDrive(this);
		mecanumDrive.init();
		bot.init();
		Robot.turret.useGamepad();
		bot.camera.setTargetTag(Tag.PGP);
//		bot.turret.useGamepad();


		mgamepad = new GamepadManager(gamepad2);
	}

	@Override
	public void loop() {
		// enable or disable parts of the robot
		if (gamepad2.dpad_up)
			Robot.turretEnabled = !Robot.turretEnabled;
		if (gamepad2.dpad_right)
			Robot.intakeEnabled = !Robot.intakeEnabled;
		if (gamepad2.dpad_down)
			Robot.handsOfGodEnabled = !Robot.handsOfGodEnabled;
		if (gamepad2.dpad_left)
			Robot.drivetrainEnabled = !Robot.drivetrainEnabled;

		mecanumDrive.loop();
		bot.loop();

		bot.intake.setSpeed(gamepad2.right_trigger * ((gamepad2.y) ? -1 : 1));

		if (mgamepad.justPressed(GamepadManager.Button.X)) {
			handsUp = !handsUp;
		}

		if (gamepad1.a) {
			while (bot.shoot(Robot.BallPosition.Hands)) {
			}
		}

		if (gamepad1.b) {
			while (bot.shoot(Robot.BallPosition.Right)) {
			}
		}

		if (gamepad1.x) {
			while (bot.shoot(Robot.BallPosition.Left)) {
			}
		}

		Robot.intake.setHeight(-gamepad2.right_stick_y);

		Robot.turret.setAngle(-gamepad2.left_stick_y);
		Robot.turret.setSpeed(gamepad2.left_trigger);

		if (gamepad1.start) {
			while (bot.shootAllTryingMotif().run(null)) {
			}
		}

		telemetry.addData("Ball", Robot.intake.getBallType().toString());

		if (gamepad1.y)
			bot.setBalls(new Field.Ball[]{Field.Ball.Purple, Field.Ball.Green, Field.Ball.Purple});
		if (mgamepad.justPressed(GamepadManager.Button.RIGHT_BUMPER)) {
			rightPalmOpen = !rightPalmOpen;
		}

		if (mgamepad.justPressed(GamepadManager.Button.LEFT_BUMPER)) {
			leftPalmOpen = !leftPalmOpen;
		}

		bot.handsOfGod.setPosition((handsUp) ? HandsOfGod.Position.Up : HandsOfGod.Position.Down);
		bot.palmsOfGod.setLeftPalm((leftPalmOpen) ? PalmsOfGod.Position.Up : PalmsOfGod.Position.Down);
		bot.palmsOfGod.setRightPalm((rightPalmOpen) ? PalmsOfGod.Position.Up : PalmsOfGod.Position.Down);

		bot.palmsOfGod.getLeftBall();
		bot.palmsOfGod.getRightBall();

		telemetry.update();
		mgamepad.poll();
	}
}
