package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GamepadManager;
import org.firstinspires.ftc.teamcode.plugin.plugins.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.Depot;
import org.firstinspires.ftc.teamcode.robot.Field;
import org.firstinspires.ftc.teamcode.robot.HandsOfGod;
import org.firstinspires.ftc.teamcode.robot.PalmsOfGod;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Tag;
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.robot.TurretPose2d;

import java.util.Objects;

@TeleOp(name = "RedOpMode")
public class RedOpMode extends OpMode {
	Robot bot;
	MecanumDrive mecanumDrive;
	org.firstinspires.ftc.teamcode.RR.MecanumDrive rr_Mecanum;
	GamepadManager mgamepad;
	double speedTrim = 0;
	boolean handsUp = false;
	boolean shootRight, shootHands, shootLeft;
	boolean leftPalmOpen = false, rightPalmOpen = false;

	@Override
	public void init() {
		bot = new Robot(this).enableIntake().enableHandsOfGod().enablePalmsOfGod().enableTurret();
		bot.init();
		try {
			rr_Mecanum = new org.firstinspires.ftc.teamcode.RR.MecanumDrive(hardwareMap, Turret.getSavedPosition().pose2d);
			Robot.turret = new Turret(this, Turret.getSavedPosition());
			Robot.turret.init();
		} catch (Exception e) {
			throw new RuntimeException(e);
		}
		Robot.turret.setTarget(new Depot(Field.Alliance.Red).getPosition()).trackTarget().autoTracking(rr_Mecanum).run(null);
		mecanumDrive = new MecanumDrive(this);
		mecanumDrive.init();
		bot.camera.setTargetTag(Tag.PGP);
//		bot.turret.useGamepad();


		mgamepad = new GamepadManager(gamepad2);
	}

	@Override
	public void loop() {
		mecanumDrive.loop();
		rr_Mecanum.updatePoseEstimate();
		bot.loop();
		Robot.turret.autoTracking(rr_Mecanum).run(null);

		bot.intake.setSpeed(gamepad2.right_trigger * ((gamepad2.y) ? -1 : 1));

		if (mgamepad.justPressed(GamepadManager.Button.X)) {
			handsUp = !handsUp;
		}

		if (gamepad2.a) {
			shootHands = true;
		}

		if (gamepad2.b) {
			shootRight = true;
		}

		if (gamepad2.x) {
			shootLeft = true;
		}

		if (shootHands)
			shootHands = bot.shoot(Robot.BallPosition.Hands);

		if (shootLeft)
			shootLeft = bot.shoot(Robot.BallPosition.Left);

		if (shootRight)
			shootRight = bot.shoot(Robot.BallPosition.Right);

		Robot.intake.setHeight(gamepad1.right_trigger);

		Robot.turret.setSpeed(-gamepad2.left_stick_y);

		telemetry.addLine("Ball" + Robot.palmsOfGod.getLeftBall());

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
