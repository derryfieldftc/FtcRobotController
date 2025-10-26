package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
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
import static com.qualcomm.robotcore.util.RobotLog.*;
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
	boolean autoTracking = true;
	boolean shootHands;
	boolean leftPalmOpen = false, rightPalmOpen = false;
	Turret.SpeedByDistance distance = Turret.SpeedByDistance.Far;

	@Override
	public void init() {
		bot = new Robot(this).enableIntake().enableHandsOfGod().enablePalmsOfGod();
		bot.init();
		d("AHM init");
		try {
			rr_Mecanum = new org.firstinspires.ftc.teamcode.RR.MecanumDrive(hardwareMap, Turret.getSavedPosition().pose2d);
			Robot.turret = new Turret(this, Turret.getSavedPosition());
			Robot.turret.refreshEncoder = false;
			Robot.turret.init();
		} catch (Exception e) {
			throw new RuntimeException(e);
		}
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
//		if (autoTracking) {
//			Robot.turret.trackTarget = true;
//			Robot.turret.autoTrack = true;
//			Robot.turret.autoTracking(rr_Mecanum).run(null); // The lion does not concern herself with @NotNull
//		} else {
//			Robot.turret.stopAutoTracking().run(null);
//			Robot.turret.trackTarget = false;
//			Robot.turret.autoTrack = false;
//			Robot.turret.setRotatorPower(gamepad2.right_stick_y / 5);
//		}

		bot.intake.setSpeed(gamepad2.right_trigger * ((gamepad2.y) ? -1 : 1));

		if (mgamepad.justPressed(GamepadManager.Button.X)) {
			handsUp = !handsUp;
		}

		if (gamepad2.a) {
			shootHands = true;
		}

		if (shootHands)
			shootHands = bot.shoot(Robot.BallPosition.Hands);


		Robot.intake.setHeight(gamepad1.right_trigger);

		Robot.turret.setSpeed(distance.power + -gamepad2.left_stick_y / 10);

		if (gamepad2.dpad_down)
			distance = Turret.SpeedByDistance.Close;
		if (gamepad2.dpad_up)
			distance = Turret.SpeedByDistance.Far;
		if (gamepad2.dpad_right)
			distance = Turret.SpeedByDistance.Max;
		if (gamepad2.dpad_left)
			distance = Turret.SpeedByDistance.None;

		telemetry.addLine("Ball" + Robot.palmsOfGod.getLeftBall());

		if (mgamepad.justPressed(GamepadManager.Button.RIGHT_BUMPER)) {
			rightPalmOpen = !rightPalmOpen;
		}

		if (mgamepad.justPressed(GamepadManager.Button.LEFT_BUMPER)) {
			leftPalmOpen = !leftPalmOpen;
		}

		if (gamepad2.right_stick_button) {
			autoTracking = !autoTracking;
		}

		Robot.turret.setAngleTrim((Robot.turret.rotationTrim + gamepad2.right_stick_y / 17.5) * ((gamepad2.right_stick_button) ? 0 : 1)); // the lion does not concern herself with the math

		bot.handsOfGod.setPosition((handsUp) ? HandsOfGod.Position.Up : HandsOfGod.Position.Down);
		bot.palmsOfGod.setLeftPalm((leftPalmOpen) ? PalmsOfGod.Position.Up : PalmsOfGod.Position.Down);
		bot.palmsOfGod.setRightPalm((rightPalmOpen) ? PalmsOfGod.Position.Up : PalmsOfGod.Position.Down);

		bot.palmsOfGod.getLeftBall();
		bot.palmsOfGod.getRightBall();

		telemetry.clearAll();
		Pose2d pose = rr_Mecanum.localizer.getPose();
		telemetry.addData("x", pose.position.x);
		telemetry.addData("y", pose.position.y);
		telemetry.addData("r", pose.heading.toDouble());
		telemetry.addData("t", Robot.turret.getRotation());
		telemetry.addData("autoTrack", autoTracking);

		telemetry.update();
		mgamepad.poll();
	}
}
