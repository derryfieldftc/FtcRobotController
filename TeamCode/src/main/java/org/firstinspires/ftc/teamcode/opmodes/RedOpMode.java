package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.GamepadManager;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.plugin.plugins.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.Depot;
import org.firstinspires.ftc.teamcode.robot.Field;
import org.firstinspires.ftc.teamcode.robot.HandsOfGod;
import org.firstinspires.ftc.teamcode.robot.LimeLight;
import org.firstinspires.ftc.teamcode.robot.PalmsOfGod;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Tag;
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.robot.TurretPose;
import com.pedropathing.geometry.Pose;

import static com.qualcomm.robotcore.util.RobotLog.*;
import static java.lang.Math.abs;

@TeleOp(name = "RedOpMode")
public class RedOpMode extends OpMode {
	Robot bot;
	MecanumDrive mecanumDrive;
	GamepadManager mgamepad;
	Follower drivetrain;
	double speedTrim = 0;
	boolean handsUp = false;
	boolean autoTracking = true;
	boolean shootHands;
	boolean leftPalmOpen = false, rightPalmOpen = false;
	boolean tagMatch = false;
	boolean lastA;
	LimeLight ll;
	Tag targetTag = Tag.RED;
	TurretPose lastPose;

	@Override
	public void init() {
		bot = new Robot(this).enableIntake().enableHandsOfGod().enablePalmsOfGod();
		bot.init();
		ll = new LimeLight(this);
		ll.init();
		ll.setMode(LimeLight.LimeLightMode.AprilTag);
		d("AHM init");
		drivetrain = Constants.createFollower(hardwareMap);
		drivetrain.setStartingPose(new Pose(72, 72, 0)); //TODO THIS IS FOR TESTING

		try {
			lastPose = Turret.getSavedPosition();
		} catch (Exception e) {
			lastPose = new TurretPose(new Pose(0, 0, 0), 0);
		}

		bot.turret = new Turret(this, lastPose);
		bot.turret.init();
		mecanumDrive = new MecanumDrive(this);
		mecanumDrive.init();
//		bot.turret.useGamepad();


		mgamepad = new GamepadManager(gamepad2);
	}

	@Override
	public void loop() {
		drivetrain.update();
		mecanumDrive.loop();
		bot.loop();
		telemetry.clearAll(); // Disables telemetry from the Turret
		bot.turret.trackTarget(Depot.getPosition(Field.Alliance.Red), drivetrain.getPoseTracker()
				.getLocalizer()).run();
		bot.turret.setRotationPower(.5);

		bot.intake.setSpeed(gamepad2.right_trigger * ((gamepad2.y) ? -1 : 1));

		if (mgamepad.justPressed(GamepadManager.Button.X)) {
			handsUp = !handsUp;
		}

		if (gamepad1.a && ! lastA)
			ll.debugSnapshot();
		lastA = gamepad1.a;

		if (gamepad2.a) {
			shootHands = true;
		}

		if (shootHands)
			shootHands = bot.shoot(Robot.BallPosition.Hands).run();


		bot.intake.setHeight(gamepad1.right_trigger);

		bot.turret.setSpeed(bot.turret.getSpeedByDistance(bot.turret.getDistance(Depot.getPosition(Field.Alliance.Red))));

		telemetry.addLine("Ball" + bot.palmsOfGod.getLeftBall());

		if (mgamepad.justPressed(GamepadManager.Button.RIGHT_BUMPER)) {
			rightPalmOpen = !rightPalmOpen;
		}

		if (mgamepad.justPressed(GamepadManager.Button.LEFT_BUMPER)) {
			leftPalmOpen = !leftPalmOpen;
		}

		if (gamepad2.b) {
			leftPalmOpen = false;
			rightPalmOpen = false;
		}

		if (gamepad2.right_stick_button) {
			autoTracking = !autoTracking;
		}

		bot.turret.setAngleTrim((bot.turret.rotationTrim + gamepad2.right_stick_y / 17.5) * ((gamepad2.right_stick_button) ? 0 : 1)); // the lion does not concern herself with the math

		bot.handsOfGod.setPosition((handsUp) ? HandsOfGod.Position.Up : HandsOfGod.Position.Down);
		bot.palmsOfGod.setLeftPalm((leftPalmOpen) ? PalmsOfGod.Position.Up : PalmsOfGod.Position.Down);
		bot.palmsOfGod.setRightPalm((rightPalmOpen) ? PalmsOfGod.Position.Up : PalmsOfGod.Position.Down);

		bot.palmsOfGod.getLeftBall();
		bot.palmsOfGod.getRightBall();

		telemetry.update();
		mgamepad.poll();
	}
}
