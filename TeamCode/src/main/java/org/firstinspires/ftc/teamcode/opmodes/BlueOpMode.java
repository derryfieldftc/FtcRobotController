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

@TeleOp(name = "BlueOpMode")
public class BlueOpMode extends OpMode {
	Robot bot;
	MecanumDrive mecanumDrive;
	GamepadManager mgamepad;
	Follower drivetrain;
	double speedTrim = 0;
	boolean handsUp = false;
	boolean autoTracking = true;
	boolean shootHands;
	boolean leftPalmOpen = false, rightPalmOpen = false;
	boolean lastA;
	LimeLight ll;
	TurretPose lastPose;
	private boolean turretOn = true;

	@Override
	public void init() {
		bot = new Robot(this).enableIntake().enableHandsOfGod().enablePalmsOfGod();
		bot.init();
		ll = new LimeLight(this);
		ll.init();
		ll.setMode(LimeLight.LimeLightMode.AprilTag);
		d("AHM init");
		drivetrain = Constants.createFollower(hardwareMap);

		try {
			lastPose = Turret.getSavedPosition();
		} catch (Exception e) {
			lastPose = new TurretPose(new Pose(0, 0, 0), 0);
		}

		bot.turret = new Turret(this, lastPose);
		bot.turret.refreshEncoder = false;
		bot.turret.init();
		mecanumDrive = new MecanumDrive(this);
		mecanumDrive.init();
		drivetrain.setStartingPose(lastPose.pose);

		mgamepad = new GamepadManager(gamepad2);
	}

	@Override
	public void loop() {
		bot.turret.savePosition();
		drivetrain.update();
		mecanumDrive.loop();
		bot.loop();
		telemetry.clearAll(); // Disables telemetry from the Turret

		bot.turret.trackTarget(Depot.getPosition(Field.Alliance.Blue), drivetrain.getPoseTracker()
				.getLocalizer()).run();
		if (autoTracking) {
			bot.turret.setRotationPower(0);
		} else {
			bot.turret.setRotationPower(1);

		}

		bot.intake.setSpeed(gamepad2.right_trigger * ((gamepad2.start) ? -1 : 1));

		if (mgamepad.justPressed(GamepadManager.Button.X)) {
			handsUp = !handsUp;
		}

		if (gamepad1.a && ! lastA)
			ll.debugSnapshot();
		lastA = gamepad1.a;

		if (mgamepad.justPressed(GamepadManager.Button.DPAD_UP)) {
			speedTrim += .02;
		}
		if (mgamepad.justPressed(GamepadManager.Button.DPAD_DOWN)) {
			speedTrim -= .02;
		}
		if (mgamepad.justPressed(GamepadManager.Button.DPAD_RIGHT)) {
			speedTrim = 0;
		}

		if (mgamepad.justPressed(GamepadManager.Button.Y))
			turretOn = !turretOn;

		if (gamepad2.a) {
			shootHands = true;
		}

		if (shootHands)
			shootHands = bot.shoot(Robot.BallPosition.Hands).run();

		bot.intake.setHeight(gamepad1.right_trigger);

		bot.turret.setSpeed((speedTrim + bot.turret.getSpeedByDistance(bot.turret.getDistance(Depot.getPosition(Field.Alliance.Blue)))) * ((turretOn) ? 0 : 1));

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

		if (mgamepad.justPressed(GamepadManager.Button.DPAD_LEFT)) {
			autoTracking = !autoTracking;
		}

		bot.turret.setAngleTrim((bot.turret.rotationTrim + gamepad2.right_stick_y / 17.5) * ((gamepad2.right_stick_button) ? 0 : 1)); // the lion does not concern herself with the math

		bot.handsOfGod.setPosition((handsUp) ? HandsOfGod.Position.Up : HandsOfGod.Position.Down);
		bot.palmsOfGod.setLeftPalm((leftPalmOpen) ? PalmsOfGod.Position.Up : PalmsOfGod.Position.Down);
		bot.palmsOfGod.setRightPalm((rightPalmOpen) ? PalmsOfGod.Position.Up : PalmsOfGod.Position.Down);

		telemetry.addData("turretOn", turretOn);
		telemetry.addData("speedTrim", speedTrim);
		telemetry.addData("angleTrim", bot.turret.rotationTrim);
		telemetry.addData("autoTracking", autoTracking);
		telemetry.addData("distance", bot.turret.getDistance(Depot.getPosition(Field.Alliance.Blue)));
		telemetry.addData("velocity", bot.turret.spinner0.getVelocity());

		bot.turret.dumpTelemetry(telemetry);

		telemetry.update();
		mgamepad.poll();
	}
}
