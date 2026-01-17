package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GamepadManager;
import org.firstinspires.ftc.teamcode.autonmous.actions.TeleOpAction;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.Depot;
import org.firstinspires.ftc.teamcode.robot.Field;
import org.firstinspires.ftc.teamcode.robot.Lift;
import org.firstinspires.ftc.teamcode.robot.LimeLight;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Spindexer;
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.robot.TurretPose;
import com.pedropathing.geometry.Pose;

import static com.qualcomm.robotcore.util.RobotLog.*;
import static java.lang.Math.abs;

import java.util.function.Supplier;

@TeleOp(name = "BlueOpMode")
public class BlueOpMode extends OpMode {
	Robot bot;
	GamepadManager mgamepad1;
	GamepadManager mgamepad2;
	Follower drivetrain;
	double speedTrim = 0;
	boolean liftUp = false;
	boolean autoTracking = true;
	boolean autoMoving = false;
	TeleOpAction shootAll;
	boolean lastA;
	LimeLight ll;
	TurretPose lastPose;
	Supplier<PathChain> gotoLever;
	private boolean turretOn = true;

	@Override
	public void init() {
		bot = new Robot(this);
		ll = new LimeLight(this);
		ll.setMode(LimeLight.LimeLightMode.AprilTag);
		d("AHM init");
		drivetrain = Constants.createFollower(hardwareMap);

		shootAll = new TeleOpAction(bot.shootAll());

		try {
			lastPose = Turret.getSavedPosition();
		} catch (Exception e) {
			lastPose = new TurretPose(new Pose(0, 0, 0), 0);
		}

		bot.turret = new Turret(this, lastPose);
		bot.turret.refreshEncoder = false;
		drivetrain.setStartingPose(lastPose.pose);

		mgamepad1 = new GamepadManager(gamepad1);
		mgamepad2 = new GamepadManager(gamepad2);

		bot.spindexer.setRotatorPower(1);

		gotoLever = () -> drivetrain.pathBuilder() // Lazy Curve Generation
				.addPath(new Path(new BezierLine(drivetrain::getPose, new Pose(12, 60))))
				.setHeadingInterpolation(
						HeadingInterpolator.linearFromPoint(drivetrain::getHeading, Math.toRadians(140), 0.8))
				.build();
	}

	@Override
	public void start() {
		drivetrain.startTeleOpDrive();
	}

	@Override
	public void loop() {
		if (mgamepad1.justPressed(GamepadManager.Button.A)) {
			drivetrain.followPath(gotoLever.get()); // thx pedropathing <3
			autoMoving = true;
		}
		if (!autoMoving)
			drivetrain.setTeleOpDrive(
					-gamepad1.left_stick_y * (1 - gamepad1.right_trigger),
					-gamepad1.left_stick_x * (1 - gamepad1.right_trigger),
					-gamepad1.right_stick_x * (1 - gamepad1.right_trigger),
					true);

		if (autoMoving && (mgamepad1.justPressed(GamepadManager.Button.B) || !drivetrain.isBusy())) {
			drivetrain.startTeleopDrive();
			autoMoving = false;
		}

		bot.turret.savePosition();
		drivetrain.update();
		bot.loop();
		telemetry.clearAll(); // Disables telemetry from the Turret

		bot.turret.trackTarget(Depot.getPosition(Field.Alliance.Blue), drivetrain.getPoseTracker()
				.getLocalizer()).run();
		if (!autoTracking) {
			bot.turret.setRotationPower(0);
		} else {
			bot.turret.setRotationPower(1);
		}

		bot.intake.setSpeed(gamepad2.right_trigger * ((gamepad2.start) ? -1 : 1));
		if (!gamepad2.left_bumper) {
			bot.spindexer.setLiftPosition(Spindexer.Height.Down);
		} else {
			bot.spindexer.setLiftPosition(Spindexer.Height.Up);
		}

		if (mgamepad2.justPressed(GamepadManager.Button.RIGHT_BUMPER)) {
			liftUp = !liftUp;
		}

		if (mgamepad2.justPressed(GamepadManager.Button.X)) {
			bot.spindexer.setPosition(Spindexer.Position.Zero);
		}

		if (mgamepad2.justPressed(GamepadManager.Button.A)) {
			bot.spindexer.setPosition(Spindexer.Position.One);
		}

		if (mgamepad2.justPressed(GamepadManager.Button.B)) {
			bot.spindexer.setPosition(Spindexer.Position.Two);
		}

		bot.spindexer.setRotatorPower(1);

		if (liftUp) {
			bot.lift.setPosition(Lift.Position.Up);
		} else {
			bot.lift.setPosition(Lift.Position.Down);
		}

		if (gamepad1.a && !lastA)
			ll.debugSnapshot();
		lastA = gamepad1.a;

		if (mgamepad2.justPressed(GamepadManager.Button.DPAD_UP)) {
			speedTrim += .02;
		}
		if (mgamepad2.justPressed(GamepadManager.Button.DPAD_DOWN)) {
			speedTrim -= .02;
		}
		if (mgamepad2.justPressed(GamepadManager.Button.DPAD_RIGHT)) {
			speedTrim = 0;
		}

		if (mgamepad2.justPressed(GamepadManager.Button.Y))
			turretOn = !turretOn;

		bot.turret.setSpeed((speedTrim
				+ bot.turret.getSpeedByDistance(bot.turret.getDistance(Depot.getPosition(Field.Alliance.Blue))))
				* ((turretOn) ? 0 : 1));

		if (mgamepad2.justPressed(GamepadManager.Button.DPAD_LEFT)) {
			autoTracking = !autoTracking;
		}

		bot.turret.setAngleTrim(
				(bot.turret.rotationTrim + gamepad2.right_stick_y / 17.5) * ((gamepad2.right_stick_button) ? 0 : 1)); // the
																														// lion
																														// does
																														// not
																														// concern
																														// herself
																														// with
																														// the
																														// math

		telemetry.addData("turretOn", turretOn);
		telemetry.addData("speedTrim", speedTrim);
		telemetry.addData("angleTrim", bot.turret.rotationTrim);
		telemetry.addData("autoTracking", autoTracking);
		telemetry.addData("distance", bot.turret.getDistance(Depot.getPosition(Field.Alliance.Blue)));
		telemetry.addData("velocity", bot.turret.spinner0.getVelocity());

		bot.turret.dumpTelemetry(telemetry);

		bot.spindexer.updateBalls();
		telemetry.update();
		mgamepad1.poll();
		mgamepad2.poll();
	}
}
