package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GamepadManager;
import org.firstinspires.ftc.teamcode.autonmous.actions.Action;
import org.firstinspires.ftc.teamcode.autonmous.actions.TeleOpAction;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.Depot;
import org.firstinspires.ftc.teamcode.robot.Drawing;
import org.firstinspires.ftc.teamcode.robot.Field;
import org.firstinspires.ftc.teamcode.robot.Lift;
import org.firstinspires.ftc.teamcode.robot.LimeLight;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Spindexer;
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.robot.TurretPose;
import com.pedropathing.geometry.Pose;

import static com.qualcomm.robotcore.util.RobotLog.*;
import static org.firstinspires.ftc.teamcode.pedro.Tuning.follower;
import static java.lang.Math.abs;

import java.util.function.Supplier;

@TeleOp(name = "RedOpMode")
public class RedOpMode extends OpMode {
	double lastLoopTime = 0;
	Robot bot;
	GamepadManager mgamepad1;
	GamepadManager mgamepad2;
	Follower drivetrain;
	double speedTrim = 0;
	boolean liftUp = false;
	boolean autoTracking = false;
	boolean autoMoving = false;
	boolean shootingAll = false;
	boolean prevIntaking;
	Action shootAll;
	boolean lastA;
	LimeLight ll;
	TurretPose lastPose;
	Supplier<PathChain> gotoLever;
	private boolean turretOn = true;
	TeleOpAction resetForIntaking;

	@Override
	public void init() {
		bot = new Robot(this);
		ll = new LimeLight(this);
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
		drivetrain.setStartingPose(lastPose.pose);

		mgamepad1 = new GamepadManager(gamepad1);
		mgamepad2 = new GamepadManager(gamepad2);

		bot.spindexer.setRotatorPower(.75);

		gotoLever = () -> drivetrain.pathBuilder() //Lazy Curve Generation
				.addPath(new Path(new BezierLine(drivetrain::getPose, new Pose(132, 60))))
				.setHeadingInterpolation(HeadingInterpolator.linearFromPoint(drivetrain::getHeading, Math.toRadians(40), 0.8))
				.build();
		shootAll = bot.shootAll();
		resetForIntaking = new TeleOpAction(() -> bot.spindexerPrepIntake());
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
					(-gamepad1.right_stick_x / 1.5) * (1 - gamepad1.right_trigger),
					true);

		if (autoMoving && (mgamepad1.justPressed(GamepadManager.Button.B) || !drivetrain.isBusy())) {
			drivetrain.startTeleopDrive();
			autoMoving = false;
		}

//		bot.turret.savePosition();
		drivetrain.update();
		if (!shootingAll) {
			bot.loop();
			bot.turret.trackTarget(Depot.getPosition(Field.Alliance.Red), drivetrain.getPoseTracker()
					.getLocalizer()).run();
		}
		telemetry.clearAll(); // Disables telemetry from the Turret

		if (!autoTracking) {
			bot.turret.setRotationPower(0);
		} else {
			bot.turret.setRotationPower(1);
		}

		resetForIntaking.run();
		if (gamepad2.right_trigger > .5 && !gamepad2.start) {
			if (!prevIntaking)
				resetForIntaking.start();
			prevIntaking = true;

			if (!resetForIntaking.isRunning()) {
				bot.intake.setSpeed(gamepad2.right_trigger * ((gamepad2.start) ? -1 : 1));
				bot.spindexer.setLiftPosition(Spindexer.Height.Down);
			}
		} else {
			bot.spindexer.setLiftPosition(Spindexer.Height.Up);
			if (prevIntaking)
				bot.spindexer.updateBalls(); // only if we just stopped intaking
			prevIntaking = false;
		}

		if (gamepad2.start) {
			bot.intake.setSpeed(-1);
		}

		if (mgamepad2.justPressed(GamepadManager.Button.RIGHT_BUMPER)) {
			liftUp = !liftUp;
		}

		if (mgamepad2.justPressed(GamepadManager.Button.X)) {
			bot.spindexer.setPosition(Spindexer.Position.Zero);
			bot.spindexer.updateBalls();
		}

		if (mgamepad2.justPressed(GamepadManager.Button.A)) {
			bot.spindexer.setPosition(Spindexer.Position.One);
			bot.spindexer.updateBalls();
		}

		if (mgamepad2.justPressed(GamepadManager.Button.B)) {
			bot.spindexer.setPosition(Spindexer.Position.Two);
			bot.spindexer.updateBalls();
		}

//		bot.spindexer.setRotatorPower(1);

		if (!shootingAll) {
			if (liftUp) {
				bot.lift.setPosition(Lift.Position.Up);
			} else {
				bot.lift.setPosition(Lift.Position.Down);
			}
		}

		if (gamepad1.a && ! lastA)
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

		bot.turret.setSpeed((speedTrim + bot.turret.getSpeedByDistance(bot.turret.getDistance(Depot.getPosition(Field.Alliance.Red)))) * ((turretOn) ? 0 : 1));

		if (mgamepad2.justPressed(GamepadManager.Button.DPAD_LEFT)) {
			autoTracking = !autoTracking;
		}

		telemetry.addData("turretOn", turretOn);
		telemetry.addData("speedTrim", speedTrim);
		telemetry.addData("angleTrim", bot.turret.rotationTrim);
		telemetry.addData("autoTracking", autoTracking);
		telemetry.addData("distance", bot.turret.getDistance(Depot.getPosition(Field.Alliance.Red)));
		telemetry.addData("velocity", bot.turret.spinner0.getVelocity());

		bot.turret.dumpTelemetry(telemetry);

		telemetry.addData("loop time ", this.getRuntime() - lastLoopTime);
		lastLoopTime = this.getRuntime();

		if (shootingAll || gamepad2.left_bumper) {
			shootingAll = shootAll.run();
			telemetry.addLine("shooting all");
			if (!shootingAll) {
				shootAll = bot.shootAll(); // set it to new shootall action as the last one finished
			}
		}

		telemetry.update();
		mgamepad1.poll();
		mgamepad2.poll();

		d("AHM LLPOSE " + bot.getLLPose(drivetrain).toString());


//		Drawing.drawDebug(follower);
	}
}