package org.firstinspires.ftc.teamcode.autonmous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RR.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.Depot;
import org.firstinspires.ftc.teamcode.robot.Field;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.PalmsOfGod;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.TurretPose2d;

@Autonomous(name = "Blue1")
public class Blue1 extends OpMode {
	MecanumDrive mecanumDrive;
	Action action;
	Robot bot;
	Intake intake;
	PalmsOfGod palms;
	Depot depot = new Depot(Field.Alliance.Blue);
	Action route;
	Pose2d initPose = new Pose2d(20, -57, Math.PI / 4);

	// Odometry is definitely not perfect yet. These values are all subject to change, and a lot of RR
	// Classes need finer tuning

	@Override
	public void init() {
		bot = new Robot(this).enableTurret().enableHandsOfGod().enablePalmsOfGod().enableIntake();
		bot.init();
		Robot.turret.trackTarget().setTarget(depot.getPosition());
		intake = Robot.intake;
		palms = Robot.palmsOfGod;
		mecanumDrive = new MecanumDrive(hardwareMap, initPose);


		route = mecanumDrive.actionBuilder(initPose, AutoFunctions::mirror)
				.stopAndAdd(telemetryPacket -> {Robot.turret.setSpeed(.7); return false;})
				.waitSeconds(.5)
				.stopAndAdd(shoot())
				.turn(-Math.PI / 4)
				.stopAndAdd(intake.enable())
				.splineToConstantHeading(new Vector2d(35, -5), 0) // Collect row 2
				.splineToConstantHeading(new Vector2d(41, -5), 0) // Collect row 2
				.splineToConstantHeading(new Vector2d(62, -2), 0) // lever
				.stopAndAdd(telemetryPacket -> {Robot.turret.setSpeed(.6); return false;})
				.waitSeconds(1)
				.setReversed(true)
				.splineToConstantHeading(new Vector2d(10, 0), Math.PI) // Back to shootable
				.stopAndAdd(shoot())
				.setReversed(false)
				.stopAndAdd(intake.enable())
				.splineToConstantHeading(new Vector2d(39, 15), 0)
				.splineToConstantHeading(new Vector2d(50, 16), 0) // Collect row 3
				.setReversed(true)
				.splineToConstantHeading(new Vector2d(10, 0), Math.PI) // Back to shootable
				.stopAndAdd(shoot())
				.setReversed(false)
				.stopAndAdd(intake.enable())
				.splineToConstantHeading(new Vector2d(46, -35), 0) // collect 1
				.setReversed(true)
				.splineToConstantHeading(new Vector2d(20, -57), Math.PI)
				.stopAndAdd(shoot())
				.build();
		action = new ParallelAction(updateLastKnownPose(), route, Robot.turret.autoTracking(mecanumDrive));
	}

	@Override
	public void start() {
		Robot.turret.setSpeed(.3);
		Actions.runBlocking(action);
		stop();
	}

	public Action shoot() {
		return new SequentialAction(bot.shootAction(Robot.BallPosition.Hands),
				telemetryPacket -> {palms.setRightPalm(PalmsOfGod.Position.Up); return false;},
				new SleepAction(.5),
				bot.shootAction(Robot.BallPosition.Hands),
				telemetryPacket -> {palms.setLeftPalm(PalmsOfGod.Position.Up); return false;},
				new SleepAction(.5),
				bot.shootAction(Robot.BallPosition.Hands),
				bot.setPalms(PalmsOfGod.Position.Down, PalmsOfGod.Position.Down));
	}

	@Override
	public void loop() {
	}

	public Action updateLastKnownPose() {
		return new Action() {
			@Override
			public boolean run(@NonNull TelemetryPacket telemetryPacket) {
				Robot.finalPose = new TurretPose2d(mecanumDrive.localizer.getPose(), Robot.turret.getRotation());
				return true;
			}
		};
	}
}
