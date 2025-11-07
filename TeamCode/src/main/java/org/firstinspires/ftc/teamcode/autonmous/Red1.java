package org.firstinspires.ftc.teamcode.autonmous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
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
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.robot.TurretPose2d;
import static org.firstinspires.ftc.teamcode.autonmous.AutoFunctions.*;

@Autonomous(name = "Red1")
public class Red1 extends OpMode {
	MecanumDrive mecanumDrive;
	Action action;
	Robot bot;
	Intake intake;
	PalmsOfGod palms;
	Depot depot = new Depot(Field.Alliance.Red);
	Action route;
	Pose2d initPose = RedPoses.Init.pose;

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


		route = mecanumDrive.actionBuilder(initPose)
				.stopAndAdd(telemetryPacket -> {Robot.turret.setSpeed(Turret.SpeedByDistance.Far.power); return false;})
				.waitSeconds(.5)
				.stopAndAdd(shootFar())
				.turn(-Math.PI / 4)
				.stopAndAdd(intake.enable())
				.splineToConstantHeading(new Vector2d(35, -5), 0) // Collect row 2
				.splineToConstantHeading(new Vector2d(41, -5), 0) // Collect row 2
				.splineToConstantHeading(new Vector2d(62, -2), 0) // lever
				.stopAndAdd(intake.disable())
				.stopAndAdd(telemetryPacket -> {Robot.turret.setSpeed(Turret.SpeedByDistance.Close.power); return false;})
				.waitSeconds(1)
				.setReversed(true)
				.splineToConstantHeading(new Vector2d(10, 0), Math.PI) // Back to shootable
				.stopAndAdd(shootFar())
				.setReversed(false)
				.stopAndAdd(intake.enable())
				.splineToConstantHeading(new Vector2d(39, 15), 0)
				.splineToConstantHeading(new Vector2d(50, 16), 0) // Collect row 3
				.stopAndAdd(intake.disable())
				.setReversed(true)
				.splineToConstantHeading(new Vector2d(10, 0), Math.PI) // Back to shootable
				.stopAndAdd(shootFar())
				.setReversed(false)
				.stopAndAdd(intake.enable())
				.splineToConstantHeading(new Vector2d(46, -35), 0) // collect 1
				.stopAndAdd(intake.disable())
				.setReversed(true)
				.splineToConstantHeading(new Vector2d(20, -57), Math.PI)
				.stopAndAdd(shootFar())
				.build();
		action = new ParallelAction(bot.savePosition(new TurretPose2d(mecanumDrive.localizer.getPose(), Robot.turret.getRotation())), route, Robot.turret.autoTracking(mecanumDrive));
	}

	@Override
	public void start() {
		Robot.turret.setSpeed(.3);
		Actions.runBlocking(action);
		stop();
	}

	public Action shootFar() {
		return new SequentialAction(bot.shootAction(Robot.BallPosition.Hands),
				telemetryPacket -> {palms.setRightPalm(PalmsOfGod.Position.Up); return false;},
				new SleepAction(.5),
				bot.shootAction(Robot.BallPosition.Hands),
				telemetryPacket -> {palms.setLeftPalm(PalmsOfGod.Position.Up); return false;},
				new SleepAction(.5),
				bot.shootAction(Robot.BallPosition.Hands),
				bot.setPalms(PalmsOfGod.Position.Down, PalmsOfGod.Position.Down));
	}

	public Action shootClose() {
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

}
