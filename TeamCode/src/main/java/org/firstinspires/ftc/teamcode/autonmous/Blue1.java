package org.firstinspires.ftc.teamcode.autonmous;

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

@Autonomous(name = "Blue1")
public class Blue1 extends OpMode {
	MecanumDrive mecanumDrive;
	Action action;
	Robot bot;
	Intake intake;
	PalmsOfGod palms;
	Depot depot = new Depot(Field.Alliance.Blue);
	Action route;
	Pose2d initPose = BluePoses.Init.pose;

	// Odometry is definitely not perfect yet. These values are all subject to change, and a lot of RR
	// Classes need finer tuning

	@Override
	public void init() {
		bot = new Robot(this).enableTurret().enableHandsOfGod().enablePalmsOfGod().enableIntake();
		bot.init();
		bot.turret.trackTarget().setTarget(depot.getPosition());
		intake = bot.intake;
		palms = bot.palmsOfGod;
		mecanumDrive = new MecanumDrive(hardwareMap, initPose);


		route = mecanumDrive.actionBuilder(initPose)
				.stopAndAdd(telemetryPacket -> {bot.turret.setSpeed(Turret.SpeedByDistance.Far.power); return false;})
				.waitSeconds(.5)
				.stopAndAdd(shootFar())
				.turn(Math.PI / 4)
				.stopAndAdd(intake.enable())
				.splineToConstantHeading(BluePoses.Row2CollectionSetup.pose.position, BluePoses.Row2CollectionSetup.pose.heading) // Collect row 2
				.splineToConstantHeading(BluePoses.Row2Collection.pose.position, BluePoses.Row2Collection.pose.heading) // Collect row 2
				.splineToConstantHeading(BluePoses.Lever.pose.position, BluePoses.Lever.pose.heading) // lever
				.stopAndAdd(intake.disable())
				.stopAndAdd(telemetryPacket -> {bot.turret.setSpeed(Turret.SpeedByDistance.Close.power); return false;})
				.waitSeconds(1)
				.setReversed(true)
				.splineToConstantHeading(BluePoses.ShootableMid.pose.position, BluePoses.ShootableMid.pose.heading) // Back to shootable
				.stopAndAdd(shootFar())
				.setReversed(false)
				.stopAndAdd(intake.enable())
				.splineToConstantHeading(BluePoses.AwayFromLever.pose.position, BluePoses.AwayFromLever.pose.heading)
				.splineToConstantHeading(BluePoses.Row3.pose.position, BluePoses.Row3.pose.heading) // Collect row 3
				.stopAndAdd(intake.disable())
				.setReversed(true)
				.splineToConstantHeading(BluePoses.ShootableMid2.pose.position, BluePoses.ShootableMid2.pose.heading) // Back to shootable
				.stopAndAdd(shootFar())
				.setReversed(false)
				.stopAndAdd(intake.enable())
				.splineToConstantHeading(BluePoses.Row1Collection.pose.position, BluePoses.Row1Collection.pose.heading) // collect 1
				.stopAndAdd(intake.disable())
				.setReversed(true)
				.splineToConstantHeading(BluePoses.FarShootingPosition.pose.position, BluePoses.FarShootingPosition.pose.heading)
				.stopAndAdd(shootFar())
				.build();
		action = new ParallelAction(bot.savePosition(new TurretPose2d(mecanumDrive.localizer.getPose(), bot.turret.getRotation())), route, bot.turret.autoTracking(mecanumDrive));
	}

	@Override
	public void start() {
		bot.turret.setSpeed(.3);
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
