package org.firstinspires.ftc.teamcode.autonmous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RR.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.Depot;
import org.firstinspires.ftc.teamcode.robot.Field;
import org.firstinspires.ftc.teamcode.robot.PalmsOfGod;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.robot.TurretPose2d;

@Autonomous(name = "Blue2")
public class Blue2 extends OpMode {
	MecanumDrive mecanumDrive;
	Action route;
	Pose2d initPose = new Pose2d(45, 50, -Math.PI / 2);
	Robot bot;
	Depot depot = new Depot(Field.Alliance.Blue);
	Turret turret;
	Action action;

	@Override
	public void init() {
		bot = new Robot(this).enablePalmsOfGod().enableHandsOfGod().enableIntake().enableTurret();
		turret = bot.turret;
		bot.init();
		bot.turret.trackTarget().setTarget(depot.getPosition());
		mecanumDrive = new MecanumDrive(hardwareMap, initPose);
		bot.setBalls(Field.Ball.Purple, Field.Ball.Green, Field.Ball.Purple);
		bot.palmsOfGod.setLeftPalm(PalmsOfGod.Position.Down);
		bot.palmsOfGod.setRightPalm(PalmsOfGod.Position.Down);

		route = mecanumDrive.actionBuilder(initPose, AutoFunctions::mirror)
				.stopAndAdd(telemetryPacket -> {bot.turret.setSpeed(.4); return false;})
				.strafeTo(new Vector2d(25, 25)) // Away from goal to shootable location, also get tag here
				.stopAndAdd(bot.shootAction(Robot.BallPosition.Hands))
				.waitSeconds(1)
				.stopAndAdd(telemetryPacket -> {bot.palmsOfGod.setRightPalm(PalmsOfGod.Position.Up); return false;})
				.waitSeconds(1)
				.stopAndAdd(bot.shootAction(Robot.BallPosition.Right))
				.waitSeconds(1)
				.stopAndAdd(telemetryPacket -> {bot.palmsOfGod.setLeftPalm(PalmsOfGod.Position.Up); return false;})
				.stopAndAdd(bot.shootAction(Robot.BallPosition.Left))
				.stopAndAdd(telemetryPacket -> {bot.turret.setSpeed(.4); return false;})
				.splineTo(new Vector2d(47, 14), 0) // Collect row 3
				.stopAndAdd(telemetryPacket -> { // Lambda actions work if they are instantaneous
					bot.setBalls(Field.Ball.Purple, Field.Ball.Green, Field.Ball.Purple);
					return false;
				})
				.splineToConstantHeading(new Vector2d(50, 6), 0) // to lever
				.splineToConstantHeading(new Vector2d(53, 6), 0) // to lever
				.waitSeconds(1) // Lever
				.setReversed(true)
				.strafeTo(new Vector2d(15, 8)) // Back to shootable
				.waitSeconds(2)// FIRE
				.setReversed(false)
				.splineToConstantHeading(new Vector2d(38, -12), 0) // Collect row 2
				.splineToConstantHeading(new Vector2d(47, -12), 0) // Collect row 2
				.setReversed(true)
				.splineToConstantHeading(new Vector2d(15, 8), Math.PI) // Shootable once more
				.waitSeconds(2) // FIRE
				.splineToConstantHeading(new Vector2d(30, -34), 0) // Align to row 1
				.splineToConstantHeading(new Vector2d(45, -36), 0) // Collect row 1
				.setReversed(true)
				.splineToConstantHeading(new Vector2d(15, -54), Math.PI) // Back to shootable low
				.waitSeconds(2) // FIRE
				.setReversed(false)
				.splineTo(new Vector2d(50, -40), 0) // above human player zone
				.splineTo(new Vector2d(60, -65), -Math.PI / 2) // Human Player
				// Too slow D:
//						.strafeTo(new Vector2d(10, -60)) // Shootable
//						.waitSeconds(2) // FIRE
				.build();
		action = new ParallelAction(
				bot.savePosition(new TurretPose2d(mecanumDrive.localizer.getPose(), turret.getRotation())),
				route,
				turret.autoTracking(mecanumDrive));

	}

	@Override
	public void start() {
		// <3 composable actions I think I'm in love
		Actions.runBlocking(action);
		stop();
	}

	@Override
	public void loop() {
		stop();
	}
}
