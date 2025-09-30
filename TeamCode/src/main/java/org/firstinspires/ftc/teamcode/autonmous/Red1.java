package org.firstinspires.ftc.teamcode.autonmous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RR.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.Intake;

@Autonomous(name = "Red1test")
public class Red1 extends OpMode {
	MecanumDrive mecanumDrive;
	Intake intake;
	Action route;
	Pose2d initPose = new Pose2d(20, -57, Math.PI / 4);

	// Odometry is definitly not perfect yet. These values are all subject to change, and a lot of RR
	// Classes need finer tuning

	@Override
	public void init() {
		mecanumDrive = new MecanumDrive(hardwareMap, initPose);
		intake = new Intake(this);
		intake.init();
		route = mecanumDrive.actionBuilder(initPose)
				.splineToConstantHeading(new Vector2d(50, -55), Math.PI / 4) // collecting human player zone balls
				.splineTo(new Vector2d(60, -55), -Math.PI / 2)
				.stopAndAdd(intake.disable())
				.splineTo(new Vector2d(60, -62), -Math.PI / 2)
				.setReversed(true)
				.splineToConstantHeading(new Vector2d(15, -50), Math.PI / 2) // shoot
				.setReversed(false)
				.strafeTo(new Vector2d(15, -50))
				.turn(Math.PI / 2)
				.stopAndAdd(intake.enable())
				.splineToConstantHeading(new Vector2d(35, -5), 0) // Collect row 2
				.splineToConstantHeading(new Vector2d(41, -5), 0) // Collect row 2
				.splineToConstantHeading(new Vector2d(57, -2), 0) // lever
				.stopAndAdd(intake.disable())
				.waitSeconds(1)
				.setReversed(true)
				.splineToConstantHeading(new Vector2d(10, 0), Math.PI) // Back to shootable
				.waitSeconds(2) // FIRE
				.setReversed(false)
				.stopAndAdd(intake.enable())
				.splineToConstantHeading(new Vector2d(39, 15), 0)
				.splineTo(new Vector2d(50, 16), 0) // Collect row 3
				.stopAndAdd(intake.disable())
				.setReversed(true)
				.splineToConstantHeading(new Vector2d(10, 0), Math.PI) // Back to shootable
				.waitSeconds(2) // FIRE
				.setReversed(false)
				.stopAndAdd(intake.enable())
				.splineToConstantHeading(new Vector2d(46, -35), 0) // collect 1
				.stopAndAdd(intake.disable())
				.setReversed(true)
				.splineToConstantHeading(new Vector2d(20, -57), Math.PI)
				.build();
	}

	@Override
	public void start() {
		intake.setSpeed(1);
		Actions.runBlocking(route);
		stop();
	}

	@Override
	public void loop() {
		stop();
	}
}
