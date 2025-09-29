package org.firstinspires.ftc.teamcode.autonmous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RR.MecanumDrive;

@Autonomous(name = "Red1test")
public class Red1 extends OpMode {
	MecanumDrive mecanumDrive;
	Action route;
	Pose2d initPose = new Pose2d(20, -57, Math.PI / 4);

	@Override
	public void init() {
		mecanumDrive.actionBuilder(initPose)
				.waitSeconds(2) // FIRE
				.splineToConstantHeading(new Vector2d(50, -50), Math.PI / 4) // collecting human player zone balls
				.splineTo(new Vector2d(60, -55), -Math.PI / 2)
				.splineTo(new Vector2d(60, -60), -Math.PI / 2)
				.setReversed(true)
				.splineToConstantHeading(new Vector2d(15, -50), Math.PI / 2)
				.setReversed(false)
				.turn(Math.PI / 2) // FIRE
				.splineToConstantHeading(new Vector2d(41, -11), 0) // Collect row 2
				.splineToConstantHeading(new Vector2d(57, -4), 0) // lever
				.waitSeconds(1)
				.setReversed(true)
				.splineToConstantHeading(new Vector2d(10, 0), Math.PI) // Back to shootable
				.waitSeconds(2) // FIRE
				.setReversed(false)
				.splineToConstantHeading(new Vector2d(41, 12), 0)
				.splineTo(new Vector2d(47, 12), 0) // Collect row 1
				.setReversed(true)
				.splineToConstantHeading(new Vector2d(10, 0), Math.PI) // Back to shootable
				.waitSeconds(2) // FIRE
				.setReversed(false)
				.splineToConstantHeading(new Vector2d(44, -35), 0)
				.setReversed(true)
				.splineToConstantHeading(new Vector2d(20, -57), Math.PI)
				.build();
	}

	@Override
	public void start() {
		Actions.runBlocking(route);
		stop();
	}

	@Override
	public void loop() {
		stop();
	}
}
