package org.firstinspires.ftc.teamcode.autonmous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RR.MecanumDrive;

@Autonomous(name = "Red2test")
public class Red2 extends OpMode {
	MecanumDrive mecanumDrive;
	Action route;
	Pose2d initPose = new Pose2d(45, 50, -Math.PI / 2);

	@Override
	public void init() {
		mecanumDrive = new MecanumDrive(hardwareMap, initPose);

		route = mecanumDrive.actionBuilder(initPose)
				.strafeTo(new Vector2d(25, 25)) // Away from goal to shootable location, also get tag here
				.waitSeconds(2) // FIRE
				.splineTo(new Vector2d(47, 14), 0) // Collect row 3
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
