package org.firstinspires.ftc.teamcode.autonmous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OpModeGroups;
import org.firstinspires.ftc.teamcode.RR.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.Intake;

@Autonomous(name = "RRCircle", group = OpModeGroups.TESTS)
public class RRTest extends LinearOpMode {
	MecanumDrive mecanumDrive;

	@Override
	public void runOpMode() throws InterruptedException {
		hardwareMap.getClass();
		Intake intake = new Intake(this);
		intake.init();
		intake.setSpeed(1);
		intake.loop();
		final Pose2d initalPose = new Pose2d(-6, -60, Math.PI / 2);
		mecanumDrive = new MecanumDrive(hardwareMap, initalPose);
		Action firstMove = mecanumDrive.actionBuilder(initalPose)
				.splineTo(new Vector2d(-35, -35), Math.PI)
				.splineTo(new Vector2d(-45, -35), Math.PI)
				.setReversed(true)
				.splineTo(new Vector2d(-16, -55), -Math.PI / 2)
				.setReversed(false)
				.splineTo(new Vector2d(-55, -45), -Math.PI / 2)
				.splineToConstantHeading(new Vector2d(-60, -60), -Math.PI / 2)
				.strafeTo(new Vector2d(-16, -55))
				.turn(Math.PI)
				.build();

		waitForStart();

		Actions.runBlocking(firstMove);
	}
}
