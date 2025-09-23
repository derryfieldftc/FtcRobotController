package org.firstinspires.ftc.teamcode.autonmous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OpModeGroups;
import org.firstinspires.ftc.teamcode.RR.MecanumDrive;

@Autonomous(name = "RRBoxTest", group = OpModeGroups.TESTS)
public class RRBoxTest extends LinearOpMode {
	MecanumDrive mecanumDrive;

	@Override
	public void runOpMode() throws InterruptedException {
		mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
		Action firstMove = mecanumDrive.actionBuilder(new Pose2d(0, 0, 0))
				.splineToLinearHeading(new Pose2d(48, 0, Math.toRadians(0)), Math.toRadians(0))
				.splineToLinearHeading(new Pose2d(0, 48, Math.toRadians(0)), Math.toRadians(0))
				.splineToLinearHeading(new Pose2d(-48, 0, Math.toRadians(0)), Math.toRadians(0))
				.splineToLinearHeading(new Pose2d(0, -48, Math.toRadians(0)), Math.toRadians(0))
				.build();

		waitForStart();

		Actions.runBlocking(firstMove);
	}
}
