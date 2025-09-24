package org.firstinspires.ftc.teamcode.autonmous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
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
				.strafeTo(new Vector2d(0, 24))
				.strafeTo(new Vector2d(24, 24))
				.strafeTo(new Vector2d(24, 0))
				.strafeTo(new Vector2d(0, 0))
				.splineToConstantHeading(new Vector2d(12, 12), 0)
				.splineToConstantHeading(new Vector2d(48, 48), 0)
				.build();

		waitForStart();

		Actions.runBlocking(firstMove);
	}
}
