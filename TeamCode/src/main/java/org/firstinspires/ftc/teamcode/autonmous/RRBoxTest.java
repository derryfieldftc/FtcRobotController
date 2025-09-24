package org.firstinspires.ftc.teamcode.autonmous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OpModeGroups;
import org.firstinspires.ftc.teamcode.RR.MecanumDrive;

@Autonomous(name = "RRCircle", group = OpModeGroups.TESTS)
public class RRBoxTest extends LinearOpMode {
	MecanumDrive mecanumDrive;

	@Override
	public void runOpMode() throws InterruptedException {
		mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.PI / 2));
		Action firstMove = mecanumDrive.actionBuilder(new Pose2d(0, 0, Math.PI / 2))
				.splineTo(new Vector2d(24, 24), 0)
				.splineTo(new Vector2d(48, 0), -Math.PI / 2)
				.splineTo(new Vector2d(24, -24), Math.PI)
				.splineTo(new Vector2d(0, 0), Math.PI / 2)
				.build();

		waitForStart();

		Actions.runBlocking(firstMove);
	}
}
