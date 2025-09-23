package org.firstinspires.ftc.teamcode.autonmous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RR.MecanumDrive;

public class RRBoxTest extends LinearOpMode {
	MecanumDrive mecanumDrive;
	@Override
	public void runOpMode() throws InterruptedException {
		mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
		TrajectoryActionBuilder builder;
	}
}
