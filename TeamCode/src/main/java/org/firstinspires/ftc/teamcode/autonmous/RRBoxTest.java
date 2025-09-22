package org.firstinspires.ftc.teamcode.autonmous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.OpModeGroups;
import org.firstinspires.ftc.teamcode.RR.drive.SampleMecanumDrive;

@Autonomous(name = "RRbox", group = OpModeGroups.AUTO)
public class RRBoxTest extends OpMode {
	SampleMecanumDrive mecanumDrive = new SampleMecanumDrive(hardwareMap);
	Trajectory trajectory;

	@Override
	public void init() {
		mecanumDrive.setPoseEstimate(new Pose2d());
	}

	@Override
	public void loop() {
		//Splineless
		trajectory = mecanumDrive.trajectoryBuilder(new Pose2d()).strafeLeft(20).build();
		mecanumDrive.followTrajectory(trajectory);
		trajectory = mecanumDrive.trajectoryBuilder(new Pose2d()).forward(20).build();
		mecanumDrive.followTrajectory(trajectory);
		trajectory = mecanumDrive.trajectoryBuilder(new Pose2d()).strafeRight(20).build();
		mecanumDrive.followTrajectory(trajectory);
		trajectory = mecanumDrive.trajectoryBuilder(new Pose2d()).back(20).build();
		mecanumDrive.followTrajectory(trajectory);
	}
}
