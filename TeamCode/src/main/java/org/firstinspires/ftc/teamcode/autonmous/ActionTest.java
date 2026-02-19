package org.firstinspires.ftc.teamcode.autonmous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous
@Disabled
public class ActionTest extends AutoOpMode {
	PathChain path;

	@Override
	public void autoInit() {
		path = drive.pathBuilder()
				.addPath(new BezierLine(AutoOpMode.Position.InitalPose.pose, AutoOpMode.Position.RowOne.pose))
				.build();
		drive.setStartingPose(AutoOpMode.Position.InitalPose.pose);
	}


	@Override
	public void loop() {
		drive.followPath(path);
	}
}
