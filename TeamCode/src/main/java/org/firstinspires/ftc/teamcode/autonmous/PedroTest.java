package org.firstinspires.ftc.teamcode.autonmous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonmous.actions.Action;
import org.firstinspires.ftc.teamcode.autonmous.actions.FollowPathAction;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous
public class PedroTest extends AutoOpMode {
	PathChain forward;
	Follower follower;
	Action action;
	boolean running = true;

	@Override
	public void autoInit() {
		follower = Constants.createFollower(hardwareMap);

		forward = follower.pathBuilder().addPath(new BezierLine(new Pose(0, 72, 0), new Pose(96, 96, 0)))
				.build();

		action = new FollowPathAction(follower, forward);
	}

	@Override
	public void loop() {
		if (running)
			running = action.run();
	}
}
