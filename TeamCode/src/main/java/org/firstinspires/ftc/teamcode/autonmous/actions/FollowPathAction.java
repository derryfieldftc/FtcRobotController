package org.firstinspires.ftc.teamcode.autonmous.actions;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.ArrayList;
import java.util.Arrays;

public class FollowPathAction extends Action {
	ArrayList<PathChain> paths;
	Follower follower;
	int currentPath = -1;

	public FollowPathAction(Follower follower, PathChain... paths) {
		this.follower = follower;
		this.paths = new ArrayList<>(Arrays.asList(paths));
	}

	@Override
	public boolean run() {
		follower.update();
		if (!follower.isBusy()) {
			currentPath += 1;
			if (currentPath == paths.size())
				return false;
			follower.followPath(paths.get(currentPath), true);
		}

		return currentPath <= paths.size();
	}
}
