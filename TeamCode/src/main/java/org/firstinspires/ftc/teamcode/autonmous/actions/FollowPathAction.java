package org.firstinspires.ftc.teamcode.autonmous.actions;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;

import java.util.ArrayList;
import java.util.Arrays;

public class FollowPathAction extends Action {
	ArrayList<PathChain> paths;
	Follower follower;
	int currentPath = 0;

	public FollowPathAction(Follower follower, PathChain... paths) {
		this.follower = follower;
		this.paths = new ArrayList<>(Arrays.asList(paths));
	}

	@Override
	public boolean run() {
		follower.followPath(paths.get(currentPath));
		if (!follower.isBusy()) {
			currentPath += 1;
		}

		return currentPath <= paths.size();
	}
}
