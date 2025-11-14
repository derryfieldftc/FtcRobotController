package org.firstinspires.ftc.teamcode.autonmous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.autonmous.actions.Action;
import org.firstinspires.ftc.teamcode.autonmous.actions.ParallelAction;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.Arrays;
import java.util.Stack;

public abstract class AutoOpMode extends OpMode {
	public Follower drive;

	public enum Positions {
		InitalPose (new Pose(50, 12, 3 * Math.PI / 4)), //Make sure all of these are correct
		RowOne (new Pose(24, 36, Math.PI / 2)),
		RowTwo (new Pose(24, 60, Math.PI / 2)),
		RowThree (new Pose(24, 84, Math.PI / 2)),
		Lever (new Pose(15, 70, Math.PI)),
		CloseShot (new Pose(60, 72, Math.PI)),
		FarShot (new Pose(60, 24, Math.PI));

		public Pose pose;
		Positions(Pose pose) {
			this.pose = pose;
		}
	}

	/**
	 * Use this as opposed to the usual init(), this is so the drivetrain and other member variables are instantiated
	 */
	abstract public void autoInit();

	@Override
	public final void init() {
		// Do our pre-stuff here
		drive = Constants.createFollower(hardwareMap);

		// Call subclasses init method
		autoInit();
	}

}
