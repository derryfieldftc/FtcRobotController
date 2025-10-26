package org.firstinspires.ftc.teamcode.autonmous;

import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.Vector2d;

public class AutoFunctions {
	public static Pose2dDual<Arclength> mirror(Pose2dDual<Arclength> pose) {
		return new Pose2dDual<>(pose.position.x.unaryMinus(), pose.position.y, pose.heading.inverse());
	}

	public static enum RedPoses {
		Init (new Pose2d(20, -57, Math.PI / 4)),
		Row2CollectionSetup (new Pose2d(new Vector2d(35, -5), 0)),
		Row2Collection (new Pose2d(new Vector2d(41, -5), 0)),
		Lever (new Pose2d(new Vector2d(62, -2), 0)),
		ShootableMid (new Pose2d(new Vector2d(10, 0), Math.PI)),
		AwayFromLever (new Pose2d(new Vector2d(39, 15), 0)),
		Row3 (new Pose2d(new Vector2d(50, 16), 0)),
		ShootableMid2 (new Pose2d(new Vector2d(10, 0), Math.PI)),
		Row1Collection (new Pose2d(new Vector2d(46, -35), 0)),
		FarShootingPosition (new Pose2d(new Vector2d(20, -57), Math.PI));

		public final Pose2d pose;

		RedPoses(Pose2d pose) {
			this.pose = pose;
		}
	};

	public static enum BluePoses {
		Init (new Pose2d(-20, -57, 3 * Math.PI / 4)),
		Row2CollectionSetup (new Pose2d(new Vector2d(-35, -5), Math.PI)),
		Row2Collection (new Pose2d(new Vector2d(-41, -5), Math.PI)),
		Lever (new Pose2d(new Vector2d(-62, -2), Math.PI)),
		ShootableMid (new Pose2d(new Vector2d(-10, 0), 0)),
		AwayFromLever (new Pose2d(new Vector2d(-39, 15), Math.PI)),
		Row3 (new Pose2d(new Vector2d(-50, 16), Math.PI)),
		ShootableMid2 (new Pose2d(new Vector2d(-10, 0), 0)),
		Row1Collection (new Pose2d(new Vector2d(-46, -35), Math.PI)),
		FarShootingPosition (new Pose2d(new Vector2d(-20, -57), 0));

		public final Pose2d pose;
		BluePoses(Pose2d pose) {
			this.pose = pose;
		};
	};
}
