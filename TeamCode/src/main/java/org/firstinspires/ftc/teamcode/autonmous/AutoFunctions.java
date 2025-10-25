package org.firstinspires.ftc.teamcode.autonmous;

import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.Pose2dDual;

public class AutoFunctions {
	public static Pose2dDual<Arclength> mirror(Pose2dDual<Arclength> pose) {
		return new Pose2dDual<>(pose.position.x.unaryMinus(), pose.position.y, pose.heading.inverse());
	}
}
