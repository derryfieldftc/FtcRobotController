package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

/**
 * A way to represnt both the turret and the pose of the robot, the rotation is relative to the front of the robot
 */
public class TurretPose2d {
	public double rotation;
	public Pose2d pose2d;

	/**
	 * @param pose2d   Robot position
	 * @param rotation inital rotation
	 */
	public TurretPose2d(Pose2d pose2d, double rotation) {
		this.pose2d = pose2d;
		this.rotation = rotation;
	}

	/**
	 * Says it all in the name really
	 *
	 * @param target
	 * @return angle of the turret to the target relative to the robot
	 */
	public double getTurretAngleToTargetRelativeToRobot(Vector2d target) {
		double angle = Math.atan2((target.y - pose2d.position.y), (target.x - pose2d.position.x));
		return angle - pose2d.heading.toDouble(); // Subtract because angle is relative to forward on the robot
	}
}
