package org.firstinspires.ftc.teamcode.robot;

/**
 * A way to represnt both the turret and the pose of the robot, the rotation is relative to the front of the robot
 */
public class TurretPose2d {
	public double rotation;
	public Pose pose;

	/**
	 * @param pose2d   Robot position
	 * @param rotation inital rotation
	 */
	public TurretPose2d(Pose pose2d, double rotation) {
		this.pose = pose2d;
		this.rotation = rotation;
	}

	/**
	 * Says it all in the name really
	 *
	 * @param target
	 * @return angle of the turret to the target relative to the robot
	 */
	public double getTurretAngleToTargetRelativeToRobot(Pose target) {
		double angle = Math.atan2((target.y - pose.y), (target.x - pose.x));
		return angle - pose.theta; // Subtract because angle is relative to forward on the robot
	}
}
