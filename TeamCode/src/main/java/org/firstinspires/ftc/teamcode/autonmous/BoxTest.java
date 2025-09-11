package org.firstinspires.ftc.teamcode.autonmous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robot.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.Pose;

import java.util.ArrayList;

@Autonomous
public class BoxTest extends OpMode {
	Drivetrain drivetrain;
	ArrayList<Pose> poses = new ArrayList<>();
	Pose initPose = new Pose(0, 0, 0);

	@Override
	public void init() {
		drivetrain = new Drivetrain(hardwareMap, this);
		drivetrain.setPose(initPose);

		poses.add(new Pose(0, 50, 0));
		poses.add(new Pose(50, 50, 0));
		poses.add(new Pose(50, 0, 0));
		poses.add(new Pose(0, 0, 0));
	}

	@Override
	public void loop() {
		drivetrain.refreshPose();

		if (drivetrain.waypoint != null) {
			if (!drivetrain.applyCorrection()) {
				telemetry.addData("wp:", drivetrain.waypoint.dump());
				telemetry.addData("pose: ", drivetrain.getPose().dump());
			} else {
				drivetrain.stop();
				drivetrain.clearWaypoint();
			}
		} else if (!poses.isEmpty()) {
			drivetrain.setWaypoint(poses.get(0));
			poses.remove(0);
		}
	}
}
