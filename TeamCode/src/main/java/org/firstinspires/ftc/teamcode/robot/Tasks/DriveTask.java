package org.firstinspires.ftc.teamcode.robot.Tasks;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robot.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.Pose;

import java.util.Scanner;

public class DriveTask implements Task {
	Pose pose;
	Drivetrain drivetrain;

	public DriveTask(Pose pose) {
		this.pose = pose;
		drivetrain = new Drivetrain();
	};

	@Override
	public String asString() {
		return "WAYPOINT " + pose.dump();
	}

	@Override
	public boolean isParseable(String input) {
		return input.contains("WAYPOINT");
	}

	//TODO! write a parser :sob:
	@Override
	public Task parse(String input) {
		double x, y, t;
		input = input.substring(8);
		Scanner parser = new Scanner(input);
		x = parser.nextDouble();
		y = parser.nextDouble();
		t = parser.nextDouble();
		return new DriveTask(new Pose(x, y, t));
	}

	@Override
	public void execute() {
		drivetrain.setWaypoint(pose);
		while (!drivetrain.applyCorrection()) {
			drivetrain.refreshPose();
		}
	}

	@Override
	public TaskType getType() {
		return TaskType.WAYPOINT;
	}
}
