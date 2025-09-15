package org.firstinspires.ftc.teamcode.autonmous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robot.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.Tasks.OldTask;
import org.firstinspires.ftc.teamcode.robot.TaskList;

import java.util.ArrayList;

public class TaskListTest extends OpMode {
	TaskList list;
	Drivetrain drivetrain;

	@Override
	public void init() {
		list = new TaskList();
		list.importTasks("/sdcard/FIRST/tasks.txt");
		drivetrain = new Drivetrain(hardwareMap, this);
		drivetrain.resetOdometry();
	}

	@Override
	public void loop() {
		ArrayList<OldTask> poses = list.get();

	}
}
