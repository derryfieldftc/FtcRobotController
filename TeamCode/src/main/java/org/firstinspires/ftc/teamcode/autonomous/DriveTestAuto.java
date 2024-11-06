package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.OpModeGroups;
import org.firstinspires.ftc.teamcode.RobotTask;
import org.firstinspires.ftc.teamcode.tasks.DelayTask;
import org.firstinspires.ftc.teamcode.tasks.SimpleDriveTask;

@Autonomous(name = "auto test", group = OpModeGroups.AUTO)
public class DriveTestAuto extends TaskAutonomous {
	@Override
	public RobotTask[] createTasksList() {
		return new RobotTask[] {
				new SimpleDriveTask(this).forward(.3).time(1000),
				new DelayTask(this).waitMillis(1000),
				new SimpleDriveTask(this).forward(-.3).time(1000),
				new DelayTask(this).waitMillis(1000),
				new SimpleDriveTask(this).strafe(.3).time(1000),
				new DelayTask(this).waitMillis(1000),
				new SimpleDriveTask(this).strafe(-.3).time(1000)
		};
	}
}
