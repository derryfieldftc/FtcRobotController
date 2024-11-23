package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.OpModeGroups;
import org.firstinspires.ftc.teamcode.RobotTask;
import org.firstinspires.ftc.teamcode.tasks.CompSetup;
import org.firstinspires.ftc.teamcode.tasks.SimpleDriveTask;

@Autonomous(name = "RevParkAuto", group = OpModeGroups.AUTO)
public class RevParkAuto extends TaskAutonomous {
	@Override
	public RobotTask[] createTasksList() {
		return new RobotTask[] {
				new CompSetup(this),
				new SimpleDriveTask(this).forward(-.2).time(10000),
		};
	}
}
