package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpModeGroups;
import org.firstinspires.ftc.teamcode.RobotTask;
import org.firstinspires.ftc.teamcode.tasks.SimpleDriveTask;

@Autonomous(name = "\"park\"", group = OpModeGroups.AUTO)
public class ParkAuto extends TaskAutonomous {
	@Override
	public RobotTask[] createTasksList() {

		return new RobotTask[] {
				new SimpleDriveTask(this).forward(.1).time(7000),
		};
	}
}
