package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.OpModeGroups;
import org.firstinspires.ftc.teamcode.RobotTask;
import org.firstinspires.ftc.teamcode.tasks.OdometryRunToPositionTask;

@Autonomous(name = "OdometryTestAutonomous", group = OpModeGroups.TESTS)
public class OdometryTestAutonomous extends TaskAutonomous {
	@Override
	public RobotTask[] createTasksList() {
		return new RobotTask[] {
				new OdometryRunToPositionTask(this, 24, 24)
		};
	}
}
