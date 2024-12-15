package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.OpModeGroups;
import org.firstinspires.ftc.teamcode.RobotTask;
import org.firstinspires.ftc.teamcode.tasks.OdometryRunToPositionTask;

@Autonomous(name = "BinaryBots Auto", group = OpModeGroups.AUTO)
public class BinaryBotsBackwardsAuto extends TaskAutonomous {
	@Override
	public RobotTask[] createTasksList() {
		return new RobotTask[] {
				new OdometryRunToPositionTask(this, 0, 24),
		};
	}
}
