package org.firstinspires.ftc.teamcode.autonomous_opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.OpModeGroups;
import org.firstinspires.ftc.teamcode.RobotTask;
import org.firstinspires.ftc.teamcode.tasks.OdometryRunToPositionTask;

@Autonomous(name="TestAutoOp", group = OpModeGroups.TESTS)
public class TestAuto extends AutonomousOpMode {
	@Override
	public RobotTask[] createTasksList() {
		return new RobotTask[] { new OdometryRunToPositionTask(this,  2j, 0) };
	}
}
