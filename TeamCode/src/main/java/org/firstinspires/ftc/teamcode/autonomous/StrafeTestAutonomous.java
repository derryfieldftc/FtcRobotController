package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.OpModeGroups;
import org.firstinspires.ftc.teamcode.RobotTask;
import org.firstinspires.ftc.teamcode.tasks.OdomStrafe;

@Autonomous(name="Strafe&drivetest", group = OpModeGroups.TESTS)
public class StrafeTestAutonomous extends TaskAutonomous {
	public RobotTask[] createTasksList() {
		OdomStrafe odomStrafe = new OdomStrafe(this);
		return new RobotTask[] {
			odomStrafe
		};
	}
}
