package org.firstinspires.ftc.teamcode.autonomous_opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.OpModeGroups;
import org.firstinspires.ftc.teamcode.RobotTask;
import org.firstinspires.ftc.teamcode.tasks.DelayTask;
import org.firstinspires.ftc.teamcode.tasks.ExampleTask;

@Autonomous(name="ExampleAutonomousOpMode", group = OpModeGroups.SAMPLES)
public class ExampleAutonomousOpMode extends AutonomousOpMode {
	public RobotTask[] createTasksList() {
		return new RobotTask[] {
				new ExampleTask(this, .3, 1),
				new DelayTask(this, 2000),
				new ExampleTask(this, -.3, 1),
		};
	}
}