package org.firstinspires.ftc.teamcode.autonomous_opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.OpModeGroups;
import org.firstinspires.ftc.teamcode.RobotTask;
import org.firstinspires.ftc.teamcode.tasks.DelayTask;
import org.firstinspires.ftc.teamcode.tasks.ExampleTask;

@Autonomous(name="ExampleAutonomousOpMode", group = OpModeGroups.SAMPLES)
public class ExampleAutonomousOpMode extends AutonomousOpMode {
	public RobotTask[] createTasksList() {
		ExampleTask exampleTask = new ExampleTask(this);
		DelayTask delayTask = new DelayTask(this);
		return new RobotTask[] { exampleTask.run(0.5, 1), delayTask.run(1000), exampleTask.run(0.5, 1) };
	}
}