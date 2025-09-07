package org.firstinspires.ftc.teamcode.plugin.autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.OpModeGroups;
import org.firstinspires.ftc.teamcode.plugin.RobotTask;
import org.firstinspires.ftc.teamcode.plugin.tasks.DelayTask;
import org.firstinspires.ftc.teamcode.plugin.tasks.ExampleTask;

@Autonomous(name = "ExampleAutonomous", group = OpModeGroups.AUTO)
public class ExampleAutonomous extends TaskAutonomous {
	public RobotTask[] createTasksList() {
		RobotTask moveForward = new ExampleTask(this).speed(0.3).seconds(1);
		RobotTask moveBackward = new ExampleTask(this).speed(-0.3).seconds(1);

		return new RobotTask[] {
				moveForward,
				new DelayTask(this).waitMillis(1000),
				moveBackward,
		};
	}
}