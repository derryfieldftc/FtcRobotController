package org.firstinspires.ftc.teamcode.autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.OpModeGroups;
import org.firstinspires.ftc.teamcode.RobotTask;
import org.firstinspires.ftc.teamcode.tasks.DelayTask;
import org.firstinspires.ftc.teamcode.tasks.ExampleTask;

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