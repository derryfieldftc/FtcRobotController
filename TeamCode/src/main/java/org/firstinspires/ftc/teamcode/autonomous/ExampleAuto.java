package org.firstinspires.ftc.teamcode.autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.OpModeGroups;
import org.firstinspires.ftc.teamcode.RobotTask;
import org.firstinspires.ftc.teamcode.tasks.DelayTask;
import org.firstinspires.ftc.teamcode.tasks.ExampleTask;

@Autonomous(name = "ExampleAutonomous", group = OpModeGroups.AUTO)
public class ExampleAuto extends TaskAutonomous {
	public RobotTask[] createTasksList() {
        return new RobotTask[] {
                new ExampleTask(this).speed(0.3).seconds(1),
				new DelayTask(this).waitMillis(2000),
                new ExampleTask(this).speed(-0.3).seconds(1),
		};
	}
}