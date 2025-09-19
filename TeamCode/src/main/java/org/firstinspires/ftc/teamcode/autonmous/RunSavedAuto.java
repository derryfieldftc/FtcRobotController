package org.firstinspires.ftc.teamcode.autonmous;

import static org.firstinspires.ftc.teamcode.robot.Drivetrain.State.IDLE;
import static org.firstinspires.ftc.teamcode.robot.Drivetrain.State.valueOf;
import static org.firstinspires.ftc.teamcode.robot.Task.Type.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.robot.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.Task;
import org.firstinspires.ftc.teamcode.robot.TaskList;

import java.util.ArrayList;

// Eventually add selection in the init phase for which OpMode to run
@Autonomous(name = "RunSavedAuto")
public class RunSavedAuto extends OpMode {

	public Drivetrain drivetrain;
	public ElapsedTime runtime;
	public ArrayList<Task> tasks;
	Task currentTask;

	@Override
	public void init() {
		drivetrain = new Drivetrain(this.hardwareMap, this);
		tasks = TaskList.importTasks("/sdcard/FIRST/auto/tasks.txt");
		runtime = new ElapsedTime();
		drivetrain.resetOdometry();
	}

	@Override
	public void loop() {
		switch(drivetrain.getState()) {
			case IDLE:
				// are there any tasks to process?
				if (tasks.size() > 0) {
					currentTask = tasks.get(0);
					tasks.remove(0);
					switch (currentTask.getType()) {
						case WAYPOINT:
							drivetrain.setWaypoint(currentTask.getPose());
							drivetrain.setState(Drivetrain.State.NAVIGATING);
							RobotLog.d("TIE: WAYPOINT");
							break;
						case DELAY:
							// reset timer.
							runtime.reset();
							// switch to delaying state.
							drivetrain.setState(Drivetrain.State.DELAYING);
							RobotLog.d("TIE: DELAY");
							break;
					}
					break;
				}
				break;
			case NAVIGATING:
				// we are currently auto-navigating.
				// apply correction to move towards current waypoint.
				if (drivetrain.applyCorrection()) {
					// if applyCorrection() returns true, then we have arrived at the waypoint.
					drivetrain.stop();
					RobotLog.d("TIE: made it to waypoint. Clearing waypoint...");
					drivetrain.clearWaypoint();
					RobotLog.d("TIE: cleared!");
					// go back to idle state.
					drivetrain.setState(IDLE);
				}
				break;
			case DELAYING:
				double milli = runtime.milliseconds();
//                    RobotLog.d(String.format("TIE: DELAYING - milli = %.2f", milli));
//                    RobotLog.d(String.format("TIE: DELAYING - period = %.2f", (double)currentTask.getPeriod()));

				if (milli > (double)currentTask.getPeriod()) {
					// timer has expired.
					// switch back to idle state.
					drivetrain.setState(IDLE);
					RobotLog.d("TIE: Done DELAYING.");
				}
				break;
		}
	}
}