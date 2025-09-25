/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.GamepadManager;
import org.firstinspires.ftc.teamcode.robot.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.PID;
import org.firstinspires.ftc.teamcode.robot.Pose;
import org.firstinspires.ftc.teamcode.robot.Task;
import org.firstinspires.ftc.teamcode.robot.TaskList;

import java.util.ArrayList;

@TeleOp(name = "TestDSTasks", group = "Drivetrain")
@Disabled
public class TestDSTasks extends OpMode
		//
{
	enum Mode {
		MANUAL,
		AUTONOMOUS
	}

	Mode currMode;

	// Declare OpMode members.
	private ElapsedTime runtime = new ElapsedTime();
	private double start_time;

	private Drivetrain drivetrain;

	private PID pid = null;

	ArrayList<Task> tasks = null;
	Task currentTask;

	Pose initPose = null;

	boolean turboMode = false;

	private GamepadManager enhanced1 = null;

	private static final double SNAIL_FACTOR = 0.3;

	/*
	 * Code to run ONCE when the driver hits INIT
	 */
	@Override
	public void init() {
		String path = "/sdcard/FIRST/auto/tasks.txt";
		RobotLog.d("TIE: ds_log created (%s)", path);
		telemetry.addData("Status", "Initialized");

		// create driveTrain Object.
		drivetrain = new Drivetrain(hardwareMap, this);

		// set initial pose of drive train.
		initPose = new Pose(0, 0, 0);
		drivetrain.setPose(initPose);

		// get and set PID controllers for x, y, and theta directions.
		pid = PID.importPID("/sdcard/FIRST/pid/pid.txt");
		drivetrain.setPIDX(pid);
		pid = PID.importPID("/sdcard/FIRST/pid/pid.txt");
		drivetrain.setPIDY(pid);
		pid = PID.importPID("/sdcard/FIRST/pid/pid.txt");
		drivetrain.setPIDTheta(pid);
		RobotLog.d("TIE: imported PID values for x, y, and theta.");

		// import tasks.
		currentTask = null;
		tasks = new ArrayList<>();
		tasks = TaskList.importTasks(path);
		RobotLog.d("TIE: imported tasks");

		// display.
		RobotLog.d("TIE: " + tasks.toString());

		// current mode of this op mode.
		currMode = Mode.MANUAL;

		// use an enhanced gamepad to keep track of button presses.
		enhanced1 = new GamepadManager(gamepad1);
	}

	/*
	 * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
	 */
	@Override
	public void init_loop() {
	}

	/*
	 * Code to run ONCE when the driver hits START
	 */
	@Override
	public void start() {
		runtime.reset();
		start_time = runtime.milliseconds();
	}

	/*
	 * Code to run REPEATEDLY after the driver hits START but before they hit STOP
	 */
	@Override
	public void loop() {
		// refresh pose.
		drivetrain.refreshPose();

		// refresh gamepad data.
		enhanced1.poll();

		// reset odometry system?
		if (enhanced1.justPressed(GamepadManager.Button.X)) {
			// reset the odometry system.
			drivetrain.resetOdometry();

			// reset init pose.
			drivetrain.setPose(initPose);
		}

		// enable turbo mode?
		if (enhanced1.justPressed(GamepadManager.Button.B)) {
			// toggle between snail mode (slower) and regular mode
			turboMode = !turboMode;
		}

		if (enhanced1.justPressed(GamepadManager.Button.DPAD_UP)) {
			boolean val = drivetrain.getMotorCorrectionEnabled();
			val = !val;
			drivetrain.setMotorCorrectionEnabled(val);
		}

		if (enhanced1.justPressed(GamepadManager.Button.DPAD_DOWN)) {
			toggleAuto();
		}

		if (currMode == Mode.AUTONOMOUS) {
			// check current state.
			switch (drivetrain.getState()) {
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
						drivetrain.setState(Drivetrain.State.IDLE);
					}
					break;
				case DELAYING:
					double milli = runtime.milliseconds();
//                    RobotLog.d(String.format("TIE: DELAYING - milli = %.2f", milli));
//                    RobotLog.d(String.format("TIE: DELAYING - period = %.2f", (double)currentTask.getPeriod()));

					if (milli > (double) currentTask.getPeriod()) {
						// timer has expired.
						// switch back to idle state.
						drivetrain.setState(Drivetrain.State.IDLE);
						RobotLog.d("TIE: Done DELAYING.");
					}
					break;
			}
		} else {
			// get driver input.
			// also get driver input and drive robot.
			double drive = turboMode ? -gamepad1.left_stick_y : -SNAIL_FACTOR * gamepad1.left_stick_y;
			double strafe = turboMode ? gamepad1.left_stick_x : SNAIL_FACTOR * gamepad1.left_stick_x;
			double turn = turboMode ? -gamepad1.right_stick_x : -SNAIL_FACTOR * gamepad1.right_stick_x;
			drivetrain.drive(drive, strafe, turn);
		}

		// update display.
		telemetry.addData("Instructions", "Dpad down toggle auto");
		telemetry.addData("Mode", currMode);
		// display current waypoint.
		telemetry.addData("Current Waypoint", drivetrain.getCurrentWaypoint());

		// Show the elapsed game time and wheel power.
		telemetry.addData("Turbo Mode", turboMode);
		telemetry.addData("Motor Correction Enabled", drivetrain.getMotorCorrectionEnabled());

		// encoder data
		telemetry.addData("x", drivetrain.pose.x);
		telemetry.addData("y", drivetrain.pose.y);
		telemetry.addData("theta", drivetrain.pose.theta);

		// update telemetry.
		telemetry.update();
	}

	/*
	 * Code to run ONCE after the driver hits STOP
	 */
	@Override
	public void stop() {
		RobotLog.d("TIE: ds_log closed.");
	}

	public void toggleAuto() {
		if (currMode == Mode.AUTONOMOUS) {
			currMode = Mode.MANUAL;
		} else {
			currMode = Mode.AUTONOMOUS;
		}
	}
}
