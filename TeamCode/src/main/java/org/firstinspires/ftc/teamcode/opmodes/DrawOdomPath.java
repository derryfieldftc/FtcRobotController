package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.GamepadManager;
import org.firstinspires.ftc.teamcode.OpModeGroups;
import org.firstinspires.ftc.teamcode.plugin.plugins.MecanumDrive;
import org.firstinspires.ftc.teamcode.plugin.tasks.DelayTask;
import org.firstinspires.ftc.teamcode.robot.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.Pose;
import org.firstinspires.ftc.teamcode.robot.Task;

import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.concurrent.atomic.AtomicReference;

@TeleOp(name = "Draw Odom Path", group = OpModeGroups.TESTS)
public class DrawOdomPath extends OpMode {
	Drivetrain drivetrain;
	MecanumDrive mecanumDrive;
	GamepadManager mgamepad;
	ArrayList<Task> tasks = new ArrayList<>();

	@Override
	public void init() {
		hardwareMap.getClass(); //Needed to appease the java garbage collecting gods
		mgamepad = new GamepadManager(gamepad1);
		drivetrain = new Drivetrain(hardwareMap, this);
		mecanumDrive = new MecanumDrive(this);
		mecanumDrive.init();
	}

	@Override
	public void loop() {
		mecanumDrive.loop();
		drivetrain.refreshPose();
		mgamepad.poll();

		if (mgamepad.justPressed(GamepadManager.Button.Y)) {
			tasks = new ArrayList<>();
			drivetrain.resetOdometry();
		}

		if (mgamepad.justPressed(GamepadManager.Button.A)) {
			// have to make an actual copy <3 thx java very cool
			Pose newPose = new Pose(drivetrain.getPose().x, drivetrain.getPose().y, drivetrain.getPose().theta);
			tasks.add(new Task(Task.Type.WAYPOINT, newPose));
		}

		if (mgamepad.justPressed(GamepadManager.Button.B)) {
			tasks.remove(tasks.size() - 1);
		}

		if (mgamepad.justPressed(GamepadManager.Button.X)) {
			tasks.add(new Task(Task.Type.DELAY, 1000));
		}

		if (mgamepad.justPressed(GamepadManager.Button.START)) {
			try {
				saveToFile();
			} catch (IOException e) {
				telemetry.addLine(e.toString());
			}
		}

		Pose tempPose = drivetrain.getPose();
		telemetry.addLine(String.format("x: %.3f y: %.3f t: %.3f", tempPose.x, tempPose.y, tempPose.theta));
		for (int i = tasks.size() - 1; i >= 0; i--) {
			if (tasks.get(i).getType() == Task.Type.DELAY) {
				telemetry.addData("("+i+")", tasks.get(i).getPeriod());
			} else if (tasks.get(i).getType() ==Task.Type.WAYPOINT) {
				Pose pose = tasks.get(i).getPose();
				telemetry.addLine(String.format("(%d) x: %.3f y: %.3f t: %.3f", i, pose.x, pose.y, pose.theta));
			}
		}
		telemetry.update();
	}

	/**
	 * Would just use a TaskList now
	 * @throws IOException
	 */
	private void saveToFile() throws IOException {
		AtomicReference<String> filename = new AtomicReference<>("tasks.txt");

		// She might be on to something
		Iterator<String> allNames = hardwareMap.getAllNames(DistanceSensor.class).stream().iterator();
		allNames.forEachRemaining((name) -> {
			if (name.contains("FNPRE_")) {
				filename.set(name.substring(6));
			}
		});

		File file = new File("/sdcard/FIRST/auto/" + filename.get());
		file.createNewFile();
		PrintWriter writer = new PrintWriter(file);
		for (Task task : tasks) {
			if (task.getType() == Task.Type.DELAY) {
				writer.println(String.format("DELAY: %d", task.getPeriod()));
			} else if (task.getType() == Task.Type.WAYPOINT) {
				Pose pose = task.getPose();
				writer.println(String.format("WAYPOINT: %.6f %.6f %.6f", pose.x, pose.y, pose.theta));
			}
		}
		writer.flush();
		writer.close();
		telemetry.addData("Saved, filename: ", file.getAbsoluteFile());
	}
}
