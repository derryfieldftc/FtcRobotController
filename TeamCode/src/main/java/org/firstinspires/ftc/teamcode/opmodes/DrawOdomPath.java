package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GamepadManager;
import org.firstinspires.ftc.teamcode.OpModeGroups;
import org.firstinspires.ftc.teamcode.plugin.plugins.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.Pose;
import org.firstinspires.ftc.teamcode.robot.Task;

import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;

@TeleOp(name = "Draw Odom Path", group = OpModeGroups.TESTS)
public class DrawOdomPath extends OpMode {
	Drivetrain drivetrain;
	MecanumDrive mecanumDrive;
	GamepadManager mgamepad;
	ArrayList<Pose> poses = new ArrayList<>();

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
		mgamepad.poll();

		if (mgamepad.justPressed(GamepadManager.Button.Y)) {
			poses = new ArrayList<>();
			drivetrain.resetOdometry();
		}

		if (mgamepad.justPressed(GamepadManager.Button.A)) {
			poses.add(drivetrain.getPose());
		}

		if (mgamepad.justPressed(GamepadManager.Button.B)) {
			poses.remove(poses.size() - 1);
		}

		if (mgamepad.justPressed(GamepadManager.Button.START)) {
			try {
				saveToFile();
			} catch (IOException e) {
				telemetry.addLine(e.toString());
			}
		}

		for (int i = 0; i < poses.size(); i++) {
			Pose pose = poses.get(i);
			telemetry.addData("(" + i + ")", "x: " + pose.x + " y: " + pose.y + " t: " + pose.theta);
		}
		telemetry.update();
	}

	private void saveToFile() throws IOException {
		File file = new File("/sdcard/FIRST/" + this.getRuntime());
		file.createNewFile();
		PrintWriter writer = new PrintWriter(file);
		for (Pose pose:poses) {
			writer.println("WAYPOINT " + pose.x + " " + pose.y + " " + pose.theta);
			writer.println("DELAY 1000"); //hacky TODO! fix this later and implement the write to file method in the task class
		}
	}
}
