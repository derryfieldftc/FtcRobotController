package org.firstinspires.ftc.teamcode.plugins;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RobotPlugin;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.PrintWriter;

public class FSTest extends RobotPlugin {
	Environment environment;
	OpMode opMode;
	PrintWriter writer;
	File motorData;
	boolean fileExists = true;

	public FSTest(OpMode opMode) {
		environment = new Environment();
		this.opMode = opMode;
		motorData = new File(environment.getDataDirectory().toURI().toString() + "motorPos.txt");
		if (!motorData.exists()) {
			try {
				fileExists = motorData.createNewFile();
			} catch (IOException e) {
				throw new RuntimeException(e);
			}
		}
		if (motorData.isFile() && fileExists) {
			try {
				writer = new PrintWriter(motorData);
			} catch (FileNotFoundException e) {
				throw new RuntimeException(e);
			}
		}
	}

	@Override
	public void loop() {
		if (fileExists) {
			writer.println("this is a new line");
		}
	}
}
