package org.firstinspires.ftc.teamcode.opmodes;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import java.io.File;
import java.io.PrintWriter;

@TeleOp(name = "Color Sensor Data Collection")
public class ColorSensorTrainingData extends OpMode {
	ColorSensor colorSensor;
	File file;
	PrintWriter writer;

	@Override
	public void init() {
		colorSensor = hardwareMap.colorSensor.get("intakeColorSensor");
		file = new File("/sdcard/FIRST/training");
		try {
			file.createNewFile();
			writer = new PrintWriter(file);
		} catch (Exception e) {
			throw new RuntimeException(e);
		}
		writer.println("red, green, blue, alpha");

	}

	@Override
	public void loop() {
		if (gamepad1.a) {
			float[] hsv = new float[3];
			Color.RGBToHSV(colorSensor.red(), colorSensor.green(), colorSensor.blue(), hsv);
			String string = String.format("%.3f, %,3f, %.3f", hsv[0], hsv[1], hsv[2]);
			telemetry.addLine(string);
			telemetry.update();
			writer.println(string);
		}

		if (gamepad1.b) {
			writer.flush();
			writer.close();
		}
	}
}
