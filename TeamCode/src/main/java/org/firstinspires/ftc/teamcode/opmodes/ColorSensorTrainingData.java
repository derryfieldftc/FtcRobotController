package org.firstinspires.ftc.teamcode.opmodes;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.robot.RobotPart;

import java.io.File;
import java.io.PrintWriter;

@TeleOp(name = "Color Sensor Data Collection")
public class ColorSensorTrainingData extends OpMode {
	ColorSensor colorSensor0, colorSensor1, colorSensor2;
	File file;
	PrintWriter writer;

	@Override
	public void init() {
		colorSensor0 = hardwareMap.colorSensor.get(RobotPart.Part.SpindexerColor0.name);
		colorSensor1 = hardwareMap.colorSensor.get(RobotPart.Part.SpindexerColor1.name);
		colorSensor2 = hardwareMap.colorSensor.get(RobotPart.Part.SpindexerColor2.name);

		file = new File("/sdcard/FIRST/training");
		try {
			file.createNewFile();
			writer = new PrintWriter(file);
		} catch (Exception e) {
			throw new RuntimeException(e);
		}
		writer.println("h, s, v");

	}

	@Override
	public void loop() {
		float[] hsv = new float[3];

		Color.RGBToHSV(colorSensor2.red(), colorSensor2.green(), colorSensor2.blue(), hsv);
		String string2 = String.format("h %0.3f, s %0.3f, v %0.3f", hsv[0], hsv[1], hsv[2]);
		telemetry.addLine("2");
		telemetry.addLine(String.format("r %0.3f, g %0.3f, b %0.3f", colorSensor2.red(), colorSensor2.green(), colorSensor2.blue()));
		telemetry.addLine(string2);

		if (gamepad1.x) {
			writer.println(string2);
		}

		Color.RGBToHSV(colorSensor1.red(), colorSensor1.green(), colorSensor1.blue(), hsv);
		String string1 = String.format("%.3f, %,3f, %.3f", hsv[0], hsv[1], hsv[2]);
		telemetry.addLine("1");
		telemetry.addLine(String.format("r %0.3f, g %0.3f, b %0.3f", colorSensor1.red(), colorSensor1.green(), colorSensor1.blue()));
		telemetry.addLine(string1);

		if (gamepad1.b) {
			writer.println(string1);
		}

		Color.RGBToHSV(colorSensor0.red(), colorSensor0.green(), colorSensor0.blue(), hsv);
		String string0 = String.format("%.3f, %,3f, %.3f", hsv[0], hsv[1], hsv[2]);
		telemetry.addLine("0");
		telemetry.addLine(String.format("r %0.3f, g %0.3f, b %0.3f", colorSensor0.red(), colorSensor0.green(), colorSensor0.blue()));
		telemetry.addLine(string0);
		telemetry.update();

		if (gamepad1.a) {
			writer.println(string0);
		}

		if (gamepad1.y) {
			writer.flush();
			writer.close();
		}
	}
}
