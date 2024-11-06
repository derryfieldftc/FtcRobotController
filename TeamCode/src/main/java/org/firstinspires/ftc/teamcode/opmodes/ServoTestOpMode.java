package org.firstinspires.ftc.teamcode.opmodes;

import android.util.JsonReader;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpModeGroups;
import org.firstinspires.ftc.teamcode.RobotPlugin;
import org.firstinspires.ftc.teamcode.plugins.ServoTest;
import org.json.JSONException;
import org.json.JSONObject;
import org.json.JSONTokener;

import java.io.FileNotFoundException;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.Scanner;

@TeleOp(name = "Servo Test", group = OpModeGroups.TESTS)
public class ServoTestOpMode extends PluginOpMode {
	public RobotPlugin[] initPlugins() {
		FileReader fileReader;
		try {
			fileReader = new FileReader("/sdcard/servos.txt");
		} catch (FileNotFoundException e) {
			throw new RuntimeException(e);
		}
		Scanner content = new Scanner(fileReader).useDelimiter("\n");
		ArrayList<String> servoNames = new ArrayList<>();
		// read every line into servoNames
		while (content.hasNext()) {
			servoNames.add(content.next());
		}

		RobotPlugin tester = new ServoTest(this, servoNames.toArray(new String[0]));
        return new RobotPlugin[] { tester };
	}
}