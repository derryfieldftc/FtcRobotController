package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.RobotPart;

@TeleOp
public class ValidateRobotPartsExistence extends OpMode {

	@Override
	public void init() {
		if (RobotPart.Part.validateAll(hardwareMap)) {
			telemetry.addLine("All devices are valid");
		} else {
			telemetry.addLine("Some Devices are not valid");
			for (RobotPart.Part part : RobotPart.Part.values()) {
				if (!part.exists(hardwareMap)) {
					telemetry.addLine(part.name);
				};
			}
		}
	}

	@Override
	public void loop() {

	}
}
