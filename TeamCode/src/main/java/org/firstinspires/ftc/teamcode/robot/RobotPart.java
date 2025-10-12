package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Class to extend for all robot parts
 */
public class RobotPart {
	public OpMode opMode;
	public HardwareMap hardwareMap;
	public Telemetry telemetry;
	public VoltageSensor voltageSensor;

	public RobotPart(OpMode opMode) {
		this.opMode = opMode;
		hardwareMap = opMode.hardwareMap;
		telemetry = opMode.telemetry;
		voltageSensor = hardwareMap.voltageSensor.iterator().next();
	}
}
