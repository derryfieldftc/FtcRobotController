package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class RobotVoltageSensor extends RobotPart {
	VoltageSensor voltageSensor;

	// TODO! actually get empirical values
	enum VoltageLevel {
		Perfect(13.0, 1),
		Good(12.0, 1),
		Average(11.0, 1),
		Worrisome(10.0, .9),
		Terrible(9.0, .7);

		public final double voltage;
		public final double safetyCoefficient;

		VoltageLevel(double voltage, double safetyCoefficient) {
			this.voltage = voltage;
			this.safetyCoefficient = safetyCoefficient;
		}
	}

	public RobotVoltageSensor(OpMode opMode) {
		super(opMode);
		voltageSensor = hardwareMap.voltageSensor.iterator().next();
	}
}
