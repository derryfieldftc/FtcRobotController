package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.configuration.DeviceConfiguration;

@TeleOp
public class I2CTest extends OpMode {
	ColorSensor sensor;
	@Override
	public void init() {
		sensor = hardwareMap.colorSensor.get("cs");

	}

	@Override
	public void loop() {

	}
}
