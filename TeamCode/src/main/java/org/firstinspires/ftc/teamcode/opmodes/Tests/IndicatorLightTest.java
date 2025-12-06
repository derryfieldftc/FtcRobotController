package org.firstinspires.ftc.teamcode.opmodes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.IndicatorLight;

@TeleOp
@Disabled
public class IndicatorLightTest extends OpMode {
	IndicatorLight light;

	@Override
	public void init() {
		light = new IndicatorLight(this, "RGB_Turret");
	}

	@Override
	public void loop() {
		light.setRawColor(-gamepad1.left_stick_y);
		telemetry.addData("color", -gamepad1.left_stick_y);

	}
}
