package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "RawOdom")
@Disabled
public class RawOdomPodData extends OpMode {
	DcMotor encoderL, encoderR, encoderAux;

	@Override
	public void init() {
		encoderL = hardwareMap.dcMotor.get("motorBL");
		encoderR = hardwareMap.dcMotor.get("motorFL");
		encoderAux = hardwareMap.dcMotor.get("motorBR");

	}

	@Override
	public void loop() {
		telemetry.addData("l", encoderL.getCurrentPosition());
		telemetry.addData("r", encoderR.getCurrentPosition());
		telemetry.addData("a", encoderAux.getCurrentPosition());
		telemetry.update();
	}
}
