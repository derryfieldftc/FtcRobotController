package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class ClearLimeLightSnapShots extends OpMode {
	Limelight3A limelight3A;

	@Override
	public void init() {
		limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
		limelight3A.deleteSnapshots();
		telemetry.addLine("Done!");

	}

	@Override
	public void loop() {
		stop();
	}
}
