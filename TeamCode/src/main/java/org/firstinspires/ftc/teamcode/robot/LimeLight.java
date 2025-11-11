package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class LimeLight extends RobotPart {
	Limelight3A ll;
	public enum LimeLightMode { // <3 enums
		Default (0),
		AprilTag (1),
		Localization (2),
		Color (3);

		int pipeline;

		LimeLightMode(int pipeline) {
			this.pipeline = pipeline;
		}
	}

	public LimeLight(OpMode opMode) {
		super(opMode);
	}

	public void init() {
		ll = (Limelight3A) hardwareMap.get(Robot.Part.LimeLight.type, Robot.Part.LimeLight.name);
		ll.start();
	}

	public LimeLight setMode(LimeLightMode mode) {
		ll.pipelineSwitch(mode.pipeline);
		return this;
	}

	public LLResult getResults() {
		return ll.getLatestResult();
	}

	public void debugSnapshot() {
		ll.captureSnapshot(String.valueOf(opMode.getRuntime()));
	}


}
