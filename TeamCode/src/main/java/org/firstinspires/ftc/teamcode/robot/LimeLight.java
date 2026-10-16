package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.localization.Localizer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class LimeLight extends RobotPart {
	public Limelight3A ll;

	public static class Tags {
		interface Tag {}
		public static class Red implements Tag {
			/**
			 * As seen from robot's perspective.
			 */
			public enum Tag {
				INIT_LEFT(34),
				INIT_MIDLEFT(35),
				INIT_MIDRIGHT(36),
				INIT_RIGHT(37),
				REVERSE_LEFT(30),
				REVERSE_MIDLEFT(31),
				REVERSE_MIDRIGHT(32),
				REVERSE_RIGHT(33);


				public int number;
				Tag(int number) {
					this.number = number;
				}

			}

		}
		public static class Blue implements Tag {
			/**
			 * As seen from robot's perspective.
			 */
			public enum Tag {
				INIT_LEFT(42),
				INIT_MIDLEFT(43),
				INIT_MIDRIGHT(44),
				INIT_RIGHT(45),
				REVERSE_LEFT(38),
				REVERSE_MIDLEFT(39),
				REVERSE_MIDRIGHT(40),
				REVERSE_RIGHT(41);


				public int number;
				Tag(int number) {
					this.number = number;
				}

			}

		}
	}

	public enum LimeLightMode { // <3 enums
		Default(0),
		AprilTag(1),
		Localization(2),
		Color(3);

		int pipeline;

		LimeLightMode(int pipeline) {
			this.pipeline = pipeline;
		}
	}

	public LimeLight(OpMode opMode) {
		super(opMode);
		ll = (Limelight3A) hardwareMap.get(RobotPart.Part.LimeLight.type, RobotPart.Part.LimeLight.name);
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

	// TODO! these
	public void updateLocalizer(Localizer localizer, double rotation) {
	}

	public void updateLocalizer(Localizer localizer) {
	}

}
