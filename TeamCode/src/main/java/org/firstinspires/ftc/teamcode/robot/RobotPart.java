package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

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

	// <3 enums as lookup tables... sure wish there were macros in java for compile
	// time existence checking
	/**
	 * List of robot parts.
	 * use Part.name when getting a part, so if a configuration is changed, this can
	 * be updated simply.
	 * Would recommend importing this as static
	 */
	@Configurable
	// Notice how doc can be added to variants to make them more understandable,
	// please use this liberally, but not excessively
	public enum Part {
		/**
		 * Front Right drive motor
		 */
		MotorFR("motorFR", DcMotor.class),
		/**
		 * Front Left drive motor
		 */
		MotorFL("motorFL", DcMotor.class),
		/**
		 * Back Right drive motor
		 */
		MotorBR("motorBR", DcMotor.class),
		/**
		 * Back Left drive motor
		 */
		MotorBL("motorBL", DcMotor.class),
		LimeLight("limelight", Limelight3A.class),
		// TODO! make this list exhaustive
		intakeMotor("intakeMotor", DcMotor.class),;
		public final String name;
		public final Class<? extends HardwareDevice> type;

		Part(String name, Class<? extends HardwareDevice> type) {
			this.name = name;
			this.type = type;
		};

		// Note that methods can also be created
		public static List<Part> Servos() {
			return Stream.of(Part.values()).filter(Part::isServo).collect(Collectors.toList());
		}

		public static List<Part> Motors() {
			return Stream.of(Part.values()).filter(Part::isMotor).collect(Collectors.toList());
		}

		public boolean isServo() {
			return this.type == Servo.class;
		}

		public boolean isMotor() {
			return this.type == DcMotor.class;
		}

		// Don't know when you would use this, its more of an example
		public static List<Part> driveMotors() {
			return Arrays.asList(MotorFR, MotorFL, MotorBR, MotorBL);
		}

		/**
		 * Returns true if the hardware device exists in the Driver Hub configuration, a
		 * return of true means that it is safe to call hardwaremap.get() on it
		 * 
		 * @param hardwareMap
		 * @return Devices existence
		 */
		public boolean exists(HardwareMap hardwareMap) {
			// Kinda silly way to do this, if only errors as values were a thing
			try {
				hardwareMap.get(this.name);
				return true;
			} catch (Exception ignored) {
				return false;
			}
		}

		/**
		 * Checks to make sure every Part is valid / exists
		 * 
		 * @param hardwareMap
		 * @return if all Parts are valid
		 */
		public static boolean validateAll(HardwareMap hardwareMap) {
			return Stream.of(Part.values()).anyMatch(part -> !part.exists(hardwareMap));
		}
	}
}
