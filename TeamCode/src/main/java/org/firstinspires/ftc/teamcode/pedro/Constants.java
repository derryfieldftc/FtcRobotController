package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.ftc.localization.localizers.ThreeWheelLocalizer;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.RobotPart;

public class Constants {
	public static FollowerConstants followerConstants = new FollowerConstants();

	public static MecanumConstants mecanumDrive = new MecanumConstants();

	public static PinpointConstants localizerConstants = new PinpointConstants();

	public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

	public static Follower createFollower(HardwareMap hardwareMap) {
		mecanumDrive.setUseBrakeModeInTeleOp(true);
		return new FollowerBuilder(followerConstants, hardwareMap)
				.mecanumDrivetrain(mecanumDrive)
				.pathConstraints(pathConstraints)
				.setLocalizer(new PinpointLocalizer(hardwareMap, localizerConstants))
				.build();
	}
}
