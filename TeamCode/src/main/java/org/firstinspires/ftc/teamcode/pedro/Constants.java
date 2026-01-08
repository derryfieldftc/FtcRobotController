package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.ftc.localization.localizers.ThreeWheelLocalizer;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.RobotPart;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10.4326) //KG
            .forwardZeroPowerAcceleration(-28.571047441654848)
            .lateralZeroPowerAcceleration(-73.4325534098)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0.0001, 0.0001, 0.0002));

    public static MecanumConstants mecanumDrive = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName(RobotPart.Part.MotorFR.name)
            .rightRearMotorName(RobotPart.Part.MotorBR.name)
            .leftRearMotorName(RobotPart.Part.MotorBL.name)
            .leftFrontMotorName(RobotPart.Part.MotorFL.name)
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(66.84776316581288)
            .yVelocity(54.147475146056685);


    public static ThreeWheelConstants threeWheelLocalizer = new ThreeWheelConstants()
            .leftPodY(8)
            .rightPodY(-8)
            .strafePodX(-7)
            .leftEncoder_HardwareMapName(RobotPart.Part.LeftDriveEncoder.name)
            .rightEncoder_HardwareMapName(RobotPart.Part.RightDriveEncoder.name)
            .strafeEncoder_HardwareMapName(RobotPart.Part.StrafeEncoder.name)
            .leftEncoderDirection(Encoder.FORWARD)
            .rightEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.FORWARD)
            .forwardTicksToInches(0.0019895402114732674)
            .strafeTicksToInches(0.0019896225811645574)
            .turnTicksToInches(0.00197568911474871);




    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(mecanumDrive)
                .pathConstraints(pathConstraints)
                .setLocalizer(new ThreeWheelLocalizer(hardwareMap, threeWheelLocalizer))
                .build();
    }
}
