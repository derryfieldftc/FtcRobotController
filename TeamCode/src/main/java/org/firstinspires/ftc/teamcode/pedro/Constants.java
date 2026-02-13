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
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.339) //Kg
            .forwardZeroPowerAcceleration(-28.80868329)
            .lateralZeroPowerAcceleration(-57.99970102)
            .translationalPIDFCoefficients(new PIDFCoefficients(.3, 0, .05, .01)) // if anything fix this one
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(.2, .00001, .0001, .01)) // if anything fix this one
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(.1, .001, 0.0001, .6, .1))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(.05, .1, .00005, .6, .01))
            .headingPIDFCoefficients(new PIDFCoefficients(3, 0, .01, .01))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(5, .001, .001, .00))
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true);

    public static MecanumConstants mecanumDrive = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName(RobotPart.Part.MotorFR.name)
            .rightRearMotorName(RobotPart.Part.MotorBR.name)
            .leftRearMotorName(RobotPart.Part.MotorBL.name)
            .leftFrontMotorName(RobotPart.Part.MotorFL.name)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(59.639027)
            .yVelocity(42.776740);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(6.5)
            .strafePodX(-(3 + 7 / 8.0))
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);


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
