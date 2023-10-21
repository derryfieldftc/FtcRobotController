package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Helper.MecanumDrive;

@TeleOp(name="MecanumDriveTest", group="Tests")
public class MecanumDriveTest extends LinearOpMode {

    public static final String rightFrontMotorName = "motorFR";
    public static final String leftFrontMotorName = "motorFL";
    public static final String rightRearMotorName = "motorBR";
    public static final String leftRearMotorName = "motorBL";

    DcMotor rightFrontMotor,
            leftFrontMotor,
            rightRearMotor,
            leftRearMotor;

    @Override
    public void runOpMode() {

        initMotors();
        MecanumDrive mecanum = new MecanumDrive(
            rightFrontMotor,
            leftFrontMotor,
            rightRearMotor,
            leftRearMotor
        );

        waitForStart();

        double forward, strafe, rotate, scale; // Drive variables

        while (opModeIsActive()) {

            forward = -gamepad1.left_stick_y; // Up is negative; we want up to be positive, so we *(-1)
            strafe = gamepad1.left_stick_x;   // Perfect child, no flip
            rotate = -gamepad1.right_stick_x; // Positive is CCW; we want positive to be CW, so we *(-1)
            scale = 0.8;

            mecanum.drive(forward, strafe, rotate, scale);

            telemetry.addData("forward", forward);
            telemetry.addData("strafe", forward);
            telemetry.addData("rotate", forward);
            telemetry.update();
        }

    }

    public void initMotors() {
        rightFrontMotor = (DcMotor)hardwareMap.get(rightFrontMotorName);
        leftFrontMotor = (DcMotor)hardwareMap.get(leftFrontMotorName);
        rightRearMotor = (DcMotor)hardwareMap.get(rightRearMotorName);
        leftRearMotor = (DcMotor)hardwareMap.get(leftRearMotorName);

        rightFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRearMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRearMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
}
