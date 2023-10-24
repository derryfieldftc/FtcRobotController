package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Helper.MecanumDrive;

@TeleOp(name="MecanumDriveExample", group="Tests")
public class MecanumDriveExample extends LinearOpMode {

    public static final String rightFrontName = "motorFR";
    public static final String leftFrontName = "motorFL";
    public static final String rightRearame = "motorBR";
    public static final String leftRearName = "motorBL";

    MecanumDrive bot;

    @Override
    public void runOpMode() {
        // get a reference to our mecanum drive.
        bot = new MecanumDrive(hardwareMap, rightFrontName, leftFrontName, rightRearame, leftRearName);

        // wait for user to press start button on Driver Hub.
        waitForStart();

        double forward, strafe, rotate; // Drive variables

        while (opModeIsActive()) {
            forward = -gamepad1.left_stick_y; // Up is negative; we want up to be positive, so we *(-1)
            strafe = gamepad1.left_stick_x;   // Perfect child, no flip
            rotate = -gamepad1.right_stick_x; // Positive is CCW; we want positive to be CW, so we *(-1)

            bot.drive(forward, strafe, rotate);

            telemetry.addData("forward", forward);
            telemetry.addData("strafe", forward);
            telemetry.addData("rotate", forward);
            telemetry.update();
        }
    }
}
