package org.firstinspires.ftc.teamcode.OpMode;

import static org.firstinspires.ftc.teamcode.Helper.AssignMotors.INTAKE_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.OpMode.MecanumDriveTest.*;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.Helper.*;

@TeleOp(name="IntakeTest")
public class IntakeTest extends LinearOpMode {
    MecanumDrive d;
    boolean intakeSlow;
    boolean intakeFast;
    boolean intakeReg;

    @Override
    public void runOpMode() throws InterruptedException {


        waitForStart();
        d = new MecanumDrive(
                hardwareMap,
                RIGHT_FRONT_MOTOR_NAME,
                LEFT_FRONT_MOTOR_NAME,
                RIGHT_REAR_MOTOR_NAME,
                LEFT_REAR_MOTOR_NAME,
                INTAKE_MOTOR_NAME,
                IMU_NAME,
                ENCODER_RESOLUTION,
                WHEEL_DIAMETER_CM,
                this
        );
        telemetry.update();

        while (opModeIsActive()) {
            intakeSlow = gamepad2.a;
            intakeFast = gamepad2.x;
            intakeReg = gamepad2.b;
            if (intakeSlow) {
                d.intakeMotor.setPower(0.3);
            }
            if (intakeFast) {
                d.intakeMotor.setPower(0.6);
            }
            if (intakeReg) {
                d.intakeMotor.setPower(0.9);
            }
        }
    }
}
