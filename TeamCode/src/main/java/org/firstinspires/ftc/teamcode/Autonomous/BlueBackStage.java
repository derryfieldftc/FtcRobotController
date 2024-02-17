package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Helper.AssignMotors.ENCODER_RESOLUTION;
import static org.firstinspires.ftc.teamcode.Helper.AssignMotors.IMU_NAME;
import static org.firstinspires.ftc.teamcode.Helper.AssignMotors.INTAKE_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.Helper.AssignMotors.LEFT_FRONT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.Helper.AssignMotors.LEFT_REAR_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.Helper.AssignMotors.RIGHT_FRONT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.Helper.AssignMotors.RIGHT_REAR_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.Helper.AssignMotors.WHEEL_DIAMETER_CM;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Helper.MecanumDrive;

@Autonomous(name = "BlueBackstage")
public class BlueBackStage extends LinearOpMode {
    MecanumDrive ewok;
    @Override
    public void runOpMode() throws InterruptedException {
        //we have landed on a moon of Endor.
        ewok = new MecanumDrive(
                hardwareMap,
                RIGHT_FRONT_MOTOR_NAME,
                LEFT_FRONT_MOTOR_NAME,
                RIGHT_REAR_MOTOR_NAME,
                LEFT_REAR_MOTOR_NAME,
                IMU_NAME,
                ENCODER_RESOLUTION,
                WHEEL_DIAMETER_CM,
                this
        );
        double turnPower;


        waitForStart();
        if(opModeIsActive()) {
            ewok.driveCentimetersStrafe(-1, 0.2);
            ewok.driveCentimetersForward(-47, 0.3);
            ewok.driveCentimetersForward(5, 0.3);

        }
    }
}
