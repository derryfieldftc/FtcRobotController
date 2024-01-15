package org.firstinspires.ftc.teamcode.Auto;

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

@Autonomous(name = "BlueCenterStage")
public class BlueCenterStage extends LinearOpMode {
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
                    INTAKE_MOTOR_NAME,
                    IMU_NAME,
                    ENCODER_RESOLUTION,
                    WHEEL_DIAMETER_CM,
                    this
            );

            waitForStart();
            if(opModeIsActive()) {
/*
                ewok.driveCentimetersStrafe(-3,0.2);
                ewok.driveCentimetersForward(-20,0.3);
                ewok.driveCentimetersStrafe(-25,0.12);
                ewok.driveCentimetersForward(-47 ,0.3);
                ewok.driveCentimetersForward(2, 0.2);

            */
            ewok.driveCentimetersForward(-48, 0.3);
            ewok.leftRear.setPower(-0.3);
            ewok.leftFront.setPower(-0.3);
            Thread.sleep(2800);
            ewok.driveCentimetersForward(-76, 0.3);
            ewok.driveCentimetersForward(3,0.3);


            }

        }
    }
