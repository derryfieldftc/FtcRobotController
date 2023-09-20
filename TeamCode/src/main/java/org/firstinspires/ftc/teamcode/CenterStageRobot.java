package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class CenterStageRobot implements iRobot{
    private final LinearOpMode creator;
    private final HardwareMap hardwareMap;
    public Telemetry telemetry;

    private DcMotorEx rfMotor;
    private DcMotorEx rrMotor;
    private DcMotorEx lfMotor;
    private DcMotorEx lrMotor;

    private final double MIN_ROBOT_SPEED = 0.15; //min speed it will drive at
    private final double MAX_ROBOT_SPEED = 1.00; //soft limit robot speed
    private final double NORMAL_ROBOT_SPEED = 0.80; // The normal speed we drive at, 60% of full motor speed
    public CenterStageRobot(LinearOpMode creator) {
        this.creator = creator;
        this.hardwareMap = creator.hardwareMap;
        this.telemetry = creator.telemetry;
    }
    @Override
    public void initHardware() {
        lfMotor = hardwareMap.get(DcMotorEx.class, "LFMotor");
        lrMotor = hardwareMap.get(DcMotorEx.class, "LRMotor");
        rfMotor = hardwareMap.get(DcMotorEx.class, "RFMotor");
        rrMotor = hardwareMap.get(DcMotorEx.class, "RRMotor");
        lfMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        lrMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void drive(double distance) {

    }

    @Override
    public void strafe(double distance) {

    }

    @Override
    public void rotate(double degrees) {

    }

    @Override
    public void driveXYRB(double x, double y, double r, double b, double bd) {
        /*
            Because we use Mecanum wheels, we can move forward, rotate, and strafe.
            Here, we are taking into account the direction each wheel should travel at in
            order to move in the direction we want the robot to move.
        */
            double lfSpeed = -((y - x - r) * NORMAL_ROBOT_SPEED);  // Left Front motor speed.
            double rfSpeed = -((y + x + r) * NORMAL_ROBOT_SPEED);  // Right Front motor speed.
            double lrSpeed = -((y + x - r) * NORMAL_ROBOT_SPEED);  // Left Rear motor speed.
            double rrSpeed = -((y - x + r) * NORMAL_ROBOT_SPEED);  // Right Rear motor speed.

            // Calculates and sets power based on its arguments
            setPowerWithAcceleration(lfMotor, lfSpeed, b, bd);
            setPowerWithAcceleration(rfMotor, rfSpeed, b, bd);
            setPowerWithAcceleration(lrMotor, lrSpeed, b, bd);
            setPowerWithAcceleration(rrMotor, rrSpeed, b, bd);

            telemetry.addData("LF", lfMotor.getPower());
            telemetry.addData("LR", lrMotor.getPower());
            telemetry.addData("RF", rfMotor.getPower());
            telemetry.addData("RR", rrMotor.getPower());
        }

        public void teleOpTestDrive(double x, double y, double r) {
            double lfSpeed = -((y - x - r) * NORMAL_ROBOT_SPEED);  // Left Front motor speed.
            double rfSpeed = -((y + x + r) * NORMAL_ROBOT_SPEED);  // Right Front motor speed.
            double lrSpeed = -((y + x - r) * NORMAL_ROBOT_SPEED);  // Left Rear motor speed.
            double rrSpeed = -((y - x + r) * NORMAL_ROBOT_SPEED);  // Right Rear motor speed.

            lfMotor.setPower(lfSpeed);
            rfMotor.setPower(rfSpeed);
            lrMotor.setPower(lrSpeed);
            rrMotor.setPower(rrSpeed);

            telemetry.addData("LF", lfMotor.getPower());
            telemetry.addData("RF", rfMotor.getPower());
            telemetry.addData("LR", lrMotor.getPower());
            telemetry.addData("RR", rrMotor.getPower());
            telemetry.update();
        }


    /**
     * Sets the appropriate speeds for motors after acceleration of deceleration (TeleOp).
     * Confirms that speeds being set will not exceed the maximum or minimum
     * @param motorSpeed the speed of a motor before acceleration or deceleration has been applied
     * @param percentAcceleration the percent of acceleration or deceleration that a motor will use. This value should be acquired from a gamepad. Positive value for acceleration, negative for deceleration.
     */
    //these are all calcs i did over the summer, need to confirm
    private void setPowerWithAcceleration(DcMotorEx motor, double motorSpeed, double percentAcceleration, double accelerationDirection) {
        // The acceleration speed set on normal speed.
        double accelerationSpeed = 0.0;
        if (accelerationDirection == -1.0) {
            accelerationSpeed = MIN_ROBOT_SPEED - NORMAL_ROBOT_SPEED;
        }
        else if (accelerationDirection == 1.0) {
            accelerationSpeed = MAX_ROBOT_SPEED - NORMAL_ROBOT_SPEED;
        }

        double projectedPower = Math.abs(motorSpeed) + (accelerationSpeed * percentAcceleration);
        if (projectedPower > MAX_ROBOT_SPEED || projectedPower < MIN_ROBOT_SPEED) {
            if (Math.abs(motorSpeed) > NORMAL_ROBOT_SPEED) {
                if (motorSpeed > 0) {
                    motorSpeed = NORMAL_ROBOT_SPEED;
                    motor.setPower(motorSpeed + (accelerationSpeed * percentAcceleration));
                }
                else if (motorSpeed < 0) {
                    motorSpeed = -NORMAL_ROBOT_SPEED;
                    motor.setPower(motorSpeed - (accelerationSpeed * percentAcceleration));
                }
                else {
                    motor.setPower(0);
                }
            }
            else if (Math.abs(motorSpeed) < MIN_ROBOT_SPEED) {
                if (motorSpeed > 0) {
                    motorSpeed = MIN_ROBOT_SPEED;
                    motor.setPower(motorSpeed + (accelerationSpeed * percentAcceleration));
                }
                else if (motorSpeed < 0) {
                    motorSpeed = -MIN_ROBOT_SPEED;
                    motor.setPower(motorSpeed - (accelerationSpeed * percentAcceleration));
                }
                else {
                    motor.setPower(0);
                }
            }
        }
        else {
            if (motorSpeed > 0) {
                motor.setPower(motorSpeed + (accelerationSpeed * percentAcceleration));
            }
            else if (motorSpeed < 0) {
                motor.setPower(motorSpeed - (accelerationSpeed * percentAcceleration));
            }
            else {
                motor.setPower(0);
            }
        }
    }

    @Override
    public void driveTank(double l, double r, double b) {

    }

    @Override
    public double getIMUHeading() {
        return 0;
    }

    @Override
    public void driveStop() {

    }

    @Override
    public void setMotorMode(DcMotor.RunMode mode) {

    }

    @Override
    public void stopAll() {

    }

    @Override
    public double normalizeHeading(double heading) {
        return 0;
    }
}
