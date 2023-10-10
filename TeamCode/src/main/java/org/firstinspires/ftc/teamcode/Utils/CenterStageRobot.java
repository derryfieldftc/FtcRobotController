package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class CenterStageRobot implements iRobot {
    private final LinearOpMode creator;
    private final HardwareMap hardwareMap;
    public Telemetry telemetry;

    private DcMotorEx rfMotor;
    private DcMotorEx rrMotor;
    private DcMotorEx lfMotor;
    private DcMotorEx lrMotor;

    private BNO055IMU imu;

    private final double MIN_ROBOT_SPEED = 0.15; // Min speed it will drive at

    private final double MAX_ROBOT_SPEED = 1.00; // Soft limit robot speed
    private final double NORMAL_ROBOT_SPEED = 0.80; // Percentage speed
    private final double correctionSpeed = 0.1; //How fast we want to go while correcting heading

    // TODO: PIDF values must be updated to work for this year.
    private final double longPID = 2.0; // For distance >= 20"
    private final double shortPID = 5.0; // For distances <= 20"

    private final double ticksPerMotorRevolution = 288; //TODO Must check motor
    private final double wheelCircumferenceInInches = (2.9) * Math.PI;
    private final double ticksPerInch = ticksPerMotorRevolution / wheelCircumferenceInInches;


//TODO: This all is practical motor adjustments, as some motors cannot actually
// reach predicted tps, and we want to be as accurate as possible
    private final int lfMotorMaxTps = 1400;
    private final int rfMotorMaxTps = 1400;
    private final int lrMotorMaxTps = 1400;
    private final int rrMotorMaxTps = 1400;

    private final static double HOLD_TIME = 1000; // In ms

    private double delta;

    private final double deltaThreshold = 1;

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

        initializeIMU();
    }
    /**
     * Initializes IMU to give us the robots heading
     */
    private void initializeIMU() {
        BNO055IMU.Parameters imuParameters;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        telemetry.addData("Status", "Calibrating IMU...");
        telemetry.update();
        imuParameters = new BNO055IMU.Parameters();
        imuParameters.mode = BNO055IMU.SensorMode.IMU;
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.loggingEnabled = false;
        imu.initialize(imuParameters);
        telemetry.addData("Status", "Calibrating IMU...done");
        telemetry.update();
    }

    /**
     * Displays telemetry information on the Driver Hub
     */
    public void telemetryDashboard(@SuppressWarnings("unused") String msg) {
        telemetry.addData("Heading", "Desired: %.0f, Current: %.0f, Delta: %.0f",
                getIMUHeading(), getIMUHeading(), delta);
        telemetry.addData("Target", "LF: %d, LR: %d, RF: %d, RR: %d",
                lfMotor.getTargetPosition(), lrMotor.getTargetPosition(), rfMotor.getTargetPosition(), rrMotor.getTargetPosition());
        telemetry.addData("Position", "LF: %d, LR: %d, RF: %d, RR: %d",
                lfMotor.getCurrentPosition(), lrMotor.getCurrentPosition(), rfMotor.getCurrentPosition(), rrMotor.getCurrentPosition());
        telemetry.addData("Power", "LF: %.1f, LR: %.1f, RF: %.1f, RR: %.1f",
                lfMotor.getPower(), lrMotor.getPower(), rfMotor.getPower(), rrMotor.getPower());

        double imuHeading = getIMUHeading();

        telemetry.addData("IMU Heading", "%.0f", imuHeading);
        telemetry.update();
    }

    /**
     * @param direction  1 = forward, 0 = stop, -1 = backwards
     * @param accelSlope The acceleration slope
     * @param decelSlope The deceleration slope
     * @return Returns false if drive is not correct, true if it may continue
     */
    public boolean driveCorrect(int direction, double accelSlope, double decelSlope) {
        // We are checking to make sure it is doing what we think it should be
        if (direction == -1 && accelSlope > 0) {
            System.out.println("ERROR: Uh-oh, the robot tried to accelerate forwards when you said" +
                    "to go backwards. ");
            return false;
        }
        if (direction == 1 && accelSlope < 0) {
            System.out.println("ERROR: Uh-oh, the robot tried to accelerate backwards when you said" +
                    "to go forward. ");
            return false;
        }
        if (direction == 0 && accelSlope != 0) {
            System.out.println("ERROR: Uh-oh, the robot tried to accelerate when you said" +
                    "to go nowhere. ");
            return false;
        }
        if (direction == -1 && decelSlope < 0) {
            System.out.println("ERROR: Uh-oh, the robot tried to decelerate backwards when you said" +
                    "to go backwards. ");
            return false;
        }
        if (direction == 1 && decelSlope > 0) {
            System.out.println("ERROR: Uh-oh, the robot tried to decelerate forwards when you said" +
                    "to go forwards. ");
            return false;
        }
        if (direction == 0 && decelSlope != 0) {
            System.out.println("ERROR: Uh-oh, the robot tried to decelerate  when you said" +
                    "to go nowhere. ");
            return false;
        } else {
            return true;
        }
    }

    private void driveDelta(double desiredHeading, double power) {
        double currentHeading = getIMUHeading();
        delta = normalizeHeading(desiredHeading - currentHeading);
        double adjustSpeed = 0;


        if (Math.abs(delta) > deltaThreshold) {
            adjustSpeed = correctionSpeed;
            if (delta > 0) {
                adjustSpeed *= -1;
            }
        }

        double leftSpeed = power + adjustSpeed;
        double rightSpeed = power - adjustSpeed;

        if (leftSpeed > MAX_ROBOT_SPEED) {
            leftSpeed = MAX_ROBOT_SPEED;
        }
        if (rightSpeed > MAX_ROBOT_SPEED) {
            rightSpeed = MAX_ROBOT_SPEED;
        }
        powerTheWheels(leftSpeed, leftSpeed, rightSpeed, rightSpeed);
        System.out.println("DEBUG: Delta power (left): " + leftSpeed + " (right): " + rightSpeed);
    }


    /**
     * @param distance Distance the robot should travel in inches, positive for forwards, negative for backwards
     */
@Override
    public void drive(double distance) {
        drive(distance, MIN_ROBOT_SPEED, MAX_ROBOT_SPEED);
    }

    /**
     * @param distance Distance the robot should travel in inches, positive for forwards, negative for backwards
     * @param minSpeed Minimum speed the robot should drive at ALWAYS POSITIVE, Range: 0-1
     * @param maxSpeed Maximum speed the robot should drive at ALWAYS POSITIVE, Range: 0-1
     */
    public void drive(double distance, double minSpeed, double maxSpeed) {

        if (distance >= 20) {
            lfMotor.setPositionPIDFCoefficients(longPID);
            rfMotor.setPositionPIDFCoefficients(longPID);
            lrMotor.setPositionPIDFCoefficients(longPID);
            rrMotor.setPositionPIDFCoefficients(longPID);
        }
        if(distance < 20) {
            lfMotor.setPositionPIDFCoefficients(shortPID);
            rfMotor.setPositionPIDFCoefficients(shortPID);
            lrMotor.setPositionPIDFCoefficients(shortPID);
            rrMotor.setPositionPIDFCoefficients(shortPID);
        }

        double desiredHeading = getIMUHeading();

        if (distance == 0) {
            System.out.println("Success! The robot did not move. The distance entered was 0.");
            return;
        }

        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorDistanceToTravel(distance, new int[]{1, 1, 1, 1});
//something
        int direction = (distance > 0) ? 1 : -1;
        System.out.println("Direction: " + direction);

        double power;

        double minPower;
        if (direction == -1) {
            minPower = -1 * (minSpeed);
        } else {
            minPower = minSpeed;
        }

        double maxPower;
        if (direction == -1) {
            maxPower = -1 * (maxSpeed);
        } else {
            maxPower = maxSpeed;
        }

        //rise-over-run code for accel/decel slope
        double accelRun = 2; //inches to accelerate up to max speed.
        double decelRun = 2; //inches to decelerate down to stopping.
        System.out.println("DEBUG: Distance = " + accelRun + decelRun);

        double accelRise;
        if (direction == -1) {
            accelRise = -1 * (maxSpeed - minSpeed);
        } else {
            accelRise = maxSpeed - minSpeed;
        }

        double decelRise;
        if (direction == -1) {
            decelRise = -1 * (0 - maxSpeed);
        } else {
            decelRise = 0 - maxSpeed;
        }

        double accelSlope = accelRise / accelRun;

        //decel goes from max to stopped
        double decelSlope = decelRise / decelRun;

        if (!driveCorrect(direction, accelSlope, decelSlope)) {
            return;
        }


        telemetryDashboard("");
        double distanceTraveled = getMotorPosition();

        if (Math.abs(distance) <= accelRun + decelRun) {
            System.out.println("DEBUG: Going less than 6 inches");
            //Cruising speed
            while (Math.abs(distance) > Math.abs(distanceTraveled)) {
                distanceTraveled = getMotorPosition();
                power = maxPower;
                System.out.println("DEBUG: " + power);
                powerTheWheels(power, power, power, power);
            }
        } else {
            while ((Math.abs(distance)) > (Math.abs(distanceTraveled))) {
                //Acceleration to cruising speed
                System.out.println("DEBUG Accelerating: ");
                while ((Math.abs(accelRun)) > (Math.abs(distanceTraveled))) {
                    distanceTraveled = getMotorPosition();
                    power = Math.abs(distanceTraveled) * accelSlope + minPower;
                    driveDelta(desiredHeading, power);
                    System.out.println("DEBUG Speed: " + power + ", Distance Travelled: " + distanceTraveled + ", Desired Distance: " + distance);
                }

                //Cruising speed
                System.out.println("DEBUG Cruising: ");
                while (((Math.abs(distance)) - (Math.abs(distanceTraveled))) > decelRun) {
                    distanceTraveled = getMotorPosition();
                    power = maxPower;
                    driveDelta(desiredHeading, power);
                    System.out.println("DEBUG Speed: " + power + ", Distance Travelled: " + distanceTraveled + ", Desired Distance: " + distance);
                }

                distanceTraveled = getMotorPosition();
                distance -= distanceTraveled;
                setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                setMotorDistanceToTravel(distance, new int[]{1, 1, 1, 1});

                //Deceleration to stopping
                System.out.println("DEBUG Decelerating: ");
                while ((Math.abs(distance)) > (Math.abs(distanceTraveled))) {
                    distanceTraveled = getMotorPosition();
                    power = maxPower + distanceTraveled * decelSlope;
                    if (power <= minPower) {
                        power = minPower;
                    }
                    driveDelta(desiredHeading, power);
                    System.out.println("DEBUG Speed: " + power + ", Distance Travelled: " + distanceTraveled + ", Desired Distance: " + distance);
                }
                System.out.println("DEBUG: Finished decel");
            }
        }
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        powerTheWheels(0, 0, 0, 0);

    }

    @Override
    public void strafe(double distance) {
        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        double strafeSlippage = 1.1;
        double desiredHeading = getIMUHeading();
        double leftSpeed;
        double rightSpeed;
        double strafePositionPIDF = 6.5;
        lfMotor.setPositionPIDFCoefficients(strafePositionPIDF);
        rfMotor.setPositionPIDFCoefficients(strafePositionPIDF);
        lrMotor.setPositionPIDFCoefficients(strafePositionPIDF);
        rrMotor.setPositionPIDFCoefficients(strafePositionPIDF);
        setMotorDistanceToTravel(distance * strafeSlippage, new int[]{-1, 1, 1, -1});
        double speed = MIN_ROBOT_SPEED;
        leftSpeed = speed;
        rightSpeed = -speed;
        powerTheWheels(rightSpeed, leftSpeed, leftSpeed, rightSpeed);
        telemetryDashboard("");
        while (creator.opModeIsActive() && motorsShouldContinue(distance, new int[]{-1, 1, 1, -1})) {
            double imuHeading = getIMUHeading();
            delta = normalizeHeading(desiredHeading - imuHeading);
            double adjustSpeed = 0;
            if (Math.abs(delta) > deltaThreshold) {
                //e
                adjustSpeed = correctionSpeed;
                if (delta > 0) {
                    adjustSpeed *= -1;
                }
            }
            leftSpeed = speed + adjustSpeed;
            rightSpeed = -speed - adjustSpeed;
            powerTheWheels(rightSpeed, leftSpeed, leftSpeed, rightSpeed);
            telemetryDashboard("");

            double strafeRobotSpeed = 0.5;
            if (speed < strafeRobotSpeed) {
                double driveAccelerationIncrement = 0.075;
                speed += driveAccelerationIncrement;
            }
        }
        if (!creator.opModeIsActive()) {
            throw new EmergencyStopException("Strafe");
        }
        // Reset motor mode
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        powerTheWheels(0, 0, 0, 0);
        lfMotor.setPositionPIDFCoefficients(shortPID);
        rfMotor.setPositionPIDFCoefficients(shortPID);
        lrMotor.setPositionPIDFCoefficients(shortPID);
        rrMotor.setPositionPIDFCoefficients(shortPID);

    }

    @Override
    public void rotate(double degrees) {
        System.out.println("[**]DEBUG - Entered rotate");
        System.out.println("[**]DEBUG - Degrees: " + degrees);
        double minTurnSpeed = 0.2;
        double currentHeading = getIMUHeading();
        System.out.println("[**]DEBUG - Current heading: " + currentHeading);
        double leftSpeed;
        double rightSpeed;
        delta = normalizeHeading(degrees - currentHeading);
        System.out.println("[**]DEBUG - Delta: " + delta);
        double priorDelta = delta;
        int ringingCount = 0;
        double turnDeltaThreshold = 5;
        while (creator.opModeIsActive() && Math.abs(delta) > turnDeltaThreshold && ringingCount <= 3) {
            currentHeading = getIMUHeading();
            delta = normalizeHeading(degrees - currentHeading);
            double deltaPercentage = powerPercentage(delta);
            double turnSpeed = 0.5;
            double currentTurnSpeed = turnSpeed * deltaPercentage + minTurnSpeed;
            if (delta < 0) {
                currentTurnSpeed = -currentTurnSpeed;
            }
            leftSpeed = -currentTurnSpeed;
            rightSpeed = currentTurnSpeed;
            powerTheWheels(leftSpeed, leftSpeed, rightSpeed, rightSpeed);
            System.out.println("DEBUG: Current Heading: " + currentHeading + " Desired Heading: " + degrees + " Speed: " + currentTurnSpeed);
            telemetryDashboard("");
            if (Math.signum(delta) != Math.signum(priorDelta) && delta != 0 && priorDelta != 0) {
                ringingCount++;
            }
            priorDelta = delta;
            System.out.println("[**]DEBUG - Speed: " + leftSpeed + rightSpeed);
        }
        if (!creator.opModeIsActive()) {
            throw new EmergencyStopException("Turn");
        }
        powerTheWheels(0, 0, 0, 0);
        hold(degrees);

    }
    private void hold(double degrees) {
        ElapsedTime timer;

        double leftSpeed;
        double rightSpeed;
        double currentHeading = getIMUHeading();
        delta = normalizeHeading(degrees - currentHeading);
        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        timer.reset();
        while (creator.opModeIsActive() && Math.abs(delta) > 0.5 && timer.time() < HOLD_TIME) {
            double holdSpeed = 0.15;
            if (delta > 0) {
                leftSpeed = -holdSpeed;
                rightSpeed = holdSpeed;
            }
            else {
                leftSpeed = holdSpeed;
                rightSpeed = -holdSpeed;
            }
            powerTheWheels(leftSpeed, leftSpeed, rightSpeed, rightSpeed);
            creator.sleep(75);
            powerTheWheels(0, 0, 0, 0);
            telemetryDashboard("");
            currentHeading = getIMUHeading();
            delta = normalizeHeading(degrees - currentHeading);
        }

        if (!creator.opModeIsActive()) {
            throw new EmergencyStopException("Hold");
        }

        powerTheWheels(0, 0, 0, 0);
    }

    @Override
    public void driveXYRB(double x, double y, double r, double boost, double boostDirection) {
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
        this.setPowerWithAcceleration(lfMotor, lfSpeed, boost, boostDirection);
        this.setPowerWithAcceleration(rfMotor, rfSpeed, boost, boostDirection);
        this.setPowerWithAcceleration(lrMotor, lrSpeed, boost, boostDirection);
        this.setPowerWithAcceleration(rrMotor, rrSpeed, boost, boostDirection);

        telemetry.addData("LF", lfMotor.getPower());
        telemetry.addData("LR", lrMotor.getPower());
        telemetry.addData("RF", rfMotor.getPower());
        telemetry.addData("RR", rrMotor.getPower());
    }


    /**
     * Sets the appropriate speeds for motors after acceleration of deceleration (TeleOp).
     * Confirms that speeds being set will not exceed the maximum or minimum
     *
     * @param motorSpeed          the speed of a motor before acceleration or deceleration has been applied
     * @param percentAcceleration the percent of acceleration or deceleration that a motor will use. This value should be acquired from a gamepad. Positive value for acceleration, negative for deceleration.
     */
    //these are all calcs i did over the summer, need to confirm
    private void setPowerWithAcceleration(DcMotorEx motor, double motorSpeed, double percentAcceleration, double accelerationDirection) {
        // The acceleration speed set on normal speed.
        double accelerationSpeed = 0.0;
        if (accelerationDirection == -1.0) {
            accelerationSpeed = MIN_ROBOT_SPEED - NORMAL_ROBOT_SPEED;
        } else if (accelerationDirection == 1.0) {
            accelerationSpeed = MAX_ROBOT_SPEED - NORMAL_ROBOT_SPEED;
        }

        double projectedPower = Math.abs(motorSpeed) + (accelerationSpeed * percentAcceleration);
        if (projectedPower > MAX_ROBOT_SPEED || projectedPower < MIN_ROBOT_SPEED) {
            if (Math.abs(motorSpeed) > NORMAL_ROBOT_SPEED) {
                if (motorSpeed > 0) {
                    motorSpeed = NORMAL_ROBOT_SPEED;
                    motor.setPower(motorSpeed + (accelerationSpeed * percentAcceleration));
                } else if (motorSpeed < 0) {
                    motorSpeed = -NORMAL_ROBOT_SPEED;
                    motor.setPower(motorSpeed - (accelerationSpeed * percentAcceleration));
                } else {
                    motor.setPower(0);
                }
            } else if (Math.abs(motorSpeed) < MIN_ROBOT_SPEED) {
                if (motorSpeed > 0) {
                    motorSpeed = MIN_ROBOT_SPEED;
                    motor.setPower(motorSpeed + (accelerationSpeed * percentAcceleration));
                } else if (motorSpeed < 0) {
                    motorSpeed = -MIN_ROBOT_SPEED;
                    motor.setPower(motorSpeed - (accelerationSpeed * percentAcceleration));
                } else {
                    motor.setPower(0);
                }
            }
        } else {
            if (motorSpeed > 0) {
                motor.setPower(motorSpeed + (accelerationSpeed * percentAcceleration));
            } else if (motorSpeed < 0) {
                motor.setPower(motorSpeed - (accelerationSpeed * percentAcceleration));
            } else {
                motor.setPower(0);
            }
        }
    }

    @Override
    public void driveTank(double l, double r) {
        lfMotor.setPower(l * NORMAL_ROBOT_SPEED);
        rfMotor.setPower(r * NORMAL_ROBOT_SPEED);
        lrMotor.setPower(l * NORMAL_ROBOT_SPEED);
        rrMotor.setPower(r * NORMAL_ROBOT_SPEED);
    }
    private boolean motorsShouldContinue(double distance, int[] motorDirection) {
        boolean motorsAreBusy = lfMotor.isBusy() && rfMotor.isBusy() && lrMotor.isBusy() && rrMotor.isBusy();
        boolean aMotorHasPassedPosition = false;
        if (motorsAreBusy) {
            aMotorHasPassedPosition = checkMotorPosition(lfMotor, distance * motorDirection[0])
                    || checkMotorPosition(lrMotor, distance * motorDirection[1])
                    || checkMotorPosition(rfMotor, distance * motorDirection[2])
                    || checkMotorPosition(rrMotor, distance * motorDirection[3]);
        }
        return motorsAreBusy && !aMotorHasPassedPosition;
    }

    private boolean checkMotorPosition(DcMotorEx motor, double distance) {
        //checks to see if we have gotten there yet
        if (distance > 0) {
            return motor.getCurrentPosition() + 10 > motor.getTargetPosition();
        }
        else {
            return motor.getCurrentPosition() < motor.getTargetPosition();
        }
    }
    @Override
    public double getIMUHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    /**
     * @return returns the distance the robot has traveled in inches
     */
    private double getMotorPosition() {
        double lfPosition = lfMotor.getCurrentPosition();
        double rfPosition = rfMotor.getCurrentPosition();
        double lrPosition = lrMotor.getCurrentPosition();
        double rrPosition = rrMotor.getCurrentPosition();

        double motorPositionAverage = (lfPosition + rfPosition + lrPosition + rrPosition) / 4;

        return motorPositionAverage / ticksPerInch;
    }

    @Override
    public void driveStop() {

    }

    @Override
    public void setMotorMode(DcMotor.RunMode mode) {
            lfMotor.setMode(mode);
            lrMotor.setMode(mode);
            rfMotor.setMode(mode);
            rrMotor.setMode(mode);

    }
    private void setMotorDistanceToTravel(double distance, int[] direction) {

        if (direction.length != 4) {
            throw new IllegalArgumentException("You must provide an array with exactly 4 elements!");
        }

        for (int i = 0; i < 4; i++) {
            if (direction[i] > 1 || direction[i] < -1) {
                throw new IllegalArgumentException("Elements must be -1, 0, or 1.");
            }
        }

        double distanceInTicks = distance * ticksPerInch;
        int leftFrontTargetPosition = (int) (lfMotor.getCurrentPosition() + distanceInTicks);
        int leftRearTargetPosition = (int) (lrMotor.getCurrentPosition() + distanceInTicks);
        int rightFrontTargetPosition = (int) (rfMotor.getCurrentPosition() + distanceInTicks);
        int rightRearTargetPosition = (int) (rrMotor.getCurrentPosition() + distanceInTicks);

        lfMotor.setTargetPosition(direction[0] * leftFrontTargetPosition);
        lrMotor.setTargetPosition(direction[1] * leftRearTargetPosition);
        rfMotor.setTargetPosition(direction[2] * rightFrontTargetPosition);
        rrMotor.setTargetPosition(direction[3] * rightRearTargetPosition);

        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Powers all 4 of the robot's wheels.
     * If the motor mode is set to RUN_USING_ENCODER, then PTW sets the velocity.
     * If the motor mode is set to RUN_TO_POSITION, then PTW sets the power.
     * <p>
     * The robot is only capable of accepting speeds of -1 --> 1.
     * If you give a value out of that range, PTW will scale down the numbers appropriately.
     *
     * @param lfPower power/velocity applied to the left front wheel.
     * @param lrPower power/velocity applied to the left rear wheel.
     * @param rfPower power/velocity applied to the right front wheel.
     * @param rrPower power/velocity applied to the right rear wheel.
     */
    private void powerTheWheels(double lfPower, double lrPower, double rfPower, double rrPower) {
        double leftMax = Math.max(Math.abs(lfPower), Math.abs(lrPower));
        double rightMax = Math.max(Math.abs(rfPower), Math.abs(rrPower));
        double max = Math.max(leftMax, rightMax);

        if (max > MAX_ROBOT_SPEED) {
            lfPower /= max;
            lrPower /= max;
            rfPower /= max;
            rrPower /= max;
        }

        if (lfMotor.getMode().equals(DcMotor.RunMode.RUN_USING_ENCODER)) {
            double lfVelocity = lfPower * lfMotorMaxTps;
            double lrVelocity = lrPower * lrMotorMaxTps;
            double rfVelocity = rfPower * rfMotorMaxTps;
            double rrVelocity = rrPower * rrMotorMaxTps;

            if (creator.opModeIsActive()) {

                lfMotor.setVelocity(lfVelocity);
                lrMotor.setVelocity(lrVelocity);
                rfMotor.setVelocity(rfVelocity);
                rrMotor.setVelocity(rrVelocity);
            } else {
                throw new EmergencyStopException("PowerTheWheels");
            }
        } else {
            // We assume that we will be using RUN_TO_POSITION mode.
            if (creator.opModeIsActive()) {
                lfMotor.setPower(lfPower);
                lrMotor.setPower(lrPower);
                rfMotor.setPower(rfPower);
                rrMotor.setPower(rrPower);
            } else {
                throw new EmergencyStopException("PowerTheWheels");
            }
        }
    }

    @Override
    public void stopAll() {

    }

    private double powerPercentage(double delta) {
        double powerPercent = -0.000027 * Math.pow(Math.abs(delta) - 180, 2) + 1;
        if (powerPercent > 1 || powerPercent < 0) {
            System.out.println("*** WARNING! POWER PERCENT IS OUT OF RANGE: delta = " + delta + ", " +
                    "powerPercent = " + powerPercent + " ***");
        }

        return powerPercent;
    }

    @Override
    public double normalizeHeading(double heading) {

        while (heading >= 180.0 || heading < -180.0) {
            if (heading >= 180.0) {
                heading -= 360.0;
            } else {
                heading += 360.0;
            }
        }
        return heading;
    }
}
