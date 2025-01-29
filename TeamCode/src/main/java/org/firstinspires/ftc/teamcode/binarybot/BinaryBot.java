package org.firstinspires.ftc.teamcode.binarybot;

import static androidx.core.math.MathUtils.clamp;

import static org.firstinspires.ftc.teamcode.binarybot.BinaryBot.State.*;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;


import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class BinaryBot {
    // ******************************************************************
    // enumerations.
    // ******************************************************************
    // measured movement system state.
    public enum State {
        // robot is available.
        IDLE,

        // robot is doing a measured drive, strafe, or turn.
        DRIVING_FORWARD,
        DRIVING_BACKWARD,

        STRAFING_RIGHT,
        STRAFING_LEFT,

        TURNING_CLOCKWISE,
        TURNING_COUNTERCLOCKWISE,

        // robot is placing a specimen.
        SPECIMEN_RAISING,
        SPECIMEN_APPROACHING,
        SPECIMEN_HOOKING,
        SPECIMEN_BACK_OFF,

        SPECIMEN_PICK_RAISING,
        SPECIMEN_PICK_APPROACHING,
        SPECIMEN_DONE_PICK,
        SPECIMEN_DONE_PICK_BACK_OFF
    }

    // ******************************************************************
    // Constants
    // ******************************************************************
    // The bot uses REV Robotics thru bore encoders which have a resolution of
    // 8192 counts per revolution
    static final double     POD_COUNTS_PER_MOTOR_REV    = 8192;
    static final double     POD_DRIVE_GEAR_REDUCTION    = 1;
    // REV robotics smaller omni wheels are 60mm in diameter.
    static final double     POD_WHEEL_DIAMETER_INCHES   = 1.37795;
    static final double     POD_WHEEL_CIRCUM_INCHES = Math.PI * POD_WHEEL_DIAMETER_INCHES;
    static final double     POD_COUNTS_PER_INCH         =   (POD_COUNTS_PER_MOTOR_REV * POD_DRIVE_GEAR_REDUCTION) /
            (POD_WHEEL_CIRCUM_INCHES);


    static final double     DRIVE_COUNTS_PER_MOTOR_REV    = 537.7;
    static final double     DRIVE_DRIVE_GEAR_REDUCTION    = 1;

    // GoBilda new mecanum wheels have 104mm diameter (4.094 inches)
    static final double     DRIVE_WHEEL_DIAMETER_INCHES   = 4.094;
    static final double     DRIVE_WHEEL_CIRCUM_INCHES = Math.PI * DRIVE_WHEEL_DIAMETER_INCHES;
    static final double     DRIVE_COUNTS_PER_INCH         =   (DRIVE_COUNTS_PER_MOTOR_REV * DRIVE_DRIVE_GEAR_REDUCTION) /
            (DRIVE_WHEEL_CIRCUM_INCHES);

    static final boolean USE_ODOMETRY_POD = true;

    static final double STRAFE_ENCODER_FUDGE_FACTOR = 1.0;

    static final double SPECIMEN_APPROACH_DISTANCE_INCHES = 3.0;
    static final double SPECIMEN_APPROACH_PICK_DISTANCE_INCHES = -3;
    static final double SPECIMEN_BACKOFF_DISTANCE_INCHES = 9.0;
    static final float SPECIMEN_APPROACH_POWER = 0.5f;
    static final double SPECIMEN_PICK_BACKOFF_DISTANCE_INCHES = 3.0;

    // ******************************************************************
    // private member variables
    // ******************************************************************
    // drive system
    private DcMotor motorFL;
    private DcMotor motorBL;
    private DcMotor motorFR;
    private DcMotor motorBR;

    public DcMotor driveEncoder;
    public DcMotor strafeEncoder;
    RevHubOrientationOnRobot orientationOnRobot;
    public DistanceSensor distanceRightRear;

    // op mode related items.
    private OpMode opMode;
    private HardwareMap hardwareMap;

    private BNO055IMU imu;
    private Orientation angles;
    private Acceleration gravity;
    private double previousAngle = 0;
    public double integratedAngle = 0;

    private final boolean USE_AUTO_CORRECT = true;
    private final double P_COEFFICIENT_DRIVE = 0.1;
    private final double P_COEFFICIENT_STRAFE = 0.025;

    public Manipulator manipulator;
    public State state = State.IDLE;

    public BinaryBot(HardwareMap hardwareMap, OpMode opMode) {
        this.opMode = opMode;
        this.hardwareMap = hardwareMap;
        initHardware();
    }

    private void initHardware() {
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBR = hardwareMap.dcMotor.get("motorBR");

        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);

        manipulator = new Manipulator(hardwareMap, opMode);

        if (USE_ODOMETRY_POD) {
            strafeEncoder = hardwareMap.dcMotor.get("strafe");
            driveEncoder = hardwareMap.dcMotor.get("drive");

            strafeEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
            driveEncoder.setDirection(DcMotorSimple.Direction.REVERSE);

            strafeEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            driveEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } else {
            strafeEncoder = motorFL;
            driveEncoder = motorFL;
        }

        initIMU();

        distanceRightRear = hardwareMap.get(DistanceSensor.class, "rightRearDistance");
    }

    /**
     * calibrate encoder
     * reset the strafing and drive encoders
     */
    public void calibrateOdometry() {
        // for now, reset the encoders
        // reset encoders.
        driveEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        strafeEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        strafeEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    private void initIMU() {
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = false;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public double getDistance() {
        return distanceRightRear.getDistance(DistanceUnit.INCH);
    }

    public void updateAngles() {
        float currAngle;
        double deltaAngle;

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currAngle = angles.firstAngle;

        // the binary bot team uses the convention that a positive angle is a clockwise rotation.
        // we need to flip the currAngle to work with this convention.
        currAngle = - currAngle;

        deltaAngle = currAngle - previousAngle;
        if (deltaAngle < -180) {
            // Went from +180 to -180 (direction).
            deltaAngle = deltaAngle + 360;
        } else if (deltaAngle > 180) {
            // Went from -180 to +180 (direction).
            deltaAngle = deltaAngle - 360;
        }
        integratedAngle = integratedAngle + deltaAngle;
        previousAngle = currAngle;
    }

    public void resetAngles() {
        integratedAngle = 0;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        previousAngle = angles.firstAngle;

        // the binary bot team uses the convention that a positive angle is a clockwise rotation.
        // we need to flip the currAngle to work with this convention.
        previousAngle = - previousAngle;
    }

    public double getCurrentAngle() {
        return integratedAngle;
    }

    public double getPreviousAngle() {
        return previousAngle;
    }

    // ******************************************************************
    // public methods
    // ******************************************************************
    /**
     * mecanum drive function.
     *
     * @param drive forward / backward (negative) power
     * @param strafe rightwards / leftwards (negative) power
     * @param twist clockwise / counterclockwise (negative) power
     */
    public void drive(double drive,  double strafe, double twist) {
        double powerFL = drive + strafe + twist;
        double powerBL = drive - strafe + twist;
        double powerFR = drive - strafe - twist;
        double powerBR = drive + strafe - twist;

        powerFL = clamp(1, -1, powerFL);
        powerBL = clamp(1, -1, powerBL);
        powerFR = clamp(1, -1, powerFR);
        powerBR = clamp(1, -1, powerBR);

        motorFL.setPower(powerFL);
        motorBL.setPower(powerBL);
        motorFR.setPower(powerFR);
        motorBR.setPower(powerBR);
    }

    /**
     * Stop the robot.
     */
    public void stop() {
        drive(0, 0, 0);
        RobotLog.vv("STATE", "stop(): IDLE");
        state = State.IDLE;
    }

    // drive a specified distance in inches.
    public int currentPos = 0;
    public int initPos = 0;
    public int targetPos = 0;
    public double tgtAngle = 0;
    public float measuredPower = 0;

    // this variable target distance is used to specify how
    // far the bot should travel in inches.
    public double targetDistanceInches = 0;

    /**
     * drive forward/backwards using odometery a measured distance.
     * use negative distance to go backwards.
     *
     * @param power - magnitude of forward drive power (from 0 to 1).
     * @param distance - distance to drive (inches).
     */
    public void measuredDrive(double power, double distance)  {
        // get init position.
        // NOTE: we assume the encoder will not rollover (which it shouldn't)
        // and don't bother to check for this condition.
        initPos = driveEncoder.getCurrentPosition();

        // offset in encoder ticks.
        double offset;
        if (USE_ODOMETRY_POD) {
            offset = distance * POD_COUNTS_PER_INCH;
        } else {
            offset = distance * DRIVE_COUNTS_PER_INCH;
        }

        targetPos = (int)Math.round(offset + initPos);
        if (offset < 0) {
            measuredPower = -(float)Math.abs(power);
            RobotLog.vv("STATE", String.format("measuredDrive(%f, %f): DRIVING_BACKWARD", power, distance));
            state = State.DRIVING_BACKWARD;
        } else {
            measuredPower = (float)Math.abs(power);
            RobotLog.vv("STATE", String.format("measuredDrive(%f, %f): DRIVING_FORWARD", power, distance));
            state = State.DRIVING_FORWARD;
        }

        // is auto correct enabled (which helps keep the bot drive straight)?
        if (USE_AUTO_CORRECT) {
            // reset angles.
            resetAngles();
            // set the target angle equal to the current angle.
            tgtAngle = integratedAngle;
        }
    }

    /**
     * Strafe a measured distance.  Positive power is to the right.
     * @param power the magnitude of the strafe power (0 to 1).
     * @param distance the measured distance in inches.
     */
    public void measuredStrafe(double power, double distance)  {
        // get init position.
        // NOTE: we assume the encoder will not rollover (which it shouldn't)
        // and don't bother to check for this condition.
        initPos = strafeEncoder.getCurrentPosition();

        // offset in encoder ticks.
        double offset = 0;
        if (USE_ODOMETRY_POD) {
            offset = distance * POD_COUNTS_PER_INCH;
        } else {
            offset = distance * DRIVE_COUNTS_PER_INCH * STRAFE_ENCODER_FUDGE_FACTOR;
        }

        targetPos = (int)Math.round(offset + initPos);
        if (offset < 0) {
            measuredPower = -(float)Math.abs(power);
            RobotLog.vv("STATE", String.format("measuredStrafe(%f, %f): STRAFING_LEFT", power, distance));
            state = State.STRAFING_LEFT;
        } else {
            measuredPower = (float)Math.abs(power);
            RobotLog.vv("STATE", String.format("measuredStrafe(%f, %f): STRAFING_RIGHT", power, distance));
            state = State.STRAFING_RIGHT;
        }

        // is auto correct enabled (which helps keep the bot drive straight)?
        if (USE_AUTO_CORRECT) {
            // reset angles.
            resetAngles();
            // set the target angle equal to the current angle.
            tgtAngle = integratedAngle;
        }
    }

    /**
     * Turn a specified number of degrees.
     * Positive angles imply clockwise turn.
     * @param power magnitude of turn power (0 to 1)
     * @param angle turn angle (degrees)
     */
    public void measuredTurn(double power, double angle)  {
        // reset angles.
        resetAngles();

        tgtAngle = angle;
        if (tgtAngle < 0) {
            measuredPower = -(float)Math.abs(power);
            RobotLog.vv("STATE", String.format("measuredTurn(%f, %f): TURNING_COUNTERCLOCKWISE", power, angle));
            state = State.TURNING_COUNTERCLOCKWISE;
        } else {
            measuredPower = (float)Math.abs(power);
            RobotLog.vv("STATE", String.format("measuredTurn(%f, %f): TURNING_CLOCKWISE", power, angle));
            state = State.TURNING_CLOCKWISE;
        }
    }

    /**
     * calculate the correction power.
     * proportional to the angle error.
     * @return correction power.
     */
    private float propCorrection(double p_coefficient) {
        float value;
        double error;
        // are we using auto correct?
        if (USE_AUTO_CORRECT) {
            // update angles.
            updateAngles();
            // calculate error (degrees).
            error = integratedAngle - tgtAngle;
            // corrective "twist" power is proportional to error.
            // should be applied in opposite direction.
            value = -(float)(p_coefficient * error);
        } else {
            // no autocorrect power will be applied.
            value = 0;
        }
        return value;
    }

    public void placeSpecimenHigh(double targetDistanceInches) {
        if (state != State.IDLE) {
            // robot is not available.
            return;
        }

        // use the specified approach distance value.
        this.targetDistanceInches = targetDistanceInches;

        // put green thingy in place.
        manipulator.greenThing.setPosition(Manipulator.GREEN_DEPLOYED);

        // raise slide.
        manipulator.slide.setTargetPosition(Manipulator.SLIDE_HIGH_SPECIMEN_POSITION);

        // put it in new state.
        RobotLog.vv("STATE", String.format("placeSpecimenHigh(%f): SPECIMEN_RAISING", targetDistanceInches));
        state = State.SPECIMEN_RAISING;
    }

    public void placeSpecimenHigh() {
        // use the default approach distance value.
        placeSpecimenHigh(SPECIMEN_APPROACH_DISTANCE_INCHES);

    }

    public void pickSpecimen(double targetDistanceInches) {
        if (state != State.IDLE) {
            //robot is not available
            return;
        }
        // use the specified approach distance value.
        this.targetDistanceInches = targetDistanceInches;



        // raise slide.
        manipulator.slide.setTargetPosition(Manipulator.SLIDE_SPECIMEN_PICK);

        // put it in new state.
        RobotLog.vv("STATE", String.format("pickSpecimen(%f): SPECIMEN_PICK_RAISING", targetDistanceInches));
        state = State.SPECIMEN_PICK_RAISING;
    }
    public void pickSpecimen() {
        // use the default approach distance value.
        pickSpecimen(SPECIMEN_APPROACH_PICK_DISTANCE_INCHES);

    }
    /**
     *
     * @return true if still in measured mode.
     */
    public boolean update() {
        float correction = 0;
        currentPos = driveEncoder.getCurrentPosition();


        if (state == IDLE) {
            return false;
        } else if (state == DRIVING_FORWARD) {
            if (currentPos > targetPos) { // done moving
                stop();
                return false;
            } else { // keep moving
                correction = propCorrection(P_COEFFICIENT_DRIVE);
                drive(measuredPower, 0, correction);
                return true;
            }
        } else if (state == DRIVING_BACKWARD) {
            currentPos = driveEncoder.getCurrentPosition();
            if (currentPos < targetPos) { // done moving
                stop();
                return false;
            } else { // keep moving
                correction = propCorrection(P_COEFFICIENT_DRIVE);
                drive(measuredPower, 0, correction);
                return true;
            }
        } else if (state == STRAFING_RIGHT) {
            currentPos = strafeEncoder.getCurrentPosition();
            if (currentPos > targetPos) { // done moving
                stop();
                return false;
            } else { // keep moving
                correction = propCorrection(P_COEFFICIENT_STRAFE);
                drive(0, measuredPower, correction);
                return true;
            }
        } else if (state == STRAFING_LEFT) {
            currentPos = strafeEncoder.getCurrentPosition();
            if (currentPos < targetPos) { // done moving
                stop();
                return false;
            } else { // keep moving
                correction = propCorrection(P_COEFFICIENT_STRAFE);
                drive(0, measuredPower, correction);
                return true;
            }
        } else if (state == TURNING_CLOCKWISE) {
            updateAngles();
            if (integratedAngle > tgtAngle) { // done turning
                stop();
                return false;
            } else { // keep turning
                drive(0, 0, measuredPower);
                return true;
            }
        } else if (state == TURNING_COUNTERCLOCKWISE) {
            updateAngles();
            if (integratedAngle < tgtAngle) { // done turning
                stop();
                return false;
            } else {
                drive(0, 0, measuredPower);
                return true;
            }
        } else if (state == SPECIMEN_RAISING) {
            if (!manipulator.slide.isBusy()) { // slide is done raising
                initPos = driveEncoder.getCurrentPosition();

                double inchesToTicks = USE_ODOMETRY_POD ? POD_COUNTS_PER_INCH : DRIVE_COUNTS_PER_INCH;
                double offset = targetDistanceInches * inchesToTicks; // offset in encoder ticks.
                targetPos = (int) Math.round(initPos - offset); // robot has to move backwards.
                measuredPower = -SPECIMEN_APPROACH_POWER;

                if (USE_AUTO_CORRECT) {
                    resetAngles();
                    tgtAngle = integratedAngle; // set the target angle equal to the current angle.
                }

                RobotLog.vv("STATE", "SPECIMEN_RAISING -> SPECIMEN_APPROACHING");
                state = State.SPECIMEN_APPROACHING;
                return true;
            } else { // slide is not done raising, wait
                return true;
            }
        } else if (state == SPECIMEN_APPROACHING) { // approach the sub until we've moved the required distance.
            currentPos = driveEncoder.getCurrentPosition();
            if (currentPos < targetPos) { // done approaching
                stop();
                manipulator.greenThing.setPosition(manipulator.GREEN_DEPLOYED);
                // now we need to lower the slide.
                manipulator.slide.setTargetPosition(Manipulator.SLIDE_HIGH_SPECIMEN_RELEASE);

                RobotLog.vv("STATE", "SPECIMEN_APPROACHING -> SPECIMEN_HOOKING");
                state = SPECIMEN_HOOKING;
                return true;
            } else { // keep driving with correction
                correction = propCorrection(P_COEFFICIENT_DRIVE);
                drive(measuredPower, 0, correction);
                return true;
            }
        } else if (state == SPECIMEN_HOOKING) {
            if (manipulator.slide.isBusy() == false) { // slide is done lowering
                // we need to back off.
                initPos = driveEncoder.getCurrentPosition();

                // offset in encoder ticks.
                double inchesToTicks = USE_ODOMETRY_POD ? POD_COUNTS_PER_INCH : DRIVE_COUNTS_PER_INCH;
                double offset = SPECIMEN_BACKOFF_DISTANCE_INCHES * inchesToTicks;
                targetPos = (int) Math.round(initPos + offset); // we're going forward.
                measuredPower = SPECIMEN_APPROACH_POWER;

                if (USE_AUTO_CORRECT) {
                    resetAngles();
                    tgtAngle = integratedAngle; // set the target angle equal to the current angle.
                }

                RobotLog.vv("STATE", "SPECIMEN_HOOKING -> SPECIMEN_BACK_OFF");
                state = SPECIMEN_BACK_OFF;
                return true;
            } else {
                return true; // wait for slide to lower
            }
        } else if (state == SPECIMEN_BACK_OFF) {
            currentPos = driveEncoder.getCurrentPosition();
            if (currentPos > targetPos) { // done backing off
                stop();
                return false;
            } else { // keep backing off
                correction = propCorrection(P_COEFFICIENT_DRIVE);
                drive(measuredPower, 0, correction);
                return true;
            }
        } else if (state == SPECIMEN_PICK_RAISING) {
            if (manipulator.slide.isBusy() == false) { // is the slide done raising into position?
                initPos = driveEncoder.getCurrentPosition();

                double inchesToTicks = USE_ODOMETRY_POD ? POD_COUNTS_PER_INCH : DRIVE_COUNTS_PER_INCH;
                double offset = targetDistanceInches * inchesToTicks; // offset in encoder ticks.
                targetPos = (int) Math.round(initPos - offset); // robot has to move backwards.
                measuredPower = -SPECIMEN_APPROACH_POWER;

                if (USE_AUTO_CORRECT) {
                    resetAngles();
                    tgtAngle = integratedAngle; // set the target angle equal to the current angle.
                }
                // switch to approach sub state.
                RobotLog.vv("STATE", "SPECIMEN_PICK_RAISING -> SPECIMEN_PICK_APPROACHING");
                state = State.SPECIMEN_PICK_APPROACHING;
                return true;
            } else {
                return true; // not done raising slide, wait
            }
        } else if (state == SPECIMEN_PICK_APPROACHING) { // approach the sub until we've moved the required distance.
            currentPos = driveEncoder.getCurrentPosition();
            if (currentPos < targetPos) { // done approaching
                stop();
                // now we need to raise the slide.
                manipulator.greenThing.setPosition(manipulator.GREEN_DEPLOYED);
                manipulator.slide.setTargetPosition(Manipulator.SLIDE_HIGH_SPECIMEN_RELEASE);

                RobotLog.vv("STATE", "SPECIMEN_PICK_APPROACHING -> SPECIMEN_DONE_PICK");
                state = SPECIMEN_DONE_PICK;
                return true;
            } else { // keep going
                correction = propCorrection(P_COEFFICIENT_DRIVE);
                drive(measuredPower, 0, correction);
                return true;
            }
        } else if (state == SPECIMEN_DONE_PICK) {
            if (!manipulator.slide.isBusy()) { // we're done moving slide.
                // we need to back off
                initPos = driveEncoder.getCurrentPosition();
                // offset in encoder ticks.
                double inchesToTicks = USE_ODOMETRY_POD ? POD_COUNTS_PER_INCH : DRIVE_COUNTS_PER_INCH;
                double offset = SPECIMEN_PICK_BACKOFF_DISTANCE_INCHES * inchesToTicks;
                targetPos = (int) Math.round(initPos + offset); // we're going forward.
                measuredPower = SPECIMEN_APPROACH_POWER;

                if (USE_AUTO_CORRECT) {
                    resetAngles();
                    tgtAngle = integratedAngle; // set the target angle equal to the current angle.
                }
                // next we need to back off
                RobotLog.vv("STATE", "SPECIMEN_DONE_PICK -> SPECIMEN_DONE_PICK_BACK_OFF");
                state = SPECIMEN_DONE_PICK_BACK_OFF;
            }
            return true; // always busy in this state
        } else if (state == SPECIMEN_DONE_PICK_BACK_OFF) {
            currentPos = driveEncoder.getCurrentPosition();
            if (currentPos > targetPos) { // are we there yet?
                stop();
                return false;
            } else { // keep going
                correction = propCorrection(P_COEFFICIENT_DRIVE);
                drive(measuredPower, 0, correction);
                return true;
            }
        } else {
            RobotLog.vv("STATE", "Unknown state, calling stop()");
            stop();
            return false;
        }
    }
}