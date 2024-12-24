package org.firstinspires.ftc.teamcode.binarybot;

import static androidx.core.math.MathUtils.clamp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class BinaryBot {
    // ******************************************************************
    // enumerations.
    // ******************************************************************
    // odometry system state.
    public enum MeasuredState {
        IDLE,
        FORWARD,
        BACKWARD,
        RIGHTWARD,
        LEFTWARD,
        CLOCKWISE,
        COUNTERCLOCKWISE
    }

    // ******************************************************************
    // Constants
    // ******************************************************************
    // The bot uses REV Robotics thru bore encoders which have a resolution of
    // 2048 cycles per revolution
    static final double     COUNTS_PER_MOTOR_REV    = 2048;
    static final double     DRIVE_GEAR_REDUCTION    = 1;
    static final double     WHEEL_DIAMETER_INCHES   = 2;
    static final double     COUNTS_PER_INCH         =   (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                        (WHEEL_DIAMETER_INCHES * 3.1415);

    // ******************************************************************
    // private member variables
    // ******************************************************************
    // drive system
    private DcMotor motorFL;
    private DcMotor motorBL;
    private DcMotor motorFR;
    private DcMotor motorBR;

    DcMotor driveEncoder;
    DcMotor strafeEncoder;
    RevHubOrientationOnRobot orientationOnRobot;

    // op mode related items.
    private OpMode opMode;
    private HardwareMap hardwareMap;

    private BNO055IMU imu;
    private Orientation angles;
    private Acceleration gravity;
    private double previousAngle = 0;
    private double integratedAngle = 0;

    private boolean useAutoCorrect = true;
//    private float autocorrectPower = 0;
//    private double autocorrectError = 0;
    private final double AUTOCORRECT_P_COEFFICIENT = 1.0;

    // ******************************************************************
    // public member variables.
    // ******************************************************************
    // ITD arm.
    // make it publicly visible.
    public Manipulator manipulator;

    // current odometry state.
    public MeasuredState measuredState = MeasuredState.IDLE;

    // ******************************************************************
    // construction
    // ******************************************************************
    public BinaryBot(HardwareMap hardwareMap, OpMode opMode) {
        this.opMode = opMode;
        this.hardwareMap = hardwareMap;
        initHardware();
    }

    // ******************************************************************
    // private / helper methods.
    // ******************************************************************
    /**
     * Init robot hardware.
     */
    private void initHardware() {
        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");

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

        // init the arm.
        manipulator = new Manipulator(hardwareMap, opMode);

        // odometry.
        driveEncoder = hardwareMap.get(DcMotor.class, "drive");
        strafeEncoder = hardwareMap.get(DcMotor.class, "strafe");

        // IMU
        initIMU();
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

    private void updateAngles() {
        float currAngle;
        double deltaAngle;


        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currAngle = angles.firstAngle;
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

    private void resetAngles() {
        integratedAngle = 0;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        previousAngle = angles.firstAngle;
    }

    private double getCurrentAngle() {
        return integratedAngle;
    }

    private double getPreviousAngle() {
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

        // set state to idle.
        measuredState = MeasuredState.IDLE;
    }

    // drive a specified distance in inches.
    public int currPos = 0;
    public int initPos = 0;
    public int tgtPos = 0;
    public double tgtAngle = 0;
    public float measuredPower = 0;

    /**
     * drive forward/backwards using odometery a measured distance.
     * use negative distance to go backwards.
     *
     * @param power - forward drive power.
     * @param distance - distance to drive (inches).
     */
    public void measuredDrive(double power, double distance)  {

        // get init distance.
        // NOTE: we assume the encoder will not rollover (which it shouldn't)
        // and don't bother to check for this condition.
        initPos = driveEncoder.getCurrentPosition();

        // calculate target position.
        // number of wheel rotations.
        double wheelRev = distance / WHEEL_DIAMETER_INCHES;

        // offset in encoder ticks.
        double offset = wheelRev / COUNTS_PER_INCH;
        tgtPos = (int)offset + initPos;
        if (offset < 0) {
            measuredPower = -(float)Math.abs(power);
            measuredState = MeasuredState.BACKWARD;
        } else {
            measuredPower = (float)Math.abs(power);
            measuredState = MeasuredState.FORWARD;
        }

        // is auto correct enabled (which helps keep the bot drive straight)?
        if (useAutoCorrect) {
            // reset angles.
            resetAngles();
            // set the target angle equal to the current angle.
            tgtAngle = integratedAngle;
        }
    }

    public void measuredStrafe(double power, double distance)  {

        // get init distance.
        // NOTE: we assume the encoder will not rollover (which it shouldn't)
        // and don't bother to check for this condition.
        initPos = strafeEncoder.getCurrentPosition();

        // calculate target position.
        // number of wheel rotations.
        double wheelRev = distance / WHEEL_DIAMETER_INCHES;

        // offset in encoder ticks.
        double offset = wheelRev / COUNTS_PER_INCH;
        tgtPos   = (int)offset + tgtPos;
        if (offset < 0) {
            measuredPower = -(float)Math.abs(power);
            measuredState = MeasuredState.LEFTWARD;
        } else {
            measuredPower = (float)Math.abs(power);
            measuredState = MeasuredState.RIGHTWARD;
        }

        // is auto correct enabled (which helps keep the bot drive straight)?
        if (useAutoCorrect) {
            // reset angles.
            resetAngles();
            // set the target angle equal to the current angle.
            tgtAngle = integratedAngle;
        }
    }

    public void measuredTurn(double power, double angle)  {
        // reset angles.
        resetAngles();

        tgtAngle = angle;
        if (tgtAngle < 0) {
            measuredPower = -(float)Math.abs(power);
            measuredState = MeasuredState.COUNTERCLOCKWISE;
        } else {
            measuredPower = (float)Math.abs(power);
            measuredState = MeasuredState.CLOCKWISE;
        }
    }

    private float propCorrection() {
        float value;
        double error;
        // are we using auto correct?
        if (useAutoCorrect) {
            // update angles.
            updateAngles();

            // calculate error (degrees).
            error = integratedAngle - tgtAngle;

            // corrective "twist" power is proportional to error.
            // should be applied in opposite direction.
            value = -(float)(AUTOCORRECT_P_COEFFICIENT * error);
        } else {
            // no autocorrect power will be applied.
            value = 0;
        }
        return value;
    }
    /**
     *
     * @return true if still in measured mode.
     */
    public boolean measuredUpdate() {
        float correction = 0;
        switch(measuredState) {
            case IDLE:
                // done.
                return false;
            case FORWARD:
                // update current position
                currPos = driveEncoder.getCurrentPosition();
                // are we there yet?
                if (currPos > tgtPos) {
                    measuredState = MeasuredState.IDLE;
                    return false;
                } else {
                    correction = propCorrection();
                    drive(measuredPower, 0, correction);
                    return true;
                }
            case BACKWARD:
                // update current position
                currPos = driveEncoder.getCurrentPosition();
                // are we there yet?
                if (currPos < tgtPos) {
                    measuredState = MeasuredState.IDLE;
                    return false;
                } else {
                    correction = propCorrection();
                    drive(measuredPower, 0, correction);
                    return true;
                }
            case RIGHTWARD:
                // update current position
                currPos = strafeEncoder.getCurrentPosition();
                // are we there yet?
                if (currPos > tgtPos) {
                    measuredState = MeasuredState.IDLE;
                    return false;
                } else {
                    correction = propCorrection();
                    drive(measuredPower, 0, correction);
                    return true;
                }
            case LEFTWARD:
                // update current position
                currPos = strafeEncoder.getCurrentPosition();
                // are we there yet?
                if (currPos < tgtPos) {
                    measuredState = MeasuredState.IDLE;
                    return false;
                } else {
                    correction = propCorrection();
                    drive(measuredPower, 0, correction);
                    return true;
                }
            case CLOCKWISE:
                // update angles
                updateAngles();

                if (integratedAngle > tgtAngle) {
                    measuredState = MeasuredState.IDLE;
                    return false;
                } else {
                    drive(0, 0, measuredPower);
                    return true;
                }
            case COUNTERCLOCKWISE:
                // update angles
                updateAngles();

                if (integratedAngle < tgtAngle) {
                    measuredState = MeasuredState.IDLE;
                    return false;
                } else {
                    drive(0, 0, measuredPower);
                    return true;
                }
            default:
                return false;
        }
    }
}
