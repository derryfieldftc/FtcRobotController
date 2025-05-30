package org.firstinspires.ftc.teamcode.binarybot;

import static androidx.core.math.MathUtils.clamp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Drivetrain {
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
    }

    public Pose pose;

    // keep track of previous encoder positions (as encoder counts)
    int prevLeftPos;
    int prevRightPos;
    int prevAuxPos;

    // ******************************************************************
    // Constants
    // ******************************************************************
    // Assume GoBilda 4-Bar Odometry Pod
    // see https://www.gobilda.com/4-bar-odometry-pod-32mm-wheel/
    // ******************************************************************
    static final double     POD_COUNTS_PER_REV    = 2000;
    static final double POD_WHEEL_DIAMETER_CM = 3.2;
    static final double POD_WHEEL_RADIUS_CM = POD_WHEEL_DIAMETER_CM / 2.0;

    static final double POD_WHEEL_CIRCUM_CM = Math.PI * POD_WHEEL_DIAMETER_CM;
    static final double POD_COUNTS_PER_CM = POD_COUNTS_PER_REV / POD_WHEEL_CIRCUM_CM;
    static final double     POD_COUNTS_PER_INCH         =   POD_COUNTS_PER_CM * 2.54;

    // spacing between left and right encoder wheels.
    static final double POD_SPACING_LR_CM = 41.0;

    // spacing from center point to side-to-side encoder wheel.
    static final double POD_SPACING_AUX_CM = 0.0;

    static final double     DRIVE_COUNTS_PER_REV    = 537.7;
    static final double     DRIVE_DRIVE_GEAR_REDUCTION    = 1;

    // GoBilda new mecanum wheels have 104mm diameter (4.094 inches)
    static final double     DRIVE_WHEEL_DIAMETER_INCHES   = 4.094;
    static final double     DRIVE_WHEEL_CIRCUM_INCHES = Math.PI * DRIVE_WHEEL_DIAMETER_INCHES;
    static final double     DRIVE_COUNTS_PER_INCH         =   (DRIVE_COUNTS_PER_REV * DRIVE_DRIVE_GEAR_REDUCTION) /
            (DRIVE_WHEEL_CIRCUM_INCHES);

    static final boolean USE_ODOMETRY_POD = true;
    static final double STRAFE_ENCODER_FUDGE_FACTOR = 1.0;

    // ******************************************************************
    // private member variables
    // ******************************************************************
    // drive system
    private DcMotor motorFL;
    private DcMotor motorBL;
    private DcMotor motorFR;
    private DcMotor motorBR;

    public DcMotor encoderLeft;
    public DcMotor encoderRight;
    public DcMotor encoderAux;

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
    private final double P_COEFFICIENT_STRAFE = 0.05;

    public State state = State.IDLE;

    public Drivetrain(HardwareMap hardwareMap, OpMode opMode) {
        pose = new Pose (0, 0, 0);
        this.opMode = opMode;
        this.hardwareMap = hardwareMap;
        initHardware();
    }

    public Drivetrain(HardwareMap hardwareMap, OpMode opMode, double x, double y, double theta) {
        this(hardwareMap, opMode);
        pose = new Pose(x, y, theta);
    }

    public void refreshPose() {
        // current encoder values (in counts or ticks).
        int currLeftPos = encoderLeft.getCurrentPosition();
        int currRightPos = encoderRight.getCurrentPosition();
        int currAuxPos = encoderAux.getCurrentPosition();

        // how much did each encoder count change?
        int dnLeft = currLeftPos - prevLeftPos;         // dn1
        int dnRight = currRightPos - prevRightPos;      // dn2
        int dnAux = currAuxPos - prevAuxPos;            // dn3

        // find change in center of robot in robot coordinate system.
        double dtheta = (dnRight - dnLeft) / POD_SPACING_LR_CM / POD_COUNTS_PER_CM;
        double dx = (dnRight + dnLeft) / 2.0 / POD_COUNTS_PER_CM;
        double dy = (dnAux - POD_SPACING_AUX_CM * (dnRight - dnLeft) / POD_SPACING_LR_CM) / POD_COUNTS_PER_CM;

        // find changes in field coordinates.
        double h = dtheta / 2.0;
        double deltaX = dx * Math.cos(h) - dy * Math.sin(h);
        double deltaY = dx * Math.sin(pose.theta) + dy * Math.cos(pose.theta);
        double deltaTheta = dtheta;

        // update pose.
        pose.x += deltaX;
        pose.y -= deltaY;
        pose.theta += deltaTheta;

        // current values become previous ones.
        prevLeftPos = currLeftPos;
        prevRightPos = currRightPos;
        prevAuxPos = currAuxPos;
    }

    private void initHardware() {
        // Get and configure drive motors.
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

        // get and configure encoders.
        if (USE_ODOMETRY_POD) {
            encoderLeft = hardwareMap.dcMotor.get("encoderLeft");
            encoderRight = hardwareMap.dcMotor.get("encoderRight");
            encoderAux = hardwareMap.dcMotor.get("encoderAux");

            encoderLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            encoderRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            encoderAux.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            prevLeftPos = encoderLeft.getCurrentPosition();
            prevRightPos = encoderRight.getCurrentPosition();
            prevAuxPos = encoderAux.getCurrentPosition();
        } else {
            encoderLeft = motorFL;
            encoderAux = motorFL;
        }

        initIMU();
    }

    /**
     * calibrate encoder
     * reset the strafing and drive encoders
     */
    public void resetOdometry() {
        // for now, reset the encoders
        // reset encoders.
        encoderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoderAux.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderAux.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // set prev values to zero.
        prevLeftPos = 0;
        prevRightPos = 0;
        prevAuxPos = 0;
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
     * @param twist counterclockwise / clockwise (negative) power - follows right hand rule
     */
    public void drive(double drive,  double strafe, double twist) {
        double powerFL = drive + strafe - twist;
        double powerBL = drive - strafe - twist;
        double powerFR = drive - strafe + twist;
        double powerBR = drive + strafe + twist;

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
        initPos = encoderLeft.getCurrentPosition();

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
            state = State.DRIVING_BACKWARD;
        } else {
            measuredPower = (float)Math.abs(power);
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
        initPos = encoderAux.getCurrentPosition();

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
            state = State.STRAFING_LEFT;
        } else {
            measuredPower = (float)Math.abs(power);
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
            state = State.TURNING_COUNTERCLOCKWISE;
        } else {
            measuredPower = (float)Math.abs(power);
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
}