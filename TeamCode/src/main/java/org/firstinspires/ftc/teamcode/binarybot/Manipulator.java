package org.firstinspires.ftc.teamcode.binarybot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.DeviceConfiguration;

public class Manipulator {

    // ******************************************************************
    // enumerations
    // ******************************************************************
    public enum ManipulatorState {
        AVAILABLE,
        TRANSFER_WAIT_FOR_SLIDE,
        TRANSFER_WAIT_FOR_SHOULDER,
        TRANSFER_OPEN_CLAW,
        DUMP_HIGH_WAIT_FOR_SLIDE,
        DUMP_HIGH_TIP_BUCKET,
        DUMP_HIGH_UNTIP_BUCKET,
        DEPLOY_MOVE_SLIDE,
        DEPLOY_MOVE_SHOULDER,
        PICK_FROM_FLOOR_MOVE_SHOULDER,
        PICK_FROM_FLOOR_CLOSE_CLAW
    }

    // ******************************************************************
    // constants.
    // ******************************************************************
    //default motor speed
    public static final double MOTOR_SPEED = .5;
    // ******************************************************************
    // slide-related constants
    // ******************************************************************
    // motor power
    public static final double SLIDE_POWER = 1;

    // slide motor positions.
    public static final int MAX_SLIDE_EXTENDED_POSITION = 4900;
    public static final int SLIDE_MID_POSITION = 3000;
    public static final int MIN_SLIDE_RETRACTED_POSITION = 0;
    public static final int SLIDE_EXTENDED_POSITION = 4250;
    public static final int SLIDE_RETRACTED_POSITION = 0;
    public static int SLIDE_TRANSFER_POSITION = 400;
    public static int SLIDE_HIGH_DUMP_POSITION = 4900;
    public static int SLIDE_HIGH_SPECIMEN_POSITION = 3700;
    public static int SLIDE_HIGH_SPECIMEN_RELEASE = 2450;
    public static int SLIDE_SPECIMEN_PICK = 500;

    // step size for adjusting position.
    public static final int SLIDE_DELTA = 10;

    // ******************************************************************
    // shoulder-related constants
    // ******************************************************************
    // shoulder motor power
    public static final double SHOULDER_POWER = 0.8;

    // shoulder motor positions
    public static int SHOULDER_ELBOW_BOUNDARY = 4200;
    public static int SHOULDER_CLEAR_BAR = 200;

    public static final int MAX_SHOULDER_POSITION = 8000;
    public static final int MIN_SHOULDER_POSITION = 0;
    public static final int SHOULDER_PICK_POSITION = 5950;
    public static final int SHOULDER_TILT_BOUNDARY = 4100;
    private static final int SHOULDER_OPENEING_SAFTEY = 2000;

    public static int SHOULDER_TRANSFER = 2600;
    public static int SHOULDER_AFTER_TRANSFER = 5200;

    // step size for adjusting position
    public static final int SHOULDER_DELTA = 10;

    // ******************************************************************
    // bucket-related constants
    // ******************************************************************
    public static final double BUCKET_NEUTRAL_POSITION = 0.55;

    // ******************************************************************
    // elbow-related constants
    // ******************************************************************
    public static double ELBOW_DEPLOYED = 0.95;
    public static double ELBOW_AFTER_TRANSFER;
    public static double ELBOW_TRANSFER = 0.8;
    public static double ELBOW_CLEAR_BAR = 0.55;
    public static double ELBOW_RETRACTED = 0.0;

    // ******************************************************************
    // tilt-related constants
    // ******************************************************************
    public static double TILT_DEPLOYED = 0;
    private static double TILT_RETRACTED = 0.75;
    private static double TILT_LEFT = 0.8;
    private static double TILT_RIGHT = 1;
//    NOT CORRECT VALUE
    public static final double GREEN_DEPLOYED =.65;
    public static final double GREEN_RETRACTED = .45;
    // ******************************************************************
    // claw-related constants
    // ******************************************************************
    //public static double CLAW_OPENED = 1;
    public static double CLAW_OPENED = 0.7;
    public static double CLAW_CLOSED = 0.5;

    // ******************************************************************
    // wrist-related constants
    // ******************************************************************
    public static double WRIST_ROTATED_POSITION = 0.65;
    public static double WRIST_UNROTATED_POSITION = 0.15;

    // ******************************************************************
    // timing-related constants
    // ******************************************************************

    // time (msec) before moving the shoulder after the block has been released.
    public static int TRANSFER_DELAY = 150;
    public static int DUMP_HIGH_TIP_DELAY = 600;
    public static int DUMP_HIGH_UNTIP_DELAY = 700;
    public static int PICK_CLOSING_CLAW_DELAY = 300;
    public static int LRDistance;
    public static int RRDistance;


    // ******************************************************************
    // private member variables.
    // ******************************************************************
    public DcMotor shoulder;
    public DcMotor slide;
    private Servo claw;
    private Servo bucket;
    private Servo wrist;
    public Servo tilt;
    public Servo elbow;
    public Servo greenThing;
    public DigitalChannel limitSlide;
    public DigitalChannel limitShoulder;
   public DistanceSensor rightRearDistance;
   public DistanceSensor leftRearDistance;
    private ManipulatorState manipulatorState = ManipulatorState.AVAILABLE;
    private long startTime = 0;
    private OpMode opMode;
    private HardwareMap hardwareMap;

    // ******************************************************************
    // state variables.
    // ******************************************************************
    private boolean  clawOpen = false;

    // ******************************************************************
    // construction
    // ******************************************************************
    public Manipulator(HardwareMap hardwareMap, OpMode opMode) {
        this.opMode = opMode;
        this.hardwareMap = hardwareMap;

        // get references to the manipulator hardware.
        shoulder = hardwareMap.dcMotor.get("shoulder");
        slide = hardwareMap.dcMotor.get("slide");
        claw = hardwareMap.servo.get("claw");
        wrist = hardwareMap.servo.get("wrist");
        tilt = hardwareMap.servo.get("tilt");
        elbow = hardwareMap.servo.get("elbow");
//        GREEN THING- rubber thing on servo
        greenThing = hardwareMap.servo.get("greenThing");
        //get references to the limit switches
        limitShoulder = hardwareMap.get(DigitalChannel.class, "limitShoulder");
        limitSlide = hardwareMap.get(DigitalChannel.class, "limitSlide");
        //get references to the distance sensors
        rightRearDistance = hardwareMap.get(DistanceSensor.class, "rightRearDistance");
        rightRearDistance = hardwareMap.get(DistanceSensor.class, "rightRearDistance");
        leftRearDistance = hardwareMap.get(DistanceSensor.class, "leftRearDistance");
        // set zero power brake mode for motors.
        shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // reverse the shoulder motor so positive position corresponds to deployed arm.
        shoulder.setDirection(DcMotorSimple.Direction.REVERSE);

        // put into run to position mode.
        shoulder.setTargetPosition(shoulder.getCurrentPosition());
        shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoulder.setPower(SHOULDER_POWER);

        // put slide into run to position mode.
        slide.setTargetPosition(slide.getCurrentPosition());
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(SLIDE_POWER);
        
        // reverse the slide motor so positive position corresponds to elevated slide.
        slide.setDirection(DcMotorSimple.Direction.REVERSE);

        // bucket.
        bucket = hardwareMap.servo.get("bucket");
    }

    public void pickFromFloor() {
        if (manipulatorState != ManipulatorState.AVAILABLE) {
            // already busy.
            // can't pick right now.
            return;
        }

        // open the claw
        openClaw();

        // set shoulder target position.
        shoulder.setTargetPosition(SHOULDER_PICK_POSITION);

        // set state.
        manipulatorState = ManipulatorState.PICK_FROM_FLOOR_MOVE_SHOULDER;
    }

    /**
     * reset servo and slide positions for initial state.
     */
    public void resetPositions() {
        resetSlide();

        // test servo.
        bucket.setPosition(BUCKET_NEUTRAL_POSITION);

        // retract elbow.
        elbow.setPosition(ELBOW_RETRACTED);

        // retract tilt
        tilt.setPosition(TILT_RETRACTED);
        //Retract green thing
        greenThing.setPosition(GREEN_RETRACTED);
    }
    public void calibrate(){
        boolean shouldCalibrate = true; //limitShoulder.getState() == false;
        if (shouldCalibrate) {
            //limit switch is pressed
            // zero encoder.
            shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            //set back to run to position mode
            shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        if (shouldCalibrate) {
            slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
    // ******************************************************************
    // private/helper methods
    // ******************************************************************
    // reset the slide encoder to zero.
    private void resetSlide() {
        // reset encoder to zero.
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // put into run to position mode.
        slide.setTargetPosition(slide.getCurrentPosition());
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(SLIDE_POWER);
    }

    // update the tilt position depending on current shoulder position.
    private void updateTilt() {
        // the elbow position depends on shoulder position.
        int pos = shoulder.getCurrentPosition();

        if (pos > SHOULDER_TILT_BOUNDARY) {
            tilt.setPosition(TILT_DEPLOYED);
        } else {
            tilt.setPosition(TILT_RETRACTED);
        }
    }

    // ******************************************************************
    // public methods
    // ******************************************************************
    public boolean isAvailable() {
        if (manipulatorState == ManipulatorState.AVAILABLE) {
            return true;
        } else {
            return false;
        }
    }
    public void tiltBucket (float input) {
        bucket.setPosition(input*0.45 + BUCKET_NEUTRAL_POSITION);
    }

    public void updateElbow() {
        // the elbow position depends on shoulder position.
        int pos = shoulder.getCurrentPosition();


        if (pos > SHOULDER_ELBOW_BOUNDARY) {
            elbow.setPosition(ELBOW_DEPLOYED);
        } else if (pos > SHOULDER_CLEAR_BAR) {
            elbow.setPosition(ELBOW_CLEAR_BAR);
        } else {
            elbow.setPosition(ELBOW_RETRACTED);
        }
    }

    public void startTransfer() {
        // make sure shoulder is not too close to the slide.
        if (shoulder.getCurrentPosition() < SHOULDER_TRANSFER - 25) {
            return;
        } if (manipulatorState != ManipulatorState.AVAILABLE) {
            // already busy.
            return;
        } else {
                // start the transfer.
                manipulatorState = ManipulatorState.TRANSFER_WAIT_FOR_SLIDE;

                // lower slide
                // lower slide.
                slide.setTargetPosition(SLIDE_TRANSFER_POSITION);

                // tilt claw.
                tilt.setPosition(TILT_RETRACTED);

                // make sure elbow is retracted.
                elbow.setPosition(ELBOW_RETRACTED);

                // move wrist to unrotated position.
                unrotateWrist();

        }
    }

    /**
     * Dump a sample into high corner goal.
     */
    public void startHighDump() {
        if (manipulatorState != ManipulatorState.AVAILABLE) {
            // already busy.
            return;
        } else {
            // start high dump.
            manipulatorState = ManipulatorState.DUMP_HIGH_WAIT_FOR_SLIDE;
            slide.setTargetPosition(SLIDE_HIGH_DUMP_POSITION);
        }
    }

    /**
     * If the manipulator is busy, update its state.
     * @return true if still busy, false if available.
     */
    public boolean update() {
        long current = 0;
        long elapsed = 0;
        switch(manipulatorState) {
            case AVAILABLE:
                return false;
            case TRANSFER_WAIT_FOR_SLIDE:
                if (slide.isBusy() == false) {
                    // slide is done.
                    // start moving shoulder.
                    manipulatorState = ManipulatorState.TRANSFER_WAIT_FOR_SHOULDER;
                    // move shoulder.
                    shoulder.setTargetPosition(SHOULDER_TRANSFER);
                    // indicate that we are still busy.
                    return true;
                } else {
                    // slide is still moving.
                    // indicate that we are still busy.
                    return true;
                }
            case TRANSFER_WAIT_FOR_SHOULDER:
                if (shoulder.isBusy() == false) {
                    // shoulder is done.
                    manipulatorState = ManipulatorState.TRANSFER_OPEN_CLAW;
                    openClaw();
                    // get the start time for our wait period.
                    startTime = System.currentTimeMillis();
                    // indicate that we're still busy.
                    return true;
                } else {
                    // indicate that we are still busy.
                    return true;
                }
            case TRANSFER_OPEN_CLAW:
                // check the elapsed time.
                current = System.currentTimeMillis();
                elapsed = current - startTime;
                if (elapsed > TRANSFER_DELAY) {
                    // move shoulder out of the way.
                    shoulder.setTargetPosition(SHOULDER_AFTER_TRANSFER);
                    // manipulator is available.
                    manipulatorState = ManipulatorState.AVAILABLE;
                    // we're done.
                    return false;
                } else {
                    // not enough time has elapsed.
                    // we're still busy.
                    return true;
                }
            case DUMP_HIGH_WAIT_FOR_SLIDE:
                if(slide.isBusy() == false) {
                    // slide is done and should be in position.
                    // tip bucket.
                    manipulatorState = ManipulatorState.DUMP_HIGH_TIP_BUCKET;
                    tipBucket();
                    // get start time (to keep track of elapsed time while we wait).
                    startTime = System.currentTimeMillis();
                    // indicate that we're still busy.
                    return true;
                } else {
                    // manipulator is still busy.
                    return true;
                }
            case DUMP_HIGH_TIP_BUCKET:
                // check the elapsed time.
                current = System.currentTimeMillis();
                elapsed = current - startTime;
                if (elapsed > DUMP_HIGH_TIP_DELAY) {
                    // untip bucket.
                    untipBucket();
                    manipulatorState = ManipulatorState.DUMP_HIGH_UNTIP_BUCKET;
                    // get start time (to keep track of elapsed time while we wait).
                    startTime = System.currentTimeMillis();
                    // indicate that we're still busy.
                    return true;
                } else {
                    // not enough time has elapsed.
                    // we're still busy.
                    return true;
                }
            case DUMP_HIGH_UNTIP_BUCKET:
                // check the elapsed time.
                current = System.currentTimeMillis();
                elapsed = current - startTime;
                if (elapsed > DUMP_HIGH_UNTIP_DELAY) {
                    // we're done.
                    manipulatorState = ManipulatorState.AVAILABLE;
                    // indicate that we are done.
                    return false;
                } else {
                    // not enough time has elapsed.
                    // we're still busy.
                    return true;
                }
            case DEPLOY_MOVE_SLIDE:
                //Is it done moving?
                if(slide.isBusy() == false) {
                    //slide is done
                    manipulatorState = manipulatorState.DEPLOY_MOVE_SHOULDER;
                    shoulder.setTargetPosition(SHOULDER_TILT_BOUNDARY);
                    //indicate still busy
                    return true;
                } else {
                    //indicate still busy
                    return true;
                }
            case DEPLOY_MOVE_SHOULDER:
                //Done moving?
                if (shoulder.isBusy()==false) {
                    //shoulder is done moving
                    //rotate wrist
                    rotateWrist();
                    //set state back to available
                    manipulatorState = ManipulatorState.AVAILABLE;
                    //return false because not moving
                    return false;
                }else {
                    return true;
                }
            case PICK_FROM_FLOOR_MOVE_SHOULDER:
                if (shoulder.isBusy()) {
                    // shoulder is still moving.
                    return true;
                } else {
                    // close the claw.
                    closeClaw();
                    manipulatorState = ManipulatorState.PICK_FROM_FLOOR_CLOSE_CLAW;

                    // reset timing variables.
                    startTime = System.currentTimeMillis();

                    // indicate that we're still busy.
                    return true;
                }
            case PICK_FROM_FLOOR_CLOSE_CLAW:
                current = System.currentTimeMillis();
                elapsed = current - startTime;
                if (elapsed > PICK_CLOSING_CLAW_DELAY) {
                    // done waiting.
                    manipulatorState = ManipulatorState.AVAILABLE;
                    return false;
                } else {
                    // we're still waiting.
                    return true;
                }
            default:
                return false;
        }
    }

    /**
     * This is a blocking version of the transfer method.
     * When you invoke it, the manipulator is busy and unresponsive until process is done.
     * This method should only be used in a LinearOpMode (because of the opModeIsActivr() check).
     *
     */
    public void transfer() {
        startTransfer();
        // make sure shoulder is not too close to the slide.
//        if (shoulder.getCurrentPosition() < SHOULDER_TRANSFER - 25) {
//            return;
//        }

        // lower slide.
//        slide.setTargetPosition(SLIDE_TRANSFER_POSITION);

        // tilt claw.
//        tilt.setPosition(TILT_RETRACTED);

        // make sure elbow extended.
//        elbow.setPosition(ELBOW_DEPLOYED);

        // move wrist to unrotated position.
//        unrotateWrist();

        // delay until slide is in place
        LinearOpMode linear_op_mode = (LinearOpMode)opMode;
        long sleepTime = 40;
        while (slide.isBusy()) {
            if (linear_op_mode.opModeIsActive() == false) {
                return;
            }
            linear_op_mode.sleep(sleepTime);
        }

        // move shoulder.
        shoulder.setTargetPosition(SHOULDER_TRANSFER);

        // wait until shoulder is in place
        int shoulderPos = shoulder.getCurrentPosition();
        while (shoulderPos > SHOULDER_TRANSFER) {
            shoulderPos = shoulder.getCurrentPosition();
            if (linear_op_mode.opModeIsActive() == false) {
                // quit.
                return;
            }
            opMode.telemetry.addData("shoulder curr pos", shoulderPos);
            opMode.telemetry.addData("shoulder tgt pos", shoulder.getTargetPosition());
            opMode.telemetry.update();
            linear_op_mode.sleep(sleepTime);
        }

        opMode.telemetry.addData("shoulder curr pos", shoulderPos);
        opMode.telemetry.addData("shoulder tgt pos", shoulder.getTargetPosition());
        opMode.telemetry.update();
        linear_op_mode.sleep(sleepTime);

        // release element.
        openClaw();
        slide.setTargetPosition(SLIDE_RETRACTED_POSITION);
        // move shoulder out of the way.
        linear_op_mode.sleep(TRANSFER_DELAY);
        elbow.setPosition(ELBOW_AFTER_TRANSFER);
        shoulder.setTargetPosition(SHOULDER_AFTER_TRANSFER);
    }

    public void toggleClaw() {
        claw.setPosition((clawOpen) ? CLAW_CLOSED : CLAW_OPENED);
        clawOpen = !clawOpen;
    }

    public void openClaw() {
        clawOpen = true;
        claw.setPosition(CLAW_OPENED);
    }

    public void closeClaw() {
        clawOpen = false;
        claw.setPosition(CLAW_CLOSED);
    }

    boolean wristRotated = true;
    public void toggleWrist() {
        wrist.setPosition((wristRotated) ? WRIST_ROTATED_POSITION : WRIST_UNROTATED_POSITION);
        wristRotated = !wristRotated;
    }

    public void rotateWrist() {
        wrist.setPosition(WRIST_ROTATED_POSITION);
        wristRotated = true;
    }

    public void unrotateWrist() {
        wrist.setPosition(WRIST_UNROTATED_POSITION);
        wristRotated = false;
    }

    public void extendSlide() {
        slide.setTargetPosition(SLIDE_HIGH_DUMP_POSITION);
    }

    public void retractSlide() {
        slide.setTargetPosition(SLIDE_RETRACTED_POSITION);
    }
    public void trimSlide(float input) {
        // get current target position.
        int pos = slide.getTargetPosition();

        if (Math.abs(input) < 0.25) {
            // dead zone. ignore input.
            return;
        }

        if (input >= 0.75) {
            pos = pos + 3 * SLIDE_DELTA;
        } else if (input >= 0.5) {
            pos = pos + 2 * SLIDE_DELTA;
        } else if (input >= 0.25) {
            pos = pos + SLIDE_DELTA;
        } else if (input <= -0.75) {
            pos = pos - 3 * SLIDE_DELTA;
        } else if (input <= -0.5) {
            pos = pos - 2 * SLIDE_DELTA;
        } else if (input <= -0.25) {
            pos = pos - SLIDE_DELTA;
        }

        // don't exceed limits.
        if (pos > MAX_SLIDE_EXTENDED_POSITION) {
            pos = MAX_SLIDE_EXTENDED_POSITION;
        } else if (pos < MIN_SLIDE_RETRACTED_POSITION) {
            pos = MIN_SLIDE_RETRACTED_POSITION;
        }

        // adjust target position.
        slide.setTargetPosition(pos);
    }

    public void trimShoulder(float input) {
        // get current target position.
        int pos = shoulder.getTargetPosition();

        if (Math.abs(input) < 0.25) {
            // dead zone. ignore input.
            return;
        }

        if (shoulder.getCurrentPosition() < SHOULDER_OPENEING_SAFTEY && slide.getCurrentPosition() < SLIDE_SPECIMEN_PICK) {
            this.extendSlide();
            this.greenThing.setPosition(GREEN_DEPLOYED);
            return; // shouldent trim, shoulder is in the way
        }

        if (false) {
            if (input >= 0.75) {
                pos = pos + 3 * SHOULDER_DELTA;
            } else if (input >= 0.5) {
                pos = pos + 2 * SHOULDER_DELTA;
            } else if (input >= 0.25) {
                pos = pos + SHOULDER_DELTA;
            } else if (input <= -0.75) {
                pos = pos - 3 * SHOULDER_DELTA;
            } else if (input <= -0.5) {
                pos = pos - 2 * SHOULDER_DELTA;
            } else if (input <= -0.25) {
                pos = pos - SHOULDER_DELTA;
            }
        } {
            pos += (int)input * 50;
        }
        // don't exceed limits.
        if (pos > MAX_SHOULDER_POSITION) {
            pos = MAX_SHOULDER_POSITION;
        } else if (pos < MIN_SHOULDER_POSITION) {
            pos = MIN_SHOULDER_POSITION;
        }

        // adjust target position.
        shoulder.setTargetPosition(pos);

        // automatically update elbow (based on shoulder position).
        updateElbow();

        // automatically update tilt (based on shoulder position).
        updateTilt();
    }

    public void tiltLeft() {
        tilt.setPosition(TILT_LEFT);
    }

    public void tiltRight() {
        tilt.setPosition(TILT_RIGHT);
    }

    public void tipBucket() {
        tiltBucket(1);
    }

    public void untipBucket() {
        tiltBucket(0);
    }

    public void deploy() {


        if (manipulatorState != ManipulatorState.AVAILABLE){
            //manipulator is busy.
            return;
        }
        //moves slide to mid position
        slide.setTargetPosition(SLIDE_MID_POSITION);
        greenThing.setPosition(Manipulator.GREEN_DEPLOYED);

        //set state to DEPLOY_MOVE_SLIDE
        manipulatorState = ManipulatorState.AVAILABLE.DEPLOY_MOVE_SLIDE;
    }

    public void stop() {
        manipulatorState = ManipulatorState.AVAILABLE;
        slide.setTargetPosition(slide.getCurrentPosition());
        shoulder.setTargetPosition(shoulder.getCurrentPosition());
    }
}