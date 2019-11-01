package org.firstinspires.ftc.teamcode.arm;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class FourBarArm extends Arm {

    private DcMotor mArmMotor = null;

    public static final String ARM_MOTOR_NAME = "arm_motor";
    public static final String CLAW_SERVO_NAME = "claw";
    public static final String LIMIT_SENSOR_NAME = "arm_limit_sensor";

    private Servo mClawServo = null;
    private DigitalChannel mLimitSwitch = null;

     // Constants
    public static final double ARM_GEAR_REDUCTION_RATIO = Math.pow(125d / 30d, 2d);
    public static final double HEX_MOTOR_COUNTS_PER_REVOLUTION = 545d;  // This should be 288d but we are off by a factor of 2
    public static final double COUNTS_PER_ARM_360_ROTATION = ARM_GEAR_REDUCTION_RATIO * HEX_MOTOR_COUNTS_PER_REVOLUTION;

    public static final double COUNTS_PER_DEGREE = COUNTS_PER_ARM_360_ROTATION / 360;

    public static final double MAX_ANGLE = 155;
    public static final double FULL_RETRACT_ANGLE = 0;


    public static final int MANUAL_MODE = 0;
    public static final int ANGLE_MODE = 1;

    private int mMode = MANUAL_MODE;

    private double mCurrentAngle = 0.0;
    private boolean mFullRetractAngleValid = true;

    private double mMotorPower = 0.0;

    private boolean mResetToRetractInProgress = false;
    private double mResetToRetractStartTime = 0d;
    public static final double MAX_RESET_TO_RETRACT_TIME_SEC = 5d;
    public static final double RESET_TO_RETRACT_POWER = 0.5;

    /**
     * Constructs a Four-Bar arm
     * @param opMode
     * @param manualMode true for Manual, false for angle mode.
     */
    public FourBarArm(OpMode opMode, boolean manualMode) {
        super(opMode);
        if (manualMode){
            mMode = MANUAL_MODE;
        }
        else{
            mMode = ANGLE_MODE;
        }
    }

    /**
     * @return true if arm was init'ed with angle mode
     */
    public boolean isAngleMode(){
        return (mMode == ANGLE_MODE);
    }

    @Override
    public void init(HardwareMap ahwMap) throws Exception {
        hwMap = ahwMap;
        String initErrString = "";
        try {
            mArmMotor = hwMap.get(DcMotor.class, ARM_MOTOR_NAME);
        } catch (Exception e) {
            initErrString += "Arm motor init failed,";
        }
        try{
            mLimitSwitch = hwMap.get(DigitalChannel.class,LIMIT_SENSOR_NAME);
            mLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
        }
        catch(Exception e){
            initErrString += "Arm limit switch init failed,";
        }
        // now initialize the grabber server.
        try {
            mClawServo = hwMap.get(Servo.class, CLAW_SERVO_NAME);
        } catch (Exception e) {
            initErrString += "Claw servo init failed";
        }

        if (mArmMotor != null) {
            if (mMode == ANGLE_MODE) {
                mArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                mArmMotor.setTargetPosition(0);
                mArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            else{
                mArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }
    }


    public final static int RESET_RETRACT_IN_PROGRESS = 0;
    public final static int RESET_RETRACT_COMPLETE = 1;
    public final static int RESET_RETRACT_ERROR = 2;
    /**
     * This function is called from the OpMode run loop continuously to reset
     * the arm to the fully retracted position using the limit switch to detect when the
     * position has been reached.
     * @return RESET_RETRACT_IN_PROGRESS means keep calling to complete, RESET_RETRACT_COMPLETE
     * means reset is done, RESET_RETRACT_ERROR means the reset didn't complete within a maximum
     * time and cannot finish.  Also returns RESET_RETRACT_ERROR if the motor or limit switch did
     * not initialize properly.
      */
    public int resetToRetractPosition(){
        if ((mArmMotor == null) || (mLimitSwitch == null))
            return RESET_RETRACT_ERROR;

        // Lock this object with a mutex in this method
        synchronized (this) {
            if (!mResetToRetractInProgress) {
                // This is the first call.  Stop all motors, initialize the max limit timer, and
                // begin the reset
                mResetToRetractInProgress = true;
                mArmMotor.setPower(0);
                mArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                // Initialize timer to current run-time
                mResetToRetractStartTime = opMode.time;
                mArmMotor.setPower(RESET_TO_RETRACT_POWER);
                return RESET_RETRACT_IN_PROGRESS;
             }
            // Otherwise a reset is in progress.
            // Now check if limit switch has been pressed.
            if (mLimitSwitch.getState()){
                // Switch has tripped.  Stop the motor and either return or set motors back
                // to position mode.
                mArmMotor.setPower(0d);
                mFullRetractAngleValid = true;
                mResetToRetractInProgress = false;
                if (mMode == ANGLE_MODE){
                    mCurrentAngle = FULL_RETRACT_ANGLE;
                }
                return RESET_RETRACT_COMPLETE;
            }
            else{
                // arm still moving but no limit switch.  Check for max reset time
                double time = opMode.time - mResetToRetractStartTime;
                if (time > MAX_RESET_TO_RETRACT_TIME_SEC){
                    // Time exceeded but reset did not complete.  stop motor. If position mode
                    // is enabled then set error flag as we now no longer know where zero is
                    mArmMotor.setPower(0d);
                    mFullRetractAngleValid = false;
                    return RESET_RETRACT_ERROR;
                }
                // Return that the reset is still in progress
                return RESET_RETRACT_IN_PROGRESS;
            }
        }
    }

    /**
     * @return true if a reset is in progress
     */
    public boolean isResetToRetractInProgress(){
        return mResetToRetractInProgress;
    }
    /**
     * Starts moving the arm continously.
     *
     * @return false if the arm motor wasn't init correctly or the arm was initialized in angle mode
     */
    @Override
    public boolean moveArm(boolean up){
        if ((mArmMotor == null) || (mMode == ANGLE_MODE))
            return false;
        double power = mMotorPower;
        if (up == false)
            power = -power;

        mArmMotor.setPower(power);
        return true;
    }

    /**
     * @return true if arm is moving, false if stopped.
     *
     */
    public boolean isArmMoving(){
        if (mArmMotor == null)
            return true;
        return (mArmMotor.getPower() != 0.0d);
    }



    @Override
    public void stop() {
        if (mArmMotor == null)
            return;
        mArmMotor.setPower(0);
        if (mMode == ANGLE_MODE) {
            mCurrentAngle = getArmCurrentPosition() / COUNTS_PER_DEGREE;
        }
    }

    @Override
    public void gotoAngle(double targetAngle) {
        // Return without action if not init'ed for angle mode or the retract angle is invalid
        if (mMode != ANGLE_MODE){
            opMode.telemetry.addData("Error","Cannot call FourBarArm.gotoAngle in manual mode");
            opMode.telemetry.update();
            return;
        }
        if (!mFullRetractAngleValid){
            opMode.telemetry.addData("Error","Full retract angle is invalid.  Cannot move arm");
            opMode.telemetry.update();
            return;
        }

        if (targetAngle > MAX_ANGLE)
            targetAngle = MAX_ANGLE;
        if (targetAngle < FULL_RETRACT_ANGLE)
            targetAngle = FULL_RETRACT_ANGLE;
        // Compute the angle to move in degrees
        double deltaDegrees = targetAngle - mCurrentAngle;

        // Now get the number of counts to move
        int counts = (int) Math.round(deltaDegrees * COUNTS_PER_DEGREE);

        // move the arm
        setArmMotorPower(mMotorPower);
        setArmTargetPosition(counts);

        mCurrentAngle = getCurrentAngle();
    }

    /**
     * moves the arm by a delta angle from current position. ignored in manual mode
     */
    public void moveDeltaAngle(double deltaDegrees){
        if (mMode != ANGLE_MODE){
            opMode.telemetry.addData("Error","Cannot call FourBarArm.moveDeltaAngle in manual mode");
            opMode.telemetry.update();
            return;
        }
        double targetAngle = mCurrentAngle + deltaDegrees;
        if (targetAngle > MAX_ANGLE){
            targetAngle = MAX_ANGLE;
        }
        gotoAngle(targetAngle);
    }


    @Override
    public double getCurrentAngle() {
        return getArmCurrentPosition() / COUNTS_PER_DEGREE;
    }

    @Override
    public void setPower(double power) {
        mMotorPower = power;
    }

    /**
     * Internal helper to set the arm power for isolating null refs on error
     *
     * @param power
     */
    private void setArmMotorPower(double power) {
        if (mArmMotor != null) {
            mArmMotor.setPower(power);
        }
    }

    /**
     * Helper to set the arm target pos for isolating null refs on error
     *
     * @param counts
     */
    private void setArmTargetPosition(int counts) {
        if (mArmMotor != null) {
            mArmMotor.setTargetPosition(counts);
        }
    }

    /**
     * Helper to get arm target position for isolating null refs on error
     */
    private int getArmCurrentPosition() {
        if (mArmMotor != null)
            return mArmMotor.getCurrentPosition();
        else
            return 0;
    }
}