package org.firstinspires.ftc.teamcode.grabberbot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.OneShotTimer;

public class FourBarArm extends Arm {

    private DcMotor mArmMotor = null;

    public static final String ARM_MOTOR_NAME = "arm_motor";
    public static final String CLAW_SERVO_NAME = "claw";
    public static final String LIMIT_SENSOR_NAME = "armlimitsw";

    public static final double CLAW_CLOSED_POSITION = 0.0d;
    public static final double CLAW_OPEN_POSITION = 1.0d;



    private Servo mClawServo = null;
    private DigitalChannel mLimitSwitch = null;

    // Constants
    public static final double ARM_GEAR_REDUCTION_RATIO = Math.pow(125d / 30d, 2d);
    public static final double HEX_MOTOR_COUNTS_PER_REVOLUTION = 545d;  // This should be 288d but we are off by a factor of 2
    public static final double COUNTS_PER_ARM_360_ROTATION = ARM_GEAR_REDUCTION_RATIO * HEX_MOTOR_COUNTS_PER_REVOLUTION;

    public static final double COUNTS_PER_DEGREE = COUNTS_PER_ARM_360_ROTATION / 360;

    public static final double MAX_ANGLE = 155;
    public static final double FULL_RETRACT_ANGLE = 0;

    private ElapsedTime mRampTimer = new ElapsedTime();

    public static final int MANUAL_MODE = 0;
    public static final int ANGLE_MODE = 1;

    private int mMode = MANUAL_MODE;

    private double mCurrentAngle = 0.0;
    private boolean mFullRetractAngleValid = true;

    public static final double EXTEND_MAX_POWER = 0.5d;
    public static final double EXTEND_START_POWER = 0.1d;
    public static final double EXTEND_RAMP_UP_TIME = 1.0d;
    public static final double EXTEND_POWER_SLOPE_PER_SEC = (EXTEND_MAX_POWER - EXTEND_START_POWER)/ EXTEND_RAMP_UP_TIME;
    public static final double RETRACT_MAX_POWER = 0.5d;
    public static final double RETRACT_RAMP_UP_TIME = 0.5d;
    public static final double RETRACT_START_POWER = 0.1d;
    public static final double RETRACT_POWER_SLOPE_PER_SEC = (RETRACT_MAX_POWER - RETRACT_START_POWER)/ RETRACT_RAMP_UP_TIME;

    private OneShotTimer mResetRetractTimer = new OneShotTimer(MAX_RESET_TO_RETRACT_TIME_MSEC, new OneShotTimer.IOneShotTimerCallback() {
        @Override
        public void timeoutComplete() {
            stop();
        }
    });
    public static final int MAX_RESET_TO_RETRACT_TIME_MSEC = 5000;
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
     * @return true if arm was initIMU'ed with angle mode
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
            initErrString += "Arm motor initIMU failed,";
        }
        try{
            mLimitSwitch = hwMap.get(DigitalChannel.class,LIMIT_SENSOR_NAME);
            mLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
        }
        catch(Exception e){
            initErrString += "Arm limit switch initIMU failed,";
        }
        // now initialize the grabber server.
        try {
            mClawServo = hwMap.get(Servo.class, CLAW_SERVO_NAME);
        } catch (Exception e) {
            initErrString += "Claw servo initIMU failed";
        }
        if (initErrString.length() > 0){
            throw new Exception(initErrString);
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

    /**
     * This function is called from the OpMode run loop continuously to start
     * the arm to the fully retracted position using the limit switch to detect when the
     * position has been reached.
     * @return true if still in progress, false otherwise or initIMU error
     */
    public boolean resetToRetractPosition(){
        if ((mArmMotor == null) || (mLimitSwitch == null))
            return false;

        // Lock this object with a mutex in this method to prevent conflict with the control
        synchronized (this) {
            if (!mResetRetractTimer.isRunning()) {
                // This is the first call.  Check if limit switch already press.
                if (mLimitSwitch.getState()){
                    return false;   // Already retracted.
                }
                // Otehrwise, stop all motors, initialize the max limit timer, and
                // begin the start
                 mArmMotor.setPower(0);
                mArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                // start the limit timer
                mResetRetractTimer.start();
                mArmMotor.setPower(RESET_TO_RETRACT_POWER);
            }
            // Otherwise it is in progress.  Check if limit switch has been pressed.
            opMode.telemetry.addData("limitsw=",mLimitSwitch.getState());
            opMode.telemetry.update();
            if (mLimitSwitch.getState()){
                // Switch has tripped.  Stop the motor and either return or set motors back
                // to position mode.
                mArmMotor.setPower(0d);
                mFullRetractAngleValid = true;
                mResetRetractTimer.cancel();
                if (mMode == ANGLE_MODE){
                    mCurrentAngle = FULL_RETRACT_ANGLE;
                }
            }
            else{
                // Service the timeout timer
                mResetRetractTimer.checkTimer();
            }
         }
         return mResetRetractTimer.isRunning();
    }

    /**
     * @return true if a start is in progress
     */
    public boolean isResetToRetractInProgress(){
        return mResetRetractTimer.isRunning();
    }
    /**
     * Starts moving the arm continously.
     *
     * @return false if the arm motor wasn't inited correctly or the arm was initialized in angle mode
     */
    @Override
    public boolean moveArm(boolean extend){
        if ((mArmMotor == null) || (mMode == ANGLE_MODE))
            return false;
        double power = mArmMotor.getPower();
        // Use absolute value of power to do the ramp calc and then set the
        // sign for extend or retract at end
        double abspower = Math.abs(power);
        if (mArmMotor.getPower() == 0d){
            // We are stopped so start the ramp timer and set power to start
            mRampTimer.reset();
            if (extend){
                abspower = EXTEND_START_POWER;
            }
            else{
                abspower = RETRACT_START_POWER;
            }
        }
        else if (power > 0d){
            // retracting.
            if (mRampTimer.time() < RETRACT_RAMP_UP_TIME){
                abspower = abspower + (RETRACT_POWER_SLOPE_PER_SEC * mRampTimer.time());
            }
            if (abspower > RETRACT_MAX_POWER){
                abspower = RETRACT_MAX_POWER;
            }
        }
        else if (power < 0d){
            // extending.  Power goes more negative until limit
            if (mRampTimer.time() < EXTEND_RAMP_UP_TIME){
                abspower = abspower +(EXTEND_POWER_SLOPE_PER_SEC * mRampTimer.time());
            }
            if (abspower > EXTEND_MAX_POWER){
                abspower = EXTEND_MAX_POWER;
            }

        }
        // If extending, then negate the absolute power
        power = abspower;
        if (extend) {
            power = -abspower;
        }

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
        if (mArmMotor == null)
            return; // initIMU error
        // Return without action if not initIMU'ed for angle mode or the retract angle is invalid
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
        mArmMotor.setTargetPosition(counts);

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

    /**
     * Helper to get arm target position for isolating null refs on error
     */
    private int getArmCurrentPosition() {
        if (mArmMotor != null)
            return mArmMotor.getCurrentPosition();
        else
            return 0;
    }

    /**
     * Sets the claw to either open or closed
     */
    public void setClaw(boolean open){
        if (mClawServo == null){
            return;
        }
        if (open){
            mClawServo.setPosition(CLAW_OPEN_POSITION);
        }
        else{
            mClawServo.setPosition(CLAW_CLOSED_POSITION);
        }
    }
}