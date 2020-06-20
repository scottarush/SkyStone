package org.firstinspires.ftc.teamcode.drivetrain;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.util.OneShotTimer;

import java.util.ArrayList;
import java.util.Iterator;

public abstract class Drivetrain {
    /* local OpMode members. */
    public HardwareMap mHWMap = null;

    /**
     * Called when linear heading correction is non-zero. Subclasses must correct with
     * an adjustement to the left if correction > 0, adjustment to the right if < 0.
     *
     */
    public abstract void correctHeading(double correction);
    /**
     * Implemented by subclasses to indicate that the target position has been reached.
     * The reason for this is that when the motors stall, the target position is never reached
     * and the encoder fail timeout occurs.
     */
    protected abstract boolean isTargetPositionReached();

    /**
     * threshold angle delta for completion of a rotation must be provided by subclasses for each
     * drivetrain
     */
    protected abstract double getMinRotationCompleteAngle();


    /** OpMode in order to access telemetry from subclasses. **/
    protected OpMode mOpMode;
    private OneShotTimer mDriveByEncoderFailTimer = null;
    private OneShotTimer mTimedDriveTimer = null;
    private OneShotTimer mRotationTimeoutTimer = null;

    private long mLastIMUCheckTime = 0l;

    /**
     * maximum rotation speed.
     */
    private double mMaxRotationPower = 0d;
    /**
     * last angle for the imu measurement
     */
    protected Orientation mLastIMUOrientation = new Orientation();
    protected double mHeadingAngle = 0d;
    protected int mRotationTargetAngle = 0;


    protected IMU mIMU;

    private static final boolean ENABLE_LINEAR_DRIVE_CORRECTION = false;
    /**
     * Current linear drive session power.
     */
    protected double mLinearDrivePower = 1.0d;



    private ArrayList<IDriveSessionStatusListener> mDriveSessionStatusListeners = new ArrayList<>();
    private ArrayList<IRotationStatusListener> mRotationStatusListeners = new ArrayList<>();


    public Drivetrain(OpMode opMode, IMU imu) {
        init(opMode);
        mIMU = imu;
    }

    private void init(OpMode opMode){
        this.mOpMode = opMode;
        mTimedDriveTimer = new OneShotTimer(1000, new OneShotTimer.IOneShotTimerCallback() {
            @Override
            public void timeoutComplete() {
                // Get actual distance moved
                double distance = getActualEncoderDistance();
                stop();
                 for(Iterator<IDriveSessionStatusListener>iter=mDriveSessionStatusListeners.iterator();iter.hasNext();){
                    IDriveSessionStatusListener listener = iter.next();
                    listener.driveComplete(distance);
                }
            }
        });

        mDriveByEncoderFailTimer = new OneShotTimer(1000, new OneShotTimer.IOneShotTimerCallback() {
            @Override
            public void timeoutComplete() {
                // Get actual distance moved
                double distance = getActualEncoderDistance();
                stop();
                for(Iterator<IDriveSessionStatusListener>iter=mDriveSessionStatusListeners.iterator();iter.hasNext();){
                    IDriveSessionStatusListener listener = iter.next();
                    listener.driveByEncoderTimeoutFailure(distance);
                }
            }
        });
        mRotationTimeoutTimer = new OneShotTimer(1000, new OneShotTimer.IOneShotTimerCallback() {
            @Override
            public void timeoutComplete() {
                // Get the actual angle as an integer to nearest degree
                int angle = (int)Math.round(getAngle());
                stop();
                for(Iterator<IRotationStatusListener>iter=mRotationStatusListeners.iterator();iter.hasNext();){
                    IRotationStatusListener listener = iter.next();
                    listener.rotationTimeout(angle);
                }
            }
        });
    }

    /**
     * Must be overridden by subclasses to return the actual distance in inches moved on an
     * encoder or strafe operation
     */
    protected abstract double getActualEncoderDistance();
    /**
     * Must be overridden by subclasses to return the linearMillisecondsPerInch rate.
      */
    public abstract int getLinearMillisecondsPerInch();

    /**
     * Must be overridden by subclasses to return the linear Kproportional constant.
     */
    public abstract double getLinearKp();
    /**
     * Must be implemented by subclasses to return the rotation Kproportional constant
     */
    public abstract double getRotationKp();


    /* Initialize standard Hardware interfaces.
     * NOTE:  This class throws Exception on any hardware initIMU error so be sure to catch and
     * report to Telemetry in your initialization. */
    public abstract void init(HardwareMap ahwMap) throws Exception;



    /**
     * Must be called from the OpMode service loop to service rotation and encoder timers
     */
    public void loop(){

        // Service rotation and heading corrections
        long currentTime = System.currentTimeMillis();
        long delta = currentTime- mLastIMUCheckTime;
        if (delta > 50) {
            mLastIMUCheckTime = currentTime;
            checkRotation();
            // if a linear drive is active then get a correction and call the correct power function in the
            // subclass if non-zero
            if (ENABLE_LINEAR_DRIVE_CORRECTION){
                double correction = getHeadingCorrection();
                correctHeading(correction);
            }
            // And the encoder drive
            checkEncoderDrive();
        }
        // And service all the timers
        mDriveByEncoderFailTimer.checkTimer();
        mTimedDriveTimer.checkTimer();
        mRotationTimeoutTimer.checkTimer();

     }

    /**
     * starts a drive by encoder session.  Base class handles failure timer. Subclasses
     * override to add drivetrain specific behavior.  Subclasses should register for
     * ICraneMovementStatusListener to receive notification of session timeout failure.
     *
     * @param speed   speed to move
     * @param linearDistance distance in inches to move + is forward, - is backward
     * @param timeoutms timeout in ms to abort if move not completed.
     *
     * @return true if session started, false on error.
     */
     public void driveEncoder(double speed, double linearDistance, int timeoutms) {
         stop();  // case we were moving
         if (ENABLE_LINEAR_DRIVE_CORRECTION){
                 // Reset the angle in the IMU logic if we are using linear drive
                 resetAngle();

             mLinearDrivePower = speed;
             if (mLinearDrivePower > 1.0d) {
                 mLinearDrivePower = 1.0d;
             }

             if (linearDistance < 0.0d) {
                 mLinearDrivePower *= -1.0d;
             }
         }
         mDriveByEncoderFailTimer.setTimeout(timeoutms);
         mDriveByEncoderFailTimer.start();
    }

    /**
     * Drives for a set distance using open-loop, robot-specific time.  Power is assumed to be linear
     * in the computation of the time.
     * @param linearDistance desired distance + forward or - rearward in inches
     * @return time to drive in ms
     */
    public int driveLinearTime(double linearDistance, double power){
        stop();  // case we were moving
        // Reset the angle in the IMU logic
        resetAngle();

        mLinearDrivePower = power;
        if (mLinearDrivePower > 1.0d){
            mLinearDrivePower = 1.0d;
        }
        if (linearDistance < 0.0d){
            mLinearDrivePower *= -1.0d;
        }

        int timeoutms = (int) Math.abs(Math.round(linearDistance * getLinearMillisecondsPerInch() * mLinearDrivePower));
        mTimedDriveTimer.setTimeout(timeoutms);
        mTimedDriveTimer.start();
//        ENABLE_LINEAR_DRIVE_CORRECTION = true;
        return timeoutms;
    }


    /**
     * Starts IMU rotation.
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * base class function does everything except set motor power.
     * @param power 0 to 1.0
     * @param ccwDegrees Degrees to turn, - is right + is left
     * @param timeoutms milliseconds to abort if not finished.  Triggers a rotationTimeout
     */
    public void rotate(int ccwDegrees,double power,int timeoutms) {
        stop();  // case we were moving

        mMaxRotationPower = power;
        // restart imu movement tracking.
        resetAngle();
        mRotationTargetAngle = ccwDegrees;

        mRotationTimeoutTimer.setTimeout(timeoutms);
        mRotationTimeoutTimer.start();

        // Call checkRotation once to initialize imu checking
        checkRotation();
    }
    public boolean isRotationActive(){
        return mRotationTimeoutTimer.isRunning();
    }
    /**
     * Resets the cumulative angle tracking in the IMU to zero
     */
    protected void resetAngle()
    {
        if (mIMU != null) {
            if (!mIMU.isIMUInitialized())
                return ;
        }

        mLastIMUOrientation = mIMU.getBNO055IMU().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        mHeadingAngle = 0;
    }
    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    protected double getAngle() {
        if (mIMU != null) {
            if (!mIMU.isIMUInitialized())
                return 0d;
        }
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = mIMU.getBNO055IMU().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - mLastIMUOrientation.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        mHeadingAngle += deltaAngle;

        mLastIMUOrientation = angles;

        return mHeadingAngle;
    }
    /**
     * Called to check a rotation and compute the correction value use to set the
     * motor power in subclasses.
     * @return correction value from +/- 1.0 depending upon the error for +/- 180.
     */
    protected double checkRotation() {
        if (!mRotationTimeoutTimer.isRunning()) {
            return 0.0;
        }

        double angle = getAngle();
        // On a right turn we have to get off zero first so return
        // to keep turning.
//        if (angle == 0d) {
//            continueTurning = true;
//        }

        // if on a left turn, then we are waiting for the angle to go
        // positive up to the rotation target angle
        // if on a right turn, then we are waiting for the angle to become
        // more negative than the target angle
        double error = mRotationTargetAngle - angle;
        double absError = Math.abs(error);
        if (absError < getMinRotationCompleteAngle()) {
             // Otherwise we are done so stop the rotation, reset the angle, and notify listeners
            // that the rotation is complete
            stop();
            mRotationTimeoutTimer.cancel();
             // Reset for next time
            resetAngle();

            for(Iterator<IRotationStatusListener>iter=mRotationStatusListeners.iterator();iter.hasNext();){
                IRotationStatusListener listener = iter.next();
                listener.rotationComplete((int)(Math.round(angle)));
            }
           return 0.0d;
        }

        // Otherwise, normalize error to +/- 1.0 is +/- 180 degrees;
        error = error / 180d;

        // And multiply by the gain
        error = error * getRotationKp();
        // And limit to the max power
        if (Math.abs(error) > mMaxRotationPower){
            error = mMaxRotationPower * Math.signum(error);
        }
        return error;
     }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right limited to +1.0 and -1.0
     */
    private double getHeadingCorrection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction = 0.0d;
        double currentAngle = 0.0d;

        correction = getAngle();

        correction = correction * getLinearKp();

        if (Math.abs(correction) > 1.0d){
            correction = Math.signum(correction);
        }

        return correction;
    }


    /**
     * called to add a listener for drive status.
     * @param listener
     */
    public void addDriveSessionStatusListener(IDriveSessionStatusListener listener){
        if (mDriveSessionStatusListeners.contains(listener)){
            return;
        }
        mDriveSessionStatusListeners.add(listener);
    }

    /**
     * base class function must be called by subclasses
     */
    public void stop(){
        mRotationTimeoutTimer.cancel();
//        ENABLE_LINEAR_DRIVE_CORRECTION = false;
        mTimedDriveTimer.cancel();
        mDriveByEncoderFailTimer.cancel();
    }

    /**
     * continues a drive by encoder session.
     */
    private void checkEncoderDrive() {
        boolean active = mDriveByEncoderFailTimer.checkTimer();
        // Timer still active so check subclass if target position has been reached.
        if (active && isTargetPositionReached()){
            // Get actual distance moved
            double distance = getActualEncoderDistance();
            stop();    // In case the motors were actually locked up within the mininum
            mDriveByEncoderFailTimer.cancel();
            // motors stopped before timeout so signal success to listeners
            for(Iterator<IDriveSessionStatusListener>iter=mDriveSessionStatusListeners.iterator();iter.hasNext();){
                IDriveSessionStatusListener listener = iter.next();
                listener.driveComplete(distance);
            }
        }
    }

    /**
     * adds listeners for rotation complete event.
     */
    public void addRotationStatusListener(IRotationStatusListener listener){
        if (mRotationStatusListeners.contains(listener))
            return;
        mRotationStatusListeners.add(listener);
    }


    /**
         * @return the amount of time that an encoder session has been active or 0 if no session active.
         */
    public double getDriveByEncoderTime(){
        if (!mDriveByEncoderFailTimer.isRunning()){
            return 0.0d;
        }
        else
            return mDriveByEncoderFailTimer.getElapsedTimeMS();
    }
    /**
     * @return true if drive by encoder session active, false otherwise
     */
    public boolean isDriveByEncoderSessionActive(){
        return mDriveByEncoderFailTimer.isRunning();
    }

     /**
         *  Utility function to handle motor initialization.  initIMU must have been called
         *  with a non-null mHWMap or exception will be thrown.
         *
         */
    protected DcMotor tryMapMotor(String motorName) throws Exception {
        DcMotor motor = null;
        try {
            motor = mHWMap.get(DcMotor.class, motorName);
        }
        catch(Exception e){
            // Throw an exception for the caller to catch so we can debug.
            throw new Exception ("Cannot map motor: "+motorName);
        }
        return motor;
    }


}
