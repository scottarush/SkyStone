package org.firstinspires.ftc.teamcode.drivetrain;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.OneShotTimer;

public abstract class Drivetrain {


    /* local OpMode members. */
    public HardwareMap hwMap = null;

    public abstract void init(HardwareMap ahwMap) throws Exception;
    public abstract void setPower(double lf, double rf, double lr, double rr);
    public abstract void stop();
    public abstract void setMotorModes(DcMotor.RunMode mode);

     /** OpMode in order to access telemetry from subclasses. **/
    protected OpMode mOpMode;
    protected ElapsedTime mDriveByEncoderTimer = new ElapsedTime();
    protected boolean mDriveByEncoderSuccess = false;
    protected boolean mDriveByEncoderActive = false;
    protected double mDriveByEncoderTimeout = 0d;
    protected OneShotTimer mDriveByTimeTimer = null;

    protected OneShotTimer mRotationByTimeTimer = null;

    private int mRotationMicrosecondsPerDegree = 7000;

    private int mLinearMillisecondsPerInch = 10;

    public Drivetrain(OpMode opMode,int rotationMicrosecondsPerDegree, int linearMillisecondsPerInch){
        this.mOpMode = opMode;
        mRotationMicrosecondsPerDegree = rotationMicrosecondsPerDegree;
        mLinearMillisecondsPerInch = linearMillisecondsPerInch;
        mDriveByTimeTimer = new OneShotTimer(1000, new OneShotTimer.IOneShotTimerCallback() {
            @Override
            public void timeoutComplete() {
                driveByTimeComplete();
            }
        });

        mRotationByTimeTimer = new OneShotTimer(1000, new OneShotTimer.IOneShotTimerCallback() {
            @Override
            public void timeoutComplete() {
                rotationByTimeComplete();
            }
        });
    }


    /**
     * starts a drive by encoder session.  If robot is moving, it will be stopped.
     *
     * @param speed   speed to move
     * @param xdist   distance in x inches to move
     * @param ydist   distance in y inches to move
     * @param timeout timeout in seconds to abort if move not completed.
     *
     * @return true if session started, false on error.
     */
     public boolean startDriveByEncoder(double speed, double xdist, double ydist, double timeout) {
        return false;
    }

    /**
     * Drives for the set distance using the linear time rate supplied in the constructor.
     * @param linearDistance desired distance + forward or - rearward
     */
    public void driveByTime(double linearDistance){
        int timeoutms = (int) Math.round(linearDistance * mLinearMillisecondsPerInch);
        mDriveByTimeTimer.setTimeout(timeoutms);
        mDriveByTimeTimer.start();
    }

    /**
     * services the drive by time if it is currently active.
     * @return true if still active, false if stopped
     */
    public boolean continueDriveByTime(){
        return mDriveByTimeTimer.checkTimer();
    }
    /**
     * called when drive time is complete and calls stop method.
     */
    public void driveByTimeComplete(){
        stop();
    }
    /**
     * called to cancel a drive by time.
     */
    public void cancelDriveByTime(){
        mDriveByTimeTimer.cancel();
        stop();
    }
    /**
     * @return true if drive by time is active.  false, otherwise
     */
    public boolean isDriveByTimeInProgress(){
        return mDriveByTimeTimer.isRunning();
    }
    /**
     * continues a drive by encoder session.  If robot is moving, it will be stopped.
     * @return true if the session is still active.  false if session complete.
     */
    public boolean continueDriveByEncoder() {
        return false;
    }
    /**
     * @return true if the last encoder movement was successful, false if it timed out.
     **/
    public boolean driveByEncoderSuccess() {
        return mDriveByEncoderSuccess;
    }

    /**
     * open loop rotate function
     */
    public void startRotationByTime(int cwDegrees) {
        int timeoutms = cwDegrees * mRotationMicrosecondsPerDegree / 1000;
        mRotationByTimeTimer.setTimeout(timeoutms);
        mRotationByTimeTimer.start();
    }
    /**
     * called when drive time is complete and calls stop method.
     */
    public void rotationByTimeComplete(){
        stop();
    }
    /**
     * continues a rotation if one is active.  Returns true on rotation still active.
     */
    public boolean continueRotation() {
        return mRotationByTimeTimer.isRunning();
    }
    /**
     * called to cancel a rotation by time
     */
    public void cancelRotationByTime(){
        mDriveByTimeTimer.cancel();
        stop();
    }
    /**
     * @return true if a rotate is still active
     */
    public boolean isRotationInProgress(){
        return mRotationByTimeTimer.isRunning();
    }

    /**
         * @return the amount of time that an encoder session has been active or 0 if no session active.
         */
    public double getDriveByEncoderTime(){
        if (!mDriveByEncoderActive){
            return 0.0d;
        }
        else
            return mDriveByEncoderTimer.time();
    }
    /**
     * @return true if drive by encoder session active, false otherwise
     */
    public boolean isDriveByEncoderSessionActive(){
        return mDriveByEncoderActive;
    }

    /**
     * @return true if the drivetrain is moving.  false if stopped.
     */
    public abstract boolean isMoving();

    /**
         *  Utility function to handle motor initialization.  init must have been called
         *  with a non-null hwMap or exception will be thrown.
         *
         */
    protected DcMotor tryMapMotor(String motorName) throws Exception {
        DcMotor motor = null;
        try {
            motor = hwMap.get(DcMotor.class, motorName);
        }
        catch(Exception e){
            // Throw an exception for the caller to catch so we can debug.
            throw new Exception ("Cannot map motor: "+motorName);
        }
        return motor;
    }

}
