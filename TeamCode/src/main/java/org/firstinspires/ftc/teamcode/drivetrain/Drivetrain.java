package org.firstinspires.ftc.teamcode.drivetrain;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.OneShotTimer;

import java.util.ArrayList;
import java.util.Iterator;

public abstract class Drivetrain {
    /* local OpMode members. */
    public HardwareMap hwMap = null;

    public abstract void init(HardwareMap ahwMap) throws Exception;
    public abstract void setPower(double lf, double rf, double lr, double rr);
    public abstract void stop();
    public abstract void setMotorModes(DcMotor.RunMode mode);

     /** OpMode in order to access telemetry from subclasses. **/
    protected OpMode mOpMode;
    private OneShotTimer mDriveByEncoderFailTimer = null;

    private OneShotTimer mDriveByTimeTimer = null;

    private OneShotTimer mRotationByTimeTimer = null;

    private int mRotationMicrosecondsPerDegree = 7000;

    private int mLinearMillisecondsPerInch = 10;

    private ArrayList<IDriveSessionStatusListener> mDriveSessionStatusListeners = new ArrayList<>();
    private ArrayList<IRotationStatusListener> mRotationStatusListeners = new ArrayList<>();

    public Drivetrain(OpMode opMode,int rotationMicrosecondsPerDegree, int linearMillisecondsPerInch){
        this.mOpMode = opMode;
        mRotationMicrosecondsPerDegree = rotationMicrosecondsPerDegree;
        mLinearMillisecondsPerInch = linearMillisecondsPerInch;
        mDriveByTimeTimer = new OneShotTimer(1000, new OneShotTimer.IOneShotTimerCallback() {
            @Override
            public void timeoutComplete() {
                stop();
                for(Iterator<IDriveSessionStatusListener>iter=mDriveSessionStatusListeners.iterator();iter.hasNext();){
                    IDriveSessionStatusListener listener = iter.next();
                    listener.driveComplete();
                }
            }
        });

        mRotationByTimeTimer = new OneShotTimer(1000, new OneShotTimer.IOneShotTimerCallback() {
            @Override
            public void timeoutComplete() {
                stop();
                for (Iterator<IRotationStatusListener> iter = mRotationStatusListeners.iterator(); iter.hasNext(); ) {
                    IRotationStatusListener listener = iter.next();
                    listener.rotationComplete();
                }
            }
        });
        mDriveByEncoderFailTimer = new OneShotTimer(1000, new OneShotTimer.IOneShotTimerCallback() {
            @Override
            public void timeoutComplete() {
                stop();
                for(Iterator<IDriveSessionStatusListener>iter=mDriveSessionStatusListeners.iterator();iter.hasNext();){
                    IDriveSessionStatusListener listener = iter.next();
                    listener.driveByEncoderTimeoutFailure();
                }
            }
        });
    }

    /**
     * Must be called from the OpMode service loop to service timers for drive and rotation handling.
     */
    public void doService(){
        mDriveByEncoderFailTimer.checkTimer();
        continueDriveByEncoder();
        mRotationByTimeTimer.checkTimer();
        mDriveByTimeTimer.checkTimer();
    }

    /**
     * starts a drive by encoder session.  Base class handles failure timer. Subclasses
     * override to add drivetrain specific behavior.  Subclasses should register for
     * IDriveSessionStatusListener to receive notification of session timeout failure.
     *
     * @param speed   speed to move
     * @param xdist   distance in x inches to move
     * @param ydist   distance in y inches to move
     * @param timeoutms timeout in ms to abort if move not completed.
     *
     * @return true if session started, false on error.
     */
     public void driveByEncoder(double speed, double xdist, double ydist, int timeoutms) {
         mDriveByEncoderFailTimer.setTimeout(timeoutms);
    }

    /**
     * Drives for the set distance using the linear time rate supplied in the constructor.
     * @param linearDistance desired distance + forward or - rearward in inches
     */
    public void driveByTime(double linearDistance){
        int timeoutms = (int) Math.abs(Math.round(linearDistance * mLinearMillisecondsPerInch));
        mDriveByTimeTimer.setTimeout(timeoutms);
        mDriveByTimeTimer.start();
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
     * called to cancel a drive by time.
     */
    public void cancelDriveByTime(){
        mDriveByTimeTimer.cancel();
        stop();
    }

    /**
     * continues a drive by encoder session.
     */
    private void continueDriveByEncoder() {
        boolean active = mDriveByEncoderFailTimer.checkTimer();

        if (active && !isMoving()){
            mDriveByEncoderFailTimer.cancel();
            // motors stopped before timeout so signal success to listeners
            for(Iterator<IDriveSessionStatusListener>iter=mDriveSessionStatusListeners.iterator();iter.hasNext();){
                IDriveSessionStatusListener listener = iter.next();
                listener.driveComplete();
            }
        }
    }

    /**
     * open loop rotate function.
     */
    public void doRotationByTime(int cwDegrees) {
        int timeoutms = Math.abs(cwDegrees * mRotationMicrosecondsPerDegree / 1000);
        mRotationByTimeTimer.setTimeout(timeoutms);
        mRotationByTimeTimer.start();
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
     * called to cancel a rotation by time
     */
    public void cancelRotationByTime(){
        mDriveByTimeTimer.cancel();
        stop();
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
