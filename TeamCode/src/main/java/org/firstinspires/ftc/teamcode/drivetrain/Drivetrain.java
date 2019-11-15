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

    private OneShotTimer mTimedDriveTimer = null;

    private OneShotTimer mTimedRotationTimer = null;

    private int mRotationDegreesPerSecond = 90;

    private int mLinearMillisecondsPerInch = 10;

    private ArrayList<IDriveSessionStatusListener> mDriveSessionStatusListeners = new ArrayList<>();
    private ArrayList<IRotationStatusListener> mRotationStatusListeners = new ArrayList<>();

    public Drivetrain(OpMode opMode,int rotationDegreesPerSecond, int linearMillisecondsPerInch){
        this.mOpMode = opMode;
        mRotationDegreesPerSecond = rotationDegreesPerSecond;
        mLinearMillisecondsPerInch = linearMillisecondsPerInch;
        mTimedDriveTimer = new OneShotTimer(1000, new OneShotTimer.IOneShotTimerCallback() {
            @Override
            public void timeoutComplete() {
                stop();
                for(Iterator<IDriveSessionStatusListener>iter=mDriveSessionStatusListeners.iterator();iter.hasNext();){
                    IDriveSessionStatusListener listener = iter.next();
                    listener.driveComplete();
                }
            }
        });

        mTimedRotationTimer = new OneShotTimer(1000, new OneShotTimer.IOneShotTimerCallback() {
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
        serviceEncoderDrive();
        mTimedRotationTimer.checkTimer();
        mTimedDriveTimer.checkTimer();
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
     public void encoderDrive(double speed, double xdist, double ydist, int timeoutms) {
         mDriveByEncoderFailTimer.setTimeout(timeoutms);
         mDriveByEncoderFailTimer.start();
    }

    /**
     * Drives for a set distance using open-loop, robot-specific time.
     * @param linearDistance desired distance + forward or - rearward in inches
     * @return time to drive in ms
     */
    public int doLinearTimedDrive(double linearDistance){
        int timeoutms = (int) Math.abs(Math.round(linearDistance * mLinearMillisecondsPerInch));
        mTimedDriveTimer.setTimeout(timeoutms);
        mTimedDriveTimer.start();
        return timeoutms;
    }

    /**
     * Drives in an arbitrary vector using the linear
     * @param xdistance  x coord of movement vector in in
     * @param ydistance y coord of movement vector in in
     * @return time in ms for the drive or -1 if not supported for this drivetrain.
     */
    public int doVectorTimedDrive(double xdistance,double ydistance){
        int timeoutms = -1;
        if (isVectorTimedDriveSupported()){
            double angleRate = Math.sqrt(Math.pow(getVectorTimedXMSPerInch(),2.0d) + Math.pow(getVectorTimedYMSPerInch(),2.0d));
            timeoutms = (int) Math.abs(angleRate);
            mTimedDriveTimer.setTimeout(timeoutms);
            mTimedDriveTimer.start();
        }
        return timeoutms;
    }

    /**
     * Must be implemented by subclasses to provide time values and whether or not angled drive
     * is supported.
     * base function just saves values and returns false
     * @return true if supported, false if not
     */
    public boolean isVectorTimedDriveSupported(){
        return false;
    }

    /**
     * must be implemented by subclasses to return the x timed rates if isVectorTimedDriveSupported
     * base class just returns 0
     */
    public int getVectorTimedXMSPerInch(){
        return 0;
    }

    /**
     * must be implemented by subclasses to return the \y timed rates if isVectorTimedDriveSupported
     * base class just returns 0
     */
    public int getVectorTimedYMSPerInch(){
        return 0;
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
    public void cancelTimedDrive(){
        mTimedDriveTimer.cancel();
        stop();
    }

    /**
     * continues a drive by encoder session.
     */
    private void serviceEncoderDrive() {
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
    public void doTimedRotation(int cwDegrees) {
        double dd = (double)cwDegrees;
        double rate = (double)mRotationDegreesPerSecond;
        double dt = dd/rate * 1000d;
        int timeoutms = (int)Math.round(Math.abs(dt));
        mTimedRotationTimer.setTimeout(timeoutms);
        mTimedRotationTimer.start();
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
        mTimedDriveTimer.cancel();
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
