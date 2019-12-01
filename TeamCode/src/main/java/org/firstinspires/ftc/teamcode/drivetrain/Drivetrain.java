package org.firstinspires.ftc.teamcode.drivetrain;

import com.qualcomm.hardware.bosch.BNO055IMU;
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

    public abstract void setPower(double lf, double rf, double lr, double rr);
    public abstract void setMotorModes(DcMotor.RunMode mode);

    /**
     * Called when linear heading correction is non-zero. Subclasses must correct with
     * an adjustement to the left if correction > 0, adjustment to the right if < 0.
     *
     */
    public abstract void correctHeading(double correction);

     /** OpMode in order to access telemetry from subclasses. **/
    protected OpMode mOpMode;
    private OneShotTimer mDriveByEncoderFailTimer = null;

    private OneShotTimer mTimedDriveTimer = null;

    private long mLastIMUCheckTime = 0l;
    /**
     * proportional constant for rotation angle.
     */
    private double mRotationKp = 4.0d;
    /**
     * last angle for the imu measurement
     */
    protected Orientation mLastIMUOrientation = new Orientation();
    protected double mHeadingAngle = 0d;
    protected int mRotationTargetAngle = 0;

    private boolean mRotationActive = false;

    private boolean mLinearDriveActive = false;
    /**
     * Current linear drive session power.
     */
    protected double mLinearDrivePower = 1.0d;

    /**
     * proportional constant for linear direction correction.
     */
    private double mLinearKp = 1.0d;
    /**
     * IMU inside REV hub
     */
    private BNO055IMU mIMU = null;
    private boolean mIMUInitialized = false;
    private int mLinearMillisecondsPerInch = 10;

    private ArrayList<IDriveSessionStatusListener> mDriveSessionStatusListeners = new ArrayList<>();
    private ArrayList<IRotationStatusListener> mRotationStatusListeners = new ArrayList<>();


    public Drivetrain(OpMode opMode,int linearMillisecondsPerInch,double rotationKp,double linearKp){
        this.mOpMode = opMode;
        mLinearMillisecondsPerInch = linearMillisecondsPerInch;
        mRotationKp = rotationKp;
        mLinearKp = linearKp;

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
     *
     * IMPORTANT:  IMU can take a long time to initialize
     */
    public void init(HardwareMap hwMap) throws Exception {
        this.mHWMap = hwMap;
        try{
            // Initialize the IMU
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

            parameters.mode                = BNO055IMU.SensorMode.IMU;
            parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.loggingEnabled      = false;

            // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
            // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
            // and named "imu".
            mIMU = mHWMap.get(BNO055IMU.class, "imu");
            mIMU.initialize(parameters);


            mIMUInitialized = true;
        }
        catch(Exception e){
            throw new Exception(e);
        }
        mIMUInitialized = true;
    }

    public boolean isIMUInitialized(){
        return mIMUInitialized;
    }

    /**
     * Must be called from the OpMode service loop to service timers for drive and rotation handling.
     */
    public void doLoop(){

        // Service rotation and heading corrections
        long currentTime = System.currentTimeMillis();
        long delta = currentTime- mLastIMUCheckTime;
        if (delta > 50) {
            mLastIMUCheckTime = currentTime;
            checkRotation();
            // if a linear drive is active then get a correction and call the correct power function in the
            // subclass if non-zero
            if (mLinearDriveActive){
                double correction = getHeadingCorrection();
                correctHeading(correction);
            }
        }
        // And service all the timers
        mDriveByEncoderFailTimer.checkTimer();
        serviceEncoderDrive();
        mTimedDriveTimer.checkTimer();
    }

    /**
     * starts a drive by encoder session.  Base class handles failure timer. Subclasses
     * override to add drivetrain specific behavior.  Subclasses should register for
     * IDriveSessionStatusListener to receive notification of session timeout failure.
     *
     * @param speed   speed to move
     * @param linearDistance distance in inches to move + is forward, - is backward
     * @param timeoutms timeout in ms to abort if move not completed.
     *
     * @return true if session started, false on error.
     */
     public void driveEncoder(double speed, double linearDistance, int timeoutms) {
         stop();  // case we were moving
         // Reset the angle in the IMU logic
         resetAngle();

         mLinearDrivePower = speed;
         if (mLinearDrivePower > 1.0d){
             mLinearDrivePower = 1.0d;
         }

         if (linearDistance < 0.0d){
             mLinearDrivePower *= -1.0d;
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

        int timeoutms = (int) Math.abs(Math.round(linearDistance * mLinearMillisecondsPerInch * mLinearDrivePower));
        mTimedDriveTimer.setTimeout(timeoutms);
        mTimedDriveTimer.start();
        mLinearDriveActive = true;
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
     * Starts IMU rotation.
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * base class function does everything except set motor power.
     * @param ccwDegrees Degrees to turn, - is right + is left
     */
    public void rotate(int ccwDegrees) {
        stop();  // case we were moving

        // restart imu movement tracking.
        resetAngle();
        mRotationTargetAngle = ccwDegrees;
        mRotationActive = true;
        // Call checkRotation to check imu
        checkRotation();
    }
    public boolean isRotationActive(){
        return mRotationActive;
    }
    /**
     * Resets the cumulative angle tracking in the IMU to zero
     */
    protected void resetAngle()
    {
        if (!mIMUInitialized)
            return;
        mLastIMUOrientation = mIMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        mHeadingAngle = 0;
    }
    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    protected double getAngle() {
        if (!mIMUInitialized)
            return 0d;
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = mIMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

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
        if (!mRotationActive) {
            return 0.0;
        }
         boolean continueTurning = false;

        double angle = getAngle();
        // On a right turn we have to get off zero first so return
        // to keep turning.
        if (angle == 0d) {
            continueTurning = true;
        }
        // if on a left turn, then we are waiting for the angle to go
        // positive up to the rotation target angle
        if (angle < mRotationTargetAngle) {
            continueTurning =true;
        }
        // if on a right turn, then we are waiting for the angle to become
        // more negative than the target angle
        if (angle < mRotationTargetAngle) {
            continueTurning =true;
        }
        if (!continueTurning){
            // Otherwise we are done so stop the rotation, reset the angle, and notify listeners
            // that the rotation is complete
            stop();
            mRotationActive = false;
            resetAngle();
            for(Iterator<IRotationStatusListener>iter=mRotationStatusListeners.iterator();iter.hasNext();){
                IRotationStatusListener listener = iter.next();
                listener.rotationComplete();
            }
           return 0.0d;
        }
        // Otherwise, update the power to the motors
        double error = mRotationTargetAngle - angle;
        // Normalize error to +/- 1.0 is +/- 180 degrees;
        error = error / 180d;

        // And multiply by the gain
        error = error * mRotationKp;
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

        correction = correction * mLinearKp;

        if (Math.abs(correction) > 1.0d){
            correction = Math.signum(correction);
        }

        return correction;
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
     * base class function must be called by subclasses
     */
    public void stop(){
        mRotationActive = false;
        mLinearDriveActive = false;
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
     * @return true if the drivetrain is moving.  false if stopped.
     */
    public abstract boolean isMoving();


    /**
         *  Utility function to handle motor initialization.  init must have been called
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
