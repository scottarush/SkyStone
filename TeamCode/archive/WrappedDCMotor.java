package org.firstinspires.ftc.teamcode.drivetrain;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class WrappedDCMotor implements DcMotor {

    DcMotor mMotor = null;

    private double mKp = 0.0d;
    private double mKi = 0.0d;

    private int mTargetPosition = 0;

    private int mIntegralTermErrorThreshold = 0;

    private static final int NUM_ROTATIONS_INTEGRAL_TERM = 5;

    private double mIntegralError = 0d;

    private double mEncoderPowerLimit = 0d;

    public static final int STOP_ERROR_THRESHOLD = 5;

    private long mLastSystemTime = 0;
    private boolean mPILoopRunning = false;

    private RunMode mRunMode = RunMode.RUN_USING_ENCODER;

    private static final boolean ENABLE_PIMOTOR_OVERRIDES = false;
    /**
     * Constructor must be provided a valid mapped motor
     * @param theMotor
     */
    public WrappedDCMotor(DcMotor theMotor, int encoderCountsPerRev, double kp, double ki){
        mMotor = theMotor;
        mIntegralTermErrorThreshold = encoderCountsPerRev * NUM_ROTATIONS_INTEGRAL_TERM;
    }

    /**
     * This function must be called periodically from the loop in order to service the
     * encoder control pi loop.
     */
    public void loop(){
        if (!ENABLE_PIMOTOR_OVERRIDES)
            return;
        if (!mPILoopRunning)
            return;
        long currentTime = System.currentTimeMillis();
        long deltams = currentTime-mLastSystemTime;

        mLastSystemTime = currentTime;

        int error = Math.abs(mTargetPosition) - Math.abs(mMotor.getCurrentPosition());

        if (error < 0){
            // We went past it so stop
            mMotor.setPower(0d);
            mPILoopRunning = false;
        }
        else if (Math.abs(error) < STOP_ERROR_THRESHOLD){
            mMotor.setPower(0d);
            mPILoopRunning = false;
        }
        else if (error < mIntegralTermErrorThreshold){
            // And compute new power but get correct sign on error for reverse motor
            if (mMotor.getDirection() == Direction.REVERSE) {
                error = -error;
            }

            // Error is less than threshold.  Begin computation of integral term
            mIntegralError += error * (double)deltams/1000d;

            double power = (mKp * error * + mKi * mIntegralError);
            // Threshold
            if (Math.abs(power) > mEncoderPowerLimit){
                power = Math.signum(power) * mEncoderPowerLimit;
            }
            mMotor.setPower(power);
        }
    }

    @Override
    public void setDirection(Direction direction) {
        mMotor.setDirection(direction);
    }

    @Override
    public Direction getDirection() {
        return mMotor.getDirection();
    }

    @Override
    public void setPower(double power) {
        if (ENABLE_PIMOTOR_OVERRIDES) {
            if ((mPILoopRunning == false) && (mMotor.getMode() == RunMode.RUN_TO_POSITION)) {
                mPILoopRunning = true;
                mLastSystemTime = System.currentTimeMillis();
                mEncoderPowerLimit = power;
                loop();
            } else {
                mPILoopRunning = false;
                // Motor is moving so just set the power
                mMotor.setPower(power);
            }
        }
        else{
            mMotor.setPower(power);
        }
    }

    @Override
    public double getPower() {
        return mMotor.getPower();
    }

    @Override
    public Manufacturer getManufacturer() {
        return mMotor.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return mMotor.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return mMotor.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return mMotor.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        mMotor.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        mMotor.close();
    }

    @Override
    public void setTargetPosition(int position) {
        if (ENABLE_PIMOTOR_OVERRIDES)
            mTargetPosition = position;
        mMotor.setTargetPosition(position);
    }

    @Override
    public int getCurrentPosition() {
        return mMotor.getCurrentPosition();
    }

    @Override
    public MotorConfigurationType getMotorType() {
        return mMotor.getMotorType();
    }

    @Override
    public void setMotorType(MotorConfigurationType motorType) {
        mMotor.setMotorType(motorType);
    }

    @Override
    public DcMotorController getController() {
        return mMotor.getController();
    }

    @Override
    public int getPortNumber() {
        return mMotor.getPortNumber();
    }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        mMotor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {
        return mMotor.getZeroPowerBehavior();
    }

    @Override
    @Deprecated
    public void setPowerFloat() {

    }

    @Override
    @Deprecated
    public boolean getPowerFloat() {
        return false;
    }

    @Override
    public int getTargetPosition() {
        return mMotor.getTargetPosition();
    }

    @Override
    public boolean isBusy() {
        return mMotor.isBusy();
    }

    @Override
    public void setMode(RunMode mode) {
        if (ENABLE_PIMOTOR_OVERRIDES){
            mMotor.setMode(mode);
            return;
        }
        // Otherwise do override functionality
        mRunMode = mode;
        switch(mRunMode){
            case STOP_AND_RESET_ENCODER:
                mMotor.setPower(0d);
                mMotor.setMode(RunMode.STOP_AND_RESET_ENCODER);
                break;
            case RUN_TO_POSITION:   // Interecept RUN_TO_POSITION so we can do the power control
            case RUN_USING_ENCODER:
                mMotor.setMode(RunMode.RUN_USING_ENCODER);
                break;
        }
    }

    @Override
    public RunMode getMode() {
        if (ENABLE_PIMOTOR_OVERRIDES)
            return mRunMode;
        else
            return mMotor.getMode();
    }
}
