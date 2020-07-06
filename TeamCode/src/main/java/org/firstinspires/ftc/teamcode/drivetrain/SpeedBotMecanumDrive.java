package org.firstinspires.ftc.teamcode.drivetrain;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * This is the speed bot with the 1-4 gear.
 */
public class SpeedBotMecanumDrive extends BaseMecanumDrive {

    /**
     * Approximate number of milliseconds per inch forward and rearward at full power.
     */
    public static final int LINEAR_MILLISECONDS_PER_INCH = 50;

    @Override
    public int getLinearMillisecondsPerInch() {
        return LINEAR_MILLISECONDS_PER_INCH;
    }

    /*
    * With core hex motors there are 72 counts/axle rotation x 1:4 gear output ratio
     */
    @Override
    protected int getEncoderCountsPerRev() {
        return 288;
    }

    @Override
    public double getRotationKp() {
        return 1.8d;
    }

    @Override
    protected double getMinRotationCompleteAngle() {
        return 5;
    }

    @Override
    public double getLinearKp() {
        return 1.0d;
    }

    /**
     * Core hex motor from the specification
     */
    public static final int ENCODER_COUNTS_PER_MOTOR_SHAFT_ROTATION = 288;
    /**
     * Number of counts per inch of direct wheel movement.
     **/
    public static final double COUNTS_PER_INCH = ENCODER_COUNTS_PER_MOTOR_SHAFT_ROTATION /
            (4 * MECANUM_WHEEL_CIRCUMFERENCE);


    public SpeedBotMecanumDrive(OpMode opMode, GuidanceController imu){
        super(opMode,imu);
    }

    @Override
    protected double getEncoderCountsPerInchRotation() {
        return COUNTS_PER_INCH;
    }

    public static final double ENCODER_COUNTS_MIN_THRESHOLD_WHEEL_ROTATION_FRACTION = 0.15d;
    @Override
    protected int getEncoderDriveCountsMinThreshold() {
        // Return a fraction of a wheel rotation in counts
        return (int)Math.round(COUNTS_PER_INCH *
                ENCODER_COUNTS_MIN_THRESHOLD_WHEEL_ROTATION_FRACTION *
                MECANUM_WHEEL_CIRCUMFERENCE);
    }

    /* Initialize standard Hardware interfaces.
     * NOTE:  This class throws Exception on any hardware initIMU error so be sure to catch and
     * report to Telemetry in your initialization. */
    public void init(HardwareMap ahwMap) throws Exception {
        // Save reference to Hardware map
        mHWMap = ahwMap;

        // Define and Initialize Motors
        String motorInitError = "";
        DcMotor motor = null;
        try {
            motor = tryMapMotor("lf");
//            lfMotor = new WrappedDCMotor(motor,ENCODER_COUNTS_PER_MOTOR_SHAFT_ROTATION, PIMOTOR_KP, PIMOTOR_KI);
            mLFMotor = motor;
            mLFMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        catch (Exception e){
            motorInitError += "lf,";
        }
        try {
            motor = tryMapMotor("rf");
//            rfMotor = new WrappedDCMotor(motor,ENCODER_COUNTS_PER_MOTOR_SHAFT_ROTATION, PIMOTOR_KP, PIMOTOR_KI);
            mRFMotor = motor;
        }
        catch(Exception e){
            motorInitError += "rf,";
        }
        try {
            motor = tryMapMotor("lr");
//            lrMotor = new WrappedDCMotor(motor,ENCODER_COUNTS_PER_MOTOR_SHAFT_ROTATION, PIMOTOR_KP, PIMOTOR_KI);
            mLRMotor = motor;
            mLRMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        catch(Exception e){
            motorInitError += "lr,";
        }
        try {
            motor = tryMapMotor("rr");
//            rrMotor = new WrappedDCMotor(motor,ENCODER_COUNTS_PER_MOTOR_SHAFT_ROTATION, PIMOTOR_KP, PIMOTOR_KI);
            mRRMotor = motor;
        }
        catch(Exception e){
            motorInitError += "rr,";
        }

        // Set all motors to zero power
        setPower(0, 0, 0, 0);

        // Set all motors to run without encoders for maximum power
        setMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (motorInitError.length() > 0){
            throw new Exception("Motor initIMU errs: '"+motorInitError+"'");
        }

    }


}
