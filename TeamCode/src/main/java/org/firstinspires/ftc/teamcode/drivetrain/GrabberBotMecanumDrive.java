package org.firstinspires.ftc.teamcode.drivetrain;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class GrabberBotMecanumDrive extends BaseMecanumDrive {

    /**
     * Core hex motor from the specification
     */
    public static final int ENCODER_COUNTS_PER_ROTATION = 288;
    /**
     * Number of counts per inch of direct wheel movement.
     **/
    public static final double COUNTS_PER_INCH = ENCODER_COUNTS_PER_ROTATION / MECANUM_WHEEL_CIRCUMFERENCE;
    public static final double PIMOTOR_KP = 1.0;
    public static final double PIMOTOR_KI = 1.0;
    public static final double ROTATION_KP = 4.0d;
    public static final double ROTATION_KI  = 1.0d;
    /**
     * Approximate number of milliseconds per inch forward and rearward at full power.
     */
    public static final int LINEAR_MILLISECONDS_PER_INCH = 50;

    @Override
    public int getLinearMillisecondsPerInch() {
        return LINEAR_MILLISECONDS_PER_INCH;
    }

    @Override
    public double getRotationKp() {
        return ROTATION_KP;
    }

    @Override
    public double getRotationKi() {
        return ROTATION_KI;
    }

    @Override
    public double getLinearKp() {
        return 1.0d;
    }

    @Override
    protected double getEncoderCountsPerInchRotation() {
        return COUNTS_PER_INCH;
    }

    public GrabberBotMecanumDrive(OpMode opMode){
        super(opMode);
    }

    /* Initialize standard Hardware interfaces.
     * NOTE:  This class throws Exception on any hardware initIMU error so be sure to catch and
     * report to Telemetry in your initialization. */
    public void initIMU(HardwareMap ahwMap) throws Exception {
        // Save reference to Hardware map
        mHWMap = ahwMap;

        // Define and Initialize Motors
        String motorInitError = "";
        DcMotor motor = null;
        try {
            motor = tryMapMotor("lf");
            lfMotor = new WrappedDCMotor(motor,ENCODER_COUNTS_PER_ROTATION, PIMOTOR_KP, PIMOTOR_KI);
        }
        catch (Exception e){
            motorInitError += "lf,";
        }
        try {
            motor = tryMapMotor("rf");
            rfMotor = new WrappedDCMotor(motor,ENCODER_COUNTS_PER_ROTATION, PIMOTOR_KP, PIMOTOR_KI);
        }
        catch(Exception e){
            motorInitError += "rf,";
        }
        try {
            motor = tryMapMotor("lr");
            lrMotor = new WrappedDCMotor(motor,ENCODER_COUNTS_PER_ROTATION, PIMOTOR_KP, PIMOTOR_KI);
            lrMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        catch(Exception e){
            motorInitError += "lr,";
        }
        try {
            motor = tryMapMotor("rr");
            rrMotor = new WrappedDCMotor(motor,ENCODER_COUNTS_PER_ROTATION, PIMOTOR_KP, PIMOTOR_KI);
            rrMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        catch(Exception e){
            motorInitError += "rr,";
        }
        // Initialize the IMU with the superclass initialization
        try{
            super.initIMU(ahwMap);
        }
        catch(Exception e){
            // This exception can't actually happen but change in the future to catch this somehow
            motorInitError += "IMU initIMU";
        }

        // Set all motors to zero power
        setPower(0, 0, 0, 0);

        // Set all motors to run with encoders.
        setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);

        if (motorInitError.length() > 0){
            throw new Exception("Motor initIMU errs: '"+motorInitError+"'");
        }

    }


}
