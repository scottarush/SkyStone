package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.arm.Arm;

public class Grabber {
    public static final double BUMPER_REVERSE_SPEED = 0.5d;

    private DcMotor mLeftMotor = null;
    private DcMotor mRightMotor = null;

    public static final String LEFT_GRABBER_MOTOR_NAME = "lgrabber";
    public static final String RIGHT_GRABBER_MOTOR_NAME = "rgrabber";

    protected OpMode opMode;

    protected HardwareMap hwMap = null;

    public Grabber(OpMode opMode) {
    }

    public void init(HardwareMap ahwMap) throws Exception {
        hwMap = ahwMap;
        String initErrString = "";
        try {
            mLeftMotor = tryMapMotor(LEFT_GRABBER_MOTOR_NAME);
            mLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception e) {
            initErrString += "left grabber,";
         }
        try {
            mRightMotor = tryMapMotor(RIGHT_GRABBER_MOTOR_NAME);
            mRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            // Right side is reversed to suck in on postive power
            mRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        } catch (Exception e) {
            initErrString += "right grabber,";
        }
        // If we have an error then throw an exception at the end so that
        // owner can catch and post a message
        if (initErrString.length() > 0)
            throw new Exception("Grabber init errs: '"+initErrString+"'");

    }

    public void stop () {
        if (mLeftMotor != null)
            mLeftMotor.setPower(0);
        if (mRightMotor != null)
            mRightMotor.setPower(0);
    }

    private void setPower ( double leftPower, double rightPower){
        if (mLeftMotor != null)
            mLeftMotor.setPower(leftPower);
        if (mRightMotor != null)
            mRightMotor.setPower(rightPower);
    }

    /**
     * This is the function called from the opmode run loop to actually move the grabber
     * @param leftBumper
     * @param rightBumper
     * @param leftPower
     * @param rightPower
     */
    public void moveGrabber ( boolean leftBumper, boolean rightBumper, double leftPower,
                              double rightPower){
        if (leftBumper || rightBumper) {
            if (leftBumper) {
                leftPower = -BUMPER_REVERSE_SPEED;
                if (rightBumper)
                    rightPower = -BUMPER_REVERSE_SPEED;
                else
                    rightPower = 0;
            }

            if (rightBumper) {
                rightPower = -BUMPER_REVERSE_SPEED;
                if (leftBumper)
                    leftPower = -BUMPER_REVERSE_SPEED;
                else
                    leftPower = 0;
            }
        }
        setPower(leftPower, rightPower);

    }

    /**
     *  Utility function to handle motor initialization.  init must have been called
     *  with a non-null hwMap or exception will be thrown.
     *
     */
    private DcMotor tryMapMotor (String motorName) throws Exception {
        DcMotor motor = null;
        try {
            motor = hwMap.get(DcMotor.class, motorName);
        } catch (Exception e) {
            // Throw an exception for the caller to catch so we can debug.
            throw new Exception("Cannot map grabber motor: " + motorName);
        }
        return motor;
    }

}