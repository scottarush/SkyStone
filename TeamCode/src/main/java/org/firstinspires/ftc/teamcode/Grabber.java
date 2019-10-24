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

    public static final String LEFT_GRABBER_MOTOR_NAME = "lgrabmotor";
    public static final String RIGHT_GRABBER_MOTOR_NAME = "rgrabmotor";

    protected OpMode opMode;

    protected HardwareMap hwMap = null;

    public Grabber(OpMode opMode){
    }

    public void init(HardwareMap ahwMap) throws Exception {
        hwMap = ahwMap;
        try {
            if (hwMap == null){
                throw new Exception("try called with null hwMap. Must call init first.");
            }
            mLeftMotor = hwMap.get(DcMotor.class, LEFT_GRABBER_MOTOR_NAME);
            mRightMotor = hwMap.get(DcMotor.class, RIGHT_GRABBER_MOTOR_NAME);
        }
        catch(Exception e){
            //e.printStackTrace();
            opMode.telemetry.addData("Grabber Motor Init Failed: ",e.getMessage());
            opMode.telemetry.update();
            return;
        }
        // now set the motor mode
        mLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Right side is reversed to suck in on postive power
        mRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void stop() {
         mLeftMotor.setPower(0);
         mRightMotor.setPower(0);
    }

    private void setPower(double leftPower, double rightPower){
        mLeftMotor.setPower(leftPower);
        mRightMotor.setPower(rightPower);
    }

    /**
     * This is the function called from the opmode run loop to actually move the grabber
     * @param leftBumper
     * @param rightBumper
     * @param leftPower
     * @param rightPower
     */
    public void moveGrabber(boolean leftBumper,boolean rightBumper,double leftPower,double rightPower){
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
        setPower(leftPower,rightPower);

    }
}
