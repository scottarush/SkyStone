package org.firstinspires.ftc.teamcode.speedbot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This class encapsulates the two servos on the front of the robot.
 * left hook servo is "lhook"
 * right hook servo is "rhook"
 */
public class FrontHooks {
    private Servo mLeftServo = null;
    private Servo mRightServo = null;
    private HardwareMap mHWMap = null;
    private OpMode mOpMode = null;

    private DigitalChannel mLimitSwitch = null;

    public static final String LEFT_HOOK_SERVO_NAME = "lhook";
    public static final String RIGHT_HOOK_SERVO_NAME = "rhook";

    private static final double OPEN_POSITION = 1.0d;
    private static final double CLOSED_POSITION = 0.0d;


    public FrontHooks(OpMode opMode){
        opMode = mOpMode;
    }

    public void init(HardwareMap ahwMap) throws Exception {
        mHWMap = ahwMap;
        String initErrString = "";
        try {
            mLeftServo = mHWMap.get(Servo.class, LEFT_HOOK_SERVO_NAME);
            mLeftServo.setPosition(OPEN_POSITION);
        } catch (Exception e) {
            initErrString += "left hook servo err";
        }
        try {
            mRightServo = mHWMap.get(Servo.class, RIGHT_HOOK_SERVO_NAME);
            mRightServo.setPosition(OPEN_POSITION);
        } catch (Exception e) {
            initErrString += "right hook servo err";
        }

        if  (initErrString.length() > 0){
            throw new Exception(initErrString);
        }
     }

    public void openHooks(){
        if (mLeftServo != null){
            if (mRightServo != null){
                mLeftServo.setPosition(OPEN_POSITION);
                mRightServo.setPosition(OPEN_POSITION);
            }
        }
    }
    public void closeHooks(){
        if (mLeftServo != null){
            if (mRightServo != null){
                mLeftServo.setPosition(CLOSED_POSITION);
                mRightServo.setPosition(CLOSED_POSITION);
            }
        }
    }
 }

