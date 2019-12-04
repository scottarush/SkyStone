package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This is the hook on the front of the robot that can be used to pull as well
 * as push bocks into the grabber.
 */
public class Hook {
    private Servo mServo = null;
    private HardwareMap mHWMap = null;
    private OpMode mOpMode = null;

    private DigitalChannel mLimitSwitch = null;

    public static final String HOOK_SERVER_NAME = "hookservo";

    private static final double OPEN_POSITION = 0.0d;
    private static final double CLOSED_POSITION = 1.0d;

    public static final int OPEN = 0;
    public static final int CLOSED = 1;
    private int mPosition = CLOSED;

    public Hook(OpMode opMode){
        opMode = mOpMode;
    }

    public void init(HardwareMap ahwMap) throws Exception {
        mHWMap = ahwMap;
        String initErrString = "";
        try {
            mServo = mHWMap.get(Servo.class, HOOK_SERVER_NAME);
            // Force servo to closed position to start
            mServo.setPosition(Hook.CLOSED);
        } catch (Exception e) {
            initErrString += "Hook Servo initIMU failed,";
        }

        if  (initErrString.length() > 0){
            throw new Exception(initErrString);
        }
     }

    /**
     *
     */
    public int getPosition(){
         return mPosition;
    }
    /** Sets the hook to either OPEN, CLOSED, or RETRACTED.
     * @return true on success, false on any error. **/
    public boolean setPosition(int position){
        if (mServo == null)
            return false;
        switch(position) {
            case OPEN:
                mServo.setPosition(OPEN_POSITION);
                mPosition = OPEN;
                return true;
            case CLOSED:
                mServo.setPosition(CLOSED_POSITION);
                mPosition = CLOSED;
                return true;
        }
        return false;
     }

 }

