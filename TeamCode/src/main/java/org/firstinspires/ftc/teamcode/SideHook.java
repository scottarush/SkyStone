package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This is the hook on the side of the robot that can be deployed to pull the foundation
 */
public class SideHook {
    private Servo mServo = null;
    private HardwareMap mHWMap = null;
    private OpMode mOpMode = null;

    private DigitalChannel mLimitSwitch = null;

    public static final String SIDE_HOOK_SERVO_NAME = "sidehook";

    private static final double DOWN_POSITION = 0.0d;
    private static final double UP_POSITION = 1.0d;

    public static final int DOWN = 0;
    public static final int UP = 1;
    private int mPosition = UP;

    public SideHook(OpMode opMode){
        opMode = mOpMode;
    }

    public void init(HardwareMap ahwMap) throws Exception {
        mHWMap = ahwMap;
        String initErrString = "";
        try {
            mServo = mHWMap.get(Servo.class, SIDE_HOOK_SERVO_NAME);
            // Force servo to closed position to start
            mServo.setPosition(SideHook.UP);
        } catch (Exception e) {
            initErrString += "SideHook initIMU failed,";
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
    /** Sets the hook to either DOWN or UP.
     * @return true on success, false on any error. **/
    public boolean setPosition(int position){
        if (mServo == null)
            return false;
        switch(position) {
            case DOWN:
                mServo.setPosition(DOWN_POSITION);
                mPosition = DOWN;
                return true;
            case UP:
                mServo.setPosition(UP_POSITION);
                mPosition = UP;
                return true;
        }
        return false;
     }

 }

