package org.firstinspires.ftc.teamcode.archive.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Encapsulates an option selected via the joystick.
 */
public class GamepadMenu {
    protected OpMode mOpMode = null;

    private String[] mOptionStrings = new String[1];
    private String mCaption = "";
    private int mSelectedOption = -1;

    public GamepadMenu(OpMode opMode, String caption, String[] options){
        mOpMode = opMode;
        mCaption = caption;
        mOptionStrings = options;
    }

    /**
     * Overriden by subclasses to add the specific menu items
     * @return true when the option has been selected.
     */
    public boolean doOptionMenu(){
        if (mOpMode.gamepad1.b){
            return true;
        }
        if (mOpMode.gamepad1.dpad_up){
            mSelectedOption++;
            if (mSelectedOption > mOptionStrings.length-1){
                mSelectedOption = 0;
            }
        }
        else if (mOpMode.gamepad1.dpad_down){
            mSelectedOption--;
            if (mSelectedOption < 0){
                mSelectedOption = mOptionStrings.length-1;
            }
        }
        // Show the currently option
        mOpMode.telemetry.addData(mCaption, mOptionStrings[mSelectedOption]);
        mOpMode.telemetry.update();
        return true;
    }

    /**
     * returns the option that was selected.
     */
    public int getSelectedOption(){
        return mSelectedOption;
    }
}
