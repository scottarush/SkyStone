package org.firstinspires.ftc.teamcode.speedbot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.grabberbot.FourBarArm;
import org.firstinspires.ftc.teamcode.grabberbot.Grabber;
import org.firstinspires.ftc.teamcode.grabberbot.Hook;

/**
 * This is the Mecanum Frame Development Bot
 * -------------------------------------------------
 * HUB Layout:
 * -------------------------------------------------
 * Port0:  left front wheel hex motor "lf"
 * Port1:  right front wheel hex motor "rf"
 * Port2:  left rear wheel hex motor "lr"
 * Port3:  right rear wheel hex motor "rr"
 *
 * Camera is "webcam"
 */
public class SpeedBot extends Robot {

    private final FourBarArm mDummyArm = new FourBarArm(null,false);
    private final Grabber mDummyGrabber = new Grabber(null);
    private final Hook mDummyHook = new Hook(null);

    public SpeedBot(OpMode opMode,boolean enableIMU){
        super(DriveTrainStyle.SPEED_BOT_MECANUM_DRIVE,opMode,enableIMU);
    }


    /**
     * Override base class function to initialize the rest of the
     * bot.
     * @throws Exception
     */
    public void init() throws Exception {
        String initErrString = "";
        try {
            super.init();
        }
        catch (Exception e){
            initErrString += e.getMessage();
        }
        if (initErrString.length() > 0){
            throw new Exception(initErrString);
        }
    }

    /**
     * Dummy implementation to allow this class to be used in same OpMode and AutonomousController
     * classes as other bots.
     * @return
     */
    @Override
    public FourBarArm getArm() {
        return mDummyArm;
    }

    /**
     * Dummy implementation to allow this class to be used in same OpMode and AutonomousController
     * classes as other bots.
     * @return
     */
    @Override
    public Grabber getGrabber() {
        return mDummyGrabber;
    }

/**
 * Dummy implementation to allow this class to be used in same OpMode and AutonomousController
 * classes as other bots.
 **/
    @Override
    public Hook getHook() {
        return mDummyHook;
    }
}
