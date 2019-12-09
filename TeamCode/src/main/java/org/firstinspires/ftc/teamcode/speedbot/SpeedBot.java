package org.firstinspires.ftc.teamcode.speedbot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.drivetrain.BaseMecanumDrive;
import org.firstinspires.ftc.teamcode.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.drivetrain.SpeedBotMecanumDrive;

/**
 * This is the Speed Bot
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
public class SpeedBot  {

    protected OpMode mOpMode;

    private SpeedBotMecanumDrive mDrivetrain = null;
    private boolean mEnableIMU = false;

    private Crane mCrane = null;

    private  FrontHooks mFrontHooks = null;

    public SpeedBot(OpMode opMode,boolean enableIMU){
        this.mOpMode = opMode;
        this.mEnableIMU = enableIMU;
        mDrivetrain = new SpeedBotMecanumDrive(opMode);
        mCrane = new Crane(opMode);

        mFrontHooks = new FrontHooks(opMode);
    }

    public BaseMecanumDrive getDrivetrain(){
        return mDrivetrain;
    }

    public Crane getCrane(){
        return  mCrane;
    }

    public FrontHooks getFrontHooks(){
        return mFrontHooks;
    }
    /**
     *
     * @throws Exception
     */
    public void init() throws Exception {
        String initErrString = "";
        try {
            mDrivetrain.init(mOpMode.hardwareMap,mEnableIMU);

        }
        catch (Exception e){
            initErrString += e.getMessage();
        }
        try{
            mCrane.init(mOpMode.hardwareMap);
        }
        catch (Exception e){
            initErrString += e.getMessage();
        }
        if (initErrString.length() > 0){
            throw new Exception(initErrString);
        }
    }

}
