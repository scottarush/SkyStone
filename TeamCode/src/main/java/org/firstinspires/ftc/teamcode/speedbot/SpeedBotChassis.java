package org.firstinspires.ftc.teamcode.speedbot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.drivetrain.BaseMecanumDrive;
import org.firstinspires.ftc.teamcode.drivetrain.SpeedBotMecanumDrive;

/**
 * This is the Speed Bot Chassis version used for development of the autonomous filter
 * -------------------------------------------------
 * HUB Layout:
 * -------------------------------------------------
 * Port0:  left front wheel hex motor "lf"
 * Port1:  right front wheel hex motor "rf"
 * Port2:  left rear wheel hex motor "lr"
 * Port3:  right rear wheel hex motor "rr"
 *
 */
public class SpeedBotChassis {

    protected OpMode mOpMode;

    private SpeedBotMecanumDrive mDrivetrain = null;

    private  FrontHooks mFrontHooks = null;

    public SpeedBotChassis(OpMode opMode, boolean enableIMU){
        this.mOpMode = opMode;
         mDrivetrain = new SpeedBotMecanumDrive(opMode);

        mFrontHooks = new FrontHooks(opMode);
    }

    public BaseMecanumDrive getDrivetrain(){
        return mDrivetrain;
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
            mDrivetrain.init(mOpMode.hardwareMap,false);

        }
        catch (Exception e){
            initErrString += e.getMessage();
        }
         try{
            mFrontHooks.init(mOpMode.hardwareMap);
        }
        catch (Exception e){
            initErrString += e.getMessage();
        }

        if (initErrString.length() > 0){
            throw new Exception(initErrString);
        }
    }

}
