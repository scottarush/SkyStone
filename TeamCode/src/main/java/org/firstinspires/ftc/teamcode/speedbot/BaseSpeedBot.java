package org.firstinspires.ftc.teamcode.speedbot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.drivetrain.BaseMecanumDrive;
import org.firstinspires.ftc.teamcode.drivetrain.GuidanceController;
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
public class BaseSpeedBot {

    protected OpMode mOpMode;

    protected SpeedBotMecanumDrive mDrivetrain = null;

    private  FrontHooks mFrontHooks = null;

    private GuidanceController mGuidanceController = null;

    private boolean mEnableIMU = false;
    /**
     * distance from IMU to far wheel axes = {@value} in mm
     */
    public static double LX_MM = 100.0d;

    /**
     * distance from IMU to wheel = {@value} in mm
     */
    public static double LY_MM = 25.0d;

    /**
     * Wheel radius in mm
     *
     */
    public static double WHEEL_RADIUS_MM = 98.0d;

    public BaseSpeedBot(OpMode opMode, boolean enableIMU){
        this.mOpMode = opMode;
        mEnableIMU = enableIMU;
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

        if (mEnableIMU){
            try {
                mGuidanceController = new GuidanceController();
                GuidanceController.Parameters gcParams = new GuidanceController.Parameters();
                mGuidanceController.init(mOpMode.hardwareMap, gcParams);
            }
            catch(Exception e){
                initErrString += e.getMessage();
            }
        }
        try {
            mDrivetrain = new SpeedBotMecanumDrive(mOpMode, mGuidanceController);
            mDrivetrain.init(mOpMode.hardwareMap);

        }
        catch (Exception e){
            initErrString += e.getMessage();
        }
         try{
             mFrontHooks = new FrontHooks(mOpMode);
             mFrontHooks.init(mOpMode.hardwareMap);
        }
        catch (Exception e){
            initErrString += e.getMessage();
        }

        if (initErrString.length() > 0){
            throw new Exception(initErrString);
        }

    }

    /**
     * gets a reference to the GuidanceController
     */
    public GuidanceController getGuidanceController(){
        return mGuidanceController;
    }
}
