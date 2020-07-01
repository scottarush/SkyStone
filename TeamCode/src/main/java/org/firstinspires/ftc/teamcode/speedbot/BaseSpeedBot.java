package org.firstinspires.ftc.teamcode.speedbot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.drivetrain.BaseMecanumDrive;
import org.firstinspires.ftc.teamcode.drivetrain.IMU;
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

    private IMU mIMU = null;

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
        if (enableIMU){
            mIMU = new IMU();
        }
        mDrivetrain = new SpeedBotMecanumDrive(opMode,mIMU);

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
        if (mIMU != null){
            try {
                mIMU.initIMU(mOpMode.hardwareMap);
            }
            catch(Exception e){
                initErrString += e.getMessage();
            }
        }
        try {
            mDrivetrain.init(mOpMode.hardwareMap);

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

    /**
     * gets a reference to the IMU
     */
    public IMU getIMU(){
        return mIMU;
    }
}
