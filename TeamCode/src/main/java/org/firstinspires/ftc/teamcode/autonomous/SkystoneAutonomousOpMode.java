package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.speedbot.SpeedBot;
import org.firstinspires.ftc.teamcode.grabberbot.MecanumGrabberBot;

//@Autonomous(name="AutonomousMode", group="Robot")
//@Disabled
public class SkystoneAutonomousOpMode {

    private MecanumGrabberBot mGrabberBot = null;
    private SpeedBot mSpeedBot = null;

    private static final boolean USE_GRABBER_BOT = false;

    private AutonomousController autoController = null;


    private OpMode mOpmode = null;

    private boolean mBlueAlliance = false;
    /**
     * Reference to VuforiaTensorFlowObjectDetector utilities
     */
    VuforiaTargetLocator mVuforia = null;

    public SkystoneAutonomousOpMode(OpMode opMode, boolean isBlueTeam){
        mOpmode = opMode;
        // lengthen init timeout to give time to initialize the IMU
        mOpmode.msStuckDetectInit = 40000;
     }

    public void init() {
        String initErrs = "";
        try {
            if (USE_GRABBER_BOT){
                mGrabberBot = new MecanumGrabberBot(mOpmode, true);
                mGrabberBot.init();
            }
            else{
                mSpeedBot = new SpeedBot(mOpmode, true);
                mSpeedBot.init();
            }
        }
        catch(Exception e){
            initErrs += ","+e.getMessage();
        }
        // Initialize Vuforia
        mVuforia = new VuforiaTargetLocator();
        try{
            mVuforia.init(mOpmode);
        }
        catch(Exception e){
            initErrs += ", Vuforia init error";
        }
        // Activate vuforia
        mVuforia.activate();
        // Initialize the controller based on the bot
        if (USE_GRABBER_BOT){
            autoController = new AutonomousController(mOpmode,mGrabberBot,mVuforia, mBlueAlliance);
        }
        else{
            autoController = new AutonomousController(mOpmode,mSpeedBot,mVuforia, mBlueAlliance);
        }

        if (initErrs.length() == 0){
            mOpmode.telemetry.addData("Status:","Robot init complete");
            mOpmode.telemetry.update();
        }
        else{
            mOpmode.telemetry.addData("Init errors:",initErrs);
            mOpmode.telemetry.update();
        }


    }

    public void loop() {
         if (!autoController.isAutonomousComplete()){
            autoController.loop();
        }

    }

/**
    public static void main(String[] args) {
        SkystoneAutonomousOpMode mode = new SkystoneAutonomousOpMode();
        mOpmode.mode.gamepad1 = new Gamepad();
        mOpmode.mode.telemetry = new TelemetryImpl(mode);
        mode.init_loop();
    }
**/

}
