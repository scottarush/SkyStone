package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.grabberbot.MecanumGrabberBot;
import org.firstinspires.ftc.teamcode.speedbot.CraneSpeedBot;


public class FilterDevelopmentOpMode {



    private MecanumGrabberBot mGrabberBot = null;
    private CraneSpeedBot mSpeedBot = null;

    private static final boolean USE_GRABBER_BOT = false;

    private AutonomousController autoController = null;


    private OpMode mOpmode = null;

    private boolean mBlueAlliance = false;

    private int mSequence = 0;

    /**
     *
     * @param opMode
     * @param isBlueTeam
     * @param sequence
     */
    public FilterDevelopmentOpMode(OpMode opMode, boolean isBlueTeam, int sequence){
        mOpmode = opMode;
        mBlueAlliance = isBlueTeam;
        // lengthen init timeout to give time to initialize the IMU
        mOpmode.msStuckDetectInit = 40000;
        mSequence = sequence;
     }

    public void init() {
        String initErrs = "";
        try {
            if (USE_GRABBER_BOT){
                mGrabberBot = new MecanumGrabberBot(mOpmode, true);
                mGrabberBot.init();
            }
            else{
                mSpeedBot = new CraneSpeedBot(mOpmode, true);
                mSpeedBot.init();
            }
        }
        catch(Exception e){
            initErrs += ","+e.getMessage();
        }
            autoController = new AutonomousController(mOpmode,mSpeedBot, mBlueAlliance,mSequence);


        if (initErrs.length() == 0){
            mOpmode.telemetry.addData("Status:","Robot init complete");
            mOpmode.telemetry.update();
        }
        else {
            mOpmode.telemetry.addData("Init errors:", initErrs);
            mOpmode.telemetry.update();
        }
    }

    public void loop() {
        autoController.loop();
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
