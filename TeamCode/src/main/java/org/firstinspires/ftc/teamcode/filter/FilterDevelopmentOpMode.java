package org.firstinspires.ftc.teamcode.filter;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousController;
import org.firstinspires.ftc.teamcode.grabberbot.MecanumGrabberBot;
import org.firstinspires.ftc.teamcode.speedbot.BaseSpeedBot;
import org.firstinspires.ftc.teamcode.speedbot.CraneSpeedBot;


public class FilterDevelopmentOpMode  extends  OpMode{



    private BaseSpeedBot mSpeedBot = null;

    private OpMode mOpmode = null;

    private int mSequence = 0;

    /**
     *
     * @param opMode
     * @param sequence
     */
    public FilterDevelopmentOpMode(OpMode opMode, int sequence){
        mOpmode = opMode;
        // lengthen init timeout to give time to initialize the IMU
        mOpmode.msStuckDetectInit = 40000;
        mSequence = sequence;
     }

    public void init() {
        String initErrs = "";
        try {
            mSpeedBot = new BaseSpeedBot(mOpmode, true);
            mSpeedBot.init();
        }
        catch(Exception e){
            initErrs += ","+e.getMessage();
        }

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
//        autoController.loop();

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
