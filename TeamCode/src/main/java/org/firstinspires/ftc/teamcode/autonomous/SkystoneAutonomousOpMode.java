package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.FrameDevelopmentBot;
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.MecanumGrabberBot;
import org.firstinspires.ftc.teamcode.Robot;

//@Autonomous(name="AutonomousMode", group="Robot")
//@Disabled
public class SkystoneAutonomousOpMode {

 //   private MecanumGrabberBot robot = new MecanumGrabberBot(this, Robot.DriveTrainStyle.MECANUM_HEX_BOT,true);
    private Robot robot = null;


    private AutonomousController autoController = null;


    private OpMode mOpmode = null;

    private boolean mBlueAlliance = false;
    /**
     * Reference to VuforiaTensorFlowObjectDetector utilities
     */
    VuforiaTargetLocator mVuforia = null;

    public SkystoneAutonomousOpMode(OpMode opMode, boolean isBlueTeam){
        mOpmode = opMode;
        mBlueAlliance = isBlueTeam;
     }

    public void init() {
        String initErrs = "";
        try {
            if (Globals.USE_DEV_FRAME_BOT){
                robot = new FrameDevelopmentBot(mOpmode);
            }
            else {
                robot = new MecanumGrabberBot(mOpmode,true);
            }
            robot.init();
        }
        catch(Exception e){
            initErrs += e.getMessage();
        }
        // Initialize Vuforia
        mVuforia = new VuforiaTargetLocator();
        try{
            mVuforia.init(mOpmode);
        }
        catch(Exception e){
            initErrs += e.getMessage();
        }
        // Activate vuforia
        mVuforia.activate();
        // Initialize the controller
        autoController = new AutonomousController(mOpmode,robot,mVuforia, mBlueAlliance,AutonomousController.OPEN_LOOP_TIME);

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
            autoController.doOpmode();
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
