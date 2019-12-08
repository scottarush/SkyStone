package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.drivetrain.GrabberBotMecanumDrive;
import org.firstinspires.ftc.teamcode.drivetrain.SpeedBotMecanumDrive;
import org.firstinspires.ftc.teamcode.speedbot.SpeedBot;
import org.firstinspires.ftc.teamcode.grabberbot.MecanumGrabberBot;
import org.firstinspires.ftc.teamcode.Robot;

//@Autonomous(name="AutonomousMode", group="Robot")
//@Disabled
public class SkystoneAutonomousOpMode {

 //   private MecanumGrabberBot robot = new MecanumGrabberBot(this, Robot.DriveTrainStyle.MECANUM_HEX_BOT,true);
    private Robot robot = null;

    private static final boolean USE_GRABBER_BOT = true;

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
                robot = new MecanumGrabberBot(mOpmode, true);
            }
            else{
                robot = new SpeedBot(mOpmode, true);
            }
             robot.init();
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
        // Initialize the controller
        autoController = new AutonomousController(mOpmode,robot,mVuforia, mBlueAlliance);

        if (initErrs.length() == 0){
            mOpmode.telemetry.addData("Status:","Robot initIMU complete");
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
