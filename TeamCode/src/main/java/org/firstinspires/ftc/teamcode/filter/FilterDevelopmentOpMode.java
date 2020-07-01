package org.firstinspires.ftc.teamcode.filter;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drivetrain.BaseMecanumDrive;
import org.firstinspires.ftc.teamcode.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.drivetrain.IMU;
import org.firstinspires.ftc.teamcode.speedbot.BaseSpeedBot;


public class FilterDevelopmentOpMode  extends  OpMode{

    public static final double INIT_PX = 0D;
    public static final double INIT_PY = 0D;
    public static final double INIT_HEADING = Math.PI/2;
    public static final double T = 0.050d;
    private static final int T_MS = Math.round((float)(T * 1000d));

    private BaseSpeedBot mSpeedBot = null;

    private OpMode mOpmode = null;

    private int mSequence = 0;

    private IKalmanTracker mKalmanTracker = null;

    private long mLastSystemTime = 0;
    private boolean mFirstTime = true;
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
        // Initialize the KalmanTracker
        mKalmanTracker = new KalmanTracker();
        mKalmanTracker.init(T,INIT_PX,INIT_PY,INIT_HEADING,BaseSpeedBot.LX_MM,BaseSpeedBot.LY_MM,BaseSpeedBot.WHEEL_RADIUS_MM);
    }

    public void loop() {
        if (mFirstTime){
            mFirstTime = false;
            mLastSystemTime = System.currentTimeMillis();
            updateTracker();
            return;
        }
        // Compute the delta time and update the Tracker if we are at the sampel period T
        long systemTime = System.currentTimeMillis();
        int deltat_ms = (int)(systemTime-mLastSystemTime);
        if (deltat_ms > T_MS){
            updateTracker();
            mLastSystemTime = systemTime;   // save for next loop
        }
        // TODO: Send the estimated position and heading to the state machine controller
        // TODO: update the robot speed
    }

    /**
     * helper function to update the Tracker
     */
    private void updateTracker(){
        // Get the wheel speeds
        double[] wheelSpeeds = mSpeedBot.getDrivetrain().getWheelSpeeds();

        // Now get the IMU data
        IMU imu = mSpeedBot.getIMU();
        Acceleration acceleration = imu.getBNO055IMU().getLinearAcceleration();
        Orientation orientation = imu.getBNO055IMU().getAngularOrientation();
        // Update the tracker
        mKalmanTracker.updateMeasurement(wheelSpeeds[BaseMecanumDrive.LF_WHEEL_ARRAY_INDEX],
                    wheelSpeeds[BaseMecanumDrive.LR_WHEEL_ARRAY_INDEX],
                wheelSpeeds[BaseMecanumDrive.RF_WHEEL_ARRAY_INDEX],
                wheelSpeeds[BaseMecanumDrive.RR_WHEEL_ARRAY_INDEX],
                acceleration.xAccel,
                acceleration.yAccel,
                orientation.firstAngle);
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
