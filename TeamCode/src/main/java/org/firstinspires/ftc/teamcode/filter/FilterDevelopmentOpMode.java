package org.firstinspires.ftc.teamcode.filter;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drivetrain.BaseMecanumDrive;
import org.firstinspires.ftc.teamcode.speedbot.BaseSpeedBot;
import org.firstinspires.ftc.teamcode.util.LogFile;

@TeleOp(name="FilterDevelopment", group="Robot")
public class FilterDevelopmentOpMode extends OpMode{
    public static final String LOG_PATHNAME = "/sdcard";

    public static final String LOG_FILENAME = "kflog.csv";
    public static final String[] LOG_COLUMNS = {"time","w_lf","w_rf","w_lr","w_rr","theta_imu","target_ang_deg","kf_px","kf_py","kf_wz","kf_heading","kf_heading_deg","rot_cmd","pwr_cmd","steer_cmd"};
    private LogFile mLogFile;

    public static final double INIT_PX = 0D;
    public static final double INIT_PY = 0D;
    public static final double INIT_HEADING = 0;
    public static final double T = 0.050d;
    private static final int T_NS = Math.round((float)(T * 1e9d));

    private BaseSpeedBot mSpeedBot = null;

    private KalmanTracker mKalmanTracker = null;
    private Acceleration mIMUAcceleration;
    private Orientation mIMUOrientation;
    private long mLastSystemTimeNS = 0;
    private long mElapsedTimeNS = 0;
    private long mStartTimeNS = 0;

    @Override
    public void init() {
        msStuckDetectInit = 1000000;
        mElapsedTimeNS = 0;

        String initErrs = "";
        try {
            mSpeedBot = new BaseSpeedBot(this, true);
            mSpeedBot.init();
         }
        catch(Exception e){
            initErrs += ","+e.getMessage();
        }

        if (initErrs.length() == 0){
            telemetry.addData("Status:","Robot init complete");
        }
        else {
            telemetry.addData("Init errors:", initErrs);
        }
        telemetry.addData("IMU cal status",mSpeedBot.getGuidanceController().getIMUCalibrationStatus());
        telemetry.update();

        // Initialize the KalmanTracker
        mKalmanTracker = new KalmanTracker();
        mKalmanTracker.init(T,INIT_PX,INIT_PY,INIT_HEADING,BaseSpeedBot.LX,BaseSpeedBot.LY,BaseSpeedBot.WHEEL_RADIUS);

        // And the wheel speed log file
        mLogFile = new LogFile(LOG_PATHNAME, LOG_FILENAME, LOG_COLUMNS);
        mLogFile.openFile();
    }

    @Override
    public void stop() {
        mLogFile.closeFile();
        super.stop();
    }

    @Override
    public void start() {
        mLastSystemTimeNS = System.nanoTime();
        mElapsedTimeNS = 0;
        mStartTimeNS = mLastSystemTimeNS;

         super.start();
    }

    public void loop() {
        // Compute the delta time and update the Tracker if we are at the sample period T
        long systemTime = System.nanoTime();

        // Service the drivetrain loop to update wheel speed measurements
        mSpeedBot.getDrivetrain().loop();

        int deltat_ns = (int)(systemTime-mLastSystemTimeNS);
        if (deltat_ns >= T_NS){
            mElapsedTimeNS = systemTime-mStartTimeNS;
            updateTracker();
            mLastSystemTimeNS = systemTime;   // save for next loop
            // TODO: Get the target position from the state machine controller
            double targetX = 1d;
            double targetY = 1d;

            // Update the guidance controller
            double rotationModeThreshold = Math.PI/32;
            GuidanceController gc = mSpeedBot.getGuidanceController();
            if (gc.isIMUInitialized()) {
                // Set the target and current position
                gc.setTargetPosition(targetX, targetY);
                gc.setCurrentPosition(mKalmanTracker.getEstimatedHeading(),mKalmanTracker.getEstimatedXPosition(),mKalmanTracker.getEstimatedYPosition());
                // Check the angle to the target
                double angle = gc.getHeadingAngleToTarget();

                if (Math.abs(angle) > rotationModeThreshold) {
                    // Rotate the robot first instead in rotation mode
                    gc.updateRotationMode(mKalmanTracker.getEstimatedHeading(), mKalmanTracker.getEstimatedXPosition(), mKalmanTracker.getEstimatedXPosition());
                }
                else{
                    // update in steering mode
                    gc.updateSteeringMode(mKalmanTracker.getEstimatedHeading(), mKalmanTracker.getEstimatedXPosition(), mKalmanTracker.getEstimatedXPosition());
                }
                // Now apply output to motors depending on rotation or normal mode
                if (gc.isRotationModeActive()){
                    mSpeedBot.getDrivetrain().setRotationCommand(gc.getRotationCommand());
                }
                else {
                    mSpeedBot.getDrivetrain().stop();
   //                 mSpeedBot.getDrivetrain().setSteeringCommand(gc.getSteeringCommand(),gc.getPowerCommand());
                }
            }
            // Log a record of data
            logData();
        }
     }

    private void logData(){
        // Now output the wheel speeds to telemetry and to the log file
        double[] speeds = mSpeedBot.getDrivetrain().getWheelSpeeds();
        telemetry.addData("WhlSpds:","%4.2f,%4.2f,%4.2f,%4.2f",speeds[0],speeds[1],speeds[2],speeds[3]);
        telemetry.update();

        // Now form the record for the log
        String[] logRecord = new String[LOG_COLUMNS.length];
        int logIndex = 0;
        double time = (double)mElapsedTimeNS/1e9d;
        logRecord[logIndex++] = String.format("%4.3f",time);
        // Now the speeds
        for(int i=0;i < speeds.length;i++){
            logRecord[logIndex++] = String.format("%4.2f",speeds[i]);
        }
        // IMU data
         logRecord[logIndex++] = String.format("%4.2f",mIMUOrientation.firstAngle);
        logRecord[logIndex++] = String.format("%2.2f",mSpeedBot.getGuidanceController().getHeadingAngleToTarget()*180d/Math.PI);

        // Kalman outputs
        logRecord[logIndex++] = String.format("%2.2f",mKalmanTracker.getEstimatedXPosition());
        logRecord[logIndex++] = String.format("%2.2f",mKalmanTracker.getEstimatedYPosition());
        logRecord[logIndex++] = String.format("%2.2f",mKalmanTracker.getEstimatedAngularVelocity());
        logRecord[logIndex++] = String.format("%2.2f",mKalmanTracker.getEstimatedHeading());
        logRecord[logIndex++] = String.format("%2.2f",mKalmanTracker.getEstimatedHeading()*180d/Math.PI);
        logRecord[logIndex++] = String.format("%2.2f",mSpeedBot.getGuidanceController().getRotationCommand());
        logRecord[logIndex++] = String.format("%2.2f",mSpeedBot.getGuidanceController().getPowerCommand());
        logRecord[logIndex++] = String.format("%2.2f",mSpeedBot.getGuidanceController().getSteeringCommand());

        mLogFile.writeLogRow(logRecord);

    }

    /**
     * helper function to update the Tracker
     */
    private void updateTracker(){
        // Get the wheel speeds
        double[] wheelSpeeds = mSpeedBot.getDrivetrain().getWheelSpeeds();

        // Now get the IMU orientation
        GuidanceController gc = mSpeedBot.getGuidanceController();
        mIMUOrientation = gc.getIMU().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        // Update the tracker.  Have to negate the IMU angle because IMU angles go positive to the left and we want to
        // use the compass where angle increases to the right
        mKalmanTracker.updateMeasurement(wheelSpeeds[BaseMecanumDrive.LF_WHEEL_ARRAY_INDEX],
                    wheelSpeeds[BaseMecanumDrive.LR_WHEEL_ARRAY_INDEX],
                wheelSpeeds[BaseMecanumDrive.RF_WHEEL_ARRAY_INDEX],
                wheelSpeeds[BaseMecanumDrive.RR_WHEEL_ARRAY_INDEX],
                -mIMUOrientation.firstAngle);

     }


}
