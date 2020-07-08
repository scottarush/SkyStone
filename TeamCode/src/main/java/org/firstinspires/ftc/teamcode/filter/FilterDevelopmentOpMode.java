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
    public static final String[] LOG_COLUMNS = {"time","w_lf","w_rf","w_lr","w_rr","theta_imu","tgt_ang_deg","trgt_dist","kf_px","kf_py","kf_wz","kf_heading","kf_heading_deg","rot_cmd","pwr_cmd","steer_cmd"};
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

    private boolean mRotationModeExit = false;

    private GuidanceController mGuidanceController = null;
    @Override
    public void init() {
        msStuckDetectInit = 1000000;
        mElapsedTimeNS = 0;

        String initErrs = "";
        try {
            mSpeedBot = new BaseSpeedBot(this, true);
            mSpeedBot.init("IMUCalCtrlHubOne.json");
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
        telemetry.addData("IMU cal status",mSpeedBot.getIMUCalibrationStatus());
        telemetry.update();

        // Initialize the KalmanTracker
        mKalmanTracker = new KalmanTracker();
        mKalmanTracker.init(T,INIT_PX,INIT_PY,INIT_HEADING,BaseSpeedBot.LX,BaseSpeedBot.LY,BaseSpeedBot.WHEEL_RADIUS);

        // Initialize the guidance controller
        mGuidanceController = new GuidanceController(new GuidanceController.Parameters(),mKalmanTracker);

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
            double targetX = 0d;
            double targetY = 1.22d;

//            double xleft = gamepad1.left_stick_x;
//            double yleft = -gamepad1.left_stick_y;
//            double xright = gamepad1.right_stick_x;
//            double yright = -gamepad1.right_stick_y;
//
//            // the speeds with the new gamepad inputs
//            mSpeedBot.getDrivetrain().setTankDriveJoystickInput(xleft,yleft,xright,yright);
//            logData();

            // Update the guidance controller
            double rotationModeThreshold = Math.PI/16;

            if (mSpeedBot.isIMUInitialized()) {
                // Set the target and current position
                mGuidanceController.setTargetPosition(targetX, targetY);
                // Check the delta angle between the heading and the target
                double angle = mGuidanceController.getHeadingToTargetDeltaAngle();
                if ((Math.abs(angle) > rotationModeThreshold) && !mRotationModeExit) {
                    // Rotate the robot first instead in rotation mode
                    mGuidanceController.updateRotationMode();
                }
                else{
                    // update in steering mode
                    mRotationModeExit = true;  // Prevent entry into rotation mode for rest of cycle
                    mGuidanceController.updateSteeringMode();
                }
                // Now apply output to motors depending on rotation or normal mode
                if (mGuidanceController.isRotationModeActive()){
                    mSpeedBot.getDrivetrain().setRotationCommand(mGuidanceController.getRotationCommand());
                }
                else {
                    mSpeedBot.getDrivetrain().setSteeringCommand(mGuidanceController.getSteeringCommand(),mGuidanceController.getPowerCommand());
                }
            }
            telemetry.addData("KF Data","heading=%5.2f px=%4.1f py=%4.1f",mKalmanTracker.getEstimatedHeading()*180d/Math.PI,mKalmanTracker.getEstimatedXPosition(),mKalmanTracker.getEstimatedYPosition());
            telemetry.update();
            // Log a record of data
            logData();
        }
     }

    private void logData(){
         // Now form the record for the log
        String[] logRecord = new String[LOG_COLUMNS.length];
        int logIndex = 0;
        double time = (double)mElapsedTimeNS/1e9d;
        logRecord[logIndex++] = String.format("%4.3f",time);
        // Now the speeds
        double[] speeds = mSpeedBot.getDrivetrain().getWheelSpeeds();
        for(int i=0;i < speeds.length;i++){
            logRecord[logIndex++] = String.format("%4.2f",speeds[i]);
        }

        // IMU data
        logRecord[logIndex++] = String.format("%4.2f",mIMUOrientation.firstAngle);
        logRecord[logIndex++] = String.format("%5.2f",mGuidanceController.getHeadingToTargetDeltaAngle()*180d/Math.PI);
        logRecord[logIndex++] = String.format("%4.3f",mGuidanceController.getDistanceToTarget());

        // Kalman outputs
        logRecord[logIndex++] = String.format("%4.2f",mKalmanTracker.getEstimatedXPosition());
        logRecord[logIndex++] = String.format("%4.2f",mKalmanTracker.getEstimatedYPosition());
        logRecord[logIndex++] = String.format("%4.2f",mKalmanTracker.getEstimatedAngularVelocity());
        logRecord[logIndex++] = String.format("%5.2f",mKalmanTracker.getEstimatedHeading());
        logRecord[logIndex++] = String.format("%5.2f",mKalmanTracker.getEstimatedHeading()*180d/Math.PI);
        logRecord[logIndex++] = String.format("%4.2f",mGuidanceController.getRotationCommand());
        logRecord[logIndex++] = String.format("%4.2f",mGuidanceController.getPowerCommand());
        logRecord[logIndex++] = String.format("%4.2f",mGuidanceController.getSteeringCommand());

        mLogFile.writeLogRow(logRecord);

    }

    /**
     * helper function to update the Tracker
     */
    private void updateTracker(){
        // Get the wheel speeds
        double[] wheelSpeeds = mSpeedBot.getDrivetrain().getWheelSpeeds();

        // Now get the IMU orientation
        mIMUOrientation = mSpeedBot.getIMU().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        // Update the tracker.  Have to negate the IMU angle because IMU angles go positive to the left and we want to
        // use the compass where angle increases to the right
        mKalmanTracker.updateMeasurement(wheelSpeeds[BaseMecanumDrive.LF_WHEEL_ARRAY_INDEX],
                    wheelSpeeds[BaseMecanumDrive.LR_WHEEL_ARRAY_INDEX],
                wheelSpeeds[BaseMecanumDrive.RF_WHEEL_ARRAY_INDEX],
                wheelSpeeds[BaseMecanumDrive.RR_WHEEL_ARRAY_INDEX],
                -mIMUOrientation.firstAngle);

     }


}
