package org.firstinspires.ftc.teamcode.guidance;

import android.graphics.PointF;

import org.firstinspires.ftc.teamcode.util.LogFile;
import org.firstinspires.ftc.teamcode.util.MiniPID;

import java.util.ArrayList;
import java.util.Iterator;

/**
 * GuidanceController encapsulates a PID algorithm that generates corrective
 * gain for guidance to an x,y field target position.
 */
public class GuidanceController {

    private class Point {
        public Point(double x, double y){
            this.x = x;
            this.y = y;
        }
        public double x = 0d;
        public double y = 0d;

        public Point rotate(double theta){
            double rx = Math.cos(theta) * x - Math.sin(theta) * y;
            double ry = Math.sin(theta) * x + Math.cos(theta) * y;
            return new Point(rx,ry);
        }
    }

    private MiniPID mRotationModePID = null;
    private MiniPID mPathModePID = null;

    private double mRotationCommand = 0d;
    private double mPathCommand = 0d;

    private double mCommandPower = 0d;

    public static final int STOPPED = 0;
    public static final int ROTATION_MODE = 1;
    public static final int PATH_MODE = 2;
    private int mMode = STOPPED;

    private double mTargetHeading = 0d;

    private KalmanTracker mKalmanTracker = null;
    private LogFile mLogFile = null;
    private static boolean ENABLE_LOGGING = false;
    public static final String[] LOG_COLUMNS = {"px","py","theta","distance","projection","angle"};

    private Point mPathLineStart = null;
    private Point mPathLineEnd = null;

    private ArrayList<IGuidanceControllerListener> mCommandListeners = new ArrayList<>();

    private GuidanceControllerParameters mGCParameters = new GuidanceControllerParameters();

    public static class GuidanceControllerParameters {
        /**
         * Minimum angle threshold for rotation mode to complete
         */
        public double rotationModeStopAngleError = 3d*Math.PI/180;
        /**
         * Angular velocity threshold to stop in radians per sec.
         */
        public double rotationModeStopAngularVelocityThreshold = 10.0d*Math.PI/180;
        public double rotationModePropGain = 0.5d;
        public double rotationModeIntegGain = 0d;
        public double rotationModeDerivGain = 0d;

        /**
         * Maximum angle to target to be able to enter path mode.
         */
        public double pathModeMaxEntryAngle = 20.0d*Math.PI/180;

        public double pathModePropGain = 0.05d;
        public double pathModeIntegGain = 0.1d;
        public double pathModeDerivGain = 2d;

    }

    public GuidanceController(GuidanceControllerParameters gcParameters, KalmanTracker kalmanTracker){
        mKalmanTracker = kalmanTracker;
        mGCParameters = gcParameters;

        // Rotation controller limited to -1.0 to 1.0
        mRotationModePID = new MiniPID(mGCParameters.rotationModePropGain,mGCParameters.rotationModeIntegGain,mGCParameters.rotationModeDerivGain);
        mRotationModePID.setOutputLimits(-1.0,1.0d);

        // Straight controller limited to -1.0 to 1.0
        mPathModePID = new MiniPID(mGCParameters.pathModePropGain,mGCParameters.pathModeIntegGain,mGCParameters.pathModeDerivGain);
        mPathModePID.setOutputLimits(-1.0,1.0d);

        if (ENABLE_LOGGING){
            mLogFile = new LogFile("/sdcard","gclog.csv",LOG_COLUMNS);
            mLogFile.openFile();
        }
    }

    public void closeLogFile(){
        if (ENABLE_LOGGING)
            mLogFile.closeFile();
    }

    /**
     * Engages path mode to a line
     * Starting point of the line is the current center of the robot.
     * @param px x coordinate of line terminus
     * @param py y coordinate of line terminus
     * @param power power value to use duing path follow
     * @return true if successful, false if the current robot heading is beyond the maximum allowed entry angle (and must be rotated first)
     */
    public boolean doPathFollow(double px,double py,double power){
        // Compute the heading angle of the robot relative to the point
        // TODO finish this

        mCommandPower = power;

        mPathLineStart = new Point(mKalmanTracker.getEstimatedXPosition(),mKalmanTracker.getEstimatedYPosition());
        mPathLineEnd = new Point(px,py);
        mPathModePID.reset();

        // Stop the robot in case it was moving
        clearAllCommands();
        // Set mode to path and wait for update
        mMode = PATH_MODE;
        return true;
    }

    /**
     * Sets a target heading which cancels any other pending maneuver and transitions to
     * ROTATION_MODE.  The rotation will be completed and notified to listeners via the
     * IGuidanceControllerCommandListener interface.
     **/
    public void setTargetHeading(double heading){
        mMode = ROTATION_MODE;
        mTargetHeading = heading;
        mRotationModePID.reset();
    }

    /**
     * adds  listener for guidance controller command output and maneuver notifications.
     */
    public void addGuidanceControllerCommandListener(IGuidanceControllerListener listener){
        if (mCommandListeners.contains(listener)){
            return;
        }
        mCommandListeners.add(listener);
    }

    /**
     * update function triggers update of the controller.  Should ideally be called at the
     * sample rate.
     *
     */
    public void updateCommand(){
        // check if we need to make a mode transition
        switch(mMode){
             case ROTATION_MODE:
                 updateRotationMode();
               break;
             case PATH_MODE:
                 updatePathMode();
                break;
            case STOPPED:
                break;
        }
     }
    /**
     * update function used to rotate toward the current target heading.
     */
    private void updateRotationMode(){
        // Compute current heading to target error threshold to decide if we need to stop
        double error = mKalmanTracker.getEstimatedHeading()-mTargetHeading;
        if (Math.abs(error) <= mGCParameters.rotationModeStopAngleError){
            // Now check if the angular velocity has slowed enough to stop
            if (Math.abs(mKalmanTracker.getEstimatedAngularVelocity()) <= mGCParameters.rotationModeStopAngularVelocityThreshold) {
                mMode = STOPPED;
                // Notify listeners that rotation is complete.
                for (Iterator<IGuidanceControllerListener> iter = mCommandListeners.iterator(); iter.hasNext(); ) {
                    IGuidanceControllerListener listener = iter.next();
                    listener.rotationComplete();
                    listener.setRotationCommand(0d);
                }
            }
            return;
        }
        // Otherwise, drop through to compute regular rotation command
        mRotationCommand = mRotationModePID.getOutput(mKalmanTracker.getEstimatedHeading(),mTargetHeading);
        for(Iterator<IGuidanceControllerListener> iter = mCommandListeners.iterator(); iter.hasNext();){
            IGuidanceControllerListener listener = iter.next();
            listener.setRotationCommand(mRotationCommand);
        }
    }

    /**
     * utility sends a zero to null all commands to zero
     */
    private void clearAllCommands(){
        for(Iterator<IGuidanceControllerListener> iter = mCommandListeners.iterator(); iter.hasNext();){
            IGuidanceControllerListener listener = iter.next();
            listener.setStraightCommand(0d);
            listener.setRotationCommand(0d);
        }
    }
    /**
     * Returns the current guidance controller operating mode.
     */
    public int getMode(){
        return mMode;
    }

    /**
     * updates the path mode PID
     */
    private void updatePathMode(){
        double theta = mKalmanTracker.getEstimatedHeading();
        // rotate the coordinates of the robot and the line points.
        Point rotatedEnd = mPathLineEnd.rotate(theta);
        Point rotatedStart = mPathLineStart.rotate(theta);

        Point robotPos = new Point(mKalmanTracker.getEstimatedXPosition(),mKalmanTracker.getEstimatedYPosition());
        Point rotRobotPos = robotPos.rotate(theta);

        // First check if we have pass the y end of the line and stop
        boolean stop = false;
        if (rotatedEnd.y < 0){
            if (rotRobotPos.y <= rotatedEnd.y){
                stop = true;
            }
        }
        else{
            if (rotRobotPos.y >= rotatedEnd.y){
                stop = true;
            }
        }
        if (stop){
            // Set path and power to 0 and drop through to send command
            mPathCommand = 0d;
            mCommandPower = 0d;
        }
        else {
            // Not yet at the target so find rotated xintercept with rotated line
            // Line formula:  y = mx + b where m = (endy-starty)/(endx-startx) and b = starty
            // Solve for x:  x=(y-b)m
            double m = (rotatedEnd.y - rotatedStart.y) / (rotatedEnd.x - rotatedStart.x);
            double b = rotatedStart.y;
            double xintercept = (robotPos.y - b) / m;
            // And compute command to drive robot toward the rotated xintercept
            mPathCommand = mPathModePID.getOutput(rotRobotPos.x, xintercept);
        }

        for(Iterator<IGuidanceControllerListener> iter = mCommandListeners.iterator(); iter.hasNext();){
            IGuidanceControllerListener listener = iter.next();
            listener.setSteeringCommand(mPathCommand,mCommandPower);
        }
    }

     /**
     * returns the rotation command  for logging
     */
    public double getRotationCommand(){
        return mRotationCommand;
    }
    /**
     * returns the steering command for logging
     */
    public double getSteeringCommand(){
        return mPathCommand;
    }
    /**
     * returns the power command for logging
     */
    public double getCommandPower(){
        return mCommandPower;
    }
    /**
     * returns the current mode as a string for logging.
     */
    public String getModeString(){
        switch(mMode) {
            case ROTATION_MODE:
                return "ROTATION";
            case PATH_MODE:
                return "PATH";
            case STOPPED:
                return "STOPPED";
            default:
                return "INVALID_MODE";  // Can't happen
        }
    }

}
