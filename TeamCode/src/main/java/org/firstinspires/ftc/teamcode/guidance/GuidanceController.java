package org.firstinspires.ftc.teamcode.guidance;

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
    private MiniPID mPathSteeringPID = null;
    private MiniPID mPathPowerPID = null;
    private double mRotationCommand = 0d;
    private double mPathSteeringCommand = 0d;

    private double mPathPowerCommand = 0d;

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

    private ArrayList<IGuidanceControllerCommandListener> mCommandListeners = new ArrayList<>();
    private ArrayList<IGuidanceControllerStatusListener> mStatusListeners = new ArrayList<>();

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
        public double rotationModeMaxIntegGain = 0.2d;
        public double rotationModeDerivGain = 0d;

        /**
         * Maximum angle to target to be able to enter path mode.
         */
        public double pathModeMaxEntryAngle = 20.0d*Math.PI/180;

        public double pathModeSteeringPropGain = 0.1d;
        public double pathModeSteeringIntegGain = 0d;
        public double pathModeSteeringMaxIntegOutput = 0.2d;
        public double pathModeSteeringDerivGain = 0d;

        public double pathModePowerPropGain = 0.7d;
        public double pathModePowerIntegGain = 0.01d;
        public double pathModePowerMaxIntegOutput = 0.2d;
        public double pathModePowerDerivGain = 0d;
    }

    public GuidanceController(GuidanceControllerParameters gcParameters, KalmanTracker kalmanTracker){
        mKalmanTracker = kalmanTracker;
        mGCParameters = gcParameters;

        // Rotation controller limited to -1.0 to 1.0
        mRotationModePID = new MiniPID(mGCParameters.rotationModePropGain,mGCParameters.rotationModeIntegGain,mGCParameters.rotationModeDerivGain);
        mRotationModePID.setMaxIOutput(mGCParameters.rotationModeMaxIntegGain);
        mRotationModePID.setOutputLimits(-1.0,1.0d);

        mPathSteeringPID = new MiniPID(mGCParameters.pathModeSteeringPropGain,mGCParameters.pathModeSteeringIntegGain,mGCParameters.pathModeSteeringDerivGain);
        mPathSteeringPID.setMaxIOutput(mGCParameters.pathModeSteeringMaxIntegOutput);
        mPathSteeringPID.setOutputLimits(-1.0,1.0d);

        mPathPowerPID = new MiniPID(mGCParameters.pathModePowerPropGain,mGCParameters.pathModePowerIntegGain,mGCParameters.pathModePowerDerivGain);
        mPathPowerPID.setMaxIOutput(mGCParameters.pathModePowerMaxIntegOutput);
        mPathPowerPID.setOutputLimits(0d,1.0d);

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
     * @return true if successful, false if the current robot heading is beyond the maximum allowed entry angle (and must be rotated first)
     */
    public boolean doPathFollow(double px,double py){
        mPathLineStart = new Point(mKalmanTracker.getEstimatedXPosition(),mKalmanTracker.getEstimatedYPosition());
        mPathLineEnd = new Point(px,py);

        // Compute the heading angle of the robot relative to the end point and determine
        // if we are within the maximum entry angle for stability
        double theta = mKalmanTracker.getEstimatedHeading();
        Point rotatedEnd = mPathLineEnd.rotate(theta);
        Point rotatedStart = mPathLineStart.rotate(theta);
        Point robotPos = new Point(mKalmanTracker.getEstimatedXPosition(),mKalmanTracker.getEstimatedYPosition());
        double m = (rotatedEnd.y - rotatedStart.y) / (rotatedEnd.x - rotatedStart.x);
        double angle = -Math.atan(m);
        if (Math.abs(angle) >= mGCParameters.pathModeMaxEntryAngle){
            // Robot heading is too far for stability so return flase
            return false;
        }
        // Start the path follow
        mPathSteeringPID.reset();
        mPathPowerPID.reset();

        // Stop the robot in case it was moving
        clearAllCommands();
        // Set mode to path and wait for update
        mMode = PATH_MODE;
        return true;
    }

    /**
     * Does a rotation maneuver The rotation will be completed and notified to listeners via the
     * IGuidanceControllerStatusListener interface.
     **/
    public void doRotation(double heading){
        mMode = ROTATION_MODE;
        mTargetHeading = heading;
        // Clear all commands in case robot was moving
        clearAllCommands();
        mRotationModePID.reset();
    }

    /**
     * adds  listener for guidance controller command output notifications.
     */
    public void addGuidanceControllerCommandListener(IGuidanceControllerCommandListener listener){
        if (mCommandListeners.contains(listener)){
            return;
        }
        mCommandListeners.add(listener);
    }

    /**
     * adds  listener for guidance controller status notifications.
     */
    public void addGuidanceControllerStatusListener(IGuidanceControllerStatusListener listener){
        if (mStatusListeners.contains(listener)){
            return;
        }
        mStatusListeners.add(listener);
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
                // Notify command listeners to stop the rotation command
                for (Iterator<IGuidanceControllerCommandListener> iter = mCommandListeners.iterator(); iter.hasNext(); ) {
                    IGuidanceControllerCommandListener listener = iter.next();
                    listener.setRotationCommand(0d);
                }
                // And status listeners that the maneuver is complete
                for (Iterator<IGuidanceControllerStatusListener> iter = mStatusListeners.iterator(); iter.hasNext(); ) {
                    IGuidanceControllerStatusListener listener = iter.next();
                    listener.rotationComplete();
                }
            }
            return;
        }
        // Otherwise, drop through to compute regular rotation command
        mRotationCommand = mRotationModePID.getOutput(mKalmanTracker.getEstimatedHeading(),mTargetHeading);
        for(Iterator<IGuidanceControllerCommandListener> iter = mCommandListeners.iterator(); iter.hasNext();){
            IGuidanceControllerCommandListener listener = iter.next();
            listener.setRotationCommand(mRotationCommand);
        }
    }

    /**
     * utility sends a zero to null all commands to zero
     */
    private void clearAllCommands(){
        for(Iterator<IGuidanceControllerCommandListener> iter = mCommandListeners.iterator(); iter.hasNext();){
            IGuidanceControllerCommandListener listener = iter.next();
            listener.setStraightCommand(0d);
            listener.setRotationCommand(0d);
            listener.setSteeringCommand(0d,0d);
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
            mPathSteeringCommand = 0d;
            mPathPowerCommand = 0d;
            mMode = STOPPED;
        }
        else {
            // Not yet at the target so control steering using the angle between the rotated
            // heading (which is now = 0) and the angle of the line (which is the arctan
            // of the slope.
            // Line formula:  y = mx + b where m = (endy-starty)/(endx-startx) and b = starty
            double m = (rotatedEnd.y - rotatedStart.y) / (rotatedEnd.x - rotatedStart.x);
            // Angle is negative because our steering command is + to the right.
            double angle = -Math.atan(m);
            mPathSteeringCommand = mPathSteeringPID.getOutput(angle, 0);
            // And control the power based on the distance to the target's y coordinate
            mPathPowerCommand = mPathPowerPID.getOutput(rotRobotPos.y,rotatedEnd.y);
        }

        for(Iterator<IGuidanceControllerCommandListener> iter = mCommandListeners.iterator(); iter.hasNext();){
            IGuidanceControllerCommandListener listener = iter.next();
            listener.setSteeringCommand(mPathSteeringCommand, mPathPowerCommand);
        }
        if (mMode == STOPPED){
            // notify that the path follow was complete
            for(Iterator<IGuidanceControllerStatusListener> iter = mStatusListeners.iterator(); iter.hasNext();){
                IGuidanceControllerStatusListener listener = iter.next();
                listener.pathFollowComplete();
            }
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
    public double getPathSteeringCommand(){
        return mPathSteeringCommand;
    }
    /**
     * returns the power command for logging
     */
    public double getPathPowerCommand(){
        return mPathPowerCommand;
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
