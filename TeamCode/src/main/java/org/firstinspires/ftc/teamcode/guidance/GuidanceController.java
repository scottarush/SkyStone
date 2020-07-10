package org.firstinspires.ftc.teamcode.guidance;

import android.util.Log;

import org.firstinspires.ftc.teamcode.util.LogFile;
import org.firstinspires.ftc.teamcode.util.MiniPID;

import java.util.ArrayList;
import java.util.Iterator;

/**
 * GuidanceController encapsulates a PID algorithm that generates corrective
 * gain for guidance to an x,y field target position.
 */
public class GuidanceController {


    private MiniPID mSteeringModeSteeringPID = null;
    private MiniPID mSteeringModePowerPID = null;
    private MiniPID mRotationModePID = null;
    private MiniPID mStraightModePID = null;

    private double mSteeringPowerCommand = 0d;
    private double mSteeringCommand = 0d;
    private double mRotationCommand = 0d;
    private double mStraightCommand = 0d;

    public static final int STOPPED = 0;
    public static final int STEERING_MODE = 1;
    public static final int STRAIGHT_MODE = 2;
    public static final int ROTATION_MODE = 3;
    private int mMode = STOPPED;

    private double mTargetPX = 0d;
    private double mTargetPY = 0d;

    private KalmanTracker mKalmanTracker = null;
    private LogFile mLogFile = null;
    private static boolean ENABLE_LOGGING = false;
    public static final String[] LOG_COLUMNS = {"px","py","theta","distance","projection","angle"};

    private ArrayList<IGuidanceControllerCommandListener> mCommandListeners = new ArrayList<>();

    private GuidanceControllerParameters mGCParameters = new GuidanceControllerParameters();

    public static class GuidanceControllerParameters {
        public double steeringModePropGain = 0.1d;
        public double steeringModeIntegGain = 0.05d;
        public double steeringModeDerivGain = 0.05d;

        /**
         * Minimum distance in meters for steering mode operation.  Below this distance
         * steering mode will automatically transition to forward approach mode.
         */
        public double steeringModeMinimumDistance = 0.4d;

        /**
         * Minimum angle threshold for transition from rotation to steering mode.
         */
        public double steeringModeMinimumTargetAngle = Math.PI/8;

        public double steeringModePowerPropGain = 0.1d;
        public double steeringModePowerIntegGain = 0.05d;
        public double steeringModePowerDerivGain = 0.05d;

        /**
         * Minimum angle threshold for rotation mode.
         */
        public double rotationModeMinimumAngle = Math.PI/8;
        public double rotationModePropGain = 0.5d;
        public double rotationModeIntegGain = 0.07d;
        public double rotationModeDerivGain = 0.3d;

        /**
         * Minimum delta between distance to target and straight mode distance
         * used to determine when the projection of the line has been reached
         */
        public double straightModeProjectionStopDistance = 0.01;
        public double straightModePropGain = 0.5d;
        public double straightModeIntegGain = 0.003d;
        public double straightModeDerivGain = 0.5d;
    }

    public GuidanceController(GuidanceControllerParameters gcParameters, KalmanTracker kalmanTracker){
        mKalmanTracker = kalmanTracker;
        mGCParameters = gcParameters;

        // Steering controller limited to +/- 1
        mSteeringModeSteeringPID = new MiniPID(mGCParameters.steeringModePropGain,mGCParameters.steeringModeIntegGain,mGCParameters.steeringModeDerivGain);
        mSteeringModeSteeringPID.setOutputLimits(-1d,+1d);

        // Power controller limited to 0 to 1.0
        mSteeringModePowerPID = new MiniPID(mGCParameters.steeringModePowerPropGain,mGCParameters.steeringModePowerIntegGain,mGCParameters.steeringModePowerDerivGain);
        mSteeringModePowerPID.setOutputLimits(0d,1.0d);

        // Rotation controller limited to -1.0 to 1.0
        mRotationModePID = new MiniPID(mGCParameters.rotationModePropGain,mGCParameters.rotationModeIntegGain,mGCParameters.rotationModeDerivGain);
        mRotationModePID.setOutputLimits(-1.0,1.0d);

        // Straight controller limited to -1.0 to 1.0
        mStraightModePID = new MiniPID(mGCParameters.straightModePropGain,mGCParameters.straightModeIntegGain,mGCParameters.straightModeDerivGain);
        mStraightModePID.setOutputLimits(-1.0,1.0d);

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
     * sets the target position.  The correct mode will be selected based on the distance
     * to the target and the current orientation of the robot.
     */
    public void setTargetPosition(double px,double py){
        mTargetPX = px;
        mTargetPY = py;
        // Compute the delta angle for the heading to determine if we need to do a
        // rotation first
        double targetAngle = getHeadingToTargetDeltaAngle();
        if (Math.abs(targetAngle) <= mGCParameters.steeringModeMinimumTargetAngle){
            // Already within minimum so check for steering vs. approach.
            double distance = getDistanceToTarget();
            if (distance <= mGCParameters.steeringModeMinimumDistance){
                // Go directly to approach mode
                mMode = STRAIGHT_MODE;
            }
            else{
                mMode = STEERING_MODE;
            }
        }
        else{
            // Do a rotation first
            mMode = ROTATION_MODE;
        }
    }

    /**
     *
     */

    /**
     * adds  listener for guidance controller command output.
     */
    public void addGuidanceControllerCommandListener(IGuidanceControllerCommandListener listener){
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
            case STEERING_MODE:
                // Compute distance to the target
                double distance = getDistanceToTarget();
                if (distance <= mGCParameters.steeringModeMinimumDistance){
                    // Transition to approach mode
                    mMode = STRAIGHT_MODE;
                    mStraightModePID.reset();
                }
                break;
            case ROTATION_MODE:
                // Compute delta angle to the target
                double angle = getHeadingToTargetDeltaAngle();
                if (Math.abs(angle) <= mGCParameters.rotationModeMinimumAngle){
                    // Transition to either steering or straight depending upon distance to target
                    distance = getDistanceToTarget();
                    if (distance <= mGCParameters.steeringModeMinimumDistance){
                        if (distance <= getStraightModeDistanceToTarget()){
                            // As close as we can get so stop
                            mMode = STOPPED;
                        }
                        else{
                            // steer the rest of the way
                            mMode = STEERING_MODE;
                            mSteeringModePowerPID.reset();
                            mSteeringModeSteeringPID.reset();
                        }
                    }
                    else{
                        // Otherwise, too close for steer mode so go straight the
                        // rest of the way.
                        mMode = STRAIGHT_MODE;
                        mStraightModePID.reset();
                    }
                }
                // Otherwise, drop through to continue in rotation mode
                break;
            case STRAIGHT_MODE:
                distance = getStraightModeDistanceToTarget();
                 if (distance <= mGCParameters.straightModeProjectionStopDistance){
                    mMode = STOPPED;
                }
                break;
            case STOPPED:
                break;
        }
        // Now update the current (possibly new) mode
        switch(mMode) {
            case STEERING_MODE:
                updateSteeringMode();
                break;
            case ROTATION_MODE:
                updateRotationMode();
                break;
            case STRAIGHT_MODE:
                updateStraightMode();
                break;
            case STOPPED:
                for(Iterator<IGuidanceControllerCommandListener> iter=mCommandListeners.iterator();iter.hasNext();){
                    IGuidanceControllerCommandListener listener = iter.next();
                    listener.setStraightCommand(0d);
                }
                break;
        }
     }

    /**
     * Returns the current guidance controller operating mode.
     */
    public int getMode(){
        return mMode;
    }


    /**
     * updates the steering mode PID
     */
    private void updateSteeringMode(){
         // Compute the angle to the target relative to the current position
        double angleToTarget = getHeadingToTargetDeltaAngle();

        // And compute the distance
        // Power command is computed using the new distance to the target
        mSteeringPowerCommand = mSteeringModePowerPID.getOutput(-getDistanceToTarget(),0d);
        // Steering  command is computed using angleToTarget as the new setpoint and
        // the current heading
        mSteeringCommand = mSteeringModeSteeringPID.getOutput(mKalmanTracker.getEstimatedHeading(),angleToTarget);
        for(Iterator<IGuidanceControllerCommandListener> iter=mCommandListeners.iterator();iter.hasNext();){
            IGuidanceControllerCommandListener listener = iter.next();
            listener.setSteeringCommand(mSteeringCommand, mSteeringPowerCommand);
        }
    }

    /**
     * Returns delta angle between the robot heading and the angle to the target.
     * when pointed at the target, this function should return 0.
     */
    public double getHeadingToTargetDeltaAngle(){
        return getAngleToTarget()-mKalmanTracker.getEstimatedHeading();
    }

    /**
     * returns linear distance to target from robot.
     */
    public double getDistanceToTarget(){
        return Math.sqrt(Math.pow(mTargetPX-mKalmanTracker.getEstimatedXPosition(),2.0d)+
                Math.pow(mTargetPY-mKalmanTracker.getEstimatedYPosition(),2.0d));
    }

    /**
     * internal utility computes straight line distance to target for straight mode only
     */
    private double getStraightModeDistanceToTarget(){
        // Compute the vector from the current position to the target as distance /_ angle
        double angle = getHeadingToTargetDeltaAngle();
        double distance = getDistanceToTarget();
        // Now compute the projection of this vector into the straight line path of the robot
        // This will give us the closest point we can approach to the target on a straight line.
        double projection = distance / (Math.cos(angle));
        if (ENABLE_LOGGING) {
            int logIndex = 0;
            String logRecord[] = new String[LOG_COLUMNS.length];
            logRecord[logIndex++] = String.format("%4.2f", mKalmanTracker.getEstimatedXPosition());
            logRecord[logIndex++] = String.format("%4.2f", mKalmanTracker.getEstimatedYPosition());
            logRecord[logIndex++] = String.format("%5.2f", mKalmanTracker.getEstimatedHeading() * 180d / Math.PI);
            logRecord[logIndex++] = String.format("%4.2f", distance);
            logRecord[logIndex++] = String.format("%4.2f", projection);
            logRecord[logIndex++] = String.format("%4.2f", angle);

            mLogFile.writeLogRow(logRecord);
        }
        return projection;
    }
    /**
     * update function used to rotate toward the current target position. This mode will only rotate
     * the robot, not move it forward or backward.
     */
    private void updateRotationMode(){
         mRotationCommand = mRotationModePID.getOutput(mKalmanTracker.getEstimatedHeading(),getAngleToTarget());
         for(Iterator<IGuidanceControllerCommandListener> iter=mCommandListeners.iterator();iter.hasNext();){
             IGuidanceControllerCommandListener listener = iter.next();
             listener.setRotationCommand(mRotationCommand);
         }
    }
    /**
     * update function used to approach a target position.  Only moves the robot forward or backward
     * assuming that the robot is pointing directly at the target.
     */
    private void updateStraightMode(){
        // Look for the distance to target and straight mode delta to determine when to stop
        double straightModeDistance = getStraightModeDistanceToTarget();
        if (straightModeDistance <= mGCParameters.straightModeProjectionStopDistance){
            mStraightCommand = 0d;
            mMode = STOPPED;
        }
        else{
            mStraightCommand = mStraightModePID.getOutput(straightModeDistance,0d);
            // Invert the command to go forward
            mStraightCommand *= -1d;
        }
       for(Iterator<IGuidanceControllerCommandListener> iter=mCommandListeners.iterator();iter.hasNext();){
            IGuidanceControllerCommandListener listener = iter.next();
            listener.setStraightCommand(mStraightCommand);
        }

    }


    /**
     * utility computes the angle between the current robot position and the
     * target position from 0..PI = N->E->S and -PI..0 = N->W->S
     */
    private double getAngleToTarget(){
        double xrel = mTargetPX-mKalmanTracker.getEstimatedXPosition();
        double yrel = mTargetPY-mKalmanTracker.getEstimatedYPosition();
        // Compute the angle assuming the robot is pointed straight north and add
        // the current heading afterward
        double angleToTarget = 0d;
        double invtan = 0d;
        // Handle straight line case with yrel = 0 as inverse tan will blow up at 0 and PI
        if (yrel == 0d){
            if (xrel >= 0d){
                return Math.PI/2d;
            }
            else{
                return -Math.PI/2d;
            }
        }
        // Otherwise compute angle from the inverse tangent
        invtan = Math.abs(Math.atan(Math.abs(xrel)/Math.abs(yrel)));
        if (xrel >= 0d){
            if (yrel >= 0d){
                // northeast quadrant
                angleToTarget = invtan;
            }
            else{
                // southeast quadrant
                angleToTarget = Math.PI-invtan;
            }
        }
        else{
            // xrel is negative
            if (yrel >= 0d){
                // northwest quadrant
                angleToTarget = -invtan;
            }
            else{
                // southwest quadrant
                angleToTarget = -(Math.PI-invtan);
            }

        }
        return angleToTarget;
    }

    /**
     * returns the steering command  for logging
     */
    public double getSteeringCommand(){
        return mSteeringCommand;
    }
    /**
     * returns the steering power command  for logging
     */
    public double getSteeringPowerCommand(){
        return mSteeringPowerCommand;
    }
    /**
     * returns the rotation command  for logging
     */
    public double getRotationCommand(){
        return mRotationCommand;
    }
    /**
     * returns the straight command  for logging
     */
    public double getStraightCommand(){
        return mStraightCommand;
    }
    /**
     * returns the current mode as a string for logging.
     */
    public String getModeString(){
        switch(mMode) {
            case STEERING_MODE:
                return "STEERING";
            case ROTATION_MODE:
                return "ROTATION";
            case STRAIGHT_MODE:
                return "STRAIGHT";
            case STOPPED:
                return "STOPPED";
            default:
                return "INVALID_MODE";  // Can't happen
        }
    }

}
