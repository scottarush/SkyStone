package org.firstinspires.ftc.teamcode.filter;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.util.MiniPID;

import java.io.File;

/**
 * GuidanceController encapsulates a PID algorithm that generates corrective
 * gain for guidance to an x,y field target position.
 */
public class GuidanceController {


    private MiniPID mSteeringPID = null;
    private MiniPID mPowerPID = null;
    private MiniPID mRotationPID = null;

    private double mPowerCommand = 0d;
    private double mSteeringCommand = 0d;
    private double mRotationCommand = 0d;
    private boolean mRotationModeActive = false;

    private double mTargetPX = 0d;
    private double mTargetPY = 0d;

    private KalmanTracker mKalmanTracker = null;

    public static class Parameters {
        public double steeringPropGain = 0.1d;
        public double steeringIntegGain = 0.05d;
        public double steeringDerivGain = 0.05d;

        public double powerPropGain = 0.1d;
        public double powerIntegGain = 0.05d;
        public double powerDerivGain = 0.05d;

        public double rotationPropGain = 0.5d;
        public double rotationIntegGain = 0.07d;
        public double rotationDerivGain = 0.3d;
    }

    public GuidanceController(Parameters pidParams,KalmanTracker kalmanTracker){
        mKalmanTracker = kalmanTracker;

        // Steering controller limited to +/- 1
        mSteeringPID = new MiniPID(pidParams.steeringPropGain,pidParams.steeringIntegGain,pidParams.steeringDerivGain);
        mSteeringPID.setOutputLimits(-1d,+1d);

        // Power controller limited to 0 to 1.0
        mPowerPID = new MiniPID(pidParams.powerPropGain,pidParams.powerIntegGain,pidParams.powerDerivGain);
        mPowerPID.setOutputLimits(0d,1.0d);

        // Rotation controller limited to -1.0 to 1.0
        mRotationPID = new MiniPID(pidParams.rotationPropGain,pidParams.rotationIntegGain,pidParams.rotationDerivGain);
        mRotationPID.setOutputLimits(-1.0,1.0d);

    }

    /**
     * sets the target position.
     */
    public void setTargetPosition(double px,double py){
        mTargetPX = px;
        mTargetPY = py;
    }


    /**
     * updates the steering mode PID
     * After calling, the commanded values can be retrieved with {@link #getPowerCommand()}
     * and {@link #getSteeringCommand()}
     */
    public void updateSteeringMode(){
        if (mRotationModeActive){
            // Cancel rotation mode and reset both PIDs
            mRotationModeActive = false;
            mSteeringPID.reset();
            mPowerPID.reset();
        }
        // Compute the angle to the target relative to the current position
        double angleToTarget = getHeadingToTargetDeltaAngle();

        // And compute the distance
        // Power command is computed using the new distance to the target
        mPowerCommand = mPowerPID.getOutput(-getDistanceToTarget(),0d);
        // Steering  command is computed using angleToTarget as the new setpoint and
        // the current heading
        mSteeringCommand = mSteeringPID.getOutput(mKalmanTracker.getEstimatedHeading(),angleToTarget);
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
     * update function used to rotate toward the current target position.  The computed
     * {@link #getRotationCommand()}  for this mode will only rotate
     * the robot, not move it forward or backward.
     *
     * The caller will have to threshold the returned command to decide when to
     * stop the rotation.
     *
     * On first call to this function from the normal update mode, both PID will be reset.
     */
    public void updateRotationMode(){
        if (!mRotationModeActive){
            // Cancel rotation mode and reset both PIDs
            mRotationModeActive = true;
            mRotationCommand = 0d;
            mRotationPID.reset();
        }
        mRotationCommand = mRotationPID.getOutput(mKalmanTracker.getEstimatedHeading(),getAngleToTarget());
    }

    public boolean isRotationModeActive(){
        return mRotationModeActive;
    }
    /**
     * utility computes the angle between the current robot position and the
     * target position from 0..PI = N->E->S and -PI..0 = N->W->S
     */
    private double getAngleToTarget(){
        double xrel = mTargetPX-mKalmanTracker.getEstimatedYPosition();
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
     * returns the current steering command only valid in normal mode
     * @return current steering command -1 for left, +1 for right.
     */
    public double getSteeringCommand(){
        return mSteeringCommand;
    }
    /**
     * returns the current rotation command only valid in rotation mode
     * @return current steering command -1 for left, +1 for right.
     */
    public double getRotationCommand(){
        return mRotationCommand;
    }   /**
     * @return commanded power - range -1.0 (reverse) to +1.0 forward
     */
    public double getPowerCommand(){
        return mPowerCommand;
    }
}
