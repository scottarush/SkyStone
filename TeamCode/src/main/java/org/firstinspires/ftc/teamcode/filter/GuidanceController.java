package org.firstinspires.ftc.teamcode.filter;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.util.MiniPID;

import java.io.File;

/**
 * GuidanceController encapsulates the IMU and a PID algorithm that generates corrective
 * gain for guidance to an x,y field target position.
 */
public class GuidanceController {

    /**
     * IMU inside REV hub
     */
    private BNO055IMU mIMU = null;
    private boolean mIMUInitialized = false;

    private MiniPID mSteeringPID = null;
    private MiniPID mPowerPID = null;
    private MiniPID mRotationPID = null;

    private Parameters mPIDParameters = null;

    private double mPowerCommand = 0d;
    private double mSteeringCommand = 0d;
    private double mRotationCommand = 0d;
    private boolean mRotationModeActive = false;

    private double mTargetPX = 0d;
    private double mTargetPY = 0d;
    private double mLastPosX = 0d;
    private double mLastPosY = 0d;
    private double mLastHeading = 0d;

    public static class Parameters {
        public String imuCalibrationDataFilename = "IMUCalCtrlHubOne.json";
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

    /**
     *
     * IMPORTANT:  IMU can take a long time to initialize
     */
    public void init(HardwareMap hwMap,Parameters pidParams) throws Exception {
        mPIDParameters = pidParams;
        try{
            // Initialize the IMU
            BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();

            imuParameters.mode                = BNO055IMU.SensorMode.IMU;
            imuParameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
            imuParameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            imuParameters.calibrationDataFile = pidParams.imuCalibrationDataFilename;
//            parameters.loggingEnabled      = true;
//            parameters.loggingTag          = "IMU";
            imuParameters.loggingEnabled      = false;

            // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
            // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
            // and named "imu".
            try {
                mIMU = hwMap.get(BNO055IMU.class, "imu");

                mIMU.initialize(imuParameters);

                // Now load calibrations
                File file = AppUtil.getInstance().getSettingsFile(pidParams.imuCalibrationDataFilename);
                BNO055IMU.CalibrationData calibrationData = BNO055IMU.CalibrationData.deserialize(ReadWriteFile.readFile(file));
                mIMU.writeCalibrationData(calibrationData);

            }
            catch(Exception e){
                throw new Exception("IMU Initialization error:"+e.getMessage());
            }

            mIMUInitialized = true;
        }
        catch(Exception e){
            throw new Exception(e);
        }
        mIMUInitialized = true;
        //  Now initialize the pids

        // Steering controller limited to +/- PI/2
        mSteeringPID = new MiniPID(pidParams.steeringPropGain,pidParams.steeringIntegGain,pidParams.steeringDerivGain);
        mSteeringPID.setOutputLimits(-Math.PI/2,Math.PI/2);

        // Power controller limited to -1.0 to 1.0
        mPowerPID = new MiniPID(pidParams.powerPropGain,pidParams.powerIntegGain,pidParams.powerDerivGain);
        mPowerPID.setOutputLimits(-1.0,1.0d);

        // Rotation controller limited to -1.0 to 1.0
        mRotationPID = new MiniPID(pidParams.rotationPropGain,pidParams.rotationIntegGain,pidParams.rotationDerivGain);
        mRotationPID.setOutputLimits(-1.0,1.0d);

    }
    public boolean isIMUInitialized(){
        return mIMUInitialized;
    }


    /**
     * sets the target position.
     */
    public void setTargetPosition(double px,double py){
        mTargetPX = px;
        mTargetPY = py;
    }
    /**
     * sets the current position of the robot.  Utility function allows us to
     * call this and then call {@link #getHeadingToTargetDeltaAngle()} for decision
     * making prior to an update call
     */
    public void setCurrentPosition(double heading,double px,double py){
        mLastPosX = px;
        mLastPosY = py;
        mLastHeading = heading;
    }

    /**
     * updates the PID controllers with the new current heading and x,y position from
     * the Kalman filter for a direct drive toward the target.
     * After calling, the commanded values can be retrieved with {@link #getPowerCommand()}
     * and {@link #getSteeringCommand()}
     */
    public void updateSteeringMode(double heading, double xpos, double ypos){
        if (mRotationModeActive){
            // Cancel rotation mode and reset both PIDs
            mRotationModeActive = false;
            mSteeringPID.reset();
            mPowerPID.reset();
        }
        mLastPosX = xpos;
        mLastPosY = ypos;
        mLastHeading = heading;
        // Compute the angle to the target relative to the current position
        double angleToTarget = getHeadingToTargetDeltaAngle();

        // And compute the distance
        double distance = Math.sqrt(Math.pow(mTargetPX-xpos,2.0d)+Math.pow(mTargetPY-ypos,2.0d));
        // Power command is computed using the new distance to the target
        mPowerCommand = mPowerPID.getOutput(distance);
        // Steering  command is computed using angleToTarget as the new setpoint and
        // the current heading
        mSteeringCommand = mSteeringPID.getOutput(heading,angleToTarget);
    }

    /**
     * Returns delta angle between the robot heading and the angle to the target.
     * when pointed at the target, this function should return 0.
     */
    public double getHeadingToTargetDeltaAngle(){
        double angleToTarget = getAngleToTarget();
        angleToTarget = angleToTarget-mLastHeading;
        return angleToTarget;
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
    public void updateRotationMode(double heading,double xpos, double ypos){
        if (!mRotationModeActive){
            // Cancel rotation mode and reset both PIDs
            mRotationModeActive = true;
            mRotationCommand = 0d;
            mRotationPID.reset();
        }
        mLastPosX = xpos;
        mLastPosY = ypos;
        mLastHeading = heading;

        mRotationCommand = mRotationPID.getOutput(heading,getAngleToTarget());
    }

    public boolean isRotationModeActive(){
        return mRotationModeActive;
    }
    /**
     * utility computes the angle between the current robot position and the
     * target position
     */
    private double getAngleToTarget(){
        double xrel = mTargetPX-mLastPosX;
        double yrel = mTargetPY-mLastPosY;
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
                return Math.PI*3d/2d;
            }
        }
        // Otherwise compute angle from the inverse tangent
        invtan = Math.abs(Math.atan(Math.abs(xrel)/Math.abs(yrel)));
        if (xrel >= 0d){
            if (yrel >= 0d){
                // northeast quadrant is 0 to PI/2
                angleToTarget = invtan;
            }
            else{
                // southeast quadrant
                angleToTarget = invtan + Math.PI*2d;
            }
        }
        else{
            // xrel is negative
            if (yrel >= 0d){
                // northwest quadrant
                angleToTarget = Math.PI/2-invtan;
            }
            else{
                // southwest quadrant is PI to 3PI/2;
                angleToTarget = invtan + Math.PI;
            }

        }
        return angleToTarget;
    }
    /**
     * Returns the IMU calibration status string for development
     */
    public String getIMUCalibrationStatus(){
        if (mIMUInitialized){
            return mIMU.getCalibrationStatus().toString();
        }
        else{
            return "Error:  IMU not initialized";
        }
    }

    /** Returns the IMU for development ONLY
     * TODO:  Eliminate this function
     */
    public BNO055IMU getIMU(){
        return mIMU;
    }
    /**
     * returns the current steering command only valid in normal mode
     * @return current steering command +1 for left, -1 for right.
     */
    public double getSteeringCommand(){
        return mSteeringCommand;
    }
    /**
     * returns the current rotation command only valid in rotation mode
     * @return current steering command +1 for left, -1 for right.
     */
    public double getRotationCommand(){
        return mRotationCommand;
    }   /**
     * @return commanded power - range -1.0 (reverse) to 1.0 (forward)
     */
    public double getPowerCommand(){
        return mPowerCommand;
    }
}
