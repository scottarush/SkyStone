package org.firstinspires.ftc.teamcode.drivetrain;

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

    private Parameters mPIDParameters = null;

    private double mCommandedPower = 0d;
    private double mCommandedSteeringAngle = 0d;

    private double mTargetPX = 0d;
    private double mTargetPY = 0d;

    public static class Parameters {
        public String imuCalibrationDataFilename = "IMUCal.json";
        public double steeringPropGain = 0d;
        public double steeringIntegGain = 0d;
        public double steeringDerivGain = 0d;

        public double powerPropGain = 0d;
        public double powerIntegGain = 0d;
        public double powerDerivGain = 0d;
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

                String filename = "IMUCalCtrlHubOne.json";
                File file = AppUtil.getInstance().getSettingsFile(filename);
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
     * updates the PID controllers with the new current heading and x,y position from
     * the Kalman filter.
     * After calling, the commanded values can be retrieved with getCommandedPower
     * and getCommandedSteeringAngle
     */
    public void update(double heading, double xpos, double ypos){
        // Compute the distance vector and angle to target
        double xrel = mTargetPX-xpos;
        double yrel = mTargetPY-ypos;
        double angleToTarget = Math.atan(xrel/yrel);
        double distance = Math.sqrt(Math.pow(xrel,2.0d)+Math.pow(yrel,2.0d));
        // Power command is computed using the new distance to the target
        mCommandedPower = mPowerPID.getOutput(distance);
        // Steering angle command is computed using angleToTarget as the new setpoint and
        // the current heading
        mCommandedSteeringAngle = mSteeringPID.getOutput(heading,angleToTarget);
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
     * returns the current steering command
     * @return current commanded steering angle - range +/- PI/2
     */
    public double getCommandedSteeringAngle(){
        return mCommandedSteeringAngle;
    }
    /**
     * returns the current commanded power
     * @return commanded power - range -1.0 (reverse) to 1.0 (forward)
     */
    public double getCommandedPower(){
        return mCommandedPower;
    }
}
