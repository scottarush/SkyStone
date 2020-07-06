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

    private MiniPID mHeadingPID = null;
    private MiniPID mPositionPID = null;

    private Parameters mPIDParameters = null;

    private double mCommandedPower = 0d;
    private double mSteeringAngle = 0d;

    private double mCurrentPX = 0d;
    private double mCurrentPY = 0d;
    private double mCurrentHeading = 0d;

    private double mTargetPX = 0d;
    private double mTargetPY = 0d;

    public static class Parameters {
        public String imuCalibrationDataFilename = "IMUCal.json";
        public double headingPropGain = 0d;
        public double headingIntegGain = 0d;
        public double headingDerivGain = 0d;
        public double positionPropGain = 0d;
        public double positionIntegGain = 0d;
        public double positionDerivGain = 0d;
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

                String filename = "IMUCal_ctrlhub1.json";
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
        mHeadingPID = new MiniPID(pidParams.headingPropGain,pidParams.headingIntegGain,pidParams.headingDerivGain);
        mPositionPID = new MiniPID(pidParams.positionPropGain,pidParams.positionIntegGain,pidParams.positionDerivGain);

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
     * sets the current heading and position to the guidance controller for feedback controller
     */
    public void update(double heading, double xpos, double ypos){
        mCurrentHeading = heading;
        mCurrentPX = xpos;
        mCurrentPY = ypos;

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
    public double getSteeringAngle(){
        return 0d;
    }
    /**
     * returns the current commanded power
     * @return command forward and reverse power - range +/- 1.0
     */
    public double getCommandedPower(){
        return 0d;
    }
}
