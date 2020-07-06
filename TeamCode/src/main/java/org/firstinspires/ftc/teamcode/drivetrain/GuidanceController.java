package org.firstinspires.ftc.teamcode.drivetrain;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
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

    private MiniPID mPID = null;
    private Parameters mPIDParameters = null;

    public static class Parameters {
        public String imuCalibrationDataFilename = "IMUCal.json";
        public double proportionalGain = 0d;
        public double integralGain = 0d;
        public double derivativeGain = 0d;
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
        //  Now initialize the pid
        mPID = new MiniPID(pidParams.proportionalGain,pidParams.integralGain,pidParams.derivativeGain);
    }
    public boolean isIMUInitialized(){
        return mIMUInitialized;
    }

    /**
     * helper function only for exposing IMU interface to Drivetrain legacy uses
     */
    public BNO055IMU getBNO055IMU(){
        return mIMU;
    }

    /**
     * sets the current heading and position feedback to the guidance controller
     */
    public void updateFeedback(double heading, double px, double py){

    }


}
