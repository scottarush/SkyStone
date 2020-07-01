/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.drivetrain;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;

/**
 * This is NOT an opmode.
 *
 * This is the base class for a Mecanum Drive robot.  Because the motor direction configurations
 * must be different between the GrabberBot and the development frame bot, this class has
 * been made with an abstract initIMU method.  The rest of the behavior is the same between the
 * subclasses.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lowerManualRamp case and some have single spaces between words.
 *
 * Motor channel:  Left front drive motor:    "lf"
 * Motor channel:  Right front drive motor:  "rf"
 * Motor channel:  Left rear drive motor:    "lr"
 * Motor channel:  Right rear drive motor:   "r"
 *
 */
public abstract class BaseMecanumDrive extends Drivetrain{

    protected DcMotor mLFMotor = null;
    protected DcMotor mRFMotor = null;
    protected DcMotor mLRMotor = null;
    protected DcMotor mRRMotor = null;

    private ArrayList<DcMotor> mMotorList = new ArrayList<>();

    /**
     * Index of LF wheel in returned wheel speed array.
     */
    public static int LF_WHEEL_ARRAY_INDEX = 0;
    /**
     * Index of RF wheel in returned wheel speed array.
     */
    public static int RF_WHEEL_ARRAY_INDEX = 1;
    /**
     * Index of LR wheel in returned wheel speed array.
     */
    public static int LR_WHEEL_ARRAY_INDEX = 2;
    /**
     * Index of RR wheel in returned wheel speed array.
     */
    public static int RR_WHEEL_ARRAY_INDEX = 3;

    private int mMotorPositions[] = new int[4];
    private double mWheelSpeeds[] = new double[4];


    private boolean mFirstLoopInit = false;

    private long mLastLoopTime = 0;
    /**
     * Wheel circumference in inches
     **/
    public static final double MECANUM_WHEEL_CIRCUMFERENCE = 12.1211;

    /**
     * scale factor for strafe lateral distance.
     */
    public static final double STRAFE_ENCODER_DISTANCE_COEFFICIENT = 1.08d;


    private boolean mWasLastMovementStrafe = false;

    /**
     * @param
     **/
    public BaseMecanumDrive(OpMode opMode,IMU imu) {
        super(opMode,imu);
    }

    /**
     * subclasses must implement to return the number of encoder counts per inch of rotation
     */
    protected abstract double getEncoderCountsPerInchRotation();
    /**
     * Must be implemented by subclasses to provide the encoder counts threshold constant
     * for finishing an encoder drive.
     */
    protected abstract int getEncoderDriveCountsMinThreshold();

    /**
     * Must be implemented by subclasses to provide the number of counts per wheel revolution
     */
    protected abstract int getEncoderCountsPerRev();

    /**
     * Overridden to compute wheel speeds.  Must call base class function too.
     */
    @Override
    public void loop() {
        super.loop();
        long newtime = System.nanoTime();
        if (!mFirstLoopInit) {
            mLastLoopTime = newtime;
            computeWheelSpeeds(0d);
            mFirstLoopInit = true;
            return;
        }

        double deltat = (mLastLoopTime-newtime)*1e-9d;
        computeWheelSpeeds(deltat);
        // Compute delta t since last computation
        mLastLoopTime = newtime;  // save for next time

    }

    /**
     * helper to compute wheel speeds called from loop
     */
    private void computeWheelSpeeds(double deltaT){
        if (!mFirstLoopInit){
            // This is the first call so initialize everything
            mMotorList.add(mLFMotor);
            mMotorList.add(mRFMotor);
            mMotorList.add(mLRMotor);
            mMotorList.add(mRRMotor);

            // so just pull the positions from the motors
            for(int i=0;i < mMotorList.size();i++){
                mMotorPositions[i] = getCurrentPosition(mMotorList.get(i));
                mWheelSpeeds[i] = 0d;
            }
            return;
        }
        // Otherwise compute the angular velocities
        for(int i=0;i < mMotorList.size();i++){
            int newpos = getCurrentPosition(mMotorList.get(i));
            double angle = (newpos - mMotorPositions[i])/ getEncoderCountsPerRev() * 2*Math.PI;
            mWheelSpeeds[i] = angle /deltaT;
            mMotorPositions[i] = newpos;  // Transfer to current pos array for next time
        }
    }

    /**
     * returns array list of wheel angular speeds in radians/sec
     * array order is:  {LF,RF,LR,RR}
     */
    public double[] getWheelSpeeds(){
        double speeds[] = new double[4];
        for(int i=0;i < mWheelSpeeds.length;i++){
            speeds[i] = mWheelSpeeds[i];
        }
        return speeds;
    }

    /**
     * private helper function for set target position.  Does nothing if motor pointer is
     * null to allow fail-op operation.
     */
    private void setTargetPosition(DcMotor motor,int position){
        if (motor != null){
            motor.setTargetPosition(position);
        }
    }

    /**
     * private helper function for get current position.  Returns 0 if
     * null to allow fail-op operation.
     */
    private int getCurrentPosition(DcMotor motor){
        if (motor != null){
            return motor.getCurrentPosition();
        }
        return 0;
    }

    /**
     * helper function to stop all motors on the robot and return motor modes to
     * manual mode if they had been changed to run to position.
     * Make sure to call base class.
     */
    @Override
    public void stop() {
        super.stop();
        setPower(0.0, 0.0, 0.0, 0.0);
        // Return motors to manual control
        setMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
    /**
     * strafes sideways by time
     *
     * @param speed   speed to move
     * @param strafeDistance distance in inches. Right is positive, left is negative
     * @param timeoutms timeout in msseconds to abort if move not completed.
     *
     * @return true if session started, false on error.
     */
    public void strafeByTime(double speed, double strafeDistance, int timeoutms) {
        if (isDriveByEncoderSessionActive()){
            stop();
        }
        super.driveEncoder(speed,strafeDistance,timeoutms);
        mWasLastMovementStrafe = true;
        // Stop and reset the encoders first
        setMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Compute the number of encoder counts for each wheel to move the requested distance
        double scaledDistance = strafeDistance * STRAFE_ENCODER_DISTANCE_COEFFICIENT;
        int lfDeltaCounts = (int) Math.round(scaledDistance * getEncoderCountsPerInchRotation());
        int lrDeltaCounts = (int) Math.round(-scaledDistance * getEncoderCountsPerInchRotation());
        int rfDeltaCounts = (int) Math.round(-scaledDistance * getEncoderCountsPerInchRotation());
        int rrDeltaCounts = (int) Math.round(scaledDistance * getEncoderCountsPerInchRotation());

        // Set target counts for each motor to the above
        setTargetPosition(mLFMotor,lfDeltaCounts+getCurrentPosition(mLFMotor));
        setTargetPosition(mRFMotor,rfDeltaCounts+getCurrentPosition(mRFMotor));
        setTargetPosition(mLRMotor,lrDeltaCounts+getCurrentPosition(mLRMotor));
        setTargetPosition(mRRMotor,rrDeltaCounts+getCurrentPosition(mRRMotor));
        //       mOpMode.telemetry.addData("counts","lf="+lfDeltaCounts+",rf="+rfDeltaCounts+",lr="+lrDeltaCounts+",rr="+rrDeltaCounts);
        //     mOpMode.telemetry.update();
        // Set mode to run to position
        setMotorModes(DcMotor.RunMode.RUN_TO_POSITION);

        // set motor power
        double aspeed = Math.abs(speed);
        if (aspeed > 1.0)
            aspeed = 1.0;
        setPower(aspeed, aspeed, aspeed, aspeed);
    }

    /**
     * strafes sideways by encoder
     *
     * @param speed   speed to move
     * @param strafeDistance distance in inches. Right is positive, left is negative
     * @param timeoutms timeout in msseconds to abort if move not completed.
     *
     * @return true if session started, false on error.
     */
    public void strafeEncoder(double speed, double strafeDistance, int timeoutms) {
         super.driveEncoder(speed,strafeDistance,timeoutms);
        mWasLastMovementStrafe = true;
        // Stop and reset the encoders first
        setMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Run without encoders to go as fast as possible.
        setMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

         // Pad the distance with the min threshold distance.  This a hack to get around the
        // stall problem when using encoders .
        double scaledDistance = strafeDistance * STRAFE_ENCODER_DISTANCE_COEFFICIENT;
        double counts = scaledDistance * getEncoderCountsPerInchRotation();
        double paddedCounts = Math.abs(counts)+ getEncoderDriveCountsMinThreshold();
        paddedCounts = paddedCounts * Math.signum(counts);

        // Set the number of encoder counts for each wheel to move the requested distance
        int lfDeltaCounts = (int) Math.round(paddedCounts);
        int lrDeltaCounts = (int) Math.round(-paddedCounts);
        int rfDeltaCounts = (int) Math.round(-paddedCounts);
        int rrDeltaCounts = (int) Math.round(paddedCounts);

        // Set target counts for each motor to the above
        setTargetPosition(mLFMotor,lfDeltaCounts+getCurrentPosition(mLFMotor));
        setTargetPosition(mRFMotor,rfDeltaCounts+getCurrentPosition(mRFMotor));
        setTargetPosition(mLRMotor,lrDeltaCounts+getCurrentPosition(mLRMotor));
        setTargetPosition(mRRMotor,rrDeltaCounts+getCurrentPosition(mRRMotor));

        //       mOpMode.telemetry.addData("counts","lf="+lfDeltaCounts+",rf="+rfDeltaCounts+",lr="+lrDeltaCounts+",rr="+rrDeltaCounts);
        //     mOpMode.telemetry.update();
        // Set mode to run to position
        setMotorModes(DcMotor.RunMode.RUN_TO_POSITION);

        // set motor power
        double aspeed = Math.abs(speed);
        if (aspeed > 1.0)
            aspeed = 1.0;
        setPower(aspeed, aspeed, aspeed, aspeed);
    }

    /**
     * starts a drive by encoder session.  If robot is moving, it will be stopped.
     *
     * @param speed   speed to move
     * @param linearDistance distance in inches
     * @param timeoutms timeout in msseconds to abort if move not completed.
     *
     * @return true if session started, false on error.
     */
    @Override
    public void driveEncoder(double speed, double linearDistance, int timeoutms) {
        super.driveEncoder(speed,linearDistance,timeoutms);
        mWasLastMovementStrafe = false;
        // Stop and reset the encoders first
        setMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Run with encoders to try to stay straight
        setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);

        // Pad the distance with the min threshold distance.  This a hack to get around the
        // stall problem when using encoders .
        double counts = linearDistance * getEncoderCountsPerInchRotation();
        double paddedCounts = Math.abs(counts)+ getEncoderDriveCountsMinThreshold();
        paddedCounts = paddedCounts * Math.signum(counts);
        // Compute the number of encoder counts for each wheel to move the requested distanc
        int lfDeltaCounts = (int) Math.round(paddedCounts);
        int rfDeltaCounts = (int) Math.round(paddedCounts);
        int lrDeltaCounts = (int) Math.round(paddedCounts);
        int rrDeltaCounts = (int) Math.round(paddedCounts);

        // Set target counts for each motor to the above
        setTargetPosition(mLFMotor,lfDeltaCounts+getCurrentPosition(mLFMotor));
        setTargetPosition(mRFMotor,rfDeltaCounts+getCurrentPosition(mRFMotor));
        setTargetPosition(mLRMotor,lrDeltaCounts+getCurrentPosition(mLRMotor));
        setTargetPosition(mRRMotor,rrDeltaCounts+getCurrentPosition(mRRMotor));

 //       mOpMode.telemetry.addData("counts","lf="+lfDeltaCounts+",rf="+rfDeltaCounts+",lr="+lrDeltaCounts+",rr="+rrDeltaCounts);
 //       mOpMode.telemetry.update();
        // Set mode to run to position
        setMotorModes(DcMotor.RunMode.RUN_TO_POSITION);

        // set motor power as absolute value of speed
        double aspeed = Math.abs(speed);
        if (aspeed > 1.0)
            aspeed = 1.0;
        setPower(aspeed, aspeed, aspeed, aspeed);
 //       mOpMode.telemetry.addData("driveEncoder:","lf=%d aspeeed=%1.1f",lfDeltaCounts,aspeed);
 //       mOpMode.telemetry.update();
    }

    @Override
    protected boolean isTargetPositionReached() {
        if (!isDriveByEncoderSessionActive()){
            return true;
        }
        if (mLFMotor != null){
            int delta = Math.abs(mLFMotor.getTargetPosition() - mLFMotor.getCurrentPosition());
            if (delta > getEncoderDriveCountsMinThreshold()){
                return false;
            }
        }
        if (mLRMotor != null){
            int delta = Math.abs(mLRMotor.getTargetPosition() - mLRMotor.getCurrentPosition());
            if (delta > getEncoderDriveCountsMinThreshold()){
                return false;
            }
        }
        if (mRFMotor != null){
            int delta = Math.abs(mRFMotor.getTargetPosition() - mRFMotor.getCurrentPosition());
            if (delta > getEncoderDriveCountsMinThreshold()){
                return false;
            }
        }
        if (mRRMotor != null){
            int delta = Math.abs(mRRMotor.getTargetPosition() - mRRMotor.getCurrentPosition());
            if (delta > getEncoderDriveCountsMinThreshold()){
                return false;
            }
        }
        return true;
    }

    @Override
    protected double getActualEncoderDistance() {
        if (mWasLastMovementStrafe){
            double avg = (getStrafeMovement(mLFMotor) +
                        getStrafeMovement(mLRMotor)+
                        getStrafeMovement(mRRMotor)+
                        getStrafeMovement(mRFMotor))/4;
            return avg;
        }
        else{
            double avg = (getLinearMovement(mLFMotor) +
                    getLinearMovement(mLRMotor)+
                    getLinearMovement(mRRMotor)+
                    getLinearMovement(mRFMotor))/4;
            return avg;
        }
    }

    /**
     * helper function to compute average distance in getActualEncoderDistance
     */
    private double getStrafeMovement(DcMotor motor){
        if (motor != null) {
            int counts = motor.getCurrentPosition();
            double distance = counts / getEncoderCountsPerInchRotation();
            distance = distance * STRAFE_ENCODER_DISTANCE_COEFFICIENT;
            return distance;
        }
        return 0d;
    }
    /**
     * helper function to compute average distance in getActualEncoderDistance
     */
    private double getLinearMovement(DcMotor motor){
        if (motor != null) {
            int counts = motor.getCurrentPosition();
            double distance = counts / getEncoderCountsPerInchRotation();
            return distance;
        }
        return 0d;
    }

//    public boolean isMoving() {
//        if (lfMotor != null){
//            if (lfMotor.isBusy())
//                return true;
//        }
//        if (rfMotor != null){
//            if (rfMotor.isBusy())
//                return true;
//        }
//        if (lrMotor != null){
//            if (lrMotor.isBusy())
//                return true;
//        }
//        if (rrMotor != null){
//            if (rrMotor.isBusy())
//                return true;
//        }
//        return false;
//    }

    /**
     * This is a helper function that takes input from a dual joy stick and computes the speed
     * of each Mecanum wheel motor.
     *
     * positive x is to the right
     * positive y is up
     *
     * @param xleft x coordinate of left stick
     * @param yleft y coordinate of left stick
     * @param xright x coordinate of right stick
     * @param yright y coordinate of right stick
     *
     **/
    public void setTankDriveJoystickInput(double xleft, double yleft, double xright, double yright) {

        double lfPower = yleft+(xleft+xright)/2;
        double rfPower = yright - (xleft+xright)/2;
        double lrPower = yleft-(xleft + xright)/2;
        double rrPower = yright + (xleft+xright)/2;

        /**
         * Now normalize the wheel speed commands:
         * Let speedmax be the maximum absolute value of the four wheel speed commands.
         * If speedmax is greater than 1, then divide each of the four wheel speed commands by speedmax.
         **/
        double speedmax = Math.abs(lfPower + rfPower + lrPower + rrPower);
        if (speedmax > 4.0){
            lfPower = lfPower /speedmax;
            rfPower = rfPower / speedmax;
            lrPower = lrPower / speedmax;
            rrPower = rrPower / speedmax;
        }
//        mOpMode.telemetry.addData("power","lf %1.2f rf %1.2f lr %1.2f rr %1.2f",lfPower,rfPower,lrPower,rrPower);
//        mOpMode.telemetry.update();

        setPower(lfPower,rfPower,lrPower,rrPower);
    }

    /**
     * Drives for a set distance using open-loop, robot-specific time.
     * @param linearDistance desired distance + forward or - rearward in inches
     * @return time to drive in ms
     */
    @Override
    public int driveLinearTime(double linearDistance, double power){
       int timeout = super.driveLinearTime(linearDistance,power);  // Call base class for timer handling

        setPower(power,power,power,power);
        return timeout;
    }

    @Override
    public void correctHeading(double correction) {
        double power = mLinearDrivePower;

        double lfPower = power - correction;
        double lrPower = power + correction;
        double rfPower = power + correction;
        double rrPower = power - correction;
        setPower(lfPower,rfPower,lrPower,rrPower);
    }


    /**
     * Overridden function calls base class to pass correction value into motors.
     */
    @Override
    protected double checkRotation() {
        // Check if a rotation is active and return if not
        if (!isRotationActive()){
            return 0d;
        }
        // disables PID control in case it was set before.
 //       setMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double rotpower = super.checkRotation();
        if (Math.abs(rotpower) > 1.0d) {
            rotpower = Math.signum(rotpower);
        }

        double lfPower = -rotpower;
        double rfPower = +rotpower;
        double lrPower = -rotpower;
        double rrPower = +rotpower;
        setPower(lfPower,rfPower,lrPower,rrPower);
        return rotpower;
    }

    /**
     * Helper function to set power to the wheel drive motors
     *
     * @param lf left front motor power
     * @param rf right front motor power
     * @param lr left rear motor power
     * @param rr right rear motor power
     */
    public void setPower(double lf, double rf, double lr, double rr) {
        if (mLFMotor != null)
            mLFMotor.setPower(lf);
        if (mRFMotor != null)
            mRFMotor.setPower(rf);
        if (mLRMotor != null)
        mLRMotor.setPower(lr);
        if (mRRMotor != null)
            mRRMotor.setPower(rr);
    }

    /**
     * Helper function sets all motor modes to the same mode
     *
     * @param mode
     */
    public void setMotorModes(DcMotor.RunMode mode) {
        if (mLFMotor != null)
            mLFMotor.setMode(mode);
        if (mRFMotor != null)
            mRFMotor.setMode(mode);
        if (mLRMotor != null)
            mLRMotor.setMode(mode);
        if (mRRMotor != null)
            mRRMotor.setMode(mode);
    }


}