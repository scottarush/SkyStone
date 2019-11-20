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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This is NOT an opmode.
 *
 * This class define's specific hardware for our team's Mecanum robot.  It was created starting
 * from the HardwareRobot class within the samples directory.
 *
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot"
 * in the samples for help in creating usage.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left front drive motor:        "left_front_drive"
 * Motor channel:  Right front drive motor:  "right_front_drive"
 * Motor channel:  Left rear drive motor:    "left_rear_drive"
 * Motor channel:  Right rear drive motor:   "right_rear_drive"
 *
 */
public class MecanumDrive extends Drivetrain{
    /* Public OpMode members. */
    private DcMotor lfMotor = null;
    private DcMotor rfMotor = null;
    private DcMotor lrMotor = null;
    private DcMotor rrMotor = null;

    /**
     * Wheel circumference in inches
     **/
    public static final double MECANUM_WHEEL_CIRCUMFERENCE = 12.1211;


    public static final int ENCODER_COUNTS_PER_ROTATION = 1120;  // This is the Rev robotics motor.
    /**
     * Number of counts per inch of direct wheel movement.
     **/
    public static final int COUNTS_PER_INCH = (int) Math.round(ENCODER_COUNTS_PER_ROTATION / MECANUM_WHEEL_CIRCUMFERENCE);

    /**
     * Approximate number of milliseconds per inch forward and rearward at full power.
     */
    public static final int LINEAR_MILLISECONDS_PER_INCH = 50;
    /**
     * Approximate number of milliseconds per inch inch direction for vector drive
     */
    public static final int VECTOR_XMILLISECONDS_PER_INCH = 50;
    /**
     * Approximate number of milliseconds per inch in Y direction for vector drive
     */
    public static final int VECTOR_YMILLISECONDS_PER_INCH = 50;


    private boolean mDemoFrameBot = false;
    /**
     * @param
     * @param demoFrameBot true if using demo, false for grabber bot
     **/
    public MecanumDrive(OpMode opMode,boolean demoFrameBot) {
        super(opMode,LINEAR_MILLISECONDS_PER_INCH);
        mDemoFrameBot = demoFrameBot;
    }


    /* Initialize standard Hardware interfaces.
     * NOTE:  This class throws Exception on any hardware init error so be sure to catch and
     * report to Telemetry in your initialization. */
    @Override
    public void init(HardwareMap ahwMap) throws Exception {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        String motorInitError = "";
        try {
            lfMotor = tryMapMotor("lf");
            if (mDemoFrameBot){
                lfMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            }
        }
        catch (Exception e){
            motorInitError += "lf,";
        }
        try {
            rfMotor = tryMapMotor("rf");
        }
        catch(Exception e){
            motorInitError += "rf,";
        }
        try {
            lrMotor = tryMapMotor("lr");
            lrMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        catch(Exception e){
            motorInitError += "lr,";
        }
        try {
            rrMotor = tryMapMotor("rr");
            if (mDemoFrameBot){
                rrMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            }
            else{
                rrMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            }
        }
        catch(Exception e){
            motorInitError += "rr,";
        }


        // Set all motors to zero power
        setPower(0, 0, 0, 0);

        // Set all motors to run with encoders.
        setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);

        if (motorInitError.length() > 0){
            throw new Exception("Motor init errs: '"+motorInitError+"'");
        }

    }


    /**
     * Helper function to set power to the wheel drive motors
     *
     * @param lf left front motor power
     * @param rf right front motor power
     * @param lr left rear motor power
     * @param rr right rear motor power
     */
    @Override
    public void setPower(double lf, double rf, double lr, double rr) {
        if (lfMotor != null)
            lfMotor.setPower(lf);
        if (rfMotor != null)
            rfMotor.setPower(rf);
        if (lrMotor != null)
            lrMotor.setPower(lr);
        if (rrMotor != null)
            rrMotor.setPower(rr);
    }

    /**
     * Helper function sets all motor modes to the same mode
     *
     * @param mode
     */
    @Override
    public void setMotorModes(DcMotor.RunMode mode) {
        if (lfMotor != null)
            lfMotor.setMode(mode);
        if (rfMotor != null)
            rfMotor.setMode(mode);
        if (lrMotor != null)
            lrMotor.setMode(mode);
        if (rrMotor != null)
            rrMotor.setMode(mode);
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
     * private helper function for get target position.  Returns 0 if
     * null to allow fail-op operation.
     */
    private int getTargetPosition(DcMotor motor){
        if (motor != null){
            return motor.getTargetPosition();
        }
        return 0;
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
     */
    @Override
    public void stop() {
        super.stop();
        setPower(0.0, 0.0, 0.0, 0.0);
        // Return motors to manual control
        setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);

    }


    /**
     * starts a drive by encoder session.  If robot is moving, it will be stopped.
     *
     * @param speed   speed to move
     * @param xdist   distance in x inches to move where +x is to the robot's right.
     * @param ydist   distance in y inches to move where +y is forward.
     * @param timeoutms timeout in msseconds to abort if move not completed.
     *
     * @return true if session started, false on error.
     */
    @Override
    public void encoderDrive(double speed, double xdist, double ydist, int timeoutms) {
        if (isDriveByEncoderSessionActive()){
            return;
        }
        super.encoderDrive(speed,xdist,ydist,timeoutms);
        // Stop and reset the encoders first
        setMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);

        // Compute the number of encoder counts for each wheel to move the requested distanc
        int lfDeltaCounts = (int) Math.round((ydist + xdist) * MecanumDrive.COUNTS_PER_INCH);
        int rfDeltaCounts = (int) Math.round((ydist - xdist) * MecanumDrive.COUNTS_PER_INCH);
        int lrDeltaCounts = (int) Math.round((ydist - xdist) * MecanumDrive.COUNTS_PER_INCH);
        int rrDeltaCounts = (int) Math.round((ydist + ydist) * MecanumDrive.COUNTS_PER_INCH);

        // Set target counts for each motor to the above
        setTargetPosition(lfMotor,lfDeltaCounts+getCurrentPosition(lfMotor));
        setTargetPosition(rfMotor,rfDeltaCounts+getCurrentPosition(rfMotor));
        setTargetPosition(lrMotor,lrDeltaCounts+getCurrentPosition(lrMotor));
        setTargetPosition(rrMotor,rrDeltaCounts+getCurrentPosition(rrMotor));

        mOpMode.telemetry.addData("counts","lf="+lfDeltaCounts+",rf="+rfDeltaCounts+",lr="+lrDeltaCounts+",rr="+rrDeltaCounts);
        mOpMode.telemetry.update();
        // Set mode to run to position
        setMotorModes(DcMotor.RunMode.RUN_TO_POSITION);

        // set motor power
        double aspeed = Math.abs(speed);
        if (aspeed > 1.0)
            aspeed = 1.0;
        setPower(aspeed, aspeed, aspeed, aspeed);
    }

    @Override
    public boolean isMoving() {
        if (lfMotor != null){
            if (lfMotor.isBusy())
                return true;
        }
        if (rfMotor != null){
            if (rfMotor.isBusy())
                return true;
        }
        if (lrMotor != null){
            if (lrMotor.isBusy())
                return true;
        }
        if (rrMotor != null){
            if (rrMotor.isBusy())
                return true;
        }
        return false;
    }

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

        setPower(lfPower,rfPower,lrPower,rrPower);
    }

    @Override
    public boolean isVectorTimedDriveSupported() {
        return true;
    }

    @Override
    public int getVectorTimedXMSPerInch() {
        return VECTOR_XMILLISECONDS_PER_INCH;
    }

    @Override
    public int getVectorTimedYMSPerInch() {
        return VECTOR_YMILLISECONDS_PER_INCH;
    }

    /**
     * Drives for a set distance using open-loop, robot-specific time.
     * @param linearDistance desired distance + forward or - rearward in inches
     * @return time to drive in ms
     */
    @Override
    public int doLinearTimedDrive(double linearDistance){
        int timeout = super.doLinearTimedDrive(linearDistance);  // Call base class for timer handling
        double power = 1.0d;
        if (linearDistance < 0.0d){
            power = -power;
        }

        setPower(power,power,power,power);
        return timeout;
    }

    @Override
    public int doVectorTimedDrive(double xdistance, double ydistance) {
        int timeout = super.doVectorTimedDrive(xdistance, ydistance);
        // TODO Need to figure out how to translate the vector into the power split
        // TODO Make it do a vector.  For now only use ydistance to know which direction
        double power = 1.0d;
        if (xdistance > 0d){
            power = -power;
        }
        double lfPower = power;
        double rfPower = -power;
        double lrPower = power;
        double rrPower = -power;
        setPower(lfPower,rfPower,lrPower,rrPower);
        return timeout;
    }

    /**
     * Overridden function calls base class to pass correction value into motors.
     */
    @Override
    public double checkRotation() {
        double rotpower = super.checkRotation();
        if (Math.abs(rotpower) > 1.0d) {
            rotpower = Math.signum(rotpower);
        }

        double lfPower = rotpower;
        double rfPower = -rotpower;
        double lrPower = rotpower;
        double rrPower = -rotpower;
        setPower(lfPower,rfPower,lrPower,rrPower);
        return rotpower;
    }
}