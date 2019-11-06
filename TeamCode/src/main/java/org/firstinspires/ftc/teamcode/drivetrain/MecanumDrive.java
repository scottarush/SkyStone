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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    /**
     * Number of counts per inch of direct wheel movement.
     **/
    public static final int COUNTS_PER_INCH = (int) Math.round(ENCODER_COUNTS_PER_ROTATION / MECANUM_WHEEL_CIRCUMFERENCE);

    /* Constructor */
    public MecanumDrive(OpMode opMode) {
        super(opMode);
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
             rrMotor.setDirection(DcMotorSimple.Direction.REVERSE);
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
     * helper function to stop all motors on the robot.
     */
    @Override
    public void stop() {
        setPower(0.0, 0.0, 0.0, 0.0);
    }

    /** helper function returns true if any motor is busy. **/
    private boolean isAnyMotorBusy(){
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
     * Utility function moves the robot a delta x and y distance using MecanumDrive
     *
     * @param speed   speed to move
     * @param xdist   distance in x inches to move
     * @param ydist   distance in y inches to move
     * @param timeout timeout in seconds to abort if move not completed.
     */
    @Override
    public void driveByEncoder(double speed, double xdist, double ydist, double timeout) {
        if (!(opMode instanceof  LinearOpMode)){
            opMode.telemetry.addData("Error:  Can only use driveByEncoder with a LinearOpMode",0);
            opMode.telemetry.update();
            return;
        }
        LinearOpMode myOpMode = (LinearOpMode)opMode;

        // Compute the number of encoder counts for each wheel to move the requested distanc
        int lfDeltaCounts = (int) Math.round((xdist + ydist) * MecanumDrive.COUNTS_PER_INCH);
        int rfDeltaCounts = (int) Math.round((ydist - xdist) * MecanumDrive.COUNTS_PER_INCH);
        int lrDeltaCounts = (int) Math.round((ydist - xdist) * MecanumDrive.COUNTS_PER_INCH);
        int rrDeltaCounts = (int) Math.round((xdist + ydist) * MecanumDrive.COUNTS_PER_INCH);

        // Set target counts for each motor to the above
        setTargetPosition(lfMotor,lfDeltaCounts+getCurrentPosition(lfMotor));
        setTargetPosition(rfMotor,lfDeltaCounts+getCurrentPosition(rfMotor));
        setTargetPosition(lrMotor,lfDeltaCounts+getCurrentPosition(lrMotor));
        setTargetPosition(rrMotor,lfDeltaCounts+getCurrentPosition(rrMotor));

        // Set mode to run to position
        setMotorModes(DcMotor.RunMode.RUN_TO_POSITION);

        // Reset timer and set motor power
        ElapsedTime drivetimeout = new ElapsedTime();
        drivetimeout.reset();
        double aspeed = Math.abs(speed);
        if (aspeed > 1.0)
            aspeed = 1.0;
        setPower(aspeed, aspeed, aspeed, aspeed);

        while (myOpMode.opModeIsActive() && (drivetimeout.seconds() < timeout) &&
                isAnyMotorBusy()) {

            myOpMode.telemetry.addData("TargetPositions", "lf:%7d rf:%7d lr:%7d rr:%7d",
                    getTargetPosition(lfMotor),
                    getTargetPosition(rfMotor),
                   getTargetPosition(lrMotor),
                    getTargetPosition(rrMotor));
            opMode.telemetry.addData("Positions:", "lf:%7d rf:%7d lr:%7d rr:%7d",
                    getCurrentPosition(lfMotor),
                    getCurrentPosition(rfMotor),
                    getCurrentPosition(lrMotor),
                    getCurrentPosition(rrMotor));
            opMode.telemetry.update();
        }
        // Stop all
        stop();
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

}