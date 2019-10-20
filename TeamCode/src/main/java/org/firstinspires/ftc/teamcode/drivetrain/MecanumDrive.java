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

import org.firstinspires.ftc.robotcore.external.Telemetry;

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
        lfMotor = tryMapMotor("left_front_drive");
        rfMotor = tryMapMotor("right_front_drive");
        lrMotor = tryMapMotor("left_rear_drive");
        rrMotor = tryMapMotor("right_rear_drive");

        // Left side motors are reversed
        lfMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        lrMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set all motors to zero power
        setPower(0, 0, 0, 0);

        // Set all motors to run with encoders.
        setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);

        // TODO Define and initialize ALL installed servos once the actuators are defined by the build team

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
        lfMotor.setPower(lf);
        rfMotor.setPower(rf);
        lrMotor.setPower(lr);
        rrMotor.setPower(rr);
    }

    /**
     * Helper function sets all motor modes to the same mode
     *
     * @param mode
     */
    @Override
    public void setMotorModes(DcMotor.RunMode mode) {
        lfMotor.setMode(mode);
        rfMotor.setMode(mode);
        lrMotor.setMode(mode);
        rrMotor.setMode(mode);
    }


    /**
     * helper function to stop all motors on the robot.
     */
    @Override
    public void stop() {
        setPower(0.0, 0.0, 0.0, 0.0);
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
        lfMotor.setTargetPosition(lfDeltaCounts+lfMotor.getCurrentPosition());
        rfMotor.setTargetPosition(lfDeltaCounts+rfMotor.getCurrentPosition());
        lrMotor.setTargetPosition(lfDeltaCounts+lrMotor.getCurrentPosition());
        rrMotor.setTargetPosition(lfDeltaCounts+rrMotor.getCurrentPosition());

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
                (lfMotor.isBusy() || rfMotor.isBusy() || lrMotor.isBusy() || rrMotor.isBusy())) {

            myOpMode.telemetry.addData("TargetPositions", "lf:%7d rf:%7d lr:%7d rr:%7d",
                    lfMotor.getTargetPosition(),
                    rfMotor.getTargetPosition(),
                    lrMotor.getTargetPosition(),
                    rrMotor.getTargetPosition());
            opMode.telemetry.addData("Positions:", "lf:%7d rf:%7d lr:%7d rr:%7d",
                    lfMotor.getCurrentPosition(),
                    rfMotor.getCurrentPosition(),
                    lrMotor.getCurrentPosition(),
                    rrMotor.getCurrentPosition());
            opMode.telemetry.update();
        }
        // Stop all
        stop();
    }

}