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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
public class MecanumRobotHardware {
    /* Public OpMode members. */
    public DcMotor lfMotor = null;
    public DcMotor rfMotor = null;
    public DcMotor lrMotor = null;
    public DcMotor rrMotor = null;

    /* local OpMode members. */
    public HardwareMap hwMap = null;


    /**
     * Number of encoder counts of each rotation of the shaft.
     **/
    public static final double ENCODER_COUNTS_PER_ROTATION = 1120;

    /**
     * Wheel circumference in inches
     **/
    public static final double MECANUM_WHEEL_CIRCUMFERENCE = 12.1211;

    /**
     * Number of counts per inch of direct wheel movement.
     **/
    public static final int COUNTS_PER_INCH = (int) Math.round(ENCODER_COUNTS_PER_ROTATION / MECANUM_WHEEL_CIRCUMFERENCE);


    /* Constructor */
    public MecanumRobotHardware() {

    }

    /* Initialize standard Hardware interfaces.
     * NOTE:  This class throws Exception on any hardware init error so be sure to catch and
     * report to Telemetry in your initialization. */
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
        setDriveMotorPower(0, 0, 0, 0);

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
    public void setDriveMotorPower(double lf, double rf, double lr, double rr) {
        lfMotor.setPower(lf);
        rfMotor.setPower(rf);
        lrMotor.setPower(lr);
        rrMotor.setPower(rr);
    }

    /**
     * helper function to stop all motors on the robot.
     */
    public void stopAll() {
        setDriveMotorPower(0, 0, 0, 0);
    }


    /**
     * Helper function sets all motor modes to the same mode
     *
     * @param mode
     */
    public void setMotorModes(DcMotor.RunMode mode) {
        lfMotor.setMode(mode);
        rfMotor.setMode(mode);
        lrMotor.setMode(mode);
        rrMotor.setMode(mode);
    }


    /**
     *  Utility function to handle motor initialization.
     */
    private DcMotor tryMapMotor(String motorName){
        DcMotor motor = null;
        try {
            motor = hwMap.get(DcMotor.class, motorName);
        }
        catch(Exception e){
            e.printStackTrace();
        }
        return motor;
    }
}