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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


/**
This class implements the equations that Marcus derived on October 3.
  */

@TeleOp(name="Tank Drive", group="Robot")
//@Disabled
public class MecanumTankOpMode extends OpMode{

    /* Declare OpMode members. */
    private MecanumRobotHardware robot = null;

    /** current motor speeds. **/
    private double lfPower = 0;
    private double rfPower = 0;
    private double lrPower = 0;
    private double rrPower = 0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        try {
            robot = new MecanumRobotHardware();
            robot.init(hardwareMap);
        }
        catch(Exception e){
            telemetry.addData("Robot Init Error","%s",e.getMessage());
            telemetry.update();
            return;
        }

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Init Complete");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
     }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
         // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        double xleft = gamepad1.left_stick_x;
        double yleft = -gamepad1.left_stick_y;
        double xright = gamepad1.right_stick_x;
        double yright = -gamepad1.right_stick_y;

        // the speeds with the new gamepad inputs
        computeMotorPower(xleft,yleft,xright,yright);

        // Set the motor power to the speeds
        robot.setDriveMotorPower(lfPower, rfPower, lrPower, rrPower);
        telemetry.addData("Gamepad:","xl=%.2f yl=%.2f xr=%.2f yr=%.2f",
            xleft,yleft,xright,yright);
        // log the speeds to telemetry
        telemetry.addData("Wheel speeds:", "lf=%.2f rf=%.2f lr=%.2f rr=%.2f",
                lfPower, rfPower, lrPower, rrPower);
        telemetry.addData("Encoder positions","lf=%d rf=%d lr=%d rr=%d",
                robot.lfMotor.getCurrentPosition(),
                robot.lrMotor.getCurrentPosition(),
                robot.rfMotor.getCurrentPosition(),
                robot.rrMotor.getCurrentPosition());

         telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        lfPower = 0;
        rfPower = 0;
        lrPower = 0;
        rrPower = 0;

        robot.stopAll();
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
    private void computeMotorPower(double xleft, double yleft, double xright, double yright) {

        lfPower = yleft+(xleft+xright)/2;
        rfPower = yright - (xleft+xright)/2;
        lrPower = yleft-(xleft + xright)/2;
        rrPower = yright + (xleft+xright)/2;

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
    }

 }