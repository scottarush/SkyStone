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

package org.firstinspires.ftc.teamcode.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drivetrain.BaseMecanumDrive;
import org.firstinspires.ftc.teamcode.drivetrain.GrabberBotMecanumDrive;
import org.firstinspires.ftc.teamcode.drivetrain.IDriveSessionStatusListener;
import org.firstinspires.ftc.teamcode.drivetrain.IRotationStatusListener;
import org.firstinspires.ftc.teamcode.drivetrain.SpeedBotMecanumDrive;


/**
This class implements the equations that Marcus derived on October 3.
  */

@TeleOp(name="TestIMURotation", group="Robot")
@Disabled
public class TestIMURotation extends OpMode implements IDriveSessionStatusListener, IRotationStatusListener {
    private static final int MIN_DELTA_UPDATE_TIME_MS = 100;

    private long mLastUpdateTime = 0L;

    /* Declare OpMode members. */
    private BaseMecanumDrive drivetrain = null;

    private static final boolean USE_GRABBER_BOT = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The initIMU() method of the hardware class does all the work here
         */
        try {
            if (USE_GRABBER_BOT){
                drivetrain = new GrabberBotMecanumDrive(this);
                drivetrain.init(hardwareMap);
            }
            else{
                drivetrain = new SpeedBotMecanumDrive(this,null);
                drivetrain.init(hardwareMap);
            }
         }
        catch(Exception e){
            telemetry.addData("Robot Init Error","%s",e.getMessage());
            telemetry.update();
            return;
        }
        drivetrain.addDriveSessionStatusListener(this);
        drivetrain.addRotationStatusListener(this);
        mLastUpdateTime = System.currentTimeMillis();
        // Send telemetry message to signify drivetrain waiting;
        telemetry.addData("Say", "Init Complete");
        telemetry.update();
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
        long delta = System.currentTimeMillis() - mLastUpdateTime;
        if (delta > MIN_DELTA_UPDATE_TIME_MS) {
            mLastUpdateTime = System.currentTimeMillis();
            if (gamepad1.a) {
                stop();
                drivetrain.rotate(90,1.0d,3000);
            }

            if (gamepad1.b) {
                stop();
                drivetrain.rotate(-90,1.0d,3000);
            }
            if (gamepad1.a) {
                stop();
                drivetrain.rotate(90,1.0d,3000);
            }

            if (gamepad1.x) {
                stop();
                drivetrain.rotate(-45,1.0d,3000);
            }
            if (gamepad1.y) {
                stop();
                drivetrain.rotate(45,1.0d,3000);
            }

            if (gamepad1.dpad_up) {
                drivetrain.driveEncoder(1.0d, 12.0d, 3000);

            }
            if (gamepad1.dpad_down) {
                drivetrain.driveEncoder(1.0d, -12.0d, 3000);

            }
            if (gamepad1.dpad_right) {
                drivetrain.strafeEncoder(1.0d, 12.0d, 1000);
            }
            if (gamepad1.dpad_left) {
                drivetrain.strafeEncoder(1.0d, -12.0d, 1000);
            }
        }
        drivetrain.loop();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
         drivetrain.stop();
    }

    @Override
    public void driveComplete(double distance) {
        int time = (int)System.currentTimeMillis();
        telemetry.addData("Status","drive complete: time=%d, distance=%3.1f",time,distance);
        telemetry.update();
    }

    @Override
    public void driveByEncoderTimeoutFailure(double distance) {
        int time = (int)System.currentTimeMillis();
        telemetry.addData("Status","drive timeout: time=%d, distance=%3.1f",time,distance);
        telemetry.update();
    }

    @Override
    public void rotationComplete(int angle) {
        int time = (int)System.currentTimeMillis();
        telemetry.addData("Status","rotation complete: time=%d, angle=%d",time,angle);
        telemetry.update();

    }

    @Override
    public void rotationTimeout(int angle) {
        int time = (int)System.currentTimeMillis();
        telemetry.addData("Status","rotation timeout: time=%d, angle=%d",time,angle);
        telemetry.update();

    }
}