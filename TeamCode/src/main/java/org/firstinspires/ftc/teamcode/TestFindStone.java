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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drivetrain.MecanumDrive;


/**
This class tests the findStone utility.
  */

@TeleOp(name="TestFindStone", group="Robot")
//@Disabled
public class TestFindStone extends LinearOpMode {

    /* Declare OpMode members. */
    private MecanumDrive drivetrain = null;

    private VuforiaMotionMethods utilities = null;

    public TestFindStone() {

    }

    @Override
    public void runOpMode() {
        try {
            drivetrain = new MecanumDrive(this);
            drivetrain.init(hardwareMap);

            utilities = new VuforiaMotionMethods(drivetrain);
            // Initialie the utility method
            utilities.initFindStone();

        } catch (Exception e) {
            telemetry.addData("Robot Init Error", "%s", e.getMessage());
            return;
        }
        telemetry.addData("Say","Robot Initialized");
        waitForStart();

        int retcode = utilities.findStone(this, false, true,10);
        switch(retcode) {
            case VuforiaMotionMethods.NO_STONES_FOUND:
                telemetry.addData("Status", "No Stones Found");
                break;
            case VuforiaMotionMethods.FOUND_STONE:
                telemetry.addData("Status", "Found Stone");
                break;
            case VuforiaMotionMethods.FOUND_SKYSTONE:
                telemetry.addData("Status", "Found Skystone");
                break;
        }

         telemetry.update();
        drivetrain.stop();
    }

}