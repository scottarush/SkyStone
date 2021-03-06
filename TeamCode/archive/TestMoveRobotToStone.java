package org.firstinspires.ftc.teamcode.TestOpModes;/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.VuforiaMotionMethods;


/**
This class tests the move drivetrain to stone utility
  */

@TeleOp(name="TestMoveRobotToStone", group="Robot")
@Disabled
public class TestMoveRobotToStone extends LinearOpMode {

    /* Declare OpMode members. */
    private MecanumDrive drivetrain = null;

    private VuforiaMotionMethods utilities = null;

    public TestMoveRobotToStone() {

    }

    @Override
    public void runOpMode() {
        try {
            drivetrain = new MecanumDrive(this,true);
            drivetrain.init(hardwareMap);

            utilities = new VuforiaMotionMethods(drivetrain);
            // Initialize both motion utilites
            utilities.initFindStone();
            utilities.initMoveRobotToStone();

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
                // Now move the drivetrain to the stone.
                utilities.moveRobotToStone(this,5);
                break;
            case VuforiaMotionMethods.FOUND_SKYSTONE:
                telemetry.addData("Status", "Found Skystone");
                // Now move the drivetrain to the stone.
                utilities.moveRobotToStone(this,5);
                break;
        }

         telemetry.update();
        drivetrain.stop();
    }

}