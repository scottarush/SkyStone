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

package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.util.VuforiaCommon;
import org.firstinspires.ftc.teamcode.drivetrain.Drivetrain;

import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

/**
 * This class contains reusable methods for motion using Vuforia
 *
 * Opmodes use this class by creating an instance and passing a reference to the
 * drivetrain in the constructor
 *
 */
public class VuforiaMotionMethods {

    private OpenGLMatrix lastLocation = null;


    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // Distance of camera in front of drivetrain-center
    final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // Height of camera above ground
    final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the drivetrain's center line


    private Drivetrain drivetrain;

    /** reference to vuforia base.  **/
    private VuforiaCommon vuforiaCommon;

    /* Constructor */
    public VuforiaMotionMethods(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        vuforiaCommon = new VuforiaCommon(drivetrain.hwMap);
     }

    public static final int INITIALIZATION_ERROR = -1;
    public static final int NO_STONES_FOUND = 0;
    public static final int FOUND_SKYSTONE = 1;
    public static final int FOUND_STONE = 2;


    /**
     * initialization function for findStone.
     */
    public void initFindStone() throws Exception{
        vuforiaCommon.initTensorFlowObjectDetection();
    }
    /**
     * helper function to find a Stone using the Vurforia TensorFlow by strafing in the x direction
      * @param opmode the calling opmode needed for Telemetry
     * @param searchRight set true to search right, false to search left
     * @param onlySkystone keep searching until a SkyStone is found.
     * @param timeout timeout to give up if no stone found
     * @return INITIALIZATION_ERROR, NO_STONES_FOUND, FOUND_SKYSTONE, or FOUND_STONE
     */
    public int findStone(LinearOpMode opmode, boolean searchRight, boolean onlySkystone,double timeout){
        /**
         * Activate TensorFlow Object Detection
         **/
        if (vuforiaCommon.tensorFlowObjDetector != null) {
            vuforiaCommon.tensorFlowObjDetector.activate();
        }
        else{
            return INITIALIZATION_ERROR;
        }

        ElapsedTime findTimer = new ElapsedTime();
         // Set return code assuming we don't find a stone
        int retcode = NO_STONES_FOUND;
         // Set 1/4 power
        double power = 0.25;
        if (searchRight) {
            // Strafe to right
            drivetrain.setPower(power, -power, -power, power);
        }
        else{
            // Strafe to left
            drivetrain.setPower(-power, power, power, -power);
        }

        boolean keepSearching = true;
       while (opmode.opModeIsActive() && (findTimer.seconds() < timeout) &&
               (keepSearching == true)) {
           // Now check for recognitions
           List<Recognition> updatedRecognitions = vuforiaCommon.tensorFlowObjDetector.getUpdatedRecognitions();
           if (updatedRecognitions != null) {
               opmode.telemetry.addData("# Object Detected", updatedRecognitions.size());
               opmode.telemetry.update();
               // step through the list of recognitions and display boundary info.
               int i = 0;
               for (Recognition recognition : updatedRecognitions) {
                   opmode.telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                   opmode.telemetry.update();
                   if (recognition.getLabel().compareToIgnoreCase("Stone") == 0) {
                       retcode = FOUND_STONE;
                       // Check if we have to keep searching for a Skystone.
                       if (onlySkystone == false){
                           keepSearching = false;
                       }
                       break;  // Break out to return
                   } else if (recognition.getLabel().compareToIgnoreCase("Skystone") == 0) {
                       retcode = FOUND_SKYSTONE;
                       keepSearching = false;
                       break; // Break out to return
                   }
               }
           }
       }
        // Stop the motors
        drivetrain.stop();
        // Shut down the engine before returning
        vuforiaCommon.tensorFlowObjDetector.shutdown();

        return retcode;
    }
    /**
     * initialization function for moveRobotToStone.
     */
    public void initMoveRobotToStone() throws Exception{
        vuforiaCommon.initTensorFlowObjectDetection();
    }
    /**
     * helper function to move drivetrain in front of a stone using Vuforia Navigation.
     * The drivetrain must be sitting in front of a stone assumed to be found using
     * findStone method.
     * @param opmode the calling opmode needed for Telemetry
     * @param timeout timeout to give up if no stone found
     * @return true if move was successfully completed. false if Skystone went out of view
     */
    public boolean moveRobotToStone(LinearOpMode opmode, double timeout){

        ElapsedTime findTimer = new ElapsedTime();
        // Set return code assuming we have an error locating to the skysteon
        boolean retcode = false;
        // Set 1/4 power
        double power = 0.25;


        /** Assume stone was visible on entry. **/
        boolean stoneVisible = true;

        OpenGLMatrix lastLocation = null;

        // Assume that the skyStone is somewhere in front of the drivetrain so start the stoneTarget
        vuforiaCommon.stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, 0, 0)));

        // Activate the targets
        vuforiaCommon.targetsSkyStone.activate();

        while (opmode.opModeIsActive() && (findTimer.seconds() < timeout) &&
                (stoneVisible == true)) {
            stoneVisible = false;
            for (VuforiaTrackable trackable : vuforiaCommon.allTrackables) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    opmode.telemetry.addData("Visible Target", trackable.getName());
                    opmode.telemetry.update();
                    if (trackable.getName().compareToIgnoreCase(vuforiaCommon.stoneTarget.getName())==0){
                        stoneVisible = true;
                    }

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }

            // Provide feedback as to where the drivetrain is located (if we know).
            if (stoneVisible) {
                // express position (translation) of drivetrain in inches.
                VectorF translation = lastLocation.getTranslation();
                opmode.telemetry.addData("Stone Pos:", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                // express the rotation of the drivetrain in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                opmode.telemetry.addData("Stone Rot", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            }
            else {
                opmode.telemetry.addData("Lost Stone", "No stone found.");
            }
            opmode.telemetry.update();
        }

        // Disable Tracking when we are done;
        vuforiaCommon.targetsSkyStone.deactivate();
        // Stop the motors
        drivetrain.stop();

        return retcode;
    }

 }