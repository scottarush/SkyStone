package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name="EGGGGGGGGSSSSS", group="Sensor Tests")
//@Disabled
public class StoneColorSensor extends LinearOpMode {
    ColorSensor colorSensor;


    @Override
    public void runOpMode() throws InterruptedException {
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSen");


        float hsvValues[] = {0F, 0F, 0F};

        final float values[] = hsvValues;


        final double SCALE_FACTOR = 255;

        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);


        waitForStart();
        while(opModeIsActive()){
            Color.RGBToHSV((int) (colorSensor.red() * SCALE_FACTOR),
                    (int) (colorSensor.green() * SCALE_FACTOR),
                    (int) (colorSensor.blue() * SCALE_FACTOR),
                    hsvValues);

           if (hsvValues[0] > 0 && hsvValues[0] < 35) {
               if (hsvValues[1] < 0.3){
                   telemetry.addData("Grey/Black", "Sat is less than 0.3");
               } else {
                   telemetry.addData("Red", " AKA 0-35");
               }
           } else if (hsvValues[0] > 34 && hsvValues[0] < 75) {
               if (hsvValues[1] < 0.3){
                   telemetry.addData("Grey/Black", "Sat is less than 0.3");
               } else {
                   telemetry.addData("Yellow", " AKA 35-75");
               }
           } else if (hsvValues[0] > 74 && hsvValues[0] < 140){
               if (hsvValues[1] < 0.3){
                   telemetry.addData("Grey/Black", "Sat is less than 0.3");
               } else {
                   telemetry.addData("Green", " AKA 75-140");
               }
           } else if (hsvValues[0] > 139 && hsvValues[0] < 210){
               if (hsvValues[1] < 0.3){
                   telemetry.addData("Grey/Black", "Sat is less than 0.3");
               } else {
                   telemetry.addData("Cyan", " AKA 140-210");
               }
           } else if (hsvValues[0] > 209 && hsvValues[0] < 275){
               if (hsvValues[1] < 0.3){
                   telemetry.addData("Grey/Black", "Sat is less than 0.3");
               } else {
                   telemetry.addData("Blue", " AKA 210-275");
               }
           } else if (hsvValues[0] > 274 && hsvValues[0] < 345){
               if (hsvValues[1] < 0.3){
                   telemetry.addData("Grey/Black", "Sat is less than 0.3");
               } else {
                   telemetry.addData("Pink", " AKA 275-345");
               }
           } else if (hsvValues[0] > 344 && hsvValues[0] < 360){
               if (hsvValues[1] < 0.3){
                   telemetry.addData("Grey/Black", "Sat is less than 0.3");
               } else {
                   telemetry.addData("Red", " AKA 345-360");
               }
           } else {
               if (hsvValues[1] < 0.3){
                   telemetry.addData("Grey/Black", "Sat is less than 0.3");
               } else {
                   telemetry.addData("Not a valid color!", " Please try again!");
               }
           }
            telemetry.addData("Hue", hsvValues[0]);
            telemetry.addData("Saturation", hsvValues[1]);
            telemetry.addData("Value", hsvValues[2]);
            telemetry.update();

            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                }
            });

        }



    }
}
