package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Grabber;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.drivetrain.MecanumDrive;

/**
 * This is the Mechanm bot with a grabber arm.
 */
public class MecanumGrabberBot extends Robot {

    private Grabber grabber;

    public MecanumGrabberBot(OpMode opMode){
        super(DriveTrainStyle.MECANUM,opMode);
    }

    public MecanumDrive getDrivetrain(){
        return (MecanumDrive) super.getDrivetrain();
    }

    /**
     * Returns the grabber.
     */
    public Grabber getGrabber(){
        return grabber;
    }
    /**
     * Override base class function to initialize the rest of the
     * bot.
     * @throws Exception
     */
    public void init() throws Exception {
        String initErrString = "";
        try {
            super.init();
        }
        catch (Exception e){
            initErrString += e.getMessage();
        }
        // base drivetrain init must have been OK so init the rest of the bot.
        try{
            grabber = new Grabber(opMode);
        }
        catch (Exception e) {
            initErrString += e.getMessage();
        }
        throw new Exception ("Robot init errs: "+initErrString);
    }

}
