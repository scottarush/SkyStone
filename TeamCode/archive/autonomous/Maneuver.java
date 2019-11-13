package org.firstinspires.ftc.teamcode.archive.autonomous;

/**
 * Class encapsulating a maneuver.
 */
public class Maneuver {
    private String mName = null;

    public Maneuver(String name){
        mName = name;
     }

    public String getName(){
        return  mName;
    }
}
