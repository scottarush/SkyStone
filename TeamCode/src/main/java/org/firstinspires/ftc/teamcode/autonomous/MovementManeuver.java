package org.firstinspires.ftc.teamcode.autonomous;

public class MovementManeuver extends Maneuver {

    public double xDelta;
    public double yDelta;
    public int destTilenum;

    public MovementManeuver(double xDelta,double yDelta,int destTilenum){
        super("Movement");
        this.xDelta = xDelta;
        this.yDelta = yDelta;
        this.destTilenum = destTilenum;
    }
}
