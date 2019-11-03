package org.firstinspires.ftc.teamcode.autonomous;

public class MovementManeuver extends Maneuver {

    public double xDelta;
    public double yDelta;
    public Route route;
    public int transitionIndex;

    public MovementManeuver(double xDelta,double yDelta,Route r,int transitionIndex){
        super("Movement");
        this.xDelta = xDelta;
        this.yDelta = yDelta;
        this.route = r;
        this.transitionIndex = transitionIndex;
    }

}
