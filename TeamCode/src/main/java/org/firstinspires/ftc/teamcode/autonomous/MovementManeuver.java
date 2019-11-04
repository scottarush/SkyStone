package org.firstinspires.ftc.teamcode.autonomous;

public class MovementManeuver extends Maneuver {

    public double xDelta;
    public double yDelta;
    public RouteTransition transition;

    public MovementManeuver(double xDelta,double yDelta,RouteTransition transition){
        super("Movement");
        this.xDelta = xDelta;
        this.yDelta = yDelta;
        this.transition = transition;
    }

    @Override
    public String toString(){
        return ("Vector movement delta: ("+xDelta+","+yDelta+") from tile: "+
                transition.startTile.getTileNum()+
                " to:"+transition.endTile.getTileNum());
    }

}
