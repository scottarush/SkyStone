package org.firstinspires.ftc.teamcode.autonomous;

public class RouteTransition {

    public FieldTile startTile;
    public FieldTile endTile;
    public int transitionNumber;

    public RouteTransition(FieldTile startTile,FieldTile endTile,int transitionNumber){
        this.startTile = startTile;
        this.endTile = endTile;
        this.transitionNumber = transitionNumber;
    }
}
