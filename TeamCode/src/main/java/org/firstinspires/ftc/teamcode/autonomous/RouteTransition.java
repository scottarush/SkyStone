package org.firstinspires.ftc.teamcode.autonomous;

public class RouteTransition {

    public FieldTile startTile;
    public int startTileOutEdge = FieldTile.EDGE_INVALID;
    public FieldTile endTile;
    public int endTileInEdge = FieldTile.EDGE_INVALID;
    public int transitionIndex;

    public RouteTransition(FieldTile startTile,int startTileOutEdge,FieldTile endTile,int endTileInEdge,int transitionNumber){
        this.startTile = startTile;
        this.startTileOutEdge = startTileOutEdge;
        this.endTile = endTile;
        this.endTileInEdge = endTileInEdge;
        this.transitionIndex = transitionNumber;
    }
}
