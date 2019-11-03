package org.firstinspires.ftc.teamcode.autonomous;

import java.util.ArrayList;
import java.util.Iterator;

/**
 * This the class for a route.
 */
public class Route {
    private String mName;
    private FieldGraph mGraph;

    FieldTile mCurrentBuildTile = null;

    private ArrayList<RouteTransition> mRouteTileTransitions = new ArrayList<>();

    private int mCurrentTransitionIndex = 0;

    public Route(String name,FieldGraph graph){
        mName = name;
        mGraph = graph;
    }

    public String getName(){
        return mName;
    }

    /**
     * Sets the starting tile of the route.
     * @return false if tileNum invalid, false otherwise
     */
    public boolean setStartTile(int tileNum){
        FieldTile tile = mGraph.getTile(tileNum);
        if (tile == null)
            return false;
        mRouteTileTransitions.add(tile);
        tile.setStartOfRoute(this);
        mCurrentBuildTile = tile;
        return true;
    }

    public boolean addRouteTransition(int destTileNum,Action entryAction){
        FieldTile tile = mGraph.getTile(destTileNum);
        if (tile == null)
            return false;
        RouteTransition transition = mCurrentBuildTile.addRouteTransition(this,tile,entryAction);
        if (transition != null){
            mCurrentBuildTile = tile;
            // Set the transition number now that we know it to the next index
            transition.transitionNumber = mRouteTileTransitions.size();
            mRouteTileTransitions.add(transition);
        }
        return (transition != null);
    }

    /**
     * @return number of transitions on the route.
     */
    public int getNumTransitions(){
        return mRouteTileTransitions.size();
    }
    /**
     * @return the number of the current transition on this route
     */
    public int getCurrentTransitionNum(){
        return mCurrentTransitionIndex;
    }

    /**
     * Retrieves the transition index for the provided tile.  If there are multiple transitions
     * on this tile, the incoming transition will be selected.
     * @return index or -1 if tile is not on this route
     */
    private int getTransitionIndex(FieldTile tile){
        // crawl the route to find the first occurence which have an outgoing edge or the endge tile
        for(int index = 0;index < mRouteTileTransitions.size();index++){
            FieldTile aTile = mRouteTileTransitions.get(index);
            if (aTile == tile){
                return index;
            }
        }
        return -1;
    }

    /**
     * Resets the route to run on the provided tile
     * @return true if reset, false if tile was invalid.
     */
    public boolean resetRouteOnTile(FieldTile tile){
        return getTransitionIndex(tile) >= 0;
    }
    /**
     * Sets the ending tile of the route.  This route must have
     * been previously added to the tile or will return an error
     * @return false if tileNum invalid, false otherwise
     */
    public boolean setEndTile(int tileNum){
        FieldTile tile = mGraph.getTile(tileNum);
        if (tile == null)
            return false;
        boolean retcod = tile.setEndOfRoute(this);
        mRouteTileTransitions.add(tile);
        return retcod;
    }


}
