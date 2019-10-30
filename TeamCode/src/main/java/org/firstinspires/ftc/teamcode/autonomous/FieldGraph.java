package org.firstinspires.ftc.teamcode.autonomous;

import android.util.ArrayMap;

import java.lang.reflect.Field;
import java.util.ArrayList;

/**
 * This class is the graph used to define the Skystone playing field
 * squares.  Creators add routes into the fieldgraph to be used in
 * autonomous mode.
 */
public class FieldGraph {

    private FieldBlock[][] mFieldBlocks;
    private int mRows = 0;
    private int mCols = 0;
    public FieldGraph(int rows,int columns){
        mRows = rows;
        mCols = columns;
        // Create the graph with default connections
        mFieldBlocks = new FieldBlock[mRows][mCols];
        int blockNumCount = 0;
        for(int row=0;row < mRows;row++){
            for(int col=0;col < mCols;col++){
                // Create the block
                FieldBlock block = new FieldBlock(blockNumCount++);
            }
        }
        // Now loop through newly created blocks o set the neighbors
        for(int row=0;row < mRows;row++){
            for(int col=0;col < mCols;col++){
                // Create the block
                FieldBlock block = mFieldBlocks[row][col];
                for(int i=0;i < 4;i++){
                    switch(i) {
                        case FieldBlock.TOP:
                            if (row > 0) {
                                FieldBlock top = mFieldBlocks[row - 1][col];
                                block.setNeighbor(FieldBlock.TOP, top);
                            }
                            break;
                        case FieldBlock.LEFT:
                            if (col > 0) {
                                FieldBlock leftBlock = mFieldBlocks[row][col - 1];
                                block.setNeighbor(FieldBlock.LEFT, leftBlock);
                            }
                            break;
                        case FieldBlock.RIGHT:
                            if (col < mCols - 1) {
                                FieldBlock rightBlock = mFieldBlocks[row][col + 1];
                                block.setNeighbor(FieldBlock.RIGHT, rightBlock);
                            }
                        case FieldBlock.BOTTOM:
                            if (row < mRows - 1) {
                                FieldBlock bottomBlock = mFieldBlocks[row+1][col];
                                block.setNeighbor(FieldBlock.RIGHT, bottomBlock);
                            }
                    }
                }
            }
        }
    }

    /**
     * Adds a route segment
     * @param blockNumber
     * @param edge FieldBlock perimeter constant of the OUT_CONNECTION to add
     * @return true if add, false if invalid block or edge was a perimeter (and can't add connection)
     */
    public boolean addRouteSegment(int blockNumber,int edge){
        FieldBlock block = getBlock(blockNumber);
        if (block == null)
            return false;
        if (!FieldBlock.isEdgeValid(edge)){
            return false;
        }
         return true;
    }

    private FieldBlock getBlock(int blockNum) {
        for (int row = 0; row < mRows; row++) {
            for (int col = 0; col < mCols; col++) {
                FieldBlock block = mFieldBlocks[row][col];
                if (block.getBlockNum() == blockNum) {
                    return block;
                }
            }
        }
        // Invalid block number
        return null;
    }

    public boolean addRoute(int routeHandle,ArrayList<Integer> routeList){
        return true;
    }
}
