/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package team3647.lib;

/**
 * Add your docs here.
 */
public class IndexerSignal {
    private final double leftVerticalOutput;
    private final double rightVerticalOutput;
    private final double tunnelOutput;
    private final double horizontalRollersOutput;

    public static IndexerSignal STOP = new IndexerSignal(0, 0, 0, 0);
    public static IndexerSignal GO = new IndexerSignal(1, .8, .7, .4);
    public static IndexerSignal GO_TUNNEL_STOP = new IndexerSignal(0, 0, .7, 0);
    public static IndexerSignal SPITOUT = new IndexerSignal(-1, -.8, -1, -1);
    public static IndexerSignal TUNNELHOLD_GO = new IndexerSignal(.5, .3, 0, 0);
    public static IndexerSignal GO_SLOW = new IndexerSignal(1, .8, .5, .4);
    public static IndexerSignal GO_FAST = new IndexerSignal(1, .8, 1, .6);
    public static IndexerSignal TUNNELDOWN_HOTDOGOUT = new IndexerSignal(-.7, -.5, -.7, -.5);
    public static IndexerSignal INDEXERFWD_SLOW = new IndexerSignal(1, .7, .3, .5);

    public IndexerSignal(double leftVerticalOutput, double rightVerticalOutput, double tunnelOutput,
            double horizontalRollersOutput) {
        this.leftVerticalOutput = leftVerticalOutput;
        this.tunnelOutput = tunnelOutput;
        this.horizontalRollersOutput = horizontalRollersOutput;
        this.rightVerticalOutput = rightVerticalOutput;
    }

    public double getLeftVerticalOutput() {
        return this.leftVerticalOutput;
    }

    public double getTunnelOutput() {
        return this.tunnelOutput;
    }

    public double getHorizontalRollersOutput() {
        return this.horizontalRollersOutput;
    }

    public double getRightVerticalOutput() {
        return this.rightVerticalOutput;
    }
}
