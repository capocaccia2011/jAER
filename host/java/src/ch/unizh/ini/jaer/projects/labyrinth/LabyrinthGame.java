/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package ch.unizh.ini.jaer.projects.labyrinth;

import javax.media.opengl.GLAutoDrawable;
import net.sf.jaer.chip.AEChip;
import net.sf.jaer.event.EventPacket;
import net.sf.jaer.eventprocessing.*;
import net.sf.jaer.eventprocessing.EventFilter2DMouseAdaptor;
import net.sf.jaer.graphics.FrameAnnotater;
import net.sf.jaer.graphics.MultilineAnnotationTextRenderer;

/**
 * Top level labyrinth robot class.
 * @author tobi
 */
public class LabyrinthGame extends EventFilter2DMouseAdaptor  {

    public static String getDescription() {
        return "Top level labyinth game class";
    }
    LabyrinthBallController controller;
    LabyrinthVirtualBall virtualBall=null;
//    LabyrinthMap map;
    FilterChain filterChain;
    enum State {Starting, Running, Finished, LostTracking, PathNotFound};
    State state=State.Starting;

    public LabyrinthGame(AEChip chip) {
        super(chip);
        controller = new LabyrinthBallController(chip);
        virtualBall=new LabyrinthVirtualBall(chip,this);
        filterChain = new FilterChain(chip);

//        filterChain.add(map=new LabyrinthMap(chip));
        filterChain.add(virtualBall);
        filterChain.add(controller);
        setEnclosedFilterChain(filterChain);
        setPropertyTooltip("clearMap", "clears the map; use for bare table");
        setPropertyTooltip("loadMap","loads a map from an SVG file");
        setPropertyTooltip("controlTilts", "shows a GUI to directly control table tilts with mouse");
        setPropertyTooltip("centerTilts","centers the table tilts");
        setPropertyTooltip("disableServos","disables the servo motors by turning off the PWM control signals; digital servos may not relax however becuase they remember the previous settings");
        setPropertyTooltip("jiggleTable", "jiggle the table according to the jitter settings for the LabyrinthHardware");
    }

    @Override
    public EventPacket<?> filterPacket(EventPacket<?> in) {
        out= filterChain.filterPacket(in);
        if(controller.isLostTracking()){
            state=State.LostTracking;
        }else if(controller.isPathNotFound()){
            state=State.PathNotFound;
        }else if(controller.isAtMazeStart()){
            state=State.Starting;
        }else if(controller.isAtMazeEnd()){
            state=State.Finished;
        }else{
            state=State.Running;
        }
        return out;
    }

    @Override
    public void resetFilter() {
        filterChain.reset();
    }

    @Override
    public void initFilter() {
        resetFilter();
    }

    public void doDisableServos() {
        controller.disableServos();
    }

    public void doCenterTilts() {
        controller.centerTilts();
    }

    public void doControlTilts() {
        controller.controlTilts();
    }

    
    
    @Override
    public synchronized void setFilterEnabled(boolean yes) {
        super.setFilterEnabled(yes);
        virtualBall.setFilterEnabled(false); // don't enable by default
    }

    public void doLoadMap() {
        controller.loadMap();
    }

    public synchronized void doClearMap() {
        controller.clearMap();
    }

    public void doJiggleTable() {
        controller.doJiggleTable();
    }

    @Override
    public void annotate(GLAutoDrawable drawable) {
        super.annotate(drawable);
        MultilineAnnotationTextRenderer.resetToYPositionPixels(chip.getSizeY()-5);
        MultilineAnnotationTextRenderer.renderMultilineString("LabyrinthGate: State="+state.toString());
    }

    
    
}