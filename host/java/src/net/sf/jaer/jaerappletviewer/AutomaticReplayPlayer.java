/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package net.sf.jaer.jaerappletviewer;
import com.sun.opengl.util.*;
import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.EOFException;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.util.logging.Level;
import java.util.logging.Logger;
import javax.media.opengl.GL;
import javax.media.opengl.GLAutoDrawable;
import net.sf.jaer.aemonitor.*;
import net.sf.jaer.chip.AEChip;
import net.sf.jaer.event.EventPacket;
import net.sf.jaer.eventio.*;
import net.sf.jaer.eventio.AEOutputStream;
import net.sf.jaer.eventprocessing.EventFilter2D;
import net.sf.jaer.eventprocessing.FilterChain;
import net.sf.jaer.eventprocessing.filter.EventRateEstimator;
import net.sf.jaer.eventprocessing.filter.Info;
import net.sf.jaer.graphics.FrameAnnotater;
import net.sf.jaer.util.filter.LowpassFilter;
/**
 * Monitors input events for sudden increases in activity. When this increase is detected, a recording is started to memory. When the buffer is full or live input activity stops for a sufficiently
 * long period of time, the recording is substituted for the live input. The recording plays in a loop until live activity is detected again.
 *
 * @author tobi
 *
 * This is part of jAER
<a href="http://jaer.wiki.sourceforge.net">jaer.wiki.sourceforge.net</a>,
licensed under the LGPL (<a href="http://en.wikipedia.org/wiki/GNU_Lesser_General_Public_License">http://en.wikipedia.org/wiki/GNU_Lesser_General_Public_License</a>.
 */
public class AutomaticReplayPlayer extends EventFilter2D implements FrameAnnotater{
    public static String getDescription (){
        return "Automatically replays input when input activity falls below a threshold for long enough.";
    }

    public static DevelopmentStatus getDevelopmentStatus (){
        return DevelopmentStatus.Alpha;
    }
    private AEOutputStream os;
    private AEInputStream is;
    private int MAX_NUM_EVENTS_DEFAULT = 1000000;
    private int maxNumEventsToRecord = getPrefs().getInt("AutomaticReplayPlayer.maxNumEventsToRecord",MAX_NUM_EVENTS_DEFAULT);
    private ByteArrayOutputStream bos = null;
    private ByteArrayInputStream bis = null;
    private float eventRateMeasured = 0;
    private float eventRateTauMs = getPrefs().getFloat("AutomaticReplayPlayer.eventRateTauMs",100);
    private int activityTimeoutMs = getPrefs().getInt("AutomaticReplayPlayer.activityTimeoutMs",5000);
    private float eventRateThresholdEPS = getPrefs().getFloat("AutomaticReplayPlayer.eventRateThresholdEPS",10000);
    private int lastActiveTime = 0;
    private int numEventsRecorded = 0;
//    private boolean recordingEnabled = false;
    private int numReplayEventsPerSlice = getPrefs().getInt("AutomaticReplayPlayer.numReplayEventsPerSlice",1000);
    /** Possible states */
    public enum State{
        Init, Live, Replay
    };
    private State state = State.Init;
    private Info info = null;
    private EventRateEstimator eventRateFilter;
    private int currentReplayPosition = 0;

    public AutomaticReplayPlayer (AEChip chip){
        super(chip);
        setPropertyTooltip("eventRateTauMs","lowpass time constant in ms for filtering event rate");
        FilterChain fc = new FilterChain(chip);
        eventRateFilter = new EventRateEstimator(chip);
        eventRateFilter.setEventRateTauMs(eventRateTauMs);
        info = new Info(chip);
        fc.add(info);
        info.setEventRate(true);
        setEnclosedFilterChain(fc);

        setPropertyTooltip("maxNumEventsToRecord","length of buffer in events to replay");
        setPropertyTooltip("eventRateTauMs","lowpass filter time constant for measuring live input event rate");
        setPropertyTooltip("activityTimeoutMs","timeout in mx to stop recording activity");
        setPropertyTooltip("eventRateThresholdEPS","threshold in events per second to show live input");
        setPropertyTooltip("numReplayEventsPerSlice","# of events per shown slice to show during replay");
    }

    @Override
    synchronized public EventPacket<?> filterPacket (EventPacket<?> in){
        if ( in.getSize() == 0 ){
            return in;
        }
        // always measure live input
        eventRateFilter.filterPacket(in);
        eventRateMeasured = eventRateFilter.getFilteredEventRate();
        switch ( state ){
            case Init:
                if ( isActiveNow() ){
                    setState(State.Live);
                }
                out = in;  // just show input until we have gone live
                break;
            case Live:
                if ( isActiveNow() ){
                    lastActiveTime = in.getLastTimestamp();
                }
                if ( isActiveNow() || ( !isActiveNow() && !isTimedOut(in.getFirstTimestamp()) ) ){
                    recordPacket(in);
                    // stay Live
                    out = in;
                } else if ( ( !isActiveNow() ) && isTimedOut(in.getFirstTimestamp()) ){
                    setState(State.Replay);
                    try{
                        // allocate a new AEInputStream that wraps a byte array copied from the last recording
                        bis = new ByteArrayInputStream(bos.toByteArray()); // bis will have just the recorded events
                        is = new AEInputStream(bis);
                        is.setAddressType(Integer.TYPE);  // default is Short.TYPE
                    } catch ( IOException ex ){
                        log.warning(ex.toString());
                    }
                }
                break;
            case Replay:
                if ( isActiveNow() ){
                    setState(State.Live);
                    resetRecording();
                    recordPacket(in);
                    out = in;  // immediately show live input
                }
                AEPacketRaw replayRawInput;
                try{
                    if ( currentReplayPosition + numReplayEventsPerSlice > numEventsRecorded ){
                        resetReplay();
                    }
                    replayRawInput = is.readPacketByNumber(numReplayEventsPerSlice);
                    chip.getEventExtractor().extractPacket(replayRawInput,out);
                } catch ( EOFException ex ){
                    bis.reset();
                    try{
                        replayRawInput = is.readPacketByNumber(getNumReplayEventsPerSlice());
                        chip.getEventExtractor().extractPacket(replayRawInput,out);
                    } catch ( IOException ex1 ){
                        log.warning("on reading recorded packet again after rewind caught " + ex1.toString());
                        return in;
                    }
                } catch ( IOException ioe ){
                    log.warning("on reading recorded data caught exception " + ioe + ", returning live input");
                    out = in;
                }
                currentReplayPosition += out.getSize();
                break;
        }
        info.filterPacket(out);
        return out;
    }
    private GLUT glut = new GLUT();

    public void annotate (GLAutoDrawable drawable){
        GL gl = drawable.getGL();
        gl.glPushMatrix();
        String s = state.toString();
        gl.glScalef(.1f,.1f,1);
//        float l=glut.glutStrokeLengthf(GLUT.STROKE_ROMAN,s);
        gl.glTranslatef(0,chip.getSizeY() * .8f,0);
        gl.glLineWidth(3);
        gl.glColor3f(1,0,0);
        glut.glutStrokeString(GLUT.STROKE_ROMAN,s);
        gl.glPopMatrix();
        switch ( state ){
            case Replay:
                // draw replay progress bar
                gl.glPushMatrix();
                gl.glColor3f(0,0,1);
                gl.glRectf(1,1,chip.getSizeX() * (float)currentReplayPosition / numEventsRecorded,3);
                gl.glPopMatrix();
                break;
            case Live:
            case Init:
                info.annotate(drawable);
        }
    }

    private void resetReplay (){
        bis.reset();
        currentReplayPosition = 0;
    }

    private void resetRecording (){
        bos.reset();
        numEventsRecorded = 0;
        currentReplayPosition = 0;
    }

    private void recordPacket (EventPacket in){
        AEPacketRaw raw = chip.getEventExtractor().reconstructRawPacket(in);
        if ( numEventsRecorded + in.getSize() > MAX_NUM_EVENTS_DEFAULT ){ // if buffer would be overfilled, reset to start
            resetRecording();
        }
        try{
            os.writePacket(raw); // append data to output stream
            numEventsRecorded += raw.getNumEvents();
        } catch ( IOException ex ){
            log.warning("when writing recording to output memory stream, caught " + ex + ", resetting recording");
            resetRecording();
            return;
        }
    }

    private boolean isActiveNow (){
        return eventRateMeasured >= eventRateThresholdEPS;
    }

    private int timeSinceActive (int t){
        return t - lastActiveTime;
    }

    private boolean isTimedOut (int t){
        return timeSinceActive(t) >= activityTimeoutMs * AEConstants.TICK_DEFAULT_US * 1000;
    }

    private boolean shouldRecordNow (int t){
        return isActiveNow() || timeSinceActive(t) < activityTimeoutMs;
    }

    synchronized public void setState (State state){
        State old = this.state;
        this.state = state;
        log.info("State " + old + " -> State " + state);
        support.firePropertyChange("state",old,state);
    }

    public State getState (){
        return state;
    }

    private void checkBuffers () throws IOException{
        if ( bos != null ){
            return;
        }
        bos = new ByteArrayOutputStream(AEFileInputStream.EVENT32_SIZE * MAX_NUM_EVENTS_DEFAULT);
        os = new AEOutputStream(bos);
    }

    @Override
    synchronized public void resetFilter (){
        try{
            checkBuffers();
        } catch ( IOException ex ){
            log.warning(ex.toString());
        }
        resetRecording();
        info.resetFilter();
        eventRateFilter.resetFilter();
        setState(State.Init);
    }

    @Override
    public void initFilter (){
        resetFilter();
    }

    public float getEventRateTauMs (){
        return eventRateTauMs;
    }

    /** Time constant of event rate lowpass filter in ms */
    synchronized public void setEventRateTauMs (float eventRateTauMs){
        if ( eventRateTauMs < 0 ){
            eventRateTauMs = 0;
        }
        this.eventRateTauMs = eventRateTauMs;
        eventRateFilter.setEventRateTauMs(eventRateTauMs);
        getPrefs().putFloat("AutomaticReplayPlayer.eventRateTauMs",eventRateTauMs);
    }

    /**
     * @return the activityTimeoutMs
     */
    public int getActivityTimeoutMs (){
        return activityTimeoutMs;
    }

    /**
     * @param activityTimeoutMs the activityTimeoutMs to set
     */
    public void setActivityTimeoutMs (int activityTimeoutMs){
        this.activityTimeoutMs = activityTimeoutMs;
        getPrefs().putInt("AutomaticReplayPlayer.activityTimeoutMs",activityTimeoutMs);
    }

    /**
     * @return the eventRateThresholdEPS
     */
    public float getEventRateThresholdEPS (){
        return eventRateThresholdEPS;
    }

    /**
     * @param eventRateThresholdEPS the eventRateThresholdEPS to set
     */
    public void setEventRateThresholdEPS (float eventRateThresholdEPS){
        this.eventRateThresholdEPS = eventRateThresholdEPS;
        getPrefs().putFloat("AutomaticReplayPlayer.eventRateThresholdEPS",eventRateThresholdEPS);
    }

    /**
     * @return the maxNumEventsToRecord
     */
    public int getMaxNumEventsToRecord (){
        return maxNumEventsToRecord;
    }

    /**
     * @param maxNumEventsToRecord the maxNumEventsToRecord to set
     */
    public void setMaxNumEventsToRecord (int maxNumEventsToRecord){
        this.maxNumEventsToRecord = maxNumEventsToRecord;
        getPrefs().putInt("AutomaticReplayPlayer.maxNumEventsToRecord",maxNumEventsToRecord);
    }

    /**
     * @return the numReplayEventsPerSlice
     */
    public int getNumReplayEventsPerSlice (){
        return numReplayEventsPerSlice;
    }

    /**
     * @param numReplayEventsPerSlice the numReplayEventsPerSlice to set
     */
    public void setNumReplayEventsPerSlice (int numReplayEventsPerSlice){
        this.numReplayEventsPerSlice = numReplayEventsPerSlice;
        getPrefs().putInt("AutomaticReplayPlayer.numReplayEventsPerSlice",numReplayEventsPerSlice);
    }
}