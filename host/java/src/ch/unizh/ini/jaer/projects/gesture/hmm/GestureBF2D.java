/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package ch.unizh.ini.jaer.projects.gesture.hmm;

import ch.unizh.ini.jaer.projects.gesture.blurringFilter.BlurringFilter2DTracker;
import java.awt.*;
import java.awt.event.*;
import java.awt.geom.Point2D;
import java.io.*;
import java.util.*;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;
import javax.media.opengl.GLAutoDrawable;
import javax.swing.*;
import javax.swing.Timer;
import net.sf.jaer.chip.AEChip;
import net.sf.jaer.event.EventPacket;
import net.sf.jaer.eventprocessing.*;
import net.sf.jaer.eventprocessing.tracking.ClusterPathPoint;
import net.sf.jaer.graphics.FrameAnnotater;
import net.sf.jaer.util.filter.LowpassFilter2d;

/**
 * Gesture recognition system using a single DVS sensor.
 * BluringFilter2DTracker is used to obtain the trajectory of moving object (eg, hand)
 * HMM is used for classification. But, HMM is not used for spoting gestures (i.e., finding the start and end timing of gestures)
 * Gesture spotting is done by the tracker by assuming that there is slow movement or abrupt change of moving direction between gestures.
 *
 * @author Jun Haeng Lee
 */
public class GestureBF2D extends EventFilter2D implements FrameAnnotater,Observer{
    /**
     * a cluster with points more than this amount will be checked for gesture recognition.
     */
    private int numPointsThreshold = getPrefs().getInt("GestureBF2D.numPointsThreshold", 50);

    /**
     * retries HMM after this percents of head points is removed from the trajectory when the first tiral is failed.
     */
    private int headTrimmingPercents = getPrefs().getInt("GestureBF2D.headTrimmingPercents", 30);

    /**
     * retries HMM after this percents of tail points is removed from the trajectory when the first tiral is failed.
     */
    private int tailTrimmingPercents = getPrefs().getInt("GestureBF2D.tailTrimmingPercents", 10);

    /**
     * enables lowpass filter to smooth the gesture trajectory
     */
    private boolean enableLPF = getPrefs().getBoolean("GestureBF2D.enableLPF", true);

    /**
     * lowpass filter time constant for gesture trajectory in ms
     */
    private float tauPathMs = getPrefs().getFloat("GestureBF2D.tauPathMs",5.0f);

    /**
     * refractory time in ms between gestures.
     */
    private int refractoryTimeMs = getPrefs().getInt("GestureBF2D.refractoryTimeMs", 700);


    /**
     * lowpass filter time constant for gesture trajectory in ms
     */
    private float GTCriterion = getPrefs().getFloat("GestureBF2D.GTCriterion",3.0f);
    /**
     * enables using prevPath to find gesture pattern
     */
    private boolean usePrevPath = getPrefs().getBoolean("GestureBF2D.usePrevPath", true);

    /**
     * time to wait for auto logout
     */
    private int autoLogoutTimeMs = getPrefs().getInt("GestureBF2D.autoLogoutTimeMs", 10000);
    private boolean enableAutoLogout = getPrefs().getBoolean("GestureBF2D.enableAutoLogout", true);

    /**
     * maximum time difference between correlated segments
     */
    private int maxTimeDiffCorrSegmentsUs = getPrefs().getInt("GestureBF2D.maxTimeDiffCorrSegmentsUs", 300000);

    /**
     * hmm save file
     */
    private String HmmSaveFilePath = getPrefs().get("GestureBF2D.HmmSaveFilePath", "");

    /**
     * if false, disable gesture recognition
     */
    protected boolean enableGestureRecognition = true;

    
    /**
     * true if the gesture recognition system is activated.
     */
    protected boolean login = false;

    /**
     * if true, a picture will be shown on the drqwing panel after gesture recognition
     */
    protected boolean showGesturePic = true;

    /**
     * if true, shows raw trace for diagnosis
     */
    protected boolean showRawTrace = false;

    /**
     * path of gesture picture files
     */
    public static String pathGesturePictures = "D:/user/gesture pictures/";

    /**
     * images for gestures
     */
    protected Image   imgHi = null, imgBye = null, imgLeft = null, imgRight = null, imgUp = null,
                    imgDown = null, imgCW = null, imgCCW = null, imgCheck = null, imgPush = null,
                    imgDCW = null, imgDCCW = null;

    /**
     * timmings in the current and previous gestures
     */
    protected int startTimeGesture, endTimeGesture, endTimePrevGesture = 0;

    /**
     * 'Check' gesture is recognized by a check shape or a sequence of 'SlashDown' and 'SlashUp'
     * checkActivated is true if 'SlashDown' is detected. It's false otherwise.
     */
    protected boolean checkActivated = false;

    /**
     * previous path
     */
    protected ArrayList<ClusterPathPoint> prevPath;

    /**
     * the latest location of cluster
     */
    protected Point2D.Float latestLocation = new Point2D.Float();



    /**
     * moving object tracker
     */
    protected BlurringFilter2DTracker tracker;

    /**
     * feature extractor
     */
    protected FeatureExtraction fve = null;

    /**
     * Hand drawing panel with gesture HMM module
     */
    private HmmDrawingPanel hmmDP = null;

    /**
     * low pass filter to smoothe the trajectory of gestures
     */
    protected LowpassFilter2d lpf;

    /**
     * timer for auto logout
     */
    protected Timer autoLogoutTimer;

    /**
     * if true, save the path
     */
    protected boolean savePath = true;

    /**
     * Queue for detected gesture history
     */
    protected LinkedList<Gesture> gestureQueue = new LinkedList<Gesture>();

    /**
     * maximum number of gestures in the queue
     */
    protected static int maxGestureQueue = 5;


    /**
     * constructor
     * 
     * @param chip
     */
    public GestureBF2D(AEChip chip) {
        super(chip);
        this.chip = chip;
        chip.addObserver(this);

        autoLogoutTimer = new Timer(autoLogoutTimeMs, autoLogoutAction);

        String trimming = "Trimming", selection  = "Selection", lpfilter = "Low pass filter", gesture = "Gesture", autoLogout = "Auto logout", hmm = "HMM";
        setPropertyTooltip(selection,"numPointsThreshold","a cluster with points more than this amount will be checked for gesture recognition.");
        setPropertyTooltip(selection,"maxSpeedThreshold_kPPT","speed threshold of the cluster to be a gesture candidate (in kPPT).");
        setPropertyTooltip(trimming,"headTrimmingPercents","retries HMM after this percents of head points is removed from the trajectory when the first tiral is failed.");
        setPropertyTooltip(trimming,"tailTrimmingPercents","retries HMM after this percents of tail points is removed from the trajectory when the first tiral is failed.");
        setPropertyTooltip(lpfilter,"enableLPF","enables lowpass filter to smooth the gesture trajectory");
        setPropertyTooltip(lpfilter,"tauPathMs","lowpass filter time constant for gesture trajectory in ms");
        setPropertyTooltip(gesture,"refractoryTimeMs","refractory time in ms between gestures");
        setPropertyTooltip(gesture,"GTCriterion","criterion of Gaussian threshold");
        setPropertyTooltip(gesture,"usePrevPath","enables using prevPath to find gesture pattern");
        setPropertyTooltip(gesture,"maxTimeDiffCorrSegmentsUs","maximum time difference between correlated segments");
        setPropertyTooltip(autoLogout,"autoLogoutTimeMs","time in ms to wait before auto logout");
        setPropertyTooltip(autoLogout,"enableAutoLogout","if true, auto logout is done if it's been more than autoLogoutTimeMs since the last gesture");
        setPropertyTooltip(hmm,"HmmSaveFilePath","absolute path of a trained HMM save file");

        // low pass filter
        this.lpf = new LowpassFilter2d();

        // encloses tracker
        filterChainSetting ();
    }

    /**
     * action listener for timer events
     */
    ActionListener autoLogoutAction = new ActionListener() {
        @Override
        public void actionPerformed(ActionEvent evt) {
            doLogout();
        }
    };

    /**
     * sets the BlurringFilter2DTracker as a enclosed filter to find cluster
     */
   protected void filterChainSetting (){
        tracker = new BlurringFilter2DTracker(chip);
        ( (EventFilter2D)tracker ).addObserver(this);
        setEnclosedFilterChain(new FilterChain(chip));
        getEnclosedFilterChain().add((EventFilter2D)tracker);
        ( (EventFilter2D)tracker ).setEnclosed(true,this);
        ( (EventFilter2D)tracker ).setFilterEnabled(isFilterEnabled());
    }


    @Override
    public EventPacket<?> filterPacket(EventPacket<?> in) {
        out = tracker.filterPacket(in);
        return out;
    }

    @Override
    public void initFilter() {
        tracker.initFilter();
        endTimePrevGesture = 0;
        lpf.setTauMs(tauPathMs);
    }

    @Override
    public void resetFilter() {
        tracker.resetFilter();
        endTimePrevGesture = 0;
        lpf.setTauMs(tauPathMs);
        resetPrevPath();
    }

    @Override
    public synchronized void setFilterEnabled (boolean filterEventsEnabled){
        super.setFilterEnabled(filterEventsEnabled);
        
        if ( filterEventsEnabled ){
            if(hmmDP == null){
                // hand drawing panel with gesture HMM
                String [] bNames = {"Add", "Remove", "Reset", "Show", "Learn", "Guess"};
                hmmDP = new HmmDrawingPanel("HMM based gesture recognition test using hand drawing panel", bNames);
                // load gesture images into the memory
                loadGestureImages();
                // load HMM save
                if(!HmmSaveFilePath.equals(""))
                    hmmDP.openFile(HmmSaveFilePath);
            } else
                hmmDP.setVisible(true);
        } else{
            if(hmmDP != null)
                hmmDP.setVisible(false);
        }
    }

    @Override
    public void annotate(GLAutoDrawable drawable) {
        // do nothing
    }

    /**
     * load gesture Images
     */
    protected final void loadGestureImages(){
        Toolkit myToolkit = Toolkit.getDefaultToolkit();

        imgHi = myToolkit.getImage(pathGesturePictures + "hi.jpg");
        hmmDP.putImage(imgHi);
        imgBye = myToolkit.getImage(pathGesturePictures + "bye.jpg");
        hmmDP.putImage(imgBye);
        imgLeft = myToolkit.getImage(pathGesturePictures + "left.jpg");
        hmmDP.putImage(imgLeft);
        imgRight = myToolkit.getImage(pathGesturePictures + "right.jpg");
        hmmDP.putImage(imgRight);
        imgUp = myToolkit.getImage(pathGesturePictures + "up.jpg");
        hmmDP.putImage(imgUp);
        imgDown = myToolkit.getImage(pathGesturePictures + "Down.jpg");
        hmmDP.putImage(imgDown);
        imgCW = myToolkit.getImage(pathGesturePictures + "clockwise.jpg");
        hmmDP.putImage(imgCW);
        imgCCW = myToolkit.getImage(pathGesturePictures + "counterclockwise.jpg");
        hmmDP.putImage(imgCCW);
        imgCheck = myToolkit.getImage(pathGesturePictures + "check.jpg");
        hmmDP.putImage(imgCheck);
        imgDCW = myToolkit.getImage(pathGesturePictures + "doubleclockwise.jpg");
        hmmDP.putImage(imgDCW);
        imgDCCW = myToolkit.getImage(pathGesturePictures + "doublecounterclockwise.jpg");
        hmmDP.putImage(imgDCCW);
//        imgPush = myToolkit.getImage(pathGesturePictures + "push.jpg");
//        hmmDP.putImage(imgPush);
    }

    @Override
    public void update(Observable o, Object arg) {
        if ( !(o instanceof BlurringFilter2DTracker) )
            return;

        UpdateMessage msg = (UpdateMessage)arg;

        if( enableGestureRecognition ){
            List<BlurringFilter2DTracker.Cluster> cl = tracker.getClusters();
            ArrayList<ClusterPathPoint> path = selectClusterTrajectory(cl);

            if(path == null)
                return;

             // default trimming
            ArrayList<ClusterPathPoint> trimmedPath = trajectoryTrimmingPointBase(path, 1, 1);

            // doesn't have to classify short trajectroies
            if(trimmedPath.size() < numPointsThreshold)
            {
                // stores the small fraction to use it later
                storePath(trimmedPath, true);
                return;
            }


            String bmg = null;
            if(login){
                // estimates the best matching gesture
                bmg = estimateGesture(trimmedPath);

                if(usePrevPath && bmg == null)
                    bmg = estimateGestureWithPrevPath(trimmedPath, maxTimeDiffCorrSegmentsUs, true, true);

                System.out.println("Best matching gesture is " + bmg);

                if(bmg != null){
                    if(afterRecognitionProcess(bmg, trimmedPath)){
                        endTimePrevGesture = endTimeGesture;

                        // starts or restarts the auto logout timer
                        if(isLogin()){ // has to check if the gesture recognition system is still active (to consider logout)
                            if(autoLogoutTimer.isRunning())
                                autoLogoutTimer.restart();
                            else
                                autoLogoutTimer.start();
                        }
                    }
                }else
                    savePath = true;
                
            } else {
                if(detectStartingGesture(trimmedPath)){
                    System.out.println("Gesture recognition system is enabled.");
                    afterRecognitionProcess("Infinite", trimmedPath);
                    endTimePrevGesture = endTimeGesture;
                } else
                    savePath = true;
            }

            if(savePath)
                storePath(trimmedPath, false);
            else
                resetPrevPath();
        }

        // callback to update() of any listeners on us, e.g. VirtualDrummer
        callUpdateObservers(msg.packet, msg.timestamp);
    }

    /**
     * resets prevPath
     * make prevPath null
     */
    public void resetPrevPath(){
        prevPath = null;
    }

    /**
     * stores the path into prevPath
     *
     * @param path
     * @param accumulate : if true, appends the path at the end of prevPath
     */
    public void storePath(ArrayList<ClusterPathPoint> path, boolean accumulate){
        // doesn't have to store very very long path
        if(FeatureExtraction.calTrajectoryLength(path) > chip.getSizeX() * 2){
            prevPath = null;
            return;
        }

        if(prevPath == null || !accumulate || !doesAccumulate(path, maxTimeDiffCorrSegmentsUs, true, true))
            prevPath = new ArrayList<ClusterPathPoint>();

        for(int i=0; i<path.size(); i++){
            ClusterPathPoint pt = path.get(i);
            ClusterPathPoint clonePt = new ClusterPathPoint(pt.x, pt.y, pt.t, pt.getNEvents());
            clonePt.stereoDisparity = pt.stereoDisparity;
            if(clonePt.velocityPPT != null){
                clonePt.velocityPPT.x = pt.velocityPPT.x;
                clonePt.velocityPPT.y = pt.velocityPPT.y;
            }
            prevPath.add(clonePt);
//            prevPath.add((ClusterPathPoint) pt.clone());
        }
    }

    /**
     * detects the startinf gesture (ie. 'Infinite' shape)
     * It tries several times by trimming the input trajectory.
     * 
     * @param path
     * @return
     */
    protected boolean detectStartingGesture(ArrayList<ClusterPathPoint> path){
        boolean ret = false;
        String bmg = estimateGesture(path);

        if(bmg != null && bmg.startsWith("Infinite"))
            ret = true;
        else {
            if(usePrevPath){
                bmg = estimateGestureWithPrevPath(path, maxTimeDiffCorrSegmentsUs, true, true);
                if(bmg != null && bmg.startsWith("Infinite")){
                    ret = true;
                }
            }
        }

        return ret;
    }


    /**
     * detects the specified gesture with prevPath added
     *
     * @param path
     * @param prevPathTrimmingPercent
     * @param gestureName
     * @param timeDiffTolerenceUs
     * @param useTimeCons
     * @param useDistCons
     * @return
     */
    public boolean tryGestureWithPrevPath(ArrayList<ClusterPathPoint> path, int prevPathTrimmingPercent, String gestureName, int timeDiffTolerenceUs, boolean useTimeCons, boolean useDistCons){
        boolean ret = false;
        if(doesAccumulate(path, timeDiffTolerenceUs, useTimeCons, useDistCons)){
            ArrayList<ClusterPathPoint> trimmedPath = trajectoryTrimming(prevPath, prevPathTrimmingPercent, 0, FeatureExtraction.calTrajectoryLength(prevPath));

            trimmedPath.addAll(path);
            String bmg = estimateGesture(trimmedPath);
            if(bmg != null && bmg.startsWith(gestureName))
                ret = true;
        }

        return ret;
    }

    /**
     * estimates best matching gesture
     * It tries several times by trimming the input trajectory.
     *
     * @param path
     * @return
     */
    protected String estimateGesture(ArrayList<ClusterPathPoint> path){
        if(path.size() < numPointsThreshold)
            return null;

        double pathLength = FeatureExtraction.calTrajectoryLength(path);

        String bmg = getBestmatchingGesture(path, -200, pathLength);
        if(bmg == null){
            for(int i = 1; i <= 2 ; i++){
                for(int j = 0; j<=1; j++){
                    // retries with the head trimming if failed
                    ArrayList<ClusterPathPoint> trimmedPath = trajectoryTrimming(path, i*headTrimmingPercents/2, j*tailTrimmingPercents, pathLength);
                    bmg = getBestmatchingGesture(trimmedPath, -200 + ((i-1)*2+j+1)*100, pathLength*(1.0-(double)(i*headTrimmingPercents/2 + j*tailTrimmingPercents)*0.01));

                    if(bmg != null)
                        return bmg;
                }
            }
        }

        return bmg;
    }

    /**
     * tries classification with the prevPath added
     *
     * @param path
     * @param timeDiffTolerenceUs
     * @param useTimeCons
     * @param useDistCons
     * @return
     */
    protected String estimateGestureWithPrevPath(ArrayList<ClusterPathPoint> path, int timeDiffTolerenceUs, boolean useTimeCons, boolean useDistCons){
        String bmg = null;

        if(doesAccumulate(path, timeDiffTolerenceUs, useTimeCons, useDistCons)){
            // saws two segments
            ArrayList<ClusterPathPoint> path2 = new ArrayList<ClusterPathPoint>();
            path2.addAll(prevPath);
            path2.addAll(path);
            bmg = estimateGesture(path2);
            prevPath = null; // makes it null since it's used
        }

        return bmg;
    }

    /**
     * returns true if the path has enough correlation with prevPath in time and distance
     *
     * @param path
     * @param timeDiffTolerenceUs
     * @param useTimeCons
     * @param useDistCons
     * @return
     */
    public boolean doesAccumulate(ArrayList<ClusterPathPoint> path, int timeDiffTolerenceUs, boolean useTimeCons, boolean useDistCons){
        boolean ret = false;

        if(prevPath != null){
            ClusterPathPoint lastPointPrevPath = prevPath.get(prevPath.size()-1);
            ClusterPathPoint firstPointPath = path.get(0);

            // better to get the mean position of the tail of the prevPath since it could be quite noisy
            Point2D.Float meanTailPos = new Point2D.Float(0,0);
            for(int i=0; i<prevPath.size()*0.05; i++){
                meanTailPos.x += prevPath.get(prevPath.size() - 1 - i).x;
                meanTailPos.y += prevPath.get(prevPath.size() - 1 - i).y;
            }
            meanTailPos.x /= (prevPath.size()*0.05);
            meanTailPos.y /= (prevPath.size()*0.05);

            // better to get the mean position of the head of the current path since it could be quite noisy
            Point2D.Float meanHeadPos = new Point2D.Float(0,0);
            for(int i=0; i<path.size()*0.05; i++){
                meanHeadPos.x += path.get(i).x;
                meanHeadPos.y += path.get(i).y;
            }
            meanHeadPos.x /= (path.size()*0.05);
            meanHeadPos.y /= (path.size()*0.05);

            // checks time diffence
            if(!useTimeCons || (firstPointPath.t - lastPointPrevPath.t) <= timeDiffTolerenceUs){
                // checks distance
                float distance = (float) FeatureExtraction.distance(meanHeadPos, meanTailPos);
                if(!useDistCons || distance <= chip.getSizeX()*0.4f)
                    ret = true;
                else{
//                    System.out.println("distance constraint : " + distance);
                    prevPath = null;
                }
            } else {
//                System.out.println("time constraint");
                prevPath = null;
            }
        } else {
//            System.out.println("null constraint");
        }

        return ret;
    }

    /**
     * returns the best matching gesture
     *
     * @param path
     * @param offset
     * @param totalTrajLen
     * @return
     */
    private String getBestmatchingGesture(ArrayList<ClusterPathPoint> path, int offset, double totalTrajLen){
        String[] codewards = null;
        String bmg = null;

        if(fve != null)
            codewards = fve.convTrajectoryToCodewords(path, totalTrajLen);
        else
            return null;

        if(hmmDP != null && hmmDP.ghmm != null && fve != null)
            bmg = hmmDP.ghmm.getBestMatchingGesture(codewards, fve.vectorAngleSeq);
        else
            return null;


        if(showRawTrace){
            // draws the quantized vectors
            if(offset == -200)
                hmmDP.clearImage();
            hmmDP.drawTrajectory(FeatureExtraction.convAnglesToTrajectoryInScaledArea(new Point2D.Float(hmmDP.centerX+offset, hmmDP.centerY+offset), hmmDP.centerY/2, fve.vectorAngleSeq));

            // draws the trajectory
            ArrayList<Point2D.Float> tmpPath = new ArrayList<Point2D.Float>();
            for(ClusterPathPoint pt:path)
                tmpPath.add(new Point2D.Float(pt.x*2 + 200 + offset, pt.y*2));
            hmmDP.drawTrajectoryDot(tmpPath);

            hmmDP.repaint();
            System.out.println(offset + ": " + bmg);
        }

        return bmg;
    }

    /**
     * puts an image on the screen based on the result of gesture recognition
     *
     * @param bmg
     * @param path
     * @return
     */
    protected boolean afterRecognitionProcess(String bmg, ArrayList<ClusterPathPoint> path){
        if(bmg == null)
            return false;

        boolean ret = true;

        savePath = false;
        if(login){
             if(bmg.startsWith("Infinite")){
                doLogout();
            } else if(bmg.startsWith("Push")){
                doPush();
            } else if(bmg.startsWith("SlashUp")){
                ret = doSlashUp(path, true);
                if(!ret)
                    savePath = true;
            } else {
                if(bmg.startsWith("SlashDown")){
                    doSlashDown(path);
                    savePath = true;
                } else {                   
                    // doesn't have consider refractory time for CW and CCW
                    if(bmg.startsWith("CW")){
                        doCW(path);
                    }else if(bmg.startsWith("CCW")){
                        doCCW(path);
                    }else if(bmg.startsWith("DCW")){
                        doDCW(path);
                    }else if(bmg.startsWith("DCCW")){
                        doDCCW(path);
                    }

                    // has to consider refractory time for Left, Right, Up, Down, and Check
                    // doesn't have to consider refractory time if checkActivated is true (i.e. SlashDown is detected) becase SlashDown is a partial gesture
                    if(checkActivated || startTimeGesture >= endTimePrevGesture + refractoryTimeMs*1000){
                        if(bmg.startsWith("Left")){
                            doLeft(path);
                        }else if(bmg.startsWith("Right")){
                            doRight(path);
                        }else if(bmg.startsWith("Up")){
                            doUp(path);
                        }else if(bmg.startsWith("Down")){
                            doDown(path);
                        }else if(bmg.startsWith("Check")){
                            doCheck(path);
                        }                        
                    } else {
                        endTimePrevGesture -= (refractoryTimeMs*1000);
                        savePath = true;
                        ret = false;
                    }

                    checkActivated = false;
                }
            }
        } else {
            if(bmg.startsWith("Infinite")){
                doLogin();
            }
        }

        return ret;
    }


    /**
     * selects the best trajectory from clusters
     *
     * @param cl
     * @return
     */
    protected ArrayList<ClusterPathPoint> selectClusterTrajectory(List<BlurringFilter2DTracker.Cluster> cl){
        ArrayList<ClusterPathPoint> selectedTrj = null;
        BlurringFilter2DTracker.Cluster selectedCluster = null;

        int maxNumPoint = 0;

        // select a candidate trajectory
        for (BlurringFilter2DTracker.Cluster c: cl){
            latestLocation.setLocation(c.location);

            // doesn't have to check alive cluster
            if (!c.isDead()){
                continue;
            }
          
            // checks number of points
            if(c.getPath().size() < numPointsThreshold){
                continue;
            } else {
                // search the largest cluster
                ArrayList<ClusterPathPoint> path = c.getPath();
                if(path.size() > maxNumPoint){
                    selectedTrj = path;
                    maxNumPoint = path.size();
                    selectedCluster = c;
                }
            }
        }
        if(selectedTrj == null)
            return null;
        
        // gesture speed check
//        if(!checkSpeedCriterion(selectedTrj)){
//            return null;
//        }

        // low-pass filtering
        ArrayList<ClusterPathPoint> retTrj = null;
        if(enableLPF)
            retTrj = lowPassFiltering(selectedTrj);
        else
            retTrj = selectedTrj;

        // records start and end time of the selected trajectory
        if(retTrj != null){
            startTimeGesture = selectedCluster.getBirthTime();
            endTimeGesture = selectedCluster.getLastEventTimestamp();
        }

        return retTrj;
    }


    /**
     * trims the trajectory in length base
     *
     * @param trajectory
     * @param headTrimmingPercets
     * @param tailTrimmingPercets
     * @param trjLength
     * @return
     */
    protected ArrayList<ClusterPathPoint> trajectoryTrimming(ArrayList<ClusterPathPoint> trajectory, int headTrimmingPercets, int tailTrimmingPercets, double trjLength){
        ArrayList<ClusterPathPoint> trimmedTrj;

        int numPointsHeadTrimming = 0;
        int numPointsTailTrimming = 0;

        // get # of points to trim in the head
        if(headTrimmingPercets > 2){
            numPointsHeadTrimming = (int) (trajectory.size()*0.02);
            double len = FeatureExtraction.calTrajectoryLengthTo(trajectory, numPointsHeadTrimming/2);
            numPointsHeadTrimming += FeatureExtraction.getTrajectoryPositionForward(trajectory, len + trjLength*0.01*(headTrimmingPercets-1));
        } else if (headTrimmingPercets > 0)
            numPointsHeadTrimming += FeatureExtraction.getTrajectoryPositionForward(trajectory, trjLength*0.01*headTrimmingPercets);
        else{}
        
        // get # of points to trim in the tail
        if(tailTrimmingPercets > 1){
            numPointsTailTrimming = (int) (trajectory.size()*0.02);
            double len = FeatureExtraction.calTrajectoryLengthFrom(trajectory, trajectory.size() - 1 - numPointsHeadTrimming/2);
            numPointsTailTrimming += trajectory.size() - 1 - FeatureExtraction.getTrajectoryPositionBackward(trajectory, len + trjLength*0.01*(tailTrimmingPercets-1));
        } else if (tailTrimmingPercets > 0)
            numPointsTailTrimming += trajectory.size() - 1 - FeatureExtraction.getTrajectoryPositionBackward(trajectory, trjLength*0.01*tailTrimmingPercets);
        else {}

        // do trimming
        if(numPointsHeadTrimming + numPointsTailTrimming > 0 && numPointsHeadTrimming + numPointsTailTrimming < trajectory.size()){
            trimmedTrj = new ArrayList<ClusterPathPoint>();
            for(int j=numPointsHeadTrimming; j<trajectory.size()-numPointsTailTrimming; j++)
                trimmedTrj.add(trajectory.get(j));
        } else
            trimmedTrj = trajectory;

        return trimmedTrj;
    }

    /**
     * trims the trajectory in point base
     *
     * @param trajectory
     * @param headTrimmingPercets
     * @param tailTrimmingPercets
     * @return
     */
    protected ArrayList<ClusterPathPoint> trajectoryTrimmingPointBase(ArrayList<ClusterPathPoint> trajectory, int headTrimmingPercets, int tailTrimmingPercets){
        ArrayList<ClusterPathPoint> trimmedTrj;

        int numPointsHeadTrimming = (int) (trajectory.size()*0.01*headTrimmingPercets);
        int numPointsTailTrimming = (int) (trajectory.size()*0.01*tailTrimmingPercets);

        if(numPointsHeadTrimming + numPointsTailTrimming > 0 && numPointsHeadTrimming + numPointsTailTrimming < trajectory.size()){
            trimmedTrj = new ArrayList<ClusterPathPoint>();
            for(int j=numPointsHeadTrimming; j<trajectory.size()-numPointsTailTrimming; j++)
                trimmedTrj.add(trajectory.get(j));
        } else
            trimmedTrj = trajectory;

        return trimmedTrj;
    }

    /**
     * does low-pass filtering to smoothe the trajectory
     *
     * @param path
     * @return
     */
    private ArrayList<ClusterPathPoint> lowPassFiltering(ArrayList<ClusterPathPoint> path){
        ArrayList<ClusterPathPoint> lpfPath = new ArrayList<ClusterPathPoint>();
        ClusterPathPoint p = (ClusterPathPoint) path.get(0).clone();

        lpfPath.add(p);
        lpf.setInternalValue2d(path.get(0).x, path.get(0).y);
        for(int i=1; i<path.size(); i++){
            p = (ClusterPathPoint) path.get(i).clone();
            Point2D.Float pt = lpf.filter2d(p.x, p.y, p.t);
            p.x = pt.x;
            p.y = pt.y;
            lpfPath.add(p);
        }

        return lpfPath;
    }

    /**
     * returns tracker
     * @return
     */
    public BlurringFilter2DTracker getTracker() {
        return tracker;
    }

    /**
     * returns latest location
     * @return
     */
    public Point2D.Float getLatestLocation(){
        return latestLocation;
    }

    /** returns numPointsThreshold
     *
     * @return
     */
    public int getNumPointsThreshold() {
        return numPointsThreshold;
    }

    /** sets numPointsThreshold
     *
     * @param numPointsThreshold
     */
    public void setNumPointsThreshold(int numPointsThreshold) {
        int old = this.numPointsThreshold;
        this.numPointsThreshold = numPointsThreshold;
        getPrefs().putInt("GestureBF2D.numPointsThreshold",numPointsThreshold);
        support.firePropertyChange("numPointsThreshold",old,this.numPointsThreshold);
    }

    /** returns headTrimmingPercents
     *
     * @return
     */
    public int getHeadTrimmingPercents() {
        return headTrimmingPercents;
    }

    /** sets headTrimmingPercents
     *
     * @param headTrimmingPercents
     */
    public void setHeadTrimmingPercents(int headTrimmingPercents) {
        int old = this.headTrimmingPercents;
        this.headTrimmingPercents = headTrimmingPercents;
        getPrefs().putInt("GestureBF2D.headTrimmingPercents",headTrimmingPercents);
        support.firePropertyChange("headTrimmingPercents",old,this.headTrimmingPercents);
    }

    /** returns tailTrimmingPercents
     *
     * @return
     */
    public int getTailTrimmingPercents() {
        return tailTrimmingPercents;
    }

    /** sets tailTrimmingPercents
     *
     * @param tailTrimmingPercents
     */
    public void setTailTrimmingPercents(int tailTrimmingPercents) {
        int old = this.tailTrimmingPercents;
        this.tailTrimmingPercents = tailTrimmingPercents;
        getPrefs().putInt("GestureBF2D.tailTrimmingPercents",tailTrimmingPercents);
        support.firePropertyChange("tailTrimmingPercents",old,this.tailTrimmingPercents);
    }

    /** returns enableLPF
     *
     * @return
     */
    public boolean isEnableLPF() {
        return enableLPF;
    }

    /** sets enableLPF
     * 
     * @param enableLPF
     */
    public void setEnableLPF(boolean enableLPF) {
        boolean old = this.enableLPF;
        this.enableLPF = enableLPF;
        getPrefs().putBoolean("GestureBF2D.enableLPF", enableLPF);
        support.firePropertyChange("enableLPF",old,this.enableLPF);
    }


    /**
     * @return the tauMs
     */
    public float getTauPathMs (){
        return tauPathMs;
    }

    /**
     * The lowpass time constant of the trajectory.
     *
     * @param tauPathMs the tauMs to set
     */
    synchronized public void setTauPathMs (float tauPathMs){
        float old = this.tauPathMs;
        this.tauPathMs = tauPathMs;
        getPrefs().putFloat("GestureBF2D.tauPathMs",tauPathMs);
        support.firePropertyChange("tauPathMs",old,this.tauPathMs);
        lpf.setTauMs(tauPathMs);
    }

    /**
     * returns refractoryTimeMs
     *
     * @return
     */
    public int getRefractoryTimeMs() {
        return refractoryTimeMs;
    }

    /**
     * sets refractoryTimeMs
     * 
     * @param refractoryTimeMs
     */
    public void setRefractoryTimeMs(int refractoryTimeMs) {
        int old = this.refractoryTimeMs;
        this.refractoryTimeMs = refractoryTimeMs;
        getPrefs().putInt("GestureBF2D.refractoryTimeMs", refractoryTimeMs);
        support.firePropertyChange("refractoryTimeMs",old,this.refractoryTimeMs);
    }

    /**
     * returns the Gaussian threshold criterion
     *
     * @return GTCriterion
     */
    public float getGTCriterion() {
        return GTCriterion;
    }

    /**
     * sets GTCriterion
     *
     * @param GTCriterion
     */
    public void setGTCriterion(float GTCriterion) {
        float old = this.GTCriterion;
        this.GTCriterion = GTCriterion;
        getPrefs().putFloat("GestureBF2D.GTCriterion", GTCriterion);
        support.firePropertyChange("GTCriterion",old,this.GTCriterion);

        hmmDP.setGTCriterion(GTCriterion);
    }


    /**
     * if true, considers prevPath in the gesture estimation process
     *
     * @return
     */
    public boolean isUsePrevPath() {
        return usePrevPath;
    }

    /**
     * sets usePrevPath
     *
     * @param usePrevPath
     */
    public void setUsePrevPath(boolean usePrevPath) {
        boolean old = this.usePrevPath;
        this.usePrevPath = usePrevPath;
        getPrefs().putBoolean("GestureBF2D.usePrevPath", usePrevPath);
        support.firePropertyChange("usePrevPath",old,this.usePrevPath);
    }

    /**
     * returns enableAutoLogout
     *
     * @return
     */
    public boolean isEnableAutoLogout() {
        return enableAutoLogout;
    }

    /**
     * sets enableAutoLogout
     *
     * @param enableAutoLogout
     */
    public void setEnableAutoLogout(boolean enableAutoLogout) {
        boolean old = this.enableAutoLogout;
        this.enableAutoLogout = enableAutoLogout;
        getPrefs().putBoolean("GestureBF2D.enableAutoLogout",enableAutoLogout);
        support.firePropertyChange("enableAutoLogout",old,this.enableAutoLogout);

        if(!enableAutoLogout && autoLogoutTimer.isRunning()){
            autoLogoutTimer.stop();
        }
    }


    /**
     * returns autoLogoutTimeMs
     *
     * @return
     */
    public int getAutoLogoutTimeMs() {
        return autoLogoutTimeMs;
    }

    /**
     * sets autoLogoutTimeMs
     *
     * @param autoLogoutTimeMs
     */
    public void setAutoLogoutTimeMs(int autoLogoutTimeMs) {
        int old = this.autoLogoutTimeMs;
        this.autoLogoutTimeMs = autoLogoutTimeMs;
        getPrefs().putInt("GestureBF2D.autoLogoutTimeMs",autoLogoutTimeMs);
        support.firePropertyChange("autoLogoutTimeMs",old,this.autoLogoutTimeMs);

        if(autoLogoutTimer.isRunning()){
            autoLogoutTimer.stop();
            autoLogoutTimer = new Timer(autoLogoutTimeMs, autoLogoutAction);
            autoLogoutTimer.start();
        } else {
            autoLogoutTimer = new Timer(autoLogoutTimeMs, autoLogoutAction);
        }
    }

    /**
     *
     * @return prevPath
     */
    public ArrayList<ClusterPathPoint> getPrevPath() {
        return prevPath;
    }

    /**
     * returns maxTimeDiffCorrSegmentsUs
     * @return
     */
    public int getMaxTimeDiffCorrSegmentsUs() {
        return maxTimeDiffCorrSegmentsUs;
    }

    /**
     * sets maxTimeDiffCorrSegmentsUs
     * 
     * @param maxTimeDiffCorrSegmentsUs
     */
    public void setMaxTimeDiffCorrSegmentsUs(int maxTimeDiffCorrSegmentsUs) {
        int old = this.maxTimeDiffCorrSegmentsUs;
        this.maxTimeDiffCorrSegmentsUs = maxTimeDiffCorrSegmentsUs;
        getPrefs().putInt("GestureBF2D.maxTimeDiffCorrSegmentsUs", maxTimeDiffCorrSegmentsUs);
        support.firePropertyChange("maxTimeDiffCorrSegmentsUs",old,this.maxTimeDiffCorrSegmentsUs);
    }

    public String getHmmSaveFilePath() {
        return HmmSaveFilePath;
    }

    public void setHmmSaveFilePath(String HmmSaveFilePath) {
        this.HmmSaveFilePath = HmmSaveFilePath;
        getPrefs().put("GestureBF2D.HmmSaveFilePath", HmmSaveFilePath);
    }

    /**
     * returns true if gesture recognition is enabled
     * @return
     */
    public boolean isEnableGestureRecognition() {
        return enableGestureRecognition;
    }

    /**
     * sets enableGestureRecognition
     * @param enableGestureRecognition
     */
    public void setEnableGestureRecognition(boolean enableGestureRecognition) {
        this.enableGestureRecognition = enableGestureRecognition;
    }

    /**
     * Class for HMM and GUI
     */
    class HmmDrawingPanel extends TrajectoryDrawingPanel implements ItemListener{
        /**
         * Button names
         */
        public final String REMOVE = "Remove";
        public final String ADD = "Add";
        public final String SHOW = "Show";
        public final String RESET = "Reset";
        public final String LEARN = "Learn";
        public final String GUESS = "Guess";

        /**
         * Optional HMM models
         */
        public final String ERGODIC = "ERGODIC";
        public final String LR = "LR";
        public final String LRB = "LRB";
        public final String LRC = "LRC";
        public final String LRBC = "LRBC";

        /**
         * Stores gesture names in a set to guarantee the uniqueness of names
         */
        public HashSet<String> gestureItems = new HashSet<String>();

        /**
         * combo box for choosing a gesture from the registered gesture set
         */
        protected JComboBox gestureChoice;
        /**
         * combo box for choosing a HMM model
         */
        protected JComboBox hmmModelChoice;
        /**
         * text field for entering the name of a new gesture to register
         */
        protected JTextField newGesture;
        /**
         * for saving and loading of gesture HMM
         */
        protected JFileChooser fileChooser;
        /**
         * make it true to manually activate gesture recognition system
         */
        protected JCheckBoxMenuItem checkGestureAction;
        /**
         * make it true to show gesture pictures when a gesture is detected
         */
        protected JCheckBoxMenuItem showGesturePicture;

        /**
         * if true, you can see the raw trace and feature vectors of trajectory on the drawing panel
         */
        protected JCheckBoxMenuItem showGestureRawTrace;

        /**
         * All gestures have the same number of states.
         */
        protected int numState = 5;

        /**
         * size of feature vector sequence
         */
        private int seqSize;

        /**
         *  Feature vector space consists of 16 quantized vectors.
         */
        protected final String[] featureVectorSpace = new String[] {"0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12", "13", "14", "15"};

        /**
         * use dynamic threshold model. If you set 'false' instead of 'true', you can use a static threshold model.
         */
        protected GestureHmm ghmm = null;  // = new GestureHmm(featureVectorSpace, GestureHmm.GAUSSIAN_THRESHOLD);

        /**
         * Output statement buffer
         */
        protected String msg = "";

        /**
         * x of the center x of image panel
         */
        protected float centerX = imgPanelWidth/2;
        /**
         * y of the center of image panel
         */
        protected float centerY = imgPanelHeight/2;
        /**
         * size of show panel
         */
        protected float showPanelSize = Math.min(centerX, centerY);

        /**
         * timer for image load
         */
        protected Timer timer;

        /**
         * folder
         */
        private String defaultFolder = "";

        /**
         * constructor
         * @param title
         * @param buttonNames
         */
        public HmmDrawingPanel(String title, String[] buttonNames) {
            super(title, 700, 700, buttonNames);

            //creates a file chooser
            fileChooser = new JFileChooser();

            // creates a timer
            timer = new Timer(700, clearImageAction);
            setGTCriterion(GTCriterion);
        }

        @Override
        public void buttonLayout(String[] componentNames) {
            gestureChoice = new JComboBox();
            gestureChoice.setName("gestureChoice");
            gestureChoice.addItem("Select a gesture");
            hmmModelChoice = new JComboBox();
            hmmModelChoice.setName("hmmModelChoice");
            hmmModelChoice.addItem("Select HMM model");
            hmmModelChoice.addItem(ERGODIC);
            hmmModelChoice.addItem(LR);
            hmmModelChoice.addItem(LRB);
            hmmModelChoice.addItem(LRC);
            hmmModelChoice.addItem(LRBC);
            newGesture = new JTextField();
            newGesture.setText("New gesture name");

            // configuration of button panel
            buttonPanel.setLayout(new GridLayout(2, (componentNames.length+3)/2));

            // adds gesture choice
            buttonPanel.add(gestureChoice, "1");
            gestureChoice.addItemListener(this);

            // adds new gesture name
            buttonPanel.add(newGesture, "2");

            // adds HMM model choice
            buttonPanel.add(hmmModelChoice, "3");
            hmmModelChoice.addItemListener(this);

            // adds buttons
            JButton newButton;
            for(int i = 0; i< componentNames.length; i++){
                newButton = new JButton(componentNames[i]);
                buttonPanel.add(newButton, ""+(i+4));
                newButton.addActionListener(buttonActionListener);
            }
            JButton clearButton = new JButton(clearButtonName);
            buttonPanel.add(clearButton, ""+ (componentNames.length + 4));
            clearButton.addActionListener(buttonActionListener);
        }

        @Override
        public void menuLayout() {
            // creates and adds drop down menus to the menu bar
            JMenu fileMenu = new JMenu("File");
            menuBar.add(fileMenu);
            JMenu gestureMenu = new JMenu("Gesture");
            menuBar.add(gestureMenu);

            // creates and adds menu items to menus
            JMenuItem newAction = new JMenuItem("New");
            JMenuItem loadAction = new JMenuItem("Load");
            JMenuItem saveAction = new JMenuItem("Save");
            fileMenu.add(newAction);
            fileMenu.add(loadAction);
            fileMenu.add(saveAction);

            // Create and add CheckButton for enabling gesture recognition
            checkGestureAction = new JCheckBoxMenuItem("Activate Gesture Recognition");
            checkGestureAction.setState(login);
            gestureMenu.add(checkGestureAction);

            // Create and add CheckButton for enabling gesture pictures
            showGesturePicture = new JCheckBoxMenuItem("Show Gesture pictures");
            showGesturePicture.setState(showGesturePic);
            gestureMenu.add(showGesturePicture);

            // Create and add CheckButton for enabling diagnosis view
            showGestureRawTrace = new JCheckBoxMenuItem("Show Gesture Raw Trace & Features");
            showGestureRawTrace.setState(showRawTrace);
            gestureMenu.add(showGestureRawTrace);

            // add action listeners
            newAction.addActionListener(menuActionListener);
            loadAction.addActionListener(menuActionListener);
            saveAction.addActionListener(menuActionListener);
            checkGestureAction.addActionListener(new ActionListener() {
                @Override
                public void actionPerformed(ActionEvent e) {
                    AbstractButton aButton = (AbstractButton) e.getSource();
                    if(aButton.getModel().isSelected()){
                        login = true;
                        System.out.println("Gesture recognition is mannually activated.");
                    }else{
                        doLogout();
                        System.out.println("Gesture recognition is mannually Inactivated.");
                    }

                    clearImage();
                }
            });
            showGesturePicture.addActionListener(new ActionListener() {
                @Override
                public void actionPerformed(ActionEvent e) {
                    AbstractButton aButton = (AbstractButton) e.getSource();
                    if(aButton.getModel().isSelected()){
                        showGesturePic = true;
                        System.out.println("Enabled to show gesture pictures.");
                    }else{
                        showGesturePic = false;
                        System.out.println("Disabled to show gesture pictures.");
                    }

                    clearImage();
                }
            });
            showGestureRawTrace.addActionListener(new ActionListener() {
                @Override
                public void actionPerformed(ActionEvent e) {
                    AbstractButton aButton = (AbstractButton) e.getSource();
                    if(aButton.getModel().isSelected()){
                        showRawTrace = true;
                        System.out.println("Enabled to show the raw data of trajectories.");
                    }else{
                        showRawTrace = false;
                        System.out.println("Disabled to show the raw data of trajectories.");
                    }

                    clearImage();
                }
            });
        }

        @Override
        public void buttonAction(String buttonName) {
            if(buttonName.equals(LEARN)){
                doLearn();
                clearImage();
            } else if(buttonName.equals(ADD)){
                doAddGesture();
            } else if(buttonName.equals(REMOVE)){
                doRemoveGesture();
            } else if(buttonName.equals(GUESS)){
                doGuess();
            } else if(buttonName.equals(RESET)){
                doReset();
                clearImage();
            } else if(buttonName.equals(SHOW)){
                doShow();
            }
        }

        @Override
        @SuppressWarnings("CallToThreadDumpStack")
        public void menuAction(String menuName) {
            if(menuName.equals("New")){
                doNew();
            } else if(menuName.equals("Load")){
                try{
                    doLoad();
                } catch(ClassNotFoundException e){
                    e.printStackTrace();
                }
            } else if(menuName.equals("Save")){
                doSave();
            }

            repaint();
        }


        /**
         * excutes Remove button
         */
        public void doRemoveGesture(){
            String gesName = (String) gestureChoice.getSelectedItem();
            if(gesName == null || gesName.equals("") || gesName.equals("Select a gesture")){
                System.out.println("Warning: Gesture is not selected.");
                return;
            }

            ghmm.removeGesture(gesName);
            gestureChoice.removeItem(gesName);
            gestureItems.remove(gesName);
            System.out.println(gesName + " was removed.");
        }

        /**
         * excutes Add button
         */
        public void doAddGesture(){
            String newGestName = newGesture.getText();
            if(newGestName.equals("")){
                System.out.println("Warning: Gesture name is not specified.");
                return;
            }

            if(((String) hmmModelChoice.getSelectedItem()).startsWith("Select HMM model")) {
                System.out.println("Warning: HMM model is not specified.");
                return;
            }

            String gestName = newGestName+"_"+hmmModelChoice.getSelectedItem();

            if(ghmm == null){
                doNew();
            }

            if(!gestureItems.contains(gestName)){
                String numStr = JOptionPane.showInputDialog(this, "Input the numver of states for the gesture model?");
                int numStates = Integer.parseInt(numStr);

                gestureItems.add(gestName);
                gestureChoice.addItem(gestName);
                HiddenMarkovModel.ModelType selectedModel;
                if(hmmModelChoice.getSelectedItem().equals("ERGODIC"))
                    selectedModel = HiddenMarkovModel.ModelType.ERGODIC_RANDOM;
                else if(hmmModelChoice.getSelectedItem().equals("LR"))
                    selectedModel = HiddenMarkovModel.ModelType.LR_RANDOM;
                else if(hmmModelChoice.getSelectedItem().equals("LRB"))
                    selectedModel = HiddenMarkovModel.ModelType.LRB_RANDOM;
                else if(hmmModelChoice.getSelectedItem().equals("LRC"))
                    selectedModel = HiddenMarkovModel.ModelType.LRC_RANDOM;
                else if(hmmModelChoice.getSelectedItem().equals("LRBC"))
                    selectedModel = HiddenMarkovModel.ModelType.LRBC_RANDOM;
                else{
                    System.out.println("Warning: Failed to add a new gesture.");
                    return;
                }

                ghmm.addGesture(gestName, numStates,  selectedModel);
                ghmm.initializeGestureRandom(gestName);

                System.out.println("A new gesture ("+ gestName + ") is added.");
            }
            gestureChoice.setSelectedItem(gestName);
            newGesture.setText("");
        }

        /**
         * excutes Learn button
         */
        public void doLearn(){
            String gesName = (String) gestureChoice.getSelectedItem();
            if(gesName == null || gesName.equals("") || gesName.equals("Select a gesture")){
                System.out.println("Warning: Gesture is not selected.");
                return;
            }

            String[] fv = fve.convTrajectoryToCodewords(trajectory, -1);
            if(fv[0] == null){
                System.out.println("Warning: No trajectory is dected.");
                return;
            }
            System.out.println("Learning " + gesName);

            boolean learningSuccess;
            HiddenMarkovModel.ModelType modelType = ghmm.getGestureHmm(gesName).getModelType();

            // for LRC & LRBC, we don't have to update start probability
            if(modelType == HiddenMarkovModel.ModelType.LRC_RANDOM ||  modelType == HiddenMarkovModel.ModelType.LRBC_RANDOM)
                learningSuccess = ghmm.learnGesture(gesName, fv, fve.vectorAngleSeq, false, true, true);
            else
                learningSuccess = ghmm.learnGesture(gesName, fv, fve.vectorAngleSeq, true, true, true);

            if(learningSuccess){
                if(ghmm.getGestureHmm(gesName).getNumTraining() == 1)
                    System.out.println(gesName+" is properly registered. Log{P(O|model)} = " + Math.log10(ghmm.getGestureLikelyhood(gesName, fv)));
                else if(ghmm.getGestureHmm(gesName).getNumTraining() == 2)
                    System.out.println(gesName+" has been trained twice. Log{P(O|model)} = " + Math.log10(ghmm.getGestureLikelyhood(gesName, fv)));
                else
                    System.out.println(gesName+" has been trained " + ghmm.getGestureHmm(gesName).getNumTraining() + " times. Log{P(O|model)} = " + Math.log10(ghmm.getGestureLikelyhood(gesName, fv)));

//                ghmm.printGesture(gesName);
//                ghmm.printThresholdModel();
//                ghmm.getGestureHmm(gesName).viterbi(fv);
//                System.out.println("Viterbi path : " + ghmm.getGestureHmm(gesName).getViterbiPathString(fv.length));
            }
        }

        /**
         * excutes Guess button
         */
        public void doGuess(){
            String[] fv = fve.convTrajectoryToCodewords(trajectory, -1);

            if(fv[0] == null){
                System.out.println("Warning: No trajectory is dected.");
                return;
            }

            String bmg = ghmm.getBestMatchingGesture(fv, fve.vectorAngleSeq);
            gImg.setFont(new Font("Arial", Font.PLAIN, 24));

            // erase previous message
            Color tmpColor = getColor();
            gImg.setColor(this.getBackground());
            gImg.drawString(msg, 40 + imgPanelWidth/2 - msg.length()*12/2, imgPanelHeight - 20);
            gImg.setColor(tmpColor);

            if(bmg == null){
                msg = "No gesture is found.";
                System.out.println(msg);
            }else{
                msg = String.format("Best matching gesture is %s", bmg);
                System.out.println(msg +" with probability "+Math.log10(ghmm.getGestureLikelyhood(bmg, fv)));
//                ghmm.getGestureHmm(bmg).viterbi(fv);
//                System.out.println("Viterbi path : " + ghmm.getGestureHmm(bmg).getViterbiPathString(fv.length));
            }
            gImg.drawString(msg, 40 + imgPanelWidth/2 - msg.length()*12/2, imgPanelHeight - 20);
            repaint();

            resetTrajectory();
        }


        /**
         * excutes Show button
         */
        public void doShow(){
            String gesName = (String) gestureChoice.getSelectedItem();
            if(gesName == null || gesName.equals("") || gesName.equals("Select a gesture")){
                System.out.println("Warning: Gesture is not selected.");
                return;
            }

            double[] meanFVarray = ghmm.getAverageFeaturesToArray(gesName);

            clearImage();

            // draws frame
            int margin = 30;
            int shadow = 10;
            Color tmp = getColor();
            gImg.setColor(Color.DARK_GRAY);
            gImg.fillRect((int) (centerX - showPanelSize/2) - margin + shadow, (int) (centerY - showPanelSize/2) - margin + shadow, (int) showPanelSize + 2*margin, (int) showPanelSize + 2*margin);
            gImg.setColor(Color.WHITE);
            gImg.fillRect((int) (centerX - showPanelSize/2) - margin, (int) (centerY - showPanelSize/2) - margin, (int) showPanelSize + 2*margin, (int) showPanelSize + 2*margin);
            gImg.setColor(Color.BLACK);
            gImg.drawRect((int) (centerX - showPanelSize/2) - margin, (int) (centerY - showPanelSize/2) - margin, (int) showPanelSize + 2*margin, (int) showPanelSize + 2*margin);
            gImg.setFont(new Font("Arial", Font.PLAIN, 24));
            gImg.drawString(gesName+" (# of training: "+ghmm.getGestureHmm(gesName).getNumTraining()+")", (int) centerX - (int) showPanelSize/2 - margin, (int) centerY - (int) showPanelSize/2 - margin - 10);
            gImg.setColor(tmp);

            // draws trajectory
            if(ghmm.getGestureHmm(gesName).getNumTraining() > 0)
                drawTrajectory(FeatureExtraction.convAnglesToTrajectoryInScaledArea(new Point2D.Float(centerX, centerY), showPanelSize, meanFVarray));
            else
                gImg.drawString("Hey, man.", (int) centerX - 50, (int) centerY);

            repaint();

            // prints HMM characteristics
            ghmm.getGestureHmm(gesName).printAllProbability();
            System.out.println("\n------------   Gaussian Threshold   --------------");
            System.out.print("mu:");
            for(double mu:ghmm.getGTModel(gesName).getMuToArray())
                System.out.print(mu+", ");
            System.out.print("\nsigma:");
            for(double sigma:ghmm.getGTModel(gesName).getSigmaToArray())
                System.out.print(sigma+", ");
            System.out.println();
        }

        /**
         * excutes Reset button
         */
        public void doReset(){
            String gesName = (String) gestureChoice.getSelectedItem();
            if(gesName == null || gesName.equals("") || gesName.equals("Select a gesture")){
                System.out.println("Warning: Gesture is not selected.");
                return;
            }

            ghmm.resetGesture(gesName);
            System.out.println(gesName + " is reset now.");
        }

        /**
         * excutes New menu
         */
        public void doNew(){
            String numStr = JOptionPane.showInputDialog(this, "Input the numver of observation sequence for classification?");

            if(numStr == null || numStr.equals(""))
                return;

            seqSize = Integer.parseInt(numStr);

            fve = new FeatureExtraction(featureVectorSpace.length, seqSize);
            ghmm = new GestureHmm(featureVectorSpace, GestureHmm.GAUSSIAN_THRESHOLD, seqSize);
            ghmm.setGTCriterion(GTCriterion);
            gestureItems.clear();
            gestureChoice.removeAllItems();
            gestureChoice.addItem("Select a gesture");

            System.out.println("Created a new gesture set.");
      }

        /**
         * excutes Save menu
         */
        @SuppressWarnings("CallToThreadDumpStack")
        public void doSave(){
            if(ghmm == null){
                log.log(Level.WARNING, "There is nothing to save.");
                return;
            }

            int returnVal = fileChooser.showSaveDialog(HmmDrawingPanel.this);
            if (returnVal == JFileChooser.APPROVE_OPTION) {
                File file = fileChooser.getSelectedFile();

                // do saving things here
                try{
                    FileOutputStream fos = new FileOutputStream(file.getAbsoluteFile());
                    BufferedOutputStream bos = new BufferedOutputStream(fos);
                    ObjectOutputStream oos = new ObjectOutputStream(bos);

                    oos.writeObject(ghmm);
                    oos.close();
                    log.log(Level.WARNING, "Gesture HMM has been saved in {0}", file.getAbsoluteFile());
                } catch (IOException e){
                    e.printStackTrace();
                }
            } else {
                // canceled
            }
        }

        /**
         * excutes Save menu
         *
         * @throws ClassNotFoundException
         */
        @SuppressWarnings("CallToThreadDumpStack")
        public void doLoad() throws ClassNotFoundException{
            int returnVal = fileChooser.showOpenDialog(HmmDrawingPanel.this);

            if (returnVal == JFileChooser.APPROVE_OPTION) {
                setHmmSaveFilePath(fileChooser.getSelectedFile().getAbsolutePath());
                openFile(HmmSaveFilePath);
/*
                // do loading things here
                try{
                    FileInputStream fis = new FileInputStream(file.getAbsoluteFile());
                    BufferedInputStream bis = new BufferedInputStream(fis);
                    ObjectInputStream ois = new ObjectInputStream(bis);

                    ghmm = (GestureHmm) ois.readObject();
                    seqSize = ghmm.getSeqSize();
                    ghmm.setGTCriterion(GTCriterion);

                    // for backward compatibility
                    if(seqSize == 0)
                        seqSize = 16;

                    fve = new FeatureExtraction(featureVectorSpace.length, seqSize);
                    gestureItems.clear();
                    gestureItems.addAll(ghmm.getGestureNames());
                    gestureChoice.removeAllItems();
                    gestureChoice.addItem("Select a gesture");
                    for(String gname:gestureItems)
                        gestureChoice.addItem(gname);

                    ois.close();
                    System.out.println("Gesture HMM has been loaded from " + file.getAbsoluteFile());
                    System.out.println("----------- Summary of gesture models -------------");
                    System.out.println("Size of feature vector sequence : " + seqSize);
                    System.out.println("Number of gestures : " + ghmm.getNumGestures());
                    System.out.println("Defined gestures (# of states) : ");
                    for(String gestName:ghmm.getGestureNames()){
                        System.out.println("\t"+gestName+"("+ghmm.getGestureHmm(gestName).getNumStates()+")");
                    }
                    System.out.println("-----------       End of summary      -------------");


//                    for(int i=0; i<16; i++)
//                        System.out.println("sigma(" + i + ") = " + ghmm.getGTModel("Left_LRB").getSigma(i));
//                    String[] bestSeq = new String[] {"15", "8", "10", "4", "15", "8", "10", "4", "15", "8", "10", "4", "15", "8", "10", "4"};
//                    String[] idealSeq = new String[] {"12", "13", "14", "15", "0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11"};
//                    String[] localSeq = new String[] {"12", "13", "14", "15", "0", "1", "2", "3", "13", "14", "15", "0", "1", "2", "3", "4"};
//                    System.out.println("bestSeq = " + ghmm.getBestMatchingGesture(bestSeq) + ", " + ghmm.getGestureHmm("CW_LRBC").forward(bestSeq));
//                    System.out.println("idealSeq = " + ghmm.getBestMatchingGesture(bestSeq) + ", " + ghmm.getGestureHmm("CW_LRBC").forward(idealSeq));
//                    System.out.println("localSeq = " + ghmm.getBestMatchingGesture(bestSeq) + ", " + ghmm.getGestureHmm("CW_LRBC").forward(localSeq));
//                    double[] bestSeqAng = {5.890486225, 3.141592654, 3.926990817, 1.570796327, 5.890486225, 3.141592654, 3.926990817, 1.570796327,
//                                            5.890486225, 3.141592654, 3.926990817, 1.570796327, 5.890486225, 3.141592654, 3.926990817, 1.570796327};
//                    drawTrajectory(FeatureExtraction.convAnglesToTrajectoryInScaledArea(new Point2D.Float(centerX, centerY), showPanelSize, bestSeqAng));
                } catch (IOException e){
                    e.printStackTrace();
                }
 */           } else {
                // canceled
            }
        }

        protected void openFile(String path){
            // do loading things here
            try{
                File file = new File(path);
                FileInputStream fis = new FileInputStream(file.getAbsoluteFile());
                BufferedInputStream bis = new BufferedInputStream(fis);
                ObjectInputStream ois = new ObjectInputStream(bis);
                try {
                    ghmm = (GestureHmm) ois.readObject();
                } catch (ClassNotFoundException ex) {
                    Logger.getLogger(GestureBF2D.class.getName()).log(Level.SEVERE, null, ex);
                    return;
                }

                seqSize = ghmm.getSeqSize();
                ghmm.setGTCriterion(GTCriterion);

                // for backward compatibility
                if(seqSize == 0)
                    seqSize = 16;


                fve = new FeatureExtraction(featureVectorSpace.length, seqSize);
                gestureItems.clear();
                gestureItems.addAll(ghmm.getGestureNames());
                gestureChoice.removeAllItems();
                gestureChoice.addItem("Select a gesture");
                for(String gname:gestureItems)
                    gestureChoice.addItem(gname);

                ois.close();
                System.out.println("Gesture HMM has been loaded from " + file.getAbsoluteFile());
                System.out.println("----------- Summary of gesture models -------------");
                System.out.println("Size of feature vector sequence : " + seqSize);
                System.out.println("Number of gestures : " + ghmm.getNumGestures());
                System.out.println("Defined gestures (# of states) : ");
                for(String gestName:ghmm.getGestureNames()){
                    System.out.println("\t"+gestName+"("+ghmm.getGestureHmm(gestName).getNumStates()+")");
                }
                System.out.println("-----------       End of summary      -------------");


//                    for(int i=0; i<16; i++)
//                        System.out.println("sigma(" + i + ") = " + ghmm.getGTModel("Left_LRB").getSigma(i));
//                    String[] bestSeq = new String[] {"15", "8", "10", "4", "15", "8", "10", "4", "15", "8", "10", "4", "15", "8", "10", "4"};
//                    String[] idealSeq = new String[] {"12", "13", "14", "15", "0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11"};
//                    String[] localSeq = new String[] {"12", "13", "14", "15", "0", "1", "2", "3", "13", "14", "15", "0", "1", "2", "3", "4"};
//                    System.out.println("bestSeq = " + ghmm.getBestMatchingGesture(bestSeq) + ", " + ghmm.getGestureHmm("CW_LRBC").forward(bestSeq));
//                    System.out.println("idealSeq = " + ghmm.getBestMatchingGesture(bestSeq) + ", " + ghmm.getGestureHmm("CW_LRBC").forward(idealSeq));
//                    System.out.println("localSeq = " + ghmm.getBestMatchingGesture(bestSeq) + ", " + ghmm.getGestureHmm("CW_LRBC").forward(localSeq));
//                    double[] bestSeqAng = {5.890486225, 3.141592654, 3.926990817, 1.570796327, 5.890486225, 3.141592654, 3.926990817, 1.570796327,
//                                            5.890486225, 3.141592654, 3.926990817, 1.570796327, 5.890486225, 3.141592654, 3.926990817, 1.570796327};
//                    drawTrajectory(FeatureExtraction.convAnglesToTrajectoryInScaledArea(new Point2D.Float(centerX, centerY), showPanelSize, bestSeqAng));

            } catch (IOException e){
                e.printStackTrace();
            }
        }

        /**
         * puts an image on the drawing panel
         *
         * @param img
         */
        public void putImage(Image img){
            clearImage();
            gImg.drawImage(img, (int) centerX - img.getWidth(this)/2, (int) centerY - img.getHeight(this)/2, this);
            repaint();

            if(timer.isRunning())
                timer.restart();
            else
                timer.start();
        }

        /**
         * action listener for timer events
         */
        ActionListener clearImageAction = new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent evt) {
               clearImage();
               timer.stop();
            }
        };

        @Override
        protected void initialDeco() {
            super.initialDeco();
            Color tmpColor = getColor();
            Font tmpFont = getFont();
            gImg.setFont(new Font("Arial", Font.BOLD|Font.ITALIC, 20));
            if(login){
                gImg.setColor(Color.RED);
                gImg.drawString("Active", imgPanelWidth - 100, 20);
            }else{
                gImg.setColor(Color.GRAY);
                gImg.drawString("Inactive", imgPanelWidth - 100, 20);
            }
            gImg.setColor(tmpColor);
            gImg.setFont(tmpFont);

        }


        /**
         * processes Choice events
         * @param e
         */
        @Override
        public void itemStateChanged(ItemEvent e) {
            if(String.valueOf(e.getSource()).contains("gestureChoice")){
                if(e.getStateChange() == ItemEvent.SELECTED && !String.valueOf(e.getItem()).equals("Select a gesture")){
                    System.out.println("Gesture selection : " + e.getItem() + " is selected.");
                }
            } else {
                if(e.getStateChange() == ItemEvent.SELECTED && !String.valueOf(e.getItem()).equals("Select HMM model")){
                    System.out.println("HMM model selection: " + e.getItem() + " is selected.");
                }
            }
        }

        @Override
        public void windowClosing(WindowEvent we) {
            // set the window just invisible
            hmmDP.setVisible(false);
        }

        public final void setGTCriterion(float criterion) {
            if(ghmm !=null )
                ghmm.setGTCriterion(criterion);
        }
    }

    /**
     * returns true if login is true
     *
     * @return
     */
    public boolean isLogin() {
        return login;
    }

    /**
     *
     */
    public final void ignoreGesture(){
        endTimePrevGesture -= refractoryTimeMs*1000;
    }


    /**
     * defines the jobs to do during login
     */
    protected void doLogin(){
        if(showGesturePic && !showRawTrace)
            hmmDP.putImage(imgHi);
        login = true;
        hmmDP.checkGestureAction.setState(login);

        // starts the auto logout timer
        autoLogoutTimer.start();
    }

    /**
     * defines the jobs to do during logout
     */
    protected void doLogout(){
        if(showGesturePic && !showRawTrace)
            hmmDP.putImage(imgBye);
        login = false;
        hmmDP.checkGestureAction.setState(login);

        // stop auto-logout timer
        if(autoLogoutTimer.isRunning())
            autoLogoutTimer.stop();
    }

    /**
     * defines the jobs to do when Push is detected
     */
    protected void doPush(){
        // for stereo vision
//        hmmDP.putImage(imgPush);
    }

    /**
     * defines the jobs to do when SlashUp is detected
     *
     * @param path
     * @return
     */
    private boolean doSlashUp(ArrayList<ClusterPathPoint> path, boolean realPath){
        boolean ret = true;

        // checks over-segmentation
        if(prevPath != null){
            System.out.print("Check the previous segment ---> ");

            if(tryGestureWithPrevPath(path, 20, "Check", maxTimeDiffCorrSegmentsUs, true, false))
            {
                System.out.println("Check");
                checkActivated = true;
            } else {
                System.out.println("null");
                ret = false;
            }
        }

        if(ret && checkActivated){
            if(prevPath != null){
                ArrayList<ClusterPathPoint> tmpPath = new ArrayList<ClusterPathPoint> ();
                tmpPath.addAll(prevPath);
                tmpPath.addAll(path);
                addGestureToQ(new Gesture("Check", tmpPath.get(0).t, tmpPath.get(tmpPath.size()-1).t, (float) FeatureExtraction.calTrajectoryLength(tmpPath), 0.0f, new Point2D.Float(tmpPath.get(0).x, tmpPath.get(0).y), new Point2D.Float(tmpPath.get(tmpPath.size()-1).x, tmpPath.get(tmpPath.size()-1).y)));
            }else{
                // I don't know this is really necessary.
                addGestureToQ(new Gesture("Check", path.get(0).t, path.get(path.size()-1).t, (float) FeatureExtraction.calTrajectoryLength(path), 0.0f, new Point2D.Float(path.get(0).x, path.get(0).y), new Point2D.Float(path.get(path.size()-1).x, path.get(path.size()-1).y)));
            }
            doCheck();
        } else{
            if(realPath){
                addGestureToQ(new Gesture("SlashUp", path.get(0).t, path.get(path.size()-1).t, (float) FeatureExtraction.calTrajectoryLength(path), (float) fve.getAverageAngle(0.2, 0.8), new Point2D.Float(path.get(0).x, path.get(0).y), new Point2D.Float(path.get(path.size()-1).x, path.get(path.size()-1).y)));
                doSlashUp();
            } else
                ret = false;
        }

        checkActivated = false;

        return ret;
    }

    /**
     * defines the jobs to do when SlashUp is detected
     */
    protected void doSlashUp(){
        // do nothing
    }

    /**
     * defines the jobs to do when SlashDown is detected
     */
    private void doSlashDown(ArrayList<ClusterPathPoint> path){
        addGestureToQ(new Gesture("SlashDown", path.get(0).t, path.get(path.size()-1).t, (float) FeatureExtraction.calTrajectoryLength(path), (float) fve.getAverageAngle(0.2, 0.8), new Point2D.Float(path.get(0).x, path.get(0).y), new Point2D.Float(path.get(path.size()-1).x, path.get(path.size()-1).y)));

        checkActivated = true;
        doSlashDown();
    }

    /**
     * defines the jobs to do when SlashDown is detected
     */
    protected void doSlashDown(){
        // do nothing
    }

    /**
     * checks CW to detect broken infinite shaped gestures
     *
     * @param path
     */
    private void doCW(ArrayList<ClusterPathPoint> path){
        // to detect broken infinite shaped gestures
        if(prevPath != null && tryGestureWithPrevPath(path, 0, "Infinite", maxTimeDiffCorrSegmentsUs, true, false)){
            System.out.println("----------> might be an inifnite-shaped gesture");
            doLogout();
        } else {
            addGestureToQ(new Gesture("CW", path.get(0).t, path.get(path.size()-1).t, (float) FeatureExtraction.calTrajectoryLength(path), 0.0f, new Point2D.Float(path.get(0).x, path.get(0).y), new Point2D.Float(path.get(path.size()-1).x, path.get(path.size()-1).y)));
            doCW();
        }
    }

    /**
     * defines the jobs to do when CW is detected
     */
    protected void doCW(){
        if(showGesturePic && !showRawTrace)
            hmmDP.putImage(imgCW);
    }

    /**
     * checks CCW to detect broken infinite shaped gestures
     *
     * @param path
     */
    private void doCCW(ArrayList<ClusterPathPoint> path){
        // to detect broken infinite shaped gestures
        if(prevPath != null && tryGestureWithPrevPath(path, 0, "Infinite", maxTimeDiffCorrSegmentsUs, true, false)){
            System.out.println("----------> might be an inifnite-shaped gesture");
            doLogout();
        } else {
            addGestureToQ(new Gesture("CCW", path.get(0).t, path.get(path.size()-1).t, (float) FeatureExtraction.calTrajectoryLength(path), 0.0f, new Point2D.Float(path.get(0).x, path.get(0).y), new Point2D.Float(path.get(path.size()-1).x, path.get(path.size()-1).y)));
            doCCW();
        }
    }

    /**
     * defines the jobs to do when CCW is detected
     */
    protected void doCCW(){
        if(showGesturePic && !showRawTrace)
            hmmDP.putImage(imgCCW);
    }

    /**
     * defines the jobs to do when Left is detected
     */
    protected void doLeft(){
        if(showGesturePic && !showRawTrace)
            hmmDP.putImage(imgLeft);
    }

    /**
     * defines the jobs to do when Left is detected
     */
    private void doLeft(ArrayList<ClusterPathPoint> path){
        addGestureToQ(new Gesture("Left", path.get(0).t, path.get(path.size()-1).t, (float) FeatureExtraction.calTrajectoryLength(path), (float) fve.getAverageAngle(0.2, 0.8), new Point2D.Float(path.get(0).x, path.get(0).y), new Point2D.Float(path.get(path.size()-1).x, path.get(path.size()-1).y)));
        doLeft();
    }

    /**
     * defines the jobs to do when Right is detected
     */
    protected void doRight(){
        if(showGesturePic && !showRawTrace)
            hmmDP.putImage(imgRight);
    }

    /**
     * defines the jobs to do when Right is detected
     * @param path
     */
    protected void doRight(ArrayList<ClusterPathPoint> path){
        addGestureToQ(new Gesture("Right", path.get(0).t, path.get(path.size()-1).t, (float) FeatureExtraction.calTrajectoryLength(path), (float) fve.getAverageAngle(0.2, 0.8), new Point2D.Float(path.get(0).x, path.get(0).y), new Point2D.Float(path.get(path.size()-1).x, path.get(path.size()-1).y)));
        doRight();
    }

    /**
     * defines the jobs to do when Up is detected
     * @param path
     */
    private void doUp(ArrayList<ClusterPathPoint> path){
        boolean thisIsUp = true;
        if(fve.getAverageAngle(0.2, 0.8) > Math.PI*0.56){
            if(doSlashUp(path, false)){
                thisIsUp = false;
            }
        }

        if(thisIsUp){
            addGestureToQ(new Gesture("Up", path.get(0).t, path.get(path.size()-1).t, (float) FeatureExtraction.calTrajectoryLength(path), (float) fve.getAverageAngle(0.2, 0.8), new Point2D.Float(path.get(0).x, path.get(0).y), new Point2D.Float(path.get(path.size()-1).x, path.get(path.size()-1).y)));
            doUp();
        }
    }

    /**
     * defines the jobs to do when Up is deteced
     */
    protected void doUp(){
        if(showGesturePic && !showRawTrace)
            hmmDP.putImage(imgUp);
    }

    /**
     * defines the jobs to do when Down is detected
     */
    protected void doDown(){
        if(showGesturePic && !showRawTrace)
            hmmDP.putImage(imgDown);
    }

    /**
     * defines the jobs to do when Down is detected
     * @param path
     */
    protected void doDown(ArrayList<ClusterPathPoint> path){
        addGestureToQ(new Gesture("Down", path.get(0).t, path.get(path.size()-1).t, (float) FeatureExtraction.calTrajectoryLength(path), (float) fve.getAverageAngle(0.2, 0.8), new Point2D.Float(path.get(0).x, path.get(0).y), new Point2D.Float(path.get(path.size()-1).x, path.get(path.size()-1).y)));
        doDown();
    }

    /**
     * defines the jobs to do when Check is detected
     */
    protected void doCheck(){
        if(showGesturePic && !showRawTrace)
            hmmDP.putImage(imgCheck);
    }

    /**
     * defines the jobs to do when Check is detected
     * @param path
     */
    protected void doCheck(ArrayList<ClusterPathPoint> path){
        addGestureToQ(new Gesture("Check", path.get(0).t, path.get(path.size()-1).t, (float) FeatureExtraction.calTrajectoryLength(path), 0.0f, new Point2D.Float(path.get(0).x, path.get(0).y), new Point2D.Float(path.get(path.size()-1).x, path.get(path.size()-1).y)));
        doCheck();
    }

    /**
     * defines the jobs to do when DCW is detected
     *
     * @param path
     */
    private void doDCW(ArrayList<ClusterPathPoint> path){
        addGestureToQ(new Gesture("DCW", path.get(0).t, path.get(path.size()-1).t, (float) FeatureExtraction.calTrajectoryLength(path), 0.0f, new Point2D.Float(path.get(0).x, path.get(0).y), new Point2D.Float(path.get(path.size()-1).x, path.get(path.size()-1).y)));
        doDCW();
    }

    /**
     * defines the jobs to do when DCW is detected
     */
    protected void doDCW(){
        if(showGesturePic && !showRawTrace)
            hmmDP.putImage(imgDCW);
    }

    /**
     * defines the jobs to do when DCCW is detected
     *
     * @param path
     */
    private void doDCCW(ArrayList<ClusterPathPoint> path){
        addGestureToQ(new Gesture("DCCW", path.get(0).t, path.get(path.size()-1).t, (float) FeatureExtraction.calTrajectoryLength(path), 0.0f, new Point2D.Float(path.get(0).x, path.get(0).y), new Point2D.Float(path.get(path.size()-1).x, path.get(path.size()-1).y)));
        doDCCW();
    }

    /**
     * defines the jobs to do when DCCW is detected
     */
    protected void doDCCW(){
        if(showGesturePic && !showRawTrace)
            hmmDP.putImage(imgDCCW);
    }

    /**
     * puts a new gesture into the Q
     * @param inGesture
     */
    public void addGestureToQ(Gesture inGesture){
        gestureQueue.push(inGesture);
        if(gestureQueue.size() > maxGestureQueue)
            gestureQueue.removeLast();
    }

    /**
     * clear the gesture Q
     */
    public void resetGestureQ(){
        gestureQueue.clear();
    }

    /**
     * retieves a gesture from the Q
     * @param pos
     * @return
     */
    public Gesture getGestureFromQ(int pos){
        if(pos >= gestureQueue.size())
            return null;

        return gestureQueue.get(pos);
    }

    /**
     * retieves the most recent gesture from the Q
     * @return
     */
    public Gesture getGestureFromQ(){
        return gestureQueue.getFirst();
    }

}


