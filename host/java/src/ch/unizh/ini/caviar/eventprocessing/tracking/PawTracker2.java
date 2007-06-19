/*
 * PawTracker2.java
 * Tracks the paw of a rat in the grasping task experiment. see [ref]
 * ( different approach than PawTracker '1' )
 *
 * This class finds the contour of the paw in the accumulation of incoming events
 * After low-pass filter
 * Then detection of fingers tips by template matching
 * + alternative algorithms trials
 *
 * work under (a lot of) progress
 *
 * Paul Rogister, Created on May, 2007
 *
 */


/** to do : */


package ch.unizh.ini.caviar.eventprocessing.tracking;
import ch.unizh.ini.caviar.aemonitor.AEConstants;
import ch.unizh.ini.caviar.chip.*;
import ch.unizh.ini.caviar.eventprocessing.EventFilter2D;
import ch.unizh.ini.caviar.event.*;
import ch.unizh.ini.caviar.event.EventPacket;
import ch.unizh.ini.caviar.graphics.*;
import com.sun.opengl.util.*;
import java.awt.*;
//import ch.unizh.ini.caviar.util.PreferencesEditor;
import java.awt.geom.*;
import java.io.*;
import java.util.*;
import java.util.prefs.*;
import javax.media.opengl.*;
import javax.media.opengl.GL;
import javax.media.opengl.GLAutoDrawable;
import java.lang.Math.*;
import javax.swing.*;
import javax.media.opengl.glu.GLU;


/**
 * Tracks Rat's Paw
 *<p>
 * New angle of view.
 * Accumulate event then try to find fingers by moving finger tip sized object inside paw contour
 * </p>
 *
 * @author rogister
 */
public class PawTracker2 extends EventFilter2D implements FrameAnnotater, Observer /*, PreferenceChangeListener*/ {
    
    
    private static Preferences prefs=Preferences.userNodeForPackage(PawTracker2.class);
    
    
    // Global constant values
    
    protected AEChip chip;
    private AEChipRenderer renderer;
    
    /** the number of classes of objects */
    private final int NUM_CLASSES=2;
    // max number of orientations
    private final int MAX_SEGMENTS=10000;
    //private final int MAX_DISTGC=300;//depends
    //private final int MAX_SEQ_LENGTH=50;
    //private final int MAX_NB_SEQ=310;
    
    private final int MAX_NB_FINGERS=4;
    
    private double maxOrientation = 180;
    private double maxDistGC = 200;
            
   
   
    // Parameters appearing i nthe GUI
   
    private int door_xa=prefs.getInt("PawTracker2.door_xa",50);
    {setPropertyTooltip("door_xa","lower x bound of the cage door (x,y inverted)");}
    private int door_xb=prefs.getInt("PawTracker2.door_xb",127);
    {setPropertyTooltip("door_xb","higher x bound of the cage door (x,y inverted)");}
    private int door_ya=prefs.getInt("PawTracker2.door_ya",52);
    {setPropertyTooltip("door_ya","lower y bound of the cage door (x,y inverted)");}
    private int door_yb=prefs.getInt("PawTracker2.door_yb",88);
    {setPropertyTooltip("door_yb","higher y bound of the cage door (x,y inverted)");}
    
    private int retinaSize=prefs.getInt("PawTracker2.retinaSize",128);
    {setPropertyTooltip("retinaSize","resolution of the retina");}
    private int linkSize=prefs.getInt("PawTracker2.linkSize",2);
    {setPropertyTooltip("linkSize","Neighbourhood range for linking contours");}
    private int segSize=prefs.getInt("PawTracker2.segSize",2);
    {setPropertyTooltip("segSize","Neighbourhood range for shape segment creation");}
    
    private int maxSegSize=prefs.getInt("PawTracker2.maxSegSize",4);
    {setPropertyTooltip("segSize","MAximum size of a shape segment");}
    
    private int minZeroes=prefs.getInt("PawTracker2.minZeroes",2);
    private int maxZeroes=prefs.getInt("PawTracker2.maxZeroes",6);
    private int doorMinZeroes=prefs.getInt("PawTracker2.doorMinZeroes",2);
    private int doorMaxZeroes=prefs.getInt("PawTracker2.doorMaxZeroes",6);
    
    private int in_length=prefs.getInt("PawTracker2.in_length",3);
    private int in_test_length=prefs.getInt("PawTracker2.in_test_length",3);
    private float in_threshold=prefs.getFloat("PawTracker2.in_threshold",0.4f);
    private float line_threshold=prefs.getFloat("PawTracker2.line_threshold",0f);
    private int score_threshold=prefs.getInt("PawTracker2.score_threshold",37);
    private float score_in_threshold=prefs.getFloat("PawTracker2.score_in_threshold",0.2f);
    private float score_sup_threshold=prefs.getFloat("PawTracker2.score_sup_threshold",0.1f);
        
    private int boneSize=prefs.getInt("PawTracker2.boneSize",2);
        
    private int score_range=prefs.getInt("PawTracker2.score_range",3);
    private int line_range=prefs.getInt("PawTracker2.line_range",8);
    private int lines_n_avg=prefs.getInt("PawTracker2.lines_n_avg",8);
   
    private float parallel_mix=prefs.getFloat("PawTracker2.parallel_mix",0.4f);
    private int parallel_range=prefs.getInt("PawTracker2.parallel_range",10);
   
    private float knuckle_inertia=prefs.getFloat("PawTracker2.knuckle_inertia",0.5f);
       
    private int cluster_lifetime=prefs.getInt("PawTracker2.cluster_lifetime",10);
    
    private int fing_maxRange=prefs.getInt("PawTracker2.fing_maxRange",5);
    private int fing_minRange=prefs.getInt("PawTracker2.fing_minRange",3);
   
    private int intensityZoom = prefs.getInt("PawTracker2.intensityZoom",2);
     
    private int decayLimit = prefs.getInt("PawTracker2.decayLimit",2);
    private boolean decayOn = prefs.getBoolean("PawTracker2.decayOn",false); 
    
    private int subzone_xa=prefs.getInt("PawTracker2.subzone_xa",30);
    private int subzone_xb=prefs.getInt("PawTracker2.subzone_xb",127);
    private int subzone_ya=prefs.getInt("PawTracker2.subzone_ya",110);
    private int subzone_yb=prefs.getInt("PawTracker2.subzone_yb",127);
    
    private int fingertip_size=prefs.getInt("PawTracker2.fingertip_size",4);   
        
    private int max_fingers=prefs.getInt("PawTracker2.max_fingers",4);
    
    private int minSeqLength=prefs.getInt("PawTracker2.minSeqLength",4);
                
    private float seqTolerance=prefs.getFloat("PawTracker2.seqTolerance",10f);
    
    private float minDiff=prefs.getFloat("PawTracker2.minDiff",2f);
    private float maxDiff=prefs.getFloat("PawTracker2.maxDiff",2f);
    private float doorMinDiff=prefs.getFloat("PawTracker2.doorMinDiff",2f);
    private float doorMaxDiff=prefs.getFloat("PawTracker2.doorMaxDiff",2f);
    private float minAngle=prefs.getFloat("PawTracker2.minAngle",90f);
       
    private float finger_cluster_range=prefs.getFloat("PawTracker2.finger_cluster_range",2);
    private float finger_ori_variance=prefs.getFloat("PawTracker2.finger_ori_variance",60);
   
    
    private float node_range=prefs.getFloat("PawTracker2.node_range",2);
    
    
    private float finger_start_range=prefs.getFloat("PawTracker2.finger_start_range",2);
    private int finger_start_threshold=prefs.getInt("PawTracker2.finger_start_threshold",45);
    private float finger_length=prefs.getFloat("PawTracker2.finger_length",10.0f);
    private float finger_mv_smooth=prefs.getFloat("PawTracker2.finger_mv_smooth",0.1f);
  
    private float finger_sensitivity=prefs.getFloat("PawTracker2.finger_sensitivity",2.0f);
         
    private float tracker_time_bin=prefs.getFloat("PawTracker2.tracker_time_bin",1000);
 
    private float contour_act_thresh=prefs.getFloat("PawTracker2.contour_act_thresh",0);
    private float contour_min_thresh=prefs.getFloat("PawTracker2.contour_min_thresh",0);
    private int contour_range=prefs.getInt("PawTracker2.contour_range",1);
   
    private int densityMinIndex=prefs.getInt("PawTracker2.densityMinIndex",0);
    private int densityMaxIndex=prefs.getInt("PawTracker2.densityMaxIndex",0);
    private boolean showDensity = prefs.getBoolean("PawTracker2.showDensity",false);
    
    private boolean scaleInDoor = prefs.getBoolean("PawTracker2.scaleInDoor",false);
       
    private boolean showSegments = prefs.getBoolean("PawTracker2.showSegments",true);
    private boolean showFingers = prefs.getBoolean("PawTracker2.showFingers",true);
    private boolean showFingerTips = prefs.getBoolean("PawTracker2.showFingerTips",true);
    private boolean showClusters = prefs.getBoolean("PawTracker2.showClusters",true);
 
  
    private boolean showZones = prefs.getBoolean("PawTracker2.showZones",true);
    private boolean showAll = prefs.getBoolean("PawTracker2.showAll",true);
    // hide activity inside door
    private boolean hideInside = prefs.getBoolean("PawTracker2.hideInside",false);
    // show intensity inside shape
    private boolean showIntensity = prefs.getBoolean("PawTracker2.showIntensity",false);
  
    private boolean showAcc = prefs.getBoolean("PawTracker2.showAcc",false);
    private boolean scaleIntensity = prefs.getBoolean("PawTracker2.scaleIntensity",false);
    private boolean scaleAcc = prefs.getBoolean("PawTracker2.scaleAcc",true);
    
    
    private boolean showWindow = prefs.getBoolean("PawTracker2.showWindow",true);
    private boolean showScore = prefs.getBoolean("PawTracker2.showScore",false);
   
              
    private boolean useSimpleContour = prefs.getBoolean("PawTracker2.useSimpleContour",true);
    private boolean useFingerDistanceSmooth = prefs.getBoolean("PawTracker2.useFingerDistanceSmooth",true);
              
    private boolean showShapePoints = prefs.getBoolean("PawTracker2.showShapePoints",true);
  
    private boolean showShape = prefs.getBoolean("PawTracker2.showShape",true);
  
    private boolean smoothShape = prefs.getBoolean("PawTracker2.smoothShape",true);
    //   private boolean showKnuckles = prefs.getBoolean("PawTracker2.showKnuckles",true);
      
    private int lowFilter_radius=prefs.getInt("PawTracker2.lowFilter_radius",10);
    private int lowFilter_density=prefs.getInt("PawTracker2.lowFilter_density",17);
    private float lowFilter_threshold=prefs.getFloat("PawTracker2.lowFilter_threshold",0);   
    
     
    private boolean thinning = prefs.getBoolean("PawTracker2.thinning",true);
    private float thin_threshold=prefs.getFloat("PawTracker2.thin_threshold",0);
    private boolean showThin = prefs.getBoolean("PawTracker2.showThin",false);
    private boolean showPalm = prefs.getBoolean("PawTracker2.showPalm",false);   
    
    private boolean showSkeletton = prefs.getBoolean("PawTracker2.showSkeletton",false);
    
    // do not forget to add a set and a get/is method for each new parameter, at the end of this .java file
    
    
    // Global variables
    
     private int nbFingers = 0; //number of fingers tracked, maybe put it somewhere else
    
     
    protected float defaultClusterRadius;
    //   protected float mixingFactorClusterForce=prefs.getFloat("PawTracker2.mixingFactorClusterForce",0.01f);
    protected float mixingFactor=prefs.getFloat("PawTracker2.mixingFactor",0.01f); // amount each event moves COM of cluster towards itself
   
    private boolean logDataEnabled=false;
    private PrintStream logStream=null;
   
    protected boolean pawIsDetected = false;
    protected boolean showDetectedClusters = false;
    
    protected Palm palm = new Palm(); //change to add parameters, and in reset too
    
    protected Contour contour = new Contour();
    protected Segment segments[];
    protected int nbSegments = 0;
    protected int accOrientationMax = 1;
    
    protected int nbFingersActive = 0;
    protected boolean knuckle_polygon_created = false;
    /** accumulate events as would do the accumulate view mode 'P' from the gui */
    // x,y axis are swapped to y,x in accEvents 
    protected float accEvents[][][]; 
    protected float filteredEvents[][][]; 
    protected int thinned[][][];
    
    protected Contour skeletton = new Contour();
    protected Segment bones[];
    protected int nbBones = 0;
    protected Vector nodes = new Vector();
    
    protected float grayValue = 0.5f;
    protected int colorScale = 2;
    
    /** intensity inside the paw shape as accumulated value of projections from border segments */
    protected float insideIntensities[][][];
    protected float intensityIncrease = 0.1f;
    
    protected float scoresFrame[][][];
    
    protected Vector fingerTips = new Vector();
    protected Vector fingerLines = new Vector();
    
    // array of finger tips to track
    protected FingerCluster[] fingerTipClusters = new FingerCluster[MAX_NB_FINGERS];
    
    float[] densities = new float[lowFilter_density];
    
    private boolean activity_started = false;
    private boolean grasp_started = false;
    //protected float startPoint = 0f;
   // protected float stopPoint = 0f;
    
    /** Creates a new instance of PawTracker */
    public PawTracker2(AEChip chip) {
        super(chip);
        this.chip=chip;
        renderer=(AEChipRenderer)chip.getRenderer();
        chip.getRenderer().addAnnotator(this); // to draw the clusters
        chip.getCanvas().addAnnotator(this);
        initFilter();
        resetPawTracker();
        chip.addObserver(this);
//        prefs.addPreferenceChangeListener(this);
        
        
    }
    
    public void initFilter() {
        initDefaults();
        // defaultClusterRadius=(int)Math.max(chip.getSizeX(),chip.getSizeY())*getClusterSize();
    }
    
    private void initDefaults(){
        //System.out.println("initDefaults");
        initDefault("PawTracker2.clusterLifetimeWithoutSupport","10000");
        initDefault("PawTracker2.maxNumClusters","10");
        initDefault("PawTracker2.clusterSize","0.15f");
        initDefault("PawTracker2.numEventsStoredInCluster","100");
        initDefault("PawTracker2.thresholdEventsForVisibleCluster","30");
        initDefault("PawTracker2.thresholdISIForVisibleCluster","2.0f");
        
//        initDefault("PawTracker2.","");
    }
    
    private void resetPawTracker(){
        // finger_sensitivity is tolerance of closeness between end node and finger tip
        palm = new Palm(finger_sensitivity,finger_length/2);
        
        grasp_started = false;
      //  activity_started = false;
      //  startPoint = 0f;
      //  stopPoint = 0f;
        
       // resetFingerClusters();
                
                
        pawIsDetected = false;
        showDetectedClusters= false;
        
        //System.out.println("resetPawTracker");
        accEvents = new float[retinaSize][retinaSize][3]; 
        resetArray(accEvents,0);
        //resetArray(accEvents,grayValue);
        
        
        filteredEvents = new float[retinaSize][retinaSize][3]; 
        resetArray(filteredEvents,0);
                
                
        insideIntensities = new float[retinaSize][retinaSize][1]; // should remove last dimension
        resetArray(insideIntensities,0);  
        
        scoresFrame = new float[retinaSize][retinaSize][1]; // should remove last dimension
        resetArray(scoresFrame,0);  
       
        thinned = new int[retinaSize][retinaSize][3];
       
       // sequences = new Sequence[MAX_NB_SEQ];
       // sequenceSize = 0;
        segments = new Segment[MAX_SEGMENTS];
        nbSegments = 0;
        
        bones = new Segment[MAX_SEGMENTS];
        nbBones = 0;
        
        accOrientationMax = 1;
        contour.reset();
       
        // set subzone parameters here
        
        nbFingers = 0;
        setResetPawTracking(false);//this should also update button in panel but doesn't'
    }
    
    /**
     private void resetFingerClusters(){
         
        nbFingersAssigned = 0;
        knuckle_polygon_created  = false; 
        fingerTips = new Vector();
        fingerLines = new Vector();
    
    // array of finger tips to track
        fingerTipClusters = new FingerCluster[MAX_NB_FINGERS];
    
    
        // reset finger tips clusters
        float a = 0;
        for (int i=0; i<fingerTipClusters.length; i++){
            int y = door_xa;
            int x = (int)(door_ya+((float)(door_yb-door_ya)*a));
            //System.out.println("init: x :"+x+" a:"+a+" fingerTipClusters.length:"+fingerTipClusters.length);
            fingerTipClusters[i] = new FingerCluster();
            a += 1/(float)(fingerTipClusters.length-1);
        }
        // set to initial values (to parametrize)
       
         
     }
     */
     
    private void initDefault(String key, String value){
        if(prefs.get(key,null)==null) prefs.put(key,value);
    }
    
   
    
    
    // the method that actually does the tracking
    synchronized private void track(EventPacket<TypedEvent> ae){
        
        
        if(isResetPawTracking()){
            // reset
            resetPawTracker();
            return; //maybe continue then
        }
        
        int n=ae.getSize();
        if(n==0) return;
        
        resetDensities(lowFilter_density);
        
       
        
        // for each event, accumulate into main zone
        
      
        
        float step = 2f / (colorScale + 1);
       
        // accumulate the events
        //float currentTime = (float)ae.getLastTimestamp();
        //float firstTime = (float)ae.getFirstTimestamp();
        //float difftime = currentTime - firstTime; 
        //System.out.println("time: "+currentTime);
        
        //System.out.println("first: "+firstTime+" last:"+currentTime+" diff:"+difftime);
        
        
        float startTime = (float)ae.getFirstTimestamp();
        float lastTime = (float)ae.getLastTimestamp();
        float currentTime = startTime;
        for(TypedEvent e:ae){
            int type=e.getType();
            float a = (accEvents[e.y][e.x][0]);
            a += step * (type - grayValue);
            
            // if previous value is zero, add value before reset then set this value to zero
          
           
            accEvents[e.y][e.x][0] = a + accEvents[e.y][e.x][1];
            accEvents[e.y][e.x][1] = 0;
            //accEvents[e.y][e.x][1] = accEvents[e.y][e.x][0];
            
            // accEvents[e.y][e.x][1] is used for value before reset for correcting decay
            
            currentTime = (float)e.timestamp;
            accEvents[e.y][e.x][2] = (float)e.timestamp; // use it for time stamp, small hack also...
            
           
        
            
            
            // track every time-bin as defined in parameters
            // potential problem if timestamp changing from increasing to decreasing (talk to Tobi)
            if(currentTime-startTime>tracker_time_bin){
                startTime = currentTime;
                // process tracker
                processTracking(currentTime);
            }
            
        }
        
        // end loop on events, if last != start, process again
        // also normal use if tracker_time_bin too big
        if(lastTime!=startTime){
            processTracking(lastTime);
        }
    
        
    }
    
    public void processTracking( float currentTime ){
        // reset
        contour.reset();
        skeletton.reset();
        nodes = new Vector();
        
        resetArray(insideIntensities,0);
        //resetArray(filteredEvents,0);
        
            // special scale for inside door
        if(scaleInDoor){
            scale(accEvents,door_xa,door_ya,door_xb,door_yb);
        }
        // keep in range [0-1]
        if(scaleAcc){
            scale(accEvents);
        }
        
       // if not stopped, process
            
           
          
        
        filteredEvents = new float[retinaSize][retinaSize][accEvents[0][0].length];
        filteredEvents = lowFilterFrame(accEvents,getLowFilter_radius(),getLowFilter_density());
        // apply border filter to find contours
        contour = findContour(filteredEvents);
        
        if(thinning){
            //thresholdImage(filteredEvents,thin_threshold);
            thinned = thin(thresholdImage(filteredEvents,thin_threshold));
            // create contour out of thinned
            skeletton = new Contour(thinned,1);
            //nbBones = detectSegments( skeletton, 1, boneSize, bones, filteredEvents, false );
            nodes = detectNodes(skeletton,1,boneSize);
            
        }
         
            
            //
            
            //contour = findContour(renderer.getFr());
            
            // actually contour is made of several contours, (should rename contour to contours)
            // and here we label each contour with a number starting at 2
            // label 1 is reserved for the contours that touches the door
            // as the paw ocntour will always come though the door of the cage, which coordinates
            // must be well defined
            contour.link();
            // here we change to 1 the label of contours touching the door to identify them
            contour.highlightTouchingDoor();
            
          
            
            
            //  finding shape segments
            nbSegments = detectSegments( contour, 1, segSize, segments, filteredEvents, showIntensity );
        
            // finding finger tip by template matching
            fingerTips = detectFingersTips(insideIntensities,filteredEvents);
         
          // starts but also stops
           // if no activity
            if(!grasp_started&&checkActivity()) {
                grasp_started = checkGraspStart(fingerTips);
            } else {
                if(!checkActivity()&&!checkGraspStart(fingerTips)){
                 
                 grasp_started = false;
                 // reset
                 //resetFingerClusters();
                }
            }
           
            
           if(grasp_started){ 
            
                
            
            // here drives finger tip clusters!
            //driveFingerClusters(fingerTips,finger_cluster_range,finger_start_range,finger_start_threshold);

            // detect fingers lines from finger tips cluster(to change to), use global variable contour
            //fingerLines = detectFingersLines(fingerTips,insideIntensities,filteredEvents);
            //fingerLines = detectFingersLines(filteredEvents);


            // now try to attach model
            if(thinning){
                palm.updateModel(fingerTips,nodes,finger_cluster_range,finger_start_range,finger_start_threshold,node_range);
                
                palm.attachModel(nodes,skeletton,fingerTips,filteredEvents);
                
                
            }
                            
        } 
          
      
        if(decayOn){
                float decay = (float)decayLimit;
                //System.out.println("decay is on");
                // decay onloy inside door
                //decayArray(accEvents,currentTime,door_xa,door_xb,door_ya,door_yb,decay);
                // decay all
                decayArray(accEvents,currentTime,0,retinaSize-1,0,retinaSize-1,decay);
        }
    }
    
    
    public String toString(){
        String s="PawTracker2";
        return s;
    }
    
    public void resetArray( float[][][] array, float value ){
        
        // more efficient to just set all elements to value, instead of allocating new array of zeros
        // profiling shows that copying array back to matlab takes most cycles!!!!
        for (int i = 0; i<array.length; i++)
            for (int j = 0; j < array[i].length; j++){
            float[] f = array[i][j];
                for (int k = 0; k < f.length; k++){
                    f[k]=value;            
                }
            }
    }
    
    // reset float[][][k] array with k specified
    public void resetArray( float[][][] array, float value , int k){
        
        for (int i = 0; i<array.length; i++)
            for (int j = 0; j < array[i].length; j++){
            float[] f = array[i][j];
                if(k>0&&k<f.length){
                    f[k]=value;            
                }
            }
    }
    
    // copy float[][][k1] array into float[][][k2]
    public void emptyIntoSameArray( float[][][] array, int k1 , int k2){
         for (int i = 0; i<array.length; i++)
            for (int j = 0; j < array[i].length; j++){
            float[] f = array[i][j];
                if((k1>=0&&k1<f.length)&&(k2>=0&&k2<f.length)){
                    f[k2]=f[k1];  
                    f[k1]=0;
                }
            }
    }
    
    
    
    // here not scaling but clipping values to [0..1]
      protected void scale(float[][][] fr ){
          // just clip values to 0..1 
          for (int i = 0; i<fr.length; i++){
              for (int j = 0; j<fr[i].length; j++){
//                  for (int k = 0; k<3; k++){
//                      float f = fr[i][j][k];
//                      if(f<0) f=0; else if(f>1) f=1;
//                      fr[i][j][k]=f;
//                  }
                  float f = fr[i][j][0];
                  if(f<0) f=0; else if(f>1) f=1;
                  fr[i][j][0]=f;
              }
          }
      }
      
      // scale, but only on the first of n float value of a float[][][n] array
       protected void scale(float[][][] fr, int x1, int y1, int x2, int y2 ){
           
          if(x1>=fr.length)return;
          if(x2>=fr.length)return;
          if(y1>=fr.length)y1=fr.length-1;
          if(y2>=fr[0].length)y2=fr[0].length-1;
          //find max
          float max = 0;
          
          for (int i = x1; i<y1; i++){
              for (int j = x2; j<y2; j++){
                  
                      float v = fr[i][j][0];
                      if(v>max)max=v;
                      
                  
              }
          }
          // scale only positive values, neg value to zero
          for (int i = x1; i<y1; i++){
              for (int j = x2; j<y2; j++){
                  
                      float f = fr[i][j][0];
                      if(f<0) f=0; else {
                          f=f/max;
                          
                      }
                      fr[i][j][0]=f;
                  
              }
          }
      }

       
      public void decayArray(float[][][] array , float currentTime, int xa, int xb, int ya, int yb, float decayLimit){
          
          // more efficient to just set all elements to value, instead of allocating new array of zeros
          // profiling shows that copying array back to matlab takes most cycles!!!!
          if(xb>=array.length) xa = array.length;
          if(yb>=array[0].length) ya = array[0].length;
          if(xa<0)xa=0;
          if(ya<0)ya=0;
          for (int i = xa; i<xb; i++){
              for (int j = ya; j < yb; j++){
                  float[] f = array[i][j];
                  float diff = currentTime-f[2];
                  //System.out.println("f.length:"+f.length+" currentTime-f[2]:"+diff+" < "+decayLimit);
                  if(f.length==3&&f[0]>0){// hack, i know
                      
                      // store initial value before decay, important to compute future updated values
                      if(f[1]==0)f[1] = f[0];
                      
                      // decay proportionnally
                      if(diff>=decayLimit) {
                          f[0] = 0;
                      } else {
                        f[0] = f[0] * (decayLimit-diff)/decayLimit;
                      }
//                      if(currentTime-f[2]<decayLimit) {
//                         //System.out.println("< true, remove point "+i+","+j);
//                          
//                              f[0]-=0.1;
//                              if(f[0]<=0){
//                                  f[0]=0;
//                                  f[1]=0;
//                                  f[2]=0;
//                              }
//                              
//                      }
                  }
              }
          }
      }
      
      
    
      
      
      
      // check activity iside door to avoid extensive computation if there is only 
      private boolean checkActivity(){
          // look into accEvents
          // simple check : look if points in door pass from zero to 1
          int y = (int)(door_yb*.5+door_ya*.5);
          int x = (int)(door_xb*.7+door_xa*.3);
          //System.out.println("activity check at "+x+","+y);
          
          if((accEvents[x][y][0]>0.5)){
              
              return true;
          } else {
             
              //return false;
          }
          // for the moment always rturn true until better detection is made
          return true;
      }
      
      
      
      // check grasping start
      private boolean checkGraspStart( Vector points ){
          // for all points compare score to start threshold          
          for(Object o:points){
              Point p = (Point)o;
              if(p.score>finger_start_threshold){
                  if(!insideDoor(p.y,p.x)) return true; // inverted x,y
                        
              }
          }         
          return false;          
          
      }
      
      private boolean checkGraspStop( Vector points ){
         
             // if all points below threshold return true
          
          for(Object o:points){
              Point p = (Point)o;
              if(p.score>finger_start_threshold){                  
                  return false;
              }
          }         
          return true;   
          
          
      }
    
      
      private class Finger{
          // two segments: base finger and forefinger
          int x0,y0;
          int x1,y1;
          int x2,y2; //4 coordinates, finger segments could be detached??
          int x3,y3;
          float length1;
          float length2;
          float orientation1;
          float orientation2;
          
          public Finger( Line line1, Line line2 ){
              x0 = line1.x0;
              y0 = line1.y0;
              x1 = line1.x1;
              y1 = line1.y1;
              x2 = line2.x0;
              y2 = line2.y0;
              x3 = line2.x1;
              y3 = line2.y1;
              length1 = line1.length;
              length2 = line2.length;
              orientation1 = lineOrientation(line1);
              orientation2 = lineOrientation(line2);
          }
      
      }
      
      // Class Palm
      private class Palm{
          //Finger fingers[];
          FingerCluster[] fingers = new FingerCluster[MAX_NB_FINGERS];
          
          //int nbFingerAttached = 0;
          int nbFingersActive = 0;
          float maxDistance;
          float halfFingerLength = 10;
          
          public Palm(){
            //fingers = new Finger[MAX_NB_FINGERS];
             
          }
          
          public Palm(float maxDistance, float halfFingerLength){
              
              this.maxDistance = maxDistance; // to remove
              this.halfFingerLength = halfFingerLength;
          }
          
          public void updateParameters(float maxDistance, float halfFingerLength){
              
              this.maxDistance = maxDistance; // to remove
              this.halfFingerLength = halfFingerLength;
          }
          
          
          protected void updateModel( Vector fingerTips, Vector nodes, float range, float start_range, int start_threshold, float node_range ){
              
              // resest assignment
              for (int i=0;i<fingers.length;i++){
                  if(fingers[i]==null)fingers[i]=new FingerCluster();
                  fingers[i].assigned = false;
                  
              }
                            
              // for all active fingers while not all assigned
              int nbActiveAssigned=0;
              boolean outOfRange = false;
              while(nbActiveAssigned<nbFingersActive&&!outOfRange){
                  
                  
                  float min = 1000;
                  Point minPoint = new Point();
                  int minCluster = 0;
                  // for all unassigned activated finger
                  for (int i=0;i<fingers.length;i++){
                      if(fingers[i].activated){

                          // find for each finger the closest point
                          if(!fingers[i].assigned){
                              // for all points
                              for (Object o:fingerTips){
                                  Point p = (Point)o;
                                  if(p.disabled!=1){// if point not already assigned
                                      float distance = distanceBetween(p.x,p.y,fingers[i].tip_x_end[0],fingers[i].tip_y_end[0]);
                                      if(distance<range){
                                          if(distance<min){
                                              min = distance;
                                              minPoint = p;
                                              minCluster = i;
                                          }
                                          
                                      }
                                  }
                              }
                          } // end for each finger find the closest point

                         
                          
                      }
                  }
                  // assign min, remove p and c from further computation
                  if(min<1000){
                      nbActiveAssigned++;
                      
                      fingers[minCluster].addTip(minPoint); // which set its "assigned" to true
                      
                      //System.out.println("update finger cluster tip at"+minPoint.x+","+minPoint.y);
                      // disable point
                      minPoint.disabled = 1;
                  } else {
                      outOfRange = true; //break loop because no point found in range
                  }
              } // end while
                      
               // for all active fingers while not all assigned
              int nbNewlyAssigned=0;
              outOfRange = false;
              int nbYetToAssign = MAX_NB_FINGERS-nbFingersActive;
              while(nbNewlyAssigned<nbYetToAssign&&!outOfRange){
                  
                  
                  float min = 1000;
                  Point minPoint = new Point();
                  int minCluster = 0;
                  // for all unassigned and not yet activated finger
                  for (int i=0;i<fingers.length;i++){
                      if(!fingers[i].activated){ // only for not yet activated clusters
                          
                          // find for each finger the closest point
                          if(!fingers[i].assigned){
                              // for all points
                              for (Object o:fingerTips){
                                  Point p = (Point)o;
                                  if(p.score>score_threshold&&p.disabled!=1){ // thus checking if p already used
                                      
                                      // find node in range with finger tip
                                      float min_dist_node = 1000;
                                      
                                      for (Object no:nodes){
                                          Node n = (Node)no;
                                          float dist_node = distanceBetween(p.x,p.y,n.x,n.y);
                                          if(dist_node<node_range){
                                              // found a potential finger
                                              if(dist_node<min_dist_node){
                                                  min_dist_node = dist_node;
                                                  break; // if any below range then it is ok so exit "for (Object no:nodes)"
                                              }
                                          }
                                      }
                                      // if any in range
                                      if(min_dist_node<1000){
                                          min = 0; //we select this one for being a new cluster
                                          // no need to see if another finger cluster nearer
                                          minPoint = p;
                                          minCluster = i;
                                          break; // found node in range so we exit loop "for (Object o:fingerTips)" to continue
                                      }
                                      
                                      
                                  }
                              } // end for each finger find the closest point
                              
                              
                              
                          }
                      }
                  }
                  // assign min, remove p and c from further computation
                  if(min<1000){
                      nbNewlyAssigned++;
                      if(!fingers[minCluster].activated) nbFingersActive++;
                      fingers[minCluster].addTip(minPoint); // which set its "assigned" to true
                      
                      //System.out.println("add new finger cluster tip at"+minPoint.x+","+minPoint.y);
                      // disable point
                      minPoint.disabled = 1;
                  } else {
                      outOfRange = true; //break loop because no point found in range
                  }
              }     // end while
          }
              
          
              
           
   
       /**
          public void assignNeigbours(){
                  // if all assigned for first time this grasp, create knucle polygon by assigning closest neighbours
              if(!knuckle_polygon_created){
                  //System.out.println("nb assigned: "+nbFingersAssigned+"<"+MAX_NB_FINGERS);
                  if(nbFingersAssigned==MAX_NB_FINGERS){
                      knuckle_polygon_created = true;
                      boolean endFoundBefore = false;
                      FingerCluster endCluster = null;
                      // assign closest neighbours
                      // for all clusters find closest neighbours
                      for (int i=0;i<fingers.length;i++){
                          
                          int x = fingers[i].x;
                          int y = fingers[i].y;
                          Vector minima = new Vector();
                          for (int j=0;j<fingers.length;j++){
                              if(j!=i){
                                  // compute distance
                                  float distance = distanceBetween(x,y,fingers[j].x,fingers[j].y);
                                  // add to list of minima
                                  minima.add(new Minimum((int)distance,j));
                                  
                              }
                              
                          }
                          // all values in minima
                          // sort minima in ascending order
                          Collections.sort(minima, new MinimaComparer());
                          // add first neighbour to cluster
                          if(minima.size()>0){
                              fingers[i].neighbour1 = fingers[((Minimum)minima.elementAt(0)).index];
                              
                          }
                          if(minima.size()>1){ // another one, must be in different plane
                              int xb = fingers[i].finger_base_x;
                              int yb = fingers[i].finger_base_y;
                              // compute separation line equation
                              float b = ((float)(y*xb) - (float)(x*yb))  / (float)(xb-x);
                              float a = (y-b)/(float)x;
                              boolean above = false;
                              // in what plane is the first neighbour?
                              if(fingers[i].neighbour1.y>(a*fingers[i].neighbour1.x+b)){
                                  above = true;
                              }
                              
                              // while minima left, not assigned
                              // check if minima is in other plane from previous one
                              // if no, assign and quit
                              // else continue
                              boolean found = false;
                              int k=1;
                              while(!found&&k<minima.size()){
                                  int index = ((Minimum)minima.elementAt(k)).index;
                                  int x2 = fingers[index].x;
                                  int y2 = fingers[index].y;
                                  
                                  // check if different plane
                                  if(y2>(a*x2+b)){
                                      if(!above){
                                          // ok this one is above but before was below
                                          found = true;
                                          fingers[i].neighbour2 = fingers[index];
                                      }
                                  } else {
                                      if(above){
                                          // ok this one is below but before was above
                                          found = true;
                                          fingers[i].neighbour2 = fingers[index];
                                      }
                                  }
                                  
                                  
                                  k++;
                              }//end while not found second minimum
                              if(!found){
                                  if(endFoundBefore){
                                      fingers[i].neighbour2 = endCluster;
                                      endCluster.neighbour2 = fingers[i];
                                  } else {
                                      endCluster = fingers[i];
                                      endFoundBefore = true;
                                  }
                              }
                              
                          }//end if more than one minimum
                          
                      }// end for all clusters find closest neighbours
                  }//end if nb assigned == nb max fingers
              }// end if need create knuckle polygon
          }
         */ 
          
          
          public void attachModel(Vector nodes, Contour thin, Vector fingerTips, float[][][] frame ){
              // if end node near fingertip for the first time and less than four fingers attached do :
              // find two consecutive best line of length L starting from end node or finger tip
              maxDistance = finger_sensitivity;
              halfFingerLength = finger_length/2;
                      
              // for all fingers
              for (Object o:fingers){
                  
                  FingerCluster f = (FingerCluster)o;
                  
                  if(f.activated){
                                        
                      // for each finger, find the line that fit best the thinned skeletton image
                     
                      Line line1 = mostValuableLine(f.tip_x_end[0],f.tip_y_end[0], (int)halfFingerLength, thin , frame);
                      Line line2 = mostValuableLine(line1.x1,line1.y1, (int)halfFingerLength, lineDirection(line1), finger_ori_variance, thin , frame);
                     // System.out.println("For tip "+f.tip_x_end[0]+","+f.tip_y_end[0]+" lines found: "+line1.length+" "+line2.length);
                     // System.out.println("Line 1 ["+line1.x0+","+line1.y0+"] to ["+line1.x1+","+line1.y1+"]");
                     // System.out.println("Line 2 ["+line2.x0+","+line2.y0+"] to ["+line2.x1+","+line2.y1+"]");
              
                      // updateLines is quite important
                      // it defines the coordinates of the two segments composing a finger
                      // but also, if these segments were previously defined, it apply a chosen
                      // policy for merging them so as to preserv ekinetic constraints or others
                  
                      f.updateLines(line1,line2);
                      
                          
                  }
                  
              }
              
             
              
              
          }
          
         
                 
                  
          public void driveModel(Vector nodes, int [][][] image, Vector fingerTips ){
              
             
              //  move cluster and check constraint like
              // if too far from other finger, move toward..
              // or delete if life time set and reached ...
              
//              for (int i=0;i<fingers.length;i++){
//                  // if not assigned
//                  if(!fingers[i].assigned){
//                      fingers[i].updatePosition(fingers,i);
//                  }
//              }
              
              
              
          }
              
              
      } // end Class Palm 
         
 
          
      
      
      // Class FingerCluster
      private class FingerCluster{
          // int[2] for storing previous value also
          int[] tip_x_end = new int[2]; //end tip
          int[] tip_y_end = new int[2];
          int[] tip_x_start = new int[2]; // beginning  tip
          int[] tip_y_start = new int[2];
          int[] base_x_start = new int[2]; // beginning first joint
          int[] base_y_start = new int[2];
          int[] base_x_end = new int[2]; // end first joint
          int[] base_y_end = new int[2];
          
         
          int length_tip;
          int length_base;
          float orientation_tip;
          float orientation_base;
          float direction_tip;
          float direction_base;
          int lifetime;
                   
          boolean assigned = false;
          boolean activated = false;
         
         
          FingerCluster neighbour1 = null;
          FingerCluster neighbour2 = null;
          
          public FingerCluster(  ){
             
              reset();
          }
          
          
          
          public void reset(){
              activated = false;
              assigned = false;
              lifetime = 0;
              neighbour1 = null;
              neighbour2 = null;
              
              tip_x_end = new int[2]; //end tip
              tip_y_end = new int[2];
              tip_x_start = new int[2]; // beginning  tip
              tip_y_start = new int[2];
              base_x_start = new int[2]; // beginning first joint
              base_y_start = new int[2];
              base_x_end = new int[2]; // end first joint
              base_y_end = new int[2];
          }   
          
          public void addTip( Point p){
             
              // memorize previous values
              tip_x_end[1] = tip_x_end[0];
              tip_y_end[1] = tip_y_end[0];
            
              tip_x_end[0] = p.x;
              tip_y_end[0] = p.y;
              // if not previously assigned this grasp : 
              
              
              assigned = true;
              activated = true;
              lifetime = cluster_lifetime;         
          }
          
          public void updateLines( Line line1, Line line2 ){
              // memorize previous values
               // memorize previous values
              // tip_x0[0] and tip_y0[0] already saved
              tip_x_start[1] = tip_x_start[0];
              tip_y_start[1] = tip_y_start[0];
              base_x_start[1] = base_x_start[0];
              base_y_start[1] = base_y_start[0];
              base_x_end[1] = base_x_end[0];
              base_y_end[1] = base_y_end[0];
              
              if(length_tip!=0){ 
                  if(line1.length>0){
                     // tip_x_end[0] = (int)(tip_x_end[0]*finger_mv_smooth + line1.x1*(1-finger_mv_smooth));
                    // tip_y_end[0] = (int)(tip_y_end[0]*finger_mv_smooth + line1.y1*(1-finger_mv_smooth));
                      tip_x_start[0] = (int)(tip_x_start[0]*finger_mv_smooth + line1.x1*(1-finger_mv_smooth));
                      tip_y_start[0] = (int)(tip_y_start[0]*finger_mv_smooth + line1.y1*(1-finger_mv_smooth));
                      orientation_tip = lineOrientation(new Line(tip_x_end[0],tip_y_end[0],tip_x_start[0],tip_y_start[0]));
                      length_tip = line1.length;
                  }
                  if(line2.length>0){
                      base_x_start[0] = (int)(base_x_start[0]*finger_mv_smooth + line2.x1*(1-finger_mv_smooth));
                      base_y_start[0] = (int)(base_y_start[0]*finger_mv_smooth + line2.y1*(1-finger_mv_smooth));
                      base_x_end[0] = (int)(base_x_end[0]*finger_mv_smooth + line2.x0*(1-finger_mv_smooth));
                      base_y_end[0] = (int)(base_y_end[0]*finger_mv_smooth + line2.y0*(1-finger_mv_smooth));
                      
                      orientation_base = lineOrientation(new Line(base_x_end[0],base_y_end[0],base_x_start[0],base_y_start[0]));
                      
                      // compute new length also.. to do
                      length_base = line2.length;
                  }
              } else {
                  if(line1.length>0){
                    tip_x_start[0] = line1.x1;
                    tip_y_start[0] = line1.y1;
                    length_tip = line1.length;
                    orientation_tip = lineOrientation(line1);
                  }
                  if(line2.length>0){
                       base_x_start[0] = line2.x1;
                       base_y_start[0] = line2.y1;
                       base_x_end[0] = line2.x0;
                       base_y_end[0] = line2.y0;
                       length_base = line2.length;
                       orientation_base = lineOrientation(line2);
                  }
                 
                 direction_tip = lineDirection(new Line(tip_x_start[0],tip_y_start[0],tip_x_end[0],tip_y_end[0])); 
                 direction_base = lineDirection(new Line(base_x_start[0],base_y_start[0],base_x_end[0],base_y_end[0])); 
                  
                  
                  
              }     
             
              
              
              
              
          }
                          
          
          // update finger position every time bin, do nothing new if finger alive but
          // if no assigned fingertip for too long then pull by other fingers tips if
          // they are activated and alive
          public void updatePosition( FingerCluster[] allClusters, int ownIndex ){
              // if activated and lifetime > zero
              if(activated){
                 if (lifetime>0){
                    lifetime--;
                 } else {
                     // if activated but lifetime at zero
                     // allow pulling by closest neighbours
                     
                     // find closest neighbours that are activated and alive
                     // if one or two found, apply pull to nearest valid zone
                     // or detect possible best location (like if range too short but
                     // some good fingertips are available
                 }
              } else {
                  lifetime=0;
              }
        
          }
          
      } // end Class FingerCluster
     
      
      protected float meanParallelToContour( int x1, int y1, int x2, int y2, float orientation, float[][][] fr){
          float meanOrientation = -1f;
          // find a point in the middle of closer to x1,y1
          float closeness = 0.6f;
          int xm = (int)(closeness*x1+(1-closeness)*x2);
          int ym = (int)(closeness*y1+(1-closeness)*y2);
          float closeness2 = 0.3f;
          int xm2 = (int)(closeness2*x1+(1-closeness2)*x2);
          int ym2 = (int)(closeness2*y1+(1-closeness2)*y2);
          
          // from point xm,ym find intersection with contour of two lines leaving at 33 and 66 degrees
          Vector somelines = new Vector();
          int range = parallel_range;
          Line l1 = longestLine(xm,ym,range,orientation+90,fr);
          
          Line l2 = longestLine(xm2,ym2,range,orientation+90,fr);
          
          somelines.add(new Line(l1.x1,l1.y1,l2.x1,l2.y1));

           
          Line l3 = longestLine(xm,ym,range,orientation-90,fr);
          Line l4 = longestLine(xm2,ym2,range,orientation-90,fr);

          somelines.add(new Line(l3.x1,l3.y1,l4.x1,l4.y1));

          // compute mean line
          Line meanLine = meanLineOf(somelines);
          meanOrientation = meanLine.orientation;
          
          return meanOrientation;
      }
         
      // thinning picture
      // Thinning algorithm: CACM 1984 march (Zhang and Suen)
      
      int[][][] thresholdImage( float[][][] image, float threshold ){
          int[][][] img = new int[image.length][image[0].length][image[0][0].length];
         
          for(int i=0;i<image.length;i++){
              for(int j=0;j<image[0].length;j++){
                 if(image[i][j][0]<=threshold){
                     img[i][j][0]=0;
                 } else {
                    img[i][j][0]=1; 
                 }
                  
              }
          }
          
          return img;
          
      }
      
      // destructive and on previously thresholded image
      int[][][] thin( int[][][] image ){
          int i=0,j=0,n=0,m=0,k=0, br=0,ar=0,p1=0,p2=0;
          int nn = image.length;
          int mm = image[0].length;
          boolean cont = true;
          int[] a = new int[8];
          int[] arbr = new int[2];
          int[][][] y = new int[image.length][image[0].length][image[0][0].length];
          
          while(cont){
              cont  = false;
              /*	Sub-iteration 1: */
		for (i=0; i<nn; i++)
		  for (j=0; j<mm; j++) {		/* Scan the entire image */
			if (image[i][j][0] == 0 ) {
				y[i][j][0] = 0;
				continue;
			}
			arbr = t1a (image, i, j, a, br,nn,mm);	/* Function A */
                        ar = arbr[0];
                        br = arbr[1];
                        
			p1 = a[0]*a[2]*a[4];
			p2 = a[2]*a[4]*a[6];
			if ( (ar == 1) && ((br>=2) && (br<=6)) &&
				(p1 == 0) && (p2 == 0) )  {
					y[i][j][0] = 1;
					cont = true;
			}
			else y[i][j][0] = 0;
		}
		subtr (y, image,nn,mm);
              
                /* Sub iteration 2: */
		for (i=0; i<nn; i++)
		  for (j=0; j<mm; j++) {		/* Scan the entire image */
			if (image[i][j][0] == 0 ) {
				y[i][j][0] = 0;
				continue;
			}
			arbr = t1a (image, i, j, a, br,nn,mm);	/* Function A */
                        ar = arbr[0];
                        br = arbr[1];
			p1 = a[0]*a[2]*a[6];
			p2 = a[0]*a[4]*a[6];
			if ( (ar == 1) && ((br>=2) && (br<=6)) &&
				(p1 == 0) && (p2 == 0) )  {
					y[i][j][0] = 1;
					cont = true;
			}
			else y[i][j][0] = 0;
		}
		subtr (y, image,nn,mm);
              
          }
         return image;
      }
      
      int[] t1a( int[][][] image, int i, int j, int[] a, int br, int nn, int mm ){
          int[] arbr = new int[2];
          //...
          /*	Return the number of 01 patterns in the sequence of pixels
	P2 p3 p4 p5 p6 p7 p8 p9.					*/

	int n,m;
        int b;
	for (n=0; n<8; n++) a[n] = 0;
	if (i-1 >= 0) {
		a[0] = image[i-1][j][0];
		if (j+1 < mm) a[1] = image[i-1][j+1][0];
		if (j-1 >= 0) a[7] = image[i-1][j-1][0];
	}
	if (i+1 < nn) {
		 a[4] = image[i+1][j][0]; 
		if (j+1 < mm) a[3] = image[i+1][j+1][0];
		if (j-1 >= 0) a[5] = image[i+1][j-1][0];
	}
	if (j+1 < mm) a[2] = image[i][j+1][0];
	if (j-1 >= 0) a[6] = image[i][j-1][0];

	m= 0;	b = 0;
	for (n=0; n<7; n++) {
		if ((a[n]==0) && (a[n+1]==1)) m++;
		b = b + a[n];
	}
	if ((a[7] == 0) && (a[0] == 1)) m++;
	b = b + a[7];
	
        arbr[0] = m;
        arbr[1] = b;
          return arbr;
      }
      
      void subtr( int[][][] a, int[][][] b, int n, int m){
          
            for (int i=0; i<n; i++){
		for (int j=0; j<m; j++) {
			b[i][j][0] -= a[i][j][0];
                }
            }
      }
      
      private class Line{
        
        int length;
        int x0;
        int y0;
        int x1;
        int y1;
        int midx;
        int midy;
        float orientation=0;
        
        public boolean touchesDoor = false;
        
        public Line(){
            length = 0;
            x0 = 0;
            y0 = 0;
            x1 = 0;
            y1 = 0;
            midx = 0;
            midy = 0;
        }
        public Line( int x0, int y0, int x1, int y1){
            
            this.x0 = x0;
            this.y0 = y0;
            this.x1 = x1;
            this.y1 = y1;
        }
        public Line( int x0, int y0, int x1, int y1, int length){
            this.length = length;
            this.x0 = x0;
            this.y0 = y0;
            this.x1 = x1;
            this.y1 = y1;
        }
        public Line( int x0, int y0, int midx, int midy, int x1, int y1, int length){
            this.length = length;
            this.x0 = x0;
            this.y0 = y0;
            this.x1 = x1;
            this.y1 = y1;
            this.midx = midx;
            this.midy = midy;
        }                     
    }
    
      
      
      
    private class LineLengthComparer implements Comparator {
                public int compare(Object obj1, Object obj2)
                {
                        int l1 = ((Line)obj1).length;
                        int l2 = ((Line)obj2).length;
        
                        return l2 - l1;
                }
    }
       
       
    protected float distanceBetween( int x1, int y1, int x2, int y2){
          
          double dx = (double)(x1-x2);
          double dy = (double)(y1-y2);
          
          float dist = (float)Math.sqrt((dy*dy)+(dx*dx));
          return dist;
      }
      
    private class Point{
        
        int score;
        int disabled;
        int x;
        int y;
        
        public Point(){
            score = 0;
            disabled = 0;
            x = 0;
            y = 0;
        }
        public Point( int x, int y){
            this.score = 0;
            this.disabled = 0;
            this.x = x;
            this.y = y;
        }
        public Point( int x, int y, int score){
            this.score = score;
            disabled = 0;
            this.x = x;
            this.y = y;
        }
        
        
    }
    
    
    
    private class ScoreComparer implements Comparator {
                public int compare(Object obj1, Object obj2)
                {
                        int i1 = ((Point)obj1).score;
                        int i2 = ((Point)obj2).score;
        
                        return i2 - i1;
                }
    }

       private class Minimum{
        
        int value;
        int index;
       
        public Minimum(){
            value = 0;
            index = 0;
            
        }
        public Minimum( int value, int index){
            this.value = value;
            this.index = index;
            
        }
        
        
    }
    
    
    private class MinimaComparer implements Comparator {
                public int compare(Object obj1, Object obj2)
                {
                        int i1 = ((Minimum)obj1).value;
                        int i2 = ((Minimum)obj2).value;
        
                        return i1 - i2;
                }
    }
    
       //detect finger lines in array of inside intensity and array of accumulated contrast change
    // after use of finger tips are detected
    /**
    private Vector detectFingersLines(  float[][][] fr ){
        Vector lines = new Vector();
        
        // for each points in tips        
        // find longest line with acc values > threshold, and stopping if touching door
        // at tip distance, if pt.x,y too close to shape, put it to mindistance
        // if too close from both sides, put it at the middle
        
        
        for(int i=0;i<fingerTipClusters.length;i++){
             // find longest line with acc values > threshold, and stopping if touching door
            FingerCluster fc = fingerTipClusters[i];
            if(fc.activated){
                int range = 70; //to parametrize or #define
                int midlength = 13;//to parametrize although it is unused yet
                Vector somelines = longestLines(fc.x,fc.y,range,midlength,lines_n_avg,fr);

                // compute mean line
                Line meanLine = meanLineOf(somelines);
                // lines.addAll(somelines);
                if (meanLine!=null) {
                    lines.add(meanLine);
                    // add finger information to cluster
                    fc.addFinger(meanLine,fr);
                    
                }
            }
        }
      
        return lines;
    }
    */
    
     //detect finger lines in array of inside intensity and array of accumulated contrast change
    // after finger tips are detected
    private Vector detectFingersLines( Vector tips, float[][][] fr ){
        Vector lines = new Vector();
        
        // for each points in tips        
        // find longest line with acc values > threshold, and stopping if touching door
        // at tip distance, if pt.x,y too close to shape, put it to mindistance
        // if too close from both sides, put it at the middle
        
        
        for(int i=0;i<tips.size();i++){
             // find longest line with acc values > threshold, and stopping if touching door
            Point sc = (Point)tips.elementAt(i);
            int range = 70;
            int midlength = 13;
            Vector somelines = longestLines(sc.x,sc.y,range,midlength,lines_n_avg,fr);

            lines.addAll(somelines);
            
        }
        
        
        // line = startpt, midpt, endpt
        // if endpt not touching door, try finding a new longest line to door
        
        // then remove duplicate.
        
        // to put back, paul :
       
//        
//        if(lines.size()>4){
//            
//            for(int i=0;i<lines.size();i++){
//               Line l1 = (Line)lines.elementAt(i);
//               for(int j=i+1;j<lines.size();j++){
//                    Line l2 = (Line)lines.elementAt(j);
//                    if(tooClose(l1.midx,l1.midy,l2.midx,l2.midy,line_range)){
//                        if(l1.length>l2.length){
//                            // remove l2
//                            lines.remove(j);
//                            j--;
//                        } else {
//                            // remove l1
//                            lines.remove(i);
//                            i--;
//                            j = lines.size();
//                        }
//                    }
//               } 
//            }
//        }
        // if mid pt too close to another line, keep longest touching door
        
        
        return lines;
    }
    
   
    //detect finger tip in array of inside intensity and array of accumulated contrast change
    private Vector detectFingersTips( float[][][] ir, float[][][] fr ){
          
        Vector scores = new Vector();
          // look at intensities, (or another array if not define?)
          // for each point store x,y and score if score < threshold
        // score is matching score, 1 pts for right matching per pixel, -1 for wrong(?)
          int score = 0;
          for (int i = 0; i<ir.length; i++){
              for (int j = 0; j<ir.length; j++){
                 if(ir[i][j][0]>0){
                     score = testFingerTipPattern(fr,j,i); //careful with x,y or y,x
                     if(score>score_threshold){
                          // store
                          Point sc = new Point(i,j,score);
                          
                          scores.addElement(sc);
                          //System.out.println("add scores "+sc.x+","+sc.y+" : "+score);
                     } 
                     scoresFrame[i][j][0] = score;
                     
                 } else {
                         scoresFrame[i][j][0] = 0;
                 }
              }
          }
          
          
        
        // sort score and points
        Collections.sort(scores, new ScoreComparer());  
        int range = score_range;  
        
        // for all best scores
        // then remove duplicate, for each best score define area
        // for all other points
        // if x,y in area of previous best, delete
        // then next best score
        
        /* comment to show finger nails :) */
        for(int i=0;i<scores.size();i++){
            Point sc = (Point)scores.elementAt(i);
            for(int j=i+1;j<scores.size();){
                Point sc2 = (Point)scores.elementAt(j);
                // if sc2 in area of sc1, delete
                if(((sc2.x<sc.x+range)&&(sc2.x>sc.x-range))&&
                       ((sc2.y<sc.y+range)&&(sc2.y>sc.y-range)) ){
                    //delete
                    scores.remove(j);
                   // j++; //if no remove for debug
                } else {
                // increase only if no deletion, to avoid jumping scores
                    j++;
                }
            }
            
        }
        
       
        
        /* */
        // then return all best score in order, into pawtippoints array or such
//        Point p0;
//        if(scores.size()>0)  {
//        p0 = (Point)scores.elementAt(0);
//        System.out.println("1 best score:"+p0.score+" for x:"+p0.x+" y:"+p0.y);
//        }
//        if(scores.size()>1)  {
//        p0 = (Point)scores.elementAt(1);
//        System.out.println("2 best score:"+p0.score+" for x:"+p0.x+" y:"+p0.y);
//        }
//        if(scores.size()>2)  {
//         p0 = (Point)scores.elementAt(2);
//        System.out.println("3 best score:"+p0.score+" for x:"+p0.x+" y:"+p0.y);
//        }
//        if(scores.size()>3)  {
//         p0 = (Point)scores.elementAt(3);
//        System.out.println("4 best score:"+p0.score+" for x:"+p0.x+" y:"+p0.y);
//        }
//        if(scores.size()>4)  {
//         p0 = (Point)scores.elementAt(4);
//        System.out.println("5 best score:"+p0.score+" for x:"+p0.x+" y:"+p0.y);
//        }
          return scores;
          
    }
      
    private int testFingerTipPattern( float[][][] fr, int x, int y){
        int score = 0;
        float threshold = score_in_threshold; // to adapt
        
        int xInside = 2;
        int yInside = 2;
        int xOutside = 3;
        int yOutside = 3;
        int distance1 = 3;
        int distance2 = 7;
        
        // look for positive value around x,y
        //test points in an area around
        for (int i = x-xInside; i<x+xInside; i++){
              if(i>=0&&i<fr.length){
                  for (int j = y-yInside; j<y+yInside; j++){
                      if(j>=0&&j<fr[i].length){
                      
                        if(fr[i][j][0]>threshold){
                            score++;
                         }
                      }
                  }
              }
        }
              
        // look for negative lines around 
        for (int i = x-xOutside; i<x+xOutside; i++){
              if(i>=0&&i<fr.length){
                 
                      if(y-distance1>=0){
                         if(fr[i][y-distance1][0]<threshold){
                            score++;
                         }  
                      }
                      if(y+distance1<fr.length){
                         if(fr[i][y+distance1][0]<threshold){
                            score++;
                         }
                       
                      }
                   
                      if(y-distance2>=0){
                         if(fr[i][y-distance2][0]<threshold){
                            score++;
                         }  
                      }
                      if(y+distance2<fr.length){
                         if(fr[i][y+distance2][0]<threshold){
                            score++;
                         }
                       
                      }
                  
              }
        }
        for (int j = y-yOutside; j<y+yOutside; j++){
              if(j>=0&&j<fr.length){
                 
                      if(x-distance1>=0){
                         if(fr[x-distance1][j][0]<threshold){
                            score++;
                         }  
                      }
                      if(x+distance1<fr.length){
                         if(fr[x+distance1][j][0]<threshold){
                            score++;
                         }
                       
                      }
                      if(x-distance2>=0){
                         if(fr[x-distance2][j][0]<threshold){
                            score++;
                         }  
                      }
                      if(x+distance2<fr.length){
                         if(fr[x+distance2][j][0]<threshold){
                            score++;
                         }
                       
                      }
              }
              
        }
        // remove points with no support from actual real accumulated image
        if(fr[x][y][0]<=score_sup_threshold){
            score = 0;
        }        
        
        return score;
        //return (int) (score * fr[x][y][0] * 10);
    }
      
    // check if point is on paw's shape's border
    private boolean isOnPawShape(int x, int sx, int y, int sy, float min, float max, float mean, int minz, int maxz, float[][][] f){
        
        int zerosAround = 0;
        
        // only positive point above 0.6
//        if((x+1>=retinaSize)||(y+1>=retinaSize)){
//            return false;
//        }
       
        if((f[x+1][y+1][0])<max){//||(f[x+1][y+1][0])==mean){
            return false;
        }
        
        // look around
        for(int i=x;i<=sx;i++){
            for(int j=y;j<=sy;j++){
                // if inside limits
                if((i>0)&&(j>0)&&(i<retinaSize)&&(j<retinaSize)){
                    //if (x==entry_x-1&&y==entry_y-1) System.out.println("events at "+i+","+j+" ="+f[i][j][0]+" "+f[i][j][1]+" "+f[i][j][2]);
                    if((f[i][j][0])<min||(f[i][j][0])==mean){
                        zerosAround++;
                    }
                    
                }
            }
        }
        
        
        if(zerosAround>minz&&zerosAround<maxz){
            return true;
        }
        
        
        return false;
    }
    
       // check if point is on paw's shape's border
    // to change to implement steep border detector using lines
    private boolean isShapeBorder(int x, int y, float min, float act_threshold, float[][][] f){
        
        
        // only positive point above 0.6
//        if((x+1>=retinaSize)||(y+1>=retinaSize)){
//            return false;
//        }
       
        // look only at points above threshold
        if((f[x][y][0])<act_threshold){//||(f[x+1][y+1][0])==mean){
            return false;
        }
        
        // look around point for neighbour at zero
        for(int i=x-contour_range;i<=x+contour_range;i++){
            for(int j=y-contour_range;j<=y+contour_range;j++){
                // if inside limits
                if((i>0)&&(j>0)&&(i<retinaSize)&&(j<retinaSize)){
                   
                    //if (x==entry_x-1&&y==entry_y-1) System.out.println("events at "+i+","+j+" ="+f[i][j][0]+" "+f[i][j][1]+" "+f[i][j][2]);
                    if(f[i][j][0]<=min){
                        return true;
                    }
                    
                }
            }
        }
        
       
        
        return false;
    }
    
    
    
    
    
    public Contour findContour(float[][][] f){
        // return contour
        Contour contour = new Contour();
        
        // for all positive points look if surrounded by at least 3 neg and 3 pos
        for (int i=1;i<retinaSize;i++){
            for (int j=1;j<retinaSize;j++){
                // if inside door, correction
                boolean onShape = false;
//                if (i>door_xa&&i<door_xb&&j>door_ya&&j<door_yb){
//                    onShape = isOnPawShape(i-1,i+1,j-1,j+1,doorMinDiff,doorMaxDiff,0.4f,doorMinZeroes,doorMaxZeroes,f);
//                    //onShape = isOnPawShape(i-1,i+1,j-1,j+1,minDiff,maxDiff,0.5f,minZeroes,maxZeroes,f);
//                } else {
                   if(useSimpleContour){
                    onShape = isShapeBorder(i,j,contour_min_thresh,contour_act_thresh,f);
                    
                   } else {
                    onShape = isOnPawShape(i-1,i+1,j-1,j+1,minDiff,maxDiff,0.5f,minZeroes,maxZeroes,f);
                   }
              //  }
                if(onShape){
                    contour.add(j,i);
                }
                
            }
        }
        
        
        return contour;
        
    }
    
    public class Segment{
        protected int maxX=128,maxY=128;
        
        public int x1;
        public int y1;
        public int x2;
        public int y2;
        
        // point inside paw        
        public int xi =0;
        public int yi =0;
        // point outside paw        
        public int xo =0;
        public int yo =0;
        
        public int acc; //number of identical values //to get rid of
        public double distGC;
        public double orientation;
        public double size;
        
        public Segment(){
            reset();
        }
        public Segment(ContourPoint c1, ContourPoint c2 ){
            x1 = c1.x;
            y1 = c1.y;
            x2 = c2.x;
            y2 = c2.y;
        }
        
        
        public void reset(){
            
            
        }
        
        public int midx(){
            return (int)((x1+x2)/2); //+0.5?
        }
        public int midy(){ 
            return (int)((y1+y2)/2);
        }
        
        
    }
    
    // find the gravity center of a labelled contour
    public ContourPoint findContourGC( Contour contour, int label ){
        float x = 0f;
        float y = 0f;
        int n = 0;
        ContourPoint gc = new ContourPoint();
        // for all contour points
        
        for (int i=0;i<contour.getMaxX();i++){
            for (int j=0;j<contour.getMaxY();j++){
                if (contour.eventsArray[i][j].on==1){
                    if (contour.eventsArray[i][j].label==label){
                        x += contour.eventsArray[i][j].x;
                        y += contour.eventsArray[i][j].y;
                        n++;
                    }
                }
            }
        }
        x = x/n;
        y = y/n;
        gc.x = new Float(x).intValue();
        gc.y = new Float(y).intValue();
        return gc;
    }
    
    protected void resetDoubleArray( int[][] array, int x_max, int y_max ){
        for(int i=0;i<x_max;i++){
                    for(int j=0;j<y_max;j++){                     
                        array[i][j] = 0;
                    }
        }
    }
    
    // test if no points in a vector are touching  
    protected boolean testSeparation( Vector points ){
        for (Object o1:points){
            ContourPoint cp1 = (ContourPoint)o1;
            for (Object o2:points){
                ContourPoint cp2 = (ContourPoint)o2;
                if(o1!=o2){
                    if(distanceBetween(cp1.x,cp1.y,cp2.x,cp2.y)<=1){
                        return false;
                    }                    
                }
            }
            
        }
        return true;
    }
    
    public Vector detectNodes( Contour contour, int label, int range  ) {
        int nbNodes = 0;
        Vector nodes = new Vector();
        // for each point, look at neighbours. if only one, end node
        // if two, if not "aligned" maybe joint node
        // if three, cross node
        int maxX = contour.getMaxX();
        int maxY = contour.getMaxY();
        for (int i=0;i<maxX;i++){
            for (int j=0;j<maxY;j++){
                if (contour.eventsArray[i][j].on==1){ //could optimize this by putting al on points into a specific array
                    if (contour.eventsArray[i][j].label==label){ 
                        contour.eventsArray[i][j].used = 0;
                        Vector neighbours = contour.getNeighbours(i,j,range);
                        if(neighbours.size()==1){
                            // end node
                            Node endNode = new Node(i,j,0); // 0 for end node, to define somewhere
                            // measure node value (high if end of long segment)
                            //endNode.measureSupport(contour);
                            endNode.measureSupport(contour);
                            nodes.add(endNode);
                        } else if(neighbours.size()>2){
                          //  if(testSeparation(neighbours)){
                           //     Node crossNode = new Node(i,j,1); // 1 for cross node, to define somewhere
                           //     nodes.add(crossNode);
                          //}
                        } //else if(neighbours.size()==2){
                            // could be joint node
                            
                       // }
                        
                    }                                             
                }
            }
        }
        
        
        return nodes;       
    }
    
    /** detectSegments : new method to detect segments, replaces computeLocalOrientations
     *  create segments out of points from the contour labelled 1 (touching door)
     *  
     *
     **/
     public int detectSegments( Contour contour, int label, int range, Segment[] segments, float[][][] fr, boolean useIntensity ){
        //gc = findContourGC(contour,label);
        
      //  gc.x = 71;
      //  gc.y = 62;
       // resetDoubleArray(orientations,MAX_ORIENTATIONS,MAX_DISTGC);
//        maxOrientation = 0;
//        maxDistGC = 0;
 
       
        int indexSegment = 0;
        int overMaxNbSegments = 0;
        int indexSeq = 0;
        accOrientationMax = 1;
        // for each point find segment it belongs to
        int maxX = contour.getMaxX();
        int maxY = contour.getMaxY();
        for (int i=0;i<maxX;i++){
            for (int j=0;j<maxY;j++){
                if (contour.eventsArray[i][j].on==1){ //could optimize this by putting all on points into a specific array
                    if (contour.eventsArray[i][j].label==label){                                                

                        // for all neighbours
                        
                        // remove point from further search
                        contour.eventsArray[i][j].used = 1;
                        
                        // find random neighbour in range
                        ContourPoint neighbour = contour.getNeighbour(i,j,range);
                        
                                              
                        if(neighbour!=null){
                            // remove point from further search
                            neighbour.used = 1;
                            
                            //create segment for this two points, to be extended
                            Segment segment = new Segment(contour.eventsArray[i][j],neighbour);
                            
                            // determine policy
                             
                            int x_min_policy = 0;
                            int x_max_policy = 0;
                            int y_min_policy = 0;
                            int y_max_policy = 0;
                            
                            if(neighbour.x<i){
                                x_min_policy = -range;
                                if(neighbour.y==j) x_max_policy = -1;
                                // x_max_policy = 0;
                            }
                            if(neighbour.x==i){
                                x_min_policy = -1;
                                x_max_policy = 1;
                            }
                            if(neighbour.x>i){
                                if(neighbour.y==j) x_min_policy = 1;
                                //x_min_policy = 0;
                                x_max_policy = range;
                            }
                            //same for y
                            if(neighbour.y<j){
                                y_min_policy = -range;
                                if(neighbour.x==i) y_max_policy = -1;
                                // x_max_policy = 0;
                            }
                            if(neighbour.y==j){
                                y_min_policy = -1;
                                y_max_policy = 1;
                            }
                            if(neighbour.y>j){
                                if(neighbour.x==i) y_min_policy = 1;
                                //x_min_policy = 0;
                                y_max_policy = range;
                            }
                          
                            
                            // recursive call for finding further points of the segment
                            contour.extendSegmentFollowingPolicy(segment,neighbour,x_min_policy,x_max_policy,y_min_policy,y_max_policy,1,false);
                            //after building a segment, we free its start point so that other segment can link to it
                            contour.eventsArray[i][j].used = 0;

                            
                            // now we have final segment
                            // compute its orientation
                            double dx = (double)(segment.x2-segment.x1);
                            double dy = (double)(segment.y2-segment.y1);
                                                  
                            segment.size = Math.sqrt((dy*dy)+(dx*dx));
                                                     
                            segment.orientation = Math.toDegrees(Math.acos(dx/segment.size));
//                            if (segment.y1>segment.y2){
//                                segment.orientation = 180-segment.orientation;
//                            }
                            
                            if(useIntensity){
                                // if paint inside
                                // for all points in inside array
                                // increase intensity by x
                                // along inside axe perpendicular to orientation
                                // function :
                                double insideDir = findInsideDirOfSegment(segment,fr);
                                
                                if(insideDir!=400){
                                    // then find start point of rectangle
                                    //           xi = cos(inside_dir)(degreetorad) * length
                                    //           yi = sin(inside_dir)(degreetorad) * length
                                    
                                    int xi = segment.x1 + (int)(Math.cos(Math.toRadians(insideDir))*in_length+0.5); // +0.5 for rounding, maybe not
                                    int yi = segment.y1 + (int)(Math.sin(Math.toRadians(insideDir))*in_length+0.5); // +0.5 for rounding, maybe not
                                    
                                    //  then increase intensity in rectangle
                                    increaseIntensityOfRectangle(segment.x1,segment.y1,xi,yi,segment.x2,segment.y2);
                                } 
                            }

                            // store segment
                            if(indexSegment<segments.length){
                               segments[indexSegment] = segment;
                               indexSegment++;
                            } else {
                                overMaxNbSegments++;
                            }
                            
//
//            System.out.println("after store segment "+i+","+j+" segment: "+segment);
//
//            try {
//
//                String str = new BufferedReader(new InputStreamReader(System.in)).readLine();
//            } catch (IOException e) {
//            // TODO do exception handling
//            }
                            
                        } //end if neighbour is null
                    }

                }
            }
        }
        if (overMaxNbSegments>0) {
            int exceed = overMaxNbSegments+indexSegment;
            System.out.println("detectSegments segments exceed max: "+exceed+" > "+MAX_SEGMENTS);
            
        }
       
        return indexSegment;
    }
     
     
   
     // detect inside direction
//           test both orthogonal directions to segment orientation
//                   first orthogonal direction:
//                   point a : xa = cos(ori+90)(degreetorad) * testlength
//                   ya = sin(ori+90)(degreetorad) * testlength
//                   then second :
//                   point b : xb = cos(ori-90)(degreetorad) * testlength
//                   yb = sin(ori-90)(degreetorad) * testlength
//
//                   va = testline(segment.midx,midy,xa,ya)
//                   vb = testline(segment.midx,midy,xb,yb)
//                   if va>vb inside_dir = ori+90 else = ori-90

    protected double findInsideDirOfSegment( Segment s, float[][][] fr){
        double dir = 0;
        double testLength = in_test_length; // 
        
        int midx = s.midx(); //int)((s.x1+s.x2)/2); //+0.5?
        int midy = s.midy(); //int)((s.y1+s.y2)/2);
        
        // points defining lines at the two orthogonal directions to segment orientation
        int xa = midx + (int)(Math.cos(Math.toRadians(s.orientation+90))*testLength); // +0.5 for rounding, maybe not
        int ya = midy + (int)(Math.sin(Math.toRadians(s.orientation+90))*testLength); // +0.5 for rounding, maybe not
        int xb = midx + (int)(Math.cos(Math.toRadians(s.orientation-90))*testLength); // +0.5 for rounding, maybe not
        int yb = midy + (int)(Math.sin(Math.toRadians(s.orientation-90))*testLength); // +0.5 for rounding, maybe not
 
        
        if(xa<0||xa>=retinaSize){           
           
            //System.out.println("findInsideDirOfSegment error xa="+xa+" for midx="+midx+" and ori="+s.orientation+"+90");
            return 400;
        }
        if(xa<0||xa>=retinaSize){
         
            //System.out.println("findInsideDirOfSegment error ya="+ya+" for midy="+midy+" and ori="+s.orientation+"+90");
            return 400;
        }
         if(xb<0||xb>=retinaSize){           
          
            //System.out.println("findInsideDirOfSegment error xb="+xb+" for midx="+midx+" and ori="+s.orientation+"-90");
            return 400;
        }
        if(yb<0||yb>=retinaSize){          
        
            //System.out.println("findInsideDirOfSegment error yb="+yb+" for midy="+midy+" and ori="+s.orientation+"-90");
            return 400;
        }
        float va = meanValueOfLine(midx,midy,xa,ya,fr);
        float vb = meanValueOfLine(midx,midy,xb,yb,fr);
        
        if(va>vb){
            
                dir = s.orientation+90;
                s.xi = xa;
                s.yi = ya;
                s.xo = xb;
                s.yo = yb;
                
            
        } else { //va<vb
           
                dir = s.orientation-90;
                s.xi = xb;
                s.yi = yb;
                s.xo = xa;
                s.yo = ya;
            
        }
        
        return dir;
    }
   
    
      // stops if touching door or pixel below threshold
    protected Line longestLine(int x, int y, int range, float orientation, float[][][] accEvents){
           float threshold = line_threshold;
           int shape_min_distance = 1;
           // find x1,y1 at range long orientation
           int x1 = x + (int)(Math.cos(Math.toRadians(orientation))*range);
           int y1 = y + (int)(Math.sin(Math.toRadians(orientation))*range);
          
           int x0 = x;
           int y0 = y;
          
             
                    
                    int dy = y1 - y0;
                    int dx = x1 - x0;
                    int stepx, stepy;
                    int length = 0;
                    
                    if (dy < 0) { dy = -dy;  stepy = -1; } else { stepy = 1; }
                    if (dx < 0) { dx = -dx;  stepx = -1; } else { stepx = 1; }
                    dy <<= 1;                                                  // dy is now 2*dy
                    dx <<= 1;                                                  // dx is now 2*dx

        
                    if (dx > dy) {
                        int fraction = dy - (dx >> 1);                         // same as 2*dy - dx
                        while (x0 != x1) {
                            if (fraction >= 0) {
                                y0 += stepy;
                                fraction -= dx;                                // same as fraction -= 2*dx
                            }
                            x0 += stepx;
                            fraction += dy;                                    // same as fraction -= 2*dy
                            if(x0>0&&x0<retinaSize&&y0>0&&y0<retinaSize){
                                if(accEvents[y0][x0][0]>threshold){
                                    
                                    if((length+1<shape_min_distance)||((contour.eventsArray[x0][y0].label!=1)
                                      ||(contour.eventsArray[x0][y0].on!=1))){//and check if within shape only after a few timestep
                                    length++;
                                                                     
                                    } else {
                                        break;
                                    }
                                    
                                } else {
                                    break;
                                }
                            } else {
                                break;
                            }

                            
                        }
                    } else {
                        int fraction = dx - (dy >> 1);
                        while (y0 != y1) {
                            if (fraction >= 0) {
                                x0 += stepx;
                                fraction -= dy;
                            }
                            y0 += stepy;
                            fraction += dx;
                            if(x0>0&&x0<retinaSize&&y0>0&&y0<retinaSize){
                                if(accEvents[y0][x0][0]>threshold){
                                    // 3 : here is the area around the start point in which we dont check fo shape border, in case
                                    // of border noise // should parametrize
                                 if((length+1<shape_min_distance)||((contour.eventsArray[x0][y0].label!=1)
                                      ||(contour.eventsArray[x0][y0].on!=1))){//and check if within shape only after a few timestep
                              
                                    length++;
                                   
                                    } else {
                                        break;
                                    }
                                } else {
                                    break;
                                }
                            } else {
                                break;
                                
                            }
                        }
                    }
                   // end computing line, end point in x0,y0
                    
                 
                   // store  line
                    Line line = new Line(x,y,x0,y0,length);
            
        return line; 
    }
    
     // stops if touching shape or pixel below threshold and value increase if touching point in image
    protected Line mostValuableLine(int x, int y, int range, Contour thin , float[][][] accEvents){
     
        return mostValuableLine(x, y, range, 180, 360, thin , accEvents);
    }
     // stops if touching shape or pixel below threshold and value increase if touching point in image
    protected Line mostValuableLine(int x, int y, int range, float orientation, float variation, Contour thin , float[][][] accEvents){
        
        
        Line result = new Line();
        
           // compute x1s and y1s based on range
           // for all points in a square outline centered on x0,y0 with side size range+1/2
           // find length of line
           // if above max, x,y dest = x1,y1 and max = length, touchingdoor= true/false accordingly
           float threshold = line_threshold;
           
           int x1 = 0;
           int y1 = 0;
           int x0 = 0;
           int y0 = 0;
           int xEnd = 0;
           int yEnd = 0;
           int lengthMax = 0;
           int valueMax = 0;
           Vector visited = new Vector();
           
           // for all points in a square outline centered on x0,y0 with side size range+1/2
           for(int i=x-range;i<x+range+1;i++){
              Vector onTheLine = new Vector();
              for(int j=y-range;j<y+range+1;j++){
                if(((i<=x-range)||(i>=x+range))||((j<=y-range)||(j>=y+range))){
                    // on the square outline
                    // if within demanded orientation acceptance
                    
                    
                    x1 = i; 
                    y1 = j;
                    x0 = x;
                    y0 = y;                                      
                    
                    float targetOrientation = lineDirection(new Line(x0,y0,x1,y1));
                    
                    if((targetOrientation>orientation-variation)&&(targetOrientation<orientation+variation)){
                        
                    
                    
                    int dy = y1 - y0;
                    int dx = x1 - x0;
                    int stepx, stepy;
                    int length = 0;
                    int value = 0;
                    
                    if (dy < 0) { dy = -dy;  stepy = -1; } else { stepy = 1; }
                    if (dx < 0) { dx = -dx;  stepx = -1; } else { stepx = 1; }
                    dy <<= 1;                                                  // dy is now 2*dy
                    dx <<= 1;                                                  // dx is now 2*dx

        
                    if (dx > dy) {
                        int fraction = dy - (dx >> 1);                         // same as 2*dy - dx
                        while (x0 != x1) {
                            if (fraction >= 0) {
                                y0 += stepy;
                                fraction -= dx;                                // same as fraction -= 2*dx
                            }
                            x0 += stepx;
                            fraction += dy;                                    // same as fraction -= 2*dy
                            if(x0>0&&x0<retinaSize&&y0>0&&y0<retinaSize){
                                if(accEvents[y0][x0][0]>threshold){
                                    // if length+1<3 : so as not to stop at shape border if too early to check because of noise in shape
                                    if((length+1<3)||((contour.eventsArray[x0][y0].label!=1)
                                      ||(contour.eventsArray[x0][y0].on!=1))){//and check if within shape only after a few timestep
                                        length++;
                                        value+=thin.eventsArray[x0][y0].on; // should check dimension
                                        onTheLine.add(new Point(x0,y0));
                                   // System.out.println("value line: for img["+x0+"]["+y0+"]="+image[x0][y0][0]+" and img["+y0+"]["+x0+"]="+image[y0][x0][0]);
              
                                    } else {
                                        break;
                                    }
                                    
                                } else {
                                    break;
                                }
                            } else {
                                break;
                            }

                            
                        }
                    } else {
                        int fraction = dx - (dy >> 1);
                        while (y0 != y1) {
                            if (fraction >= 0) {
                                x0 += stepx;
                                fraction -= dy;
                            }
                            y0 += stepy;
                            fraction += dx;
                            if(x0>0&&x0<retinaSize&&y0>0&&y0<retinaSize){
                                if(accEvents[y0][x0][0]>threshold){
                                    // 3 : here is the area around the start point in which we dont check fo shape border, in case
                                    // of border noise // should parametrize
                                 if((length+1<3)||((contour.eventsArray[x0][y0].label!=1)
                                      ||(contour.eventsArray[x0][y0].on!=1))){//and check if within shape only after a few timestep
                              
                                        length++;
                                        value+=thin.eventsArray[x0][y0].on; // should check on image dimension before
                                        onTheLine.add(new Point(x0,y0));
                                    } else {
                                        break;
                                    }
                                } else {
                                    break;
                                }
                            } else {
                                break;
                                
                            }
                        }
                    }
                   // end computing line, end point in x0,y0
                    
                    // memorize max length
                     if(length>lengthMax){
                        lengthMax=length;
                    }
                    
                    xEnd = x0;
                    yEnd = y0;

                    
                    // if value above max, store
                    if(value>valueMax){
                        valueMax = value;
                    
                        Line line = new Line(x,y,xEnd,yEnd,length);
                    
                   
                        result = line;
                        visited = onTheLine;
                   } 
                    
                    
                } // end if on outline     
                }// end if within accepted orientation
              }          
           } //end for all points on square's outline
       
           // delete points if chosen so that further search will not trace the same line again
           for (Object v:visited){
               Point p = (Point)v;
               contour.eventsArray[p.x][p.y].on = 0;
               
           }
          
       
      
        return result;
    }
    
    
    // stops if touching shape or pixel below threshold
    protected Vector longestLines(int x, int y, int range, int midlength, int nb_lines_avg, float[][][] accEvents){
        Vector lines = new Vector();
        
        //Line line = new Line();
           // compute x1s and y1s based on range
           // for all points in a square outline centered on x0,y0 with side size range+1/2
           // find length of line
           // if above max, x,y dest = x1,y1 and max = length, touchingdoor= true/false accordingly
           float threshold = line_threshold;
           
           int x1 = 0;
           int y1 = 0;
           int x0 = 0;
           int y0 = 0;
           int xEnd = 0;
           int yEnd = 0;
           int lengthMax = 0;
           int xMid = 0;
           int yMid = 0;
           boolean touchesDoor = false;
           // for all points in a square outline centered on x0,y0 with side size range+1/2
           for(int i=x-range;i<x+range+1;i++){
              for(int j=y-range;j<y+range+1;j++){
                if(((i<=x-range)||(i>=x+range))||((j<=y-range)||(j>=y+range))){
                    // on the square outline
                    x1 = i; 
                    y1 = j;
                    x0 = x;
                    y0 = y;                                      
                    int midx = 0;
                    int midy = 0;
                    boolean touching = false;
                    
                    int dy = y1 - y0;
                    int dx = x1 - x0;
                    int stepx, stepy;
                    int length = 0;
                    
                    if (dy < 0) { dy = -dy;  stepy = -1; } else { stepy = 1; }
                    if (dx < 0) { dx = -dx;  stepx = -1; } else { stepx = 1; }
                    dy <<= 1;                                                  // dy is now 2*dy
                    dx <<= 1;                                                  // dx is now 2*dx

        
                    if (dx > dy) {
                        int fraction = dy - (dx >> 1);                         // same as 2*dy - dx
                        while (x0 != x1) {
                            if (fraction >= 0) {
                                y0 += stepy;
                                fraction -= dx;                                // same as fraction -= 2*dx
                            }
                            x0 += stepx;
                            fraction += dy;                                    // same as fraction -= 2*dy
                            if(x0>0&&x0<retinaSize&&y0>0&&y0<retinaSize){
                                if(accEvents[y0][x0][0]>threshold){
                                    
                                    if((length+1<3)||((contour.eventsArray[x0][y0].label!=1)
                                      ||(contour.eventsArray[x0][y0].on!=1))){//and check if within shape only after a few timestep
                                    length++;
                                    if(length==midlength){
                                        midx = x0;
                                        midy = y0;
                                        
                                    }
                                    // touching door?
                                    if(nearDoor(y0,x0,2)){
                                        touching = true;
                                        //break;
                                    }
                                    } else {
                                        break;
                                    }
                                    
                                } else {
                                    break;
                                }
                            } else {
                                break;
                            }

                            
                        }
                    } else {
                        int fraction = dx - (dy >> 1);
                        while (y0 != y1) {
                            if (fraction >= 0) {
                                x0 += stepx;
                                fraction -= dy;
                            }
                            y0 += stepy;
                            fraction += dx;
                            if(x0>0&&x0<retinaSize&&y0>0&&y0<retinaSize){
                                if(accEvents[y0][x0][0]>threshold){
                                    // 3 : here is the area around the start point in which we dont check fo shape border, in case
                                    // of border noise // should parametrize
                                 if((length+1<3)||((contour.eventsArray[x0][y0].label!=1)
                                      ||(contour.eventsArray[x0][y0].on!=1))){//and check if within shape only after a few timestep
                              
                                    length++;
                                    if(length==midlength){
                                        midx = x0;
                                        midy = y0;
                                    }
                                    // touching door?
                                    if(nearDoor(y0,x0,2)){
                                        touching = true;
                                       // break;
                                    }
                                    } else {
                                        break;
                                    }
                                } else {
                                    break;
                                }
                            } else {
                                break;
                                
                            }
                        }
                    }
                   // end computing line, end point in x0,y0
                    
                    // memorize max length
                     if(length>lengthMax){
                        lengthMax=length;
                    }
                    
                    xEnd = x0;
                    yEnd = y0;

                    xMid = midx;
                    yMid = midy;
                    touchesDoor = touching;

                    //
                    // store all lines
                    Line line = new Line(x,y,xEnd,yEnd,length);
                    line.touchesDoor = touchesDoor;
                    line.midx = xMid;
                    line.midy = yMid;
                    lines.add(line);
                 //  } 
                    
                    
                } // end if on outline                                   
              }          
           } //end for all points on square's outline
               
           
           if(lengthMax>0){
               // got some lines, select the N longest
                if(lines.size()>nb_lines_avg){
                    Collections.sort(lines, new LineLengthComparer()); 
                    for(int k=lines.size()-1; k>=lines_n_avg; k--){
                        lines.remove(k);
                    }
                }
               
               
//               Line line = new Line(x,y,xEnd,yEnd,lengthMax);
//               line.touchesDoor = touchesDoor;
//               line.midx = xMid;
//               line.midy = yMid;
//               
//               lines.add(line);
//              
               // compute mid length points
               //int[] res = computeMidShift(line);
               //line.midx = res[0];
               //line.midy = res[1];
               
           }
           
          
       
      
        return lines;
    }
    
    
    protected Line meanLineOf( Vector lines ){
        Line result = null;
    
        // return mean line from lines with same start point
        float avgOrientation = 0;
        float avgLength = 0;
        // for all lines find mean orientation
        Line line = null;
        for( Object o:lines ){
            line = (Line)o;
            avgOrientation += lineOrientation(line);
            avgLength += line.length;
        }
        avgOrientation = avgOrientation/lines.size();
        avgLength = avgLength/lines.size();
        
        if(avgLength>finger_length){
            avgLength=finger_length;
        }
        // then create end point at mean length or < if specified (?)
        if(line!=null){
        
            int xb = line.x0 + (int)(Math.cos(Math.toRadians(avgOrientation))*avgLength);
            int yb = line.y0 + (int)(Math.sin(Math.toRadians(avgOrientation))*avgLength);
        
        
            result = new Line(line.x0,line.y0,xb,yb,(int)avgLength);
            result.orientation = avgOrientation;
        }
        
        return result;
        
    }
    
    protected float lineOrientation( Line line ){
         double dx = (double)(line.x1-line.x0);
         double dy = (double)(line.y1-line.y0);                                                  
         double size = Math.sqrt((dy*dy)+(dx*dx));                                                     
         double orientation = Math.toDegrees(Math.acos(dx/size));
         
         if (line.y0>line.y1){
           orientation = 180-orientation;
          }
         return (float)orientation;
    }
    
    protected float lineDirection( Line line ){
         double dx = (double)(line.x1-line.x0);
         double dy = (double)(line.y1-line.y0);                                                  
         double size = Math.sqrt((dy*dy)+(dx*dx));                                                     
         double orientation = Math.toDegrees(Math.acos(dx/size));
         
         if (line.y0>line.y1){
           orientation = 360-orientation;
          }
         return (float)orientation;
    }
    
    
    
    // shift point to middle of finger width
    protected int[] computeMidShift( Line line ){
        int[] res = new int[2];
        int maxRange = fing_maxRange; // to parametrize in gui
        int minRange = fing_minRange; // to parametrize in gui
        // compute orientation of line
          //double dx = (double)(line.x1-line.x0);
         // double dy = (double)(line.y1-line.y0);                                                  
         // double size = Math.sqrt((dy*dy)+(dx*dx));                                                     
          double orientation = lineOrientation(line); //Math.toDegrees(Math.acos(dx/size));
        
          int xa = line.midx + (int)(Math.cos(Math.toRadians(orientation+90))*maxRange); // +0.5 for rounding, maybe not
          int ya = line.midy + (int)(Math.sin(Math.toRadians(orientation+90))*maxRange); // +0.5 for rounding, maybe not
          int xb = line.midx + (int)(Math.cos(Math.toRadians(orientation-90))*maxRange); // +0.5 for rounding, maybe not
          int yb = line.midy + (int)(Math.sin(Math.toRadians(orientation-90))*maxRange); // +0.5 for rounding, maybe not

          // now trace segment lines until shape or max
          int lengthA = traceLine(line.midx,line.midy,xa,ya,maxRange);
          int lengthB = traceLine(line.midx,line.midy,xb,yb,maxRange);
        
          if((lengthA<minRange)&&(lengthB<minRange)){
              res[0] = (int)(((float)xa+xb)/2);
              res[1] = (int)(((float)ya+yb)/2);
              
          } else if(lengthA<minRange){
              res[0] = xa + (int)(Math.cos(Math.toRadians(orientation-90))*minRange);
              res[1] = ya + (int)(Math.cos(Math.toRadians(orientation-90))*minRange);
          
          } else if(lengthB<minRange){
              res[0] = xb + (int)(Math.cos(Math.toRadians(orientation+90))*minRange);
              res[1] = yb + (int)(Math.cos(Math.toRadians(orientation+90))*minRange);
          } else {
              // at middle of max range
              res[0] = (int)(((float)xa+xb)/2);
              res[1] = (int)(((float)ya+yb)/2);
          }
          
         
          
        return res;
    }
       
    protected int traceLine(int x0, int y0, int x1, int y1, int maxrange){
             
        int length = 0;
       
        int x = x0;
        int y = y0;
        
        int dy = y1 - y0;
        int dx = x1 - x0;
        int stepx, stepy;

        if (dy < 0) { dy = -dy;  stepy = -1; } else { stepy = 1; }
        if (dx < 0) { dx = -dx;  stepx = -1; } else { stepx = 1; }
        dy <<= 1;                                                  // dy is now 2*dy
        dx <<= 1;                                                  // dx is now 2*dx   
       
        if (dx > dy) {
            int fraction = dy - (dx >> 1);                         // same as 2*dy - dx
            while (x0 != x1) {
                if (fraction >= 0) {
                    y0 += stepy;
                    fraction -= dx;                                // same as fraction -= 2*dx
                }
                x0 += stepx;
                fraction += dy;                                    // same as fraction -= 2*dy
                
                if((x0>0&&x0<retinaSize)&&(y0>0&&y0<retinaSize)){
                    if (contour.eventsArray[x0][y0].on==1){
                        if (contour.eventsArray[x0][y0].label==1){
                            // point on shape
                            // stop
                            break;
                        }
                    }
                    
                    length++;
                    if(length>maxrange){
                        break;
                    }
                } else {
                    break;                    
                }
                
            }
        } else {
            int fraction = dx - (dy >> 1);
            while (y0 != y1) {
                if (fraction >= 0) {
                    x0 += stepx;
                    fraction -= dy;
                }
                y0 += stepy;
                fraction += dx;
                
                if((x0>0&&x0<retinaSize)&&(y0>0&&y0<retinaSize)){
                    if (contour.eventsArray[x0][y0].on==1){
                        if (contour.eventsArray[x0][y0].label==1){
                            // point on shape
                            // stop
                            break;
                        }
                    }
                    
                    length++;
                    if(length>maxrange){
                        break;
                    }
                } else {
                    break;                    
                }
                
                
        
            }
        }
        
       
        return length;
         
    }
    
    protected float meanValueOfLine(int x0, int y0, int x1, int y1, float[][][] accEvents){
        
        float meanValue = 0;
        int nbValues = 0;
        
        int x = x0;
        int y = y0;
        
        int dy = y1 - y0;
        int dx = x1 - x0;
        int stepx, stepy;

        if (dy < 0) { dy = -dy;  stepy = -1; } else { stepy = 1; }
        if (dx < 0) { dx = -dx;  stepx = -1; } else { stepx = 1; }
        dy <<= 1;                                                  // dy is now 2*dy
        dx <<= 1;                                                  // dx is now 2*dx

        try {
        
        meanValue += accEvents[y0][x0][0];        
        nbValues++;
        
       
        if (dx > dy) {
            int fraction = dy - (dx >> 1);                         // same as 2*dy - dx
            while (x0 != x1) {
                if (fraction >= 0) {
                    y0 += stepy;
                    fraction -= dx;                                // same as fraction -= 2*dx
                }
                x0 += stepx;
                fraction += dy;                                    // same as fraction -= 2*dy
                meanValue += accEvents[y0][x0][0];
                nbValues++;
                
                
            }
        } else {
            int fraction = dx - (dy >> 1);
            while (y0 != y1) {
                if (fraction >= 0) {
                    x0 += stepx;
                    fraction -= dy;
                }
                y0 += stepy;
                fraction += dx;
                
                meanValue += accEvents[y0][x0][0];
                nbValues++;
                
                
        
            }
        }
        
        } catch (Exception e ){
            //System.out.println("meanValueOfLine error x0="+x0+" y0="+y0+" x1="+x1+" y1="+y1+" x="+x+" y="+y);
            //e.printStackTrace();
            
        }
        return meanValue/nbValues;
    }
    
    
     // rectangle defined by two sides: x0,y0 to x1,y1 and x0,y0 to x2,y2
    protected void increaseIntensityOfRectangle(int x0, int y0, int x1, int y1, int x2, int y2){
        // initial values
        int x = x0;
        int y = y0;
        
       
        int dy = y1 - y0;
        int dx = x1 - x0;
        int stepx, stepy;

        if (dy < 0) { dy = -dy;  stepy = -1; } else { stepy = 1; }
        if (dx < 0) { dx = -dx;  stepx = -1; } else { stepx = 1; }
        dy <<= 1;                                                  // dy is now 2*dy
        dx <<= 1;                                                  // dx is now 2*dx

        
        // create line here
        increaseIntentisyOfLine(x0,y0,x2,y2);
                
        if (dx > dy) {
            int fraction = dy - (dx >> 1);                         // same as 2*dy - dx
            while (x0 != x1) {
                if (fraction >= 0) {
                    y0 += stepy;
                    fraction -= dx;                                // same as fraction -= 2*dx
                }
                x0 += stepx;
                fraction += dy;                                    // same as fraction -= 2*dy
               
                increaseIntentisyOfLine(x0,y0,x2+x0-x,y2+y0-y);
            }
        } else {
            int fraction = dx - (dy >> 1);
            while (y0 != y1) {
                if (fraction >= 0) {
                    x0 += stepx;
                    fraction -= dy;
                }
                y0 += stepy;
                fraction += dx;
                increaseIntentisyOfLine(x0,y0,x2+x0-x,y2+y0-y);
            }
        }
    }
    
    // only looking at first value, should change it to look at float[][] because last [] is not used actually
    protected float findArrayMax( float[][][] array ){
        float max = 0;
        for(int i=0;i<array.length;i++){
            for(int j=0;j<array[i].length;j++){
                float f = array[i][j][0];
                if (f>max) max = f;
                
            }
        }
        return max;
    }
    
    
    // follow Bresenham algorithm to incrase intentisy along a discrete line between two points
     protected void increaseIntentisyOfLine(int x0, int y0, int x1, int y1){
        
        int x = x0;
        int y = y0;
         
        int dy = y1 - y0;
        int dx = x1 - x0;
        int stepx, stepy;

        if (dy < 0) { dy = -dy;  stepy = -1; } else { stepy = 1; }
        if (dx < 0) { dx = -dx;  stepx = -1; } else { stepx = 1; }
        dy <<= 1;                                                  // dy is now 2*dy
        dx <<= 1;                                                  // dx is now 2*dx

        try {
            
            
            // only increase r channel
            insideIntensities[x0][y0][0] = insideIntensities[x0][y0][0]+intensityIncrease;
            
            if (dx > dy) {
                int fraction = dy - (dx >> 1);                         // same as 2*dy - dx
                while (x0 != x1) {
                    if (fraction >= 0) {
                        y0 += stepy;
                        fraction -= dx;                                // same as fraction -= 2*dx
                    }
                    x0 += stepx;
                    fraction += dy;                                    // same as fraction -= 2*dy
                    insideIntensities[x0][y0][0] = insideIntensities[x0][y0][0]+intensityIncrease;
                }
            } else {
                int fraction = dx - (dy >> 1);
                while (y0 != y1) {
                    if (fraction >= 0) {
                        x0 += stepx;
                        fraction -= dy;
                    }
                    y0 += stepy;
                    fraction += dx;
                    insideIntensities[x0][y0][0] = insideIntensities[x0][y0][0]+intensityIncrease;
                }
            }
        } catch (Exception e ){
            //System.out.println("increaseIntentisyOfLine error x0="+x0+" y0="+y0+" x1="+x1+" y1="+y1+" x="+x+" y="+y);
            //e.printStackTrace();
            
        }
        
    }
     
     void resetDensities( int density ){
        densities = new float[density];
        for (int k=0;k<density;k++){
            densities[k] = (float)k/density;
        }
     }
 
    /**
     * filterFrame : low pass filter, apply some kind of gaussian noise on pixel of frame
     */
    float[][][] lowFilterFrame( float[][][] frame, int radius, int density ){
        float[][][] res = new float[retinaSize][retinaSize][frame[0][0].length];
        //int radius = lowFilter_radius;
        float fradius = (float)radius;
       // int density = lowFilter_density; // >0
        float invDensity = 1/(float)density;
        int radiusSq = radius*radius;
        float dist = 0;
        float f = 0;
        float dr = 0;
        int cat = 0;
        float n = 0;
        float val = 0;
        // for all points of frame
        // for all points in square around 
        // if point within circle
        //     add value by distance
        //     number++
        // end if
        // end for
        // average on number
        // add to res
        // end for
        int j=0;
        int is=0;
        int js=0;
        
        for (int i=0; i<frame.length;i++){
          for (j=0; j<frame.length;j++){ //square matrix
              if(frame[i][j][0]>=lowFilter_threshold){
                  // square inside
                  n = 0;
                  val = 0;
                  for (is=i-radius; is<i+radius+1;is++){
                      if(is>=0&&is<frame.length){
                          for (js=j-radius; js<j+radius+1;js++){
                              if(js>=0&&js<frame.length){
                                  // if within circle
                                  dist = ((is-i)*(is-i)) + ((js-j)*(js-j));

                                  if(dist<radiusSq){
                                      f = 1;
                                      dr = (float)Math.sqrt(dist)/fradius;
                                      if (dr!=0) f = 1/dr;
                                      
                                      val += frame[is][js][0] * f;
                                      n+=f;

                                  }
                              }
                          }
                      }
                  }
                  // avg point
                  val = val/n;
                  cat = (int)(val / invDensity);
                 //System.out.println("cat:"+cat+" val:"+val+" densities["+cat+"]:"+densities[cat]);
                  if(cat<0){
                      res[i][j][0] = 0;
                  } else if(cat>=density){
                      res[i][j][0] = 1;
                  } else {
                      res[i][j][0] = densities[cat];
                      //res[j][i][0] = densities[cat]; //inverted for insideIntensities
                  }
              }
          }  
        }
        
        
        return res;
    }
  
    
    
    
    void checkInsideIntensityFrame(){
        if(showWindow && insideIntensityFrame==null) createInsideIntensityFrame();
    }
     
    JFrame insideIntensityFrame=null;
    GLCanvas insideIntensityCanvas=null;
    GLU glu=null;
//    GLUT glut=null;
    void createInsideIntensityFrame(){
        insideIntensityFrame=new JFrame("Inside intensity");
        insideIntensityFrame.setPreferredSize(new Dimension(retinaSize*intensityZoom,retinaSize*intensityZoom));
        insideIntensityCanvas=new GLCanvas();
        insideIntensityCanvas.addGLEventListener(new GLEventListener(){
            public void init(GLAutoDrawable drawable) {
            }
            
            synchronized public void display(GLAutoDrawable drawable) {
                if(segments==null) return;
                GL gl=drawable.getGL();
                gl.glLoadIdentity();
                //gl.glScalef(drawable.getWidth()/2000,drawable.getHeight()/180,1);//dist to gc, orientation?
                gl.glClearColor(0,0,0,0);
                gl.glClear(GL.GL_COLOR_BUFFER_BIT);
                int font = GLUT.BITMAP_HELVETICA_12;
                //System.out.println("max point "+maxDistGC*100+" "+maxOrientation*100);
                
//                gl.glColor3f(1,0,0);
//                gl.glRectf(10,90,11,91);
//                gl.glColor3f(0,1,0);
//                gl.glRectf(20,90,21,91);
//                 gl.glColor3f(0,0,1);
//                gl.glRectf(10,45,11,46);
//              
                /*
               int size = 2;
                
               for (int k=0;k<10;k++){
                   gl.glColor3f(1,0,0);
                   gl.glRecti(90,k,90+size,k+size);
                   gl.glColor3f(0,0,1);
                   gl.glRecti(k,0,k+size,0+size);
               }
                 */
               /*
                gl.glColor3f(1,0,0);
                gl.glRecti(90,15,90+size,15+size);
                gl.glColor3f(0,1,0);
                gl.glRecti(45,3,45+size,3+size);
                gl.glColor3f(0,0,1);
                gl.glRecti(45,15,45+size,15+size);
                gl.glColor3f(0,1,1);
                gl.glRecti(45,35,45+size,35+size);
                */
                
                // display inside intensity
                
                
               if(showScore){          
                float max = findArrayMax(scoresFrame);
                for(int i=0;i<scoresFrame.length;i++){
                    for(int j=0;j<scoresFrame[i].length;j++){  
                        float f = 0f;
                        
                        f = scoresFrame[i][j][0]/max;
                       
                        
                        // f is scaled between 0-1
                        gl.glColor3f(f,0,f);
                        gl.glRectf(i*intensityZoom,j*intensityZoom,(i+1)*intensityZoom,(j+1)*intensityZoom);
                        
                        
                    }
                }
                
               } else if(showAcc){
                    
                    float max = findArrayMax(insideIntensities);
                     for (int i = 0; i<accEvents.length; i++){
                        for (int j = 0; j<accEvents[i].length; j++){
                            float f;
                             
                            f = filteredEvents[i][j][0];
                            if(showDensity){
                                float minVal = 0;
                                float maxVal = 0;
                                
                                
                                if(densityMinIndex>=lowFilter_density){
                                    minVal = 1;
                                    
                                } else {
                                    minVal =  densities[densityMinIndex];
                                }
                                if(densityMaxIndex>=lowFilter_density){
                                    maxVal = 1;
                                    
                                } else {
                                    maxVal =  densities[densityMaxIndex];
                                }
                                if(f<minVal||f>maxVal){
                                    f=0;
                                    
                                    
                                }
                            }
                                       
                             
                            float g = 0; //insideIntensities[j][i][0]/max;
                            //if(g<0)g=0;
                            gl.glColor3f(f,f+g,f);
                            gl.glRectf(j*intensityZoom,i*intensityZoom,(j+1)*intensityZoom,(i+1)*intensityZoom);
                            
                        }
                     }                                        
                    
                    
               } else {
                 /*   
                float max = 1;
                if(scaleIntensity) max = findArrayMax(insideIntensities);
                for(int i=0;i<insideIntensities.length;i++){
                    for(int j=0;j<insideIntensities[i].length;j++){  
                        float f = 0f;
                        if(scaleIntensity){
                            f = insideIntensities[i][j][0]/max;
                        } else {
                            f = insideIntensities[i][j][0];
                            if (f<in_threshold){
                                f = 0;
                            }
                        }
                        // f is scaled between 0-1
                        gl.glColor3f(f,f,f);
                        gl.glRectf(i*intensityZoom,j*intensityZoom,(i+1)*intensityZoom,(j+1)*intensityZoom);
                        
                    }
                }
                */
               }
               if(showZones){
                // draw door
                  gl.glColor3f(0,1,0);
                drawBox(gl,door_ya*intensityZoom,door_yb*intensityZoom,door_xa*intensityZoom,door_xb*intensityZoom);
               } 
                
                
               if(showThin){
                    for(int i=0;i<thinned.length;i++){
                        for(int j=0;j<thinned[i].length;j++){  
                            float f = (float)thinned[i][j][0];
                       
                            if(f>0){
                                gl.glColor3f(0,0,f);
                                gl.glRectf(j*intensityZoom,i*intensityZoom,(j+1)*intensityZoom,(i+1)*intensityZoom);
                            }
                        }
                    }
               }  
                
               if(showPalm){
                    // replace nbFingerAttached by something else
                      gl.glColor3f(1,0,1);
                      
                      for(int i=0;i<MAX_NB_FINGERS;i++){
                        // trace all fingers
                        FingerCluster f = palm.fingers[i];
                        if(f!=null){
                            if(f.length_tip!=0){
                                
                                gl.glRectf(f.tip_x_start[0]*intensityZoom,f.tip_y_start[0]*intensityZoom,(f.tip_x_start[0]+1)*intensityZoom,(f.tip_y_start[0]+1)*intensityZoom);
                                
                                
                                gl.glBegin(GL.GL_LINES);
                                {
                                    gl.glVertex2i(f.tip_x_start[0]*intensityZoom,f.tip_y_start[0]*intensityZoom);
                                    gl.glVertex2i(f.tip_x_end[0]*intensityZoom,f.tip_y_end[0]*intensityZoom);
                                }
                                gl.glEnd();
                                
                                gl.glRasterPos3f((f.tip_x_start[0]+3)*intensityZoom,f.tip_y_start[0]*intensityZoom,0);
                                chip.getCanvas().getGlut().glutBitmapString(font, String.format("%.0f", f.direction_tip ));
                                
                            }
                            if(f.length_base!=0){
                                gl.glBegin(GL.GL_LINES);
                                {
                                    gl.glVertex2i(f.base_x_start[0]*intensityZoom,f.base_y_start[0]*intensityZoom);
                                    gl.glVertex2i(f.base_x_end[0]*intensityZoom,f.base_y_end[0]*intensityZoom);
                                    
                                }
                                gl.glEnd();
                                
                                gl.glRasterPos3f((f.base_x_start[0]+3)*intensityZoom,f.base_y_start[0]*intensityZoom,0);
                                chip.getCanvas().getGlut().glutBitmapString(font, String.format("%.0f", f.direction_base ));
                            }
                        }
                        
                    }
                    
               }
                
               if(showSkeletton){
                    gl.glColor3f(0,0,1);
                    for(int i=0;i<nbBones;i++){
                        Segment o = bones[i];
                        
                        //  draw line here and
                        gl.glBegin(GL.GL_LINES);
                        {
                            
                            
                            gl.glVertex2i(o.x1*intensityZoom,o.y1*intensityZoom);
                            gl.glVertex2i(o.x2*intensityZoom,o.y2*intensityZoom);
                            
                            
                        }
                        gl.glEnd();                       
                    }
                    
                    for(Object o:nodes){
                        Node n = (Node)o;
                         if(n.type==0) gl.glColor3f(1,1,0);
                         if(n.type==1) gl.glColor3f(0,1,1);
                         gl.glRectf(n.x*intensityZoom,n.y*intensityZoom,(n.x+1)*intensityZoom,(n.y+1)*intensityZoom);
                           
                         gl.glRasterPos3f((n.x+3)*intensityZoom,n.y*intensityZoom,0);
                            
                         chip.getCanvas().getGlut().glutBitmapString(font, String.format("%d", n.support ));
                    }
                    
                    
               } 
                
               if(grasp_started ){
                 gl.glColor3f(0,1,0);
                 for(int i=0;i<nbSegments;i++){
                     Segment o = segments[i];
                     int midx = o.midx();
                     int midy = o.midy();
                     //  draw line here and
                     gl.glBegin(GL.GL_LINES);
                     {
                         
                         
                         gl.glVertex2i(o.x1*intensityZoom,o.y1*intensityZoom);
                         gl.glVertex2i(o.x2*intensityZoom,o.y2*intensityZoom);
                         /*
                         gl.glColor3f(1,0,0);
                         gl.glVertex2i(midx*intensityZoom,midy*intensityZoom);
                         gl.glVertex2i(o.xi*intensityZoom,o.yi*intensityZoom);
                         gl.glColor3f(0,0,1);
                         gl.glVertex2i(midx*intensityZoom,midy*intensityZoom);
                         gl.glVertex2i(o.xo*intensityZoom,o.yo*intensityZoom);
                          **/
                                           
                     }
                     gl.glEnd();
                     
                     /*
                     // draw text
                     int font = GLUT.BITMAP_HELVETICA_12;
                     gl.glColor3f(1,1,1);
                     gl.glRasterPos3f(midx*intensityZoom,midy*intensityZoom,0);
                     // annotate
                     
                     chip.getCanvas().getGlut().glutBitmapString(font, String.format("%.0f", (float)o.orientation ));
                     */
                     
                 }
               }

                
                if(showFingers){
                    // draw fingers here
                   gl.glColor3f(1,0,0);
                    //gl.glBlendFunc(GL.GL_ONE, GL.GL_ONE);
                    
                    for(int i=0; i<fingerLines.size(); i++){
                        try {
                        Line line = (Line)fingerLines.elementAt(i);
                        gl.glBegin(GL.GL_LINES);
                        {
                            if(line.midx==0&&line.midy==0){
                                gl.glVertex2i(line.x0*intensityZoom,line.y0*intensityZoom);
                                gl.glVertex2f(line.x1*intensityZoom,line.y1*intensityZoom);
                            } else {
                                gl.glVertex2i(line.x0*intensityZoom,line.y0*intensityZoom);
                                gl.glVertex2f(line.midx*intensityZoom,line.midy*intensityZoom);
                                gl.glVertex2i(line.midx*intensityZoom,line.midy*intensityZoom);
                                gl.glVertex2f(line.x1*intensityZoom,line.y1*intensityZoom);
                            }
                        }
                        gl.glEnd();
                        } catch (ArrayIndexOutOfBoundsException e){
                            // it's ok do nothing, problem of thred sync maybe but no consequences
                        }
                    }  
                    
                   
                     
                   
                    //gl.glBlendFunc(GL.GL_ONE, GL.GL_ZERO);
                }   // end if show fingers  
                if(showFingerTips){
                    if(grasp_started){
                        
                        gl.glColor3f(1,0,0);
                        for(int i=0;i<fingerTips.size();i++){
                            Point sc = (Point)fingerTips.elementAt(i);
                            gl.glRectf(sc.x*intensityZoom,sc.y*intensityZoom,(sc.x+1)*intensityZoom,(sc.y+1)*intensityZoom);
                            
                            
                            
                            gl.glRasterPos3f((sc.x+3)*intensityZoom,sc.y*intensityZoom,0);
                            
                            chip.getCanvas().getGlut().glutBitmapString(font, String.format("%d", sc.score ));
                        }
                    }
                }
                
                /**
                if(showClusters){
                     gl.glColor3f(0,0,1);
                         for(int i=0;i<fingerTipClusters.length;i++){
                             FingerCluster fc = fingerTipClusters[i];
                             gl.glRectf(fc.x*intensityZoom,(fc.y+1)*intensityZoom,(fc.x+1)*intensityZoom,(fc.y+2)*intensityZoom);
                             
                             
                             // finger lines
                             if(fc.finger_base_x!=0&&fc.finger_base_y!=0){
                                 gl.glBegin(GL.GL_LINES);
                                 {
                                     //gl.glColor3f(0,0,1);
                                     gl.glVertex2i(fc.x*intensityZoom,fc.y*intensityZoom);
                                     gl.glVertex2f(fc.finger_base_x*intensityZoom,fc.finger_base_y*intensityZoom);
                                     
                                 }
                                 gl.glEnd();
                             }
                             
                             // knuckles polygon
                             if(knuckle_polygon_created){
                                 gl.glBegin(GL.GL_LINES);
                                 {
                                     //gl.glColor3f(0,0,1);
                                    if(fc.neighbour1!=null){
                                     gl.glVertex2i(fc.finger_base_x*intensityZoom,fc.finger_base_y*intensityZoom);
                                     gl.glVertex2f(fc.neighbour1.finger_base_x*intensityZoom,fc.neighbour1.finger_base_y*intensityZoom);
                                    }
                                 }
                                 gl.glEnd();
                                 gl.glBegin(GL.GL_LINES);
                                 {
                                     //gl.glColor3f(0,0,1);
                                     if(fc.neighbour2!=null){
                                     gl.glVertex2i(fc.finger_base_x*intensityZoom,fc.finger_base_y*intensityZoom);
                                     gl.glVertex2f(fc.neighbour2.finger_base_x*intensityZoom,fc.neighbour2.finger_base_y*intensityZoom);
                                     }
                                 }
                                 gl.glEnd();
                                 
                                 
                                 
                             }
                         }      
                }
                */
                
                //                gl.glColor3f(1,0,0);
//                for(int i=0;i<fingerTips.size();i++){
//                    Point sc = (Point)fingerTips.elementAt(i);
//                    gl.glRectf(sc.x*intensityZoom,sc.y*intensityZoom,(sc.x+1)*intensityZoom,(sc.y+1)*intensityZoom);
////                    int range=70;
////                    for(int k=sc.x-range;k<sc.x+range+1;k++){
////                      for(int j=sc.y-range;j<sc.y+range+1;j++){
////                        if(((k<=sc.x-range)||(k>=sc.x+range))||((j<=sc.y-range)||(j>=sc.y+range))){
////                        gl.glRectf(k*intensityZoom,j*intensityZoom,(k+1)*intensityZoom,(j+1)*intensityZoom);
////                    
////                    
////                        }
////                      }
////                    }
//                    
//                }  

               /*
                for(int i=0;i<MAX_ORIENTATIONS;i++){
                    for(int j=0;j<MAX_DISTGC;j++){
                        //
                        int acc = segments[i][j];
                        float f = (float)acc/(float)accOrientationMax;
                        //int size = new Float(((float)acc/(float)accOrientationMax)*10).intValue();
                        //float f=acc;///accOrientationMax;
                        gl.glColor3f(0,f,f);
                        //gl.glRecti(j,i,j+1,i+1);
                        gl.glRecti(j,i,j+1,i+1);
//                        gl.glRectf(new Double(o.distGC*10).floatValue(),new Double(o.orientation).floatValue(),
//                                new Double(o.distGC*10+1).floatValue(),new Double(o.orientation+1).floatValue());
//                        
                         
                        
                        //if (acc>0)System.out.println("add point ("+i+","+j+") acc: "+acc+" accmax: "+accOrientationMax+" size: "+size);
                    }                   
                }
                 */
                
                //gl.glPointSize(6);
                //gl.glColor3f(1,0,0);
                //gl.glBegin(GL.GL_POINTS);
                //gl.glVertex2f(thetaMaxIndex, rhoMaxIndex);
                //gl.glEnd();
//                if(glut==null) glut=new GLUT();
                int error=gl.glGetError();
                if(error!=GL.GL_NO_ERROR){
                    if(glu==null) glu=new GLU();
                    log.warning("GL error number "+error+" "+glu.gluErrorString(error));
                }
            }
            
            synchronized public void reshape(GLAutoDrawable drawable, int x, int y, int width, int height) {
                GL gl=drawable.getGL();
                final int B=10;
                gl.glMatrixMode(GL.GL_PROJECTION);
                gl.glLoadIdentity(); // very important to load identity matrix here so this works after first resize!!!
                gl.glOrtho(-B,drawable.getWidth()+B,-B,drawable.getHeight()+B,10000,-10000);
                gl.glMatrixMode(GL.GL_MODELVIEW);
                gl.glViewport(0,0,width,height);
            }
            
            public void displayChanged(GLAutoDrawable drawable, boolean modeChanged, boolean deviceChanged) {
            }
        });
        insideIntensityFrame.getContentPane().add(insideIntensityCanvas);
        insideIntensityFrame.pack();
        insideIntensityFrame.setVisible(true);
    }
    
    private boolean tooClose(int x1, int y1, int x2, int y2, int range){
         if((Math.abs(x2-x1)<range)&&(Math.abs(y2-y1)<range)){
                return true; 
        }
        return false;
    }
    
    protected boolean nearDoor(int x, int y, int range){
        
        if((y>door_ya-range)&&(y<door_yb+range)){
            if((door_xa-x)<range&&(x-door_xb)<range){
                return true;
                
            }
            
        }
        return false;
        
    }
    
    protected boolean insideDoor(int x, int y){
        
        if((y>door_ya)&&(y<door_yb)){
            if((x>door_xa)&&(x<door_xb)){
                return true;
                
            }
            
        }
        return false;
        
    }
    
    // class Node
    public class Node{
        public int x;
        public int y;
        public int type; // change to enum?
        public int support=0;
        // + connectivity?
        
        public Node(){}
        
        public Node( int x, int y, int type){
            this.x = x;
            this.y = y;
            this.type = type;
            
        }
        
        public Node( ContourPoint cp, int type){
            if(cp!=null){
                this.x = cp.x;
                this.y = cp.y;
            
                this.type = type;
            }
        }
          
      void measureSupport( Contour contour ){
            Contour image = new Contour(contour);
            int max_support = 20;
            support=0;
            int prev_support=-1;
            int cur_x = x;
            int cur_y = y;
            while(support<max_support&&support>prev_support){
              // check neighbours
              prev_support++;
              search:
              for(int i=cur_x-1;i<=cur_x+1;i++){
                  if(i>0&&i<image.maxX){
                      for(int j=cur_y-1;j<=cur_y+1;j++){
                            if(j>0&&j<image.maxY){
                                if(image.eventsArray[i][j].on==1){
                                    support++;
                                    image.eventsArray[i][j].on=0;
                                    cur_x = i;
                                    cur_y = j;
                                    break search;
                                }
                            }
                      }
                  } 
              } // search
            
            }    // end while      
        } // end measureSupport
  
    
    }
    
    // class ContourPoint
    public class ContourPoint{
        protected int maxX=retinaSize,maxY=retinaSize;
        public int label;
        public int x;
        public int y;
        public int on;
        public int used;
        
        public ContourPoint(){
            reset();
        }
        
        public ContourPoint( ContourPoint cp ){
            this.label = cp.label;
            this.x = cp.x;
            this.y = cp.y;
            this.on = cp.on;
            //this.used = cp.used; maybe unwanted
        }
        
        
        
        public void reset(){
            label = 0;
            on = 0;
            used = 0;
        }
        
    }
    // end class ContourPoint
    
    // class Contour
    public class Contour{
        ContourPoint eventsArray[][];
        protected int maxX=retinaSize,maxY=retinaSize;
        public int nbLabels=0;
        
        public Contour(){
            
            reset();
            
        }
        
        public Contour( Contour contour ){ // only if contour same size, but all contour share global size (ugly)
            eventsArray = new ContourPoint[maxX][maxY];
             
            for (int i=0;i<maxX;i++){
                for (int j=0;j<maxY;j++){
                    eventsArray[i][j] = new ContourPoint(contour.eventsArray[i][j]);
                    
                }
            }
           
        }
        
        public Contour( int[][][] img, int label){
            reset();
            // should check on img.length < maxX ...
            for (int i=0;i<img.length;i++){
                for (int j=0;j<img[i].length;j++){
                    if(img[i][j][0]==1){
                        // inverted x,y (? why?)
                        add(j,i,label);
                    }         
                }
            }    
        }
        
        
        public void reset(){
            // should optimize that
            eventsArray = new ContourPoint[maxX][maxY];
            
            
            
            for (int i=0;i<maxX;i++){
                for (int j=0;j<maxY;j++){
                    eventsArray[i][j] = new ContourPoint();
                    
                }
            }
            
        }
        
        
        public void add(int i, int j){
            eventsArray[i][j].on = 1;
            eventsArray[i][j].x = i;
            eventsArray[i][j].y = j;
            
        }
        
        public void add(int i, int j, int label){
            eventsArray[i][j].on = 1;
            eventsArray[i][j].x = i;
            eventsArray[i][j].y = j;
             eventsArray[i][j].label = label;
        }
        
        /**
         * Set label of points to 1 if group touching door zone
         */
        public void highlightTouchingDoor(){
            for (int i=0;i<maxX;i++){
                for (int j=0;j<maxY;j++){
                    if(eventsArray[i][j].on == 1){
                        // if touch door
                        if (nearDoor(eventsArray[i][j].y,eventsArray[i][j].x,linkSize)){
                            // set label to 1
                            changeLabel(eventsArray[i][j].label,1);
                            //eventsArray[i][j].label = 1;
                            //System.out.println("near door "+eventsArray[i][j].x+","+eventsArray[i][j].y);
                        } else {
                           // System.out.println("not near door "+eventsArray[i][j].x+","+eventsArray[i][j].y);
                        }
                        
                        if (hideInside){
                        // if disable points inside door
                         if (insideDoor(eventsArray[i][j].y,eventsArray[i][j].x)){
                            eventsArray[i][j].on = 0;
                         }
                        }
                        
                    }
                }
            }
            
        }
        
      
        
        /** link, group contour points into linked groups by
         * assigning group label to each point
         *
         */
        public void link(){
            // for all points
            //    System.out.println("################### Link");
            int n = 1;
            for (int i=0;i<maxX;i++){
                for (int j=0;j<maxY;j++){
                    if(eventsArray[i][j].on == 1){
                        
                        // look for neihgbour
                        // System.out.println("findNeighbours("+i+","+j+","+linkSize+")");
                        if (eventsArray[i][j].label==0){
                            n++; //increase label
                            eventsArray[i][j].label=n;
                        }
                        findNeighbours(i,j,linkSize);
                        
                    }
                    
                }
            }
            nbLabels = n;
        }
        
       protected void extendSegmentFollowingPolicy( Segment segment, ContourPoint p , int min_x_policy, int max_x_policy, int min_y_policy, int max_y_policy, int curSegSize, boolean large ){         
           
           // look for neighbours in range allowed by policy
           ContourPoint neighbour = contour.getNeighbour(p.x,p.y,min_x_policy,max_x_policy,min_y_policy,max_y_policy,large);
                  
           // if none return             
           if(neighbour==null) return;
           if(curSegSize>=maxSegSize) return;
       
           // else 
           
           
           // add point to segment
           //neighbour.used = 1;
           p.used = 1; //thus do not set last point as used, it will be the start of another segment
           segment.x2 = neighbour.x;
           segment.y2 = neighbour.y;
           
           // recursive call
           
           extendSegmentFollowingPolicy(segment,neighbour,min_x_policy,max_x_policy,min_y_policy,max_y_policy,curSegSize+1,large);
           
          
           
       }
        
        
        
        // return one neighbour
       
       protected ContourPoint getNeighbour(int x, int y, int range ){
           return getNeighbour(x,y,range,range,range,range,false);
                 
       }
       
       protected ContourPoint getNeighbour(int x, int y, int range, boolean large ){
           return getNeighbour(x,y,range,range,range,range,large);
                 
       }
        
       protected ContourPoint getNeighbour(int x, int y, int low_xrange, int high_xrange, int low_yrange, int high_yrange){
            return getNeighbour(x,y,low_xrange,high_xrange,low_yrange,high_yrange,false);
       }
       protected ContourPoint getNeighbour(int x, int y, int low_xrange, int high_xrange, int low_yrange, int high_yrange, boolean large){
           int a = 0;
           if(large) a = 1;
           
           for (int i=x-low_xrange;i<x+high_xrange+a;i++){ //shortend range
                if (i>0&&i<maxX){                  
                    for (int j=y-low_yrange;j<y+high_yrange+a;j++){
                        if (j>0&&j<maxY){
                            if(i==x&&j==y){
                                
                            } else {
                                if(eventsArray[i][j].on == 1){
                                    if(eventsArray[i][j].used == 0){
                                        return eventsArray[i][j];
                                    }
                                }
                            }
                        }
                    }
                }
             }             
             return null;            
         }
       
       protected Vector getNeighbours(int x, int y, int range ){
           return getNeighbours(x,y,range,range,range,range);
           
           
       }
        protected Vector getNeighbours(int x, int y, int low_xrange, int high_xrange, int low_yrange, int high_yrange){
            Vector neighbours = new Vector();
            
            for (int i=x-low_xrange;i<=x+high_xrange;i++){
                if (i>0&&i<maxX){                  
                    for (int j=y-low_yrange;j<=y+high_yrange;j++){
                        if (j>0&&j<maxY){
                            if(i==x&&j==y){
                                
                            } else {
                                if(eventsArray[i][j].on == 1){
                                    if(eventsArray[i][j].used == 0){
                                        neighbours.add(eventsArray[i][j]);
                                    }
                                }
                            }
                        }
                    }
                }
             }             
             return neighbours;            
         }
                
        
        // find all neighbours and label points as neighbours
        protected void findNeighbours(int x, int y, int around){
            for (int i=x-around;i<=x+around;i++){
                if (i>0&&i<maxX){
                    
                    for (int j=y-around;j<=y+around;j++){
                        if (j>0&&j<maxY){
                            if(i==x&&j==y){
                                
                            } else {
                                if(eventsArray[i][j].on == 1){
                                    addAsNeighbours(eventsArray[x][y],eventsArray[i][j]);
                                }
                            }
                            
                        }
                        
                    }//end for j
                }
            }//end for i
        }
        
        public void addAsNeighbours( ContourPoint c1, ContourPoint c2 ){
            
            if (c1.label!=c2.label){
                
                if(c2.label==0){
                    c2.label = c1.label;
                } else {
                    int lc2 = c2.label;
                    // System.out.println("change all "+c2.label+" labels into "+c1.label);
                    changeLabel(lc2,c1.label);
//                       for (int i=0;i<maxX;i++){
//                            for (int j=0;j<maxY;j++){
//
//                                    if(eventsArray[i][j].label==lc2){
//                                         eventsArray[i][j].label = c1.label;
//                                    }
//
//                            }
//                       }
                    //c2.label = c1.label;
                }
                
                
            }
            
        }
        
        protected void changeLabel( int l2, int l1){
            for (int i=0;i<maxX;i++){
                for (int j=0;j<maxY;j++){
                    //if(i==x&&j==y){
                    //} else {
                    if(eventsArray[i][j].label==l2){
                        eventsArray[i][j].label = l1;
                    }
                    // }
                }
            }
        }
        
        public int getMaxX(){
            return maxX;
        }
        public int getMaxY(){
            return maxY;
        }
        
        
        
    }
    // end class Contour
    
    public Object getFilterState() {
        return null;
    }
    
    private boolean isGeneratingFilter() {
        return false;
    }
    
    synchronized public void resetFilter() {
        resetPawTracker();
    }
    
    public EventPacket filterPacket(EventPacket in) {
        if(in==null) return null;
        if(!filterEnabled) return in;
        if(enclosedFilter!=null) in=enclosedFilter.filterPacket(in);
        checkInsideIntensityFrame();
        track(in);
        if (showWindow) insideIntensityCanvas.repaint();
        return in;
    }
    
    
    
   
    
    public float getMixingFactor() {
        return mixingFactor;
    }
    
    public void setMixingFactor(float mixingFactor) {
        if(mixingFactor<0) mixingFactor=0; if(mixingFactor>1) mixingFactor=1f;
        this.mixingFactor = mixingFactor;
        prefs.putFloat("PawTracker2.mixingFactor",mixingFactor);
    }
    
    
    
    
    
//    /** @see #setSurround */
//    public float getSurround() {
//        return surround;
//    }
//    
//    /** sets scale factor of radius that events outside the cluster size can affect the size of the cluster if
//     * @param surround the scale factor, constrained >1 by setter. radius is multiplied by this to determine if event is within surround.
//     */
//    public void setSurround(float surround){
//        if(surround < 1) surround = 1;
//        this.surround = surround;
//        prefs.putFloat("PawTracker2.surround",surround);
//    }
    
  
    

    
//    public boolean isColorClustersDifferentlyEnabled() {
//        return colorClustersDifferentlyEnabled;
//    }
    
    
    public void update(Observable o, Object arg) {
        initFilter();
    }
    

    
    public void annotate(Graphics2D g) {
    }
    
    protected void drawBoxCentered(GL gl, int x, int y, int sx, int sy){
        gl.glBegin(GL.GL_LINE_LOOP);
        {
            gl.glVertex2i(x-sx,y-sy);
            gl.glVertex2i(x+sx,y-sy);
            gl.glVertex2i(x+sx,y+sy);
            gl.glVertex2i(x-sx,y+sy);
        }
        gl.glEnd();
    }
    
    protected void drawBox(GL gl, int x, int x2, int y, int y2){
        gl.glBegin(GL.GL_LINE_LOOP);
        {
            gl.glVertex2i(x,y);
            gl.glVertex2i(x2,y);
            gl.glVertex2i(x2,y2);
            gl.glVertex2i(x,y2);
        }
        gl.glEnd();
    }
    
    synchronized public void annotate(GLAutoDrawable drawable) {
        final float LINE_WIDTH=5f; // in pixels
        if(!isFilterEnabled()) return;
        
        
        GL gl=drawable.getGL(); // when we get this we are already set up with scale 1=1 pixel, at LL corner
        if(gl==null){
            log.warning("null GL in PawTracker2.annotate");
            return;
        }
        float[] rgb=new float[4];
        gl.glPushMatrix();
        try{
            
            
            //for all fingers
            // for(Cluster c:clusters){
            
            // get finger location
            //int x=(int)c.getLocation().x;
            // int y=(int)c.getLocation().y;
            
            
            // int sy=(int)c.radiusY; // sx sy are (half) size of rectangle
            // int sx=(int)c.radiusX;
            
            int sy=fingertip_size; // sx sy are (half) size of rectangle
            int sx=fingertip_size;
            
            // set color and line width of cluster annotation
            // to adapt
            // c.setColorAutomatically();
            // c.getColor().getRGBComponents(rgb);
            
            
            
            // paul: draw finger lines experimantal
//            if(showFingers){
//                // draw fingers here
//                
//
//               for(int i=0; i<fingerLines.size(); i++){
//                   Line line = (Line)fingerLines.elementAt(i);
//                   gl.glBegin(GL.GL_LINES);
//                   {
//                       if(line.midx==0&&line.midy==0){
//                        gl.glVertex2i(line.x0,line.y0);
//                        gl.glVertex2f(line.x1,line.y1);
//                       } else {
//                         gl.glVertex2i(line.x0,line.y0);
//                         gl.glVertex2f(line.midx,line.midy);
//                         gl.glVertex2i(line.midx,line.midy);
//                         gl.glVertex2f(line.x1,line.y1);
//                       }
//                   }
//                   gl.glEnd();
//               }
//                
//                
//            }
            
            if(!showShape){
                if(showShapePoints){
                    for (int i=0;i<contour.getMaxX();i++){
                        for (int j=0;j<contour.getMaxY();j++){
                            if (contour.eventsArray[i][j].on==1){
                                if (contour.eventsArray[i][j].label==1||showAll){
                                    
                                    //float colorPoint = ((float)(contour.nbLabels-contour.eventsArray[i][j].label-1))/contour.nbLabels;
                                    // System.out.println("colorPoint "+colorPoint+" "+contour.eventsArray[i][j].label);
                                    // if(colorPoint>1)colorPoint=1f;
                                    // draw point
                                    // gl.glColor3f(colorPoint,0,1-colorPoint);
                                    // if (contour.eventsArray[i][j].label==1){
                                    gl.glColor3f(0,1,0);
                                    // }
                                    
                                    gl.glBegin(GL.GL_LINE_LOOP);
                                    {
                                        gl.glVertex2i(i,j);
                                        gl.glVertex2i(i+1,j);
                                        gl.glVertex2i(i+1,j+1);
                                        gl.glVertex2i(i,j+1);
                                    }
                                    gl.glEnd();
                                }
                            }
                        }
                    }
                }
                
            } else { //show shape
               
                gl.glColor3f(0,1,0);
                for(int i=0;i<nbSegments;i++){
                    Segment o = segments[i];
                    //  draw line here and
                    gl.glBegin(GL.GL_LINES);
                    {
                        gl.glVertex2i(o.x1,o.y1);
                        gl.glVertex2i(o.x2,o.y2);
                    }
                    gl.glEnd();
                    
                }
                
            } // end showShape
//            
//            gl.glColor3f(1,0,0);
//            // draw gc
//            int i = gc.x;
//            int j = gc.y;
//            gl.glBegin(GL.GL_LINE_LOOP);
//            {
//                gl.glVertex2i(i,j);
//                gl.glVertex2i(i+1,j);
//                gl.glVertex2i(i+1,j+1);
//                gl.glVertex2i(i,j+1);
//            }
//            gl.glEnd();
//            
            
            // draw text avgEventRate
            // int font = GLUT.BITMAP_HELVETICA_12;
            // gl.glColor3f(1,0,0);
            // gl.glRasterPos3f(c.location.x,c.location.y,0);
            // annotate the cluster with the event rate computed as 1/(avg ISI) in keps
            //  float keps=1e3f / (c.getAvgISI()*AEConstants.TICK_DEFAULT_US);
            
            //  chip.getCanvas().getGlut().glutBitmapString(font, String.format("%.0f", c.prevNbEvents/keps ));
            
            if(showZones){
                // draw door
                gl.glColor3f(0,1,0);
                drawBox(gl,door_ya,door_yb,door_xa,door_xb);
            }
            
            
        }catch(java.util.ConcurrentModificationException e){
            // this is in case cluster list is modified by real time filter during rendering of clusters
            log.warning(e.getMessage());
        }
        gl.glPopMatrix();
    }
    
//    void drawGLCluster(int x1, int y1, int x2, int y2)
    
    /** annotate the rendered retina frame to show locations of clusters */
    synchronized public void annotate(float[][][] frame) {
        if(!isFilterEnabled()) return;
        // disable for now TODO
        if(chip.getCanvas().isOpenGLEnabled()) return; // done by open gl annotator
//            for(Cluster c:clusters){
//                if(c.isVisible()){
//                    drawCluster(c, frame);
//                }
//            }
    }
    
    
    
    
    public synchronized boolean isLogDataEnabled() {
        return logDataEnabled;
    }
    
    public synchronized void setLogDataEnabled(boolean logDataEnabled) {
        this.logDataEnabled = logDataEnabled;
        if(!logDataEnabled) {
            logStream.flush();
            logStream.close();
            logStream=null;
        }else{
            try{
                logStream=new PrintStream(new BufferedOutputStream(new FileOutputStream(new File("PawTrackerData.txt"))));
                logStream.println("# clusterNumber lasttimestamp x y avergeEventDistance");
            }catch(Exception e){
                e.printStackTrace();
            }
        }
    }
      
  
    public void setIn_threshold(float in_threshold) {
        this.in_threshold = in_threshold;
        prefs.putFloat("PawTracker2.in_threshold",in_threshold);
    }
    public float getIn_threshold() {
        return in_threshold;
    }
    
    public void setLine_threshold(float line_threshold) {
        this.line_threshold = line_threshold;
        prefs.putFloat("PawTracker2.line_threshold",line_threshold);
    }
    public float getLine_threshold() {
        return line_threshold;
    }
    
    public void setLine_range(int line_range) {
        this.line_range = line_range;
        prefs.putInt("PawTracker2.line_range",line_range);
    }
    public int getLine_range() {
        return line_range;
    }
    
    public void setLines_n_avg(int lines_n_avg) {
        this.lines_n_avg = lines_n_avg;
        prefs.putInt("PawTracker2.lines_n_avg",lines_n_avg);
    }
    public int getLines_n_avg() {
        return lines_n_avg;
    }
    
            
            
    public void getFing_maxRange(int fing_maxRange) {
        this.fing_maxRange = fing_maxRange;
        prefs.putInt("PawTracker2.fing_maxRange",fing_maxRange);
    }
    public int getFing_maxRange() {
        return fing_maxRange;
    }
     
    public void setFing_minRange(int fing_minRange) {
        this.fing_minRange = fing_minRange;
        prefs.putInt("PawTracker2.fing_minRange",fing_minRange);
    }
    public int getFing_minRange() {
        return fing_minRange;
    }
     
    
            
    public void setCluster_lifetime(int cluster_lifetime) {
        this.cluster_lifetime = cluster_lifetime;
        prefs.putInt("PawTracker2.cluster_lifetime",cluster_lifetime);
    }
    public int getCluster_lifetime() {
        return cluster_lifetime;
    }      
            
    public void setScore_range(int score_range) {
        this.score_range = score_range;
        prefs.putInt("PawTracker2.score_range",score_range);
    }
    public int getScore_range() {
        return score_range;
    }
    
    public void setScore_threshold(int score_threshold) {
        this.score_threshold = score_threshold;
        prefs.putInt("PawTracker2.score_threshold",score_threshold);
    }
    public int getScore_threshold() {
        return score_threshold;
    }
    
    public void setScore_in_threshold(float score_in_threshold) {
        this.score_in_threshold = score_in_threshold;
        prefs.putFloat("PawTracker2.score_in_threshold",score_in_threshold);
    }
    public float getScore_in_threshold() {
        return score_in_threshold;
    }
    
    public void setScore_sup_threshold(float score_sup_threshold) {
        this.score_sup_threshold = score_sup_threshold;
        prefs.putFloat("PawTracker2.score_sup_threshold",score_sup_threshold);
    }
    public float getScore_sup_threshold() {
        return score_sup_threshold;
    }
       
    public void setParallel_range(int parallel_range) {
        this.parallel_range = parallel_range;
        prefs.putInt("PawTracker2.parallel_range",parallel_range);
    }
    public int getParallel_range() {
        return parallel_range;
    }
    
            
    public void setParallel_mix(float parallel_mix) {
        this.parallel_mix = parallel_mix;
        prefs.putFloat("PawTracker2.parallel_mix",parallel_mix);
    }
    public float getParallel_mix() {
        return parallel_mix;
    }
    
    
            
    public void setKnuckle_inertia(float knuckle_inertia) {
        this.knuckle_inertia = knuckle_inertia;
        prefs.putFloat("PawTracker2.knuckle_inertia",knuckle_inertia);
    }
    public float getKnuckle_inertia() {
        return knuckle_inertia;
    }        
    
    public void setSeqTolerance(float seqTolerance) {
        this.seqTolerance = seqTolerance;
        prefs.putFloat("PawTracker2.seqTolerance",seqTolerance);
    }
    public float getSeqTolerance() {
        return seqTolerance;
    }
    
    public void setMinDiff(float minDiff) {
        this.minDiff = minDiff;
        prefs.putFloat("PawTracker2.minDiff",minDiff);
    }
    public float getMinDiff() {
        return minDiff;
    }
    
    
    public void setIntensityZoom(int intensityZoom) {
        this.intensityZoom = intensityZoom;
        prefs.putInt("PawTracker2.intensityZoom",intensityZoom);
    }
    
    public int getIntensityZoom() {
        return intensityZoom;
    }    
    
    
            
    public void setIn_length(int in_length) {
        this.in_length = in_length;
        prefs.putInt("PawTracker2.in_length",in_length);
    }
    
    public int getIn_length() {
        return in_length;
    }   
    
    public void setIn_test_length(int in_test_length) {
        this.in_test_length = in_test_length;
        prefs.putInt("PawTracker2.in_test_length",in_test_length);
    }
    
    public int getIn_test_length() {
        return in_test_length;
    }   
    
    public void setDoor_xa(int door_xa) {
        this.door_xa = door_xa;
        prefs.putInt("PawTracker2.door_xa",door_xa);
    }
    
    public int getDoor_xa() {
        return door_xa;
    }
    
    public void setDoor_xb(int door_xb) {
        this.door_xb = door_xb;
        prefs.putInt("PawTracker2.door_xb",door_xb);
    }
    
    public int getDoor_xb() {
        return door_xb;
    }
    
    public void setDoor_ya(int door_ya) {
        this.door_ya = door_ya;
        prefs.putInt("PawTracker2.door_ya",door_ya);
    }
    
    public int getDoor_ya() {
        return door_ya;
    }
    
    public void setDoor_yb(int door_yb) {
        this.door_yb = door_yb;
        prefs.putInt("PawTracker2.door_yb",door_yb);
    }
    
    public int getDoor_yb() {
        return door_yb;
    }
    
    
    public void setNode_range(float node_range) {
        this.node_range = node_range;
        prefs.putFloat("PawTracker2.node_range",node_range);
    }
    public float getNode_range() {
        return node_range;
    } 
       
    public void setFinger_sensitivity(float finger_sensitivity) {
        this.finger_sensitivity = finger_sensitivity;
        prefs.putFloat("PawTracker2.finger_sensitivity",finger_sensitivity);
    }
    public float getFinger_sensitivity() {
        return finger_sensitivity;
    } 
    
    public void setFinger_mv_smooth(float finger_mv_smooth) {
        this.finger_mv_smooth = finger_mv_smooth;
        prefs.putFloat("PawTracker2.finger_mv_smooth",finger_mv_smooth);
    }
    public float getFinger_mv_smooth() {
        return finger_mv_smooth;
    } 
    
    public void setFinger_length(float finger_length) {
        this.finger_length = finger_length;
        prefs.putFloat("PawTracker2.finger_length",finger_length);
    }
    public float getFinger_length() {
        return finger_length;
    } 
    
    public void setFinger_start_threshold(int finger_start_threshold) {
        this.finger_start_threshold = finger_start_threshold;
        prefs.putInt("PawTracker2.finger_start_threshold",finger_start_threshold);
    }
    public int getFinger_start_threshold() {
        return finger_start_threshold;
    }       
            
    public void setFinger_start_range(float finger_start_range) {
        this.finger_start_range = finger_start_range;
        prefs.putFloat("PawTracker2.finger_start_range",finger_start_range);
    }
    public float getFinger_start_range() {
        return finger_start_range;
    } 
    
    public void setFinger_cluster_range(float finger_cluster_range) {
        this.finger_cluster_range = finger_cluster_range;
        prefs.putFloat("PawTracker2.finger_cluster_range",finger_cluster_range);
    }
    public float getFinger_cluster_range() {
        return finger_cluster_range;
    }  
    
     public void setFinger_ori_variance(float finger_ori_variance) {
        this.finger_ori_variance = finger_ori_variance;
        prefs.putFloat("PawTracker2.finger_ori_variance",finger_ori_variance);
    }
    public float getFinger_ori_variance() {
        return finger_ori_variance;
    }  
    
    
    
    
    public void setContour_min_thresh(float contour_min_thresh) {
        this.contour_min_thresh = contour_min_thresh;
        prefs.putFloat("PawTracker2.contour_min_thresh",contour_min_thresh);
    }
    public float getContour_min_thresh() {
        return contour_min_thresh;
    }  
    
    public void setContour_act_thresh(float contour_act_thresh) {
        this.contour_act_thresh = contour_act_thresh;
        prefs.putFloat("PawTracker2.contour_act_thresh",contour_act_thresh);
    }
    public float getContour_act_thresh() {
        return contour_act_thresh;
    }  
      
    public void setContour_range(int contour_range) {
        this.contour_range = contour_range;
        prefs.putInt("PawTracker2.contour_range",contour_range);
    }
    public int getContour_range() {
        return contour_range;
    }
    
    
            
    public void setTracker_time_bin(float tracker_time_bin) {
        this.tracker_time_bin = tracker_time_bin;
        prefs.putFloat("PawTracker2.tracker_time_bin",tracker_time_bin);
    }
    public float getTracker_time_bin() {
        return tracker_time_bin;
    }         
            
    public void setMaxDiff(float maxDiff) {
        this.maxDiff = maxDiff;
        prefs.putFloat("PawTracker2.maxDiff",maxDiff);
    }
    public float getMaxDiff() {
        return maxDiff;
    }
    
    public void setDoorMinDiff(float doorMinDiff) {
        this.doorMinDiff = doorMinDiff;
        prefs.putFloat("PawTracker2.doorMinDiff",doorMinDiff);
    }
    public float getDoorMinDiff() {
        return doorMinDiff;
    }
    
    public void setDoorMaxDiff(float doorMaxDiff) {
        this.doorMaxDiff = doorMaxDiff;
        prefs.putFloat("PawTracker2.doorMaxDiff",doorMaxDiff);
    }
    public float getDoorMaxDiff() {
        return doorMaxDiff;
    }
    
    public void setMinZeroes(int minZeroes) {
        this.minZeroes = minZeroes;
        prefs.putInt("PawTracker2.minZeroes",minZeroes);
    }
    public int getMinZeroes() {
        return minZeroes;
    }
    
    public void setMaxZeroes(int maxZeroes) {
        this.maxZeroes = maxZeroes;
        prefs.putInt("PawTracker2.maxZeroes",maxZeroes);
    }
    
    public int getMaxZeroes() {
        return maxZeroes;
    }
    
    public void setMinAngle(float minAngle) {
        this.minAngle = minAngle;
        prefs.putFloat("PawTracker2.minAngle",minAngle);
    }
    public float getMinAngle() {
        return minAngle;
    }
    
  
    
    public void setDoorMinZeroes(int doorMinZeroes) {
        this.doorMinZeroes = doorMinZeroes;
        prefs.putInt("PawTracker2.doorMinZeroes",doorMinZeroes);
    }
    public int getDoorMinZeroes() {
        return doorMinZeroes;
    }
    
    public void setDoorMaxZeroes(int doorMaxZeroes) {
        this.doorMaxZeroes = doorMaxZeroes;
        prefs.putInt("PawTracker2.doorMaxZeroes",doorMaxZeroes);
    }
    public int getDoorMaxZeroes() {
        return doorMaxZeroes;
    }
    
    public void setFingertip_size(int fingertip_size) {
        this.fingertip_size = fingertip_size;
        prefs.putInt("PawTracker2.fingertip_size",fingertip_size);
    }
    public int getFingertip_size() {
        return fingertip_size;
    }
    
    

    
    
            
    public int getMinSeqLength() {
        return minSeqLength;
    }
    
    public void setMinSeqLength(int minSeqLength) {
        this.minSeqLength = minSeqLength;
        
    }     
            
    public int getMax_fingers() {
        return max_fingers;
    }
    
    public void setMax_fingers(int max_fingers) {
        this.max_fingers = max_fingers;
        
    }
    
//    public int getRetinaSize() {
//        return retinaSize;
//    }
//    
//    public void setRetinaSize(int retinaSize) {
//        this.retinaSize = retinaSize;
//        prefs.putInt("PawTracker2.retinaSize",retinaSize);
//        
//    }
    
    public int getLinkSize() {
        return linkSize;
    }
    
    public void setLinkSize(int linkSize) {
        this.linkSize = linkSize;
        prefs.putInt("PawTracker2.linkSize",linkSize);
    }
    
    
    public int getBoneSize() {
        return boneSize;
    }
    
    public void setBoneSize(int boneSize) {
        this.boneSize = boneSize;
        prefs.putInt("PawTracker2.boneSize",boneSize);
    }
    
    public int getSegSize() {
        return segSize;
    }
    
    public void setSegSize(int segSize) {
        this.segSize = segSize;
        prefs.putInt("PawTracker2.segSize",segSize);
    }
    
    public int getMaxSegSize() {
        return maxSegSize;
    }
    
    public void setMaxSegSize(int maxSegSize) {
        this.maxSegSize = maxSegSize;
        prefs.putInt("PawTracker2.maxSegSize",maxSegSize);
    }
    
            
    private boolean resetPawTracking=prefs.getBoolean("PawTracker2.resetPawTracking",false);
    
    
    public boolean isResetPawTracking() {
        return resetPawTracking;
    }
    public void setResetPawTracking(boolean resetPawTracking) {
        this.resetPawTracking = resetPawTracking;
        prefs.putBoolean("PawTracker2.resetPawTracking",resetPawTracking);
        
    }
    
    
    public void setUseFingerDistanceSmooth(boolean useFingerDistanceSmooth){
        this.useFingerDistanceSmooth = useFingerDistanceSmooth;
        prefs.putBoolean("PawTracker2.useFingerDistanceSmooth",useFingerDistanceSmooth);
    }
    public boolean isUseFingerDistanceSmooth(){
        return useFingerDistanceSmooth;
    }
    
    public void setUseSimpleContour(boolean useSimpleContour){
        this.useSimpleContour = useSimpleContour;
        prefs.putBoolean("PawTracker2.useSimpleContour",useSimpleContour);
    }
    public boolean isUseSimpleContour(){
        return useSimpleContour;
    }
    
    
     public void setShowSkeletton(boolean showSkeletton){
        this.showSkeletton = showSkeletton;
        prefs.putBoolean("PawTracker2.showSkeletton",showSkeletton);
    }
    public boolean isShowSkeletton(){
        return showSkeletton;
    }
    
    
    public void setShowPalm(boolean showPalm){
        this.showPalm = showPalm;
        prefs.putBoolean("PawTracker2.showPalm",showPalm);
    }
    public boolean isShowPalm(){
        return showPalm;
    }       
            
    public void setShowThin(boolean showThin){
        this.showThin = showThin;
        prefs.putBoolean("PawTracker2.showThin",showThin);
    }
    public boolean isShowThin(){
        return showThin;
    }
     public void setThinning(boolean thinning){
        this.thinning = thinning;
        prefs.putBoolean("PawTracker2.thinning",thinning);
    }
    public boolean isThinning(){
        return thinning;
    }
    
     public float getThin_Threshold() {
        return thin_threshold;
    }
    
    public void setThin_Threshold(float thin_threshold) {
        this.thin_threshold = thin_threshold;
        prefs.putFloat("PawTracker2.thin_threshold",thin_threshold);
    }
    
    
    
    public void setShowSegments(boolean showSegments){
        this.showSegments = showSegments;
        prefs.putBoolean("PawTracker2.showSegments",showSegments);
    }
    public boolean isShowSegments(){
        return showSegments;
    }
    
    
     public void setScaleAcc(boolean scaleAcc){
        this.scaleAcc = scaleAcc;
        prefs.putBoolean("PawTracker2.scaleAcc",scaleAcc);
    }
    public boolean isScaleAcc(){
        return scaleAcc;
    }
      
    public void setScaleIntensity(boolean scaleIntensity){
        this.scaleIntensity = scaleIntensity;
        prefs.putBoolean("PawTracker2.scaleIntensity",scaleIntensity);
    }
    public boolean isScaleIntensity(){
        return scaleIntensity;
    }
    
    public void setShowAcc(boolean showAcc){
        this.showAcc = showAcc;
        prefs.putBoolean("PawTracker2.showAcc",showAcc);
    }
    public boolean isShowAcc(){
        return showAcc;
    }
 
    public int getDensityMinIndex() {
        return densityMinIndex;
    }
    
    public void setDensityMinIndex(int densityMinIndex) {
        this.densityMinIndex = densityMinIndex;
        prefs.putInt("PawTracker2.densityMinIndex",densityMinIndex);
    }
    
    public int getDensityMaxIndex() {
        return densityMaxIndex;
    }
    
    public void setDensityMaxIndex(int densityMaxIndex) {
        this.densityMaxIndex = densityMaxIndex;
        prefs.putInt("PawTracker2.densityMaxIndex",densityMaxIndex);
    }
    
    public void setShowDensity(boolean showDensity){
        this.showDensity = showDensity;
        prefs.putBoolean("PawTracker2.showDensity",showDensity);
    }
    public boolean isShowDensity(){
        return showDensity;
    } 
    
    public void setScaleInDoor(boolean scaleInDoor){
        this.scaleInDoor = scaleInDoor;
        prefs.putBoolean("PawTracker2.scaleInDoor",scaleInDoor);
    }
    public boolean isScaleInDoor(){
        return scaleInDoor;
    } 
    
            
            
    public int getDecayLimit() {
        return decayLimit;
    }
    
    public void setDecayLimit(int decayLimit) {
        this.decayLimit = decayLimit;
        prefs.putInt("PawTracker2.decayLimit",decayLimit);
    }
    
    public void setDecayOn(boolean decayOn){
        this.decayOn = decayOn;
        prefs.putBoolean("PawTracker2.decayOn",decayOn);
    }
    public boolean isDecayOn(){
        return decayOn;
    } 
    
    public void setShowWindow(boolean showWindow){
        this.showWindow = showWindow;
        prefs.putBoolean("PawTracker2.showWindow",showWindow);
    }
    public boolean isShowWindow(){
        return showWindow;
    }
    
    
    public void setShowScore(boolean showScore){
        this.showScore = showScore;
        prefs.putBoolean("PawTracker2.showScore",showScore);
    }
    public boolean isShowScore(){
        return showScore;
    }       
    
    
    public void setShowIntensity(boolean showIntensity){
        this.showIntensity = showIntensity;
        prefs.putBoolean("PawTracker2.showIntensity",showIntensity);
    }
    public boolean isShowIntensity(){
        return showIntensity;
    } 
    
    public void setHideInside(boolean hideInside){
        this.hideInside = hideInside;
        prefs.putBoolean("PawTracker2.hideInside",hideInside);
    }
    public boolean isHideInside(){
        return hideInside;
    }      
    
    public void setShowFingers(boolean showFingers){
        this.showFingers = showFingers;
        prefs.putBoolean("PawTracker2.showFingers",showFingers);
    }
    public boolean isShowFingers(){
        return showFingers;
    }
    
    public void setShowClusters(boolean showClusters){
        this.showClusters = showClusters;
        prefs.putBoolean("PawTracker2.showClusters",showClusters);
    }
    public boolean isShowClusters(){
        return showClusters;
    }      
            
    public void setShowFingerTips(boolean showFingerTips){
        this.showFingerTips = showFingerTips;
        prefs.putBoolean("PawTracker2.showFingerTips",showFingerTips);
    }
    public boolean isShowFingerTips(){
        return showFingerTips;
    }
    
    public void setShowZones(boolean showZones){
        this.showZones = showZones;
        prefs.putBoolean("PawTracker2.showZones",showZones);
    }
    public boolean isShowZones(){
        return showZones;
    }
    public void setShowAll(boolean showAll){
        this.showAll = showAll;
        prefs.putBoolean("PawTracker2.showAll",showAll);
    }
    public boolean isShowAll(){
        return showAll;
    }
    
    
       
//    public void setShowSequences(boolean showSequences){
//        this.showSequences = showSequences;
//        prefs.putBoolean("PawTracker2.showSequences",showSequences);
//    }
//    public boolean isShowSequences(){
//        return showSequences;
//    }
    
    public void setShowShape(boolean showShape){
        this.showShape = showShape;
        prefs.putBoolean("PawTracker2.showShape",showShape);
    }
    public boolean isShowShape(){
        return showShape;
    }
    
    
    public void setShowShapePoints(boolean showShapePoints){
        this.showShapePoints = showShapePoints;
        prefs.putBoolean("PawTracker2.showShapePoints",showShapePoints);
    }
    public boolean isShowShapePoints(){
        return showShapePoints;
    }
    public void setSmoothShape(boolean smoothShape){
        this.smoothShape = smoothShape;
        prefs.putBoolean("PawTracker2.smoothShape",smoothShape);
    }
    public boolean isSmoothShape(){
        return smoothShape;
    }  
    
    
   
    
 
    
    
    public int getLowFilter_radius() {
        return lowFilter_radius;
    }
    
    public void setLowFilter_radius(int lowFilter_radius) {
        this.lowFilter_radius = lowFilter_radius;
        prefs.putInt("PawTracker2.lowFilter_radius",lowFilter_radius);
    }
    
    public int getLowFilter_density() {
        return lowFilter_density;
    }
    
    public void setLowFilter_density(int lowFilter_density) {
        this.lowFilter_density = lowFilter_density;
        prefs.putInt("PawTracker2.lowFilter_density",lowFilter_density);
    }
    
     public float getLowFilter_threshold() {
        return lowFilter_threshold;
    }
    
    public void setLowFilter_threshold(float lowFilter_threshold) {
        this.lowFilter_threshold = lowFilter_threshold;
        prefs.putFloat("PawTracker2.lowFilter_threshold",lowFilter_threshold);
    }
    
   
}
