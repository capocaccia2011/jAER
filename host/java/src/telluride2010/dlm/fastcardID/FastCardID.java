/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

/*
 * PcaTrackingFilter.java
 *
 * @author David Mascarnas
 * Created on July 11, 2010
 *
 * "This is meant to identify a playing card moving quickly across the field of
 *view of the retina.  It uses PCA tracking, and should be precedded by a
 *medianTrackingFilter"
 *
 * To change this template, choose Tools | Options and locate the template under
 * the Source Creation and Management node. Right-click the template and choose
 * Open. You can then make changes to the template in the Source Editor.
 */

package telluride2010.dlm.fastcardID;
import net.sf.jaer.chip.AEChip;
import net.sf.jaer.event.*;
import net.sf.jaer.eventprocessing.EventFilter2D;
import net.sf.jaer.graphics.FrameAnnotater;
import net.sf.jaer.util.filter.LowpassFilter;
import java.awt.Graphics2D;
import java.awt.geom.*;
import java.util.Arrays;
import javax.media.opengl.*;
import javax.media.opengl.GLAutoDrawable;
//DLM additions
import java.lang.Math;



public class FastCardID extends EventFilter2D implements FrameAnnotater {
    public static String getDescription(){
        return "Tracks a single object pose by Principle Components Analysis";
    }

    //Required variables for this class.
    float xsum = 0f;
    float ysum = 0f;
    float xxsum = 0f;
    float yysum = 0f;
    float xysum = 0f;

    float xmean = 0f;
    float ymean = 0f;
    float xxvar = 0f;
    float yyvar = 0f;
    float xycovar = 0f;

    int n = 4096;
    //int n = 2048;
    //int n = 1024;                    //This is the length of the ring buffer
    //int n = 512;
    //int n = 256;
    //int n = 128;          //This worked well for alot of the card data
    //int n = 64;

    //int scale_stddev = 1;


    //protected int dt=getPrefs().getInt("BackgroundActivityFilter.dt",30000);
    //{setPropertyTooltip("dt","Events with less than this delta time in us to neighbors pass through");}

    double stddev_inc = .1;     //I need this so I can exert fine control over the size of the bounding box.
    int scale_stddev=getPrefs().getInt("FastCardID.scale_stddev",10);
    {setPropertyTooltip("scale_stddev","Scale of bounding box relative to the standard deviation (.1 * standard dev)");}

     int ring_buffer_length=getPrefs().getInt("FastCardID.ring_buffer_length",1024);
    {setPropertyTooltip("ring_buffer_length","Length of the ring buffer.  Max 4092");}


    float[] ring_bufferX = new float[n];      //This is the array for the the ring buffer.  I need to make sure this is initialized to zeros
    float[] ring_bufferY = new float[n];
    int rb_index = 0;       //This is the ring buffer index

    //aspect ratio values
    double aspect_ratio = 0;    //This is the aspect ratio of the std deviations.
    double aspect_ratio_error = 0;
    double aspect_ratio_card = 1.4 ;// (3.5 in/2.5 in);

    double aspect_ratio_allowable_error_inc = .01;

    int aspect_ratio_allowable_error_pts=getPrefs().getInt("FastCardID.aspect_ratio_allowable_error_pts",10);
    {setPropertyTooltip("aspect_ratio_allowable_error_pts","Aspect ratio allowable error in pts");}

     int pip_bin_hist_threshold=getPrefs().getInt("FastCardID.pip_bin_hist_threshold",100);
    {setPropertyTooltip("pip_bin_hist_threshold","number of events needed in a bin to 'see' a pip");}


    //histogram variables
    int[] card_value_hist = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};    //This is a histogram for the number of the card
    int[] pip_bin_hist = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};             //This is the count of events in a given sector of the card
    int pip_count = 0;

    //flags
    boolean card_present_flag = false;

    double relx = 0;
    double rely = 0;
    double cardx = 0;
    double cardy = 0;


    double theta = 0f;      //The angle the data is decorrelated at

    double R1 = 0f;
    double R2 = 0f;
    double R3 = 0f;
    double R4 = 0f;

    /////////////////////////////////////////


    Point2D medianPoint=new Point2D.Float(), stdPoint=new Point2D.Float(), meanPoint=new Point2D.Float(), eigvec1Point = new Point2D.Float(), eigvec2Point = new Point2D.Float();
    Point2D square1Point=new Point2D.Float(), square2Point=new Point2D.Float(), square3Point=new Point2D.Float(), square4Point=new Point2D.Float() ;
    Point2D eventPoint = new Point2D.Float();
    Point2D stddev1Point=new Point2D.Float();
    Point2D stddev2Point=new Point2D.Float();
    float xmedian=0f;
    float ymedian=0f;
    float xstd=0f;
    float ystd=0f;
    //float xmean=0, ymean=0;
    int lastts=0, dt=0;
    int prevlastts=0;

    //This is kind of the heart of the card ID

    //three vertical columns
    double biny1 = .2;
    double biny2 = .3;
    double biny3 = .7;

    //Left and right x columns
    double binx1 = .0362;
    double binx2 = .374;
    double binx3 = .446;
    double binx4 = .784;

    //middle column
    double binx5 = .143;
    double binx6 = .3571;
    double binx7 = .643;




    /** Creates a new instance of PcaTracker */
    public FastCardID(AEChip chip) {
        super(chip);

        //This initializes the ring_buffers to all zeros.
        //These must start as all zeros for my algorithm to work.
        for (int k = 0 ; k < n; k++) {
            ring_bufferX[k] = 0;
            ring_bufferY[k] = 0;
        }

    }


   

    public void resetFilter() {
        medianPoint.setLocation(chip.getSizeX()/2,chip.getSizeY()/2);
        meanPoint.setLocation(chip.getSizeX()/2,chip.getSizeY()/2);
        stdPoint.setLocation(1,1);
    }

    public Point2D getMedianPoint() {
        return this.medianPoint;
    }

    public Point2D getStdPoint() {
        return this.stdPoint;
    }

    public Point2D getMeanPoint() {
        return this.meanPoint;
    }

    public int getScale_stddev() {
        return this.scale_stddev;
    }

    /**
     * sets the scale of the bounding box relative to stddev
     <p>
     Fires a PropertyChangeEvent "scale_stddev"

     * @see #getDt
     * @param dt delay in us
     */
    public void setScale_stddev(final int scale_stddev) {
        getPrefs().putInt("FastCardID.scale_stddev",scale_stddev);
        support.firePropertyChange("scale_std",this.scale_stddev,scale_stddev);
        this.scale_stddev = scale_stddev;
    }

    public int getAspect_ratio_allowable_error_pts() {
        return this.aspect_ratio_allowable_error_pts;
    }

     public void setAspect_ratio_allowable_error_pts(final int aspect_ratio_allowable_error_pts) {
         getPrefs().putInt("FastCardID.aspect_ratio_allowable_error_pts",aspect_ratio_allowable_error_pts);
        support.firePropertyChange("aspect_ratio_allowable_error_pts",this.aspect_ratio_allowable_error_pts,aspect_ratio_allowable_error_pts);
        this.aspect_ratio_allowable_error_pts = aspect_ratio_allowable_error_pts;
     }

    public int getPip_bin_hist_threshold() {
        return this.pip_bin_hist_threshold;
    }

    /**@param tauUs the time constant of the 1st order lowpass filter on median location */
    public void setPip_bin_hist_threshold(final int pip_bin_hist_threshold) {
        this.pip_bin_hist_threshold = pip_bin_hist_threshold;
        getPrefs().putInt("FastCardID.pip_bin_hist_threshold", pip_bin_hist_threshold);
    }



    public int getRing_buffer_length() {
        return this.ring_buffer_length;
    }

    /**
     * sets the scale of the bounding box relative to stddev
     <p>
     Fires a PropertyChangeEvent "scale_stddev"

     * @see #getDt
     * @param dt delay in us
     */
    public void setRing_buffer_length(final int ring_buffer_length) {

        if (ring_buffer_length < 16) {
            getPrefs().putInt("FastCardID.ring_buffer_length",16);
            support.firePropertyChange("ring_buffer_length",this.ring_buffer_length,16);
            this.ring_buffer_length = 16;
        } else if (ring_buffer_length > 4096) {
            getPrefs().putInt("FastCardID.ring_buffer_length",4096);
            support.firePropertyChange("ring_buffer_length",this.ring_buffer_length,4096);
            this.ring_buffer_length = 4096;
        } else {
            getPrefs().putInt("FastCardID.ring_buffer_length",ring_buffer_length);
            support.firePropertyChange("ring_buffer_length",this.ring_buffer_length,ring_buffer_length);
            this.ring_buffer_length = ring_buffer_length;
        }
    //At this point I think I need to reinitialize the filter

    //This initializes the ring_buffers to all zeros.
    //These must start as all zeros for my algorithm to work.
    for (int k = 0 ; k < n; k++) {
        ring_bufferX[k] = 0;
        ring_bufferY[k] = 0;
    }

    xsum = 0f;
    ysum = 0f;
    xxsum = 0f;
    yysum = 0f;
    xysum = 0f;

    xmean = 0f;
    ymean = 0f;
    xxvar = 0f;
    yyvar = 0f;
    xycovar = 0f;

    rb_index = 0;

    }



    public void initFilter() {
    }


    public EventPacket filterPacket(EventPacket in) {

        checkOutputPacketEventType(in); //This is what actually initiates the out EventPacket of the correct type.
        OutputEventIterator outItr=out.outputIterator();

            for(Object o:in){
                BasicEvent e=(BasicEvent)o;

                //This is to correctly increment the ring buffer.
                if (rb_index >= ring_buffer_length ){
                    rb_index = 0;
                }

                //System.out.println("=============================================");
                //System.out.println("x=" + e.x + " y=" + e.y);



                //Now calculate the mean information


                xsum = xsum + e.x - ring_bufferX[rb_index];
                ysum = ysum + e.y - ring_bufferY[rb_index];

                //System.out.println("x sum=" + xsum + " y sum=" + ysum + " rb index = " + rb_index);

                xmean = xsum/ring_buffer_length;
                ymean = ysum/ring_buffer_length;


                //System.out.println("x mean=" + xmean + " y mean=" + ymean);

                //Now calculate the variance information
                xxsum = xxsum + e.x*e.x - ring_bufferX[rb_index]*ring_bufferX[rb_index];
                yysum = yysum + e.y*e.y - ring_bufferY[rb_index]*ring_bufferY[rb_index];
                xysum = xysum + e.x*e.y - ring_bufferX[rb_index]*ring_bufferY[rb_index];

                //System.out.println("xx sum=" + xxsum + " yy sum=" + yysum + " xysum" + xysum);

                xxvar = (xxsum - ((xsum*xsum)/ring_buffer_length))/(ring_buffer_length-1);
                yyvar = (yysum - ((ysum*ysum)/ring_buffer_length))/(ring_buffer_length-1);
                xycovar = (xysum - ((xsum*ysum)/ring_buffer_length))/(ring_buffer_length);

                //System.out.println("xxvar=" + xxvar + " yyvar=" + yyvar + " xycovar=" + xycovar);

                //Now calculate the eigenvalues directly

                double a = 1;
                double b = -1 * (xxvar + yyvar);
                double c = xxvar * yyvar - xycovar * xycovar;

                double eig1presort = (-1 * b + java.lang.Math.sqrt(b*b - 4*a*c))/(2 * a);
                double eig2presort = (-1 * b - java.lang.Math.sqrt(b*b - 4*a*c))/(2 * a);

                //use the larger eigenvalue for stddev1 and the smaller eigenvalue for stddev2
                double eig1, eig2;
                if (eig1presort < eig2presort) {
                    eig1 = eig2presort;
                    eig2 = eig1presort;
                } else {
                    eig1 = eig1presort;
                    eig2 = eig2presort;
                }

                double stddev1 = java.lang.Math.sqrt(eig1);
                double stddev2 = java.lang.Math.sqrt(eig2);

                //System.out.println("Eig1=" + eig1 + " Eig2=" + eig2);

                //now calculate the eigenvectors

                double eigvec1x = eig1 - yyvar;
                double eigvec2x = eig2 - yyvar;

                double eigvec1y = xycovar;
                double eigvec2y = xycovar;

                //Now normalize the eignvectors

               double eig1norm = java.lang.Math.sqrt(eigvec1x*eigvec1x + eigvec1y*eigvec1y);
               double eig2norm = java.lang.Math.sqrt(eigvec2x*eigvec2x + eigvec2y*eigvec2y);

               eigvec1x = eigvec1x/eig1norm;
               eigvec1y = eigvec1y/eig1norm;

               eigvec2x = eigvec2x/eig2norm;
               eigvec2y = eigvec2y/eig2norm;


                //Set the points for drawing (I am not sure if this should be outside or inside the for loop)
                meanPoint.setLocation(xmean,ymean);

                eigvec1Point.setLocation(xmean + eigvec1x * stddev1, ymean + eigvec1y * stddev1);
                eigvec2Point.setLocation(xmean + eigvec2x * stddev2, ymean + eigvec2y * stddev2);      //The length of this vector is scaled by the eigenvalues.

                /*
                //This stuff really slowed everything down, but it seems to work fine without it.
                scale_stddev = getScale_stddev();
                setScale_stddev(scale_stddev);
                 */

                square1Point.setLocation(xmean + scale_stddev * stddev_inc *(     (eigvec1x * stddev1) - (eigvec2x * stddev2)), ymean + scale_stddev * stddev_inc *(      (eigvec1y * stddev1) - (eigvec2y * stddev2)));
                square2Point.setLocation(xmean + scale_stddev * stddev_inc *(-1 * (eigvec1x * stddev1) - (eigvec2x * stddev2)), ymean + scale_stddev * stddev_inc *( -1 * (eigvec1y * stddev1) - (eigvec2y * stddev2)));
                square3Point.setLocation(xmean + scale_stddev * stddev_inc *(-1 * (eigvec1x * stddev1) + (eigvec2x * stddev2)), ymean + scale_stddev * stddev_inc *( -1 * (eigvec1y * stddev1) + (eigvec2y * stddev2)));
                square4Point.setLocation(xmean + scale_stddev * stddev_inc *(     (eigvec1x * stddev1) + (eigvec2x * stddev2)), ymean + scale_stddev * stddev_inc *(      (eigvec1y * stddev1) + (eigvec2y * stddev2)));

                //eventPoint.setLocation(e.x,e.y);        //Just a point to show where the latest event occurs


                stddev1Point.setLocation(10, 10 + stddev1);
                stddev2Point.setLocation(20, 10 + stddev2);


                //Put the event data in the ring buffer
                if (rb_index < ring_buffer_length ) {
                ring_bufferX[rb_index] = e.x;
                ring_bufferY[rb_index] = e.y;

                } else {
                    rb_index = 0;
                    ring_bufferX[rb_index] = e.x;
                    ring_bufferY[rb_index] = e.y;
                }


                //Now begin checking whether or not a card is present
                //First check the aspect ratio;

                aspect_ratio  = stddev1/stddev2;

                aspect_ratio_error = java.lang.Math.abs(aspect_ratio -aspect_ratio_card)/aspect_ratio_card;

               
                if (aspect_ratio_error < (aspect_ratio_allowable_error_inc * aspect_ratio_allowable_error_pts)) {  
                    //check for a change in the flag to reset the histogram buffers
                    System.out.println("Card Present");
                    if (card_present_flag == false ) {
                        //Indicate card has entered the screen
                        //System.out.println("Card Present");
                        //Here reinitialize the card value matrix to 0s
                        for (int ii = 0; ii < card_value_hist.length; ii++){
                            card_value_hist[ii] = 0;            
                        }

                        for (int ii = 0; ii < pip_bin_hist.length; ii++){
                            pip_bin_hist[ii] = 0;
                        }
                    }
                    card_present_flag = true;
                } else {
                    System.out.println("Card Not Present");
                    if (card_present_flag == true ) {
                        //Output the card_value_hist
                        //Indicate card is gone
                        //System.out.println("Card Not Present");
                    }
                   card_present_flag = false;
                }

                if (card_present_flag == true ) {
                    
                    //If it has been determined that a card is present in the view of the camera
                    //Figure out if the event falls in a pip bin.  

                    //First transform the event to the scaled card coordinate frame.
                    //First subtract out the mean
                    relx = e.x - xmean;
                    rely = e.y - ymean;

                    //Now project this relative vector onto the card coordinate frame
                    //Then scale by the std dev in the two directions.  

                    cardx = (eigvec1x * e.x + eigvec1y * e.y)/(stddev1 * scale_stddev * stddev_inc);
                    cardy = (eigvec2x * e.x + eigvec2y * e.y)/(stddev2 * scale_stddev * stddev_inc);

                    //Now determine whether the event falls into one of the pip bins
                    //This is kind of the heart of the card ID
                    //Only pass forward events falling in a pip bin

                    

                    if ((cardy >= -1 * biny3) && (cardy <= -1 * biny2)) {

                        if ((cardx <= binx4) && (cardx >= binx3)){
                            BasicEvent i=(BasicEvent)outItr.nextOutput();
                            i.copyFrom(e);
                            pip_bin_hist[0]++;
                        } else if ((cardx <= binx2) && (cardx >= binx1 )) {
                            BasicEvent i=(BasicEvent)outItr.nextOutput();
                            i.copyFrom(e);
                            pip_bin_hist[1]++;
                        } else if ((cardx <= -1*binx1) && (cardx >= -1*binx2)) {
                            BasicEvent i=(BasicEvent)outItr.nextOutput();
                            i.copyFrom(e);
                            pip_bin_hist[2]++;
                        } else if ((cardx <= -1 * binx3 ) && (cardx >= -1 * binx4)) {
                            BasicEvent i=(BasicEvent)outItr.nextOutput();
                            i.copyFrom(e);
                            pip_bin_hist[3]++;
                        }

                    } else if ( (cardy >= -1 * biny1 ) && (cardy <= biny1 )  ) {

                        if ((cardx <= binx7) && ( cardx >= binx6)) {
                            BasicEvent i=(BasicEvent)outItr.nextOutput();
                            i.copyFrom(e);
                            pip_bin_hist[4]++;
                        } else if ((cardx <= binx5) && (cardx >= -1 * binx5)) {
                            BasicEvent i=(BasicEvent)outItr.nextOutput();
                            i.copyFrom(e);
                            pip_bin_hist[5]++;
                        } else if ((cardx <= -1 * binx6) && ( cardx >= -1 * binx7)) {
                            BasicEvent i=(BasicEvent)outItr.nextOutput();
                            i.copyFrom(e);
                            pip_bin_hist[6]++;
                        }

                    } else if ((cardy <= biny3 ) && (cardy >= biny2)) {

                        if ((cardx <= binx4) && (cardx >= binx3)){
                            BasicEvent i=(BasicEvent)outItr.nextOutput();
                            i.copyFrom(e);
                            pip_bin_hist[7]++;
                        } else if ((cardx <= binx2) && (cardx >= binx1 )) {
                            BasicEvent i=(BasicEvent)outItr.nextOutput();
                            i.copyFrom(e);
                            pip_bin_hist[8]++;
                        } else if ((cardx <= -1*binx1) && (cardx >= -1*binx2)) {
                            BasicEvent i=(BasicEvent)outItr.nextOutput();
                            i.copyFrom(e);
                            pip_bin_hist[9]++;
                        } else if ((cardx <= -1 * binx3 ) && (cardx >= -1 * binx4)) {
                            BasicEvent i=(BasicEvent)outItr.nextOutput();
                            i.copyFrom(e);
                            pip_bin_hist[10]++;
                        }
                    }

                    //Now run thru the pip_bin hist to check if the bins have
                    //passed the threshold for recognizing a pip.
                    // keep a count of the number of recognizing bins
                    //One oddity of how I do this, is that there is the possibilty of
                    //counting 11 pips when at most there can only be 10
                    //This case should probably be dealt with.
                    
                    if (card_present_flag == true ) {

                        pip_count = 0;
                        for (int ii = 0; ii<pip_bin_hist.length; ii++) {
                            if (pip_bin_hist[ii] >= pip_bin_hist_threshold) {
                                pip_count++;
                            }
                        }
                        
                    }

                    //At this point we have card recognition.
                    //Histogram the card value the system thinks it sees in
                    //card_value_hist
                    //For now assume the pip count equals the card value.
                    //Deal with the odd case of counting a value of 11 later.  

                    //card_value_hist[pip_count - 1]++;
                    
                }

                rb_index++;
            }

        return in; //I just put this there since it seems necessary and Tobi said to do it.
    }

    /** JOGL annotation */
    public void annotate(GLAutoDrawable drawable) {
        if(!isFilterEnabled()) return;
        Point2D p=meanPoint;
        Point2D s=eventPoint;
        //Point2D o=orthoPoint;
        Point2D q=eigvec1Point;
        Point2D r=eigvec2Point;

        Point2D t = square1Point;
        Point2D u = square2Point;
        Point2D v = square3Point;
        Point2D w = square4Point;

        Point2D sxx = stddev1Point;
        Point2D syy = stddev2Point;

        //These are points to make a square


        GL gl=drawable.getGL();
        // already in chip pixel context with LL corner =0,0
        gl.glPushMatrix();

        gl.glColor3f(0,1,0);
        gl.glLineWidth(4);
        gl.glBegin(GL.GL_LINES);

        //Draw the eigenvectors
        gl.glVertex2d(p.getX(), p.getY());      //Origen
        gl.glVertex2d(q.getX(), q.getY());      //End point

        gl.glVertex2d(p.getX(), p.getY());      //Origen
        gl.glVertex2d(r.getX(), r.getY());      //End point

        //Draw the current point
        //gl.glVertex2d(p.getX(), p.getY());      //Origen
        //gl.glVertex2d(s.getX(), s.getY());     //Origen

        gl.glEnd();

//==============================================================================
        // Draw the lines corresponding to the std deviation
        /*
        gl.glColor3f(0,1,1);
        gl.glLineWidth(2);
        gl.glBegin(GL.GL_LINES);
        //Draw the current point
        gl.glVertex2d(10, 10);      //Origen
        gl.glVertex2d(sxx.getX(), sxx.getY());      //Origen

        gl.glVertex2d(20, 10);      //Origen
        gl.glVertex2d(syy.getX(), syy.getY());      //Origen

        gl.glEnd();
        */

        gl.glColor3f(1,0,0);
        gl.glLineWidth(4);
        gl.glBegin(GL.GL_LINES);

        //Draw the bounding square
        gl.glVertex2d(t.getX(), t.getY());      //Origen
        gl.glVertex2d(u.getX(), u.getY());      //End point

        gl.glVertex2d(u.getX(), u.getY());      //Origen
        gl.glVertex2d(v.getX(), v.getY());      //End point

        gl.glVertex2d(v.getX(), v.getY());      //Origen
        gl.glVertex2d(w.getX(), w.getY());      //End point

        gl.glVertex2d(w.getX(), w.getY());      //Origen
        gl.glVertex2d(t.getX(), t.getY());      //return to beginning
        gl.glEnd();

        /*
        gl.glColor3f(1,1,0);
        gl.glLineWidth(4);
        gl.glBegin(GL.GL_POINTS);
        gl.glVertex2d(s.getX(), s.getY());
        gl.glEnd( );
         *
         */




        gl.glPopMatrix();
    }

}
