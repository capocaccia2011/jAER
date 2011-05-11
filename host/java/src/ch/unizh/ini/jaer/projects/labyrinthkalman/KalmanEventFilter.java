/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package ch.unizh.ini.jaer.projects.labyrinthkalman;

import java.awt.geom.Point2D;
import net.sf.jaer.chip.AEChip;
import net.sf.jaer.event.*;

import javax.media.opengl.GL;
import javax.media.opengl.GLAutoDrawable;
import javax.vecmath.Color3f;

import net.sf.jaer.Description;
import net.sf.jaer.eventprocessing.EventFilter2D;
import net.sf.jaer.graphics.FrameAnnotater;

/**
 * The specialized Kalman filter for ball tracking in the labyrinth game.
 * @author Eero, Tobias Pietzsch
 */
@Description("specialized Kalman filter for ball tracking in the labyrinth game")
public class KalmanEventFilter extends EventFilter2D implements FrameAnnotater {

	private LabyrinthBallKalmanFilter kf;
	
    // the timestamp of the most recent received event
    private int t = -1;

    // parameters
    private double measurementSigma = getDouble("measurementSigma", 3.0);
    private double processSigma = getDouble("processSigma", 100.0);
    private double measurementThreshold = getDouble("measurementThreshold", 3.0);

    private boolean annotationEnabled = true;
    LabyrinthBallController controller;
    
    public KalmanEventFilter(AEChip chip) {
    	this(chip,null);
    }

    public KalmanEventFilter(AEChip chip, LabyrinthBallController controller) {
        super(chip);
        this.controller=controller;
        this.kf = new LabyrinthBallKalmanFilter();
        setPropertyTooltip("processSigma","???"); // TODO
        setPropertyTooltip("measurementThreshold","???");
        setPropertyTooltip("measurementSigma","???");

        resetFilter();
    }

    public float getMeasurementSigma() {
        return (float)measurementSigma;
    }
    synchronized public void setMeasurementSigma(float measurementSigma) {

        if(measurementSigma < 0) measurementSigma = 0;
        putDouble("measurementSigma", measurementSigma);

        if(measurementSigma != this.measurementSigma) {
            this.measurementSigma = measurementSigma;
            resetFilter();
        }
    }

    public float getProcessSigma() {
        return (float)processSigma;
    }
    synchronized public void setProcessSigma(float processSigma) {

        if(processSigma < 0) processSigma = 0;
        putDouble("processSigma", processSigma);

        if(processSigma != this.processSigma) {
            this.processSigma = processSigma;
            resetFilter();
        }
    }

    public float getMeasurementThreshold() {
        return (float)measurementThreshold;
    }
    synchronized public void setMeasurementThreshold(float measurementThreshold) {

        if(measurementThreshold < 0) measurementThreshold = 0;
        putDouble("measurementThreshold", measurementThreshold);

        if(measurementThreshold != this.measurementThreshold) {
            this.measurementThreshold = measurementThreshold;
            resetFilter();
        }
    }

    @Override
    final public void resetFilter()
    {
    	kf.setMeasurementSigma( measurementSigma );
    	kf.setProcessSigma( processSigma );
    	kf.resetFilter();
    }

    @Override
    public void initFilter() {
        resetFilter();
    }

    @Override
    public EventPacket<?> filterPacket(EventPacket<?> in) {

        if (!isFilterEnabled())
            return in;

        if (in == null || in.getSize() == 0)
            return in;

        // TODO: get the performed actions from the controller
        final double[] act = new double[2];
        
    	int timestamp = in.getFirstTimestamp(); 
        if (t >= 0) {
            double dt = (double)(timestamp - t)/1000000.0;
            kf.predict(act, dt);        	
        }
        t = timestamp;

        final double[] bestMeas = new double[2];
        final double[] meas     = new double[2];
        double minDistance = Double.MAX_VALUE;
        for (BasicEvent event : in) {
            meas[0] = event.x;
            meas[1] = event.y;

            double distance = kf.mahalanobisToMeasurement(meas);

            if (distance < minDistance) {
                minDistance = distance;
                bestMeas[0] = meas[0];
                bestMeas[1] = meas[1];
                // timestamp = event.timestamp;
            }
        }

        if (minDistance <= measurementThreshold) {
            kf.correct(bestMeas);
        }

        return in;
    }

    @Override
    public void setAnnotationEnabled(boolean annotationEnabled) {
        this.annotationEnabled = annotationEnabled;
    }

    @Override
    public boolean isAnnotationEnabled() {
        return annotationEnabled;
    }
    
    protected static void paintCovarianceEllipse( double[] pos, double[][] cov, final double nsigmas, Color3f color, GL gl )
    {
        final int no_points_ellipse = 12;

        final double a = cov[0][0];
        final double b = cov[1][0];
        final double c = cov[1][1];
        final double d = Math.sqrt( a*a + 4*b*b - 2*a*c + c*c );
        final double scale1 = Math.sqrt( 0.5 * ( a+c+d ) ) * nsigmas;
        final double scale2 = Math.sqrt( 0.5 * ( a+c-d ) ) * nsigmas;
        final double theta = 0.5 * Math.atan2( (2*b), (a-c) ) * 180.0 / Math.PI;

        gl.glPushMatrix();

        gl.glTranslated(pos[0], pos[1], 0);
        gl.glRotated(theta, 0, 0, 1);
        gl.glScaled(scale1, scale2, 1);

        gl.glColor3f( color.x, color.y, color.z );
        gl.glLineWidth(2);    	

        gl.glBegin(GL.GL_LINE_LOOP);
        final double angle = 2*Math.PI / (double)no_points_ellipse;
        for (int i = 0;i<no_points_ellipse;i++)
            gl.glVertex2d( Math.cos( i*angle ), Math.sin( i*angle ) );
        gl.glEnd();

        gl.glPopMatrix();
    }
    
    protected static void paintKalmanFilterState( final LabyrinthBallKalmanFilter filter, GL gl, Color3f covColor, Color3f speedColor )
    {
        final double[] mu = filter.getMu();
        final double[][] Sigma = filter.getSigma();

        paintCovarianceEllipse( mu, Sigma, 3, covColor, gl );

        double velx = mu[2] * 0.1;
        double vely = mu[3] * 0.1;

        gl.glColor3f( speedColor.x, speedColor.y, speedColor.z );
        gl.glLineWidth(4);

        gl.glPushMatrix();
        gl.glTranslated(mu[0], mu[1], 0);
        gl.glBegin(GL.GL_LINES);
        gl.glVertex2d(0, 0);
        gl.glVertex2d(velx, vely);
        gl.glEnd();
        gl.glPopMatrix();

        gl.glLineWidth(1);
        gl.glColor3f(1,1,1);
        gl.glBegin(GL.GL_LINES);
        gl.glVertex2d(64, 64);
        gl.glVertex2d(mu[0], mu[1]);
        gl.glEnd();    	
    }

    @Override
    public void annotate(GLAutoDrawable drawable) {

        if(!isFilterEnabled())
            return;

        if (!annotationEnabled)
            return;

        GL gl=drawable.getGL();
        Color3f covColor = new Color3f(1,1,0);
        Color3f speedColor = new Color3f(1,0,0);
        paintKalmanFilterState( kf, gl, covColor, speedColor );
    }

    public Point2D.Float getBallPosition() {

        Point2D.Float position = new Point2D.Float();
        position.setLocation(kf.getMu()[0], kf.getMu()[1]);
        return position;
    }

    public double getBallPositionX() {
        return kf.getMu()[0];
    }

    public double getBallPositionY() {
        return kf.getMu()[1];
    }
    public Point2D.Float getBallVelocity() {

        Point2D.Float position = new Point2D.Float();
        position.setLocation(kf.getMu()[2], kf.getMu()[3]);
        return position;
    }
}
