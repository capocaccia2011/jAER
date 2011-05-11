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
import net.sf.jaer.Description;
import net.sf.jaer.eventprocessing.EventFilter2D;
import net.sf.jaer.graphics.FrameAnnotater;

/**
 * The specialized Kalman filter for ball tracking in the labyrinth game.
 * @author Eero
 */
@Description("specialized Kalman filter for ball tracking in the labyrinth game")
public class KalmanFilter extends EventFilter2D implements FrameAnnotater {

    // n == size of state vector
    // m == size of control vector
    // k == size of measurement vector

    // Kalman filter parameters:
	private double[][] At;    // n*n, project state to state
    private double[][] Bt;    // n*m, project control to state
    private double[][] Ct;    // k*n, project state to measurement

    private double[]   mu;    // n
    private double[][] Sigma; // n*n

    private double[][] S;     // innovation covariance, k*k
    private double[][] SInv;  // S^{-1}, inverse innovation covariance, k*k

    private double[][] Kt;    // n*k
    private double[][] Rt;    // n*n, process noise covariance
    private double[][] Qt;    // k*k, measurement noise covariance

    // the timestamp of the most recent received event
    private int t = -1;

    /* Auxiliary matrices used for intermediate results:*/
    private double[][] Mnn1; //n*n, i.e., the size of At
    private double[][] Mnk1; //n*k, i.e., the size of Kt and CtT

    private double[] vn1; //n*1, i.e., the size of mu
    private double[] vn2; //n*1, i.e., the size of mu
    private double[] vk1; //k*1, i.e., the size of meas
    private double[] vk2; //k*1, i.e., the size of meas

    // parameters
    private double measurementSigma = getDouble("measurementSigma", 3.0);
    private double processSigma = getDouble("processSigma", 100.0);
    private double measurementThreshold = getDouble("measurementThreshold", 3.0);

    private boolean annotationEnabled = true;
    LabyrinthBallController controller;
    
    public KalmanFilter(AEChip chip) {
    	this(chip,null);
    }

    public KalmanFilter(AEChip chip, LabyrinthBallController controller) {
        super(chip);
        this.controller=controller;
        setPropertyTooltip("processSigma","???"); // TODO
        setPropertyTooltip("measurementThreshold","???");
        setPropertyTooltip("measurementSigma","???");
        At = new double[6][6];
        Bt = new double[6][2];
        Ct = new double[2][6];
        Kt = new double[6][2];

        Qt = new double[2][2];
        Rt = new double[6][6];
        
        mu = new double[6];
        Sigma = new double[6][6];

        vn1 = new double[6];
        vn2 = new double[6];
        
        vk1 = new double[2];
        vk2 = new double[2];

        Mnn1 = new double[6][6];
        Mnk1 = new double[6][2];
        SInv = new double[2][2];
        S = new double[2][2];

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
    	final double measCov = measurementSigma * measurementSigma;
        Qt[0][0] = measCov;
        Qt[1][1] = measCov;

        for ( int i = 0; i < 6; ++i )
        {
            mu[i] = 0;
            for ( int j = 0; j < 6; ++j )
                Sigma[i][j] = 0;

//            Sigma[i][i] = 0.0; // TODO   THIS IS ONLY FOR TESTING
        }

        mu[0] = 64;
        mu[1] = 64;

        Ct[0][0] = 1;
        Ct[1][1] = 1;
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
            predict(act, dt);        	
        }
        t = timestamp;

        final double[] bestMeas = new double[2];
        final double[] meas     = new double[2];
        double minDistance = Double.MAX_VALUE;
        for (BasicEvent event : in) {
            meas[0] = event.x;
            meas[1] = event.y;

            double distance = mahalanobisToMeasurement(meas);

            if (distance < minDistance) {
                minDistance = distance;
                bestMeas[0] = meas[0];
                bestMeas[1] = meas[1];
                // timestamp = event.timestamp;
            }
        }

        if (minDistance <= measurementThreshold) {
            correct(bestMeas);
        }

        return in;
    }
    
    /**
     * Compute S and S^{-1}.
     */
    protected void computeSInv()
    {
    	MatrixOps.multABAT( Ct, Sigma, S );
    	MatrixOps.increment( S, Qt );
    	MatrixOps.invert2by2Matrix( S, SInv );
    }

    public double mahalanobisToMeasurement( final double[] meas )
    {
    	computeSInv();
        MatrixOps.mult(Ct,mu,vk1);
        MatrixOps.subtract(meas,vk1,vk2); // vk2 = nu
        // mahalanobis = nuT * S^{-1} * nu
        MatrixOps.mult( SInv, vk2, vk1 );
        return MatrixOps.dot( vk2, vk1 );
    }

    protected void predictMu( final double[] act ){ //act is m vector
    	MatrixOps.mult(At,mu,vn1);
        MatrixOps.mult(Bt,act,vn2);
        MatrixOps.add(vn1,vn2,mu);
    }

    protected void predictSigma(){
        MatrixOps.multABAT( At, Sigma, Mnn1 );
        MatrixOps.add( Mnn1, Rt, Sigma );
    }

    protected void updateKalmanGain()
    {        
        computeSInv();
    	MatrixOps.multABT( Sigma, Ct, Mnk1 );
        MatrixOps.mult( Mnk1, SInv, Kt );
    }

    protected void correctMu(double[] meas){ //meas is k vector
    	MatrixOps.mult(Ct,mu,vk1);
        MatrixOps.subtract(meas,vk1,vk2);
        MatrixOps.mult(Kt,vk2,vn1);
        MatrixOps.add(mu,vn1,mu);
    }

    protected void correctSigma(){
    	MatrixOps.multABAT( Kt, S, Mnn1 );
        MatrixOps.decrement( Sigma, Mnn1 );
    }

    public void updateFilter( final double[] act, final double[] meas, double dt ){

        predict(act, dt);
        correct(meas);
    }

    public void predict( final double[] act, double dt ) {

    	// System.out.println("predict (" + MatrixOps.toString( act ) + ", " + dt + ")" );
        updateAt(dt);
        updateBt(dt);
        updateRt(dt);
        // checkRtComputation();
        // System.out.println("At = \n" + MatrixOps.toString( At ) );
    	// System.out.println("Bt = \n" + MatrixOps.toString( Bt ) );
    	// System.out.println("Rt = \n" + MatrixOps.toString( Rt ) );
    	// System.out.println("mu = \n" + MatrixOps.toString( mu ) );
    	// System.out.println("Sigma = \n" + MatrixOps.toString( Sigma ) );
        predictMu(act);
        predictSigma();
    	// System.out.println("mu = \n" + MatrixOps.toString( mu ) );
    	// System.out.println("Sigma = \n" + MatrixOps.toString( Sigma ) );
    }

    public void correct( final double[] meas ) {

    	// System.out.println("correct (" + MatrixOps.toString( meas ) + ")" );
        updateKalmanGain();
    	// System.out.println("Kt = \n" + MatrixOps.toString( Kt ) );
    	// System.out.println("mu = \n" + MatrixOps.toString( mu ) );
    	// System.out.println("Sigma = \n" + MatrixOps.toString( Sigma ) );
        correctMu(meas);
        correctSigma();
    	// System.out.println("mu = \n" + MatrixOps.toString( mu ) );
    	// System.out.println("Sigma = \n" + MatrixOps.toString( Sigma ) );
    }

    public void updateAt(double dt){  /** Assuming At is initialized as double[6][6] */

        double a = 0.5*dt*dt;
        double b = dt;

        At[0][0] = 1; //constant
        At[0][2] = b;
        At[0][4] = a;
        At[1][1] = 1; //constant
        At[1][3] = b;
        At[1][5] = a;
        At[2][2] = 1; //constant
        At[2][4] = b;
        At[3][3] = 1; //constant
        At[3][5] = b;
        At[4][4] = 1; //constant
        At[5][5] = 1; //constant
    }

    public void updateBt(double dt){  /** Assuming Bt is initialized as double[6][2] */

        final double a = 0.5*dt*dt;
        final double b = dt;

        Bt[0][0] = a; //constant
        Bt[1][1] = a;
        Bt[2][0] = b;
        Bt[3][1] = b; //constant
        Bt[4][0] = 1;
        Bt[5][1] = 1;
    }


    public void updateRt(double dt){ /** Assuming Rt is initialized as double[6][6] */
        final double a = 0.5*dt*dt;
        final double b = dt;

        final double cov = processSigma*processSigma;
        final double acov = a*cov;
        final double bcov = b*cov;
        final double aacov = a*acov;
        final double abcov = a*bcov;
        final double bbcov = b*bcov;
        
        Rt[0][0] = aacov;
        Rt[0][2] = abcov;
        Rt[0][4] = acov;
        Rt[1][1] = aacov;
        Rt[1][3] = abcov;
        Rt[1][5] = acov;
        Rt[2][0] = abcov;
        Rt[2][2] = bbcov;
        Rt[2][4] = bcov;
        Rt[3][1] = abcov;
        Rt[3][3] = bbcov;
        Rt[3][5] = bcov;
        Rt[4][0] = acov;
        Rt[4][2] = bcov;
        Rt[4][4] = cov;
        Rt[5][1] = acov;
        Rt[5][3] = bcov;
        Rt[5][5] = cov;        
    }

    private void checkRtComputation()
    {
        final double cov = processSigma*processSigma;
	    double[][] procNoiseCov = new double[2][2];
	    procNoiseCov[0][0] = cov;
	    procNoiseCov[1][1] = cov;
	    double[][] t1 = new double[6][2];
	    MatrixOps.mult( Bt, procNoiseCov, t1 );
	    double[][] t2 = new double[6][6];
	    double[][] BtT = new double[2][6];
	    MatrixOps.transpose( Bt, BtT );
	    System.out.println( MatrixOps.toString( Bt ) );
	    System.out.println( MatrixOps.toString( BtT ) );
	    MatrixOps.mult( t1, BtT, t2 );
	    
	    boolean ok = true;
	    for ( int i = 0; i < 6; ++i )
	    {
	        for ( int j = 0; j < 6; ++j )
	        {
	        	double dsqu = ( Rt[i][j] - t2[i][j] ) * ( Rt[i][j] - t2[i][j] );
	        	if ( dsqu > 0.00000001 )
	        		ok = false;
	        }
	    }
	    
	    if (!ok) {
		    double[][] diff = new double[6][6];
		    MatrixOps.subtract( Rt, t2, diff );
		    System.out.println( "updateRt() is broken. diff to expected matrix is" );
		    System.out.println( MatrixOps.toString( diff ) );
	    }
	    else
	    {
		    System.out.println( "updateRt() is ok." );
	    }
    }
    
    

    @Override
    public void setAnnotationEnabled(boolean annotationEnabled) {
        this.annotationEnabled = annotationEnabled;
    }

    @Override
    public boolean isAnnotationEnabled() {
        return annotationEnabled;
    }

    @Override
    public void annotate(GLAutoDrawable drawable) {

        if(!isFilterEnabled())
            return;

        if (!annotationEnabled)
            return;

        GL gl=drawable.getGL();

        gl.glColor3f(1,1,0);
        gl.glLineWidth(2);

        int no_points_ellipse = 12;
        double nsigmas = 3;

        double a = Sigma[0][0];
        double b = Sigma[1][0];
        double c = Sigma[1][1];
        double d = Math.sqrt( a*a + 4*b*b - 2*a*c + c*c );
        double scale1 = Math.sqrt( 0.5 * ( a+c+d ) ) * nsigmas;
        double scale2 = Math.sqrt( 0.5 * ( a+c-d ) ) * nsigmas;
        double theta = 0.5 * Math.atan2( (2*b), (a-c) ) * 180.0 / Math.PI;

        gl.glPushMatrix();
        gl.glTranslated(mu[0], mu[1], 0);
        gl.glRotated(theta, 0, 0, 1);
        gl.glScaled(scale1, scale2, 1);
        gl.glBegin(GL.GL_LINE_LOOP);
        for (int i = 0;i<no_points_ellipse;i++){
            double cc = Math.cos(2*Math.PI*i/(double)no_points_ellipse);
            double ss = Math.sin(2*Math.PI*i/(double)no_points_ellipse);
            gl.glVertex2d(cc,ss);
        }
        gl.glEnd();
        gl.glPopMatrix();

        double velx = mu[2];
        double vely = mu[3];

        gl.glColor3f(1,0,0);
        gl.glLineWidth(4);

        gl.glPushMatrix();
        gl.glTranslated(mu[0], mu[1], 0);
        gl.glBegin(GL.GL_LINES);
        gl.glVertex2d(0, 0);
        gl.glVertex2d(velx, vely);
        gl.glEnd();
        gl.glPopMatrix();

        gl.glColor3f(1,1,1);
        gl.glBegin(GL.GL_LINES);
        gl.glVertex2d(64, 64);
        gl.glVertex2d(mu[0], mu[1]);
        gl.glEnd();
    }

    public Point2D.Float getBallPosition() {

        Point2D.Float position = new Point2D.Float();
        position.setLocation(mu[0], mu[1]);
        return position;
    }

    public double getBallPositionX() {
        return mu[0];
    }

    public double getBallPositionY() {
        return mu[1];
    }
    public Point2D.Float getBallVelocity() {

        Point2D.Float position = new Point2D.Float();
        position.setLocation(mu[2], mu[3]);
        return position;
    }

//    public void setBallPosition(Point2D.Float position) {
//        initFilter();
//        mu[0] = position.getX();
//        mu[1] = position.getY();
//    }
}
