/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package org.capocaccia.cne.jaer.cne2011;

import java.awt.geom.Point2D;
import net.sf.jaer.chip.AEChip;
import net.sf.jaer.event.*;

import javax.media.opengl.GL;
import javax.media.opengl.GLAutoDrawable;
import net.sf.jaer.eventprocessing.EventFilter2D;
import net.sf.jaer.graphics.FrameAnnotater;

/**
 *
 * @author Eero
 */
public class KalmanFilter extends EventFilter2D implements FrameAnnotater {

    /* Kalman filter parameters:*/
    protected double[][] At;
    protected double[][] AtT;
    protected double[][] Bt;
    protected double[][] Ct;
    protected double[][] CtT;

    protected double[][] mu;
    protected double[][] Sigma;

    protected double[][] Kt;
    protected double sigma_epsilon;
    protected double sigma_delta;
    protected double[][] Rt;
    protected double[][] Qt;

    // the timestamp of the most recent received event
    private int t = -1;

    /* Auxiliary matrices used for intermediate results:*/
    protected double[][] Mnn1; //n*n, i.e., the size of At
    protected double[][] Mnn2; //n*n, i.e., the size of At
    protected double[][] Mnk1; //n*k, i.e., the size of Kt and CtT
    protected double[][] Mkk1; //k*k, i.e., the size of Qt
    protected double[][] Mkk2; //k*k, i.e., the size of Qt

    protected double[][] vn1; //n*1, i.e., the size of mu
    protected double[][] vn2; //n*1, i.e., the size of mu
    protected double[][] vk1; //k*1, i.e., the size of meas
    protected double[][] vk2; //k*1, i.e., the size of meas

    /*Auxiliary matrices used for annotating:*/
    double[][] auxvec1 = new double[2][1];
    double[][] auxmatr = new double[2][2];
    double[][] auxvec2 = new double[2][1];

    // parameters
    private double measurementSigma = getPrefs().getDouble("KalmanFilter.measurementSigma", 2.0);

    public KalmanFilter(AEChip chip) {

        super(chip);

        At = new double[6][6];
        AtT = new double[6][6];
        Bt = new double[6][2];
        Ct = new double[2][6];
        CtT = new double[6][2];
        Kt = new double[6][2];

        Qt = new double[2][2];
        Rt = new double[6][6];
        
        mu = new double[6][1];
        Sigma = new double[6][6];

        vn1 = new double[6][1];
        vn2 = new double[6][1];

        Mnn1 = new double[6][6];
        Mnn2 = new double[6][6];
        Mnk1 = new double[6][2];
        Mkk1 = new double[2][2];
        Mkk2 = new double[2][2];

        resetFilter();
    }

    public double getMeasurementSigma() {
        return measurementSigma;
    }
    synchronized public void setMeasurementSigma(double measurementSigma) {

        if(measurementSigma < 0) measurementSigma = 0;
        getPrefs().putDouble("KalmanFilter.measurementSigma", measurementSigma);

        if(measurementSigma != this.measurementSigma) {
            this.measurementSigma = measurementSigma;
            resetFilter();
        }
    }

    @Override
    final public void resetFilter()
    {
        Qt[0][0] = measurementSigma;
        Qt[1][1] = measurementSigma;

        for ( int i = 0; i < 6; ++i )
        {
            mu[i][0] = 0;
            for ( int j = 0; j < 6; ++j )
                Sigma[i][j] = 0;

            Sigma[i][i] = 0.1; // TODO   THIS IS ONLY FOR TESTING 
        }
        
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

        if (t < 0)
            t = in.getFirstTimestamp();

        for (BasicEvent event : in) {

            double[][] meas = new double[2][1];
            meas[0][0] = event.x;
            meas[1][0] = event.y;

            // TODO: get the performed actions from the controller
            double[][] act = new double[2][1];

            double dt = (double)(event.timestamp - t)/100.0;
            t = event.timestamp;

            updateFilter(act, meas, dt);
        }

        return in;
    }

    protected void predictMu(double[][] act){ //act is m*1 matrix
        matrixMultiplication(At,mu,vn1);
        matrixMultiplication(Bt,act,vn2);
        matrixSum(vn1,vn2,mu);
    }

    protected void predictSigma(){
        matrixMultiplication(At, Sigma, Mnn1);
        matrixMultiplication(Mnn1,AtT,Mnn2);
        matrixSum(Mnn2,Rt,Sigma);
    }

    protected void updateKalmanGain(){
        matrixMultiplication(Sigma, CtT, Mnk1);
        matrixMultiplication(Ct,Mnk1,Mkk1);
        matrixSum(Mkk1,Qt,Mkk2);
        invert2by2Matrix(Mkk2,Mkk1); //assuming M2 is a 2*2 matrix
        matrixMultiplication(Mnk1,Mkk1,Kt);
    }

    protected void correctMu(double[][] meas){ //meas is k*1 matrix
        matrixMultiplication(Ct,mu,vk1);
        matrixSubstraction(meas,vk1,vk2);
        matrixMultiplication(Kt,vk2,vn1);
        matrixSum(mu,vn1,mu);
    }

    protected void correctSigma(){
        matrixMultiplication(Kt,Ct,Mnn1);
        matrixMultiplication(Mnn1,Sigma,Mnn2);
        matrixSubstraction(Sigma,Mnn2,Sigma);
    }

    public void updateFilter(double[][] act, double[][] meas, double dt){

        predict(act, dt);
        correct(meas);
    }

    public void predict(double[][] act, double dt) {

        System.out.println("predict (" + matrixToString( act ) + ", " + dt + ")" );
        updateAt(dt);
        updateBt(dt);
        updateRt(dt);
        System.out.println("At = \n" + matrixToString( At ) );
        System.out.println("Bt = \n" + matrixToString( Bt ) );
        System.out.println("Rt = \n" + matrixToString( Rt ) );
        predictMu(act);
        predictSigma();
    }

    public void correct(double[][] meas) {

        updateKalmanGain();
        correctMu(meas);
        correctSigma();
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

        transposeMatrix(At, AtT);
    }

    public void updateBt(double dt){  /** Assuming Bt is initialized as double[6][2] */

        double a = 0.5*dt*dt;
        double b = dt;

        Bt[0][0] = a; //constant
        Bt[1][1] = a;
        Bt[2][0] = b;
        Bt[3][1] = b; //constant
        Bt[4][0] = 1;
        Bt[5][1] = 1;
    }


    public void updateRt(double dt){ /** Assuming Rt is initialized as double[6][6] */
        double a = 0.5*dt*dt;
        double b = dt;

        Rt[0][0] = a*a;
        Rt[0][2] = a*b;
        Rt[0][4] = a;
        Rt[1][1] = a*a;
        Rt[1][3] = a*b;
        Rt[1][5] = a;
        Rt[2][0] = a*b;
        Rt[2][2] = b*b;
        Rt[2][4] = b;
        Rt[3][1] = a*b;
        Rt[3][3] = b*b;
        Rt[3][5] = b;
        Rt[4][0] = a;
        Rt[4][2] = b;
        Rt[4][4] = 1; //constant
        Rt[5][1] = a;
        Rt[5][3] = b;
        Rt[5][5] = 1; //constant
    }

    public static void matrixCopy(double[][] A, double[][] R){

        int Arow = A[0].length;
        int Acol = A.length;

        for(int i=0; i<Acol; i++){
            for(int j=0; j<Arow; j++){
                R[i][j] = A[i][j];
            }
        }
    }

    public static void matrixSum(double[][] A, double[][] B, double[][] R){ /**
                                                                             * result: R=A+B
     */
        int Arow = A[0].length;
        int Acol = A.length;

        for(int i=0; i<Acol; i++){
            for(int j=0; j<Arow; j++){
                R[i][j] = A[i][j]+B[i][j];
            }
        }
    }

    public static void matrixSubstraction(double[][] A, double[][] B, double[][] R){ /**
                                                                             * result: R=A-B
     */
        int Arow = A[0].length;
        int Acol = A.length;

        for(int i=0; i<Acol; i++){
            for(int j=0; j<Arow; j++){
                R[i][j] = A[i][j]-B[i][j];
            }
        }
    }


    public static void matrixMultiplication(double[][] A, double[][] B, double[][] R){ /**
        result: R=A*B */
        int Arow = A[0].length;
        int Acol = A.length;

    int Brow = B[0].length;
//    int Bcol = B.length;

        for(int i=0; i<Acol; i++){
            for(int j=0; j<Brow; j++){
                for(int k=0; k<Arow; k++){
                    R[i][j] += A[i][k]*B[k][j];
                }
            }
        }
    }

    public static void upperTriangularMatrixMultiplication(double[][] A, double[][] B, double[][] R){ /**
    A is an upper triangular matrix, result: R=A*B */
    int Arow = A[0].length;
    int Acol = A.length;

    int Brow = B[0].length;
//    int Bcol = B.length;

    for(int i=0; i<Acol; i++){
        for(int j=0; j<Brow; j++){
            for(int k=i; k<Arow; k++){
                R[i][j] += A[i][k]*B[k][j];
            }
        }
    }
}

    public static void lowerTriangularMatrixMultiplication(double[][] A, double[][] B, double[][] R){ /**
    B is a lower triangular matrix, result: R=A*B */
    int Arow = A[0].length;
    int Acol = A.length;

    int Brow = B[0].length;
    int Bcol = B.length;

    for(int i=0; i<Acol; i++){
        for(int j=0; j<Brow; j++){
            for(int k=0; k<=i; k++){
                R[i][j] += A[i][k]*B[k][j];
            }
        }
    }
}

    public static void transposeMatrix(double[][] A, double[][] R){/** result: R = transpose(A)*/
        int Arow = A[0].length;
        int Acol = A.length;

        for(int i=0; i<Acol; i++){
            for(int j=0; j<Arow; j++){
                R[i][j] = A[j][i];
            }
        }
    }

    public static void invert2by2Matrix(double[][] A, double[][] R){/**
                                                                 * A is 2 by 2 matrix,
                                                                 * result: R = inv (A) Ü
                                                                 * if det(A) != 0, else R = 0
     */
    double detA = A[0][0]*A[1][1]-A[0][1]*A[1][0];
    if(detA == 0){
         R[0][0] = 0;
         R[0][1] = 0;
         R[1][0] = 0;
         R[1][1] = 0;
    }
    else{
    R[0][0] = (1/detA)*A[1][1];
    R[0][1] = -(1/detA)*A[0][1];
    R[1][0] = -(1/detA)*A[1][0];
    R[1][1] = (1/detA)*A[0][0];
    }
    }

    public static String matrixToString( double[][] m )
    {
        final int rows = m.length;
        final int cols = m[0].length;
        
        String result = "";
        for ( int i = 0; i < rows; ++i )
        {
            for ( int j = 0; j < cols; ++j )
            {
                result += String.format( "%6.3f ", m[i][j] );                
            }
            result += "\n";
        }
        return result;
    }
    
    @Override
    public void setAnnotationEnabled(boolean yes) {
        throw new UnsupportedOperationException("Not supported yet.");
    }

    @Override
    public boolean isAnnotationEnabled() {
        throw new UnsupportedOperationException("Not supported yet.");
    }

    @Override
    public void annotate(GLAutoDrawable drawable) {

        if(!isFilterEnabled())
            return;

        GL gl=drawable.getGL();

        gl.glColor3f(1,1,0);
        gl.glLineWidth(2);

        int no_points_ellipse = 12;
        double ellipse_radius = 4;


        auxmatr[0][0] = Sigma[0][0];
        auxmatr[0][1] = Sigma[0][1];
        auxmatr[1][0] = Sigma[1][0];
        auxmatr[1][1] = Sigma[1][1];

        gl.glBegin(GL.GL_LINE_LOOP);
        for (int i = 0;i<no_points_ellipse;i++){
            auxvec1[0][0] = Math.cos(2*Math.PI*i/no_points_ellipse);
            auxvec1[1][0] = Math.sin(2*Math.PI*i/no_points_ellipse);
            matrixMultiplication(auxmatr,auxvec1,auxvec2);

            gl.glVertex2d(
                    mu[0][0] + ellipse_radius*auxvec2[0][0],
                    mu[1][0] + ellipse_radius*auxvec2[0][1]);
        }
        gl.glEnd();

        double velx = mu[2][0];
        double vely = mu[3][0];

        double speed = Math.sqrt(velx*velx+vely*vely);
        double direction = Math.atan(vely/velx);


        gl.glColor3f(1,0,0);
        gl.glLineWidth(4);

        gl.glBegin(GL.GL_LINE_LOOP);
        gl.glVertex2d(mu[0][0],mu[1][0]);
        gl.glVertex2d(mu[0][0] + speed*Math.cos(direction),
                      mu[1][0] + speed*Math.sin(direction));

        gl.glEnd();



    }

    public Point2D.Float getBallPosition() {

        Point2D.Float position = new Point2D.Float();
        position.setLocation(mu[0][0], mu[1][0]);
        return position;
    }

    public double getBallPositionX() {
        return mu[0][0];
    }

    public double getBallPositionY() {
        return mu[1][0];
    }
    public Point2D.Float getBallVelocity() {

        Point2D.Float position = new Point2D.Float();
        position.setLocation(mu[2][0], mu[3][0]);
        return position;
    }

    public void setBallPosition(Point2D.Float position) {
        initFilter();
        mu[0][0] = position.getX();
        mu[1][0] = position.getY();
    }
}
