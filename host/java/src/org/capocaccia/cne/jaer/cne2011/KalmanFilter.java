/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package org.capocaccia.cne.jaer.cne2011;

import java.util.Observable;
import java.util.Observer;
import javax.media.opengl.GLAutoDrawable;
import net.sf.jaer.graphics.FrameAnnotater;

/**
 *
 * @author Eero
 */
public class KalmanFilter extends EventFilter2D implements FrameAnnotater, Observer{
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

    //protected double[][] Rt;
    protected double[][] Qt;

    /* Auxiliary matrices used for intermediate results:*/
    protected double[][] Mnn1; //n*n, i.e., the size of At
    protected double[][] Mnn2; //n*n, i.e., the size of At
    protected double[][] Mnk1; //n*k, i.e., the size of Kt and CtT
    protected double[][] Mnk2; //n*k, i.e., the size of Kt and CtT
    protected double[][] Mkk1; //k*k, i.e., the size of Qt
    protected double[][] Mkk2; //k*k, i.e., the size of Qt

    protected double[][] vn1; //n*1, i.e., the size of mu
    protected double[][] vn2; //n*1, i.e., the size of mu
    protected double[][] vk1; //k*1, i.e., the size of meas
    protected double[][] vk2; //k*1, i.e., the size of meas

    // parameters
    private double measurementSigma = getPrefs().getFloat("KalmanFilter.measurementSigma", 2.0);

    public KalmanFilter() {
        At = new double[6][6];
        Bt = new double[6][2];
        Kt = new double[6][2];
        mu = new double[6];
        Sigma = new double[6][6];

        resetFilter();
    }

    public KalmanFilter(double[][] At, double[][] Bt, double[][] Ct, double[][] mu, double[][] Sigma){
        this.At = At;
        this.Bt = Bt;
        this.Ct = Ct;
        this.mu = mu;
        this.Sigma = Sigma;

        resetFilter();
    }

    public KalmanFilter(){}

    public double getMeasurementSigma() {
        return measurementSigma;
    }
    synchronized public void setMeasurementSigma(double measurementSigma) {

        if(measurementSigma < 0) measurementSigma = 0;
        getPrefs().putFloat("KalmanFilter.measurementSigma", measurementSigma);

        if(measurementSigma != this.measurementSigma) {
            resetFilter();
        }

        this.measurementSigma = measurementSigma;
    }

    @Override
    public void resetFilter() {

        transposeMatrix(At, AtT);
        transposeMatrix(Bt, BtT);
        transposeMatrix(Ct, CtT);

        Qt = new double[2][2];
        Qt[0][0] = measurementSigma;
        Qt[1][1] = measurementSigma;
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

        for (BasicEvent event : in) {

            double[][] meas = new double[2][1];
            meas[0][0] = event.x;
            meas[1][0] = event.y;

            correct(meas);
        }

        return in; // only handles control commands, no event processing
    }

    public void predictMu(double[][] act){ //act is m*1 matrix
        matrixMultiplication(At,mu,vn1);
        matrixMultiplication(Bt,act,vn2);
        matrixSum(vn1,vn2,muBel);
    }

    public void predictSigma(double[][] Rt){
        matrixMultiplication(At, Sigma, Mnn1);
        matrixMultiplication(Mnn1,AtT,Mnn2);
        matrixSum(Mnn2,Rt,SigmaBel);
    }

    public void updateKalmanGain(double[][] Qt){
        matrixMultiplication(SigmaBel, CtT, Mnk1);
        matrixMultiplication(Ct,Mnk1,Mkk1);
        matrixSum(Mkk1,Qt,Mkk2);
        invert2by2Matrix(Mkk2,Mkk1); //assuming M2 is a 2*2 matrix
        matrixMultiplication(Mnk1,Mkk1,Kt);
    }

    public void correctMu(double[][] meas){ //meas is k*1 matrix
        matrixMultiplication(Ct,muBel,vk1);
        matrixSubstraction(meas,vk1,vk2);
        matrixMultiplication(Kt,vk2,vn1);
        matrixSum(muBel,vn1,mu);
    }

    public void correctSigma(){
        matrixMultiplication(Kt,Ct,Mnn1);
        matrixMultiplication(Mnn1,SigmaBel,Mnn2);
        matrixSubstraction(SigmaBel,Mnn2,Sigma);
    }

    public void updateFilter(double[][] act, double[][] meas, double dt, double[][] Rt, double[][] Qt){

        // TODO: compute Rt from dt
        predict(act, Rt, dt);
        update(meas, Qt);
    }

    public void predict(double[][] act, double[][] Rt, double dt) {

        predictMu(act);
        predictSigma(Rt);
    }

    public void correct(double[][] meas) {
        updateKalmanGain(Qt);
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
    int Bcol = B.length;

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
    int Bcol = B.length;

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

		// draw the Hough space
		//for (int x = 0; x < cameraX; x++) {
			//for (int y = 0; y < cameraY; y++) {

				//float red   = (float)accumulatorArray[x][y]/maxValue;
				//float green = 1.0f - red;

				//gl.glColor4f(red,green,0.0f,0.1f);
				//gl.glRectf(
						//(float)x-0.5f,
						//(float)y-0.5f,
						//(float)x+0.5f,
						//(float)y+0.5f);
			//}
		//}

		// draw the circle
		gl.glColor3f(1,0,0);
		gl.glLineWidth(2);

		gl.glBegin(GL.GL_LINE_LOOP);
		for (int i = 0;i<angleListLength;i++){
			gl.glVertex2d(
					circleOutline[i].x + maxCoordinate.x,
					circleOutline[i].y + maxCoordinate.y);
		}
		gl.glEnd();

    }

    @Override
    public void update(Observable o, Object arg) {
        throw new UnsupportedOperationException("Not supported yet.");
    }

}
