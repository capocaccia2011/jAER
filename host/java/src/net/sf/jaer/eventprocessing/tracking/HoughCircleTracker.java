/*
 * HoughCircleTracker.java
 *
 * Created May 07 2011 by Jan Funke
 * inspired by HoughEyeTracker.java
 */

package net.sf.jaer.eventprocessing.tracking;

import net.sf.jaer.chip.*;
import net.sf.jaer.event.*;
import net.sf.jaer.eventprocessing.EventFilter2D;
import java.util.*;
import net.sf.jaer.eventprocessing.EventFilterDataLogger;

import net.sf.jaer.graphics.FrameAnnotater;
import java.awt.Graphics2D;
import javax.media.opengl.*;
import java.awt.*;
import java.awt.event.*;

import javax.swing.*;

/**
 * A simple circle tracker based on a hough transform.
 * @author Jan Funke
 */
public class HoughCircleTracker extends EventFilter2D implements FrameAnnotater, Observer {

	// the Hough space
	int cameraX;
	int cameraY;
	int[][] accumulatorArray;

	// history of the encountered spikes to remove the least recent one from
	// hough space
	Coordinate[] eventHistory;
	int bufferIndex = 0;

	// the running value of the current maximum in Hough space
	int maxValue =0;

	// the running maximum in Hough space
	Coordinate maxCoordinate = new Coordinate();

	// visualisation stuff
	int angleListLength = 18;
	float[] sinTau = new float[angleListLength];
	float[] cosTau = new float[angleListLength];
	Coordinate[] circleOutline = new Coordinate[angleListLength];

	EventFilterDataLogger dataLogger;
	JFrame                targetFrame;
	DrawPanel             gazePanel;

	// parameters
	private float   radius         = getPrefs().getFloat("HoughCircleTracker.radius", 12.5f);
	private int     bufferLength   = getPrefs().getInt("HoughCircleTracker.bufferLength", 200);
	private int     threshold      = getPrefs().getInt("HoughCircleTracker.threshold", 30);
	private boolean logDataEnabled = false;


	public boolean isGeneratingFilter() {
		return false;
	}

	public HoughCircleTracker(AEChip chip) {
		super(chip);
		chip.addObserver(this);
		resetFilter();
	}

	public Object getFilterState() {
		return null;
	}

	public void resetFilter() {
		initTracker();
	}

	final class Coordinate {

		public float x, y;

		Coordinate(){
			this.x = 0;
			this.y = 0;
		}

		Coordinate(float x, float y){
			this.x = x;
			this.y = y;
		}

		public void setCoordinate(float x, float y){
			this.x = x;
			this.y = y;
		}
	}

	synchronized private void initTracker() {

		System.out.println("HoughCircleTracker initialising...");

		accumulatorArray = new int[chip.getSizeX()][chip.getSizeY()];

		if(chip.getSizeX()==0 || chip.getSizeY()==0){
			return;
		}

		cameraX = chip.getSizeX();
		cameraY = chip.getSizeY();

		for(int i=0;i<chip.getSizeX();i++){
			for(int j=0; j < chip.getSizeY();j++){
				accumulatorArray[i][j]=0;
			}
		}

		eventHistory = new Coordinate[bufferLength];
		for(int i=0;i<bufferLength;i++)
			eventHistory[i] = null;

		bufferIndex = 0;
		maxValue    = 0;

		for (int i = 0;i<angleListLength;i++){
			sinTau[i] = (float)(Math.sin(2*Math.PI/angleListLength*i));
			cosTau[i] = (float)(Math.cos(2*Math.PI/angleListLength*i));
			circleOutline[i] = new Coordinate(radius*cosTau[i],radius*sinTau[i]);
		}
	}

	public void initFilter() {
		resetFilter();
	}

	public void update(Observable o, Object arg) {
		if(!isFilterEnabled()) return;
		initFilter();
	}

	public float getRadius() {
		return radius;
	}
	synchronized public void setRadius(float radius) {

		if(radius < 0) radius = 0;
		getPrefs().putFloat("HoughCircleTracker.radius", radius);

		if(radius != this.radius) {
			resetFilter();
		}

		this.radius = radius;
	}

	public int getBufferLength() {
		return bufferLength;
	}

	synchronized public void setBufferLength(int bufferLength) {

		if(bufferLength < 0)
			bufferLength=0;

		this.bufferLength = bufferLength;

		getPrefs().putInt("HoughCircleTracker.bufferLength", bufferLength);
		resetFilter();
	}

	public int getThreshold() {
		return threshold;
	}

	synchronized public void setThreshold(int threshold) {

		if(threshold < 0)
			threshold=0;

		getPrefs().putInt("HoughCircleTracker.threshold", threshold);
		this.threshold = threshold;
	}

	public void annotate(float[][][] frame) {
	}

	public void annotate(Graphics2D g) {
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

	// fast inclined ellipse drawing algorithm; ellipse eqn: A*x^2+B*y^2+C*x*y-1 = 0
	// the algorithm is fast because it uses just integer addition and subtraction
	void accumulate(Coordinate event, int weight){

		// TODO: this is a little overhead here, since we only draw circles in
		// Hough space (not ellipses)
		int centerX = (int)event.x;
		int centerY = (int)event.y;
		int aa      = Math.round(radius*radius);
		int bb      = aa;
		int twoC    = 0;

		int x = 0;
		int y = Math.round((float)Math.sqrt(bb));
		int twoaa = 2*aa;
		int twobb = 2*bb;
		int dx =   twoaa*y + twoC*x;   //slope =dy/dx
		int dy = -(twobb*x + twoC*y);
		int ellipseError = aa*(y*y-bb);

// first sector: (dy/dx > 1) -> y+1 (x+1)
// d(x,y+1)   = 2a^2y+a^2+2cx				= dx+aa
// d(x+1,y+1) = 2b^2x+b^2+2cy+2c+2a^2y+a^2+2cx = d(x,y+1)-dy+bb
		while (dy > dx){
			increaseHoughPoint(centerX+x,centerY+y,weight);
			increaseHoughPoint(centerX-x,centerY-y,weight);
			ellipseError = ellipseError + dx + aa;
			dx = dx + twoaa;
			dy = dy - twoC;
			y = y + 1;
			if (2*ellipseError-dy+bb > 0) {
				ellipseError = ellipseError - dy + bb ;
				dx = dx + twoC;
				dy = dy - twobb;
				x = x + 1;
			}
		}

// second sector: (dy/dx > 0) -> x+1 (y+1)
// d(x+1,y)   = 2b^2x+b^2+2cy				= -dy+bb
// d(x+1,y+1) = 2b^2x+b^2+2cy+2c+2a^2y+a^2+2cx = d(x+1,y)+dx+aa
		while (dy > 0){
			increaseHoughPoint(centerX+x,centerY+y,weight);
			increaseHoughPoint(centerX-x,centerY-y,weight);
			ellipseError = ellipseError - dy + bb;
			dx = dx + twoC;
			dy = dy - twobb;
			x = x + 1;
			if (2*ellipseError + dx + aa < 0){
				ellipseError = ellipseError + dx + aa ;
				dx = dx + twoaa;
				dy = dy - twoC;
				y = y + 1;
			}
		}

// third sector: (dy/dx > -1) -> x+1 (y-1)
// d(x+1,y)   = 2b^2x+b^2+2cy				= -dy+bb
// d(x+1,y-1) = 2b^2x+b^2+2cy-2c-2a^2y+a^2-2cx = d(x+1,y)-dx+aa
		while (dy > - dx){
			increaseHoughPoint(centerX+x,centerY+y,weight);
			increaseHoughPoint(centerX-x,centerY-y,weight);
			ellipseError = ellipseError - dy + bb;
			dx = dx + twoC;
			dy = dy - twobb;
			x = x + 1;
			if (2*ellipseError - dx + aa > 0){
				ellipseError = ellipseError - dx + aa;
				dx = dx - twoaa;
				dy = dy + twoC;
				y = y - 1;
			}
		}

// fourth sector: (dy/dx < 0) -> y-1 (x+1)
// d(x,y-1)   = -2a^2y+a^2-2cx			   = -dx+aa
// d(x+1,y-1) = 2b^2x+b^2+2cy-2c-2a^2y+a^2-2cx = d(x+1,y)-dy+bb
		while (dx > 0){
			increaseHoughPoint(centerX+x,centerY+y,weight);
			increaseHoughPoint(centerX-x,centerY-y,weight);
			ellipseError = ellipseError - dx + aa;
			dx = dx - twoaa;
			dy = dy + twoC;
			y = y - 1;
			if (2*ellipseError - dy + bb < 0){
				ellipseError = ellipseError - dy + bb;
				dx = dx + twoC;
				dy = dy - twobb;
				x = x + 1;
			}
		}

//fifth sector (dy/dx > 1) -> y-1 (x-1)
// d(x,y-1)   = -2a^2y+a^2-2cx				= -dx+aa
// d(x-1,y-1) = -2b^2x+b^2-2cy+2c-2a^2y+a^2-2cx = d(x+1,y)+dy+bb
		while ((dy < dx)&& (x > 0)){
			increaseHoughPoint(centerX+x,centerY+y,weight);
			increaseHoughPoint(centerX-x,centerY-y,weight);
			ellipseError = ellipseError - dx + aa;
			dx = dx - twoaa;
			dy = dy + twoC;
			y = y - 1;
			if (2*ellipseError + dy + bb > 0){
				ellipseError = ellipseError  + dy + bb;
				dx = dx - twoC;
				dy = dy + twobb;
				x = x - 1;
			}
		}

// sixth sector: (dy/dx > 0) -> x-1 (y-1)
// d(x-1,y)   = -2b^2x+b^2-2cy				= dy+bb
// d(x-1,y-1) = -2b^2x+b^2-2cy+2c-2a^2y+a^2-2cx = d(x+1,y)-dx+aa
		while ((dy < 0)&& (x > 0)){
			increaseHoughPoint(centerX+x,centerY+y,weight);
			increaseHoughPoint(centerX-x,centerY-y,weight);
			ellipseError = ellipseError + dy + bb;
			dx = dx - twoC;
			dy = dy + twobb;
			x = x - 1;
			if (2*ellipseError - dx + aa < 0){
				ellipseError = ellipseError  - dx + aa;
				dx = dx - twoaa;
				dy = dy + twoC;
				y = y - 1;
			}
		}

// seventh sector: (dy/dx > -1) -> x-1 (y+1)
// d(x-1,y)   = -2b^2x+b^2-2cy				= dy+bb
// d(x-1,y+1) = -2b^2x+b^2-2cy-2c+2a^2y+a^2+2cx = d(x+1,y)-dx+aa
		while ((dy < - dx)&& (x > 0)){
			increaseHoughPoint(centerX+x,centerY+y,weight);
			increaseHoughPoint(centerX-x,centerY-y,weight);
			ellipseError = ellipseError + dy + bb;
			dx = dx - twoC;
			dy = dy + twobb;
			x = x - 1;
			if (2*ellipseError + dx + aa > 0){
				ellipseError = ellipseError  + dx + aa;
				dx = dx + twoaa;
				dy = dy - twoC;
				y = y + 1;
			}
		}

// eight sector: (dy/dx < 0) -> y+1 (x-1)
// d(x,y+1)   = 2a^2y+a^2+2cx				 = dx+aa
// d(x-1,y+1) = -2b^2x+b^2-2cy-2c+2a^2y+a^2+2cx = d(x,y+1)+dy+bb
		while ((dy > 0 && dx < 0)&& (x > 0)){
			increaseHoughPoint(centerX+x,centerY+y,weight);
			increaseHoughPoint(centerX-x,centerY-y,weight);
			ellipseError = ellipseError + dx + aa;
			dx = dx + twoaa;
			dy = dy - twoC;
			y = y + 1;
			if (2*ellipseError + dy + bb < 0){
				ellipseError = ellipseError + dy + bb ;
				dx = dx - twoC;
				dy = dy + twobb;
				x = x - 1;
			}
		}
	}

	void increaseHoughPoint(int x, int y, int weight) {

		if (x < 0 || x > chip.getSizeX() - 1 || y < 0 || y > chip.getSizeY() - 1)
			return;

		// increase the value of the hough point
		accumulatorArray[x][y] = accumulatorArray[x][y] + weight;

		// check if this is a new maximum
		if (accumulatorArray[x][y] >= maxValue) {

			maxValue = accumulatorArray[x][y];

			if (maxValue > threshold)
				maxCoordinate.setCoordinate(x,y);
		}
	}

	synchronized public EventPacket<?> filterPacket(EventPacket<?> in) {

		if (!isFilterEnabled())
			return in;

		if (in == null || in.getSize() == 0)
			return in;

		for (BasicEvent event : in) {

			if (event.x < 0 || event.x > chip.getSizeX() - 1 || event.y < 0 || event.y > chip.getSizeY() - 1)
				continue;

			// save event in history
			eventHistory[bufferIndex] = new Coordinate(event.x, event.y);

			// accumulate all possible circle centers for the current event
			accumulate(eventHistory[bufferIndex], 1);

			// increase buffer index
			bufferIndex = (bufferIndex+1)%bufferLength;

			// remove the least recent event from hough space
			if(eventHistory[bufferIndex] != null)
				accumulate(eventHistory[bufferIndex], -1);
		}

		maxValue = 0;
		for (int x = 0; x < cameraX; x++) {
			for (int y = 0; y < cameraY; y++) {
				if (accumulatorArray[x][y] > maxValue) {
					maxValue = accumulatorArray[x][y];
					maxCoordinate.x = x;
					maxCoordinate.y = y;
				}
			}
		}

		OutputEventIterator itr = out.outputIterator();
		BasicEvent outEvent = itr.nextOutput();
		outEvent.x = (short)maxCoordinate.x;
		outEvent.y = (short)maxCoordinate.y;

		// pass events unchanged to next filter
		return in;
	}

	synchronized public boolean isLogDataEnabled() {
		return logDataEnabled;
	}

	synchronized public void setLogDataEnabled(boolean logDataEnabled) {

		this.logDataEnabled = logDataEnabled;

		if(dataLogger == null)
			dataLogger = new EventFilterDataLogger(this,"# x y");

		dataLogger.setEnabled(logDataEnabled);

		if(logDataEnabled){

			targetFrame = new JFrame("EyeTargget");
			gazePanel   = new DrawPanel();

			targetFrame.setLocation( 0, 0 );
			targetFrame.setSize( Toolkit.getDefaultToolkit().getScreenSize() );
			targetFrame.add( gazePanel );
			targetFrame.setVisible(true);
			targetFrame.addKeyListener( new KeyListener() {
				public void keyTyped( KeyEvent e ) {
//					System.out.println( "typed " + e.getKeyChar() );
//					System.out.println( "typed " + e.getKeyCode() );
					gazePanel.newPosition();
				}
				public void keyPressed( KeyEvent e ) {}
				public void keyReleased( KeyEvent e ) {}
			});

		}else
			targetFrame.setVisible(false);
	}

	@SuppressWarnings("serial")
	class DrawPanel extends JPanel {

		int width  = targetFrame.getSize().width;
		int height = targetFrame.getSize().height;
		int x = 50;
		int y = (int)(height/2);
		int count = 0;
		int w = 1;

		@Override
		protected void paintComponent( Graphics g ) {

			width  = targetFrame.getSize().width;
			height = targetFrame.getSize().height;
			x = 50 + (int)(count*(width-100)/2);;
			y = 50 + (int)(count*(height-100)/2);
			super.paintComponent(g);
			g.fillRect(x,y,10,10);
		}

		public void newPosition() {

			if (isLogDataEnabled())
				dataLogger.log(String.format("%d %d %f %f", maxCoordinate.x, maxCoordinate.y));

			count = (count+w)%3;
			if (count>1) w=-1;
			if (count<1) w=+1;

			repaint();
		}
	}
}
