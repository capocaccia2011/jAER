/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

/*
 * TrackEditor.java
 *
 * Created on Jun 11, 2010, 11:48:52 PM
 */

package ch.unizh.ini.jaer.projects.virtualslotcar;

import java.awt.Color;
import java.awt.geom.Point2D;
import java.util.LinkedList;
import java.util.ListIterator;
import java.awt.Graphics;
import java.awt.Dimension;

/**
 *
 * @author Michael Pfeiffer
 */
public class TrackEditor extends javax.swing.JPanel {

    /** The track that is drawn */
    private SlotcarTrack newTrack = null;

    /** The spline for interpolating between points of the track */
    private PeriodicSpline xySpline = null;
    
    /** The stepsize for interpolation */
    private double stepsize;

    /** Creates new form TrackEditor */
    public TrackEditor() {
        initComponents();
        stepsize = 0.05;
    }

    /** Sets a new Track to paint */
    public void setTrack(SlotcarTrack track) {
        newTrack = track;
    }

    /** This method is called from within the constructor to
     * initialize the form.
     * WARNING: Do NOT modify this code. The content of this method is
     * always regenerated by the Form Editor.
     */
    @SuppressWarnings("unchecked")
    // <editor-fold defaultstate="collapsed" desc="Generated Code">//GEN-BEGIN:initComponents
    private void initComponents() {

        setBorder(javax.swing.BorderFactory.createTitledBorder("Track"));

        javax.swing.GroupLayout layout = new javax.swing.GroupLayout(this);
        this.setLayout(layout);
        layout.setHorizontalGroup(
            layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGap(0, 388, Short.MAX_VALUE)
        );
        layout.setVerticalGroup(
            layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGap(0, 273, Short.MAX_VALUE)
        );
    }// </editor-fold>//GEN-END:initComponents

    /** Paint the race track */
    @Override
    public void paint(Graphics g) {
        super.paint(g);

        if (newTrack != null) {
            Dimension d = getSize();

            if (newTrack.getNumPoints() > 0) {
                ListIterator<Point2D> it = newTrack.getIterator();

                int oldX = -1;
                int oldY = -1;
                while (it.hasNext()) {
                    g.setColor(Color.black);
                    Point2D p = it.next();
                    int x = (int) (p.getX() * d.width);
                    int y = (int) ((1-p.getY()) * d.height);
                    if (oldX == -1)
                        g.setColor(Color.red);
                    g.drawOval(x-2, y-2, 5, 5);
                    if (oldX >= 0) {
                        g.drawLine(oldX, oldY, x, y);
                    }
                    oldX = x;
                    oldY = y;
                }
            }

            if (newTrack.getNumPoints() > 2) {
                LinkedList<Point2D> allpoints = newTrack.getSmoothPoints(stepsize);

                ListIterator<Point2D> all_it = allpoints.listIterator();
                int oldX = -1;
                int oldY = -1;
                g.setColor(Color.blue);
                while (all_it.hasNext()) {
                    Point2D p = all_it.next();
                    int x = (int) (p.getX() * d.width);
                    int y = (int) ((1-p.getY()) * d.height);
                    if (oldX >= 0) {
                        g.drawLine(oldX, oldY, x, y);
                    }
                    oldX = x;
                    oldY = y;
                }
            }
        }

    }
    
    public void setStepSize(double stepsize) {
        this.stepsize = stepsize;
        repaint();
    }


    // Variables declaration - do not modify//GEN-BEGIN:variables
    // End of variables declaration//GEN-END:variables

}
