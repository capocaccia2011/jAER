/*
 * PotPanel.java
 *
 * Created on September 21, 2005, 11:18 AM
 */

package ch.unizh.ini.caviar.biasgen;

import java.awt.*;
import java.util.*;
import javax.swing.*;
import javax.swing.event.UndoableEditListener;
import javax.swing.table.*;

/**
 * Panel for controlling a chip's set of Pots over a HardwareInterface.
 * @author  tobi
 */
public class PotPanel extends javax.swing.JPanel  {
    
    public PotArray pots=null;
    BiasgenFrame frame;
    JScrollPane scrollPane=null;
    JPanel potsPanel;
    ArrayList<Pot> potList;
    ArrayList<JComponent> componentList;
    
    /**
     * Creates new form PotPanel
     */
    public PotPanel(PotArray ipotArray, BiasgenFrame frame) {
        this.frame=frame;
        pots=ipotArray;
        initComponents();
        getInsets().set(0, 0, 0, 0);
        buildPanel();
    }
    
//    JTable table;
//
//    class PotTable extends JTable{
//        private PotPanel panel;
//        private PotArray pots;
//        TableModel model;
//        public PotTable(TableModel m,PotPanel p){
//            super(m);
//            panel=p;
//            model=m;
//        }
//    }
    
    class MyRenderer implements TableCellRenderer{
        public java.awt.Component getTableCellRendererComponent(JTable jTable, Object obj, boolean param, boolean param3, int param4, int param5) {
            return null;
        }
        
    }
    
    /** builds the panel of pots */
    private void buildPanel() {
        IPotSliderTextControl.allInstances.clear();
        potList=new ArrayList<Pot>(pots.getPots());
        componentList=new ArrayList<JComponent>();
        Collections.sort(potList, new PotDisplayComparator());
        potsPanel=new JPanel();
        potsPanel.getInsets().set(0,0,0,0);
        potsPanel.setLayout(new BoxLayout(potsPanel,BoxLayout.Y_AXIS));
        scrollPane=new JScrollPane(potsPanel);
        potsPanel.add(new PotSorter(componentList,potList));
        add(scrollPane);
        for(Pot p:potList){
            JComponent s=p.makeGUIPotControl(frame); // make a bias control gui component
            potsPanel.add(s);
            componentList.add(s);
        }
        JPanel fillPanel=new JPanel();
        fillPanel.setMinimumSize(new Dimension(0,0));
        fillPanel.setPreferredSize(new Dimension(0,0));
        fillPanel.setMaximumSize(new Dimension(32767,32767));
        potsPanel.add(fillPanel); // spacer at bottom so biases don't stretch out too much
    }
    
    private class PotDisplayComparator implements Comparator<Pot>{
        public int compare(Pot p1, Pot p2){
            if(p1.getDisplayPosition()<p2.getDisplayPosition()) return -1;
            if(p1.getDisplayPosition()==p2.getDisplayPosition()) return 0;
            return 1;
        }
        public boolean equals(IPot p1, IPot p2){
            if(p1.getDisplayPosition()==p2.getDisplayPosition()) return true; else return false;
        }
    }
    
    
    /** This method is called from within the constructor to
     * initialize the form.
     * WARNING: Do NOT modify this code. The content of this method is
     * always regenerated by the Form Editor.
     */
    // <editor-fold defaultstate="collapsed" desc="Generated Code">//GEN-BEGIN:initComponents
    private void initComponents() {

        globalValuePanel = new javax.swing.JPanel();
        jLabel1 = new javax.swing.JLabel();
        globalValueTextField = new javax.swing.JTextField();

        setBorder(javax.swing.BorderFactory.createTitledBorder("IPot Array"));
        setToolTipText("");
        setLayout(new javax.swing.BoxLayout(this, javax.swing.BoxLayout.Y_AXIS));

        globalValuePanel.setLayout(new javax.swing.BoxLayout(globalValuePanel, javax.swing.BoxLayout.X_AXIS));

        jLabel1.setLabelFor(globalValueTextField);
        jLabel1.setText("Set global value");
        globalValuePanel.add(jLabel1);

        globalValueTextField.setColumns(10);
        globalValueTextField.setToolTipText("enter a value here to set global bit value");
        globalValueTextField.setMaximumSize(new java.awt.Dimension(100, 30));
        globalValueTextField.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                globalValueTextFieldActionPerformed(evt);
            }
        });
        globalValuePanel.add(globalValueTextField);

        add(globalValuePanel);
    }// </editor-fold>//GEN-END:initComponents
        
    private void jButton1ActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_jButton1ActionPerformed
    }//GEN-LAST:event_jButton1ActionPerformed
    
    private void filterBiasnameTextFieldKeyTyped(java.awt.event.KeyEvent evt) {//GEN-FIRST:event_filterBiasnameTextFieldKeyTyped
    }//GEN-LAST:event_filterBiasnameTextFieldKeyTyped
        
    private void globalValueTextFieldActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_globalValueTextFieldActionPerformed
        int v;
        try{
            v=Integer.parseInt(globalValueTextField.getText());
            pots.setAllToBitValue(v);
        }catch(NumberFormatException e){
            globalValueTextField.selectAll();
        }
    }//GEN-LAST:event_globalValueTextFieldActionPerformed
    
    
    // Variables declaration - do not modify//GEN-BEGIN:variables
    private javax.swing.JPanel globalValuePanel;
    private javax.swing.JTextField globalValueTextField;
    private javax.swing.JLabel jLabel1;
    // End of variables declaration//GEN-END:variables
    
    
    public PotArray getPots() {
        return this.pots;
    }
    
    public void setPots(final PotArray pots) {
        this.pots = pots;
        buildPanel();
    }
    
    
}
