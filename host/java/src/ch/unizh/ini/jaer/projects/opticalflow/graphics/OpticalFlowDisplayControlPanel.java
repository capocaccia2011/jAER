/*
 * OpticalFlowDisplayControlPanel.java
 *
 * Created on December 17, 2006, 8:30 AM
 */

package ch.unizh.ini.jaer.projects.opticalflow.graphics;

import javax.swing.*;

/**
 *
 * @author  tobi
 */
public class OpticalFlowDisplayControlPanel extends javax.swing.JPanel {
    OpticalFlowDisplayMethod displayMethod=null;
    

    
    /** Creates new form OpticalFlowDisplayControlPanel
     @param displayMethod the OpticalFlowDisplayMethod to control
     */
    public OpticalFlowDisplayControlPanel(OpticalFlowDisplayMethod displayMethod) {
        this.displayMethod=displayMethod;
        initComponents();
        photoOffsetSlider.setValue(intFrom(displayMethod.getPhotoOffset()));
        photoGainSlider.setValue(intFrom(displayMethod.getPhotoGain()));
        localOffsetSlider.setValue(intFrom(displayMethod.getLocalMotionOffset()));
        localGainSlider.setValue(intFrom(displayMethod.getLocalMotionGain()));
        vectorScaleSliider.setValue(intFrom(displayMethod.getVectorLengthScale()));
        globalVectorScaleSlider.setValue(intFrom(displayMethod.getGlobalMotionGain()));
        enableGlobalMotionCheckBox.setSelected(displayMethod.isGlobalDisplayEnabled());
        enableLocalMotionCheckBox.setSelected(displayMethod.isLocalDisplayEnabled());
        enablePhotoreceptorCheckBox.setSelected(displayMethod.isPhotoDisplayEnabled());
        enabledLocalMotionColorsCheckBox.setSelected(displayMethod.isLocalMotionColorsEnabled());
        buttonGroup1.add(this.jRadioButton1);
        buttonGroup1.add(this.jRadioButton2);
        buttonGroup1.add(this.jRadioButton3);
        setControlsVisible(false);
    }
    
    // returns int 0-100 from float 0-1
    int intFrom(float f){
        return (int)(f*100);
    }
    
    // returns a float 0-1 range value from the event assuming it is from slider that has range 0-100
    float floatFrom(javax.swing.event.ChangeEvent e){
        float f=0.01f*((JSlider)e.getSource()).getValue();
        return f;
    }
    
    /** This method is called from within the constructor to
     * initialize the form.
     * WARNING: Do NOT modify this code. The content of this method is
     * always regenerated by the Form Editor.
     */
    // <editor-fold defaultstate="collapsed" desc="Generated Code">//GEN-BEGIN:initComponents
    private void initComponents() {

        buttonGroup1 = new javax.swing.ButtonGroup();
        localPanel = new javax.swing.JPanel();
        jPanel4 = new javax.swing.JPanel();
        localOffsetSlider = new javax.swing.JSlider();
        jPanel5 = new javax.swing.JPanel();
        localGainSlider = new javax.swing.JSlider();
        jPanel1 = new javax.swing.JPanel();
        vectorScaleSliider = new javax.swing.JSlider();
        jPanel2 = new javax.swing.JPanel();
        globalVectorScaleSlider = new javax.swing.JSlider();
        photoPanel = new javax.swing.JPanel();
        jPanel8 = new javax.swing.JPanel();
        photoOffsetSlider = new javax.swing.JSlider();
        jPanel9 = new javax.swing.JPanel();
        photoGainSlider = new javax.swing.JSlider();
        showHideToggleButton = new javax.swing.JToggleButton();
        displayPanel = new javax.swing.JPanel();
        enableGlobalMotionCheckBox = new javax.swing.JCheckBox();
        enableLocalMotionCheckBox = new javax.swing.JCheckBox();
        enabledLocalMotionColorsCheckBox = new javax.swing.JCheckBox();
        enablePhotoreceptorCheckBox = new javax.swing.JCheckBox();
        jButton1 = new javax.swing.JButton();
        rawChannelControlPanel = new javax.swing.JPanel();
        jRadioButton1 = new javax.swing.JRadioButton();
        jRadioButton2 = new javax.swing.JRadioButton();
        jRadioButton3 = new javax.swing.JRadioButton();

        localPanel.setBorder(javax.swing.BorderFactory.createTitledBorder("Local motion"));

        jPanel4.setBorder(javax.swing.BorderFactory.createTitledBorder("Offset"));

        localOffsetSlider.setMinimumSize(new java.awt.Dimension(36, 12));
        localOffsetSlider.setPreferredSize(new java.awt.Dimension(200, 15));
        localOffsetSlider.addChangeListener(new javax.swing.event.ChangeListener() {
            public void stateChanged(javax.swing.event.ChangeEvent evt) {
                localOffsetSliderStateChanged(evt);
            }
        });

        org.jdesktop.layout.GroupLayout jPanel4Layout = new org.jdesktop.layout.GroupLayout(jPanel4);
        jPanel4.setLayout(jPanel4Layout);
        jPanel4Layout.setHorizontalGroup(
            jPanel4Layout.createParallelGroup(org.jdesktop.layout.GroupLayout.LEADING)
            .add(org.jdesktop.layout.GroupLayout.TRAILING, jPanel4Layout.createSequentialGroup()
                .addContainerGap(org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
                .add(localOffsetSlider, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE, 133, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE)
                .add(77, 77, 77))
        );
        jPanel4Layout.setVerticalGroup(
            jPanel4Layout.createParallelGroup(org.jdesktop.layout.GroupLayout.LEADING)
            .add(jPanel4Layout.createSequentialGroup()
                .add(localOffsetSlider, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE)
                .addContainerGap(org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE))
        );

        jPanel5.setBorder(javax.swing.BorderFactory.createTitledBorder("Gain"));

        localGainSlider.setMinimumSize(new java.awt.Dimension(36, 12));
        localGainSlider.setPreferredSize(new java.awt.Dimension(200, 15));
        localGainSlider.addChangeListener(new javax.swing.event.ChangeListener() {
            public void stateChanged(javax.swing.event.ChangeEvent evt) {
                localGainSliderStateChanged(evt);
            }
        });

        org.jdesktop.layout.GroupLayout jPanel5Layout = new org.jdesktop.layout.GroupLayout(jPanel5);
        jPanel5.setLayout(jPanel5Layout);
        jPanel5Layout.setHorizontalGroup(
            jPanel5Layout.createParallelGroup(org.jdesktop.layout.GroupLayout.LEADING)
            .add(jPanel5Layout.createSequentialGroup()
                .add(localGainSlider, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE, 110, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE)
                .addContainerGap(org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE))
        );
        jPanel5Layout.setVerticalGroup(
            jPanel5Layout.createParallelGroup(org.jdesktop.layout.GroupLayout.LEADING)
            .add(jPanel5Layout.createSequentialGroup()
                .add(localGainSlider, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE)
                .addContainerGap(org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE))
        );

        jPanel1.setBorder(javax.swing.BorderFactory.createTitledBorder("local vector scale"));

        vectorScaleSliider.setMinimumSize(new java.awt.Dimension(36, 12));
        vectorScaleSliider.setPreferredSize(new java.awt.Dimension(200, 15));
        vectorScaleSliider.addChangeListener(new javax.swing.event.ChangeListener() {
            public void stateChanged(javax.swing.event.ChangeEvent evt) {
                vectorScaleSliiderStateChanged(evt);
            }
        });

        org.jdesktop.layout.GroupLayout jPanel1Layout = new org.jdesktop.layout.GroupLayout(jPanel1);
        jPanel1.setLayout(jPanel1Layout);
        jPanel1Layout.setHorizontalGroup(
            jPanel1Layout.createParallelGroup(org.jdesktop.layout.GroupLayout.LEADING)
            .add(jPanel1Layout.createSequentialGroup()
                .addContainerGap()
                .add(vectorScaleSliider, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE, 121, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE)
                .addContainerGap(org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE))
        );
        jPanel1Layout.setVerticalGroup(
            jPanel1Layout.createParallelGroup(org.jdesktop.layout.GroupLayout.LEADING)
            .add(vectorScaleSliider, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE)
        );

        jPanel2.setBorder(javax.swing.BorderFactory.createTitledBorder("global average scale"));

        globalVectorScaleSlider.setMinimumSize(new java.awt.Dimension(36, 12));
        globalVectorScaleSlider.setPreferredSize(new java.awt.Dimension(200, 15));
        globalVectorScaleSlider.addChangeListener(new javax.swing.event.ChangeListener() {
            public void stateChanged(javax.swing.event.ChangeEvent evt) {
                globalVectorScaleSliderStateChanged(evt);
            }
        });

        org.jdesktop.layout.GroupLayout jPanel2Layout = new org.jdesktop.layout.GroupLayout(jPanel2);
        jPanel2.setLayout(jPanel2Layout);
        jPanel2Layout.setHorizontalGroup(
            jPanel2Layout.createParallelGroup(org.jdesktop.layout.GroupLayout.LEADING)
            .add(jPanel2Layout.createSequentialGroup()
                .add(globalVectorScaleSlider, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE, 98, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE)
                .addContainerGap(26, Short.MAX_VALUE))
        );
        jPanel2Layout.setVerticalGroup(
            jPanel2Layout.createParallelGroup(org.jdesktop.layout.GroupLayout.LEADING)
            .add(globalVectorScaleSlider, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE, 15, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE)
        );

        org.jdesktop.layout.GroupLayout localPanelLayout = new org.jdesktop.layout.GroupLayout(localPanel);
        localPanel.setLayout(localPanelLayout);
        localPanelLayout.setHorizontalGroup(
            localPanelLayout.createParallelGroup(org.jdesktop.layout.GroupLayout.LEADING)
            .add(localPanelLayout.createSequentialGroup()
                .add(localPanelLayout.createParallelGroup(org.jdesktop.layout.GroupLayout.LEADING)
                    .add(localPanelLayout.createSequentialGroup()
                        .add(jPanel4, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE, 157, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE)
                        .addPreferredGap(org.jdesktop.layout.LayoutStyle.RELATED)
                        .add(jPanel5, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE))
                    .add(localPanelLayout.createSequentialGroup()
                        .add(jPanel1, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE)
                        .addPreferredGap(org.jdesktop.layout.LayoutStyle.RELATED)
                        .add(jPanel2, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)))
                .addContainerGap())
        );
        localPanelLayout.setVerticalGroup(
            localPanelLayout.createParallelGroup(org.jdesktop.layout.GroupLayout.LEADING)
            .add(localPanelLayout.createSequentialGroup()
                .add(localPanelLayout.createParallelGroup(org.jdesktop.layout.GroupLayout.TRAILING, false)
                    .add(org.jdesktop.layout.GroupLayout.LEADING, jPanel4, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
                    .add(org.jdesktop.layout.GroupLayout.LEADING, jPanel5, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE))
                .addPreferredGap(org.jdesktop.layout.LayoutStyle.RELATED)
                .add(localPanelLayout.createParallelGroup(org.jdesktop.layout.GroupLayout.LEADING, false)
                    .add(jPanel1, 0, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
                    .add(jPanel2, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE))
                .addContainerGap())
        );

        photoPanel.setBorder(javax.swing.BorderFactory.createTitledBorder("Photoreceptor"));

        jPanel8.setBorder(javax.swing.BorderFactory.createTitledBorder("Offset"));

        photoOffsetSlider.setMinimumSize(new java.awt.Dimension(36, 12));
        photoOffsetSlider.setPreferredSize(new java.awt.Dimension(200, 15));
        photoOffsetSlider.addChangeListener(new javax.swing.event.ChangeListener() {
            public void stateChanged(javax.swing.event.ChangeEvent evt) {
                photoOffsetSliderStateChanged(evt);
            }
        });

        org.jdesktop.layout.GroupLayout jPanel8Layout = new org.jdesktop.layout.GroupLayout(jPanel8);
        jPanel8.setLayout(jPanel8Layout);
        jPanel8Layout.setHorizontalGroup(
            jPanel8Layout.createParallelGroup(org.jdesktop.layout.GroupLayout.LEADING)
            .add(org.jdesktop.layout.GroupLayout.TRAILING, jPanel8Layout.createSequentialGroup()
                .addContainerGap(org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
                .add(photoOffsetSlider, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE)
                .addContainerGap())
        );
        jPanel8Layout.setVerticalGroup(
            jPanel8Layout.createParallelGroup(org.jdesktop.layout.GroupLayout.LEADING)
            .add(jPanel8Layout.createSequentialGroup()
                .add(photoOffsetSlider, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE)
                .addContainerGap(org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE))
        );

        jPanel9.setBorder(javax.swing.BorderFactory.createTitledBorder("Gain"));

        photoGainSlider.setMinimumSize(new java.awt.Dimension(36, 12));
        photoGainSlider.setPreferredSize(new java.awt.Dimension(200, 15));
        photoGainSlider.addChangeListener(new javax.swing.event.ChangeListener() {
            public void stateChanged(javax.swing.event.ChangeEvent evt) {
                photoGainSliderStateChanged(evt);
            }
        });

        org.jdesktop.layout.GroupLayout jPanel9Layout = new org.jdesktop.layout.GroupLayout(jPanel9);
        jPanel9.setLayout(jPanel9Layout);
        jPanel9Layout.setHorizontalGroup(
            jPanel9Layout.createParallelGroup(org.jdesktop.layout.GroupLayout.LEADING)
            .add(jPanel9Layout.createSequentialGroup()
                .add(photoGainSlider, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE)
                .addContainerGap(org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE))
        );
        jPanel9Layout.setVerticalGroup(
            jPanel9Layout.createParallelGroup(org.jdesktop.layout.GroupLayout.LEADING)
            .add(jPanel9Layout.createSequentialGroup()
                .add(photoGainSlider, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE)
                .addContainerGap(org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE))
        );

        org.jdesktop.layout.GroupLayout photoPanelLayout = new org.jdesktop.layout.GroupLayout(photoPanel);
        photoPanel.setLayout(photoPanelLayout);
        photoPanelLayout.setHorizontalGroup(
            photoPanelLayout.createParallelGroup(org.jdesktop.layout.GroupLayout.LEADING)
            .add(photoPanelLayout.createSequentialGroup()
                .add(jPanel8, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE)
                .addPreferredGap(org.jdesktop.layout.LayoutStyle.RELATED)
                .add(jPanel9, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE)
                .addContainerGap())
        );
        photoPanelLayout.setVerticalGroup(
            photoPanelLayout.createParallelGroup(org.jdesktop.layout.GroupLayout.LEADING)
            .add(jPanel8, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, 45, Short.MAX_VALUE)
            .add(jPanel9, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, 45, Short.MAX_VALUE)
        );

        showHideToggleButton.setText("Show rendering controls");
        showHideToggleButton.setToolTipText("Shows controls for display of motion data");
        showHideToggleButton.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                showHideToggleButtonActionPerformed(evt);
            }
        });

        displayPanel.setBorder(javax.swing.BorderFactory.createTitledBorder("Display Control"));

        enableGlobalMotionCheckBox.setText("global motion average");
        enableGlobalMotionCheckBox.setBorder(javax.swing.BorderFactory.createEmptyBorder(0, 0, 0, 0));
        enableGlobalMotionCheckBox.setMargin(new java.awt.Insets(0, 0, 0, 0));
        enableGlobalMotionCheckBox.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                enableGlobalMotionCheckBoxActionPerformed(evt);
            }
        });

        enableLocalMotionCheckBox.setText("local motion vectors");
        enableLocalMotionCheckBox.setBorder(javax.swing.BorderFactory.createEmptyBorder(0, 0, 0, 0));
        enableLocalMotionCheckBox.setMargin(new java.awt.Insets(0, 0, 0, 0));
        enableLocalMotionCheckBox.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                enableLocalMotionCheckBoxActionPerformed(evt);
            }
        });

        enabledLocalMotionColorsCheckBox.setText("local motion colors");
        enabledLocalMotionColorsCheckBox.setBorder(javax.swing.BorderFactory.createEmptyBorder(0, 0, 0, 0));
        enabledLocalMotionColorsCheckBox.setMargin(new java.awt.Insets(0, 0, 0, 0));
        enabledLocalMotionColorsCheckBox.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                enabledLocalMotionColorsCheckBoxActionPerformed(evt);
            }
        });

        enablePhotoreceptorCheckBox.setText("raw channel");
        enablePhotoreceptorCheckBox.setBorder(javax.swing.BorderFactory.createEmptyBorder(0, 0, 0, 0));
        enablePhotoreceptorCheckBox.setMargin(new java.awt.Insets(0, 0, 0, 0));
        enablePhotoreceptorCheckBox.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                enablePhotoreceptorCheckBoxActionPerformed(evt);
            }
        });

        org.jdesktop.layout.GroupLayout displayPanelLayout = new org.jdesktop.layout.GroupLayout(displayPanel);
        displayPanel.setLayout(displayPanelLayout);
        displayPanelLayout.setHorizontalGroup(
            displayPanelLayout.createParallelGroup(org.jdesktop.layout.GroupLayout.LEADING)
            .add(displayPanelLayout.createSequentialGroup()
                .addContainerGap()
                .add(displayPanelLayout.createParallelGroup(org.jdesktop.layout.GroupLayout.LEADING)
                    .add(enableGlobalMotionCheckBox)
                    .add(enablePhotoreceptorCheckBox)
                    .add(enabledLocalMotionColorsCheckBox)
                    .add(enableLocalMotionCheckBox))
                .addContainerGap(org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE))
        );
        displayPanelLayout.setVerticalGroup(
            displayPanelLayout.createParallelGroup(org.jdesktop.layout.GroupLayout.LEADING)
            .add(displayPanelLayout.createSequentialGroup()
                .add(enableGlobalMotionCheckBox)
                .addPreferredGap(org.jdesktop.layout.LayoutStyle.RELATED)
                .add(enableLocalMotionCheckBox)
                .addPreferredGap(org.jdesktop.layout.LayoutStyle.RELATED)
                .add(enabledLocalMotionColorsCheckBox)
                .addPreferredGap(org.jdesktop.layout.LayoutStyle.RELATED, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
                .add(enablePhotoreceptorCheckBox)
                .addContainerGap())
        );

        jButton1.setText("New Channel Viewer");
        jButton1.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                jButton1ActionPerformed(evt);
            }
        });

        rawChannelControlPanel.setBorder(javax.swing.BorderFactory.createTitledBorder("Raw Channel"));

        jRadioButton1.setText("ph");
        jRadioButton1.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                jRadioButton1ActionPerformed(evt);
            }
        });

        jRadioButton2.setText("lmc1");
        jRadioButton2.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                jRadioButton2ActionPerformed(evt);
            }
        });

        jRadioButton3.setText("lmc2");
        jRadioButton3.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                jRadioButton3ActionPerformed(evt);
            }
        });

        org.jdesktop.layout.GroupLayout rawChannelControlPanelLayout = new org.jdesktop.layout.GroupLayout(rawChannelControlPanel);
        rawChannelControlPanel.setLayout(rawChannelControlPanelLayout);
        rawChannelControlPanelLayout.setHorizontalGroup(
            rawChannelControlPanelLayout.createParallelGroup(org.jdesktop.layout.GroupLayout.LEADING)
            .add(rawChannelControlPanelLayout.createSequentialGroup()
                .addContainerGap()
                .add(rawChannelControlPanelLayout.createParallelGroup(org.jdesktop.layout.GroupLayout.LEADING)
                    .add(jRadioButton1)
                    .add(jRadioButton2)
                    .add(jRadioButton3))
                .addContainerGap(23, Short.MAX_VALUE))
        );
        rawChannelControlPanelLayout.setVerticalGroup(
            rawChannelControlPanelLayout.createParallelGroup(org.jdesktop.layout.GroupLayout.LEADING)
            .add(rawChannelControlPanelLayout.createSequentialGroup()
                .add(jRadioButton1)
                .addPreferredGap(org.jdesktop.layout.LayoutStyle.RELATED)
                .add(jRadioButton2)
                .addPreferredGap(org.jdesktop.layout.LayoutStyle.RELATED)
                .add(jRadioButton3)
                .addContainerGap(20, Short.MAX_VALUE))
        );

        org.jdesktop.layout.GroupLayout layout = new org.jdesktop.layout.GroupLayout(this);
        this.setLayout(layout);
        layout.setHorizontalGroup(
            layout.createParallelGroup(org.jdesktop.layout.GroupLayout.LEADING)
            .add(layout.createSequentialGroup()
                .addContainerGap()
                .add(layout.createParallelGroup(org.jdesktop.layout.GroupLayout.LEADING, false)
                    .add(layout.createSequentialGroup()
                        .add(localPanel, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE)
                        .addPreferredGap(org.jdesktop.layout.LayoutStyle.RELATED)
                        .add(rawChannelControlPanel, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE)
                        .addPreferredGap(org.jdesktop.layout.LayoutStyle.RELATED)
                        .add(layout.createParallelGroup(org.jdesktop.layout.GroupLayout.TRAILING)
                            .add(showHideToggleButton)
                            .add(displayPanel, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE)))
                    .add(layout.createSequentialGroup()
                        .add(photoPanel, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE)
                        .addPreferredGap(org.jdesktop.layout.LayoutStyle.RELATED, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
                        .add(jButton1, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE, 117, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE)))
                .addContainerGap(org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE))
        );
        layout.setVerticalGroup(
            layout.createParallelGroup(org.jdesktop.layout.GroupLayout.LEADING)
            .add(layout.createSequentialGroup()
                .addContainerGap()
                .add(layout.createParallelGroup(org.jdesktop.layout.GroupLayout.LEADING)
                    .add(org.jdesktop.layout.GroupLayout.TRAILING, localPanel, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE)
                    .add(org.jdesktop.layout.GroupLayout.TRAILING, displayPanel, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE)
                    .add(layout.createSequentialGroup()
                        .add(showHideToggleButton)
                        .addPreferredGap(org.jdesktop.layout.LayoutStyle.RELATED, 17, Short.MAX_VALUE)
                        .add(rawChannelControlPanel, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE)))
                .addPreferredGap(org.jdesktop.layout.LayoutStyle.RELATED)
                .add(layout.createParallelGroup(org.jdesktop.layout.GroupLayout.TRAILING)
                    .add(photoPanel, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE)
                    .add(jButton1))
                .addContainerGap())
        );
    }// </editor-fold>//GEN-END:initComponents

    private void globalVectorScaleSliderStateChanged(javax.swing.event.ChangeEvent evt) {//GEN-FIRST:event_globalVectorScaleSliderStateChanged
        displayMethod.setGlobalMotionGain(floatFrom(evt));
    }//GEN-LAST:event_globalVectorScaleSliderStateChanged
    
    private void enabledLocalMotionColorsCheckBoxActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_enabledLocalMotionColorsCheckBoxActionPerformed
        displayMethod.setLocalMotionColorsEnabled(enabledLocalMotionColorsCheckBox.isSelected());
    }//GEN-LAST:event_enabledLocalMotionColorsCheckBoxActionPerformed
    
    private void vectorScaleSliiderStateChanged(javax.swing.event.ChangeEvent evt) {//GEN-FIRST:event_vectorScaleSliiderStateChanged
        displayMethod.setVectorLengthScale(floatFrom(evt));
    }//GEN-LAST:event_vectorScaleSliiderStateChanged
    
    private void enablePhotoreceptorCheckBoxActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_enablePhotoreceptorCheckBoxActionPerformed
        displayMethod.setPhotoDisplayEnabled(enablePhotoreceptorCheckBox.isSelected());
    }//GEN-LAST:event_enablePhotoreceptorCheckBoxActionPerformed
    
    private void enableLocalMotionCheckBoxActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_enableLocalMotionCheckBoxActionPerformed
        displayMethod.setLocalDisplayEnabled(enableLocalMotionCheckBox.isSelected());
    }//GEN-LAST:event_enableLocalMotionCheckBoxActionPerformed
    
    private void enableGlobalMotionCheckBoxActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_enableGlobalMotionCheckBoxActionPerformed
        displayMethod.setGlobalDisplayEnabled(enableGlobalMotionCheckBox.isSelected());
    }//GEN-LAST:event_enableGlobalMotionCheckBoxActionPerformed
    
    void setControlsVisible(boolean yes){
        if(yes) showHideToggleButton.setText("Hide"); else showHideToggleButton.setText("Show motion rendering/acquisition controls");
        localPanel.setVisible(yes);
        photoPanel.setVisible(yes);
        displayPanel.setVisible(yes);
        rawChannelControlPanel.setVisible(yes);
        invalidate();
    }
    
    private void showHideToggleButtonActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_showHideToggleButtonActionPerformed
        setControlsVisible(showHideToggleButton.isSelected());
    }//GEN-LAST:event_showHideToggleButtonActionPerformed
    
    private void photoGainSliderStateChanged(javax.swing.event.ChangeEvent evt) {//GEN-FIRST:event_photoGainSliderStateChanged
        displayMethod.setPhotoGain(floatFrom(evt));
    }//GEN-LAST:event_photoGainSliderStateChanged
    
    private void photoOffsetSliderStateChanged(javax.swing.event.ChangeEvent evt) {//GEN-FIRST:event_photoOffsetSliderStateChanged
        displayMethod.setPhotoOffset(floatFrom(evt));
    }//GEN-LAST:event_photoOffsetSliderStateChanged
            
    private void localGainSliderStateChanged(javax.swing.event.ChangeEvent evt) {//GEN-FIRST:event_localGainSliderStateChanged
        displayMethod.setLocalMotionGain(floatFrom(evt));
    }//GEN-LAST:event_localGainSliderStateChanged
    
    
    
    private void localOffsetSliderStateChanged(javax.swing.event.ChangeEvent evt) {//GEN-FIRST:event_localOffsetSliderStateChanged
        displayMethod.setLocalMotionOffset(floatFrom(evt));
    }//GEN-LAST:event_localOffsetSliderStateChanged

    private void jButton1ActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_jButton1ActionPerformed
        ChannelViewer viewer= new ChannelViewer(MotionViewer.chip);
        viewer.setVisible(true);// TODO add your handling code here:
    }//GEN-LAST:event_jButton1ActionPerformed

    private void jRadioButton1ActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_jRadioButton1ActionPerformed
        displayMethod.setRawChannelDisplayed(0);        // TODO add your handling code here:
    }//GEN-LAST:event_jRadioButton1ActionPerformed

    private void jRadioButton2ActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_jRadioButton2ActionPerformed
       displayMethod.setRawChannelDisplayed(1); // TODO add your handling code here:
    }//GEN-LAST:event_jRadioButton2ActionPerformed

    private void jRadioButton3ActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_jRadioButton3ActionPerformed
        displayMethod.setRawChannelDisplayed(2); // TODO add your handling code here:
    }//GEN-LAST:event_jRadioButton3ActionPerformed
    
    
    // Variables declaration - do not modify//GEN-BEGIN:variables
    private javax.swing.ButtonGroup buttonGroup1;
    private javax.swing.JPanel displayPanel;
    private javax.swing.JCheckBox enableGlobalMotionCheckBox;
    private javax.swing.JCheckBox enableLocalMotionCheckBox;
    private javax.swing.JCheckBox enablePhotoreceptorCheckBox;
    private javax.swing.JCheckBox enabledLocalMotionColorsCheckBox;
    private javax.swing.JSlider globalVectorScaleSlider;
    private javax.swing.JButton jButton1;
    private javax.swing.JPanel jPanel1;
    private javax.swing.JPanel jPanel2;
    private javax.swing.JPanel jPanel4;
    private javax.swing.JPanel jPanel5;
    private javax.swing.JPanel jPanel8;
    private javax.swing.JPanel jPanel9;
    private javax.swing.JRadioButton jRadioButton1;
    private javax.swing.JRadioButton jRadioButton2;
    private javax.swing.JRadioButton jRadioButton3;
    private javax.swing.JSlider localGainSlider;
    private javax.swing.JSlider localOffsetSlider;
    private javax.swing.JPanel localPanel;
    private javax.swing.JSlider photoGainSlider;
    private javax.swing.JSlider photoOffsetSlider;
    private javax.swing.JPanel photoPanel;
    private javax.swing.JPanel rawChannelControlPanel;
    private javax.swing.JToggleButton showHideToggleButton;
    private javax.swing.JSlider vectorScaleSliider;
    // End of variables declaration//GEN-END:variables
    
}
