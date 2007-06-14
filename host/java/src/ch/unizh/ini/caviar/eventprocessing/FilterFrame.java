/*
 * FilterFrame.java
 *
 * Created on October 31, 2005, 8:29 PM
 */

package ch.unizh.ini.caviar.eventprocessing;

import ch.unizh.ini.caviar.*;
import ch.unizh.ini.caviar.chip.*;
import ch.unizh.ini.caviar.util.*;
import java.awt.BorderLayout;
import java.awt.Dimension;
import java.awt.Rectangle;
import java.awt.event.*;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.lang.reflect.*;
import java.util.*;
import java.util.logging.Logger;
import java.util.prefs.Preferences;
import javax.swing.*;
import javax.swing.BoxLayout;

/**
 * This JFrame holds all the event processing controls. It also allows global measurement of filter performance and
 * allows setting a flag that determines of the filters process events on the rendering or data acquisition cycle.
 * Export and import of filter preferences are also possible.
 * @author  tobi
 */
public class FilterFrame extends javax.swing.JFrame {
    
    final int MAX_ROWS=10; // max rows of filters, then wraps back to top
    static Preferences prefs=Preferences.userNodeForPackage(FilterFrame.class);
    
    Logger log=Logger.getLogger("filter");
    AEChip chip;
    FilterChain filterChain;
    RecentFiles recentFiles=null;
    private boolean restoreFilterEnabledStateEnabled;
    
    /** Creates new form FilterFrame */
    public FilterFrame(AEChip chip) {
        this.chip=chip;
        this.filterChain=chip.getFilterChain();
        chip.setFilterFrame(this);
        setName("FilterFrame");
//        this.realTimeFilterChain=chip.getRealTimeFilterChain();
        initComponents();
        rebuildContents();
        setRestoreFilterEnabledStateEnabled(prefs.getBoolean("FilterFrame.restoreFilterEnabledStateEnabled",true)); // sets the menu item state
        if(chip!=null){
            setTitle(chip.getName()+" filters");
        }
        WindowSaver.restoreWindowLocation(this,prefs);
        switch(filterChain.getProcessingMode()){
            case RENDERING:
                renderingModeMenuItem.setSelected(true);
                break;
            case ACQUISITION:
                acquisitionModeMenuItem.setSelected(true);
                break;
            default:
                
        }
        if(filterChain!=null) filterChain.setMeasurePerformanceEnabled(measurePerformanceCheckBoxMenuItem.isSelected());
        // recent files tracks recently used files *and* folders. recentFiles adds the anonymous listener
        // built here to open the selected file
        recentFiles=new RecentFiles(prefs, fileMenu, new ActionListener(){
            public void actionPerformed(ActionEvent evt){
                File f=new File(evt.getActionCommand());
                log.info("opening "+evt.getActionCommand());
                try{
                    if(f!=null && f.isFile()){
                        loadFile(f);
                    }else if(f!=null && f.isDirectory()){
                        prefs.put("FilterFrame.lastFile",f.getCanonicalPath());
                        loadMenuItemActionPerformed(null);
                    }
                }catch(Exception fnf){
                    fnf.printStackTrace();
                    recentFiles.removeFile(f);
                }
            }
        });
        
        // now set state of all filters enabled
        if(restoreFilterEnabledStateEnabled){
            log.info("Restoring filter enabled setting for each filter");
            for(EventFilter f:filterChain){
                boolean yes=prefs.getBoolean(f.prefsEnabledKey(),false);
                if(yes) log.info("enabling "+f);
                f.setFilterEnabled(yes);
            }
        }
    }
    
    
private void setSetTimeLimitMenuItem(){
        setTimeLimitMenuItem.setText(getTimeLimitMenuItemText());
    }

private String getTimeLimitMenuItemText(){
    return String.format("Set time limit. (Currently %d ms)",filterChain.getTimeLimitMs());
}
    
    /** This method is called from within the constructor to
     * initialize the form.
     * WARNING: Do NOT modify this code. The content of this method is
     * always regenerated by the Form Editor.
     */
    // <editor-fold defaultstate="collapsed" desc=" Generated Code ">//GEN-BEGIN:initComponents
    private void initComponents() {
        modeButtonGroup = new javax.swing.ButtonGroup();
        jToolBar1 = new javax.swing.JToolBar();
        disableFilteringToggleButton = new javax.swing.JToggleButton();
        statusPanel = new javax.swing.JPanel();
        scrollPane = new javax.swing.JScrollPane();
        filtersPanel = new javax.swing.JPanel();
        mainMenuBar = new javax.swing.JMenuBar();
        fileMenu = new javax.swing.JMenu();
        loadMenuItem = new javax.swing.JMenuItem();
        saveAsMenuItem = new javax.swing.JMenuItem();
        jSeparator2 = new javax.swing.JSeparator();
        exitMenuItem = new javax.swing.JMenuItem();
        viewMenu = new javax.swing.JMenu();
        customizeMenuItem = new javax.swing.JMenuItem();
        modeMenu = new javax.swing.JMenu();
        renderingModeMenuItem = new javax.swing.JRadioButtonMenuItem();
        acquisitionModeMenuItem = new javax.swing.JRadioButtonMenuItem();
        jSeparator1 = new javax.swing.JSeparator();
        measurePerformanceCheckBoxMenuItem = new javax.swing.JCheckBoxMenuItem();
        jSeparator3 = new javax.swing.JSeparator();
        limitTimeCheckBoxMenuItem = new javax.swing.JCheckBoxMenuItem();
        setTimeLimitMenuItem = new javax.swing.JMenuItem();
        jSeparator4 = new javax.swing.JSeparator();
        restoreFilterEnabledStateCheckBoxMenuItem = new javax.swing.JCheckBoxMenuItem();

        setDefaultCloseOperation(javax.swing.WindowConstants.DISPOSE_ON_CLOSE);
        setTitle("FilterControl");
        setMinimumSize(new java.awt.Dimension(150, 37));
        addComponentListener(new java.awt.event.ComponentAdapter() {
            public void componentMoved(java.awt.event.ComponentEvent evt) {
                formComponentMoved(evt);
            }
        });
        addWindowListener(new java.awt.event.WindowAdapter() {
            public void windowClosing(java.awt.event.WindowEvent evt) {
                formWindowClosing(evt);
            }
        });

        disableFilteringToggleButton.setText("Disable Filters");
        disableFilteringToggleButton.setToolTipText("Temporarily disables all filters");
        disableFilteringToggleButton.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                disableFilteringToggleButtonActionPerformed(evt);
            }
        });

        jToolBar1.add(disableFilteringToggleButton);

        getContentPane().add(jToolBar1, java.awt.BorderLayout.NORTH);

        statusPanel.setLayout(new java.awt.BorderLayout());

        getContentPane().add(statusPanel, java.awt.BorderLayout.SOUTH);

        filtersPanel.setLayout(new javax.swing.BoxLayout(filtersPanel, javax.swing.BoxLayout.Y_AXIS));

        filtersPanel.setMaximumSize(new java.awt.Dimension(0, 0));
        filtersPanel.setMinimumSize(new java.awt.Dimension(100, 300));
        scrollPane.setViewportView(filtersPanel);

        getContentPane().add(scrollPane, java.awt.BorderLayout.CENTER);

        fileMenu.setMnemonic('f');
        fileMenu.setText("File");
        loadMenuItem.setMnemonic('l');
        loadMenuItem.setText("Load settings...");
        loadMenuItem.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                loadMenuItemActionPerformed(evt);
            }
        });

        fileMenu.add(loadMenuItem);

        saveAsMenuItem.setText("Save settings as...");
        saveAsMenuItem.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                saveAsMenuItemActionPerformed(evt);
            }
        });

        fileMenu.add(saveAsMenuItem);

        fileMenu.add(jSeparator2);

        exitMenuItem.setMnemonic('x');
        exitMenuItem.setText("Exit");
        exitMenuItem.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                exitMenuItemActionPerformed(evt);
            }
        });

        fileMenu.add(exitMenuItem);

        mainMenuBar.add(fileMenu);

        viewMenu.setMnemonic('v');
        viewMenu.setText("View");
        customizeMenuItem.setMnemonic('c');
        customizeMenuItem.setText("Customize...");
        customizeMenuItem.setToolTipText("Choose the filters you want to see");
        customizeMenuItem.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                customizeMenuItemActionPerformed(evt);
            }
        });

        viewMenu.add(customizeMenuItem);

        mainMenuBar.add(viewMenu);

        modeMenu.setMnemonic('m');
        modeMenu.setText("Mode");
        modeMenu.addMenuListener(new javax.swing.event.MenuListener() {
            public void menuCanceled(javax.swing.event.MenuEvent evt) {
            }
            public void menuDeselected(javax.swing.event.MenuEvent evt) {
            }
            public void menuSelected(javax.swing.event.MenuEvent evt) {
                modeMenuMenuSelected(evt);
            }
        });

        modeButtonGroup.add(renderingModeMenuItem);
        renderingModeMenuItem.setMnemonic('r');
        renderingModeMenuItem.setSelected(true);
        renderingModeMenuItem.setText("Process on rendering cycle");
        renderingModeMenuItem.setToolTipText("Process events on rendering cycle");
        renderingModeMenuItem.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                renderingModeMenuItemActionPerformed(evt);
            }
        });

        modeMenu.add(renderingModeMenuItem);

        modeButtonGroup.add(acquisitionModeMenuItem);
        acquisitionModeMenuItem.setMnemonic('a');
        acquisitionModeMenuItem.setText("Process on acqusition cycle");
        acquisitionModeMenuItem.setToolTipText("Process events on hardware data acquisition cycle");
        acquisitionModeMenuItem.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                acquisitionModeMenuItemActionPerformed(evt);
            }
        });

        modeMenu.add(acquisitionModeMenuItem);

        modeMenu.add(jSeparator1);

        measurePerformanceCheckBoxMenuItem.setText("Measure filter performance");
        measurePerformanceCheckBoxMenuItem.setToolTipText("Enables instrumentation of filter performance");
        measurePerformanceCheckBoxMenuItem.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                measurePerformanceCheckBoxMenuItemActionPerformed(evt);
            }
        });

        modeMenu.add(measurePerformanceCheckBoxMenuItem);

        modeMenu.add(jSeparator3);

        limitTimeCheckBoxMenuItem.setSelected(filterChain.isTimeLimitEnabled());
        limitTimeCheckBoxMenuItem.setText("Limit processing time");
        limitTimeCheckBoxMenuItem.setToolTipText("Filters that implement limiting will be aborted if they take too longer than the limit time");
        limitTimeCheckBoxMenuItem.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                limitTimeCheckBoxMenuItemActionPerformed(evt);
            }
        });

        modeMenu.add(limitTimeCheckBoxMenuItem);

        setTimeLimitMenuItem.setText(getTimeLimitMenuItemText());
        setTimeLimitMenuItem.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                setTimeLimitMenuItemActionPerformed(evt);
            }
        });

        modeMenu.add(setTimeLimitMenuItem);

        modeMenu.add(jSeparator4);

        restoreFilterEnabledStateCheckBoxMenuItem.setText("Restore filter enabled state");
        restoreFilterEnabledStateCheckBoxMenuItem.setToolTipText("If enabled, filter enabled state is restored on startup");
        restoreFilterEnabledStateCheckBoxMenuItem.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                restoreFilterEnabledStateCheckBoxMenuItemActionPerformed(evt);
            }
        });

        modeMenu.add(restoreFilterEnabledStateCheckBoxMenuItem);

        mainMenuBar.add(modeMenu);

        setJMenuBar(mainMenuBar);

        pack();
    }// </editor-fold>//GEN-END:initComponents

    private void disableFilteringToggleButtonActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_disableFilteringToggleButtonActionPerformed
        filterChain.setFilteringEnabled(disableFilteringToggleButton.isSelected());
    }//GEN-LAST:event_disableFilteringToggleButtonActionPerformed
    
    private void setTimeLimitMenuItemActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_setTimeLimitMenuItemActionPerformed
        String limitString=JOptionPane.showInputDialog("Choose new time limit in ms",filterChain.getTimeLimitMs());
        try{
            int val=Integer.valueOf(limitString);
            filterChain.setTimeLimitMs(val);
            setSetTimeLimitMenuItem();
        }catch(Exception e){
            log.warning(e.getMessage());
        }
    }//GEN-LAST:event_setTimeLimitMenuItemActionPerformed
    
    private void limitTimeCheckBoxMenuItemActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_limitTimeCheckBoxMenuItemActionPerformed
        filterChain.setTimeLimitEnabled(limitTimeCheckBoxMenuItem.isSelected());
    }//GEN-LAST:event_limitTimeCheckBoxMenuItemActionPerformed
    
    private void customizeMenuItemActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_customizeMenuItemActionPerformed
        filterChain.customize();
    }//GEN-LAST:event_customizeMenuItemActionPerformed
    
    private void restoreFilterEnabledStateCheckBoxMenuItemActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_restoreFilterEnabledStateCheckBoxMenuItemActionPerformed
        setRestoreFilterEnabledStateEnabled(restoreFilterEnabledStateCheckBoxMenuItem.isSelected());
    }//GEN-LAST:event_restoreFilterEnabledStateCheckBoxMenuItemActionPerformed
    
    // sets the acquisition mode filtering menu item enabled depending on whether device is attached.
    private void modeMenuMenuSelected(javax.swing.event.MenuEvent evt) {//GEN-FIRST:event_modeMenuMenuSelected
        // set the acquisition processing mode filter setting enabled only if we are live
        switch(chip.getAeViewer().getPlayMode()){
            case LIVE:
                acquisitionModeMenuItem.setEnabled(true);
                break;
            default:
                acquisitionModeMenuItem.setEnabled(false);
        }
    }//GEN-LAST:event_modeMenuMenuSelected
    
    private void exitMenuItemActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_exitMenuItemActionPerformed
        System.exit(0);
    }//GEN-LAST:event_exitMenuItemActionPerformed
    
    private void measurePerformanceCheckBoxMenuItemActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_measurePerformanceCheckBoxMenuItemActionPerformed
        filterChain.setMeasurePerformanceEnabled(measurePerformanceCheckBoxMenuItem.isSelected());
    }//GEN-LAST:event_measurePerformanceCheckBoxMenuItemActionPerformed
    
    private void acquisitionModeMenuItemActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_acquisitionModeMenuItemActionPerformed
        filterChain.setProcessingMode(FilterChain.ProcessingMode.ACQUISITION);
    }//GEN-LAST:event_acquisitionModeMenuItemActionPerformed
    
    private void renderingModeMenuItemActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_renderingModeMenuItemActionPerformed
        filterChain.setProcessingMode(FilterChain.ProcessingMode.RENDERING);
    }//GEN-LAST:event_renderingModeMenuItemActionPerformed
    
    /** renews contents by newing all filters, thus filling them with preference values. This is how preferences can replace values
     * without using extensive preference change listeners
     */
    public void renewContents(){
        filterChain.renewChain();
        rebuildContents();
        
    }
    
    // list of individual filter panels
    ArrayList<FilterPanel> filterPanels=new ArrayList<FilterPanel>();
    
    /** rebuilds the frame contents using the existing filters in the filterChain */
    public void rebuildContents(){
        filterPanels.clear();
        filtersPanel.removeAll();
        int n=0; int w=100, h=30;
//        log.info("rebuilding FilterFrame for chip="+chip);
//        if(true){ //(filterChain.size()<=MAX_ROWS){
//            filtersPanel.setLayout(new BoxLayout(filtersPanel,BoxLayout.Y_AXIS));
//            filtersPanel.removeAll();
        for(EventFilter2D f:filterChain){
            FilterPanel p=new FilterPanel(f);
            filtersPanel.add(p);
            filterPanels.add(p);
            n++; h+=p.getHeight(); w=p.getWidth();
        }
//            pack();
        pack();
//        else{
//            // multi column layout
//            scrollPane.removeAll();
//            scrollPane.setLayout(new BoxLayout(scrollPane, BoxLayout.X_AXIS));
//            int filterNumber=0;
//            JPanel panel=null;
//
//            for(EventFilter2D f:filterChain){
//                if(filterNumber%MAX_ROWS==0){
//                    panel=new JPanel();
//                    panel.setLayout(new BoxLayout(panel, BoxLayout.Y_AXIS));
//                    scrollPane.add(panel);
//                }
//                FilterPanel p=new FilterPanel(f);
//                panel.add(p);
//                filterPanels.add(p);
////                if((filterNumber+1)%MAX_ROWS==0){
////                         pad last panel with box filler at botton
////                        panel.add(Box.createVerticalGlue());
////                        System.out.println("filterNumber="+filterNumber);
////                }
//                filterNumber++;
//            }
//            pack();
//        }
    }
    
    File lastFile;
    
    private void loadMenuItemActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_loadMenuItemActionPerformed
        JFileChooser fileChooser=new JFileChooser();
        String lastFilePath=prefs.get("FilterFrame.lastFile",""); // get the last folder
        lastFile=new File(lastFilePath);
        XMLFileFilter fileFilter=new XMLFileFilter();
        fileChooser.addChoosableFileFilter(fileFilter);
        fileChooser.setCurrentDirectory(lastFile); // sets the working directory of the chooser
        int retValue=fileChooser.showOpenDialog(this);
        if(retValue==JFileChooser.APPROVE_OPTION){
            File f=fileChooser.getSelectedFile();
            loadFile(f);
        }
    }//GEN-LAST:event_loadMenuItemActionPerformed
    
    private void loadFile(File f){
        try{
            FileInputStream fis=new FileInputStream(f);
            prefs.importPreferences(fis);  // we import the tree into *this* preference node, which is not the one exported (which is root node)
            prefs.put("FilterFrame.lastFile",f.getCanonicalPath());
            log.info("imported preferences from "+f);
            recentFiles.addFile(f);
            renewContents();
        }catch(Exception e){
            e.printStackTrace();
        }
    }
    
    public boolean isRestoreFilterEnabledStateEnabled() {
        return restoreFilterEnabledStateEnabled;
    }
    
    public void setRestoreFilterEnabledStateEnabled(boolean restoreFilterEnabledStateEnabled) {
        this.restoreFilterEnabledStateEnabled = restoreFilterEnabledStateEnabled;
        prefs.putBoolean("FilterFrame.restoreFilterEnabledStateEnabled",restoreFilterEnabledStateEnabled);
        restoreFilterEnabledStateCheckBoxMenuItem.setSelected(restoreFilterEnabledStateEnabled);
    }
    
    private void saveAsMenuItemActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_saveAsMenuItemActionPerformed
        JFileChooser fileChooser=new JFileChooser();
        String lastFilePath=prefs.get("FilterFrame.lastFile",""); // get the last folder
        lastFile=new File(lastFilePath);
        XMLFileFilter fileFilter=new XMLFileFilter();
        fileChooser.addChoosableFileFilter(fileFilter);
        fileChooser.setCurrentDirectory(lastFile); // sets the working directory of the chooser
        fileChooser.setDialogType(JFileChooser.SAVE_DIALOG);
        fileChooser.setDialogTitle("Save filter settings to");
        fileChooser.setMultiSelectionEnabled(false);
//            if(lastImageFile==null){
//                lastImageFile=new File("snapshot.png");
//            }
//            fileChooser.setSelectedFile(lastImageFile);
        int retValue=fileChooser.showSaveDialog(this);
        if(retValue==JFileChooser.APPROVE_OPTION){
            try{
                File file=fileChooser.getSelectedFile();
                String suffix="";
                if(!file.getName().endsWith(".xml")) suffix=".xml";
                file=new File(file.getPath()+suffix);
                Preferences prefs=Preferences.userNodeForPackage(JAERViewer.class);
                FileOutputStream fos=new FileOutputStream(file);
                prefs.exportSubtree(fos);
//                String msg="exported filter preferences to "+file;
//                log.info(msg);
//                JOptionPane.showMessageDialog(this,msg);
                fos.close();
                log.info("exported preferences to "+file);
                recentFiles.addFile(file);
            }catch(Exception e){
                e.printStackTrace();
            }
        }
    }//GEN-LAST:event_saveAsMenuItemActionPerformed
    
    private void formComponentMoved(java.awt.event.ComponentEvent evt) {//GEN-FIRST:event_formComponentMoved
//        WindowSaver.saveWindowLocation(this,prefs);
    }//GEN-LAST:event_formComponentMoved
    
    private void formWindowClosing(java.awt.event.WindowEvent evt) {//GEN-FIRST:event_formWindowClosing
//        prefs.putInt("FilterFrame.XPosition", getX());
//        prefs.putInt("FilterFrame.YPosition", getY());
    }//GEN-LAST:event_formWindowClosing
    
    private void filterVisibleBiases(String string) {
        if(string==null || string.isEmpty()){
            for(FilterPanel p:filterPanels) p.setVisible(true);
        }else{
            for(FilterPanel p:filterPanels){
                String s=p.getFilter().getClass().getSimpleName().toUpperCase();
                string=string.toUpperCase();
                if(s.indexOf(string)!=-1) p.setVisible(true); else p.setVisible(false);
            }
        }
        validate();
    }
    
    // Variables declaration - do not modify//GEN-BEGIN:variables
    private javax.swing.JRadioButtonMenuItem acquisitionModeMenuItem;
    private javax.swing.JMenuItem customizeMenuItem;
    private javax.swing.JToggleButton disableFilteringToggleButton;
    private javax.swing.JMenuItem exitMenuItem;
    private javax.swing.JMenu fileMenu;
    private javax.swing.JPanel filtersPanel;
    private javax.swing.JSeparator jSeparator1;
    private javax.swing.JSeparator jSeparator2;
    private javax.swing.JSeparator jSeparator3;
    private javax.swing.JSeparator jSeparator4;
    private javax.swing.JToolBar jToolBar1;
    private javax.swing.JCheckBoxMenuItem limitTimeCheckBoxMenuItem;
    private javax.swing.JMenuItem loadMenuItem;
    private javax.swing.JMenuBar mainMenuBar;
    private javax.swing.JCheckBoxMenuItem measurePerformanceCheckBoxMenuItem;
    private javax.swing.ButtonGroup modeButtonGroup;
    private javax.swing.JMenu modeMenu;
    private javax.swing.JRadioButtonMenuItem renderingModeMenuItem;
    private javax.swing.JCheckBoxMenuItem restoreFilterEnabledStateCheckBoxMenuItem;
    private javax.swing.JMenuItem saveAsMenuItem;
    private javax.swing.JScrollPane scrollPane;
    private javax.swing.JMenuItem setTimeLimitMenuItem;
    private javax.swing.JPanel statusPanel;
    private javax.swing.JMenu viewMenu;
    // End of variables declaration//GEN-END:variables
    
}
