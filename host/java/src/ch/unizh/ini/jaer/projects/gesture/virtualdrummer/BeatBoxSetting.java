/*
 * Select the drum sounds for the VirtualDrummer.
 * @author Jun
 */

package ch.unizh.ini.jaer.projects.gesture.virtualdrummer;

import java.awt.*;
import javax.swing.*;
import javax.sound.midi.*;
import java.util.*;
import java.awt.event.*;
import java.io.*;
import java.beans.*;
/**
 *
 * @author Administrator
 */
public class BeatBoxSetting extends javax.swing.JFrame{
    //    JFrame bbsFrame;
    private DrumSounds drumSounds;

    // Names of instruments
    String[] instrumentNames = {"Bass Drum","Closed Hi-Hat","Open Hi-Hat",
                                "Acoustic Snare","Crash Cymbal","Hand Clap",
                                "High Tom","Hi Bongo","Maracas","Whistle",
                                "Low Conga","Cowbell","Vibraslap","Low-mid Tom",
                                "High Agogo","Open Hi Conga"};

    // Instruments
    int[] instruments={35,42,46,38,49,39,50,60,70,72,64,56,58,47,67,63};

    // Hashmap for instruments search
    HashMap<String, Integer> imap = new HashMap();

    public BeatBoxSetting(DrumSounds drumSounds) {
        setTitle("VirtualDrummer.BeatBoxSetting");

        ButtonGroup bgRight = new ButtonGroup();
        ButtonGroup bgLeft = new ButtonGroup();

        setLayout(new GridLayout(17,1));
        JPanel bpanel = new JPanel();
        bpanel.setLayout(new FlowLayout(FlowLayout.LEFT));
        Label leftLabel = new Label("Left");
        Label rightLabel = new Label("Right");
        bpanel.add(leftLabel);
        bpanel.add(rightLabel);
        add(bpanel);

        JRadioButton rb;
        for(int i=0;i<16;++i){
            bpanel = new JPanel();
            bpanel.setLayout(new FlowLayout(FlowLayout.LEFT));
            for(int j=0; j<2; j++)
            {
                if(j==0)
                {
                    rb = new JRadioButton();
                    rb.setName(new String("Left.").concat(instrumentNames[i]));
                    bgLeft.add(rb);
                }
                else
                {
                    rb = new JRadioButton(instrumentNames[i]);
                    rb.setName(new String("Right.").concat(instrumentNames[i]));
                    bgRight.add(rb);
                }
                bpanel.add(rb);
 
                // initial instrument
                if(i == 0)
                    rb.setSelected(true);

                rb.addItemListener(new BbsEventHandler());
                add(bpanel);

                imap.put(rb.getName(), new Integer(instruments[i]));
            }
        }

        setBounds(50,50,300,300);
        pack();

        this.drumSounds = drumSounds;
        drumSounds.setProgram(drumSounds.LEFT_BEATING, drumSounds.getDefaultProgram());
        drumSounds.setProgram(drumSounds.RIGHT_BEATING, drumSounds.getDefaultProgram());
   }

    public void showUp(){
        setVisible(true);
    }

    public void close(){
        setVisible(false);
    }

    class BbsEventHandler implements ItemListener{
        public void itemStateChanged(ItemEvent e){
            if(e.getStateChange() == ItemEvent.SELECTED)
            {
                JRadioButton rb = (JRadioButton) e.getSource();
                String iname = rb.getName();
                System.out.println(iname);
                if(iname.startsWith("Left"))
                    drumSounds.setProgram(drumSounds.LEFT_BEATING, imap.get(iname));
                else
                    drumSounds.setProgram(drumSounds.RIGHT_BEATING, imap.get(iname));
            }
        }
    }
}
