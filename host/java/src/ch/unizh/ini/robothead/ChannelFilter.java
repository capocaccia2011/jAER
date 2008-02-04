/*
 * ChannelFilter.java
 *
 * Created on 29. Januar 2008, 10:26
 *
 * To change this template, choose Tools | Template Manager
 * and open the template in the editor.
 */

package ch.unizh.ini.robothead;


import ch.unizh.ini.caviar.chip.*;
import ch.unizh.ini.caviar.event.*;
import ch.unizh.ini.caviar.event.EventPacket;
import ch.unizh.ini.caviar.eventprocessing.*;
import ch.unizh.ini.caviar.eventprocessing.EventFilter2D;
//import com.sun.org.apache.xpath.internal.operations.Mod;
import java.util.*;
/**
 *
 * @author jaeckeld
 */

public class ChannelFilter extends EventFilter2D implements Observer{
   
    private int chMin=getPrefs().getInt("ChannelFilter.chMin",1);
    private int chMax=getPrefs().getInt("ChannelFilter.chMax",32);
    
    
    /** Creates a new instance of ChannelFilter */
    public ChannelFilter(AEChip chip) {
        super(chip);
        //initFilter();
        //resetFilter();
        setPropertyTooltip("chMin", "highest Channel");
        setPropertyTooltip("chMax", "lowest Channel");
        
    
    }
        
    public EventPacket<?> filterPacket(EventPacket<?> in) {
        if(!isFilterEnabled()){
            //System.out.print("TEST 2");
            return in;       // only use if filter enabled
        }
       checkOutputPacketEventType(in);
           
       OutputEventIterator outItr=out.outputIterator();
             
       for(Object e:in){
        
            BasicEvent i =(BasicEvent)e;
       
            //System.out.println(i.timestamp+" "+i.x+" "+i.y);
            
            if (i.x>=this.chMin-1 && i.x<=this.chMax-1){
                
                BasicEvent o=(BasicEvent)outItr.nextOutput();   //provide filtered Output!!
                o.copyFrom(i);
            }
        
       }     
        return out;
        
    }
   
    public Object getFilterState() {
        return null;
    }
    public void resetFilter(){
        System.out.println("reset!");
        
        System.out.println(this.getChMax());
        System.out.println(this.getChMin());
        
        
    }
    public void initFilter(){
        System.out.println("init!");
        
        
    }
    
    public void update(Observable o, Object arg){
        initFilter();
    }
    
    public int getChMin(){
        return this.chMin;   
    }
    public void setChMin(int chMin){
        getPrefs().putInt("ChannelFilter.chMin",chMin);
        support.firePropertyChange("chMin",this.chMin,chMin);
        this.chMin=chMin;
        
    }

    public int getChMax(){
        return this.chMax;   
    }
    public void setChMax(int chMax){
        getPrefs().putInt("ChannelFilter.chMax",chMax);
        support.firePropertyChange("chMax",this.chMax,chMax);
        this.chMax=chMax;
        
    }
}
    