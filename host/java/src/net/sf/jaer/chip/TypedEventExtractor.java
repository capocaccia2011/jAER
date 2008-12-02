/*
 * RetinaEventExtractor.java
 *
 * Created on October 21, 2005, 6:24 PM
 *
 * To change this template, choose Tools | Options and locate the template under
 * the Source Creation and Management node. Right-click the template and choose
 * Open. You can then make changes to the template in the Source Editor.
 */

package net.sf.jaer.chip;

import net.sf.jaer.aemonitor.*;
import net.sf.jaer.event.*;
import net.sf.jaer.event.EventPacket;
import net.sf.jaer.event.BasicEvent;
import net.sf.jaer.event.OutputEventIterator;
import java.util.logging.Logger;

/**
 * An abstract 2D event extractor for 16 bit raw addresses. It is called with addresses and timestamps and extracts 
 * these to {X, Y, type} arrays based on methods that you define by subclassing and overriding the abstract methods. 
 * 
 * xMask, 
 * yMask, typeMask mask for x, y address and cell type, and xShift, yShift, typeShift say how many bits to shift after
 * masking, xFlip,yFlip,typeFlip use the chip size to flip the x,y, and type to invert the addresses.
 *
 * @author tobi
 */
abstract public class TypedEventExtractor<T extends BasicEvent> implements EventExtractor2D, java.io.Serializable {
    static Logger log=Logger.getLogger("net.sf.jaer.chip");
    
//    protected AEChip chip;
    protected int xmask,ymask;
    protected byte xshift,yshift;
    protected int typemask;
    protected byte typeshift;
    protected AEChip chip=null;
    protected boolean flipx=false, flipy=false, rotate=false;
    protected boolean fliptype=false;
    protected boolean hexArrangement=false;
    // the reused output eventpacket
    protected EventPacket out=null;
//    AEPacket2D outReused=new AEPacket2D();
    Class eventClass=TypedEvent.class;
    
    /** determines subSampling of rendered events (for speed) */
    private boolean subsamplingEnabled=false;
    
    private int subsampleThresholdEventCount=50000;
    
    private short sizex,sizey; // these are size-1 (e.g. if 128 pixels, sizex=127). used for flipping below.
    private byte sizetype;
    
//    /** Creates a new instance of RetinaEventExtractor */
//    public TypedEventExtractor() {
//    }
    
    /** Creates a new instance of RetinaEventExtractor
     * @param aechip the chip this extractor extracts
     */
    public TypedEventExtractor(AEChip aechip) {
        chip=aechip;
        setchipsizes();
    }
    
    private void setchipsizes(){
        sizex=(short)(chip.getSizeX()-1);
        sizey=(short)(chip.getSizeY()-1);
        sizetype=(byte)(chip.getNumCellTypes()-1);
    }
    
    /** 
     * Gets X from raw address. declared final for speed, cannot be overridden in subclass. It expects addr to be from a certain
     * format like ???? XXX? X?X? ????. The mask 0000 1110 1010 0000 will get those Xs. So, what's left is 0000 XXX0 X0X0 0000. 
     * Then it is shifted (the triple means zero-padding regardless of sign, necessary because addr is not defined as unsigned!)
     * with the xshift parameter. So, in this case e.g. 5, what will make the result 0000 0000 0XXX 0X0X (0-64). If the incoming 
     * addr is big-endian versus small-endian the entire bit sequence has to be reversed of course. This is not what is done by
     * flipx! That is just a boolean that results in the chip size minus the result otherwise. So, in the above example the 
     * place for X is still expected in the same spots (the mask is not inverted). Last remark: the masks are normally continuous,
     * no weird blanks within them. :-) Ultimate remark: a mask of size 1 will invert the value if the flip parameter is set.
     * Super-last remark: the parameter sizex should be 7 (not 8) for a mask of size 3 (000 - 111), like mentioned before. So,
     * the sizex and sizey parameters in this TypedEventExtractor class are decremented with one compared to the same parameters
     * in the class AEChip.
     * 
     *@param addr the raw address.
     *@return physical address
     */
    public short getXFromAddress(int addr){
        if(!flipx) return ((short)((addr&xmask)>>>xshift));
        else return (short)(sizex - ((int)((addr&xmask)>>>xshift))); // e.g. chip.sizex=32, sizex=31, addr=0, getX=31, addr=31, getX=0
    }
    
    /** gets Y from raw address. declared final for speed, cannot be overridden in subclass.
     *@param addr the raw address.
     *@return physical address
     */
    public short getYFromAddress(int addr){
        if(!flipy) return ((short)((addr&ymask)>>>yshift));
        else return (short)(sizey-((int)((addr&ymask)>>>yshift)));
        
    }
    /** gets type from raw address. declared final for speed, cannot be overridden in subclass.
     *@param addr the raw address.
     *@return physical address
     */
    public byte getTypeFromAddress(int addr){
        if(!fliptype) return (byte)((addr&typemask)>>>typeshift);
        else return (byte)(sizetype-(byte)((addr&typemask)>>>typeshift));
    }
    
    
    
    /** extracts the meaning of the raw events.
     *@param in the raw events, can be null
     *@return out the processed events. these are partially processed in-place. empty packet is returned if null is supplied as in.
     */
    synchronized public EventPacket extractPacket(AEPacketRaw in) {
        if(out==null){
            out=new EventPacket<T>(chip.getEventClass());
        }else{
            out.clear();
        }
        if(in==null) return out;
        extractPacket(in,out);
        return out;
    }
    
    /**
     * Extracts the meaning of the raw events. This form is used to supply an output packet. This method is used for real time
     * event filtering using a buffer of output events local to data acquisition. An AEPacketRaw may contain multiple events, 
     * not all of them have to sent out as EventPackets. An AEPacketRaw is a set(!) of addresses and corresponding timing moments.
     * 
     * A first filter (independent from the other ones) is implemented by subSamplingEnabled and getSubsampleThresholdEventCount. 
     * The latter may limit the amount of samples in one package to say 50,000. If there are 160,000 events and there is a sub sample 
     * threshold of 50,000, a "skip parameter" set to 3. Every so now and then the routine skips with 4, so we end up with 50,000.
     * It's an approximation, the amount of events may be less than 50,000. The events are extracted uniform from the input. 
     * 
     * @param in 		the raw events, can be null
     * @param out 		the processed events. these are partially processed in-place. empty packet is returned if null is
     * 					supplied as input.
     */
    synchronized public void extractPacket(AEPacketRaw in, EventPacket out) {
        out.clear();
        if(in==null) return;
        int n=in.getNumEvents(); //addresses.length;
       
        int skipBy=1, incEach = 0, j = 0;
        if(subsamplingEnabled){
            skipBy = n/getSubsampleThresholdEventCount();
            incEach = getSubsampleThresholdEventCount() / (n % getSubsampleThresholdEventCount());
        }
        if (skipBy==0) {incEach = 0; skipBy = 1; }

        int[] a=in.getAddresses();
        int[] timestamps=in.getTimestamps();
        boolean hasTypes=false;
        if(chip!=null) hasTypes=chip.getNumCellTypes()>1;
       
        OutputEventIterator<?> outItr=out.outputIterator();
        for(int i=0; i<n; i+=skipBy){
            int addr=a[i];
            BasicEvent e=(BasicEvent)outItr.nextOutput();
            e.timestamp=(timestamps[i]);
            e.x=getXFromAddress(addr);
            e.y=getYFromAddress(addr);
            if(hasTypes){
                ((TypedEvent)e).type=getTypeFromAddress(addr);
            }
            j++; if (j==incEach) {j=0; i++;}
//            System.out.println("a="+a[i]+" t="+e.timestamp+" x,y="+e.x+","+e.y);
        }
    }

/*    synchronized public void extractPacket(AEPacketRaw in, EventPacket out) {
        out.clear();
        if(in==null) return;
        int n=in.getNumEvents(); //addresses.length;
        
        int skipBy=1;
        if(subsamplingEnabled){
            while(n/skipBy>getSubsampleThresholdEventCount()){
                skipBy++;
            }
        }
        int[] a=in.getAddresses();
        int[] timestamps=in.getTimestamps();
        boolean hasTypes=false;
        if(chip!=null) hasTypes=chip.getNumCellTypes()>1;
        
        OutputEventIterator outItr=out.outputIterator();
        for(int i=0; i<n; i+=skipBy){ // bug here? no, bug is before :-)
            int addr=a[i];
            BasicEvent e=(BasicEvent)outItr.nextOutput();
            e.timestamp=(timestamps[i]);
            e.x=getXFromAddress(addr);
            e.y=getYFromAddress(addr);
            if(hasTypes){
                ((TypedEvent)e).type=getTypeFromAddress(addr);
            }
//            System.out.println("a="+a[i]+" t="+e.timestamp+" x,y="+e.x+","+e.y);
        }
    }
*/        
    public int getTypemask() {
        return this.typemask;
    }
    
    public void setTypemask(final int typemask) {
        this.typemask = typemask;
    }
    
    public byte getTypeshift() {
        return this.typeshift;
    }
    
    public void setTypeshift(final byte typeshift) {
        this.typeshift = typeshift;
    }
    
    public int getXmask() {
        return this.xmask;
    }
    
    /** bit mask for x address, before shift */
    public void setXmask(final int xmask) {
        this.xmask = xmask;
    }
    
    public byte getXshift() {
        return this.xshift;
    }
    
    /** @param xshift the number of bits to right shift raw address after masking with {@link #setXmask} */
    public void setXshift(final byte xshift) {
        this.xshift = xshift;
    }
    
    public int getYmask() {
        return this.ymask;
    }
    
    /** @param ymask the bit mask for y address, before shift */
    public void setYmask(final int ymask) {
        this.ymask = ymask;
    }
    
    public byte getYshift() {
        return this.yshift;
    }
    
    /** @param yshift the number of bits to right shift raw address after masking with {@link #setYmask} */
    public void setYshift(final byte yshift) {
        this.yshift = yshift;
    }
    
//    short clipx(short v){ short c=(short) (v>(chip.sizeX-1)? chip.sizeX-1: v); c=c<0?0:c; return c;}
//    short clipy(short v){ short c= (short)(v>(chip.sizeY-1)? chip.sizeY-1: v); c=c<0?0:c; return c;}
    
    
    public boolean isFlipx() {
        return this.flipx;
    }
    
    public void setFlipx(final boolean flipx) {
        if(chip.getSizeX()==1){
            log.warning("setFlipx for chip"+chip+": chip sizeX=1, flipping doesn't make sense, disabling");
            this.flipx=false;
            return;
        }
        this.flipx = flipx;
    }
    
    public boolean isFlipy() {
        return this.flipy;
    }
    
    public void setFlipy(final boolean flipy) {
        if(chip.getSizeY()==1){
            log.warning("setFlipy for chip"+chip+": chip sizeY=1, flipping doesn't make sense, disabling");
            this.flipy=false;
            return;
        }
        this.flipy = flipy;
    }
    
    public boolean isFliptype() {
        return this.fliptype;
    }
    
    public void setFliptype(final boolean fliptype) {
        if(chip.getNumCellTypes()==1){
            log.warning("setFliptype for chip"+chip+": chip numTypes=1, flipping doesn't usually make sense, will treat it to make type=1 instead of default 0");
//            this.fliptype=false;
//            return;
        }
        this.fliptype = fliptype;
    }
    
    public int getUsedBits() {
        return (int)((xmask<<xshift+ymask<<yshift+typemask<<typeshift));
    }
    
    public boolean matchesAddress(int addr1, int addr2){
        return (addr1&getUsedBits())==(addr2&getUsedBits());
    }
    
    /** computes the raw address from an x,y, and type. Useful for searching for events in e.g. matlab, given the raw addresses.
     *This function does include flipped addresses - it uses flip booleans to pre-adjust x,y,type for chip.
     *@param x the x address
     *@param y the y address
     *@param type the cell type
     *@return the raw address
     */
    public int getAddressFromCell(int x, int y, int type) {
        if(flipx) x=sizex-x;
        if(flipy) y=sizey-y;
        if(fliptype) type=sizetype-type;
        
        return (int)(
                (x<<xshift)
                +(y<<yshift)
                +(type<<typeshift)
                );
    }
    
    public boolean isSubsamplingEnabled() {
        return subsamplingEnabled;
    }
    
    public void setSubsamplingEnabled(boolean subsamplingEnabled) {
        this.subsamplingEnabled = subsamplingEnabled;
    }
    
    public int getSubsampleThresholdEventCount() {
        return subsampleThresholdEventCount;
    }
    
    public void setSubsampleThresholdEventCount(int subsampleThresholdEventCount) {
        this.subsampleThresholdEventCount = subsampleThresholdEventCount;
    }
    
    AEPacketRaw raw=null;
    
    /** 
     * Reconstructs a raw packet suitable for logging to a data file, from an EventPacket that could be the result of filtering 
     * operations
     * @param packet the EventPacket
     * @return a raw packet holding the device events
     */
    public AEPacketRaw reconstructRawPacket(EventPacket packet) {
        if(raw==null) raw=new AEPacketRaw();
        raw.setNumEvents(0);
        EventRaw r=new EventRaw();
        for(Object o:packet){
            TypedEvent e=(TypedEvent)o;
            r.timestamp=e.timestamp;
            r.address=getAddressFromCell(e.x,e.y,e.type);
            raw.addEvent(r);
        }
        return raw;
    }    
}