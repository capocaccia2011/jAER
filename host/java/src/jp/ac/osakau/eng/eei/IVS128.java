/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package jp.ac.osakau.eng.eei;

import net.sf.jaer.Description;
import net.sf.jaer.DevelopmentStatus;
import net.sf.jaer.aemonitor.AEPacketRaw;
import net.sf.jaer.chip.*;
import net.sf.jaer.event.EventPacket;
import net.sf.jaer.event.OutputEventIterator;
import net.sf.jaer.event.PolarityEvent;

/**
 * The intelligent vision sensor from the Yagi lab at Osaka University.
 * @author hiro okuno
 */
@Description("The \"Intelligent Vision Sensor\" from Osaka University")
@DevelopmentStatus(DevelopmentStatus.Status.Experimental)
public class IVS128 extends AEChip {

    public IVS128() {
        setSizeX(128);
        setSizeY(128);
        setNumCellTypes(4);
        setEventExtractor(new Extractor(this));
        setEventClass(IVS128Event.class);
        setName("IVS128");
    }

    @Override
    public void setSubSamplingEnabled(boolean subSamplingEnabled) {
        log.warning("setting subsampling has no effect on this chip");
    }
    
    

    /** the event extractor for IVS128. IVS128 has four cell types in its events. It also has analog retina output not handled yet here.
     * 
     */
    public class Extractor extends RetinaExtractor {

        final short XSHIFT = 8, YSHIFT = 16;

        public Extractor(IVS128 chip) {
            super(chip);
            setNumCellTypes(4);
            setEventClass(IVS128Event.class);
        }

        /** extracts the meaning of the raw events.
         *@param in the raw events, can be null
         *@return out the processed events. these are partially processed in-place. empty packet is returned if null is supplied as in. This event packet is reused
         * and should only be used by a single thread of execution or for a single input stream, or mysterious results may occur!
         */
        @Override
        synchronized public EventPacket extractPacket(AEPacketRaw in) {
            if (out == null) {
                out = new EventPacket<IVS128Event>(chip.getEventClass());
            } else {
                out.clear();
            }
            extractPacket(in, out);
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
        @Override
        synchronized public void extractPacket(AEPacketRaw in, EventPacket out) {

            if (in == null) {
                return;
            }
            int n = in.getNumEvents(); //addresses.length;

            int[] a = in.getAddresses();
            int[] timestamps = in.getTimestamps();
            OutputEventIterator outItr = out.outputIterator();
            for (int i = 0; i < n; i ++) { // TODO bug here?
                int addr = a[i];

                if ((addr & 0x8) != 0) {
                    IVS128Event e = (IVS128Event) outItr.nextOutput();
                    e.type = (byte) 3;
                    e.x = (short) ((addr >>> XSHIFT) & 0x7F);
                    e.y = (short) ((addr >>> YSHIFT) & 0x7F);
                    e.timestamp = (timestamps[i]);

                }
                if ((addr & 0x4) != 0) {
                    IVS128Event e = (IVS128Event) outItr.nextOutput();
                    e.type = (byte) 2;
                    e.x = (short) ((addr >>> XSHIFT) & 0x7F);
                    e.y = (short) ((addr >>> YSHIFT) & 0x7F);
                    e.timestamp = (timestamps[i]);

                }
                if ((addr & 0x2) != 0) {
                    IVS128Event e = (IVS128Event) outItr.nextOutput();
                    e.type = (byte) 1;
                    e.x = (short) ((addr >>> XSHIFT) & 0x7F);
                    e.y = (short) ((addr >>> YSHIFT) & 0x7F);
                    e.timestamp = (timestamps[i]);

                }
                if ((addr & 0x1) != 0) {
                    IVS128Event e = (IVS128Event) outItr.nextOutput();
                    e.type = (byte) (1 - addr & 1);
                    e.x = (short) ((addr >>> XSHIFT) & 0x7F);
                    e.y = (short) ((addr >>> YSHIFT) & 0x7F);
                    e.timestamp = (timestamps[i]);

                }

            }
        }
    }
}
