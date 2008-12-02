/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package ch.unizh.ini.jaer.projects.pencilbalancer;

//import java.util.LinkedList;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.logging.Logger;
import net.sf.jaer.util.TobiLogger;

/**
 * Manages connection to servo via RXTX library.
 * @author conradt
 */
public class ServoConnection extends Thread {

    static Logger log = Logger.getLogger("ServoConnection");
    private HWP_RS232 rs232Port = null;
    private boolean isRunning = true;
//    private String updateCommand;
//    private LinkedList<String> cmdListToSend;
    private String received;
    TobiLogger tobiLogger=null;
    private boolean enableLogging = false;
    ArrayBlockingQueue<String> queue=new ArrayBlockingQueue<String>(1);
    private int portNumber=3;

    /** Constructs a new ServoConnection to a specified COM port number
     
     @param comPort, e.g. 3 for COM3
     */
    public ServoConnection(int comPort) {
        setName("ServoConnection");
        setPriority(MAX_PRIORITY);
        setPortNumber(comPort);
        log.info("Starting consumer thread for connection to servo board");
        
        this.start();

//        updateCommand = null;
//        cmdListToSend = new LinkedList<String>();
        received = null;
    }

    public void run() {
        isRunning = true;

        setPriority(MAX_PRIORITY);

        connectServo();

        while (isRunning) {

            try{
                String s=queue.take();
                 rs232Port.sendCommand(s);
                rs232Port.flushOutput();
                if(enableLogging && tobiLogger!=null) tobiLogger.log("");
            }catch(InterruptedException e){
                log.info("queue interrupted: "+e);
                break;
            }
//            yield();

//            if (updateCommand != null) {
//                String s=updateCommand;
//                updateCommand = null;
//                rs232Port.sendCommand(s);
//                rs232Port.flushOutput();
//                if(enableLogging && tobiLogger!=null) tobiLogger.log("");
//            }
//            if (cmdListToSend.isEmpty() == false) {
//                String s;
//                synchronized (cmdListToSend) {
//                    s = cmdListToSend.removeFirst();
//                }
//                rs232Port.sendCommand(s);
//                rs232Port.flushOutput();
//            }

//            String r = rs232Port.readLine();
//            if (r != null) {
//                received=r;
//            }
        }

        if (rs232Port != null) {
            rs232Port.sendCommand("-");
            rs232Port.sendCommand("!D-");  // turn off debug output
            rs232Port.flushOutput();
            rs232Port.close();
        }
        log.info("diconnecting!");
    }

    public void terminate() {
        isRunning = false;
    }

    private void connectServo() {

        rs232Port = new HWP_RS232();

        HWPort.PortIdentifier thisPI = null;
        HWPort.PortAttribute thisPA = null;

        for (HWPort.PortIdentifier pi : rs232Port.getPortIdentifierList()) {
            if ((pi.display).equals("  COM"+portNumber)) {
                thisPI = pi;
            }
        }
        for (HWPort.PortAttribute pa : rs232Port.getPortAttributeList()) {
            if ((pa.display).equals("  2000000Bd")) {
                thisPA = pa;
            }
        }

        if ((thisPI != null) && (thisPA != null)) {
            log.info("Opening Port " + thisPI.getID() + " with attribute " + thisPA);
            rs232Port.open(((String) thisPI.getID()), thisPA);
            rs232Port.setHardwareFlowControl(true);

            rs232Port.sendCommand("");     // send a dummy return to clear pending input
//          rs232Port.sendCommand("!D5");  // !Dx means enable debug output every 2ms * x
//          rs232Port.sendCommand("!D+");  // enable debug output
            rs232Port.sendCommand("");     // send a dummy return to clear pending input
            rs232Port.sendCommand("+");    // enable servo control
//          rs232Port.sendCommand("!S+2"); // reply only errors and debug output
            rs232Port.sendCommand("!S+3"); // reply only errors and debug output, reply only on request ("?xxx")

        } else {
            log.warning("\nError, could not find proper port or baud-rate... terminating!\n");
            isRunning = false;
        }
    }

    public void sendUpdate(String command) {
        queue.clear();
        queue.offer(command);
//        updateCommand = command;
    }
//    public void XsendCommand(String command) {
//        synchronized (cmdListToSend) {
//            cmdListToSend.add(command);
//        }
//    }

    public String readLine() {
        String r = received;
        received = null;
        return (r);
    }
    
        public boolean isEnableLogging() {
        return enableLogging;
    }
    synchronized public void setEnableLogging(boolean enableLogging) {
        this.enableLogging = enableLogging;
        if (!enableLogging) {
            if (tobiLogger != null) {
                tobiLogger.setEnabled(false);
            }

        } else {
            if (tobiLogger == null) {
                tobiLogger = new TobiLogger("ServoConnection", "nanoseconds cmd sent"); // fill in fields here to help consumer of data
                tobiLogger.setNanotimeEnabled(true);
                tobiLogger.setAbsoluteTimeEnabled(false);
            }

            tobiLogger.setEnabled(true);
        }
    }

    public int getPortNumber() {
        return portNumber;
    }

    /** Sets the COM port number that we connect to, e.g. use 3 for COM3.
     * 
     * @param portNumber
     */
    public void setPortNumber(int portNumber) {
        this.portNumber = portNumber;
    }
}
