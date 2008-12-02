/*
 * DATFileFilter.java
 *
 * Created on September 26, 2005, 3:37 PM
 */

package net.sf.jaer.util;

import net.sf.jaer.eventio.AEDataFile;
import java.io.*;

/**
 * filter for DAT ae event data files.
 * @author tobi
 */
public class DATFileFilter extends javax.swing.filechooser.FileFilter {
    
    /** Creates a new instance of XMLFileFilter */
    public DATFileFilter() {
    }
    
    public boolean accept(File f) {
        if (f.isDirectory()) {
            return true;
        }
        
        String extension = getExtension(f);
        if (extension != null) {
            if (extension.equals(EXTENSION)){
                return true;
            } else {
                return false;
            }
        }
        return true;
    }
    
    public static String getExtension(File f) {
        String ext = null;
        String s = f.getName();
        int i = s.lastIndexOf('.');
        
        if (i > 0 &&  i < s.length() - 1) {
            ext = s.substring(i+1).toLowerCase();
        }
        return ext;
    }

    public String getDescription() {
        return "DAT AER raw binary data file";
    }
    
    /** The extension, including the dot, ".xml" 
     **/
    public static final String EXTENSION;
    static{
        EXTENSION=AEDataFile.DATA_FILE_EXTENSION.substring(AEDataFile.DATA_FILE_EXTENSION.lastIndexOf(".")+1,AEDataFile.DATA_FILE_EXTENSION.length());
    }

    
}