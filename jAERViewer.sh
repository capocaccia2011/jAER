# this script starts JAERViewer under linux

if [ -f /proc/bus/usb/devices ]; 
then
	echo startup jAER...
	# cd to the root folder of jAER
	cd `dirname $0`
	#cd to host/java so that relative paths work
	cd host/java
	# -DJOGL_SINGLE_THREADED_WORKAROUND=false
	# run main class using paths relative to this path
	java -Djava.util.logging.config.file=conf/Logging.properties -Djava.library.path=jars:jars\SiLabsNativeWindows -Dsun.java2d.opengl=false -Dsun.java2d.noddraw=true -cp dist/jAER.jar:jars/spread.jar:jars/UsbIoJava.jar:jars/swing-layout-0.9.jar:jars/jogl.jar:jars/comm.jar:jars/gluegen-rt.jar:jars/jsr80-1.0.1.jar:jars/jsr80_linux-1.0.1.jar:jars/jsr80_ri-1.0.1.jar:jars ch.unizh.ini.caviar.JAERViewer 
else
	echo "please run 'sudo mount -t usbfs usbfs /proc/bus/usb -o devmode=0666' before running this script for the first time. this will install the usbfs filesystem needed by the linux driver'. note, that with this you allow every user to access your usb ports (devmode=0666)"
fi

