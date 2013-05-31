## API calls ##
import serial
import io
import time

class GardenSensor():
    baud = 19200
    port = 'COM5'
    bytes = 8
    
    def __init__( self, port = 2 ):
        self.port = port
        self.isOpen = False
        return
    
    ## Open and initize the serial port ##
    def start( self ):
        if not self.isOpen :
            self.ser = serial.Serial()
            self.ser.baudrate = self.baud
            self.ser.port = self.port
            self.ser.timeout = 2
            self.ser.open()
            print( "Opened port." )
            self.sio = io.TextIOWrapper( io.BufferedRWPair( self.ser, self.ser ) ) # TextWrapper
            self.isOpen = True
            return
    
    # Loop readings to a list
    def acquire ( self, lines = 1 ):
        try:
            result = []
            for line in range( 1, lines ):
                output = self.read( bytes )
                result.append( output.decode( 'ascii' ).strip().split( ' ' ) )
                
        finally:
            return result
            
    ## Write ##
    def write( self, command ):
        self.start()
        time.sleep( 0.1 )
        self.sio.write( command + str( "\n" ) )
        self.sio.flush()
        print( "{0:10}{1}".format( "Sent:", command ) )
        return

    ## Read ##
    def read( self, byte ):
        self.start()
        r = self.ser.read( byte )
        print( "{0:10}{1}".format( "Read:", r ) )
        return r    

    ## Close the serial port ##
    def close( self ):
        self.ser.close()
        self.isOpen = False
        print( "Closed port." )
        return