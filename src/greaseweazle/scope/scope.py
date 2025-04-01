'''
Query a siglent SDS804X HD scope over TCP
Channel 1 and 2 are hooked to test points on the floppy controller of a panasonic ju-475-4, tracing the outputs of the heads (preamplified)
Channel 3 is on the index signal test point. 
'''

import socket # for sockets
import sys # for exit
import time # for sleep
import numpy as np
import scipy as sci
import scipy.signal as sig
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt
from datetime import datetime
from greaseweazle.flux import Flux, HasFlux
import scipy

from pyvisa import ResourceManager
from pyvisa.resources import MessageBasedResource
from pyvisa.util import BINARY_HEADERS


class Scope():
    _ip: str = "127.0.0.1"
    _port: int = 5025 # the port number of the instrument service
    _resource = None

    def __init__(self, ip :str, port :int = 5025):
        self._ip = ip
        self._port = port
        #self.socketConnect()
        rm = ResourceManager()
        res = rm.open_resource("TCPIP0::"+ip+"::inst0::INSTR")
        if isinstance(res, MessageBasedResource):
            self._resource = res
        else:
            raise TypeError("Selected resource isn't a Message-based resource")
        
    def identifier(self) -> str:
        """Get the unique identifier of the deivce."""
        return self._resource.query("*IDN?")

    def reset(self):
        """Reset the instrument to a factory defined condition."""
        self._resource.write("*RST")

    def clear(self):
        """Clear the instrument status byte."""
        self._resource.write("*CLS")

    def block_until_complete(self):
        """Block the runtime until the instrument has finished all prior operations."""
        assert (
            self._resource.query("*OPC?").strip() == "1"
        ), "*OPC? returned something unexpected"

#    def socketConnect(self) -> socket.socket:
#        try:
#            #create an AF_INET, STREAM socket (TCP)
#            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#        except socket.error:
#            print ('Failed to create socket.')
#            sys.exit();
#        try:
#            #Connect to remote server
#            s.connect((self._ip , self._port))
#            s.setblocking(0) # non-blocking mode, an exception occurs when no data is detected by the receiver
#            #s.settimeout(3) 
#        except socket.error:
#             raise IOError('failed to connect to ip ' + self._ip)
#        self._socket = s
    
    def socketQuery(self, cmd):
        return self._resource.query(cmd)
        #try :
        #    #Send cmd string
        #    self._socket.sendall(bytes(cmd, "utf-8")+b'\n') 
        #    time.sleep(0.5)
        #except socket.error:
        #    #Send failed
        #    raise IOError('Send to scope failed')
        # 
        #data_body = bytes() 
        #while True:
        #    try:
        #        time.sleep(0.0005) # wait a tiny bit
        #        server_replay = self._socket.recv(1024*128)
        #        #print(len(server_replay))
        #        data_body += server_replay
        #    except BlockingIOError:
        #        #print("data received complete..")
        #        break
        #return data_body

    def socketCmd(self, cmd: str):
        self._resource.write(cmd)
        self.block_until_complete
        #try :
        #    #Send cmd string
        #    self._socket.sendall(bytes(cmd, "utf-8")+b'\n')
        #    time.sleep(0.3)
        #except socket.error:
        #    #Send failed
        #    raise IOError('Send to scope failed')
    
#    def socketClose(self):
#        if self._socket is not None:
#            #close the socket
#            self._socket.close()
#            self._socket = None
#            time.sleep(0.5)


    def setup(self):
#        # Body: Open a socket, query and save the waveform files
#        if self._socket is None:
#            self.socketConnect()
        
        # first: turn on auto mode so adjustments can be made
        self.socketCmd(":RUN")
        self.socketCmd(":TRIG:MODE AUTO")    # trigger mode auto
                
        self.socketCmd(":ACQ:TYP NORM")  
        self.socketCmd(":ACQ:MDEP 10M")  # keep 10Mio samples
        self.socketCmd(":ACQ:MODE YT") # YT mode
        self.socketCmd(":TIM:SCAL 0.1")  # set horizontal divider to 100ms
        # set up channels
        for chan in ["1","2"]:
            self.socketCmd(":CHAN"+chan+":BWL 20M")       # low pass filter 20 MHZ
            self.socketCmd(":CHAN"+chan+":COUP DC")       # DC Coupling
            self.socketCmd(":CHAN"+chan+":PROB 1")        # probe is at 1x
            self.socketCmd(":CHAN"+chan+":SCAL 0.5E-1")     # scale at 50mV/divider
            self.socketCmd(":CHAN"+chan+":INV OFF")       # no inverting
            self.socketCmd(":CHAN"+chan+":VIS ON")        # turn on display
            self.socketCmd(":CHAN"+chan+":SWIT ON")       # turn on channel
            self.socketCmd(":CHAN"+chan+":OFFS -2.775E+0")     # offset by -2.775V

        for chan in ["3"]:
            self.socketCmd(":CHAN"+chan+":BWL 20M")       # low pass filter 20 MHZ
            self.socketCmd(":CHAN"+chan+":COUP DC")       # DC Coupling
            self.socketCmd(":CHAN"+chan+":PROB 10")       # probe is at 10x
            self.socketCmd(":CHAN"+chan+":SCAL 1.0E+0")      # scale at 1V/divider
            self.socketCmd(":CHAN"+chan+":INV OFF")       # no inverting
            self.socketCmd(":CHAN"+chan+":VIS ON")        # turn on display
            self.socketCmd(":CHAN"+chan+":SWIT ON")       # turn on channel
            self.socketCmd(":CHAN"+chan+":OFFS -2.5E+0")     # offset by -2.8V

        #self.socketCmd(":CHAN4:VIS OFF")        # turn off channel4
        self.socketCmd(":CHAN4:SWIT OFF")       # turn off channel4

        self.socketCmd(":FUNC1 OFF") # turn off functions
        self.socketCmd(":FUNC2 OFF")
        self.socketCmd(":FUNC3 OFF")
        self.socketCmd(":FUNC4 OFF")
        
        # TRIGGER
        self.socketCmd(":TRIG:MODE AUTO")    # trigger mode auto
        self.socketCmd(":TRIG:TYPE EDGE")   
        self.socketCmd(":TRIG:EDGE:COUP DC") # trigger Coupling DC
        self.socketCmd(":TRIG:EDGE:SOUR C3") # trigger source Channel3
        self.socketCmd(":TRIG:EDGE:SLOP FALL") # trigger on falling edge
        self.socketCmd(":TRIG:EDGE:LEV 0.7")  # trigger level 0.7V
        #self.socketCmd(":SAVE:DEFault CUSTom") # store this as the default settings

    def query(self, base_filename:str = None) -> list:
        '''
        starts a query on the scope and retrieves the raw scope data.
        Data will additionally be saved into files for later analysis if base_filename is given
        return value will be the raw data for the three channels.
        '''
        #if self._socket is None:
        #    self.socketConnect()

        # run and fill the buffer
        self.socketCmd(":RUN")
        time.sleep(0.5)
        
        # run in single trigger mode 
        self.socketCmd(":TRIG:MODE SING")     # trigger mode "SINGLE"
        time.sleep(2)
        self.socketCmd(":STOP")
        #time.sleep(0.5)

        self.socketCmd(":WAVeform:BYTeorder LSB") 
        self.socketCmd(":WAVeform:INTerval 1") 
        self.socketCmd(":WAVeform:WIDTh WORD") 
        bytesPerPoint =2
        
        maxPts=int(self.socketQuery( ":WAVeform:MAXPoint?"))
        #print ("maxpoints is {0}".format(maxPts))
        numPoints=10000000 # 10 mio points 
            
        rawdata = [0] *3
        for chan in ["1", "2", "3"]:
            pos=0
            data = bytearray([])
            self.socketCmd(":WAVeform:SOURce C"+chan) 
            self.socketCmd(":WAVeform:POINt "+str(maxPts)) 
            print ("Receiving channel {0}".format(chan))
            #print("points:"+self.socketQuery( ":WAVeform:POINt?"))
            while len(data) < numPoints*bytesPerPoint and len(data) == pos*bytesPerPoint:
                #print ("pos {0}".format(pos))
                self.socketCmd(":WAVeform:STARt "+str(pos)) 
                #print("start:"+self.socketQuery( ":WAVeform:STARt?"))
                qStr = self._resource.query_binary_values( ":WAVeform:DATA?", datatype="B" )
                #data += qStr[12:]
                data += bytearray(qStr)
                pos+=maxPts
            if base_filename is not None:
                f=open(base_filename+"_chan"+chan+".bin",'wb')
                f.write(data)
                f.flush()
                f.close()
            rawdata[int(chan)-1] = np.frombuffer(data, dtype=np.dtype('<i2'))
        self.socketCmd(":RUN")
        return rawdata

    def getFlux(self, rawdata:list, prominence=800, distance=20, peak_height=-5000) -> Flux:
        '''
        Get the flux from the binary data.
        
        prominence is how heigh a peak needs to be compared to it's neighbours
        distance is how far it needs to be away (in units measured by the scope)
        peak_height should not be touched normally ( we set it very low to also see peaks below the zero line)

        '''
        def rollavg_convolve_edges(a,n):
            'scipy.convolve, edge handling'
            assert n%2==1
            return sig.convolve(a,np.ones(n,dtype='float'), 'same')/sig.convolve(np.ones(len(a)),np.ones(n), 'same')  
        
        #print("reading files")
        #chan1 = np.memmap('siglentC1', dtype=np.dtype('<i2'), mode='r') # import as "<" little endian "i" integer "2" bytes
        #chan2 = np.memmap('siglentC2', dtype=np.dtype('<i2'), mode='r')
        #chan3 = np.memmap('siglentC3', dtype=np.dtype('<i2'), mode='r')
        chan1=rawdata[0]
        chan2=rawdata[1]
        chan3=rawdata[2]

        # convert to 32 bit for subtraact
        chan1 = np.array(chan1, dtype=np.int32)
        chan2 = np.array(chan2, dtype=np.int32)

        data = chan1- chan2

        if np.average(np.absolute(data)) < 500:
            return None

        b, a = scipy.signal.butter(4, 0.05, analog=False)
        data = scipy.signal.filtfilt(b, a, data)

        data_index = rollavg_convolve_edges( chan3, 9)



        positive_peaks = sig.find_peaks(data, height=peak_height, distance=distance, prominence=prominence)[0]
        negative_peaks = sig.find_peaks(data*-1, height=peak_height, distance=distance, prominence=prominence)[0]

        peaks=np.sort(positive_peaks.tolist()+negative_peaks.tolist())

        timings = np.diff(peaks, axis=0)
        print (str(len(timings))+" Flux changes found")

        ## creating histogram 
        hist={}
        for i in range(1, len(timings)):
            if timings[i] in hist:
                hist[timings[i]] = hist[timings[i]]+1
            else:
                hist[timings[i]] =1

        histarray = [0]*(max(hist)+1)
        for i in sorted(hist):
            histarray[i] = hist[i]

        # interpolate 
        x = np.linspace(1, len(histarray), len(histarray))
        xx = np.linspace(1, len(histarray), ((len(histarray)-1)*100)+1)
        histarray = CubicSpline(x, histarray)(xx)

        clocks = sig.find_peaks(histarray, height=max(histarray)*0.1)[0]
        print ("clock candidates: ",clocks)

        # assume the last peak is the most precise
        # try to find the integer divider to get to the other peaks
        scores = {}
        for i in range(len(clocks)-1,-1,-1):
            for divider_candidate in range(i+1,0,-1):
                clock_candidate = clocks[i]/divider_candidate
                print ("dc",divider_candidate, "clock", clocks[i], "cc", clock_candidate)
                if clock_candidate not in scores and clock_candidate >= clocks[0]*0.9:
                    sum=0
                    for harmonic in range(1,divider_candidate+1):
                        for offset in [0]: # range(-1,2):
                            sum = sum + histarray[round(clock_candidate*harmonic)+offset]
                    scores[clock_candidate] = sum
        #print ("scores : ",scores)

        clock = max(scores, key=scores.get)/100
        print ("detected clock is : ",clock)

        avg = np.average(np.absolute(data))



        if True:
            ticks_off_allowed=clock*0.15 # 10% of clock

            # first, check the timings, and count the consecutive well-synced pulses. 
            print("searching good clock pulses")
            good_timings = np.array([0] *len(timings))
            for i,x in enumerate(timings):
                off_clock=((x+clock/2)%clock)-clock/2
                good_timings[i] = (abs(off_clock)<ticks_off_allowed and (x/clock) < 3.5)

            # count how many good are directly following each other
            print("searching consecutive good clock pulses")
            # three 'tricks': comparing two arrays will output an array
            # so good_timings[:-1] != good_timings[1:] will yealds an array one shorter than the original, telling us where it flips from true to false or vice versa
            # we need to prepend the first value to get the same size, and append a fake change at the end to count the last few
            # np.where will find out indexes where the input is "true"
            where_good = np.where(np.concatenate(([good_timings[0]],good_timings[:-1] != good_timings[1:],[True])))[0]
            # diff will now give us the deltas, but again, not the first value, so we need to prepend it
            diff_good =np.concat(([where_good[0]],np.diff(where_good)))
        
            p = 0
            for streak in diff_good:
                if good_timings[p]:
                    good_timings[p:p+streak] = [streak] *streak
                p+=streak
            #print(good_timings)

                
            #loop and take the signal strength into the equation
            for i,x in enumerate(good_timings):
                s0 = data[peaks[i]]
                s1 = data[peaks[i+1]] if i+1 < len(good_timings) else 0-s0
                strength = abs(s0-s1)
                if strength > 10000:
                    good_timings[i] = good_timings[i]*2
                elif strength < 1000:
                    good_timings[i] = good_timings[i]/2
                elif strength < 500:
                    good_timings[i] = good_timings[i]/4
                    

            ticks_off_allowed=clock*0.10 # 10% of clock
            # loop through again, find the items to remove
            to_remove=list()
            gaps_start_pos=list()
            gaps_end_pos=list()
            i=0
            score_to_keep=1.5
            while i < len(good_timings)-1: # keep the last peak no matter what
                if good_timings[i] < score_to_keep: 
                    #print("scheduling timing#",i,"for removal")
                    to_remove.append(i)
                    gaps_start_pos.append(peaks[i])
                    #skip over the other bad ones
                    while good_timings[i+1] < score_to_keep and i+1 < len(good_timings)-1:
                        #print("scheduling timing#",i+1,"for removal")
                        to_remove.append(i+1)
                        i += 1
                    gaps_end_pos.append(peaks[i+2]) # we're going to remove peak (i+1), so need to re-sync on i+2
                i += 1

            del good_timings  # not maintained or needed after this line, delete to avoid programming mistakes and free memory
            del timings  # not maintained or needed after this line, delete to avoid programming mistakes and free memory

            peaks = list(peaks) # make peaks a list, so we can delete elements
            
            for i in to_remove[::-1]: #remove them in reverse order
                #print("removing peak#",i+1 ,"at", peaks[i+1])
                del peaks[i+1] # remove the end peak. 
            
            peak_ptr=0
            for i,start_pos in enumerate(gaps_start_pos):
                end_pos = gaps_end_pos[i]
                distance_ticks=(end_pos-start_pos)
                distance=distance_ticks/clock
                deviation = (0.5-abs(0.5-(distance-int(distance))))*100
                deviation_per_clock = deviation/round(distance,0)
                #if deviation_per_clock > 10:
                #    print("WARNING: found bad alignment between",start_pos, "and", end_pos)
                #    print("WARNING: deviation from ideal clock",round(deviation,1), "% over",round(distance,0),"clocks; ", round(deviation_per_clock,0), "% per clock")

                # use adjustecd clock to match the drift. 
                adjusted_clock = distance_ticks / round (distance,0)

                # find the index of the peak matching the start position
                while peaks[peak_ptr] < start_pos:
                    peak_ptr +=1

                if peaks[peak_ptr] > start_pos:
                    print("WARNING: we accidentally deleted the start position peak at", start_pos, "???")

                while start_pos < end_pos - (adjusted_clock*1.1): 
                    # detect the number of zero bitcells - as long as the distance to the original value increases, it will be a zero. 
                    num_zero_bitcells = 0
                    dx1 = data[start_pos] - data[int(round(start_pos+adjusted_clock*(num_zero_bitcells+1)))]
                    dx2 = data[start_pos] - data[int(round(start_pos+adjusted_clock*(num_zero_bitcells+2)))]
                    while (abs(dx1) < abs(dx2) and dx1*dx2 >= 0) and (peek2_pos := int(round(start_pos+adjusted_clock*(num_zero_bitcells+2)))) < len(data):
                        num_zero_bitcells +=1
                        dx1 = data[start_pos] - data[int(round(start_pos+adjusted_clock*(num_zero_bitcells+1)))]
                        dx2 = data[start_pos] - data[peek2_pos]

                    
                    dx = int(round(adjusted_clock*(num_zero_bitcells+1),0))
                    start_pos += dx
                    # create the peak, but only if reasonably before the end pos.
                    if start_pos < end_pos - (adjusted_clock*0.5):
                        #print("adding peak", start_pos)
                        peaks.insert(peak_ptr+1, start_pos)
                        peak_ptr+=1

        # check 
        for i,x in enumerate(peaks):
            if i< len(peaks)-1 and x >= peaks[i+1]:
              print("ERROR: at peak #",i," pos ",x, "is more than the following peak",peaks[i+1])

        # re-build the timings            
        timings = np.diff(peaks, axis=0).tolist()


        index_times = np.where((data_index[:-1] >= 0) & (data_index[1:] < 0))[0]
        index_times += 1
        index_diff = np.diff(index_times, prepend=0, axis=0).tolist() 

        ###########################################################################################################################################################
        # Note: it's very important to use list objects for index and flux instead of NumPy arrays - it will seem to work, but break in very "interesting" places #
        ###########################################################################################################################################################
        result = Flux(index_list=index_diff, flux_list=timings, sample_freq=10000000, index_cued=False)
        print(str(result))
        return result


if __name__ == "__main__":
    sc = Scope(ip="10.19.3.54")
    sc.setup()
    data= sc.query()
    flux=sc.getFlux(data)

    print (str(flux))