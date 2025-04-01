# greaseweazle/tools/read.py
#
# Greaseweazle control script: Read Disk to Image.
#
# Written & released by Keir Fraser <keir.xen@gmail.com>
#
# This is free and unencumbered software released into the public domain.
# See the file COPYING for more details, or visit <http://unlicense.org>.

description = "Read a disk to the specified image file."

#### ugly hack to allow debugging
#import sys; 
#sys.path.insert(0,'C:\\Users\\Tobias\\git\\greaseweazle\\src')


import time
from typing import cast, Dict, Tuple, List, Type, Optional

import sys, copy
import json
from json import JSONEncoder
import numpy as np
import os
import sys
import math

from serial import Serial
from greaseweazle.tools import util
from greaseweazle import error
from greaseweazle import usb as USB
from greaseweazle.flux import Flux, HasFlux
from greaseweazle.codec import codec
from greaseweazle.image import image
from greaseweazle.scope import scope as sc

from greaseweazle import track

import scipy as sci
import scipy.signal as sig
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt
from threading import Thread


def rollavg_convolve_edges(a,n):
    'scipy.convolve, edge handling'
    assert n%2==1
    return sig.convolve(a,np.ones(n,dtype='float'), 'same')/sig.convolve(np.ones(len(a)),np.ones(n), 'same')  


plls = track.plls

class DummyEncoder(JSONEncoder):
    """Helper class to json dump objects that are not json dumpable"""
    def default(self, o):
        try:
            return o.__dict__
        except Exception:
            return str(o)


class RedirectStdStreams(object):
    """Helper class to teporarily silence all output"""
    def __init__(self, stdout=None, stderr=None):
        self._stdout = stdout or sys.stdout
        self._stderr = stderr or sys.stderr

    def __enter__(self):
        self.old_stdout, self.old_stderr = sys.stdout, sys.stderr
        self.old_stdout.flush(); self.old_stderr.flush()
        sys.stdout, sys.stderr = self._stdout, self._stderr

    def __exit__(self, exc_type, exc_value, traceback):
        self._stdout.flush(); self._stderr.flush()
        sys.stdout = self.old_stdout
        sys.stderr = self.old_stderr

class Offset(object):
    """Offset information for a head"""
    def __init__(self, offset : int = None, cyl_offset : int = None):
        self._offset = offset
        self._cyl_offset  = cyl_offset
    def offset(self) -> int:
        return self._offset
    def cyl_offset(self) -> int:
        return self._cyl_offset

def open_image(args, image_class: Type[image.Image]) -> image.Image:
    image = image_class.to_file(
        args.file, None if args.raw else args.fmt_cls, args.no_clobber)
    for opt, val in args.file_opts.items():
        error.check(hasattr(image, 'opts') and opt in image.opts.settings,
                    "%s: Invalid file option: %s\n" % (args.file, opt)
                    + 'Valid options: ' + ', '.join(image.opts.settings))
        setattr(image.opts, opt, val)
    image.write_on_ctrl_c = True
    return image


def read_and_normalise(usb: USB.Unit, args, revs: int, ticks=0) -> Flux:
    if args.fake_index is not None:
        drive_tpr = int(args.drive_ticks_per_rev)
        pre_index = int(usb.sample_freq * 0.5e-3)
        if ticks == 0:
            ticks = revs*drive_tpr + 2*pre_index
        flux = usb.read_track(revs=0, ticks=ticks)
        index_list = [pre_index] + [drive_tpr] * ((ticks-pre_index)//drive_tpr)
        flux.index_list = cast(List[float], index_list) # mypy
    else:
        flux = usb.read_track(revs=revs, ticks=ticks)
    flux._ticks_per_rev = args.drive_ticks_per_rev
    if args.adjust_speed is not None:
        flux.scale(args.adjust_speed / flux.time_per_rev)
    return flux


def read_with_retry(usb: USB.Unit, args, t: util.TrackSet.TrackIter, arduino = None, offset : Offset = None, scope :sc.Scope = None) -> Tuple[Flux, Optional[HasFlux]]:

    cyl, head = t.cyl, t.head
    disk : codec.DiskDef = args.fmt_cls if args.fmt_cls is not None else None

    detected_offset = 0
    detected_cylinder_offset = 0

    if offset is not None:
        detected_offset = offset.offset()
        detected_cylinder_offset = offset.cyl_offset()

    print ("additional cylinder offset ", str(detected_cylinder_offset))

    print ("setting micro offset ", str(detected_offset))
    arduino.write(str(detected_offset).encode())
    arduino.write("\n".encode())

    usb.seek(t.physical_cyl+detected_cylinder_offset, t.physical_head)
    time.sleep(0.2)

    flux = read_and_normalise(usb, args, args.revs, args.ticks)
    if disk is not None and disk.flippy and head == 1:
        flux = flux.reverse()
    flux.cue_at_index()                              

    if args.fmt_cls is None:
        print("T%u.%u: %s" % (cyl, head, flux.summary_string()))
        return flux, flux


    dat = disk.decode_flux(cyl, head, flux)
    if dat is None:
        print("T%u.%u: WARNING: Out of range for for format '%s': No format "
              "conversion applied" % (cyl, head, args.format))
        return flux, None
    for pll in plls[1:]:
        if dat.nr_missing() == 0:
            break
        dat.decode_flux(flux, pll)

    print( "T%u.%u: %s from %s" % (cyl, head, dat.summary_string(), flux.summary_string()))
    dat.guess_physical_cylinder(flux, pll)

    flux_by_offset = [flux]
    offsets = [0]
    seek_retry, retry = 0, 0
    if dat.nr_missing() > 0 and arduino is not None :
        last_missing=dat.nr_missing()
        best_score = heatmap_score(flux)
        min_search_distance = 3

        o=-1  # start with offset -1 - empiric studies say we tend to be too far
        while len(flux_by_offset) < 20 and last_missing > 0: 
            #print("offset", str(detected_offset+o) )
            arduino.write((str(detected_offset+o) +"\n").encode() )
            time.sleep(0.1)
            flux = read_and_normalise(usb, args, max(args.revs, 3))
            if disk is not None and disk.flippy and head == 1:
                flux = flux.reverse()
            flux.cue_at_index()
            score = heatmap_score(flux)

            for pll in plls:
                if dat.nr_missing() != 0:
                    dat.decode_flux(flux, pll)
            flux_by_offset.append(flux)
            offsets.append(o)
            s = "T%u.%u: %s from %s" % (cyl, head, dat.summary_string(),
                                        flux.summary_string())
            s += " (Retry offset from track-center %u)" % (o)
            print(s)
            if dat.nr_missing() < last_missing or score < best_score or abs(o) < min_search_distance:
                # going in the right direction
                o += 1 if (o>0) else -1
            else:
                # try the other direction    
                if o < 0:
                    o = 1
                    last_missing=999   # ridiculously high number
                    best_score=999
                elif o < 0:
                    o = 0 # try center once again
                else:
                    break # no sense in trying again
            
            last_missing = dat.nr_missing()
            if best_score > score:
                best_score = score

            if dat.nr_missing() == 0:
                # save this offset as the new best guess
                print ("Saving "+str(detected_offset+o)+" as new offset for head "+str(head))
                offset._offset = detected_offset+o
                break

    # find the best flux we have read, return that
    best_flux = 0
    if dat.nr_missing() != 0:
        best_score = 1000
        for  i in range(len(flux_by_offset)):
            _flux = flux_by_offset[i]
            score = heatmap_score(_flux)
            print ("raw score for offset",offsets[i],score)
            # also decode the flux again, we need to know how many are missing per offset
            _dat = disk.decode_flux(cyl, head, _flux)
            # factor in the number of missing sectors
            score = score + (_dat.nr_missing()*10) 
            print ("weighted score for offset",offsets[i],score)
            if score < best_score:
                best_flux=i
                best_score=score

    # re-select/start drive
    usb.drive_select(args.drive.unit_id)
    usb.drive_motor(args.drive.unit_id, state=True)

    # combine all the read fluxes into one big, we'll return that later
    flux = flux_by_offset[0]
    for additional_flux in flux_by_offset[1:]:
        flux.append(additional_flux)

    if scope is not None and dat.nr_missing() !=0:

        # position the track on the best position the greaseweazle could find
        print("reading scope at offset "+str(detected_offset+offsets[best_flux]))
        arduino.write((str(detected_offset+offsets[best_flux]) +"\n").encode() )
        time.sleep(0.1)
        # create a greaseweazle read thread in the background to keep the drive spinning
        thread = Thread(target = read_and_normalise, args = (usb, args, 20))
        thread.start()
        time.sleep(0.5)
        # query the data from the drive
        data = scope.query(base_filename=args.file+("_T%u.%u"%(cyl, head)))
        # wait for greaseweazle read to finish (it really should already have completed)
        thread.join()

        # get max and min for first channel
        c0max = np.max(data[0])
        c0min = np.min(data[0])

        for prominence in [ (c0max-c0min)*0.75 , (c0max-c0min)*0.5, (c0max-c0min)*0.25, (c0max-c0min)*0.1, (c0max-c0min)*0.01]:
            # compute the flux from the raw scope data
            print("searching for data with a prominence of ",prominence)
            scopeflux = scope.getFlux(data, prominence=prominence)
            if scopeflux is not None:
                scopeflux.cue_at_index()
                scopeflux.cut_at_index()
                if disk.flippy and head == 1:
                    scopeflux = scopeflux.reverse()
                #scopedat = disk.decode_flux(cyl, head, flux)
                for pll in plls:
                    if dat.nr_missing() != 0:
                        dat.decode_flux(scopeflux, pll)
                print("T%u.%u: %s from %s (Retry from scope)" % (cyl, head, dat.summary_string(),
                                                scopeflux.summary_string()))
                flux.append(scopeflux)
            else:
                print("scope detected only noise")

        # re-select/start drive
        usb.drive_select(args.drive.unit_id)
        usb.drive_motor(args.drive.unit_id, state=True)

    return flux, dat


def print_summary(args, summary: Dict[Tuple[int,int],codec.Codec]) -> None:
    if not summary:
        return
    nsec = max((summary[x].nsec for x in summary), default = None)
    if nsec is None or nsec == 0:
        return
    s = 'Cyl-> '
    p = -1
    for c in args.tracks.cyls:
        s += ' ' if c//10==p else str(c//10)
        p = c//10
    print(s)
    s = 'H. S: '
    for c in args.tracks.cyls:
        s += str(c%10)
    print(s)
    tot_sec = good_sec = 0
    for head in args.tracks.heads:
        nsec = max((summary[x].nsec for x in summary if x[1] == head),
                   default = None)
        if nsec is None:
            continue
        for sec in range(nsec):
            print("%d.%2d: " % (head, sec), end="")
            for cyl in args.tracks.cyls:
                t = summary.get((cyl,head), None)
                if t is None or sec >= t.nsec:
                    print(" ", end="")
                else:
                    tot_sec += 1
                    if t.has_sec(sec): good_sec += 1
                    print("." if t.has_sec(sec) else "X", end="")
            print()
    if tot_sec != 0:
        print("Found %d sectors of %d (%d%%)" %
              (good_sec, tot_sec, good_sec*100/tot_sec))


def heatmap_score(flux: Flux) -> float:
    """Calculates a score for how nicely aligned the bands are. 
       Lower scores greater zero are better. 
       "1" would mean flux changes at perfectly the same timing and only one band. 
    """
    ticks_per_usec = flux.sample_freq / 1000000
    ticks_per_msec = flux.sample_freq / 1000    
    heatmap = {}
    ticks=0
    indexctr=0
    for tick in flux.list:
        ticks += tick
        if ticks > flux.index_list[indexctr]:
            ticks -= flux.index_list[indexctr]
            indexctr += 1
            if indexctr >= len(flux.index_list):
                break
        ticks_msec=round(ticks / ticks_per_msec,0)    
        per_msec = heatmap.get(ticks_msec, {}) 
        tick_usec = round(tick /ticks_per_usec, 1)
        per_msec[tick_usec] = per_msec.get(tick_usec, 0) +1
        #per_msec[tick_usec] = 1
        heatmap[ticks_msec] = per_msec

    score = 0
    for msec, bands in heatmap.items():
        score += len(bands)
    score = score / len(heatmap)    

 #   if score < 10:
 #       print("Debug: ", json.dumps(heatmap, cls=DummyEncoder))

    return round(score,2) 

def tuning_scan (usb: USB.Unit, args, image: image.Image, arduino : Serial ) -> List[Offset]:
    """ Do the initial tuning scan. 
        Loop over three full tracks and detect the noise spikes inbetween. then center to the middle.
        Also try to detect which track we're on by decoding the header.
        Returns a Tuple with [offset, cylinder_offset]"""

    tpi=args.tpi
    exact_offsets_per_track = (25.4/tpi)/0.02
    offsets_per_track = math.ceil(exact_offsets_per_track) 
    nr_full_tracks=3
    num_scores = nr_full_tracks*offsets_per_track
    tracks : util.TrackSet = args.tracks

    # the scores by offset from nominal track-center [offset => score]
    raw_scores: List[List[float]] = [[],[]]
    avg_scores: List[List[float]] = [[],[]]
    interpolated: List[List[float]] = [[],[]]
    interpolation_factor = 10

    threshold: List[float] = [0, 0]
    
    heads = tracks.heads #  [0,1]
    cyl  = 0

    # seek to track 0 and calibrate
    print("Setting offset to 0")
    arduino.write("0\n".encode()) # set the offset to 0 before calibration
    time.sleep(0.2)
    print("Seeking 0")
    usb.seek(cyl=0,head=0)
    time.sleep(2) # wait until reaching cylinder 0
    print("calibrating1")
    arduino.reset_input_buffer() # clear all previous input
    arduino.write("c\n".encode())
    arduino.flush()
    ctr = 0
    sresult = arduino.read_all().decode()
    # usb.get_pin(26)
    while "Calibrated track0" not in sresult and ctr < 1000:
        sresult += arduino.read_all().decode()
        print (sresult, end="")
        ctr += 1
        time.sleep(0.01)
    time.sleep(0.25)
    usb.seek(cyl=8,head=0)
    usb.seek(cyl=0,head=0)

    arduino.reset_input_buffer() # clear all previous input
    print("\ncalibrating again")
    arduino.write("c\n".encode())
    arduino.flush()

    ctr = 0
    sresult = arduino.read_all().decode()
    # usb.get_pin(26)
    while "Calibrated track0" not in sresult and ctr < 1000:
        sresult += arduino.read_all().decode()
        print (sresult, end="")
        ctr += 1
        time.sleep(0.01)
    time.sleep(0.25)

    print("\nsetting TPI to ",tpi)
    arduino.write("t".encode())
    arduino.write(str(tpi).encode())
    arduino.write("\n".encode())

    usb.seek(cyl=cyl,head=heads[0])
    time.sleep(0.5)
    
    offset =0
    spikes = [None]*2
    diff = [None]*2
    result = [Offset(),Offset()]
    while ((offset < 100 ) and (result[0].offset() is None or result[1].offset() is None)):
        buf = (str(offset)+"\n").encode()
        for c in buf:
            arduino.write([c])
            arduino.flush()
        #arduino.write(str(offset).encode())
        #arduino.write("\n".encode())
        time.sleep(0.05)
        for head in heads:
            if result[head].offset() is None:
                usb.select_head(head)
                flux = usb.read_track(1)
                raw_scores[head] += [ heatmap_score(flux) ] # score will make the gaps inbetween tracks visible as peaks



                if len(raw_scores[head]) > 17:

                    # first: smooth out randomness in measurement a bit
                    avg_scores[head] = rollavg_convolve_edges( raw_scores[head], 3)

                    # interpolate by factor 10
                    ip = avg_scores[head]
                    x = np.linspace(1, len(ip), len(ip))
                    xx = np.linspace(1, len(ip), ((len(ip)-1)*interpolation_factor)+1)
                    interpolated[head] = CubicSpline(x, ip)(xx)   
                    avg = np.average(interpolated[head])
                    # find the peaks in the interpolated data. min distance is 10(offsets)*10(interpolation factor). 
                    # 12.7 is the distance in 0.02mm offsets at 100tpi , 10 would match 127 TPI
                    # (so this means we're not finding anything with more tpi)
                    _spikesxxl = sig.find_peaks(interpolated[head], height=avg*0.5, distance=10*interpolation_factor)[0] 

                    spikes[head] = _spikesxxl

                    # found two peaks and we gathered 4 more offsets after the second peak
                    # reading after the second peak should prevent false positives because we did not do the rolling average on the edge
                    if len(spikes[head]) >= 2 and (spikes[head][1]/interpolation_factor) + 5 < len(raw_scores[head])  :

                        trackcenter = round((_spikesxxl[1]-(_spikesxxl[1]-_spikesxxl[0])/2.0)/interpolation_factor )
                        tpi_detected=round(25.4/(0.02*(_spikesxxl[1]-_spikesxxl[0])/(2*interpolation_factor)))
                        print ("Head: ", head)
                        print ("trackcenter: ", trackcenter)
                        print ("detected track width (tpi):" +str(tpi_detected))

                        result[head] = Offset(int(round(trackcenter,0)), 0)
        offset += 1    

    # save the graphics for the head calibration
    for head in heads:
        #print ("raw_scores["+str(head)+"] = " +str(raw_scores[head]))
        fig, ax = plt.subplots()
        fig.set_dpi(600)
        fig.set_size_inches(40,8)
        plt.axhline(y=0,color='lightgrey',linewidth=1)
        plt.axhline(y=5,color='lightgrey',linewidth=1)
        plt.axhline(y=10,color='lightgrey',linewidth=1)
        plot_time = np.arange(0.0, len(raw_scores[head]), 1)
        line, = ax.plot(plot_time, raw_scores[head], lw=1, color='blue')
        line, = ax.plot(plot_time, avg_scores[head], lw=1, color='red')
        plot_time = np.arange(0.0, len(interpolated[head])/interpolation_factor, 1.0/interpolation_factor)
        line4, = ax.plot(plot_time, interpolated[head], lw=1, color='green')

        for s in spikes[head]:
            plt.axvline(x=s/interpolation_factor,color='r',linewidth=3)
        plt.savefig('offset_scores_head'+str(head)+'.png')
        #plt.show()
    #dummy = input("press enter to continue")

    for head in heads:
        if result[head].offset() is not None:
            trackcenter = result[head].offset()
            arduino.write(str(trackcenter).encode())
            arduino.write("\n".encode())
            time.sleep(0.2)

            # read the track at best position, 3 revolutions
            usb.select_head(head)
            flux = read_and_normalise(usb, args, 2, args.ticks)
        
            cyl_detected = None
            disk : Optional[codec.DiskDef] = args.fmt_cls
            cylinders_per_track=1
            if disk is not None:
                if (tpi != 48 ): 
                    cylinders_per_track=disk.step
                if disk.flippy and head == 1:
                    flux = flux.reverse()
                tmp_out_str=""
                devnull = open(os.devnull, 'w')
                #with RedirectStdStreams(stdout=devnull, stderr=devnull):
                if True:
                    dat = disk.mk_track(cyl, head)
                    if dat is not None:
                        for pll in [None] + plls[1:]:
                            cyl_detected = dat.guess_physical_cylinder(flux, pll)
                            if cyl_detected is not None:
                                cyl_detected += tracks.h_off[head]
                                tmp_out_str += "Found guessed cylinder "+str(cyl_detected)+" on head "+str(head)+" and offset "+str(trackcenter) +"\n"
                                disk_cyls = 0 if disk.cyls is None else disk.cyls
                                if head==1 and cyl_detected >= disk_cyls:
                                    cyl_detected -= disk_cyls
                                break
                if cyl_detected is not None:
                    while ( trackcenter > ((exact_offsets_per_track*cylinders_per_track)/4) and cyl_detected > cyl) :
                        tmp_out_str += "shifting guessed cyl %u at offset %f by %f\n" % (cyl_detected, trackcenter, exact_offsets_per_track*cylinders_per_track)
                        trackcenter -= exact_offsets_per_track*cylinders_per_track
                        cyl_detected -= 1

                    tmp_out_str += "detected guessed cyl %u at offset %u\n" % (cyl_detected, int(round(trackcenter,0)))
                    result[head] = Offset(int(round(trackcenter,0)), cyl-cyl_detected)
                else:
                    while ( trackcenter > (exact_offsets_per_track/2)):
                        tmp_out_str += "shifting Track at offset %f by %f\n" % (trackcenter, exact_offsets_per_track)
                        trackcenter -= exact_offsets_per_track
                    result[head] = Offset(int(round(trackcenter,0)), 0) # we found no cylinder, so just assume no shift is necessary.
                    dummy = input("No cylinder detected in track information. press enter to continue")
                    # re-select/start the greaseweazle
                    usb.drive_select(args.drive.unit_id) 
                    usb.drive_motor(args.drive.unit_id, state=True)

                print(tmp_out_str)
                time.sleep(2)

    #dummy = input("press enter to continue")

    return result

def read_to_image(usb: USB.Unit, args, image: image.Image, arduino : Serial, scope : sc.Scope = None) -> None:
    """Reads a floppy disk and dumps it into a new image file.
    """

    args.ticks, args.drive_ticks_per_rev = 0, None
    
    best_autotune_result = 0

    if args.fake_index is not None:
        args.drive_ticks_per_rev = args.fake_index * usb.sample_freq

    if isinstance(args.revs, float):
        if args.raw:
            # If dumping raw flux we want full index-to-index revolutions.
            args.revs = 2
        else:
            # Measure drive RPM.
            # We will adjust the flux intervals per track to allow for this.
            if args.drive_ticks_per_rev is None:
                args.drive_ticks_per_rev = usb.read_track(2).ticks_per_rev
            args.ticks = int(args.drive_ticks_per_rev * args.revs)
            args.revs = 2

    summary: Dict[Tuple[int,int],codec.Codec] = dict()

    autotune_offset = tuning_scan(usb,args,image, arduino)
 
    tracks : util.TrackSet = args.tracks
    for t in tracks:
        cyl, head = t.cyl, t.head

        flux, dat = read_with_retry(usb, args, t, arduino=arduino, offset=autotune_offset[t.head], scope=scope)

        if args.fmt_cls is not None and dat is not None:
            assert isinstance(dat, codec.Codec)
            summary[cyl,head] = dat
        if args.raw:
            image.emit_track(cyl, head, flux)
        elif dat is not None:
            image.emit_track(cyl, head, dat)

    if args.fmt_cls is not None:
        print_summary(args, summary)


def main(argv) -> None:

    epilog = (util.drive_desc + "\n"
              + util.speed_desc + "\n" + util.tspec_desc
              + "\n" + util.pllspec_desc
              + "\nFORMAT options:\n" + codec.print_formats()
              + "\n\nSupported file suffixes:\n"
              + util.columnify(util.image_types))
    parser = util.ArgumentParser(usage='%(prog)s [options] file',
                                 epilog=epilog)
    parser.add_argument("--device", help="device name (COM/serial port)")
    parser.add_argument("--stepperdevice", help="stepper driver device name (COM/serial port)")
    parser.add_argument("--tpi", type=util.uint, default=96, help="stepper driver tracks per inch")
    parser.add_argument("--drive", type=util.Drive(), default='A',
                        help="drive to read")
    parser.add_argument("--diskdefs", help="disk definitions file")
    parser.add_argument("--format", help="disk format (output is converted unless --raw)")
    parser.add_argument("--revs", type=util.min_int(1), metavar="N",
                        help="number of revolutions to read per track")
    parser.add_argument("--tracks", type=util.TrackSet, metavar="TSPEC",
                        help="which tracks to read")
    parser.add_argument("--raw", action="store_true",
                        help="output raw stream (--format verifies only)")
    parser.add_argument("--fake-index", type=util.period, metavar="SPEED",
                        help="fake index pulses at SPEED")
    parser.add_argument("--adjust-speed", type=util.period, metavar="SPEED",
                        help="scale track data to effective drive SPEED")
    parser.add_argument("--retries", type=util.uint, default=3, metavar="N",
                        help="number of retries per seek-retry")
    parser.add_argument("--seek-retries", type=util.uint, default=0,
                        metavar="N",
                        help="number of seek retries")
    parser.add_argument("-n", "--no-clobber", action="store_true",
                        help="do not overwrite an existing file")
    parser.add_argument("--pll", type=track.PLL, metavar="PLLSPEC",
                        help="manual PLL parameter override")
    parser.add_argument("--dd", type=util.level,
                        help="drive interface DD/HD select (H,L)")
    parser.add_argument("--scope-ip", help="ip address of the siglent scope attached to floppy drive")
    parser.add_argument("file", help="output filename")
    parser.description = description
    parser.prog += ' ' + argv[1]
    args = parser.parse_args(argv[2:])

    args.file, args.file_opts = util.split_opts(args.file)

    if args.pll is not None:
        plls.insert(0, args.pll)

    try:
        usb = util.usb_open(args.device)

        # hack - my drive mechanics are slow
        if args.tpi == 48:
            usb.step_delay = 16000
        else:
            usb.step_delay = 8000

        usb.seek_settle_delay = 250
        #usb.motor_delay = 2000

        image_class = util.get_image_class(args.file)
        if not args.format:
            args.format = image_class.default_format
        def_tracks, args.fmt_cls = None, None
        if args.format:
            args.fmt_cls = codec.get_diskdef(args.format, args.diskdefs)
            if args.fmt_cls is None:
                raise error.Fatal("""\
Unknown format '%s'
Known formats:\n%s"""
                                  % (args.format, codec.print_formats(
                                      args.diskdefs)))
            def_tracks = copy.copy(args.fmt_cls.tracks)
            if args.revs is None: args.revs = args.fmt_cls.default_revs
        if def_tracks is None:
            def_tracks = util.TrackSet('c=0-81:h=0-1')
        if args.revs is None: args.revs = 3
        if args.tracks is not None:
            def_tracks.update_from_trackspec(args.tracks.trackspec)
        args.tracks = def_tracks


        arduino = Serial(args.stepperdevice, baudrate=500000)
        # wait for arduino to become ready
        time.sleep(1)

        scope = None
        if args.scope_ip is not None:
            scope = sc.Scope( ip = args.scope_ip)
            scope.setup()

        print(("Reading %s revs=" % args.tracks) + str(args.revs))
        if args.format:
            print("Format " + args.format)
        try:
            if args.dd is not None:
                prev_pin2 = usb.get_pin(2)
                usb.set_pin(2, args.dd)
            with open_image(args, image_class) as image:
                util.with_drive_selected(
                    lambda: read_to_image(usb, args, image, arduino=arduino, scope=scope ), usb, args.drive)
        finally:
            if args.dd is not None:
                usb.set_pin(2, prev_pin2)
    except USB.CmdError as err:
        print("Command Failed: %s" % err)


if __name__ == "__main__":
    main(sys.argv)

# Local variables:
# python-indent: 4
# End:
