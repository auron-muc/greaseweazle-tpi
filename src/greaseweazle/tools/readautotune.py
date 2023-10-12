# greaseweazle/tools/read.py
#
# Greaseweazle control script: Read Disk to Image.
#
# Written & released by Keir Fraser <keir.xen@gmail.com>
#
# This is free and unencumbered software released into the public domain.
# See the file COPYING for more details, or visit <http://unlicense.org>.

description = "Read a disk to the specified image file."

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

from greaseweazle import track
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
    def __init__(self, offset : int = None, track : int = None):
        self._offset = offset
        self._track  = track
    def offset(self) -> int:
        return self._offset
    def track(self) -> int:
        return self._track

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


def read_with_retry(usb: USB.Unit, args, t, arduino = None, offset : Offset = None) -> Tuple[Flux, Optional[HasFlux]]:

    cyl, head = t.cyl, t.head

    print ("setting offset ", str(offset.offset()))
    arduino.write(str(offset.offset()).encode())
    arduino.write("\n".encode())
    usb.seek(t.physical_cyl, t.physical_head)
    time.sleep(0.2)

    flux = read_and_normalise(usb, args, args.revs, args.ticks)
    if args.fmt_cls is None:
        print("T%u.%u: %s" % (cyl, head, flux.summary_string()))
        return flux, flux

    dat = args.fmt_cls.decode_flux(cyl, head, flux)
    if dat is None:
        print("T%u.%u: WARNING: Out of range for for format '%s': No format "
              "conversion applied" % (cyl, head, args.format))
        return flux, None
    for pll in plls[1:]:
        if dat.nr_missing() == 0:
            break
        dat.decode_flux(flux, pll)

    print( "T%u.%u: %s from %s" % (cyl, head, dat.summary_string(), flux.summary_string()))

    seek_retry, retry = 0, 0
    if dat.nr_missing() > 0 and arduino is not None and offset is not None :
        for o in [ 1, -1, 2, -2, 3, -3]:
            print("offset", str(offset.offset()+o) )
            arduino.write((str(offset.offset()+o) +"\n").encode() )
            time.sleep(0.01)
            _flux = read_and_normalise(usb, args, max(args.revs, 3))
            for pll in plls:
                if dat.nr_missing() != 0:
                    dat.decode_flux(_flux, pll)
            flux.append(_flux)
            s = "T%u.%u: %s from %s" % (cyl, head, dat.summary_string(),
                                        flux.summary_string())
            s += " (Retry offset from track-center %u)" % (o)
            print(s)
            if dat.nr_missing() == 0:
                break

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


def heatmap_score(flux: Flux) -> int:
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

    return score 

def tuning_scan (usb: USB.Unit, args, image: image.Image, arduino : Serial ) -> List[Offset]:
    """ Do the initial tuning scan. 
        Loop over three full tracks and detect the noise spikes inbetween. then center to the middle.
        Also try to detect which track we're on by decoding the header.
        Returns a Tuple with [offset, detected_track]"""

    # the scores by offset from nominal track-center [offset => score]
    raw_scores: List[Dict[int, float]] = [{},{}]
    avg_scores: List[Dict[int, float]] = [{},{}]
    threshold: List[float] = [0, 0]
    heads = [0,1]
    cyl  = 0

    # seek to track 0 and calibrate
    print("Setting offset to 0")
    arduino.write("0\n".encode()) # set the offset to 0 before calibration
    time.sleep(0.2)
    print("Seeking 0")
    usb.seek(cyl=0,head=0)
    print("calibrating1")
    arduino.write("c\n".encode())
    arduino.flush();
    ctr = 0
    while usb.get_pin(26) and ctr < 1000:
        print (arduino.read_all().decode(), end="")
        ctr += 1
        time.sleep(0.01)
    print("calibrating2")
    arduino.write("c\n".encode())
    arduino.flush();
    time.sleep(0.5)
    tpi=args.tpi
    print("setting TPI to ",tpi)
    arduino.write("t".encode())
    arduino.write(str(tpi).encode())
    arduino.write("\n".encode())

    usb.seek(cyl=cyl,head=heads[0])

    exact_offsets_per_track = (25.4/tpi)/0.02

    offsets_per_track = math.ceil(exact_offsets_per_track) 
    for offset in range (0, 2*offsets_per_track):
        arduino.write(str(offset).encode())
        arduino.write("\n".encode())
        time.sleep(0.05)
        for head in heads:
            usb.select_head(head)
            flux = usb.read_track(1)
            raw_scores[head][offset] = heatmap_score(flux)
            print(offset,"raw score head",head,":", raw_scores[head][offset])

    # calculate the moving average
    for offset in range (1, (2*offsets_per_track)-1):
        for head in heads:
            avg_scores[head][offset] = (raw_scores[head][offset-1]+raw_scores[head][offset]+raw_scores[head][offset+1])/3
            print(offset,"avg score head",head,":", avg_scores[head][offset])

    result = [Offset(),Offset()]
    for head in heads:
        scores_list = np.fromiter(avg_scores[head].values(), dtype=float)
        avg = np.average(scores_list)
        stddev = np.std(scores_list)
        threshold = avg+(stddev/2)
        print ("threshold",threshold)

        spikes = []
        in_valley_at_start = (avg_scores[head][1] < threshold)
        for offset,score in avg_scores[head].items():
            # skip over any values above threshold at the beginning, partially recorded spikes would throw us off
            if in_valley_at_start:
                if score < threshold:
                    continue
                else:
                    in_valley_at_start = False
            # detect spike falling edge
            if len(spikes)%2 == 0 and score < threshold:
                spikes.append(offset-1)
            # detect spike rising edge
            if len(spikes)%2 == 1 and score >= threshold:
                spikes.append(offset)

        if len(spikes) < 2:
            print ("Warning: No full track detected in scan on head", head)
            continue

        trackcenter = (spikes[0]+spikes[1])/2
        print ("trackcenter: ", trackcenter)
        result[head] = Offset(int(round(trackcenter,0)), None)
        arduino.write(str(int(round(trackcenter,0))).encode())
        arduino.write("\n".encode())
        time.sleep(0.2)

        # read the track at best position, 3 revolutions
        usb.select_head(head)
        flux = read_and_normalise(usb, args, 2, args.ticks)
    
    
        cyl_detected = None
        if args.fmt_cls is not None:
            s=""
            devnull = open(os.devnull, 'w')
            with RedirectStdStreams(stdout=devnull, stderr=devnull):
            #if True:
                dat = args.fmt_cls.mk_track(cyl, head)
                if dat is not None:
                    for pll in [None] + plls[1:]:
                        cyl_detected = dat.guess_cylinder(flux, pll)
                        if cyl_detected is not None:
                            break
        if cyl_detected is not None:
            while ( trackcenter > (exact_offsets_per_track/4) and cyl_detected > 0) or \
                  ( trackcenter > (exact_offsets_per_track/2) and cyl_detected is None):
                s += "shifting Track %u at offset %f by %f\n" % (cyl_detected, trackcenter, exact_offsets_per_track)
                trackcenter -= exact_offsets_per_track
                if cyl_detected is not None:
                    cyl_detected -= 1
            s += "detected Track %u at offset %u\n" % (cyl_detected, int(round(trackcenter,0)))
            result[head] = Offset(int(round(trackcenter,0)), cyl_detected)
            print(s)

    return result

def read_to_image(usb: USB.Unit, args, image: image.Image, arduino : Serial) -> None:
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
 
    for t in args.tracks:
        cyl, head = t.cyl, t.head

        flux, dat = read_with_retry(usb, args, t, arduino=arduino, offset=autotune_offset[t.head])

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
    parser.add_argument("file", help="output filename")
    parser.description = description
    parser.prog += ' ' + argv[1]
    args = parser.parse_args(argv[2:])

    args.file, args.file_opts = util.split_opts(args.file)

    if args.pll is not None:
        plls.insert(0, args.pll)

    try:
        usb = util.usb_open(args.device)
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

        print(("Reading %s revs=" % args.tracks) + str(args.revs))
        if args.format:
            print("Format " + args.format)
        try:
            if args.dd is not None:
                prev_pin2 = usb.get_pin(2)
                usb.set_pin(2, args.dd)
            with open_image(args, image_class) as image:
                util.with_drive_selected(
                    lambda: read_to_image(usb, args, image, arduino=arduino), usb, args.drive)
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
