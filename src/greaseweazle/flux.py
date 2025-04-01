# greaseweazle/flux.py
#
# Written & released by Keir Fraser <keir.xen@gmail.com>
#
# This is free and unencumbered software released into the public domain.
# See the file COPYING for more details, or visit <http://unlicense.org>.

from __future__ import annotations
from typing import List, Optional, Protocol
import statistics
import numpy as np
from greaseweazle import error

class HasFlux(Protocol):
    def summary_string(self) -> str:
        ...
    def flux(self) -> Flux:
        ...
    def flux_for_writeout(self, cue_at_index) -> WriteoutFlux:
        ...

class Flux:

    _ticks_per_rev: float
    
    def __init__(self,
                 index_list: List[float],
                 flux_list: List[float],
                 sample_freq: float,
                 index_cued = True,
                 index_cut = False) -> None:
        self.index_list = index_list
        self.list = flux_list
        self.sample_freq = sample_freq
        self.splice: Optional[float] = None
        self.index_cued = index_cued
        self.index_cut = index_cut


    def __str__(self) -> str:
        s = "\nFlux: %.2f MHz" % (self.sample_freq*1e-6)
        s += ("\n Total: %u samples, %.2fms\n"
              % (len(self.list), sum(self.list)*1000/self.sample_freq))
        rev = 0
        for t in self.index_list:
            s += " Revolution %u: %.2fms\n" % (rev, t*1000/self.sample_freq)
            rev += 1
        return s[:-1]


    def summary_string(self) -> str:
        return ("Raw Flux (%u flux in %.2fms)"
                % (len(self.list), sum(self.list)*1000/self.sample_freq))


    def append(self, flux: Flux) -> None:
        '''
        Append one flux to another. Trailing flux after the last index and before the first index will be combined together.
        Note that there is no synchronization whatsoever. If combining flux from reads that are not 100% in sync, you need to
        remove the flux after the last index on the object appended to (#cut_at_index()) and the flux before the first index
        on the object being appended (#cue_at_index()). There is a high probability this will still cause an error at the stich,
        but for more than one revolution per combined object, it is made up by the added information. 
        '''
        # Scale the new flux if required, to match existing sample frequency.
        # This will result in floating-point flux values.
        if self.sample_freq == flux.sample_freq:
            f_list, i_list = flux.list, flux.index_list
        else:
            factor = self.sample_freq / flux.sample_freq
            f_list = [x*factor for x in flux.list]
            i_list = [x*factor for x in flux.index_list]
        # Any trailing flux is incorporated into the first revolution of
        # the appended flux.
        rev0 = i_list[0] + sum(self.list) - sum(self.index_list)
        self.index_list += [rev0] + i_list[1:]
        self.list += f_list


    def cue_at_index(self) -> None:
        '''
        Clips the flux before the first index signal as well as the first index signal.
        Will make sure the flux is still aligned with the exact index timing. (at the cost of altering the first flux change timing) 
        '''

        if self.index_cued:
            return

        # Clip the initial partial revolution.
        to_index = self.index_list[0]
        pos_in_list =0 # we need a variable outside of the loop scoping
        for i in range(len(self.list)-1):
            pos_in_list=i # remember the last index visited in case we drop out of the loop
            to_index -= self.list[i]
            if to_index < 0:
                break
        if to_index < 0:
            self.list = [-to_index] + self.list[pos_in_list+1:]
        else: # we ran out of flux
            self.list = []
        self.index_list = self.index_list[1:]
        self.index_cued = True

    def cut_at_index(self) -> None:
        '''
        Cut the flux after the last index signal. 
        It will keep the index and trim the last flux change to match the index signal. 
        '''

        if self.index_cut:
            return

        # Clip the partial revolution after the last index.
        from_index = sum(self.index_list)
        avg_index = statistics.fmean(self.index_list)
        to_remove = sum(self.list) - from_index

        if to_remove <= 0:
            # this means the last index signal is after the last flux change - we'll accept that and bail out. (nothing to delete...)
            self.index_cut = True
            return

        if ( to_remove > avg_index):
            # remove more than the average index length of data????
            raise RuntimeError("cut at last index would remove more than one full revolution of data")

        deleted_time=0
        items_to_remove = 0
        while deleted_time < to_remove:
            items_to_remove += 1
            deleted_time += self.list[-items_to_remove]

        # trim the last item, so we have the flux change exactly with the last index.
        # it is not perfect, especially for non-index-hole disks like C64, but will do for stiching flux together
        # accepting it will likely break the sector at the stiching point
        append_item = [] if (deleted_time-to_remove) ==0 else [(deleted_time-to_remove)]
        self.list = self.list[:-items_to_remove] + append_item
        self.index_cut = True

    def reverse(self) -> Flux:
        sumTicks = sum(self.list)
        reversedIndex: List[float] = []

        # first index is the rest after all other revolutions
        reversedIndex.append(sumTicks - sum(self.index_list))

        for idx in reversed(self.index_list):
            reversedIndex.append(idx)
        
        # need to remove the last one again
        del reversedIndex[-1]

        return Flux(reversedIndex, list(reversed(self.list)), self.sample_freq, self.index_cued)

    def flux_for_writeout(self, cue_at_index) -> WriteoutFlux:

        # Splice at index unless we know better.
        splice = 0 if self.splice is None else self.splice

        error.check(self.index_cued,
                    "Cannot write non-index-cued raw flux")
        error.check(splice == 0 or len(self.index_list) > 1,
                    "Cannot write single-revolution unaligned raw flux")
        splice_at_index = (splice == 0)

        # Copy the required amount of flux to a fresh list.
        flux_list = []
        to_index = self.index_list[0]
        remain = to_index + splice
        for f in self.list:
            if f > remain:
                break
            flux_list.append(f)
            remain -= f

        if not cue_at_index:
            # We will write more than one revolution and terminate the
            # second revolution at the splice. Extend the start of the write
            # with "safe" 4us sample values, in case the drive motor is a
            # little fast.
            if remain > 0:
                flux_list.append(remain)
            prepend = max(round(to_index/10 - splice), 0)
            if prepend != 0:
                four_us = max(self.sample_freq * 4e-6, 1)
                flux_list = [four_us]*round(prepend/four_us) + flux_list
            splice_at_index = False
        elif splice_at_index:
            # Extend with "safe" 4us sample values, to avoid unformatted area
            # at end of track if drive motor is a little slow.
            four_us = max(self.sample_freq * 4e-6, 1)
            if remain > four_us:
                flux_list.append(remain)
            for i in range(round(to_index/(10*four_us))):
                flux_list.append(four_us)
        elif remain > 0:
            # End the write exactly where specified.
            flux_list.append(remain)

        return WriteoutFlux(to_index, flux_list, self.sample_freq,
                            index_cued = cue_at_index,
                            terminate_at_index = splice_at_index)



    def flux(self) -> Flux:
        return self


    def scale(self, factor) -> None:
        """Scale up all flux and index timings by specified factor."""
        self.sample_freq /= factor


    @property
    def ticks_per_rev(self) -> float:
        """Mean time between index pulses, in sample ticks"""
        try:
            index_list = self.index_list
            if not self.index_cued:
                index_list = index_list[1:]
            ticks_per_rev = sum(index_list) / len(index_list)
        except:
            ticks_per_rev = self._ticks_per_rev
        return ticks_per_rev


    @property
    def time_per_rev(self) -> float:
        """Mean time between index pulses, in seconds (float)"""
        return self.ticks_per_rev / self.sample_freq


class WriteoutFlux:

    def __init__(
            self,
            ticks_to_index: float,
            flux_list: List[float],
            sample_freq: float,
            index_cued: bool,
            terminate_at_index: bool
    ) -> None:
        self.ticks_to_index = ticks_to_index
        self.list = flux_list
        self.sample_freq = sample_freq
        self.index_cued = index_cued
        self.terminate_at_index = terminate_at_index


    def __str__(self) -> str:
        s = ("\nWriteoutFlux: %.2f MHz, %.2fms to index, %s\n"
             " Total: %u samples, %.2fms"
             % (self.sample_freq*1e-6,
                self.ticks_to_index*1000/self.sample_freq,
                ("Write all", "Terminate at index")[self.terminate_at_index],
                len(self.list), sum(self.list)*1000/self.sample_freq))
        return s


    def summary_string(self) -> str:
        s = ("Flux: %.1fms period, %.1f ms total, %s"
             % (self.ticks_to_index*1000/self.sample_freq,
                sum(self.list)*1000/self.sample_freq,
                ("Write all", "Terminate at index")[self.terminate_at_index]))
        return s


# Local variables:
# python-indent: 4
# End:
