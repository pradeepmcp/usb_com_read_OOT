#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 
# Copyright 2019 <+YOU OR YOUR COMPANY+>.
# 
# This is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
# 
# This software is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this software; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.
# 

import numpy
from gnuradio import gr
from threading import Thread
from threading import Lock
import serial

BUFF_SIZE = 1024


class DelimiterNotFoundError(Exception):
    """Exception in the case delimiter was not found"""
    pass

class usb_com_read(gr.sync_block):
    """
    docstring for block usb_com_read
    """

    def __init__(self, device,parity, baudrate, stopbits, bytesize, wait_for_newline):
        gr.sync_block.__init__(self,
            name="usb_com_read",
            in_sig=None,
            out_sig=[numpy.int32])

        # cluster of samples from USB will be interleaved with a delimiter 0xffff (2 bytes)
        # this is to identify the start of the samples. 
        # In the data read, bytes till the first delimiter are purged.
        # in the worst case, cluster_len_samples of bytes can be lost.
        self.cluster_len_samples = 4
        self.cluster_len_bytes = self.cluster_len_samples * 2
        self.delimiter_len_bytes = 2
        self.delimiter = 0xff

        self.device = device
        self.parity = parity
        self.baudrate = baudrate
        self.stopbits = stopbits
        self.bytesize = bytesize
        self.wait_for_newline = wait_for_newline

        if self.parity == 0:
            self.parity = serial.PARITY_NONE
        elif self.parity == 1:
            self.parity = serial.PARITY_EVEN
        else:
            self.parity = serial.PARITY_ODD

        if self.stopbits == 1:
            self.stopbits = serial.STOPBITS_ONE
        elif self.stopbits == 2:
            self.stopbits = serial.STOPBITS_TWO

        if self.bytesize == 7:
            self.bytesize = serial.SEVENBITS
        else:
            self.bytesize = serial.EIGHTBITS

        # configure the serial connections (the parameters differs on the device you are connecting to)
        self.ser = serial.Serial(
            port=self.device,
            baudrate=self.baudrate,
            parity=self.parity,
            stopbits=self.stopbits,
            bytesize=self.bytesize,
            timeout=2
        )

        # Buffer and lock
        self.buff_lock = Lock()
        self.buff = []

        # rx_worker thread
        read_thread = Thread(target=self.rx_work, args=())
        read_thread.start()

    # Thread to read from the serial port and to write to the buff.
    # reads one line at a time and add to the buff.
    # @buff buffer to read into.
    # @buff_lock lock to serialize buffer access. Should be the lock from "Threading" package.
    # @ser instance of the serial class.
    def rx_work(self):
        """
        Lock mutex
            read from the serial port.
            Append the data to the self.buff list
        Unlock mutex
        """
        tmp_buff = []
        while 1:
            if self.ser.is_open is False:
                continue

            data_ser = self.ser.read(1024)
            tmp_buff.extend(data_ser)

            # Make sure we don't sleep if the lock() is taken. Instead read the serial port.
            try:
                locked = self.buff_lock.acquire(False)
                if locked:
                    self.buff.extend(tmp_buff)
                    del tmp_buff[:]
                else:
                    continue
            finally:
                if locked:
                    self.buff_lock.release()

    def find_start(self, buff):
        index = 0
        last_index = (self.cluster_len_bytes + (self.delimiter_len_bytes * 2) - 2)

        # if the array length is less than a cluster raise exception and return
        if len(buff) < last_index:
            raise IndexError
            return

        while index < last_index:
            if buff[index] == self.delimiter and buff[index + 1] == self.delimiter:
                return index + 2
            index = index + 1

        # raise an exception as the delimiter was not found.
        raise DelimiterNotFoundError

    # Convert the data to int from string object.
    # Remove the data from beginning until the first delimiter.
    # Remove the data from the end till the last delimiter by traversing back.
    # raise an exception if there is no delimiter.
    # remove delimiters
    # if the length of the self.buff is odd, raise an exception.
    # ##### TO BE CHANGED FROM HERE
    # Convert self.buff to uint16 and copy to output_items[0]

    # Convert self.buff to uint16 and store it in converted_buff.
    # Check for trigger condition and timer per division setting and copy only the required samples out from
    # converted_buff to output_items[0]
    # Clear self.buff till this point.
    #
    # for example:
    # find the sample matching the trigger condition.
    # from that sample we need to copy the number of samples based on Time per division setting.

    # Number of divisions on display = 10.
    # Sample rate = 2,000,00HZ => 5us.
    # Time per division setting = 100us
    # so each division will have 100us/5us=20 samples
    # the total samples to be considered from the point where trigger condition is satisfied is
    # 20 samples x 10 divisions = 200 samples.
    # take care of overflow.
    #
    # Trigger logic:
    # rising edge:
    # numpy.flatnonzero((conv_buff[1:] > trigger_level) & (conv_buff[:-1] < trigger_level)) + 1
    # falling edge:
    # numpy.flatnonzero((conv_buff[1:] < trigger_level) & (conv_buff[:-1] > trigger_level)) + 1
    #
    # @out output array to copy to. Should be a numpy array.
    def trigger_check(self, out):
        buff = list(map(ord, self.buff))

        # purge from the beginning
        if len(self.buff) < (self.cluster_len_bytes + (self.delimiter_len_bytes * 2) - 2):
            return 0
        idx = self.find_start(buff)
        buff = buff[idx:]

        # purge from the end
        rbuff = buff[::-1]

        if len(rbuff) < (self.cluster_len_bytes + (self.delimiter_len_bytes * 2) - 2):
            return 0
        idx = self.find_start(rbuff)
        buff = buff[:len(buff) - idx]

        # this is required to finally delete only the bytes consumed and leave the rest in the self.buff
        rem_bytes = idx

        # remove delimiters
        buff = filter(lambda elm: elm != 255, buff)

        # exception if length is odd.
        if (len(buff) % 2) == 1:
            raise IndexError

        # Convert self.buff to uint16 and copy
        n_buff = numpy.array(buff, dtype=numpy.uint8)
        copy_len = len(n_buff) >> 1
        out[0:copy_len] = n_buff.view(dtype=numpy.uint16)

        # clear the buffer
        del self.buff[:-rem_bytes]

        return copy_len

    def work(self, input_items, output_items):
        # Acquire the lock
        # copy buffer and return the number of items output
        # Release the lock
        out = output_items[0]
        try:
            locked = self.buff_lock.acquire(False)
            if locked:
                copy_len = self.trigger_check(out)
            else:
                copy_len = 0
        finally:
            if locked:
                self.buff_lock.release()

        return copy_len 

