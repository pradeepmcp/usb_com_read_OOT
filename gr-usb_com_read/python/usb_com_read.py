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
        # print(device,parity, baudrate, stopbits, bytesize, wait_for_newline)

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
        # sef.buff = numpy.array((),dtype=numpy.uint8)

        self.stop_threads= False

        # rx_worker thread
        read_thread = Thread(target=self.rx_work, args=())
        read_thread.start()

    # Thread to read from the serial port and to write to the buff.
    # reads one line at a time and add to the buff.
    # @buff buffer to read into.
    # @buff_lock lock to serialize buffer access. Should be the lock from "threadding" package.
    # @ser instance of the serial class.
    def rx_work(self):
        """
        Lock mutex
            read from the serial port. ----?? how much data to be read.
            Append the data to the self.buff list
            mark underflow or overflow if any.
        Unlock mutex
        """
        while 1:
            if self.stop_threads:
                break

            # Need to raise a non fatal exception.
            # raise IOvrror("Serial port is not open")
            if self.ser.is_open is False:
                continue

            # tmp_buff = numpy.array([1024],dtype=numpy.uint8)

            # Add exception handling
            try:
                tmp_buff = self.ser.read(1024)
            except:
                break
            
            #print(type(tmp_buff))
            #tmp_buff = list(map(ord,tmp_buff))

            # print("rx_work")
            # print(type(tmp_buff[0]))
            # print(type(tmp_buff))
            # self.bytes_read = len(tmp_buff)
            # print (bytes_read)

            with self.buff_lock:
                print("rx_wokr() entered here")
                self.buff.extend(tmp_buff)
                # numpy.insert(self.buff, len(self.buff), tmp_buff)
                # print(tmp_buff[:])
                # print(self.buff[:])
            print("rx_work() exited here")

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

    def work(self, input_items, output_items):
        # Acquire the lock
        # copy buffer and update the nooutput_items.
        # Release the lock

        out = output_items[0]
        """
        out_len = len(out)

        buff_len = len(self.buff)

        # take care of overflow
        if out_len < buff_len:
            copy_len = out_len
        else:
            copy_len = buff_len
        """

        # buff_np = numpy.array(self.buff)
        # buff_np_len = len(buff_np)
        # print(self.buff)
        # print(copy_len)

        with self.buff_lock:
            # Convert the data to int from string object.
            # Remove the data from beginning until the first delimiter.
            # Remove the data from the end till the last delimiter by traversing back.
            # raise an exception if there is no delimiter.
            # remove delimiters
            # if the length of the self.buff is odd, raise an exception.
            # Convert self.buff to uint16 and copy to output_items[0]
            # take care of overflow.

            # print("self.buff len ", len(self.buff))
            buff = list(map(ord, self.buff))

            # purge from the beginning
            # print("front purge ", len(buff))
            if len(self.buff) < (self.cluster_len_bytes + (self.delimiter_len_bytes * 2) - 2):
                return 0
            idx = self.find_start(buff)
            buff = buff[idx:]

            # purge from the end
            # print("rear purge ", len(buff))
            rbuff = buff[::-1]

            if len(rbuff) < (self.cluster_len_bytes + (self.delimiter_len_bytes * 2) - 2):
                return 0
            idx = self.find_start(rbuff)
            buff = buff[:len(buff)-idx]

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
            # out[0:copy_len] = self.buff[0:copy_len]

            # Debug prints
            print("copy_len")
            print(copy_len)
            # print(self.buff[:copy_len])
            print(out[:copy_len])

            # clear the buffer
            bytes_left = len(rbuff) - (len(buff))
            del self.buff[:-rem_bytes]
            # Below is wrong. Not sure how it is working.
            # numpy.delete(self.buff, slice(None, None), 0)

            print(len(out))
            # print("buff_lock released")

        return copy_len 
        # return len(out)

