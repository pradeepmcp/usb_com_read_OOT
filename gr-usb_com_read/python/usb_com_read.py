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

class usb_com_read(gr.sync_block):
    """
    docstring for block usb_com_read
    """

    def __init__(self, device,parity,baudrate,stopbits,bytesize,wait_for_newline):
        gr.sync_block.__init__(self,
            name="usb_com_read",
            in_sig=None,
            out_sig=[numpy.int32])

        self.device = device
        self.parity = parity
        self.baudrate = baudrate
        self.stopbits = stopbits
        self.bytesize = bytesize
        self.wait_for_newline = wait_for_newline
        # print(device,parity, baudrate, stopbits, bytesize, wait_for_newline)


        # set parity

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
        # self.rbuff = rb.ring_buffer(BUFF_SIZE)
        self.buff_lock = Lock()
        self.buff = [0]

        # write_thread = Thread(target=work_fn, args=(buff, buff_lock, ser))
        self.stop_threads= False
        read_thread = Thread(target=self.rx_work, args=(self.buff, self.buff_lock, self.ser))
        read_thread.start()

        '''
        self.ser = serial.Serial(
            port=self.device,
            baudrate=self.baudrate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS
        )
        '''

        """
        print "Opened: ",self.ser.portstr       # check which port was really used
        self.ser.write("hel;lkfsdsa;lkfjdsaflo\n\r")      # write a string
        #ser.close()             # close port
        """
    # Thread to read from the serial port and to write to the buff.
    # reads one line at a time and add to the buff.
    # @buff buffer to read into.
    # @buff_lock lock to serialize buffer access. Should be the lock from "threadding" package.
    # @ser instance of the serial class.
    def rx_work(self, buff, buff_lock, ser):
        """
        Lock mutex
            read from the serial port. ----?? how much data to be read.
            write the data into buffer.
            mark underflow or overflow if any.
        Unlock mutex
        """
        print("rx_work started")
        while 1:
            if self.stop_threads:
                break

            # Need to raise a non fatal exception.
            # raise IOvrror("Serial port is not open")
            if ser.is_open is False:
                continue

            """    
            if(self.wait_for_newline):
                line = ser.readline()
            else:
                line = ser.read()
            """
            tmp_buff = []
            try:
                tmp_buff = ser.read(1024)
            except:
                break
            print("rx_work")
            # bytes_read = len(tmp_buff)
            # print (bytes_read)

            with self.buff_lock:
                # buff = tmp_buff[:]
                buff.extend(tmp_buff[:])

            # print(tmp_buff[:])
            # print(buff[:])


    def work(self, input_items, output_items):
        # Acquire the lock
        # copy buffer and update the nooutput_items.
        # Release the lock

        out = output_items[0]
        out_len = len(out)

        buff_len = len(self.buff)

        # take care of overflow
        if out_len < buff_len:
            copy_len = out_len
        else:
            copy_len = buff_len

        # buff_np = numpy.array(self.buff)
        # buff_np_len = len(buff_np)
        # print(buff_np)

        with self.buff_lock:
            # copy from buff to output_items
            # out[:] = self.buff[:]
            out[:copy_len] = self.buff[:copy_len]

            # clear the buffer
            self.buff.clear()

        # print("buff_lock released")
        # print(self.buff[:])
        # print(out[:])

        return len(self.buff)

