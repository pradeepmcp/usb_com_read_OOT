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

from gnuradio import gr, gr_unittest
from gnuradio import blocks
from usb_com_read import usb_com_read

from threading import Thread
from threading import Lock
import serial

class qa_usb_com_read (gr_unittest.TestCase):

    def setUp (self):
        '''
        # configure the serial connections (the parameters differs on the device you are connecting to)
        self.ser = serial.Serial(
            port="/dev/ttyACM1",
            baudrate=9600,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=2
        )
        '''
        self.tb = gr.top_block ()
        
        '''
        ## Buffer and lock
        #self.rbuff = rb.ring_buffer(BUFF_SIZE)
        self.buff_lock = Lock()
        self.buff = [0]

        #write_thread = Thread(target=work_fn, args=(buff, buff_lock, ser))
        read_thread = Thread(target=self.rx_work, args=(self.buff, self.buff_lock, self.ser))
        read_thread.start()
        '''


    def tearDown (self):
        self.tb = None

    def test_001_t (self):
        # set up fga
        try:
            ucr = usb_com_read("/dev/ttyACM1",0,9600,1,8,False)
        except:
            ucr.stop_threads = True
       
        snk = blocks.vector_sink_f () 
        self.tb.connect(ucr, snk)
        self.tb.run ()
        result_data = snk.data()
        # check data
        #self.assertTupleEqual(result_data, 0)
        print(result_data)

        ucr.stop_threads = True
        



'''
    ## Thread to read from the serial port and to write to the buff. 
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
        while(1):
            ## Need to raise a non fatal exception. 
            #raise IOvrror("Serial port is not open")
            if(ser.is_open == False):
                continue
            
            """    
            if(self.wait_for_newline):
                line = ser.readline()
            else:
                line = ser.read()
            """
            tmp_buff = []
            tmp_buff = ser.read(1024)
            print("rx_work")
            bytes_read = len(tmp_buff)
            ##print (bytes_read)

            with self.buff_lock:
                ##buff = tmp_buff[:]
                buff.extend(tmp_buff[:])
                
            print(tmp_buff[:])
            print(buff[:])
       '''

if __name__ == '__main__':
    gr_unittest.run(qa_usb_com_read, "qa_usb_com_read.xml")
