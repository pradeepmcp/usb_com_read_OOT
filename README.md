# usb_com_read_OOT
This is a GNURADIO OOT module to read data bytes from USB virtual com port. Internally makes use of pyserial. 
the output vector length is 1 and type is numpy.int32. 

The bit length of the data from USB is expected to be uin16_t(16bits). The data from USB is to be interleaved with a delimiter, 0xffff (16bits). After every four samples (each sample is 2 bytes), a delimiter is placed.  
