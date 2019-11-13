# -*- coding: utf-8 -*-

# FSRobo-R Package BSDL
# ---------
# Copyright (C) 2019 FUJISOFT. All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without modification,
# are permitted provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation and/or
# other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its contributors
# may be used to endorse or promote products derived from this software without
# specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
# IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
# INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# ---------

import threading
import rblib
import re


class FSRoboRIO(object):
    def __init__(self):
        self._io = IO()
        self._io.init()

    def dout(self, addr, data):
        self._io.dout(addr, data)

    def din(self, *addr):
        if len(addr) == 1:
            retval = self._io.din(addr[0])
            return retval
        elif len(addr) == 2:
            retval = self._io.din(addr[0], addr[1])
            return retval
        else:
            raise ValueError('din')

    def adcin(self):
        retval = self._io.adcin()
        return retval

    def set_adcparam(self, ch, adc_mode):
        return self._io.set_adcparam(ch, adc_mode)

    def close(self):
        self._io.close()


class IO(object):

    def __init__(self):
      self._initialized = False
      self._rb = None
      self._lock = threading.Lock()

    def __del__(self):
        self.close()

    def close(self):
        if self._initialized and self._rb is not None:
            self._rb.close()
            self._initialized = False

    def init(self):
        self._rb = rblib.Robot('127.0.0.1', 12345)
        self._rb.open()
        self._initialized = True

        return True

    def dout(self, addr, data):
        with self._lock:
            da = re.sub('[^01*]', '', data)

            row = addr % 32
            col = addr / 32

            dat = self.make_data(da)
            data64 = self.replace('0' * 64, dat[1], row)
            mask64 = self.replace('1' * 64, dat[2], row)

            data_low = int(data64[32:], 2)
            mask_low = int(mask64[32:], 2)
            data_high = int(data64[:32], 2)
            mask_high = int(mask64[:32], 2)

            ret = self.send_data(col, data_low, mask_low, data_high, mask_high)
            return ret

    def send_data(self,col, d0, m0, d1, m1):
        self._rb.ioctrl(col, d0, m0, d1, m1)

    def din(self, *addr):
        with self._lock:
            col = addr[0] / 32
            offset = col * 32
            row = 63 -(addr[0] - offset)
            data = self.recv_data(col)
            if len(addr) == 1:
                return data[row:row+1]
            elif len(addr) == 2:
                row1 = 63 -(addr[1] - offset)
                return data[row1:row+1]
            else:
                raise ValueError('din')

    def recv_data(self, col):
         dummy = 2**32-1
         ret = self._rb.ioctrl(col,dummy,dummy,dummy,dummy)
         return self.i2bs(ret[2]) + self.i2bs(ret[1])

    def make_data(self, st):
            if isinstance(st,str) and len(st)<=32:
                rslt=st
                dat = re.sub('[^1]', '0', rslt)
                mask = re.sub('[^*]', '0', rslt).replace('*', '1')
                #print([rslt, dat, mask])
                return [rslt,dat,mask]
            else:
                raise ValueError('make_data')

    @classmethod
    def replace(cls, s, repl, n):
        if len(s) < n:
             result = s
        else:
            right_str = s[len(s) - n:]
            left_str = s[:len(s) - n - len(repl)]
            result = (left_str + repl + right_str)[-len(s):]
        return result

    @classmethod
    def i2bs(cls, n):
        return format(n, '032b')

    def adcin(self):
        with self._lock:
            dummy = 2**32 - 1
            data = self._rb.ioctrl(2, dummy, dummy, dummy, dummy)[1]
            adc0 = data & 0x0FFF
            adc1 = (data>>16) & 0x0FFF
            return ( adc0, adc1 )

    def set_adcparam(self, ch, adc_mode):
        with self._lock:
            if ch != 0 and ch != 1:
                raise ValueError('set_adcparam')
            if adc_mode == 0:
                data = 0x0 << 12
            elif adc_mode == 1:
                data = 0x1 << 12
            elif adc_mode == 2:
                data = 0x2 << 12
            else:
                raise ValueError('set_adcparam')
            mask = 0xFFFFCFFF
            if ch == 1:
                data <<= 16
                mask = 0xCFFFFFFF
            self.send_data(2, data, mask, 0xFFFFFFFF, 0xFFFFFFFF)
