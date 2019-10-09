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

"""
コマンドIDの定義
"""
# Robot操作コマンド
PROGRAM = 0x000
HOME = 0x100
JMOVE_PTP = 0x101
MOVE_PTP = 0x102
SPEED_PTP = 0x103
SPEED_LINE = 0x104
RTOJ = 0x105
QJMOVE_PTP = 0x106
SETTOOL = 0x107
SETBASE = 0x108
JMOVE_LINE = 0x109
MOVE_LINE = 0x10A
SETPOSTURE = 0x10B
GETPOSTURE = 0x10C
MARK = 0x10D
JMARK = 0x10E
ABORTM = 0x10F
SYSSTS = 0x110

# I/O操作コマンド
SETIO = 0x200
GETIO = 0x201
SETADC = 0x202
GETADC = 0x203

# その他
NOCOMMAND = 0xFFF

