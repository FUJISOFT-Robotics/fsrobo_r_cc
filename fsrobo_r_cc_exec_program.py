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
受信したプログラムを実行するモジュール
"""

import subprocess
import ErrorCode
import rblib
import time
import os

class FSRoboRCCExecProgram(object):
    """
    プログラム実行クラス
    """
    # Neativeとの通信
    _RBLIB_HOST = "127.0.0.1"
    _RBLIB_PORT = 12345

    # Neativeの初期化
    _DEFAULT_TOOL_ID = 0
    _DEFAULT_ASYNCM = 2
    _DEFAULT_PASS = 2
    _DEFAULT_OVERLAP = 0
    _DEFAULT_ZONE = 20
    _MDO_ALL = 255

    def exec_program(self, path, param):
        """
        プログラムを実行

        引数:
            path: 実行するプログラムの絶対パス
            param: プログラム実行時に使用するパラメータ JSON形式
        戻り値:
            error_code: 関数の実行結果
        """
        print "exec_program execution"
        # マニピュレータの状態を初期化
        rb = rblib.Robot(self._RBLIB_HOST, self._RBLIB_PORT)
        rb.open()
        rb.acq_permission()
        rb.changetool(self._DEFAULT_TOOL_ID)
        # 初期ではasyncmはOFFに設定する
        rb.asyncm(self._DEFAULT_ASYNCM)
        rb.passm(self._DEFAULT_PASS)
        rb.overlap(self._DEFAULT_OVERLAP)
        rb.zone(self._DEFAULT_ZONE)
        rb.disable_mdo(self._MDO_ALL)
        rb.rel_permission()
        rb.close()

        # プログラムを実行
        error_code = ErrorCode.SUCCESS
        # ファイルとPATHに分ける
        sppath = path.rsplit("/", 1)
        # 実行フォルダに移動
        os.chdir(sppath[0])
        # ファイル実行
        proc = subprocess.Popen(["python", sppath[1], param], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        proc.wait()
        error_message = proc.stderr.read()
        if len(error_message) != 0:
            print error_message
            error_code = ErrorCode.PROGRAM_ERROR
        # FSRobo-R Python APIのcloseで非同期のabortmが実行されるためabortmの完了を待つ
        time.sleep(1)
        return error_code