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
受信したコマンドを実行するモジュール
"""
from fsrobo_r_io import FSRoboRIO
import rblib
import CommandID
import ErrorCode

import traceback

class FSRoboRCCExecCommand(object):
    """
    コマンド実行クラス
    """

    # クラス変数
    # データ型のバイトサイズ
    _INT_BYTE_SIZE = 4

    # 16進数
    _HEXADECIMAL = 16

    # プログラム先読み動作
    _ASYNCM_READ = 0
    _ASYNCM_ON = 1
    _ASYNCM_OFF = 2

    # Neativeのパス動作のON/OFF状態の定義(Python Teaching Module
    _PASSM_READ = 0
    _PASSM_ON = 1
    _PASSM_OFF = 2

    # MDO設定
    _MDO_ALL = 255

    # マニピュレータの初期設定値
    _CMD_DEFAULT_CPSPEED = 8.0
    _CMD_DEFAULT_SPEED = 2
    _CMD_DEFAULT_ACCT = 0.4                                 # cpacctと共通
    _CMD_DEFAULT_DACCT = 0.4                                 # cpdacctと共通
    _CMD_DEFAULT_PASS = _PASSM_OFF
    _CMD_DEFAULT_OVERLAP = 0
    _CMD_DEFAULT_ZONE = 20

    # 多回転
    _CC_NOT_USE = 0xFF000000

    # ツールオフセットID
    _SETTOOL_ID_NOT_USE = 0
    _SETTOOL_ID_USE = 1

    # 姿勢情報
    _POSTURE_NONE = -1
    _POSTURE_DEFAULT = 7

    # マニピュレータの動作指定
    _RBCOORD_PTP = 0
    _RBCOORD_LINE = 1

    # PTP動作時の速度制限
    _SPEED_LIMIT_MAX = 100
    _SPEED_LIMIT_MIN = 1

    _IK_SOLVER_OPTION_DEFAULT = 0x11111111

    # Neativeとの通信
    _RBLIB_HOST = "127.0.0.1"
    _RBLIB_PORT = 12345

    # 共通クラス変数
    _posture = _POSTURE_DEFAULT

    def __init__(self, rblib_rob=None):
        """
        初期化
        """
        print "FSRoboRCCExecCommand init"
        if rblib_rob is None:
            print "rblib init"
            self._rblib = rblib.Robot(self._RBLIB_HOST, self._RBLIB_PORT)
        else:
            print "use existing rblib"
            self._rblib = rblib_rob

        # 各マニピュレータ制御用変数をデフォルト値に設定
        self._lin_speed = self._CMD_DEFAULT_CPSPEED
        self._jnt_speed = self._CMD_DEFAULT_SPEED
        self._dacctime = self._CMD_DEFAULT_DACCT
        self._acctime = self._CMD_DEFAULT_ACCT
        self._passm = self._CMD_DEFAULT_PASS
        self._overlap = self._CMD_DEFAULT_OVERLAP
        self._zone = self._CMD_DEFAULT_ZONE
        self._usingtool = self._SETTOOL_ID_NOT_USE
        self._isopened = False
        self._ope_permission=False

        # io
        self._io = FSRoboRIO()

    def open(self, operation=False):
        """
        コマンド実行を開始する
        """
        print "open execution"
        if self._isopened == False:
            self._rblib.open()
            if operation==True:
                print "get operation permission"
                result = self._rblib.acq_permission()
                if result == True:
                    print "open success"
                    self._isopened = True
                    self._ope_permission = True
                else:
                    print "open failed"
                    self._rblib.close()
            else:
                print "none operation permission"
                self._isopened = True
        return self._isopened

    def close(self):
        """
        コマンド実行を終了する
        """
        print "close execution"
        if self._isopened == True:
            self._isopened = False
            self._ope_permission = False
            self._rblib.rel_permission()
            self._rblib.close()
            print "close success"

    def update_operation_permission(self, permission):
        """
        操作権限の更新
        外部クラスでrblibの操作権限を管理している場合のみ使用

        引数：
            permission: 操作権限の状態
        戻り値： 現在の操作権限の状態を返す
        """
        print "update_operation_permission execution"
        # 操作権限を更新
        if permission == True:
            print "permission on"
            self._ope_permission = True
            # 操作権限を本当に持っているかを確認
            self._check_operation_permission()
        else:
            print "permission off"
            self._ope_permission = False
        return self._ope_permission

    def exec_command(self, command_id, exec_data, ret_data):
        """
        コマンドを実行する

        引数：
            command_id: 実行するコマンドID
            exec_data: 実行時に使用するデータ
            ret_data: 実行結果を返すための引数 ※参照変数
        戻り値： 関数の実行結果
        """
        print "exec_command execution"
        error_code = ErrorCode.SUCCESS
        try:
            if command_id == CommandID.JMOVE_PTP:
                error_code = self._cmd_jmove_ptp(exec_data)
            elif command_id == CommandID.MOVE_PTP:
                error_code = self._cmd_move_ptp(exec_data)
            elif command_id == CommandID.SPEED_PTP:
                error_code = self._cmd_speed_ptp(exec_data)
            elif command_id == CommandID.SPEED_LINE:
                error_code = self._cmd_speed_line(exec_data)
            elif command_id == CommandID.RTOJ:
                error_code = self._cmd_pos2joint(exec_data, ret_data)
            elif command_id == CommandID.QJMOVE_PTP:
                error_code = self._cmd_qjmove_ptp(exec_data)
            elif command_id == CommandID.SETTOOL:
                error_code = self._cmd_settool(exec_data)
            elif command_id == CommandID.SETBASE:
                error_code = self._cmd_setbase(exec_data)
            elif command_id == CommandID.HOME:
                error_code = self._cmd_home()
            elif command_id == CommandID.SETIO:
                error_code = self._cmd_setio(exec_data)
            elif command_id == CommandID.GETIO:
                error_code = self._cmd_getio(exec_data, ret_data)
            elif command_id == CommandID.SETADC:
                error_code = self._cmd_setadc(exec_data)
            elif command_id == CommandID.GETADC:
                error_code = self._cmd_getadc(ret_data)
            elif command_id == CommandID.JMOVE_LINE:
                error_code = self._cmd_jmove_line(exec_data)
            elif command_id == CommandID.MOVE_LINE:
                error_code = self._cmd_move_line(exec_data)
            elif command_id == CommandID.SETPOSTURE:
                error_code = self._cmd_setposture(exec_data)
            elif command_id == CommandID.GETPOSTURE:
                error_code = self._cmd_getposture(ret_data)
            elif command_id == CommandID.MARK:
                error_code = self._cmd_mark(ret_data)
            elif command_id == CommandID.JMARK:
                error_code = self._cmd_jmark(ret_data)
            elif command_id == CommandID.ABORTM:
                error_code = self._cmd_abortm(ret_data)
            elif command_id == CommandID.SYSSTS:
                error_code = self._cmd_syssts(exec_data, ret_data)
            else:
                print "cmdid error"
                print "cmdid:" + str(command_id)
                error_code = ErrorCode.COMMAND_ERROR

        except Exception:
            print "command execution error"
            traceback.print_exc()
            error_code = ErrorCode.DATA_ERROR

        print "exec_command ret_data:"
        print ret_data
        return error_code

    def check_open_state(self):
        """
        クラスがopen状態であるかを確認
        
        戻り値: open状態の有無 true: open、 false: close
        """
        print "check_open_state execution"
        return self._isopened

    def _cmd_home(self):
        """
        マニピュレータを原点に戻す

        戻り値:
            error_code: 関数の実行結果
        """
        print "_cmd_home execution"
        if self._check_operation_permission() == True:
            # ロボットの状態の初期化を実施
            self._initialize()
            
            # 原点に戻す
            res = self._rblib.jntmove(0, 0, 0, 0, 0, 0, self._jnt_speed, self._acctime, self._dacctime)
            error_code = self._create_error_code(res)
        else:
            print "not operation permission"
            error_code = ErrorCode.OPERATION_NONE_ERROR
        return error_code

    def _cmd_move_ptp(self, exec_data):
        """
        座標情報を使用してPTP動作でマニピュレータを操作する

        引数：
            exec_data: コマンド実行用データ JSON形式
        戻り値:
            error_code: 関数の実行結果
        """
        print "_cmd_move_ptp execution"
        # asyncmをOFFに設定する
        self._set_asyncm(self._ASYNCM_OFF)
        error_code = self._move(exec_data, self._RBCOORD_PTP)
        # 実行結果を返す
        return error_code

    def _cmd_move_line(self, exec_data):
        """
        座標情報を使用して直線補間動作でマニピュレータを操作する

        引数：
            exec_data: コマンド実行用データ JSON形式
        戻り値:
            error_code: 関数の実行結果
        """
        print "_cmd_move_line execution"
        # asyncmをOFFに設定する
        self._set_asyncm(self._ASYNCM_OFF)
        error_code = self._move(exec_data, self._RBCOORD_LINE)
        # 実行結果を返す
        return error_code

    def _cmd_jmove_ptp(self, exec_data):
        """
        軸情報を使用してPTP動作でマニピュレータを動かす

        引数:
            exec_data: コマンド実行用データ JSON形式
        戻り値:
            error_code: 関数の実行結果
        """
        print "_cmd_jmove_ptp execution"
        # asyncmをOFFに設定する
        self._set_asyncm(self._ASYNCM_OFF)
        error_code = self._jmove(exec_data, self._RBCOORD_PTP)

        # 実行結果を返す
        return error_code

    def _cmd_jmove_line(self, exec_data):
        """
        軸情報を使用して直線補間動作でマニピュレータを動かす

        引数:
            exec_data: コマンド実行用データ JSON形式
        戻り値:
            error_code: 関数の実行結果
        """
        print "_cmd_jmove_line execution"
        # asyncmをOFFに設定する
        self._set_asyncm(self._ASYNCM_OFF)
        error_code = self._jmove(exec_data, self._RBCOORD_LINE)

        # 実行結果を返す
        return error_code

    def _cmd_qjmove_ptp(self, exec_data):
        """
        軸情報を使用して先読みのPTP動作でマニピュレータを動かす

        引数:
            exec_data: コマンド実行用データ JSON形式
        戻り値:
            error_code: 関数の実行結果
        """
        print "_cmd_qjmove_ptp execution"
        # asyncmをONに設定する
        self._set_asyncm(self._ASYNCM_ON)
        error_code = self._jmove(exec_data, self._RBCOORD_PTP)
        return error_code

    def _cmd_speed_ptp(self, exec_data):
        """
        PTP動作時のスピードを設定

        引数：
            exec_data: コマンド実行用データ JSON形式
                SP: 設定するスピード情報
        戻り値： 関数の実行結果
        """
        print "_cmd_speed_ptp execution"
        error_code = ErrorCode.SUCCESS
        if self._check_operation_permission() == True:
            speed = exec_data["SP"]
            if speed >= self._SPEED_LIMIT_MIN and speed <= self._SPEED_LIMIT_MAX:
                print "speed set"
                self._jnt_speed = speed
            else:
                print "speed value error"
                error_code = ErrorCode.DATA_ERROR
        else:
            print "not operation permission"
            error_code = ErrorCode.OPERATION_NONE_ERROR
        return error_code

    def _cmd_speed_line(self, exec_data):
        """
        直線補間動作時のスピードを設定

        引数：
            exec_data: コマンド実行用データ JSON形式
                SP: 設定するスピード情報
        戻り値： 関数の実行結果
        """
        print "_cmd_speed_line execution"
        error_code = ErrorCode.SUCCESS
        if self._check_operation_permission() == True:
            self._lin_speed = exec_data["SP"]
        else:
            print "not operation permission"
            error_code = ErrorCode.OPERATION_NONE_ERROR
        return error_code

    def _cmd_pos2joint(self, exec_data, ret_data):
        """
        座標情報を軸情報に変換

        引数:
            exec_data: コマンド実行用データ JSON形式
                X: X座標
                Y: Y座標
                Z: Z座標
                Rx: 角度X座標
                Ry: 角度Y座標
                Rz: 角度Z座標
                P: 姿勢情報
            ret_data: コマンド実行結果を返す変数 JSON形式 ※参照変数
                J1: 1軸目の情報
                J2: 2軸目の情報
                J3: 3軸目の情報
                J4: 4軸目の情報
                J5: 5軸目の情報
                J6: 6軸目の情報
        戻り値: 関数の実行結果
        """
        print "_cmd_pos2joint execution"
        data_x = exec_data["X"]
        data_y = exec_data["Y"]
        data_z = exec_data["Z"]
        data_rx = exec_data["Rx"]
        data_ry = exec_data["Ry"]
        data_rz = exec_data["Rz"]
        posture = exec_data["P"]

        # 姿勢情報が-1の場合はクラスに設定されている値を使用
        if posture == self._POSTURE_NONE:
            print "set posture"
            posture = FSRoboRCCExecCommand._posture

        error_code = ErrorCode.SUCCESS
        res = self._rblib.r2j_mt(data_x, data_y, data_z, data_rz, data_ry, data_rx, posture, self._RBCOORD_LINE, self._CC_NOT_USE, self._IK_SOLVER_OPTION_DEFAULT)
        if res[0] == True:
            ret_data["J1"] = res[1]
            ret_data["J2"] = res[2]
            ret_data["J3"] = res[3]
            ret_data["J4"] = res[4]
            ret_data["J5"] = res[5]
            ret_data["J6"] = res[6]
        
        else:
            error_code = self._create_error_code(res)

        return error_code

    def _cmd_settool(self, exec_data):
        """
        ツールオフセットを設定

        引数:
            exec_data: コマンド実行用データ JSON形式
                X: X座標
                Y: Y座標
                Z: Z座標
                Rx: ラジアンX座標
                Ry: ラジアンY座標
                Rz: ラジアンZ座標
        戻り値: 関数の実行結果
        """
        print "_cmd_settool execution"
        data_x = exec_data["X"]
        data_y = exec_data["Y"]
        data_z = exec_data["Z"]
        data_rx = exec_data["Rx"]
        data_ry = exec_data["Ry"]
        data_rz = exec_data["Rz"]

        # ツールオフセット変更前にツールオフセットをリセットする
        ct_res = self._rblib.changetool(self._SETTOOL_ID_NOT_USE)
        ct_error_code = self._create_error_code(ct_res)
        if ct_error_code != ErrorCode.SUCCESS:
            return ct_error_code

        try:
            st_res = self._rblib.settool(self._SETTOOL_ID_USE, data_x, data_y, data_z, data_rz, data_ry, data_rx)
        except Exception:
            traceback.print_exc()
            # エラー発生前のツールオフセットの設定に戻す
            self._rblib.changetool(self._usingtool)
            return ErrorCode.DATA_ERROR

        st_error_code = self._create_error_code(st_res)
        if st_error_code != ErrorCode.SUCCESS:
            # エラーコードが返された場合
            return st_error_code

        self._rblib.changetool(self._SETTOOL_ID_USE)

        # 現在の使用しているツールIDを更新
        self._usingtool = self._SETTOOL_ID_USE
        return ErrorCode.SUCCESS

    def _cmd_setbase(self, exec_data):
        """
        ベースオフセットを設定

        引数:
            exec_data: コマンド実行用データ JSON形式
                X: X座標
                Y: Y座標
                Z: Z座標
                Rx: ラジアンX座標
                Ry: ラジアンY座標
                Rz: ラジアンZ座標
        戻り値: 関数の実行結果
        """
        print "_cmd_setbase execution"
        error_code = ErrorCode.SUCCESS
        if self._check_operation_permission() == True:
            data_x = exec_data["X"]
            data_y = exec_data["Y"]
            data_z = exec_data["Z"]
            data_rx = exec_data["Rx"]
            data_ry = exec_data["Ry"]
            data_rz = exec_data["Rz"]
        else:
            print "not operation permission"
            error_code = ErrorCode.OPERATION_NONE_ERROR
        return error_code

    def _cmd_setposture(self, exec_data):
        """
        姿勢情報の設定

        引数:
            exec_data: コマンド実行用データ JSON形式
            　P: 姿勢情報
        戻り値: 関数の実行結果
        """
        error_code = ErrorCode.SUCCESS
        if self._check_operation_permission() == True:
            posture = exec_data["P"]
            if posture >= 0 and posture <= 7:
                print "success set posture"
                FSRoboRCCExecCommand._posture = posture
            else:
                print "error set posture"
                error_code = ErrorCode.DATA_ERROR
        else:
            print "not operation permission"
            error_code = ErrorCode.OPERATION_NONE_ERROR
        return error_code

    def _cmd_getposture(self, ret_data):
        """
        姿勢情報の取得

        引数:
            ret_data: コマンド実行結果を返す変数 JSON形式 ※参照変数
            　P: 姿勢情報
        戻り値: 関数の実行結果
        """
        ret_data["P"] = FSRoboRCCExecCommand._posture
        return ErrorCode.SUCCESS

    def _cmd_setio(self, exec_data):
        """
        I/O出力を設定

        引数:
            exec_data: コマンド実行用データ JSON形式
                AD: 変更するメモリの開始アドレス番号
                SL: 変更する信号の文字列
        戻り値: 関数の実行結果
        """
        print "_cmd_setio execution"
        address = exec_data["AD"]
        signal = exec_data["SL"]

        self._io.dout(address, str(signal))

        return ErrorCode.SUCCESS

    def _cmd_getio(self, exec_data, ret_data):
        """
        I/Oの状態を取得

        引数:
            exec_data: コマンド実行用データ JSON形式
                SA: 出力するメモリの開始アドレス番号
                EA: 出力するメモリの終了アドレス番号
            ret_data: コマンド実行結果を返す変数 JSON形式 ※参照変数
                SL: 指定された範囲内のI/O状態
        戻り値: 関数の実行結果
        """
        print "_cmd_getio execution"
        start_address = exec_data["SA"]
        end_address = exec_data["EA"]

        signal = self._io.din(start_address, end_address)
        ret_data["SL"] = signal
        
        return ErrorCode.SUCCESS

    def _cmd_setadc(self, exec_data):
        """
        先端I/OのADC測定を設定

        引数:
            exec_data: コマンド実行用データ JSON形式
                CH: 設定するチャンネル
                MO: 測定するモード
        戻り値: 関数の実行結果
        """
        print "_cmd_setadc execution"
        channel = exec_data["CH"]
        mode = exec_data["MO"]

        self._io.set_adcparam(channel, mode)

        return ErrorCode.SUCCESS

    def _cmd_getadc(self, ret_data):
        """
        先端I/OのADC値を出力

        引数:
            ret_data: コマンド実行結果を返す変数 JSON形式 ※参照変数
                ADC: 2ch分の先端I/OのADCの値
        戻り値: 関数の実行結果
        """
        print "_cmd_getadc execution"
        adc = self._io.adcin()
        ret_data["ADC"] = adc

        return ErrorCode.SUCCESS

    def _cmd_mark(self, ret_data):
        """
        現在位置の座標情報を取得

        引数:
            ret_data: コマンド実行結果を返す変数 JSON形式 ※参照変数
                X: X座標
                Y: Y座標
                Z: Z座標
                Rx: ラジアンX座標
                Ry: ラジアンY座標
                Rz: ラジアンZ座標
                P: ロボットの姿勢情報
        戻り値: 関数の実行結果
        """
        print "_cmd_mark execution"

        error_code = ErrorCode.SUCCESS
        res = self._rblib.mark()
        if res[0] == True:
            ret_data["X"] = res[1]
            ret_data["Y"] = res[2]
            ret_data["Z"] = res[3]
            ret_data["Rz"] = res[4]
            ret_data["Ry"] = res[5]
            ret_data["Rx"] = res[6]
            ret_data["P"] = res[7]
        else:
            error_code = self._create_error_code(res)

        return error_code

    def _cmd_jmark(self, ret_data):
        """
        現在位置の軸情報を取得

        引数:
            ret_data: コマンド実行結果を返す変数 JSON形式 ※参照変数
                J1: 1軸目の情報
                J2: 2軸目の情報
                J3: 3軸目の情報
                J4: 4軸目の情報
                J5: 5軸目の情報
                J6: 6軸目の情報
        戻り値: 関数の実行結果
        """
        print "_cmd_jmark execution"

        error_code = ErrorCode.SUCCESS
        res = self._rblib.jmark()
        if res[0] == True:
            ret_data["J1"] = res[1]
            ret_data["J2"] = res[2]
            ret_data["J3"] = res[3]
            ret_data["J4"] = res[4]
            ret_data["J5"] = res[5]
            ret_data["J6"] = res[6]
        
        else:
            error_code = self._create_error_code(res)

        return error_code

    def _cmd_abortm(self, ret_data):
        """
        ロボット動作を中断

        引数:
            ret_data: コマンド実行結果を返す変数 JSON形式 ※参照変数
                ID: 中断した動作の動作ID
        戻り値: 関数の実行結果
        """
        print "_cmd_abortm execution"

        error_code = ErrorCode.SUCCESS
        res = self._rblib.abortm()
        if res[0] == True:
            ret_data["ID"] = res[1]
        
        else:
            error_code = self._create_error_code(res)

        return error_code

    def _cmd_syssts(self, exec_data, ret_data):
        """
        システム状態を取得

        引数:
            exec_data: コマンド実行用データ JSON形式
                TYPE: 取得対象データタイプ
            ret_data: コマンド実行結果を返す変数 JSON形式 ※参照変数
                RE: 取得データ
        戻り値: 関数の実行結果
        """
        print "_cmd_syssts execution"

        error_code = ErrorCode.SUCCESS
        res = self._rblib.syssts(exec_data["TYPE"])
        print('res: {}'.format(res))
        if res[0] == True:
            ret_data["RE"] = res[1]
        else:
            error_code = self._create_error_code(res)

        return error_code


    def _initialize(self):
        """
        クラス変数とマニピュレータの状態の初期化を行う
        """
        print "_initialize execution"
        # 各マニピュレータ制御用変数をデフォルト値に設定
        self._lin_speed = self._CMD_DEFAULT_CPSPEED
        self._jnt_speed = self._CMD_DEFAULT_SPEED
        self._dacctime = self._CMD_DEFAULT_DACCT
        self._acctime = self._CMD_DEFAULT_ACCT
        self._passm = self._CMD_DEFAULT_PASS
        self._overlap = self._CMD_DEFAULT_OVERLAP
        self._zone = self._CMD_DEFAULT_ZONE
        self._usingtool = self._SETTOOL_ID_NOT_USE
        # Neativeの状態を初期化
        # 移動が終わるまで待機
        self._rblib.joinm()
        self._rblib.changetool(self._usingtool)
        # 初期ではasyncmはOFFに設定する
        self._rblib.asyncm(self._ASYNCM_OFF)
        self._rblib.passm(self._passm)
        self._rblib.overlap(self._overlap)
        self._rblib.zone(self._zone)
        self._rblib.disable_mdo(self._MDO_ALL)

        FSRoboRCCExecCommand._posture = self._POSTURE_DEFAULT

    def _set_asyncm(self, status):
        """
        先読み動作の設定と関わる変数の状態を切り替える

        引数:
            status: 先読み動作の設定値
                1: ON
                2: OFF
        """
        print "_set_asyncm execution"
        if (status == self._ASYNCM_ON):
            print "setting asyncm on"
            # 動作時のパラメータを設定する
            self._acctime = 0
            self._dacctime = 0
            self._passm = self._PASSM_ON
        else:
            print "setting asyncm off"
            # 動作時のパラメータを設定
            self._acctime = self._CMD_DEFAULT_ACCT
            self._dacctime = self._CMD_DEFAULT_DACCT
            self._passm = self._PASSM_OFF

        # asyncmの設定
        asyncm_status = self._rblib.asyncm(self._ASYNCM_READ)
        if (status != asyncm_status[1]):
            # 設定する状態が現在の設定と一致しない場合のみ、設定する
            print "setting asyncm"
            # 移動が終わるまで待機
            self._rblib.joinm()
            # asyncmを設定する
            self._rblib.asyncm(status)

        # passmの設定
        passm_status = self._rblib.passm(self._PASSM_READ)
        if (self._passm != passm_status[1]):
            # 設定する状態が現在の設定と一致しない場合のみ、設定する
            print "setting passm"
            # 移動が終わるまで待機
            self._rblib.joinm()
            # passmを設定する
            self._rblib.passm(self._passm)

    def _move(self, exec_data, rbcoord):
        """
        座標情報を使用して指定された動作でマニピュレータを操作する

        引数：
            exec_data: コマンド実行用データ JSON形式
                X: X座標
                Y: Y座標
                Z: Z座標
                Rx: ラジアンX座標
                Ry: ラジアンY座標
                Rz: ラジアンZ座標
                P: ロボットの姿勢情報
                CC: ロボットの多回転情報
            rbcoord: マニピュレータの動作
                0: PTP動作
                1: 直線補間動作
        戻り値:
            error_code: 関数の実行結果
        """
        print "_move execution"

        # 受信データから座標データを取得
        pos_x = exec_data["X"]
        pos_y = exec_data["Y"]
        pos_z = exec_data["Z"]
        pos_rx = exec_data["Rx"]
        pos_ry = exec_data["Ry"]
        pos_rz = exec_data["Rz"]
        pos_posture = exec_data["P"]
        if "ATM" in exec_data:
            self._acctime = exec_data["ATM"]
        if "DTM" in exec_data:
            self._dacctime = exec_data["DTM"]

        if (rbcoord == self._RBCOORD_PTP):
            # 受信データから座標データを取得
            if "CC" in exec_data:
                pos_cc = int(exec_data["CC"], self._HEXADECIMAL)
            else:
                pos_cc = 0xFF000000
            if "SP" in exec_data:
                self._jnt_speed = exec_data["SP"]

            # 姿勢情報が-1の場合はクラスに設定されている値を使用
            if pos_posture == self._POSTURE_NONE:
                print "set posture"
                pos_posture = FSRoboRCCExecCommand._posture
            # PTP動作でマニピュレータを操作
            if pos_cc != self._CC_NOT_USE:
                print "ptpmove_mt execution"
                res = self._rblib.ptpmove_mt(pos_x, pos_y, pos_z, pos_rz, pos_ry, pos_rx,\
                                            pos_posture, self._RBCOORD_LINE, pos_cc, self._IK_SOLVER_OPTION_DEFAULT, self._jnt_speed,\
                                            self._acctime, self._dacctime)
            else:
                print "ptpmove execution"
                res = self._rblib.ptpmove(pos_x, pos_y, pos_z, pos_rz, pos_ry, pos_rx,\
                                            pos_posture, self._RBCOORD_LINE, self._jnt_speed,\
                                            self._acctime, self._dacctime)

            print "move_ptp result:"
            print res
        else:
            # 受信データから座標データを取得
            if "SP" in exec_data:
                self._lin_speed = exec_data["SP"]
            # 姿勢情報が-1の場合はクラスに設定されている値を使用
            if pos_posture == self._POSTURE_NONE:
                print "set posture"
                pos_posture = FSRoboRCCExecCommand._posture
            # 直線補間動作でマニピュレータを操作
            res = self._rblib.cpmove(pos_x, pos_y, pos_z, pos_rz, pos_ry, pos_rx,\
                                pos_posture, self._RBCOORD_LINE, self._lin_speed,\
                                self._acctime, self._dacctime)
            print "move_line result:"
            print res

        # 実行結果を生成
        error_code = self._create_error_code(res)
        # 実行結果を返す
        return error_code

    def _jmove(self, exec_data, rbcoord):
        """
        軸情報を使用して指定した動作でマニピュレータを動かす

        引数:
            exec_data: コマンド実行用データ JSON形式
                J1: 1軸目の情報
                J2: 2軸目の情報
                J3: 3軸目の情報
                J4: 4軸目の情報
                J5: 5軸目の情報
                J6: 6軸目の情報
            rbcoord: マニピュレータの動作
                0: PTP動作
                1: 直線補間動作
        戻り値:
            error_code: 関数の実行結果
        """
        print "_jmove execution"
        # タグチェック
        pos_j1 = exec_data["J1"]
        pos_j2 = exec_data["J2"]
        pos_j3 = exec_data["J3"]
        pos_j4 = exec_data["J4"]
        pos_j5 = exec_data["J5"]
        pos_j6 = exec_data["J6"]
        if "ATM" in exec_data:
            self._acctime = exec_data["ATM"]
        if "DTM" in exec_data:
            self._dacctime = exec_data["DTM"]

        # joint動作開始
        if (rbcoord == self._RBCOORD_PTP):
            # 受信データから座標データを取得
            if "SP" in exec_data:
                self._jnt_speed = exec_data["SP"]

            print "jntmove execution"
            res = self._rblib.jntmove(pos_j1, pos_j2, pos_j3, pos_j4, pos_j5, pos_j6, self._jnt_speed, self._acctime, self._dacctime)
        else:
            # 受信データから座標データを取得
            if "SP" in exec_data:
                self._lin_speed = exec_data["SP"]
            print "cpmove execution"
            p = self._rblib.j2r_mt(pos_j1, pos_j2, pos_j3, pos_j4, pos_j5, pos_j6, self._RBCOORD_LINE)
            x, y, z, rz, ry, rx = p[1:7]
            posture = p[7]
            res = self._rblib.cpmove(x, y, z, rz, ry, rx, posture, rbcoord, self._lin_speed, self._acctime, self._dacctime)
        
        # 実行結果を生成
        error_code = self._create_error_code(res)
        # 実行結果を返す
        return error_code

    def _create_error_code(self, result):
        """
        rblibから取得した結果をエラーコードに変換する

        引数:
            result: rblibから返された結果
        戻り値:
            error_code: コマンドの実行結果
        """
        print "_create_error_code excution"
        error_code = ErrorCode.SUCCESS
        if result[0] == False:
            print "command failed"
            print result
            if result[1] == 3 and result[2] == 1:
                print "not operation permission"
                error_code = ErrorCode.OPERATION_NONE_ERROR
            else:
                print "robot error"
                error_code = ErrorCode.ROBOT_ERROR
            print("error div: {}, code: {}".format(result[1], result[2]))
        return error_code

    def _check_operation_permission(self):
        """
        操作権限を持っているかを確認
        戻り値： 操作権限の有無 True:有り False:無し
        """
        print "_check_operation_permission execution"
        # 操作権限を本当に持っているかを確認
        if self._ope_permission == True:
            result = self._rblib.acq_permission()
            if result[0] == False:
                # 操作権限を持っていないので操作権限をOFFにする
                print "disable operation permission"
                self._ope_permission = False
        return self._ope_permission