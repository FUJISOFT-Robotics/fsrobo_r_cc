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
import threading
import uuid

class FSRoboRCCExecCommand(object):
    """
    コマンド実行クラス
    """

    # クラス変数
    _MOTION_MODE_NORMAL = 0
    _MOTION_MODE_ROS = 1
    _last_motion_mode = None
    _last_motion_commander_id = None

    # データ型のバイトサイズ
    _INT_BYTE_SIZE = 4

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
    _SPEED_LIMIT_MIN = 0

    _IK_SOLVER_OPTION_DEFAULT = 0x11111111

    # Neativeとの通信
    _RBLIB_HOST = "127.0.0.1"
    _RBLIB_PORT = 12345

    # 共通クラス変数
    _posture = _POSTURE_DEFAULT

    def __init__(self, rblib_rob):
        """
        初期化
        """
        self._p("FSRoboRCCExecCommand init")
        self._rblib = rblib_rob
        print('thread id: {}'.format(threading.current_thread().ident))
        self._motion_commander_id = uuid.uuid1()

        # 各マニピュレータ制御用変数をデフォルト値に設定
        self._lin_speed = self._CMD_DEFAULT_CPSPEED
        self._jnt_speed = self._CMD_DEFAULT_SPEED
        self._dacctime = self._CMD_DEFAULT_DACCT
        self._acctime = self._CMD_DEFAULT_ACCT
        self._usingtool = self._SETTOOL_ID_NOT_USE
        self._ope_permission = False

        # io
        self._io = FSRoboRIO()

    def _p(self, msg, *args):
        if True:
            if type(msg) is str:
                print(msg.format(*args))
            else:
                print(msg, args)

    def close(self):
        """
        コマンド実行を終了する
        """
        self._p("close execution")
        self._io.close()

    def update_operation_permission(self, permission):
        """
        操作権限の更新
        外部クラスでrblibの操作権限を管理している場合のみ使用

        引数：
            permission: 操作権限の状態
        戻り値： 現在の操作権限の状態を返す
        """
        self._p("update_operation_permission execution")
        self._ope_permission = permission

        return self.has_op_perm()

    def has_op_perm(self):
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
        #self._p("exec_command execution")

        op_commands = {
            CommandID.JMOVE_PTP: self._cmd_jmove_ptp,
            CommandID.MOVE_PTP: self._cmd_move_ptp,
            CommandID.SPEED_PTP: self._cmd_speed_ptp,
            CommandID.SPEED_LINE: self._cmd_speed_line,
            CommandID.QJMOVE_PTP: self._cmd_qjmove_ptp,
            CommandID.SETTOOL: self._cmd_settool,
            CommandID.SETBASE: self._cmd_setbase,
            CommandID.HOME: self._cmd_home,
            CommandID.JMOVE_LINE: self._cmd_jmove_line,
            CommandID.MOVE_LINE: self._cmd_move_line,
            CommandID.SETPOSTURE: self._cmd_setposture,
            CommandID.GETPOSTURE: self._cmd_getposture,
            CommandID.MARK: self._cmd_mark,
            CommandID.JMARK: self._cmd_jmark,
            CommandID.ABORTM: self._cmd_abortm
        }

        normal_commands = {
            CommandID.RTOJ: self._cmd_pos2joint,
            CommandID.SYSSTS: self._cmd_syssts,
            CommandID.SETIO: self._cmd_setio,
            CommandID.GETIO: self._cmd_getio,
            CommandID.SETADC: self._cmd_setadc,
            CommandID.GETADC: self._cmd_getadc
        }

        try:
            cmd = op_commands.get(command_id)
            if cmd is not None:
                if self.has_op_perm():
                    error_code = cmd(exec_data, ret_data)
                else:
                    self._p("operation is not permitted")
                    error_code = ErrorCode.OPERATION_NONE_ERROR
            else:
                cmd = normal_commands.get(command_id)
                if cmd is not None:
                    error_code = cmd(exec_data, ret_data)
                else:
                    self._p("cmdid error")
                    self._p("cmdid: {}", str(command_id))
                    error_code = ErrorCode.COMMAND_ERROR
        except Exception:
            self._p("command execution error")
            traceback.print_exc()
            error_code = ErrorCode.DATA_ERROR

        #self._p("exec_command ret_data:")
        #self._p(ret_data)
        return error_code

    def _cmd_home(self, exec_data, ret_data):
        """
        マニピュレータを原点に戻す

        戻り値:
            error_code: 関数の実行結果
        """
        print "_cmd_home execution"

        # ロボットの状態の初期化を実施
        self._reset_default_params()
        
        # 原点に戻す
        res = self._rblib.jntmove(0, 0, 0, 0, 0, 0, self._jnt_speed, self._acctime, self._dacctime)
        error_code = self._create_error_code(res)

        return error_code

    def _cmd_move_ptp(self, exec_data, ret_data):
        """
        座標情報を使用してPTP動作でマニピュレータを操作する

        引数：
            exec_data: コマンド実行用データ JSON形式
        戻り値:
            error_code: 関数の実行結果
        """
        self._p("_cmd_move_ptp execution")
        # asyncmをOFFに設定する
        self._set_normal_mode()
        error_code = self._move_ptp(exec_data)
        # 実行結果を返す
        return error_code

    def _cmd_move_line(self, exec_data, ret_data):
        """
        座標情報を使用して直線補間動作でマニピュレータを操作する

        引数：
            exec_data: コマンド実行用データ JSON形式
        戻り値:
            error_code: 関数の実行結果
        """
        self._p("_cmd_move_line execution")
        # asyncmをOFFに設定する
        self._set_normal_mode()
        error_code = self._move_line(exec_data)
        # 実行結果を返す
        return error_code

    def _cmd_jmove_ptp(self, exec_data, ret_data):
        """
        軸情報を使用してPTP動作でマニピュレータを動かす

        引数:
            exec_data: コマンド実行用データ JSON形式
        戻り値:
            error_code: 関数の実行結果
        """
        self._p("_cmd_jmove_ptp execution")
        # asyncmをOFFに設定する
        self._set_normal_mode()
        error_code = self._jmove_ptp(exec_data)

        # 実行結果を返す
        return error_code

    def _cmd_jmove_line(self, exec_data, ret_data):
        """
        軸情報を使用して直線補間動作でマニピュレータを動かす

        引数:
            exec_data: コマンド実行用データ JSON形式
        戻り値:
            error_code: 関数の実行結果
        """
        self._p("_cmd_jmove_line execution")
        # asyncmをOFFに設定する
        self._set_normal_mode()
        error_code = self._jmove_line(exec_data)

        # 実行結果を返す
        return error_code

    def _cmd_qjmove_ptp(self, exec_data, ret_data):
        """
        軸情報を使用して先読みのPTP動作でマニピュレータを動かす

        引数:
            exec_data: コマンド実行用データ JSON形式
        戻り値:
            error_code: 関数の実行結果
        """
        self._p("_cmd_qjmove_ptp execution")
        # asyncmをONに設定する
        self._set_ros_mode()
        error_code = self._jmove_ptp(exec_data)
        return error_code

    def _cmd_speed_ptp(self, exec_data, ret_data):
        """
        PTP動作時のスピードを設定

        引数：
            exec_data: コマンド実行用データ JSON形式
                SP: 設定するスピード情報
        戻り値： 関数の実行結果
        """
        self._p("_cmd_speed_ptp execution")
        error_code = ErrorCode.SUCCESS
        speed = exec_data["SP"]
        if speed > self._SPEED_LIMIT_MIN and speed <= self._SPEED_LIMIT_MAX:
            self._p("speed set: {}", speed)
            self._jnt_speed = speed
        else:
            self._p("speed value error")
            error_code = ErrorCode.DATA_ERROR
        return error_code

    def _cmd_speed_line(self, exec_data, ret_data):
        """
        直線補間動作時のスピードを設定

        引数：
            exec_data: コマンド実行用データ JSON形式
                SP: 設定するスピード情報
        戻り値： 関数の実行結果
        """
        self._p("_cmd_speed_line execution")
        error_code = ErrorCode.SUCCESS
        self._lin_speed = exec_data["SP"]
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
        self._p("_cmd_pos2joint execution")
        data_x = exec_data["X"]
        data_y = exec_data["Y"]
        data_z = exec_data["Z"]
        data_rx = exec_data["Rx"]
        data_ry = exec_data["Ry"]
        data_rz = exec_data["Rz"]
        posture = exec_data["P"]

        # 姿勢情報が-1の場合はクラスに設定されている値を使用
        if posture == self._POSTURE_NONE:
            self._p("set posture")
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

    def _cmd_settool(self, exec_data, ret_data):
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
        self._p("_cmd_settool execution")
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

        st_res = self._rblib.settool(self._SETTOOL_ID_USE, data_x, data_y, data_z, data_rz, data_ry, data_rx)

        st_error_code = self._create_error_code(st_res)
        if st_error_code != ErrorCode.SUCCESS:
            # エラーコードが返された場合
            # エラー発生前のツールオフセットの設定に戻す
            self._rblib.changetool(self._usingtool)
            return st_error_code

        self._rblib.changetool(self._SETTOOL_ID_USE)

        # 現在の使用しているツールIDを更新
        self._usingtool = self._SETTOOL_ID_USE
        return ErrorCode.SUCCESS

    def _cmd_setbase(self, exec_data, ret_data):
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
        self._p("_cmd_setbase execution")
        error_code = ErrorCode.SUCCESS
        data_x = exec_data["X"]
        data_y = exec_data["Y"]
        data_z = exec_data["Z"]
        data_rx = exec_data["Rx"]
        data_ry = exec_data["Ry"]
        data_rz = exec_data["Rz"]

        return error_code

    def _cmd_setposture(self, exec_data, ret_data):
        """
        姿勢情報の設定

        引数:
            exec_data: コマンド実行用データ JSON形式
            　P: 姿勢情報
        戻り値: 関数の実行結果
        """
        error_code = ErrorCode.SUCCESS

        posture = exec_data["P"]
        if posture >= 0 and posture <= 7:
            self._p("success set posture")
            FSRoboRCCExecCommand._posture = posture
        else:
            print "error set posture"
            error_code = ErrorCode.DATA_ERROR

        return error_code

    def _cmd_getposture(self, exec_data, ret_data):
        """
        姿勢情報の取得

        引数:
            ret_data: コマンド実行結果を返す変数 JSON形式 ※参照変数
            　P: 姿勢情報
        戻り値: 関数の実行結果
        """
        ret_data["P"] = FSRoboRCCExecCommand._posture
        return ErrorCode.SUCCESS

    def _cmd_setio(self, exec_data, ret_data):
        """
        I/O出力を設定

        引数:
            exec_data: コマンド実行用データ JSON形式
                AD: 変更するメモリの開始アドレス番号
                SL: 変更する信号の文字列
        戻り値: 関数の実行結果
        """
        self._p("_cmd_setio execution")
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
        self._p("_cmd_getio execution")
        start_address = exec_data["SA"]
        end_address = exec_data["EA"]

        signal = self._io.din(start_address, end_address)
        ret_data["SL"] = signal
        
        return ErrorCode.SUCCESS

    def _cmd_setadc(self, exec_data, ret_data):
        """
        先端I/OのADC測定を設定

        引数:
            exec_data: コマンド実行用データ JSON形式
                CH: 設定するチャンネル
                MO: 測定するモード
        戻り値: 関数の実行結果
        """
        self._p("_cmd_setadc execution")
        channel = exec_data["CH"]
        mode = exec_data["MO"]

        self._io.set_adcparam(channel, mode)

        return ErrorCode.SUCCESS

    def _cmd_getadc(self, exec_data, ret_data):
        """
        先端I/OのADC値を出力

        引数:
            ret_data: コマンド実行結果を返す変数 JSON形式 ※参照変数
                ADC: 2ch分の先端I/OのADCの値
        戻り値: 関数の実行結果
        """
        self._p("_cmd_getadc execution")
        adc = self._io.adcin()
        ret_data["ADC"] = adc

        return ErrorCode.SUCCESS

    def _cmd_mark(self, exec_data, ret_data):
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
        self._p("_cmd_mark execution")

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

    def _cmd_jmark(self, exec_data, ret_data):
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
        self._p("_cmd_jmark execution")

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

    def _cmd_abortm(self, exec_data, ret_data):
        """
        ロボット動作を中断

        引数:
            ret_data: コマンド実行結果を返す変数 JSON形式 ※参照変数
                ID: 中断した動作の動作ID
        戻り値: 関数の実行結果
        """
        self._p("_cmd_abortm execution")

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
        #self._p("_cmd_syssts execution")

        res = self._rblib.syssts(exec_data["TYPE"])
        #self._p('res: {}', res)
        error_code = self._create_error_code(res)
        if error_code == ErrorCode.SUCCESS:
            ret_data["RE"] = res[1]

        return error_code


    def _reset_default_params(self):
        """
        クラス変数とマニピュレータの状態の初期化を行う
        """
        self._p("_reset_default_params execution")
        # 各マニピュレータ制御用変数をデフォルト値に設定
        self._lin_speed = self._CMD_DEFAULT_CPSPEED
        self._jnt_speed = self._CMD_DEFAULT_SPEED
        self._dacctime = self._CMD_DEFAULT_DACCT
        self._acctime = self._CMD_DEFAULT_ACCT
        self._usingtool = self._SETTOOL_ID_NOT_USE
        # Neativeの状態を初期化
        # 移動が終わるまで待機
        self._rblib.joinm()
        self._rblib.changetool(self._SETTOOL_ID_NOT_USE)
        # 初期ではasyncmはOFFに設定する
        self._rblib.asyncm(self._ASYNCM_OFF)
        self._rblib.passm(self._CMD_DEFAULT_PASS)
        self._rblib.overlap(self._CMD_DEFAULT_OVERLAP)
        self._rblib.zone(self._CMD_DEFAULT_ZONE)
        self._rblib.disable_mdo(self._MDO_ALL)

        FSRoboRCCExecCommand._posture = self._POSTURE_DEFAULT

    def _set_ros_mode(self):
        current_id = self._motion_commander_id
        print('current_id: {}'.format(current_id))
        if FSRoboRCCExecCommand._last_motion_mode != FSRoboRCCExecCommand._MOTION_MODE_ROS \
                or FSRoboRCCExecCommand._last_motion_commander_id != current_id:
            FSRoboRCCExecCommand._last_motion_mode = FSRoboRCCExecCommand._MOTION_MODE_ROS
            FSRoboRCCExecCommand._last_motion_commander_id = current_id
            print('set ROS mode')
            self._rblib.joinm()
            self._acctime = 0
            self._dacctime = 0
            self._rblib.passm(self._PASSM_ON)
            self._rblib.asyncm(self._ASYNCM_ON)
            self._rblib.overlap(self._CMD_DEFAULT_OVERLAP)
            self._rblib.zone(self._CMD_DEFAULT_ZONE)
            self._rblib.disable_mdo(self._MDO_ALL)

    def _set_normal_mode(self):
        current_id = self._motion_commander_id
        print('current_id: {}'.format(current_id))
        if FSRoboRCCExecCommand._last_motion_mode != FSRoboRCCExecCommand._MOTION_MODE_NORMAL \
                or FSRoboRCCExecCommand._last_motion_commander_id != current_id:
            FSRoboRCCExecCommand._last_motion_mode = FSRoboRCCExecCommand._MOTION_MODE_NORMAL
            FSRoboRCCExecCommand._last_motion_commander_id = current_id
            print('set Normal mode')
            self._rblib.joinm()
            self._dacctime = self._CMD_DEFAULT_DACCT
            self._acctime = self._CMD_DEFAULT_ACCT
            self._rblib.passm(self._PASSM_OFF)
            self._rblib.asyncm(self._ASYNCM_OFF)
            self._rblib.overlap(self._CMD_DEFAULT_OVERLAP)
            self._rblib.zone(self._CMD_DEFAULT_ZONE)
            self._rblib.disable_mdo(self._MDO_ALL)

    def _move_ptp(self, exec_data):
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
        戻り値:
            error_code: 関数の実行結果
        """
        self._p("_move execution")

        # 受信データから座標データを取得
        pos_x = exec_data["X"]
        pos_y = exec_data["Y"]
        pos_z = exec_data["Z"]
        pos_rx = exec_data["Rx"]
        pos_ry = exec_data["Ry"]
        pos_rz = exec_data["Rz"]

        pos_posture = exec_data.get("P", self._POSTURE_NONE)
        if pos_posture == self._POSTURE_NONE:
            pos_posture = FSRoboRCCExecCommand._posture 

        acctime = exec_data.get("ATM", self._acctime)
        dacctime = exec_data.get("DTM", self._dacctime)
        pos_cc = int(exec_data("CC", "FF000000"), 16)
        speed = exec_data.get("SP", self._jnt_speed)

        # PTP動作でマニピュレータを操作
        if pos_cc != self._CC_NOT_USE:
            self._p("ptpmove_mt execution")
            res = self._rblib.ptpmove_mt(pos_x, pos_y, pos_z, pos_rz, pos_ry, pos_rx,
                                            pos_posture, self._RBCOORD_LINE, pos_cc, self._IK_SOLVER_OPTION_DEFAULT,
                                            speed, acctime, dacctime)
        else:
            self._p("ptpmove execution")
            res = self._rblib.ptpmove(pos_x, pos_y, pos_z, pos_rz, pos_ry, pos_rx,
                                        pos_posture, self._RBCOORD_LINE, speed, acctime, dacctime)

        self._p("move_ptp result:")
        self._p(res)

        # 実行結果を生成
        error_code = self._create_error_code(res)
        # 実行結果を返す
        return error_code

    def _move_line(self, exec_data):
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
        戻り値:
            error_code: 関数の実行結果
        """
        self._p("_move execution")

        # 受信データから座標データを取得
        pos_x = exec_data["X"]
        pos_y = exec_data["Y"]
        pos_z = exec_data["Z"]
        pos_rx = exec_data["Rx"]
        pos_ry = exec_data["Ry"]
        pos_rz = exec_data["Rz"]
        pos_posture = exec_data.get("P", self._POSTURE_NONE)
        if pos_posture == self._POSTURE_NONE:
            pos_posture = FSRoboRCCExecCommand._posture 
        acctime = exec_data.get("ATM", self._acctime)
        dacctime = exec_data.get("DTM", self._dacctime)
        speed = exec_data.get("SP", self._lin_speed)

        # 直線補間動作でマニピュレータを操作
        res = self._rblib.cpmove(pos_x, pos_y, pos_z, pos_rz, pos_ry, pos_rx,
                            pos_posture, self._RBCOORD_LINE, speed, acctime, dacctime)

        self._p("move_line result:")
        self._p(res)

        # 実行結果を生成
        error_code = self._create_error_code(res)
        # 実行結果を返す
        return error_code

    def _jmove_ptp(self, exec_data):
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
        戻り値:
            error_code: 関数の実行結果
        """
        self._p("_jmove execution")
        # タグチェック
        pos_j1 = exec_data["J1"]
        pos_j2 = exec_data["J2"]
        pos_j3 = exec_data["J3"]
        pos_j4 = exec_data["J4"]
        pos_j5 = exec_data["J5"]
        pos_j6 = exec_data["J6"]
        acctime = exec_data.get("ATM", self._acctime)
        dacctime = exec_data.get("DTM", self._dacctime)
        speed = exec_data.get("SP", self._jnt_speed)

        # joint動作開始
        self._p("jntmove execution: {} {} {} {} {} {} {} {} {}", pos_j1, pos_j2, pos_j3, pos_j4, pos_j5, pos_j6, speed, acctime, dacctime)
        res = self._rblib.jntmove(pos_j1, pos_j2, pos_j3, pos_j4, pos_j5, pos_j6, speed, acctime, dacctime)
        
        # 実行結果を生成
        error_code = self._create_error_code(res)
        # 実行結果を返す
        return error_code

    def _jmove_line(self, exec_data):
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
        戻り値:
            error_code: 関数の実行結果
        """
        self._p("_jmove execution")
        # タグチェック
        pos_j1 = exec_data["J1"]
        pos_j2 = exec_data["J2"]
        pos_j3 = exec_data["J3"]
        pos_j4 = exec_data["J4"]
        pos_j5 = exec_data["J5"]
        pos_j6 = exec_data["J6"]
        acctime = exec_data.get("ATM", self._acctime)
        dacctime = exec_data.get("DTM", self._dacctime)
        speed = exec_data.get("SP", self._lin_speed)

        # 受信データから座標データを取得
        self._p("cpmove execution")
        p = self._rblib.j2r_mt(pos_j1, pos_j2, pos_j3, pos_j4, pos_j5, pos_j6, self._RBCOORD_LINE)
        x, y, z, rz, ry, rx = p[1:7]
        posture = p[7]
        res = self._rblib.cpmove(x, y, z, rz, ry, rx, posture, self._RBCOORD_LINE, speed, acctime, dacctime)
        
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
        #self._p("_create_error_code excution")
        error_code = ErrorCode.SUCCESS
        if result[0] == False:
            self._p("command failed")
            self._p(result)
            if result[1] == 3 and result[2] == 1:
                self._p("not operation permission")
                error_code = ErrorCode.OPERATION_NONE_ERROR
            else:
                self._p("robot error")
                error_code = ErrorCode.ROBOT_ERROR
            self._p("error div: {}, code: {}", result[1], result[2])
        return error_code
