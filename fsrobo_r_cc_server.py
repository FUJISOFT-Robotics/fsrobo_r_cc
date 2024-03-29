#!/usr/bin/env python
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
FSRobo-R CCのサーバーモジュール
"""

import socket
import json

from struct import pack, unpack
import os
import sys
import fsrobo_r_cc_exec_command
import fsrobo_r_cc_exec_program
import shutil
import CommandID
import ErrorCode

import traceback
import threading
import rblib


class FSRoboRCCServer(object):
    """
    CC デーモンプログラム
    コントローラ外部からのプログラム実行、コマンドによる操作を受信するプロセス間通信クラス
    """

    # クラス変数
    #_SOCKET_IP_ADDRESS = "192.168.0.23"
    _SOCKET_IP_ADDRESS = "0.0.0.0"
    _SOCKET_PORT_NUMBER = 5500
    _SOCKET_BACKLOG = 1
    _CONNECT_DEVICE_MAX = 3

    # Nativeとの通信
    _RBLIB_HOST = "127.0.0.1"
    _RBLIB_PORT = 12345
    
    def __init__(self):
        """
        初期化
        """
        print "CCServer initalize"
        self._connection_thread = [None, None, None]
        self._rb = None
        self._rb_use_count = 0
        self._lock = threading.Lock()

    def start(self):
        """
        ソケット通信の受信処理
        """
        print "CCServer.start()"

        os.umask(0)
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self._SOCKET_IP_ADDRESS, self._SOCKET_PORT_NUMBER))
        sock.listen(self._SOCKET_BACKLOG)

        while True:

            try:
                print "accept execution"
                connection, _ = sock.accept()
                print "accept success"
                connect_permission = False
                index = 0
                while index < self._CONNECT_DEVICE_MAX and connect_permission == False:
                    # 空いているソケットが存在するかを確認
                    if self._connection_thread[index] is None:
                        print "can connect, because socket is none"
                        connect_permission = True
                    else:
                        if self._connection_thread[index].isAlive() == False:
                            print "can connect, because socket is not alive"
                            connect_permission = True
                    
                    if connect_permission == False:
                        index+=1

                rb = self._get_robot()
                service = ServiceThread(connection, connect_permission, rb, self._thread_terminates)
                service.daemon = True
                service.start()
                if connect_permission == True:
                    print "give connect permission"
                    self._connection_thread[index] = service

            except Exception:
                print "catch Exception. go out from while loop"
                print traceback.print_exc()
                break

        print "break while"
        sock.close()
        sys.exit(0)

    def _get_robot(self):
        with self._lock:
            if self._rb is None:
                print('open connection to robot')
                self._rb = rblib.Robot(self._RBLIB_HOST, self._RBLIB_PORT)
                self._rb.open()
                self._rb.acq_permission()

            self._rb_use_count += 1

            return self._rb

    def _release_robot(self):
        with self._lock:
            self._rb_use_count -= 1

            if self._rb_use_count == 0:
                print('close connection to robot')
                self._rb.close()
                self._rb = None


    def _thread_terminates(self):
        print('thread terminated')
        self._release_robot()

class ServiceThread(threading.Thread):
    """
    CC サービススレッド
    受信したデータを使用してマニピュレータを操作するクラス
    """
    # ソケット受信サイズ
    _SOCKET_RECV_BUFF_SIZE = 4096
    # ソケットのタイムアウト
    _SOCKET_RECV_TIMEOUT = 10
    _SOCKET_RECV_TIMEOUT_WAIT = None

    # データの種別
    _DATA_TYPE_CMD = 0x00
    _DATA_TYPE_PROGRAM = 0x01
    _DATA_TYPE_CONNECT_CHECK = 0x02
    _DATA_TYPE_OPERATION_GET = 0x03

    # jsonタグ
    _JSON_TAG_COMMAND = "CD"
    _JSON_TAG_PROCESS = "PR"
    _JSON_TAG_DATATYPE = "DT"
    _JSON_TAG_DATA = "DA"
    _JSON_TAG_REPLY = "RE"

    # ファイル削除のフラグ
    _FILE_DELETE_TRUE = 1


    # コンストラクタ
    def __init__(self, connection, connect_permission, robot, terminate_callback):
        """
        初期化
        """
        super(ServiceThread, self).__init__()
        print "ServiceThread initialize"
        self._connection = connection
        self._connect_permission = connect_permission
        self._operation_permission = False
        self._terminate_callback = terminate_callback
        # rblibクラスを開く
        self._rblib = robot

        # コマンド実行クラスを初期化
        self._exec_command = fsrobo_r_cc_exec_command.FSRoboRCCExecCommand(self._rblib)

    def _p(self, s, *args):
        if False:
            print(s.format(*args))

    # 実行関数
    def run(self):
        """
        スレッド内の実行処理
        """
        # Teachモジュールからのコマンドを受信する
        while True:
            try:
                rec_msg = self._socket_receive(self._connection)
            except Exception:
                # エラー出力
                self._p("receive error")
                self._p(traceback.print_exc())
                break

            if len(rec_msg) > 0:
                self._p("rec_msg:")
                self._p(rec_msg)
                send_msg = self._exec_recv_cmd(rec_msg)
                try:
                    self._connection.send(send_msg)
                except Exception:
                    # エラー出力
                    self._p("send error")
                    self._p(traceback.print_exc())
                    break
            else:
                self._p("socket close")
                self._p( "rec_msg: {}", rec_msg)
                break

        # ソケット通信終了
        # コマンド実行クラスを閉じる
        self._exec_command.close()
        # ソケットを閉じる
        self._connection.close()

        # rblibクラスを閉じる
        #self._rblib.close()
        
        self._terminate_callback()

    def _socket_receive(self, socket_obj):
        """
        ソケット通信の受信データ取得処理

        引数:
            socket_obj: ソケット通信のオブジェクト
        戻り値:
            rec_msgs: クライアントからの受信データ
        """
        # 受信データを取得
        self._p("_socket_receive function")
        rec_msgs = socket_obj.recv(self._SOCKET_RECV_BUFF_SIZE)
        if len(rec_msgs) > 0:
            self._p("rec_msgs length: {}", len(rec_msgs))
            # 受信データを全て受信するまでループ
            while self._check_recv_message(rec_msgs) == False:
                self._p("_check_recv_message Result False")
                # 無限ループ回避の為、タイムアウト時間を指定
                socket_obj.settimeout(self._SOCKET_RECV_TIMEOUT)
                try:
                    rec_msg = socket_obj.recv(self._SOCKET_RECV_BUFF_SIZE)
                except socket.timeout:
                    # クライアントに内部データエラーを返す
                    self._p("socket time out")
                    break
                # 受信データのサイズが0の場合、エラーが発生したと見なす
                if len(rec_msg) == 0:
                    raise Exception
                rec_msgs = rec_msgs + rec_msg

            # タイムアウトの設定を待機状態にする
            socket_obj.settimeout(self._SOCKET_RECV_TIMEOUT_WAIT)

        return rec_msgs

    def _check_recv_message(self, rec_msg):
        """
        クライアントから受信したデータを全て受信しているかを判断

        引数:
            rec_msg: クライアントから受信したデータ
        戻り値:
            True: 受信完了
            False: 受信途中
        """
        self._p("_check_recv_message function")
        decoder = json.JSONDecoder()
        try:
            decoder.raw_decode(rec_msg)
        except ValueError:
            self._p("JSON Decode Error")
            return False
        return True

    def _exec_recv_cmd(self, rec_msg):
        """
        クライアントから受信した命令を実行

        引数：
            rec_msg: クライアント側から受信したデータ
        戻り値:
            res_buf: 実行結果のデータ
        """
        self._p("function _exec_recv_cmd execution")
        ret_data = {}
        error_code = ErrorCode.SUCCESS

        # 受信データを各変数に切り分け
        try:
            json_data = json.loads(rec_msg)
            self._p("json_data:")
            self._p(json_data)
            cmd_id = json_data[self._JSON_TAG_COMMAND]
            sender_process = json_data[self._JSON_TAG_PROCESS]
            data_type = json_data[self._JSON_TAG_DATATYPE]
            exec_data = json.loads(json_data[self._JSON_TAG_DATA])
        except (KeyError, ValueError):
            # 受信データが異常な場合
            # クライアント側に排他制御中のエラーコードを返す
            self._p(traceback.print_exc())
            error_code = ErrorCode.DATA_ERROR
            # 送信する実行結果を作成
            res_buf = self._create_return_data(CommandID.NOCOMMAND, error_code, ret_data)
            # 実行結果を送信
            return res_buf

        #TODO 各データ確認
        self._p("cmd_id: {}", cmd_id)
        self._p("sender_process: {}", sender_process)
        self._p("data_type: {}", data_type)
        self._p("exec_data: {}", exec_data)

        self._p("Check connect permission")
        # 受信データが接続中のプロセスのものかを確認
        if self._connect_permission == False:
            # 接続権限が無い場合
            # クライアント側に排他制御中のエラーコードを返す
            print "Disable Connect Process"
            error_code = ErrorCode.PROCESS_ERROR
            # 送信する実行結果を作成
            res_buf = self._create_return_data(cmd_id, error_code, ret_data)
            # 実行結果を送信
            return res_buf

        self._p("Check Data")
        # 受信したデータを判断
        if data_type == self._DATA_TYPE_PROGRAM and cmd_id == CommandID.PROGRAM:
            # プログラムの場合
            try:
                path = exec_data["PATH"]
                del_flg = exec_data["DEL"]
                param = "{}"
                if exec_data.has_key("PAR"):
                    param = exec_data["PAR"]
            except KeyError:
                # クライアント側に内部データ異常のエラーコードを返す
                self._p("Key Error")
                error_code = ErrorCode.DATA_ERROR
                # 送信する実行結果を作成
                res_buf = self._create_return_data(cmd_id, error_code, ret_data)
                # 実行結果を送信
                return res_buf

            # 操作権の確認
            if self._operation_permission == True:
                # プログラムを実行
                self._p("Program Data")
                fsrobo_r_cc_exec_command.FSRoboRCCExecCommand._last_motion_id = None
                exec_program = fsrobo_r_cc_exec_program.FSRoboRCCExecProgram()

                # 実行するプログラムに操作権を渡す必要があるので一時的に操作権開放
                self._rblib.rel_permission()
                # プログラムを実行
                error_code = exec_program.exec_program(path, param)
                # 操作権を再取得
                result = self._rblib.acq_permission()
                if result[0] == False:
                    # 操作権の取得に失敗した場合、操作権フラグをFalseに設定
                    self._p("operation get error")
                    self._operation_permission = False
                    self._exec_command.update_operation_permission(self._operation_permission)
            else:
                self._p("Not operation permission")
                error_code = ErrorCode.OPERATION_NONE_ERROR

            if del_flg == self._FILE_DELETE_TRUE:
                    # 実行したプログラムを削除
                    self._delete_program_file(path)

        elif data_type == self._DATA_TYPE_CMD:
            # コマンドの場合
            self._p("Command Data")
            error_code = self._exec_command.exec_command(cmd_id, exec_data, ret_data)

        elif data_type == self._DATA_TYPE_CONNECT_CHECK:
            # 接続確認の場合
            self._p("Connect check data")
            error_code = ErrorCode.SUCCESS

        elif data_type == self._DATA_TYPE_OPERATION_GET:
            # 操作権限取得の場合
            self._p("Operation get data")
            self._operation_permission = True
            self._exec_command.update_operation_permission(self._operation_permission)
            error_code = ErrorCode.SUCCESS
        else:
            self._p("ErrorData")
            # データ種別の値が異常な場合
            # クライアント側に内部データ異常のエラーコードを返す
            error_code = ErrorCode.DATA_ERROR

        # 送信する実行結果を作成
        res_buf = self._create_return_data(cmd_id, error_code, ret_data)
        # 実行結果を送信
        return res_buf

    def _create_return_data(self, cmd_id, error_code, ret_data):
        """
        クライアント側に実行結果を返すためのデータを作成

        引数:
            cmd_id: 実行したコマンドID
            error_code: エラーコード
            ret_data: コマンド実行による出力
        戻り値:
            res_buf: 実行結果のデータ
        """
        self._p("_create_return_data execution")
        send_json = {
            self._JSON_TAG_COMMAND: cmd_id,
            self._JSON_TAG_REPLY: error_code,
            self._JSON_TAG_DATA: json.dumps(ret_data)
        }
        send_data = json.dumps(send_json)
        pack_format = str(len(send_data)) + "s"
        res_buf = pack(pack_format, send_data)
        return res_buf

    def _delete_program_file(self, path):
        """
        クライアント側から受信したプログラムファイルの削除

        引数：
            path： 削除するファイルのパス
        """
        self._p("_delete_program_file execution")
        if os.path.exists(path):
            self._p("exist path")
            name_index = path.rfind("/")
            folder_path = path[0:name_index]
            shutil.rmtree(folder_path)

if __name__ == "__main__":
    """
    main関数
    """
    print "main execution"
    cc_server = FSRoboRCCServer()
    cc_server.start()
