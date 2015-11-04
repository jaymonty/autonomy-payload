#!/usr/bin/env python

# Library to support recording and playing back ACS network messages

import struct
import time

import ap_lib.acs_messages as m

class Logger(object):
    ''' Abstract superclass for ACS network logging '''

    MARKER = '50'
    HEADER = '>2sLH'
    HEADER_SZ = struct.calcsize(HEADER)
    PARSE_MIN_SZ = HEADER_SZ + m.Message.hdr_size
    READ_MIN_SZ = 256

    def _check_marker(self, data):
        if data is None or \
           len(data) < len(self.MARKER) or \
           data[0:len(self.MARKER)] != self.MARKER:
            return False
        return True

    def _serialize(self, msg, t=None):
        if t is None:
            t = time.time()
        t_s = int(t)
        t_ms = int((t - t_s) * 1e3)
        hdr = struct.pack(self.HEADER, self.MARKER, t_s, t_ms)
        return hdr + msg.serialize()

    def _deserialize(self, data):
        if len(data) < self.PARSE_MIN_SZ:
            return None
        try:
            hdr = struct.unpack_from(self.HEADER, data, 0)
            if hdr[0] != self.MARKER:
                return None
            t = hdr[1] + hdr[2] / 1e3
            msg = m.Message.parse(data[self.HEADER_SZ:])
            return t, msg
        except:
            return None

    def __del__(self):
        self.close()

    def close(self):
        try:
            self._file.close()
        except:
            pass

class LogWriter(Logger):
    ''' Write ACS network messages to a log file '''

    def __init__(self, filename):
        self._file = open(filename, 'w')

    def write(self, msg, t=None):
        data = self._serialize(msg, t)
        self._file.write(data)
        self._file.flush()

class LogReader(Logger):
    ''' Read ACS network messages from a log file '''

    def __init__(self, filename):
        self._file = open(filename, 'r')
        self._buff = ''

    def readNext(self):
        while True:
            if len(self._buff) < self.READ_MIN_SZ:
                self._buff += self._file.read(self.READ_MIN_SZ)
            if len(self._buff) < self.PARSE_MIN_SZ:
                return None

            if not self._check_marker(self._buff):
                self._buff = self._buff[1:]
                continue

            t_and_msg = self._deserialize(self._buff)
            if t_and_msg is None:
                self._buff = self._buff[1:]
                continue
            else:
                msg = t_and_msg[1]
                hdr_msg_sz = self.HEADER_SZ + msg.msg_size
                self._buff = self._buff[hdr_msg_sz:]

            return t_and_msg

class NetRecorder(object):
    ''' Record to file from a Socket '''

    def __init__(self, socket, filename):
        self._socket = socket
        self._writer = LogWriter(filename)

    def __del__(self):
        self.close()

    def close(self):
        self._writer.close()

    def loop(self, recv_callback=None):
        while True:
            try:
                msg = self._socket.recv()
            except:
                continue

            if msg is None:
                time.sleep(0.1)
                continue

            self._writer.write(msg)

            try:
                recv_callback(msg)
            except:
                pass

class NetPlayer(object):
    ''' Play back a file to a Socket '''

    def __init__(self, socket, filename):
        self._socket = socket
        self._reader = LogReader(filename)

    def __del__(self):
        self.close()

    def close(self):
        self._reader.close()

    def loop(self, send_callback=None):
        t_msg_start = None
        t_wal_start = None

        while True:
            t_and_msg = self._reader.readNext()
            if t_and_msg is None:
                return

            t_msg, msg = t_and_msg
            t_wal = time.time()

            if t_msg_start is None:
                t_msg_start = t_msg
                t_wal_start = t_wal
                t_elapsed = 0
            else:
                t_elapsed = t_msg - t_msg_start
                send_time = (t_msg - t_msg_start) + t_wal_start
                if send_time > t_wal:
                    time.sleep(send_time - t_wal)

            self._socket.sendExact(msg)

            try:
                send_callback(msg, t_elapsed)
            except:
                pass
