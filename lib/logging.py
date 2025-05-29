from logging import *
import sys


class ColorfulFormatter(Formatter):
    _grey = "\033[90m"
    _yellow = "\033[93m"
    _red = "\033[31m"
    _bold_red = "\033[91m"
    _reset = "\033[0m"
    _lineno = " (%(filename)s:%(lineno)d)"
    _format = "[%(name)s] %(message)s"

    FORMATS = {
        DEBUG: _grey + _format + _lineno + _reset,
        INFO: _format + _reset,
        WARNING: _yellow + _format + _lineno + _reset,
        ERROR: _red + _format + _lineno + _reset,
        CRITICAL: _bold_red + _format + _lineno + _reset
    }

    def format(self, record):
        log_fmt = self.FORMATS.get(record.levelno)
        formatter = Formatter(log_fmt)
        return formatter.format(record)


class ColoredLogger(Logger):
    def __init__(self, name, level=DEBUG):
        Logger.__init__(self, name, level)

        color_formatter = ColorfulFormatter()

        console = StreamHandler()
        console.setFormatter(color_formatter)

        self.addHandler(console)
        return


setLoggerClass(ColoredLogger)
