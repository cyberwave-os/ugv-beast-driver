"""Small compatibility shim that adapts a standard logger interface to a
callable that accepts format-style arguments.

The Cyberwave SDK may call logging methods with format arguments (e.g.
logger.info("msg %s", arg)). ROS's rclpy logger implementations (RcutilsLogger)
do not accept format args. This shim accepts (msg, *args, **kwargs), formats
the final string safely, and forwards it to the underlying logger.
"""
from __future__ import annotations

from typing import Any


class LoggerShim:
    def __init__(self, underlying: Any) -> None:
        self._underlying = underlying

    def _format(self, msg, *args, **kwargs) -> str:
        # Try %-formatting first (common for std logging), fall back to
        # str(msg) if formatting fails.
        try:
            if args:
                return msg % args
            return str(msg)
        except Exception:
            try:
                return msg.format(*args, **kwargs)
            except Exception:
                return f"{msg} {args} {kwargs}"

    def info(self, msg, *args, **kwargs) -> None:
        try:
            self._underlying.info(self._format(msg, *args, **kwargs))
        except Exception:
            try:
                print(self._format(msg, *args, **kwargs))
            except Exception:
                pass

    def debug(self, msg, *args, **kwargs) -> None:
        try:
            self._underlying.debug(self._format(msg, *args, **kwargs))
        except Exception:
            try:
                print(self._format(msg, *args, **kwargs))
            except Exception:
                pass

    def warning(self, msg, *args, **kwargs) -> None:
        try:
            self._underlying.warning(self._format(msg, *args, **kwargs))
        except Exception:
            try:
                print(self._format(msg, *args, **kwargs))
            except Exception:
                pass

    def error(self, msg, *args, **kwargs) -> None:
        try:
            self._underlying.error(self._format(msg, *args, **kwargs))
        except Exception:
            try:
                print(self._format(msg, *args, **kwargs))
            except Exception:
                pass

    def exception(self, msg, *args, **kwargs) -> None:
        # If underlying has exception(), prefer it; otherwise fall back to
        # error(). Mirror the traceback if we fall back to error().
        import traceback
        import sys
        
        try:
            formatted = self._format(msg, *args, **kwargs)
            
            # Manually extract traceback
            exc_info = sys.exc_info()
            if exc_info[0] is not None:
                tb_lines = traceback.format_exception(*exc_info)
                
                # ROS loggers (RcutilsLogger) often truncate at newlines.
                # If we are in ROS, log each line separately to ensure visibility.
                exc_func = getattr(self._underlying, 'exception', None)
                if callable(exc_func):
                    # If it has a real exception() method (like standard Python logging)
                    exc_func("".join(tb_lines))
                else:
                    # Fallback for ROS/RcutilsLogger: log lines individually
                    self._underlying.error(formatted)
                    for line in tb_lines:
                        # Indent for readability in the console
                        clean_line = line.rstrip()
                        if clean_line:
                            self._underlying.error(f"  {clean_line}")
                    return
                
            # If no traceback or handled above, do standard logging
            exc_func = getattr(self._underlying, 'exception', None)
            if callable(exc_func):
                exc_func(formatted)
            else:
                self._underlying.error(formatted)
        except Exception:
            try:
                print(f"LoggerShim failure: {msg} {args}", file=sys.stderr)
            except Exception:
                pass
