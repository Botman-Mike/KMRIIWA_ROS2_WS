# Monkey-patch built-in print to include NTP-synced timestamp
import builtins
import datetime as _datetime
_orig_print = builtins.print
def print(*args, **kwargs):
    timestamp = _datetime.datetime.utcnow().isoformat() + 'Z'
    _orig_print(timestamp, *args, **kwargs)
builtins.print = print

# ...existing code...