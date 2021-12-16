# pid_test.py
import ctypes
import pathlib


def load(filename):
    # Load the shared library into ctypes
    return ctypes.CDLL(filename)


# Get the library
# this is object returned by gcc -shared -o pid_controller.so -fPIC pid_controller.c
pid_lib = load(pathlib.Path().absolute() / "pid_controller.so")

# create args for hi_lo_limits_w_status and declare return type
value = ctypes.c_float(999.0)
hi = ctypes.c_float(100.0)
lo = ctypes.c_float(0.0)
limit_en = ctypes.c_bool(True)
hi_sts = ctypes.c_bool(False)
lo_sts = ctypes.c_bool(False)
lim_sts = ctypes.c_bool(False)

pid_lib.hi_lo_limits_w_status.restype = ctypes.c_float


def hi_limit():
    # Test 1: check function of hi limit
    hi_value = pid_lib.hi_lo_limits_w_status(
        value,
        hi,
        lo,
        limit_en,
        ctypes.byref(hi_sts),
        ctypes.byref(lo_sts),
        ctypes.byref(lim_sts),
    )

    assert hi_value == 100.0
    assert hi_sts.value == True
    assert lim_sts.value == True


if __name__ == "__main__":
    hi_limit()
