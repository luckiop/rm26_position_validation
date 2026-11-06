#!/usr/bin/env python3
"""Exercise native case editing and repeated solves through XTest under Xvfb."""
import ctypes as ct
import ctypes.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import time


class Keyboard:
    """Send real key events to the GLFW window without a window manager."""

    def __init__(self):
        if not ct.util.find_library('Xtst'):
            print('XTest unavailable; skipping native keyboard test')
            sys.exit(77)
        self.x = ct.CDLL('libX11.so.6')
        self.test = ct.CDLL(ct.util.find_library('Xtst'))
        signatures = {
            'XOpenDisplay': ([ct.c_char_p], ct.c_void_p),
            'XDefaultRootWindow': ([ct.c_void_p], ct.c_ulong),
            'XQueryTree': ([ct.c_void_p, ct.c_ulong, ct.POINTER(ct.c_ulong),
                            ct.POINTER(ct.c_ulong), ct.POINTER(ct.POINTER(ct.c_ulong)),
                            ct.POINTER(ct.c_uint)], ct.c_int),
            'XFetchName': ([ct.c_void_p, ct.c_ulong, ct.POINTER(ct.c_char_p)], ct.c_int),
            'XFree': ([ct.c_void_p], ct.c_int),
            'XSetInputFocus': ([ct.c_void_p, ct.c_ulong, ct.c_int, ct.c_ulong], ct.c_int),
            'XStringToKeysym': ([ct.c_char_p], ct.c_ulong),
            'XKeysymToKeycode': ([ct.c_void_p, ct.c_ulong], ct.c_uint),
            'XFlush': ([ct.c_void_p], ct.c_int),
            'XCloseDisplay': ([ct.c_void_p], ct.c_int),
        }
        for name, (arguments, result) in signatures.items():
            function = getattr(self.x, name)
            function.argtypes, function.restype = arguments, result
        self.test.XTestFakeKeyEvent.argtypes = [ct.c_void_p, ct.c_uint, ct.c_int, ct.c_ulong]
        self.display = self.x.XOpenDisplay(None)
        assert self.display, 'cannot open display'

    def windows(self, window):
        name = ct.c_char_p()
        self.x.XFetchName(self.display, window, ct.byref(name))
        label = name.value.decode(errors='replace') if name.value else ''
        if name:
            self.x.XFree(name)
        yield window, label
        root, parent, count = ct.c_ulong(), ct.c_ulong(), ct.c_uint()
        children = ct.POINTER(ct.c_ulong)()
        if self.x.XQueryTree(self.display, window, ct.byref(root), ct.byref(parent),
                            ct.byref(children), ct.byref(count)):
            identifiers = list(children[:count.value])
            if children:
                self.x.XFree(children)
            for child in identifiers:
                yield from self.windows(child)

    def focus(self):
        for _ in range(600):
            windows = self.windows(self.x.XDefaultRootWindow(self.display))
            matches = [window for window, name in windows if 'Arm Planning Demo' in name]
            if matches:
                # GLFW creates the named window before mapping it on screen.
                time.sleep(1.0)
                self.x.XSetInputFocus(self.display, matches[-1], 2, 0)
                self.x.XFlush(self.display)
                time.sleep(0.3)
                return
            time.sleep(0.05)
        raise AssertionError('viewer window not found')

    def key(self, name, down):
        symbol = self.x.XStringToKeysym(name.encode())
        code = self.x.XKeysymToKeycode(self.display, symbol)
        self.test.XTestFakeKeyEvent(self.display, code, down, 0)
        self.x.XFlush(self.display)

    def tap(self, name):
        self.key(name, True)
        self.key(name, False)

    def close(self):
        self.x.XCloseDisplay(self.display)


def wait_result(path, process):
    """Wait for a complete result while checking the solver remains alive."""
    for _ in range(400):
        if path.exists():
            try:
                return json.loads(path.read_text())
            except ValueError:
                pass
        assert process.poll() is None, 'case process exited before saving result'
        time.sleep(0.05)
    raise AssertionError(f'result not saved: {path}')


def main():
    keyboard = Keyboard()
    # case0007, with nonzero parking to detect render/base coordinate mistakes.
    position = [0.0464962735830917, -0.018292744915797382, 0.4547546044982915]
    rotation = [0.29387002197888706, 0.2538105819319064,
                -0.29321797370325914, 0.8736382652899715]
    with tempfile.TemporaryDirectory() as temporary:
        output = Path(temporary)
        (output / 'config.json').write_text('{"test":true}')
        (output / 'case.json').write_text('{"id":"case_0007"}')
        command = [sys.argv[1], *map(str, position), *map(str, rotation), '2', str(output), '1',
                   '0', '0.1', '-0.1', '0', '0', str(3.141592653589793), '0', '-0.1', '0.1']
        with (output / 'stdout').open('w') as stdout, (output / 'stderr').open('w') as stderr:
            process = subprocess.Popen(command, stdout=stdout, stderr=stderr)
            try:
                original = wait_result(output / 'result.json', process)
                keyboard.focus()
                keyboard.key('i', True)
                time.sleep(1.2)
                keyboard.key('i', False)
                time.sleep(0.15)
                keyboard.tap('p')
                edited = wait_result(output / 'interactive/edit_1/result.json', process)
                moved = edited['core_in_base']['position_m']
                assert moved[0] > position[0] + 0.01, (position, moved)
                assert all(abs(moved[i] - position[i]) < 1e-10 for i in (1, 2))
                assert all(abs(a - b) < 1e-12 for a, b in
                           zip(rotation, edited['core_in_base']['quaternion_wxyz']))
                time.sleep(0.3)
                keyboard.key('Shift_L', True)
                keyboard.key('j', True)
                time.sleep(1.2)
                keyboard.key('j', False)
                keyboard.key('Shift_L', False)
                time.sleep(0.15)
                keyboard.tap('Return')
                directory = output / 'interactive/edit_2'
                rotated = wait_result(directory / 'result.json', process)
                assert any(abs(a - b) > 0.01 for a, b in
                           zip(rotation, rotated['core_in_base']['quaternion_wxyz']))
                assert all(abs(a - b) < 1e-10 for a, b in
                           zip(moved, rotated['core_in_base']['position_m']))
                saved = json.loads((directory / 'request.json').read_text())
                assert [float(value) for value in saved['command'][1:4]] == moved
                assert saved['command'][10] == '0'
                assert json.loads((directory / 'config.json').read_text()) == {'test': True}
                assert json.loads((directory / 'source_case.json').read_text())['id'] == 'case_0007'
                assert json.loads((output / 'result.json').read_text()) == original
                keyboard.tap('Escape')
                assert process.wait(timeout=8) == 0
                print('Native case translation, rotation, repeated planning and snapshots passed')
            except BaseException:
                stderr.flush()
                print((output / 'stderr').read_text(), file=sys.stderr)
                raise
            finally:
                if process.poll() is None:
                    process.terminate()
                    process.wait(timeout=8)
                keyboard.close()


if __name__ == '__main__':
    main()
