"""Compile/link main3 with the configured ESP-IDF build, without changing CMake."""
from pathlib import Path
import ctypes
import json
import os
import re
import subprocess

root = Path(__file__).resolve().parents[2]
build = root / "build"
out = root / "tmp/main3_navigation"
out.mkdir(parents=True, exist_ok=True)


def windows_args(command):
    count = ctypes.c_int()
    parse = ctypes.windll.shell32.CommandLineToArgvW
    parse.argtypes = [ctypes.c_wchar_p, ctypes.POINTER(ctypes.c_int)]
    parse.restype = ctypes.POINTER(ctypes.c_wchar_p)
    pointer = parse(command, ctypes.byref(count))
    try:
        return [pointer[i] for i in range(count.value)]
    finally:
        ctypes.windll.kernel32.LocalFree(ctypes.cast(pointer, ctypes.c_void_p))


entries = json.loads((build / "compile_commands.json").read_text(encoding="utf-8"))
entry = next(e for e in entries if "/main/" in e["file"].replace("\\", "/")
             and e["file"].endswith(".c"))
args = windows_args(entry["command"])
args[args.index("-c") + 1] = str(root / "main/main3.c")
obj = out / "main3.obj"
args[args.index("-o") + 1] = str(obj)
args.extend(["-fstack-usage", "-Wstack-usage=2048"])
environment = os.environ.copy()
environment["PATH"] = str(Path(args[0]).parent) + os.pathsep + environment["PATH"]
subprocess.run(args, cwd=entry["directory"], env=environment, check=True)
print("PASS main3.c compiled for ESP32-S3", flush=True)

ninja = Path("C:/Users/Jhon/.espressif/tools/ninja/1.12.1/ninja.exe")
command_output = subprocess.check_output([str(ninja), "-t", "commands", "Test_NRF.elf"],
                                         cwd=build, text=True)
link_line = next(s for s in reversed(command_output.splitlines()) if "g++.exe" in s)
link_segment = next(s for s in link_line.split(" && ") if "g++.exe" in s)
link_args = windows_args(link_segment)
ninja_text = (build / "build.ninja").read_text(encoding="utf-8")
block = re.search(r"^build Test_NRF\.elf:.*(?:\n  .*?)*\n", ninja_text, re.MULTILINE).group(0)
libraries = re.search(r"^  LINK_LIBRARIES = (.*)$", block, re.MULTILINE).group(1)
assert "esp-idf/main/libmain.a" in libraries
libraries = libraries.replace("esp-idf/main/libmain.a", '"' + obj.as_posix() + '"')
search_paths = re.search(r"^  LINK_PATH = (.*)$", block, re.MULTILINE)
response = out / "main3-link.rsp"
response.write_text("CMakeFiles/Test_NRF.elf.dir/project_elf_src_esp32s3.c.obj "
                    + (search_paths.group(1) if search_paths else "") + " " + libraries,
                    encoding="utf-8")
link_args = ["@" + str(response) if arg.startswith("@") else arg for arg in link_args]
link_args[link_args.index("-o") + 1] = str(out / "main3-check.elf")
link_args = ["-Wl,--Map=" + (out / "main3-check.map").as_posix()
             if arg.startswith("-Wl,--Map=") else arg for arg in link_args]
subprocess.run(link_args, cwd=build, env=environment, check=True)
print("PASS main3-check.elf linked with ESP-IDF libraries; main/CMakeLists.txt unchanged")
