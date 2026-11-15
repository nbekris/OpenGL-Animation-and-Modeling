"""Compile the real Path.cpp and open a dependency-free browser debugger.

Run: python tools/debug_path.py
Requires Python 3 and Visual Studio C++ tools (or g++ on PATH).
"""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import webbrowser


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--no-open', action='store_true')
    parser.add_argument('--seconds', type=float, default=16.0)
    args = parser.parse_args()
    if not 0 < args.seconds < float('inf'):
        parser.error('--seconds must be finite and positive')
    root = Path(__file__).resolve().parent.parent
    engine = root / 'CS560Framework'
    with tempfile.TemporaryDirectory(prefix='path-debug-') as tmp:
        executable = Path(tmp) / 'sample_path.exe'
        sources = [str(root / 'tools/sample_path.cpp'), str(engine / 'Path.cpp')]
        includes = [str(engine), str(engine / 'include')]
        if shutil.which('g++'):
            command = ['g++', '-std=c++11', *sources,
                       *['-I' + p for p in includes], '-o', str(executable)]
        else:
            command = ['cl', '/nologo', '/EHsc', *sources,
                       *['/I' + p for p in includes], '/Fe:' + str(executable)]
            if not shutil.which('cl'):
                vswhere = Path(os.environ.get('ProgramFiles(x86)', 'C:/Program Files (x86)')) / 'Microsoft Visual Studio/Installer/vswhere.exe'
                if not vswhere.exists():
                    parser.error('Install Visual Studio C++ build tools or put g++ on PATH.')
                install = subprocess.check_output([
                    str(vswhere), '-latest', '-products', '*', '-requires',
                    'Microsoft.VisualStudio.Component.VC.Tools.x86.x64',
                    '-property', 'installationPath'], text=True).strip()
                if not install:
                    parser.error('No Visual Studio installation with C++ tools found.')
                setup = Path(install) / 'VC/Auxiliary/Build/vcvars64.bat'
                batch = Path(tmp) / 'build.cmd'
                batch.write_text('@echo off\ncall "' + str(setup) +
                                 '" >nul\nif errorlevel 1 exit /b 1\n' +
                                 subprocess.list2cmdline(command) + '\n')
                command = ['cmd', '/c', str(batch)]
        subprocess.run(command, cwd=tmp, check=True)
        data = json.loads(subprocess.check_output([str(executable)], text=True))
    data['seconds'] = args.seconds
    template = (root / 'tools/path_debug.html').read_text(encoding='utf-8')
    output = Path(tempfile.mkdtemp(prefix='path-viewer-')) / 'path_debug.html'
    output.write_text(template.replace('__PATH_DATA__', json.dumps(data)), encoding='utf-8')
    print('Maximum control-point error:', data['positionError'])
    print('Maximum estimated tangent mismatch:', data['tangentError'])
    print('Maximum wrap error:', data['wrapError'])
    print('Viewer:', output)
    if not args.no_open:
        webbrowser.open(output.as_uri())


if __name__ == '__main__':
    main()
