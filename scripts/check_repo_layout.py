#!/usr/bin/env python3
"""Layout & reference checker for the R2-Ethercat_4c workspace.

Usage: python check_repo_layout.py <repo_root>

Checks (read-only, no build required):
  A. stale tokens that must not exist anywhere in the tree
  B. every src/... path referenced by any CMakeLists.txt exists
  C. every project-header #include resolves (case-sensitive, via CMake include dirs)
  D. orphan headers inside linkx_soem_demo/include
  E. sources under linkx_soem_demo/src not referenced by CMakeLists
  F. launch files: every executable='...' exists as a CMake target of that package
  G. no project header reuses a C standard library name (math.h, string.h, ...)
Findings allowlists (KNOWN_*) are empty after the 2026-10 cleanup and kept for future use.
"""
import os
import re
import sys

# Historical leftovers (npcap headers, dead CMake guards, unbuilt sources) were
# removed in the 2026-10 cleanup round; these allowlists are intentionally empty.
KNOWN_CMAKE_MISSING = set()
KNOWN_ORPHAN_PREFIX = None
KNOWN_UNBUILT = set()
STALE_TOKENS = ['device/Remote', 'teleop.params', 'imu_spec_msg', 'radar_launch',
                'can_link_test', 'up_range_subscriber', 'src/tof/README', 'extra_tests/']

# C standard library header names that project headers must not reuse: a project
# header with the same name would shadow the system header depending on -I order.
SHADOWED_HEADER_NAMES = {
    'assert.h', 'complex.h', 'ctype.h', 'errno.h', 'fenv.h', 'float.h', 'inttypes.h',
    'iso646.h', 'limits.h', 'locale.h', 'math.h', 'setjmp.h', 'signal.h', 'stdalign.h',
    'stdarg.h', 'stdatomic.h', 'stdbool.h', 'stddef.h', 'stdint.h', 'stdio.h',
    'stdlib.h', 'stdnoreturn.h', 'string.h', 'tgmath.h', 'threads.h', 'time.h',
    'uchar.h', 'wchar.h', 'wctype.h',
}


def walk(root):
    out = []
    for dirpath, dirnames, filenames in os.walk(root):
        dirnames[:] = [d for d in dirnames if d != '.git']
        for fn in filenames:
            full = os.path.join(dirpath, fn)
            out.append(os.path.relpath(full, root).replace('\\', '/'))
    return sorted(out)


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    root = os.path.abspath(sys.argv[1])
    files = walk(root)
    fileset = set(files)
    problems, known = [], []

    # A. stale tokens
    text_ext = ('.c', '.cpp', '.h', '.hpp', '.py', '.md', '.txt', '.xml', '.yaml',
                '.yml', '.sh', '.cmake', '.txt')
    for f in files:
        if not f.endswith(text_ext):
            continue
        if os.path.basename(f) == 'check_repo_layout.py':
            continue  # the checker itself lists the tokens it searches for
        try:
            text = open(os.path.join(root, f), encoding='utf-8', errors='replace').read()
        except OSError:
            continue
        for token in STALE_TOKENS:
            if token in text:
                problems.append(f'stale token "{token}" in {f}')

    # B. CMake references exist
    for cmake in [f for f in files if f.endswith('CMakeLists.txt')]:
        pkgdir = os.path.dirname(cmake)
        text = open(os.path.join(root, cmake), encoding='utf-8').read()
        refs = sorted(set(re.findall(r'(?<![\w/])src/[A-Za-z0-9_./\-]+\.(?:cpp|hpp|py|c|h)\b', text)))
        for r in refs:
            if (pkgdir + '/' + r) not in fileset:
                (known if r in KNOWN_CMAKE_MISSING else problems).append(
                    f'CMake references missing file: {cmake} -> {r}')

    # C. case-sensitive include resolution in linkx_soem_demo
    pkg = 'src/linkx_soem_demo/'
    cmake_text = open(os.path.join(root, pkg + 'CMakeLists.txt'), encoding='utf-8').read()
    m = re.search(r'set\(PROJECT_INCLUDE_DIRS(.*?)\n\)', cmake_text, re.S)
    inc_dirs = re.findall(r'^\s+([\w/\-.]+)\s*$', m.group(1), re.M) if m else []
    inc_dirs = [d if d.startswith(pkg) else pkg + d for d in inc_dirs]
    for d in inc_dirs:
        if not any(f.startswith(d + '/') for f in files):
            problems.append(f'include dir missing on disk: {d}')
    project_headers = {os.path.basename(f) for f in files
                       if f.startswith(pkg) and f.endswith(('.h', '.hpp'))}
    for f in [x for x in files if x.startswith(pkg) and x.endswith(('.c', '.cpp', '.h', '.hpp'))]:
        text = open(os.path.join(root, f), encoding='utf-8', errors='replace').read()
        for inc in re.findall(r'#include\s+"([^"]+)"', text):
            if '/' not in inc and os.path.basename(inc) not in project_headers:
                continue  # system header quoted in the source
            norm = inc.replace('\\', '/')
            d = os.path.dirname(f)
            cands = [d + '/' + norm] + [i + '/' + norm for i in inc_dirs]
            if not any(c in fileset for c in cands):
                problems.append(f'unresolved include {f}: "{inc}"')

    # D. orphan headers
    inc_tokens = set()
    for f in [x for x in files if x.startswith(pkg) and x.endswith(('.c', '.cpp', '.h', '.hpp'))]:
        text = open(os.path.join(root, f), encoding='utf-8', errors='replace').read()
        inc_tokens.update(os.path.basename(i) for i in re.findall(r'#include\s+"([^"]+)"', text))
    for f in files:
        if f.startswith(pkg + 'include/') and f.endswith(('.h', '.hpp')) and os.path.basename(f) not in inc_tokens:
            (known if (KNOWN_ORPHAN_PREFIX and f.startswith(KNOWN_ORPHAN_PREFIX)) else problems).append(f'orphan header: {f}')

    # E. sources not referenced by CMake
    for f in files:
        if f.startswith(pkg + 'src/') and f.endswith(('.c', '.cpp')) and f[len(pkg):] not in cmake_text:
            (known if f in KNOWN_UNBUILT else problems).append(f'source not referenced by CMakeLists: {f}')

    # F. launch executables exist
    targets_by_pkg = {}
    for cmake in [f for f in files if f.endswith('CMakeLists.txt')]:
        p = os.path.dirname(cmake)
        text = open(os.path.join(root, cmake), encoding='utf-8').read()
        targets = set(re.findall(r'add_executable\(([\w\\-]+)', text))
        if 'add_executable(${PROJECT_NAME}' in text:
            targets.add(os.path.basename(p))
        targets_by_pkg[p] = targets
    for f in [x for x in files if x.endswith('.launch.py')]:
        text = open(os.path.join(root, f), encoding='utf-8').read()
        for pkg_name, exe in re.findall(r"package='([\w\-]+)',\s*\n?\s*executable='([\w\-]+)'", text):
            p = 'src/' + pkg_name
            if p in targets_by_pkg and exe not in targets_by_pkg[p]:
                problems.append(f'launch {f}: executable "{exe}" not defined in {p}/CMakeLists.txt')

    # G. project headers must not shadow C standard library names
    for f in files:
        if f.startswith('src/') and os.path.basename(f).lower() in SHADOWED_HEADER_NAMES:
            problems.append(f'project header shadows a C standard library name: {f}')

    print('=== RESULT:', 'FAIL' if problems else 'OK', '===')
    for p in problems:
        print('PROBLEM:', p)
    print('--- known pre-existing findings (informational) ---')
    for k in sorted(set(known)):
        print('KNOWN:', k)
    print('--- summary ---')
    print(f'files scanned: {len(files)}; problems: {len(problems)}; known: {len(set(known))}')
    return 1 if problems else 0


if __name__ == '__main__':
    sys.exit(main())

