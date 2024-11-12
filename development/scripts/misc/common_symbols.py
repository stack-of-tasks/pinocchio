#! /usr/bin/env python3
# Find common dynamics public symbols between shared libraries.

import argparse
import itertools
import pathlib
import subprocess
import typing


def generate_symbols(shared_library: pathlib.Path) -> typing.Set[str]:
    # Show symbol
    # -D: Dynamic
    # -C: Demangled
    # -U: Defined
    # -W: Non weak
    result = subprocess.run(
        ["nm", "-DCUW", "--format=sysv", str(shared_library)],
        capture_output=True,
        text=True,
    )
    output = result.stdout
    lines_split = (line.split("|") for line in output.splitlines() if "|" in line)
    # Only keep lines with exported (upper case) symbols.
    # `u` is also a global symbol, but if we always build with compatible libraries,
    # there is no issue to find it in many places.
    return set([line[0].strip() for line in lines_split if line[2].strip().isupper()])


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        prog="common_symbol",
        description="Find common dynamics public symbols between shared libraries.",
    )
    parser.add_argument("shared_libraries", nargs="+", type=pathlib.Path)

    args = parser.parse_args()
    symbols = [
        (shared_library, generate_symbols(shared_library))
        for shared_library in args.shared_libraries
    ]

    for lib1, lib2 in itertools.combinations(symbols, 2):
        print(f"Common symbols between {lib1[0]} and {lib2[0]}")
        common_symbols = lib1[1].intersection(lib2[1])
        for common in common_symbols:
            print(f"\t{common}")
        print()
