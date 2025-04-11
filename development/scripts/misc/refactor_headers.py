#! /usr/bin/env python3
# Refactor headers according to
# https://github.com/stack-of-tasks/pinocchio/pull/2572
# Run test with python -m doctest -v refactor_headers.py

import re
import argparse
import subprocess

from pathlib import Path

GUARD_PATTERN = re.compile("^#ifndef __(.*)__$", re.MULTILINE)
INCLUDE_PATTERN = re.compile(r"^#include ([\"<].*[\">])\n", re.MULTILINE)

TEST_CPP_CONTENT_GOOD_GUARD = """//
// Copyright (c) 2015-2024 CNRS INRIA
// Copyright (c) 2016 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//

#ifndef __pinocchio_python_spatial_se3_hpp__
#define __pinocchio_python_spatial_se3_hpp__

#include <eigenpy/eigenpy.hpp>
#include <boost/python/tuple.hpp>

#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/spatial/explog.hpp"
// ...
#endif // ifndef __pinocchio_python_spatial_se3_hpp__"""

TEST_CPP_CONTENT_BAD_GUARD = """//
// Copyright (c) 2015-2024 CNRS INRIA
// Copyright (c) 2016 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//

#ifndef _pinocchio_python_spatial_se3_hpp__
#define _pinocchio_python_spatial_se3_hpp__
// ...
#endif // ifndef _pinocchio_python_spatial_se3i_hpp__"""

HPP_MODULE_INST_GUARD = """
/// Explicit template instantiation
#if PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION
    #include "{module_inst}"
#endif // PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION
"""

HPP_MODULE = """//
// Copyright (c) 2025 INRIA
//

#ifndef {guard}
#define {guard}

// Module dependencies
{dep_includes}

// Module headers
{module_includes}

{module_inst_include}

#endif // ifndef {guard}
"""

LSP_GUARD = """#ifdef PINOCCHIO_LSP
#include "{module_hpp}"
#endif // PINOCCHIO_LSP
"""


def find_guard(content: str) -> None | str:
    """Find guard in a C++ header file.
    :ivar content: File content to parse.
    :return: Guard name if found (without __), None if no guard is found.
    >>> find_guard(TEST_CPP_CONTENT_GOOD_GUARD)
    'pinocchio_python_spatial_se3_hpp'
    >>> find_guard(TEST_CPP_CONTENT_BAD_GUARD)
    """
    match = GUARD_PATTERN.search(content)
    if match:
        return match.group(1)
    return None


def update_guard(content: str, old_guard_name, new_guard_name: str) -> None | str:
    """Replace guards in a C++ header file.
    :ivar content: File content to parse.
    :ivar old_guard_name: Guard to replace.
    :ivar new_guard_name: New guard name.
    :return: New content if the 3 guards are changed, None otherwise.
    >>> res = update_guard(TEST_CPP_CONTENT_GOOD_GUARD, "pinocchio_python_spatial_se3_hpp", "pinocchio_python_spatial_se3_def_hpp")
    >>> print(res)
    //
    // Copyright (c) 2015-2024 CNRS INRIA
    // Copyright (c) 2016 Wandercraft, 86 rue de Paris 91400 Orsay, France.
    //
    <BLANKLINE>
    #ifndef __pinocchio_python_spatial_se3_def_hpp__
    #define __pinocchio_python_spatial_se3_def_hpp__
    <BLANKLINE>
    #include <eigenpy/eigenpy.hpp>
    #include <boost/python/tuple.hpp>
    <BLANKLINE>
    #include "pinocchio/spatial/se3.hpp"
    #include "pinocchio/spatial/explog.hpp"
    // ...
    #endif // ifndef __pinocchio_python_spatial_se3_def_hpp__
    >>> update_guard(TEST_CPP_CONTENT_BAD_GUARD, "pinocchio_python_spatial_se3_hpp", "pinocchio_python_spatial_se3_def_hpp")
    """
    (new_content, nr_sub) = re.subn(f"{old_guard_name}", new_guard_name, content)
    if nr_sub == 3:
        return new_content
    return None


def remove_includes(content: str, module_header: str) -> (str, list[str]):
    """Remove includes, add LSP guard instead and return them.
    :ivar content: File content to parse.
    :ivar module_header: Module header path (without < or ")
    :return: New content and list of removed includes (path with < or ")
    >>> res = remove_includes(TEST_CPP_CONTENT_GOOD_GUARD, "pinocchio/bindings/python/spatial/se3.hpp")
    >>> print(res[0])
    //
    // Copyright (c) 2015-2024 CNRS INRIA
    // Copyright (c) 2016 Wandercraft, 86 rue de Paris 91400 Orsay, France.
    //
    <BLANKLINE>
    #ifndef __pinocchio_python_spatial_se3_hpp__
    #define __pinocchio_python_spatial_se3_hpp__
    <BLANKLINE>
    #ifdef PINOCCHIO_LSP
    #include "pinocchio/bindings/python/spatial/se3.hpp"
    #endif // PINOCCHIO_LSP
    <BLANKLINE>
    // ...
    #endif // ifndef __pinocchio_python_spatial_se3_hpp__
    >>> print(res[1])
    ['<eigenpy/eigenpy.hpp>', '<boost/python/tuple.hpp>', '"pinocchio/spatial/se3.hpp"', '"pinocchio/spatial/explog.hpp"']
    """
    includes = INCLUDE_PATTERN.findall(content)
    if len(includes) == 0:
        print("\tNo include, PINOCCHIO_LSP will not be added")
    tmp_pattern = "$$$$$ BLBLBBLLB #####"
    new_content = INCLUDE_PATTERN.sub(tmp_pattern, content)
    new_content2 = new_content.replace(
        tmp_pattern, LSP_GUARD.format(module_hpp=module_header), 1
    )
    new_content3 = new_content2.replace(tmp_pattern, "")
    return new_content3, includes


def create_hpp_module(
    guard: str,
    dependencies_includes: list[str],
    module_includes: list[str],
    module_inst_include: str | None,
) -> str:
    """Create a module content.
    :ivar guard: Guard name.
    :ivar dependencies_includes: Module dependencies include paths (path with < or ").
    :ivar module_includes: Module internal include paths (path without < or ").
    :ivar module_inst: Module explicit template instantiation (path without < or ").
    :return: Module content.
    >>> res = create_hpp_module("__pinocchio_python_spatial_se3_hpp__",\
        ['<eigenpy/eigenpy.hpp>', '<boost/python/tuple.hpp>', '"pinocchio/spatial/se3.hpp"', '"pinocchio/spatial/explog.hpp"'],\
        ['pinocchio/bindings/python/spatial/se3_decl.hxx', 'pinocchio/bindings/python/spatial/se3_def.hxx'],\
        None)
    >>> print(res)
    //
    // Copyright (c) 2025 INRIA
    //
    <BLANKLINE>
    #ifndef __pinocchio_python_spatial_se3_hpp__
    #define __pinocchio_python_spatial_se3_hpp__
    <BLANKLINE>
    // Module dependencies
    #include <eigenpy/eigenpy.hpp>
    #include <boost/python/tuple.hpp>
    #include "pinocchio/spatial/se3.hpp"
    #include "pinocchio/spatial/explog.hpp"
    <BLANKLINE>
    // Module headers
    #include "pinocchio/bindings/python/spatial/se3_decl.hxx"
    #include "pinocchio/bindings/python/spatial/se3_def.hxx"
    <BLANKLINE>
    <BLANKLINE>
    <BLANKLINE>
    #endif // ifndef __pinocchio_python_spatial_se3_hpp__
    <BLANKLINE>
    >>> res = create_hpp_module("__pinocchio_python_spatial_se3_hpp__",\
        ['<eigenpy/eigenpy.hpp>', '<boost/python/tuple.hpp>', '"pinocchio/spatial/se3.hpp"', '"pinocchio/spatial/explog.hpp"'],\
        ['pinocchio/bindings/python/spatial/se3_decl.hxx', 'pinocchio/bindings/python/spatial/se3_def.hxx'],\
        "pinocchio/bindings/python/spatial/se3_inst.hxx")
    >>> print(res)
    //
    // Copyright (c) 2025 INRIA
    //
    <BLANKLINE>
    #ifndef __pinocchio_python_spatial_se3_hpp__
    #define __pinocchio_python_spatial_se3_hpp__
    <BLANKLINE>
    // Module dependencies
    #include <eigenpy/eigenpy.hpp>
    #include <boost/python/tuple.hpp>
    #include "pinocchio/spatial/se3.hpp"
    #include "pinocchio/spatial/explog.hpp"
    <BLANKLINE>
    // Module headers
    #include "pinocchio/bindings/python/spatial/se3_decl.hxx"
    #include "pinocchio/bindings/python/spatial/se3_def.hxx"
    <BLANKLINE>
    <BLANKLINE>
    /// Explicit template instantiation
    #if PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION
        #include "pinocchio/bindings/python/spatial/se3_inst.hxx"
    #endif // PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION
    <BLANKLINE>
    <BLANKLINE>
    #endif // ifndef __pinocchio_python_spatial_se3_hpp__
    <BLANKLINE>
    """
    deps = "\n".join([f"#include {d}" for d in dependencies_includes])
    modules = "\n".join([f'#include "{d}"' for d in module_includes])
    module_inst = ""
    if module_inst_include is not None:
        module_inst = HPP_MODULE_INST_GUARD.format(module_inst=module_inst_include)
    return HPP_MODULE.format(
        guard=guard,
        dep_includes=deps,
        module_includes=modules,
        module_inst_include=module_inst,
    )


def guard_from_path(path: Path) -> str:
    """Compute the guard from the relative path
    >>> guard_from_path(Path("pinocchio/algorithm/kinematics.hxx"))
    'pinocchio_algorithm_kinematics_hxx'
    """
    guard_content = "_".join(
        map(lambda x: x.replace(".", "_").replace("-", "_"), path.parts)
    )
    return f"{guard_content}"


def update_content(
    path: Path,
    module_path: Path,
    new_guard: str,
    dry: bool = False,
) -> list[str]:
    """Update path content with the following rule:
    :ivar path: File to update.
    :ivar module_path: Module path relative to root directory.
    :ivar new_guard: New guard.
    :ivar dry: Don modify the file.
    - Update guards
    - Remove includes
    - Add clangd_hack
    """
    content = ""
    with path.open() as path_desc:
        content = path_desc.read()
    old_guard = find_guard(content)
    if old_guard is None:
        raise RuntimeError(f"{path} doesn't contains valid guards")
    print(f"\tNew guard: {new_guard}")
    content = update_guard(content, old_guard, new_guard)
    if content is None:
        raise RuntimeError(f"{path} doesn't have three guards")
    content, includes = remove_includes(content, str(module_path))
    if not dry:
        with path.open("w") as path:
            path.write(content)
    return includes, old_guard


def update_module(module_path: Path, include_root_path: Path, dry: bool = False):
    """Apply new convention to a module
    :ivar module_path: Path to the hpp (module entry point) to refactor. Must be an absolute path.
    :ivar include_root_path: Path to the include directory. Me be an absolute path.
    """
    assert module_path.is_absolute()
    assert include_root_path.is_absolute()

    print(f"Update {module_path}")

    module_path_rel = module_path.relative_to(include_root_path)
    module_old_def = module_path.parent / Path(f"{module_path.stem}.hxx")
    module_old_inst = module_path.parent / Path(f"{module_path.stem}.txx")
    module_decl = module_path.parent / Path(f"{module_path.stem}.hxx")
    module_def = module_path.parent / Path(f"{module_path.stem}-def.hxx")
    module_inst = module_path.parent / Path(f"{module_path.stem}-inst.hxx")

    dependency_includes = []
    module_includes = []

    # We must process module_old_def first because his old name
    # clash with module_decl new name.
    if module_old_def.exists():
        print(f"\tConvert {module_old_def} into {module_def}")
        def_includes, _ = update_content(
            module_old_def,
            module_path_rel,
            guard_from_path(module_def.relative_to(include_root_path)),
            dry,
        )
        dependency_includes += def_includes
        if not dry:
            subprocess.run(
                [
                    "git",
                    "-C",
                    str(include_root_path),
                    "mv",
                    str(module_old_def),
                    str(module_def),
                ]
            )
        module_includes.append(str(module_def.relative_to(include_root_path)))

    print(f"\tConvert {module_path} into {module_decl}")
    # Declaration/Definition should be before definition
    module_includes.insert(0, str(module_decl.relative_to(include_root_path)))
    module_inst_include = None

    decl_includes, module_guard = update_content(
        module_path,
        module_path_rel,
        guard_from_path(module_decl.relative_to(include_root_path)),
        dry,
    )
    dependency_includes += decl_includes
    if not dry:
        subprocess.run(
            [
                "git",
                "-C",
                str(include_root_path),
                "mv",
                str(module_path),
                str(module_decl),
            ]
        )

    if module_old_inst.exists():
        print(f"\tConvert {module_old_inst} into {module_inst}")
        inst_includes, _ = update_content(
            module_old_inst,
            module_path_rel,
            guard_from_path(module_inst.relative_to(include_root_path)),
            dry,
        )
        dependency_includes += inst_includes
        if not dry:
            subprocess.run(
                [
                    "git",
                    "-C",
                    str(include_root_path),
                    "mv",
                    str(module_old_inst),
                    str(module_inst),
                ]
            )
        module_inst_include = str(module_inst.relative_to(include_root_path))

    print(f"\tCreate new module {module_path}")
    print(f"\tNew module guard: {module_guard}")
    print(f"\tNew module dependencies: {', '.join(dependency_includes)}")
    print(f"\tNew module includes: {', '.join(module_includes)}")
    print(f"\tNew module inst: {module_inst_include}")
    module_content = create_hpp_module(
        module_guard, dependency_includes, module_includes, module_inst_include
    )
    if not dry:
        with module_path.open("w") as module_path_desc:
            module_path_desc.write(module_content)
    print()


def is_dir(dir: str) -> Path:
    p = Path(dir)
    if not p.is_dir():
        raise argparse.ArgumentTypeError(f"{dir} is not a directory")
    return p


def is_file(file: str) -> Path:
    p = Path(file)
    if not p.is_file():
        raise argparse.ArgumentTypeError(f"{file} is not a file")
    return p


def argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Refactor headers in a Pinocchio subdirectory"
    )
    parser.add_argument("include_directory", help="Include directory", type=is_dir)
    parser.add_argument("--dry", help="Dry run", action="store_true")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument(
        "-m", "--module", help="Module to convert", type=is_file, default=None
    )
    group.add_argument(
        "-d", "--dir", help="Directory to convert", type=is_dir, default=None
    )
    return parser


def main(args: list[str]):
    """
    Walk in a directory
    Find all xxx.hpp
    Rename it into xxx.hxx
     Update guards
     Remove includes
     Add clangd_hack
    if xxx.hxx associated
      Rename into xxx-def.hxx
        Update guards
        Remove includes
        Add clangd_hack
    if xxx.txx associated
      Rename into xxx-inst.hxx
       Update guards
       Remove includes
        Add clangd_hack
    Create xxx.hpp
      Add guards
      Add includes form decl, def and inst
      include decl, def and inst
    """
    parser = argument_parser()
    args = parser.parse_args(args)

    if args.module is not None:
        update_module(
            args.module.absolute(), args.include_directory.absolute(), args.dry
        )
    elif args.dir is not None:
        for m in args.dir.glob("*.hpp"):
            update_module(m.absolute(), args.include_directory.absolute(), args.dry)
    else:
        raise RuntimeError("module or dir argument must be provided")


if __name__ == "__main__":
    import sys

    main(sys.argv[1:])
