# Coding convention

## Headers

Pinocchio has 4 kinds of headers:
- Public headers: `xxx.hpp`
- Private headers:
   - Declaration or declaration and definition when not spited: `xxx.hxx`
   - Definition: `xxx-def.hxx`
   - Explicit template instantiation: `xxx-inst.hxx`

Since Pinocchio is mostly a header only library, include cycles are easy to create.

As a trivial example:

`a.hpp`:
```
#pragma once
#include "b.hpp"

struct A
{
   B* b;
};
```

`b.hpp`:
```
#pragma once
#include "a.hpp"

struct B
{
   A* a;
};
```

If we build a file that include `a.hpp` we will have the following error:
```bash
b.hpp:5:3: error: ‘A’ does not name a type
```
Since `a.hpp` is already included, the guard prevent to include it again
in `b.hpp` and `A` is never defined.
This kind of error can be really tricky to debug,
especially in header only libraries that can have big include chain.

To prevent this we apply the following rule:
- Private header can't include anything
- Public header can include private header

Public headers will have the duty to include anything needed by private ones it exposes.
This will prevent cycles since transitive includes are now forbidden.

This approach have two drawbacks:
- Transitive include allow to save some code, we will have a lot of redundant code in public headers
- In private headers, language server will know no symbols (since the dependencies are not included)

To overcome the second issue, we can create the following code snippet to add after the guard of any private header:
```cpp
#ifdef PINOCCHIO_LSP
#include "xxx.hpp"
#endif // PINOCCHIO_LSP
```
Where `xxx.hpp` is the public header that expose this private header.

By setting the following `.clangd` file at the project root the LSP will be able to see the private header dependencies:
```yaml
If:
   PathMatch: .*\.hxx
CompileFlags:
   Add:
      - -DPINOCCHIO_LSP
```

### Guard

Pinocchio doesn't use `pragma once` as a guard. Instead, we use the file path converted into snake_case.

As an example `include/pinocchio/spatial/se3.hpp` will have the following guard:
```cpp
#ifndef __pinocchio_spatial_se3_hpp__
#define __pinocchio_spatial_se3_hpp__

// ...

#endif // ifndef __pinocchio_spatial_se3_hpp__
```
