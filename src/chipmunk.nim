# Copyright (C) 2015 Oleh Prypin <blaxpirit@gmail.com>
# Licensed under terms of MIT license (see LICENSE)

{.deadCodeElim: on.}
{.warning[SmallLshouldNotBeUsed]: off.}

when defined chipmunkNoDestructors:
  {.pragma: destroy.}
  {.hint: "Chipmunk: no destructors; call destroy() manually, beware of memory leaks!".}
else:
  {.experimental.}
  {.pragma: destroy, override.}

{.passL: "-lpthread".}
when defined(windows):
  const lib = "chipmunk.dll"
elif defined(mac):
  const lib = "libchipmunk.dylib"
else:
  const lib = "libchipmunk.so"


import math

proc `div`(x, y: cdouble): cdouble = x / y

converter toBool[T](x: ptr T): bool = x != nil


{.push dynlib: lib.}

include private/chipmunk_gen

{.pop.}


proc `*`*(v: Vect; s: Float): Vect = vmult(v, s)
proc `+`*(v1, v2: Vect): Vect = vadd(v1, v2)
proc `-`*(v1, v2: Vect): Vect = vsub(v1, v2)
proc `==`*(v1, v2: Vect): bool = veql(v1, v2)
proc `-`*(v: Vect): Vect = vneg(v)
