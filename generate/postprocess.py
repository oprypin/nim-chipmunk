import sys
import re
import collections


def insert_after(after, insert, in_string):
    i = in_string.index(after)+len(after)
    return in_string[:i]+insert+in_string[i:]

hierarchy_s = '''
SpatialIndex
    SpaceHash
    BBTree
    Sweep1D
Shape
    CircleShape
    SegmentShape
    PolyShape BoxShape
Constraint
    PinJoint
    SlideJoint
    PivotJoint
    GrooveJoint
    DampedSpring
    DampedRotarySpring
    RotaryLimitJoint
    RatchetJoint
    GearJoint
    SimpleMotor
'''
assoc = {}
hierarchy = {}
inheritable = set()
ptrobjs = set()
for line in hierarchy_s.splitlines():
    if not line.startswith('    '):
        base = line.strip()
        inheritable.add(base)
    else:
        sub, *assocs = line.split()
        hierarchy[sub] = base
        for a in assocs:
            assoc[a] = sub

src = sys.stdin.read()

# Trim whitespace from line ends
src = re.sub(r' *\n', '\n', src)

# Fix comments
src = src.replace('#/', '##')



# Find all groups/prefixes
#ngroups = re.findall(r'^## GROUP (.+)', src, flags=re.MULTILINE)
ngroups = set('Vect Mat2x2 BB SpatialIndex Arbiter Body Shape CircleShape SegmentShape PolyShape Constraint PinJoint SlideJoint PivotJoint GrooveJoint DampedSpring DampedRotarySpring RotaryLimitJoint RatchetJoint GearJoint SimpleMotor Space ShapeFilter Transform SpaceHash BBTree Sweep1D BoxShape'.split())

# Remove dynlib pragma
src = re.sub(r',?\s*dynlib: lib', '', src)

# Annotate proc name
src = re.sub(r'^(proc )(\w+)(\*.+inline.+=)$', r'\1\2\3\n  #"\2"', src, flags=re.MULTILINE)

# Remove private stuff
#src = re.sub(r'## @private\s+.+?\n\n', r'\n\n', src, flags=re.DOTALL)

# Strip newlines from procedure definitions
def repl(m):
    groups = [re.sub(r'\s+', ' ', gr).strip() for gr in m.groups()]
    return '{} {{.{}.}}'.format(*groups)
src = re.sub(r'(proc.+?){\.(.*?).\}', repl, src, flags=re.MULTILINE|re.DOTALL)

# Replace array-style initialization with object initialization
def repl(m):
    [typ] = re.findall(r'{}\* = object(?:\s+\w+\*: \w+)*'.format(m.group(1)), src)
    names = re.findall('(\w+)\*', typ)[1:]
    values = [v.strip() for v in m.group(2).split(',')]
    return ' = {}({})\n'.format(m.group(1), ', '.join('{}: {}'.format(*nv) for nv in zip(names, values)))
src = re.sub(r':\s*(cp\w+)\s*=\s*\[(.+?)\]\n', repl, src, flags=re.DOTALL)

# Remove some type definitions
src = re.sub(r'type\s+(cpBool)\* = .+', r'', src)

# Replace some types
src = re.sub(r'\buintptr_t\b', r'pointer', src)
src = re.sub(r'\b(u?int[0-9]+)_t\b', r'\1', src)
src = re.sub(r'\bcpBool\b', r'bool', src)

# Replace math
src = re.sub(r'\bINFINITY\b', r'Inf', src)
src = re.sub(r'\ba(sin|cos|tan|tan2)\b', r'arc\1', src)

#src = re.sub(r'proc ([a-z]+)(\w+)\*\(\1: ptr (cp\1)', r'proc \2*(\1: var \3', src, flags=re.IGNORECASE)

# Modify procedures
for proc in re.findall(r'proc (cp\w+)', src):
    name = proc
    kind = 'proc'
    # Remove alternative names
    if name.endswith('Raw'): name = name[:-3]
    elif name.endswith('2'): name = name[:-1]
    # Remove class prefixes
    for prefix in sorted(ngroups, key=len, reverse=True):
        if name.startswith('cp'+prefix):
            name = name[len('cp'+prefix):]
            kind = 'method'
            break
    else:
        name = name[2:]
        prefix = None
    name = name[0].lower()+name[1:]
    if any(name.startswith(s) for s in ['new', 'init', 'alloc']):
        if name.startswith('alloc'): name = 'allocate'+name[5:]
        elif name.startswith('init'): name = 'initialize'+name[4:]
        name += prefix
        kind = 'proc'
    elif name == 'destroy':
        name = 'finalize'
    elif name == 'free':
        name = 'destroy'
    # Handle getters and setters
    if name.startswith('get') or name.startswith('set'):
        setter = name.startswith('set')
        name = name[3].lower()+name[4:]
        if name == 'type':
            name = prefix[0].lower()+prefix[1:]+'Type'
        if setter:
            name = '`{}=`'.format(name)
    
    def repl(m):
        args, ret, rest = m.groups()
        kin = kind
        args = [a.split(': ') for a in args.split('; ') if a]
        if not (args and args[0][1].startswith('ptr ')):
            kin = 'proc'
        if name == 'destroy':
            rest = insert_after('{.', 'destroy, ', rest)
        if kin == 'method':
            args[0][1] = 'ptr cp'+prefix
        elif name.startswith('new'):
            p = 'ptr cp' if ret.startswith('ptr ') else 'cp'
            if prefix and prefix in hierarchy:
                ret = p+prefix
            elif prefix in assoc:
                ret = p+assoc[prefix]
        argss = ['; '.join('{}: {}'.format(*a) for a in args)]
        if name == 'destroy':
            t1 = args[0][1]
            assert t1.startswith('ptr cp')
            t1 = t1[6:]
            for k, v in hierarchy.items():
                if v == t1:
                    args[0][1] = 'ptr cp'+k
                    argss.append('; '.join('{}: {}'.format(*a) for a in args))
        
        return '\n'.join(
            '{} {}*({}){}{}'.format('proc', name, arg, ': '+ret if ret else '', rest)
            for arg in argss
        )
    src = re.sub(r'proc {}\*\((.*?)\)(?:: (.+?))?( \{{\..+)'.format(proc), repl, src)
    src = re.sub(r'(?<!")\b{}\b'.format(proc), name, src)

# Remove cp prefix
def repl(m):
    s = m.group(1)
    return s[0].lower()+s[1:]
src = re.sub(r'(?<!")\bcp(\w+)\b', r'\1', src)
src = re.sub(r'(?<!")\bCP_(\w+)\b', r'\1', src)

# Remove erroneous importc for variables
src = re.sub(r'\{\.importc: ".+?"\.\} =', r'=', src)

# Move type definitions to a separate location
atypes = []
def repl(m):
    atypes.append(m.group(1)+m.group(2))
    return '\n\n'
src = re.sub(r'^((?:##.+\n)*)type(?: *##.+?)?((?:\n^(?:  .+|)?$)+)\n\n', repl, src, flags=re.MULTILINE)
atypes = '\n\n'.join(atypes)+'''\n

'''
types = collections.OrderedDict()
for typ in re.findall(r'^  \w+\* .*= .+(?:\n    .+)*', atypes, flags=re.MULTILINE):
    typ = typ.rstrip()
    name = typ.partition('*')[0].strip()
    if typ.endswith('object'):
        ptrobjs.add(name)
        typ = insert_after(' = ', 'ptr ', typ)
    else:
        ptrobjs.discard(name)
    try:
        typ = insert_after('object', ' of {}'.format(hierarchy[name]), typ)
    except KeyError: pass
    if name in inheritable:
        typ = '  {0}Obj {{.inheritable.}} = object{1}\n  {0}* = ptr {0}Obj'.format(name, ''.join(typ.partition('\n')[1:3]))
    types.pop(name, None)
    types[name] = typ
types = '\n\n'.join(types.values())


# Indent comments
#src = re.sub(r'^(proc.+)\n(##.+)', r'\1\n  \2', src, flags=re.MULTILINE)




# Remove extra newlines
src = re.sub(r'\n\n+', r'\n\n', src)

src = 'type\n'+types+'\n\n\n'+src

# Remove "ptr" from ptr types
for typ in ptrobjs | set(hierarchy.keys()) | set(hierarchy.values()):
    src = re.sub(r'\bptr {}\b'.format(typ), typ, src)


print('''
{.deadCodeElim: on.}
{.hint[SmallLshouldNotBeUsed]: off.}

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

'''+src+'''

{.pop.}


proc `*`*(v: Vect; s: Float): Vect = vmult(v, s)
proc `+`*(v1, v2: Vect): Vect = vadd(v1, v2)
proc `-`*(v1, v2: Vect): Vect = vsub(v1, v2)
proc `==`*(v1, v2: Vect): bool = veql(v1, v2)
proc `-`*(v: Vect): Vect = vneg(v)
''')