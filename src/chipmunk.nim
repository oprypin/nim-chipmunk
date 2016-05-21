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


type
  Float* = cdouble

  HashValue* = pointer

  CollisionID* = uint32

  DataPointer* = pointer

  CollisionType* = pointer

  Group* = pointer

  Bitmask* = cuint

  Timestamp* = cuint

  Vect* = object
    x*: Float
    y*: Float

  Transform* = object
    a*: Float
    b*: Float
    c*: Float
    d*: Float
    tx*: Float
    ty*: Float

  Mat2x2* = object
    a*: Float
    b*: Float
    c*: Float
    d*: Float

  Array* = ptr object

  HashSet* = ptr object

  Body* = ptr object

  ShapeObj {.inheritable.} = object
  Shape* = ptr ShapeObj

  CircleShape* = ptr object of Shape

  SegmentShape* = ptr object of Shape

  PolyShape* = ptr object of Shape

  ConstraintObj {.inheritable.} = object
  Constraint* = ptr ConstraintObj

  PinJoint* = ptr object of Constraint

  SlideJoint* = ptr object of Constraint

  PivotJoint* = ptr object of Constraint

  GrooveJoint* = ptr object of Constraint

  DampedSpring* = ptr object of Constraint

  DampedRotarySpring* = ptr object of Constraint

  RotaryLimitJoint* = ptr object of Constraint

  RatchetJoint* = ptr object of Constraint

  GearJoint* = ptr object of Constraint

  SimpleMotorJoint* = ptr object

  Arbiter* = ptr object

  Space* = ptr object

  BB* = object
    l*: Float
    b*: Float
    r*: Float
    t*: Float

  SpatialIndexBBFunc* = proc (obj: pointer): BB {.cdecl.}

  SpatialIndexIteratorFunc* = proc (obj: pointer; data: pointer) {.cdecl.}

  SpatialIndexQueryFunc* = proc (obj1: pointer; obj2: pointer; id: CollisionID; data: pointer): CollisionID {.cdecl.}

  SpatialIndexSegmentQueryFunc* = proc (obj1: pointer; obj2: pointer; data: pointer): Float {.cdecl.}

  SpatialIndexObj {.inheritable.} = object
    klass*: ptr SpatialIndexClass
    bbfunc*: SpatialIndexBBFunc
    staticIndex*: SpatialIndex
    dynamicIndex*: SpatialIndex
  SpatialIndex* = ptr SpatialIndexObj

  SpaceHash* = ptr object of SpatialIndex

  BBTree* = ptr object of SpatialIndex

  BBTreeVelocityFunc* = proc (obj: pointer): Vect {.cdecl.}

  Sweep1D* = ptr object of SpatialIndex

  SpatialIndexDestroyImpl* = proc (index: SpatialIndex) {.cdecl.}

  SpatialIndexCountImpl* = proc (index: SpatialIndex): cint {.cdecl.}

  SpatialIndexEachImpl* = proc (index: SpatialIndex; `func`: SpatialIndexIteratorFunc; data: pointer) {.cdecl.}

  SpatialIndexContainsImpl* = proc (index: SpatialIndex; obj: pointer; hashid: HashValue): bool {.cdecl.}

  SpatialIndexInsertImpl* = proc (index: SpatialIndex; obj: pointer; hashid: HashValue) {.cdecl.}

  SpatialIndexRemoveImpl* = proc (index: SpatialIndex; obj: pointer; hashid: HashValue) {.cdecl.}

  SpatialIndexReindexImpl* = proc (index: SpatialIndex) {.cdecl.}

  SpatialIndexReindexObjectImpl* = proc (index: SpatialIndex; obj: pointer; hashid: HashValue) {.cdecl.}

  SpatialIndexReindexQueryImpl* = proc (index: SpatialIndex; `func`: SpatialIndexQueryFunc; data: pointer) {.cdecl.}

  SpatialIndexQueryImpl* = proc (index: SpatialIndex; obj: pointer; bb: BB; `func`: SpatialIndexQueryFunc; data: pointer) {.cdecl.}

  SpatialIndexSegmentQueryImpl* = proc (index: SpatialIndex; obj: pointer; a: Vect; b: Vect; t_exit: Float; `func`: SpatialIndexSegmentQueryFunc; data: pointer) {.cdecl.}

  SpatialIndexClass* = object
    destroy*: SpatialIndexDestroyImpl
    count*: SpatialIndexCountImpl
    each*: SpatialIndexEachImpl
    contains*: SpatialIndexContainsImpl
    insert*: SpatialIndexInsertImpl
    remove*: SpatialIndexRemoveImpl
    reindex*: SpatialIndexReindexImpl
    reindexObject*: SpatialIndexReindexObjectImpl
    reindexQuery*: SpatialIndexReindexQueryImpl
    query*: SpatialIndexQueryImpl
    segmentQuery*: SpatialIndexSegmentQueryImpl

  INNER_C_STRUCT_285136460561076997* = object
    pointA*: Vect           ## The position of the contact on the surface of each shape.
    pointB*: Vect ## Penetration distance of the two shapes. Overlapping means it will be negative.
                    ## This value is calculated as vdot(vsub(point2, point1), normal) and is ignored by `contactPointSet=`().
    distance*: Float

  ContactPointSet* = object
    count*: cint              ## The number of contact points in the set.
    ## The normal of the collision.
    normal*: Vect           ## The array of contact points.
    points*: array[2, INNER_C_STRUCT_285136460561076997]

  BodyType* {.size: sizeof(cint).} = enum
    BODY_TYPE_DYNAMIC, ## A kinematic body is an infinite mass, user controlled body that is not affected by gravity, forces or collisions.
                          ## Instead the body only moves based on it's velocity.
                          ## Dynamic bodies collide normally with kinematic bodies, though the kinematic body will be unaffected.
                          ## Collisions between two kinematic bodies, or a kinematic body and a static body produce collision callbacks, but no collision response.
    BODY_TYPE_KINEMATIC, ## A static body is a body that never (or rarely) moves. If you move a static body, you must call one of the SpaceReindex*() functions.
                            ## Chipmunk uses this information to optimize the collision detection.
                            ## Static bodies do not produce collision callbacks when colliding with other static bodies.
    BODY_TYPE_STATIC

  BodyVelocityFunc* = proc (body: Body; gravity: Vect; damping: Float; dt: Float) {.cdecl.}

  BodyPositionFunc* = proc (body: Body; dt: Float) {.cdecl.}

  BodyShapeIteratorFunc* = proc (body: Body; shape: Shape; data: pointer) {.cdecl.}

  BodyConstraintIteratorFunc* = proc (body: Body; constraint: Constraint; data: pointer) {.cdecl.}

  BodyArbiterIteratorFunc* = proc (body: Body; arbiter: Arbiter; data: pointer) {.cdecl.}

  PointQueryInfo* = object
    shape*: Shape       ## The nearest shape, NULL if no shape was within range.
    ## The closest point on the shape's surface. (in world space coordinates)
    point*: Vect            ## The distance to the point. The distance is negative if the point is inside the shape.
    distance*: Float        ## The gradient of the signed distance function.
                              ## The value should be similar to info.p/info.d, but accurate even for very small values of info.d.
    gradient*: Vect

  SegmentQueryInfo* = object
    shape*: Shape       ## The shape that was hit, or NULL if no collision occured.
    ## The point of impact.
    point*: Vect            ## The normal of the surface hit.
    normal*: Vect           ## The normalized distance along the query segment in the range [0, 1].
    alpha*: Float

  ShapeFilter* = object
    group*: Group ## Two objects with the same non-zero group value do not collide.
                    ## This is generally used to group objects in a composite object together to disable self collisions.
    ## A bitmask of user definable categories that this object belongs to.
    ## The category/mask combinations of both objects in a collision must agree for a collision to occur.
    categories*: Bitmask ## A bitmask of user definable category types that this object object collides with.
                           ## The category/mask combinations of both objects in a collision must agree for a collision to occur.
    mask*: Bitmask

  ConstraintPreSolveFunc* = proc (constraint: Constraint; space: Space) {.cdecl.}

  ConstraintPostSolveFunc* = proc (constraint: Constraint; space: Space) {.cdecl.}

  DampedSpringForceFunc* = proc (spring: Constraint; dist: Float): Float {.cdecl.}

  DampedRotarySpringTorqueFunc* = proc (spring: Constraint; relativeAngle: Float): Float {.cdecl.}

  SimpleMotor* = ptr object of Constraint

  CollisionBeginFunc* = proc (arb: Arbiter; space: Space; userData: DataPointer): bool {.cdecl.}

  CollisionPreSolveFunc* = proc (arb: Arbiter; space: Space; userData: DataPointer): bool {.cdecl.}

  CollisionPostSolveFunc* = proc (arb: Arbiter; space: Space; userData: DataPointer) {.cdecl.}

  CollisionSeparateFunc* = proc (arb: Arbiter; space: Space; userData: DataPointer) {.cdecl.}

  CollisionHandler* = object
    typeA*: CollisionType ## Collision type identifier of the first shape that this handler recognizes.
                            ## In the collision handler callback, the shape with this type will be the first argument. Read only.
    ## Collision type identifier of the second shape that this handler recognizes.
    ## In the collision handler callback, the shape with this type will be the second argument. Read only.
    typeB*: CollisionType   ## This function is called when two shapes with types that match this collision handler begin colliding.
    beginFunc*: CollisionBeginFunc ## This function is called each step when two shapes with types that match this collision handler are colliding.
                                     ## It's called before the collision solver runs so that you can affect a collision's outcome.
    preSolveFunc*: CollisionPreSolveFunc ## This function is called each step when two shapes with types that match this collision handler are colliding.
                                           ## It's called after the collision solver runs so that you can read back information about the collision to trigger events in your game.
    postSolveFunc*: CollisionPostSolveFunc ## This function is called when two shapes with types that match this collision handler stop colliding.
    separateFunc*: CollisionSeparateFunc ## This is a user definable context pointer that is passed to all of the collision handler functions.
    userData*: DataPointer

  PostStepFunc* = proc (space: Space; key: pointer; data: pointer) {.cdecl.}

  SpacePointQueryFunc* = proc (shape: Shape; point: Vect; distance: Float; gradient: Vect; data: pointer) {.cdecl.}

  SpaceSegmentQueryFunc* = proc (shape: Shape; point: Vect; normal: Vect; alpha: Float; data: pointer) {.cdecl.}

  SpaceBBQueryFunc* = proc (shape: Shape; data: pointer) {.cdecl.}

  SpaceShapeQueryFunc* = proc (shape: Shape; points: ptr ContactPointSet; data: pointer) {.cdecl.}

  SpaceBodyIteratorFunc* = proc (body: Body; data: pointer) {.cdecl.}

  SpaceShapeIteratorFunc* = proc (shape: Shape; data: pointer) {.cdecl.}

  SpaceConstraintIteratorFunc* = proc (constraint: Constraint; data: pointer) {.cdecl.}

  SpaceDebugColor* = object
    r*: cfloat
    g*: cfloat
    b*: cfloat
    a*: cfloat

  SpaceDebugDrawCircleImpl* = proc (pos: Vect; angle: Float; radius: Float; outlineColor: SpaceDebugColor; fillColor: SpaceDebugColor; data: DataPointer) {.cdecl.}

  SpaceDebugDrawSegmentImpl* = proc (a: Vect; b: Vect; color: SpaceDebugColor; data: DataPointer) {.cdecl.}

  SpaceDebugDrawFatSegmentImpl* = proc (a: Vect; b: Vect; radius: Float; outlineColor: SpaceDebugColor; fillColor: SpaceDebugColor; data: DataPointer) {.cdecl.}

  SpaceDebugDrawPolygonImpl* = proc (count: cint; verts: ptr Vect; radius: Float; outlineColor: SpaceDebugColor; fillColor: SpaceDebugColor; data: DataPointer) {.cdecl.}

  SpaceDebugDrawDotImpl* = proc (size: Float; pos: Vect; color: SpaceDebugColor; data: DataPointer) {.cdecl.}

  SpaceDebugDrawColorForShapeImpl* = proc (shape: Shape; data: DataPointer): SpaceDebugColor {.cdecl.}

  SpaceDebugDrawFlags* {.size: sizeof(cint).} = enum
    SPACE_DEBUG_DRAW_SHAPES = 1 shl 0,
    SPACE_DEBUG_DRAW_CONSTRAINTS = 1 shl 1,
    SPACE_DEBUG_DRAW_COLLISION_POINTS = 1 shl 2

  SpaceDebugDrawOptions* = object
    drawCircle*: SpaceDebugDrawCircleImpl ## Function that will be invoked to draw circles.
    ## Function that will be invoked to draw line segments.
    drawSegment*: SpaceDebugDrawSegmentImpl ## Function that will be invoked to draw thick line segments.
    drawFatSegment*: SpaceDebugDrawFatSegmentImpl ## Function that will be invoked to draw convex polygons.
    drawPolygon*: SpaceDebugDrawPolygonImpl ## Function that will be invoked to draw dots.
    drawDot*: SpaceDebugDrawDotImpl ## Flags that request which things to draw (collision shapes, constraints, contact points).
    flags*: SpaceDebugDrawFlags ## Outline color passed to the drawing function.
    shapeOutlineColor*: SpaceDebugColor ## Function that decides what fill color to draw shapes using.
    colorForShape*: SpaceDebugDrawColorForShapeImpl ## Color passed to drawing functions for constraints.
    constraintColor*: SpaceDebugColor ## Color passed to drawing functions for collision points.
    collisionPointColor*: SpaceDebugColor ## User defined context pointer passed to all of the callback functions as the 'data' argument.
    data*: DataPointer


proc message*(condition: cstring; file: cstring; line: cint; isError: cint; isHardError: cint; message: cstring) {.varargs, cdecl, importc: "cpMessage".}
## Chipmunk's floating point type.
## Can be reconfigured at compile time.

## Return the max of two Floats.

proc fmax*(a: Float; b: Float): Float {.inline, cdecl.} =
  #"cpfmax"
  return if (a > b): a else: b

## Return the min of two Floats.

proc fmin*(a: Float; b: Float): Float {.inline, cdecl.} =
  #"cpfmin"
  return if (a < b): a else: b

## Return the absolute value of a Float.

proc fabs*(f: Float): Float {.inline, cdecl.} =
  #"cpfabs"
  return if (f < 0): - f else: f

## Clamp @c f to be between @c min and @c max.

proc fclamp*(f: Float; min: Float; max: Float): Float {.inline, cdecl.} =
  #"cpfclamp"
  return fmin(fmax(f, min), max)

## Clamp @c f to be between 0 and 1.

proc fclamp01*(f: Float): Float {.inline, cdecl.} =
  #"cpfclamp01"
  return fmax(0.0, fmin(f, 1.0))

## Linearly interpolate (or extrapolate) between @c f1 and @c f2 by @c t percent.

proc flerp*(f1: Float; f2: Float; t: Float): Float {.inline, cdecl.} =
  #"cpflerp"
  return f1 * (1.0 - t) + f2 * t

## Linearly interpolate from @c f1 to @c f2 by no more than @c d.

proc flerpconst*(f1: Float; f2: Float; d: Float): Float {.inline, cdecl.} =
  return f1 + fclamp(f2 - f1, - d, d)

## Hash value type.

## Type used internally to cache colliding object info for CollideShapes().
## Should be at least 32 bits.

## Chipmunk's boolean type.

## true value.
## false value.
## Type used for user data pointers.

## Type used for Space.collision_type.

## Type used for Shape.group.

## Type used for ShapeFilter category and mask.

## Type used for various timestamps in Chipmunk.

## Value for Shape.group signifying that a shape is in no group.
## Value for Shape.layers signifying that a shape is in every layer.
## CollisionType value internally reserved for hashing wildcard handlers.
## Chipmunk's 2D vector type.
## @addtogroup Vect

## Column major affine transform.

## Allocated size for various Chipmunk buffers
## Chipmunk calloc() alias.
## Chipmunk realloc() alias.
## Chipmunk free() alias.

## Constant for the zero vector.

var vzero* = Vect(x: 0.0, y: 0.0)

## Convenience constructor for Vect structs.

proc v*(x: Float; y: Float): Vect {.inline, cdecl.} =
  #"cpv"
  var v = Vect(x: x, y: y)
  return v

## Check if two vectors are equal. (Be careful when comparing floating point numbers!)

proc veql*(v1: Vect; v2: Vect): bool {.inline, cdecl.} =
  #"cpveql"
  return v1.x == v2.x and v1.y == v2.y

## Add two vectors

proc vadd*(v1: Vect; v2: Vect): Vect {.inline, cdecl.} =
  #"cpvadd"
  return v(v1.x + v2.x, v1.y + v2.y)

## Subtract two vectors.

proc vsub*(v1: Vect; v2: Vect): Vect {.inline, cdecl.} =
  #"cpvsub"
  return v(v1.x - v2.x, v1.y - v2.y)

## Negate a vector.

proc vneg*(v: Vect): Vect {.inline, cdecl.} =
  #"cpvneg"
  return v(- v.x, - v.y)

## Scalar multiplication.

proc vmult*(v: Vect; s: Float): Vect {.inline, cdecl.} =
  #"cpvmult"
  return v(v.x * s, v.y * s)

## Vector dot product.

proc vdot*(v1: Vect; v2: Vect): Float {.inline, cdecl.} =
  #"cpvdot"
  return v1.x * v2.x + v1.y * v2.y

## 2D vector cross product analog.
## The cross product of 2D vectors results in a 3D vector with only a z component.
## This function returns the magnitude of the z value.

proc vcross*(v1: Vect; v2: Vect): Float {.inline, cdecl.} =
  #"cpvcross"
  return v1.x * v2.y - v1.y * v2.x

## Returns a perpendicular vector. (90 degree rotation)

proc vperp*(v: Vect): Vect {.inline, cdecl.} =
  #"cpvperp"
  return v(- v.y, v.x)

## Returns a perpendicular vector. (-90 degree rotation)

proc vrperp*(v: Vect): Vect {.inline, cdecl.} =
  #"cpvrperp"
  return v(v.y, - v.x)

## Returns the vector projection of v1 onto v2.

proc vproject*(v1: Vect; v2: Vect): Vect {.inline, cdecl.} =
  #"cpvproject"
  return vmult(v2, vdot(v1, v2) div vdot(v2, v2))

## Returns the unit length vector for the given angle (in radians).

proc vforangle*(a: Float): Vect {.inline, cdecl.} =
  #"cpvforangle"
  return v(cos(a), sin(a))

## Returns the angular direction v is pointing in (in radians).

proc vtoangle*(v: Vect): Float {.inline, cdecl.} =
  #"cpvtoangle"
  return arctan2(v.y, v.x)

## Uses complex number multiplication to rotate v1 by v2. Scaling will occur if v1 is not a unit vector.

proc vrotate*(v1: Vect; v2: Vect): Vect {.inline, cdecl.} =
  #"cpvrotate"
  return v(v1.x * v2.x - v1.y * v2.y, v1.x * v2.y + v1.y * v2.x)

## Inverse of vrotate().

proc vunrotate*(v1: Vect; v2: Vect): Vect {.inline, cdecl.} =
  #"cpvunrotate"
  return v(v1.x * v2.x + v1.y * v2.y, v1.y * v2.x - v1.x * v2.y)

## Returns the squared length of v. Faster than vlength() when you only need to compare lengths.

proc vlengthsq*(v: Vect): Float {.inline, cdecl.} =
  #"cpvlengthsq"
  return vdot(v, v)

## Returns the length of v.

proc vlength*(v: Vect): Float {.inline, cdecl.} =
  #"cpvlength"
  return sqrt(vdot(v, v))

## Linearly interpolate between v1 and v2.

proc vlerp*(v1: Vect; v2: Vect; t: Float): Vect {.inline, cdecl.} =
  #"cpvlerp"
  return vadd(vmult(v1, 1.0 - t), vmult(v2, t))

## Returns a normalized copy of v.

proc vnormalize*(v: Vect): Vect {.inline, cdecl.} =
  #"cpvnormalize"
  return vmult(v, 1.0 div
      (vlength(v) + (cast[cdouble](2.225073858507201e-308))))

## Spherical linearly interpolate between v1 and v2.

proc vslerp*(v1: Vect; v2: Vect; t: Float): Vect {.inline, cdecl.} =
  #"cpvslerp"
  var dot: Float = vdot(vnormalize(v1), vnormalize(v2))
  var omega: Float = arccos(fclamp(dot, - 1.0, 1.0))
  if omega < 1 - 3:
    return vlerp(v1, v2, t)
  else:
    var denom: Float = 1.0 div sin(omega)
    return vadd(vmult(v1, sin((1.0 - t) * omega) * denom),
                  vmult(v2, sin(t * omega) * denom))

## Spherical linearly interpolate between v1 towards v2 by no more than angle a radians

proc vslerpconst*(v1: Vect; v2: Vect; a: Float): Vect {.inline, cdecl.} =
  #"cpvslerpconst"
  var dot: Float = vdot(vnormalize(v1), vnormalize(v2))
  var omega: Float = arccos(fclamp(dot, - 1.0, 1.0))
  return vslerp(v1, v2, fmin(a, omega) div omega)

## Clamp v to length len.

proc vclamp*(v: Vect; len: Float): Vect {.inline, cdecl.} =
  #"cpvclamp"
  return if (vdot(v, v) > len * len): vmult(vnormalize(v), len) else: v

## Linearly interpolate between v1 towards v2 by distance d.

proc vlerpconst*(v1: Vect; v2: Vect; d: Float): Vect {.inline, cdecl.} =
  #"cpvlerpconst"
  return vadd(v1, vclamp(vsub(v2, v1), d))

## Returns the distance between v1 and v2.

proc vdist*(v1: Vect; v2: Vect): Float {.inline, cdecl.} =
  #"cpvdist"
  return vlength(vsub(v1, v2))

## Returns the squared distance between v1 and v2. Faster than vdist() when you only need to compare distances.

proc vdistsq*(v1: Vect; v2: Vect): Float {.inline, cdecl.} =
  #"cpvdistsq"
  return vlengthsq(vsub(v1, v2))

## Returns true if the distance between v1 and v2 is less than dist.

proc vnear*(v1: Vect; v2: Vect; dist: Float): bool {.inline, cdecl.} =
  #"cpvnear"
  return vdistsq(v1, v2) < dist * dist

proc newMat2x2*(a: Float; b: Float; c: Float; d: Float): Mat2x2 {.inline, cdecl.} =
  var m = Mat2x2(a: a, b: b, c: c, d: d)
  return m

proc transform*(m: Mat2x2; v: Vect): Vect {.inline, cdecl.} =
  #"cpMat2x2Transform"
  return v(v.x * m.a + v.y * m.b, v.x * m.c + v.y * m.d)

## Chipmunk's axis-aligned 2D bounding box type. (left, bottom, right, top)

## Convenience constructor for BB structs.

proc newBB*(l: Float; b: Float; r: Float; t: Float): BB {.inline, cdecl.} =
  var bb = BB(l: l, b: b, r: r, t: t)
  return bb

## Constructs a BB centered on a point with the given extents (half sizes).

proc newForExtentsBB*(c: Vect; hw: Float; hh: Float): BB {.inline, cdecl.} =
  return newBB(c.x - hw, c.y - hh, c.x + hw, c.y + hh)

## Constructs a BB for a circle with the given position and radius.

proc newForCircleBB*(p: Vect; r: Float): BB {.inline, cdecl.} =
  #"cpBBNewForCircle"
  return newForExtentsBB(p, r, r)

## Returns true if @c a and @c b intersect.

proc intersects*(a: BB; b: BB): bool {.inline, cdecl.} =
  #"cpBBIntersects"
  return a.l <= b.r and b.l <= a.r and a.b <= b.t and b.b <= a.t

## Returns true if @c other lies completely within @c bb.

proc containsBB*(bb: BB; other: BB): bool {.inline, cdecl.} =
  #"cpBBContainsBB"
  return bb.l <= other.l and bb.r >= other.r and bb.b <= other.b and
      bb.t >= other.t

## Returns true if @c bb contains @c v.

proc containsVect*(bb: BB; v: Vect): bool {.inline, cdecl.} =
  #"cpBBContainsVect"
  return bb.l <= v.x and bb.r >= v.x and bb.b <= v.y and bb.t >= v.y

## Returns a bounding box that holds both bounding boxes.

proc merge*(a: BB; b: BB): BB {.inline, cdecl.} =
  #"cpBBMerge"
  return newBB(fmin(a.l, b.l), fmin(a.b, b.b), fmax(a.r, b.r),
                 fmax(a.t, b.t))

## Returns a bounding box that holds both @c bb and @c v.

proc expand*(bb: BB; v: Vect): BB {.inline, cdecl.} =
  #"cpBBExpand"
  return newBB(fmin(bb.l, v.x), fmin(bb.b, v.y), fmax(bb.r, v.x),
                 fmax(bb.t, v.y))

## Returns the center of a bounding box.

proc center*(bb: BB): Vect {.inline, cdecl.} =
  #"cpBBCenter"
  return vlerp(v(bb.l, bb.b), v(bb.r, bb.t), 0.5)

## Returns the area of the bounding box.

proc area*(bb: BB): Float {.inline, cdecl.} =
  #"cpBBArea"
  return (bb.r - bb.l) * (bb.t - bb.b)

## Merges @c a and @c b and returns the area of the merged bounding box.

proc mergedArea*(a: BB; b: BB): Float {.inline, cdecl.} =
  #"cpBBMergedArea"
  return (fmax(a.r, b.r) - fmin(a.l, b.l)) *
      (fmax(a.t, b.t) - fmin(a.b, b.b))

## Returns the fraction along the segment query the BB is hit. Returns Inf if it doesn't hit.

proc segmentQuery*(bb: BB; a: Vect; b: Vect): Float {.inline, cdecl.} =
  #"cpBBSegmentQuery"
  var idx: Float = 1.0 div (b.x - a.x)
  var tx1: Float = (if bb.l == a.x: - (Inf) else: (bb.l - a.x) * idx)
  var tx2: Float = (if bb.r == a.x: (Inf) else: (bb.r - a.x) * idx)
  var txmin: Float = fmin(tx1, tx2)
  var txmax: Float = fmax(tx1, tx2)
  var idy: Float = 1.0 div (b.y - a.y)
  var ty1: Float = (if bb.b == a.y: - (Inf) else: (bb.b - a.y) * idy)
  var ty2: Float = (if bb.t == a.y: (Inf) else: (bb.t - a.y) * idy)
  var tymin: Float = fmin(ty1, ty2)
  var tymax: Float = fmax(ty1, ty2)
  if tymin <= txmax and txmin <= tymax:
    var min: Float = fmax(txmin, tymin)
    var max: Float = fmin(txmax, tymax)
    if 0.0 <= max and min <= 1.0: return fmax(min, 0.0)
  return Inf

## Return true if the bounding box intersects the line segment with ends @c a and @c b.

proc intersectsSegment*(bb: BB; a: Vect; b: Vect): bool {.inline, cdecl.} =
  return segmentQuery(bb, a, b) != (Inf)

## Clamp a vector to a bounding box.

proc clampVect*(bb: BB; v: Vect): Vect {.inline, cdecl.} =
  #"cpBBClampVect"
  return v(fclamp(v.x, bb.l, bb.r), fclamp(v.y, bb.b, bb.t))

## Wrap a vector to a bounding box.

proc wrapVect*(bb: BB; v: Vect): Vect {.inline, cdecl.} =
  #"cpBBWrapVect"
  var dx: Float = fabs(bb.r - bb.l)
  var modx: Float = fmod(v.x - bb.l, dx)
  var x: Float = if (modx > 0.0): modx else: modx + dx
  var dy: Float = fabs(bb.t - bb.b)
  var mody: Float = fmod(v.y - bb.b, dy)
  var y: Float = if (mody > 0.0): mody else: mody + dy
  return v(x + bb.l, y + bb.b)

## Returns a bounding box offseted by @c v.

proc offset*(bb: BB; v: Vect): BB {.inline, cdecl.} =
  #"cpBBOffset"
  return newBB(bb.l + v.x, bb.b + v.y, bb.r + v.x, bb.t + v.y)

## Identity transform matrix.

var TransformIdentity* = Transform(a: 1.0, b: 0.0, c: 0.0, d: 1.0, tx: 0.0, ty: 0.0)

## Construct a new transform matrix.
## (a, b) is the x basis vector.
## (c, d) is the y basis vector.
## (tx, ty) is the translation.

proc newTransform*(a: Float; b: Float; c: Float; d: Float; tx: Float; ty: Float): Transform {.inline, cdecl.} =
  var t = Transform(a: a, b: b, c: c, d: d, tx: tx, ty: ty)
  return t

## Construct a new transform matrix in transposed order.

proc newTransposeTransform*(a: Float; c: Float; tx: Float; b: Float; d: Float; ty: Float): Transform {.inline, cdecl.} =
  var t = Transform(a: a, b: b, c: c, d: d, tx: tx, ty: ty)
  return t

## Get the inverse of a transform matrix.

proc inverse*(t: Transform): Transform {.inline, cdecl.} =
  #"cpTransformInverse"
  var inv_det: Float = 1.0 div (t.a * t.d - t.c * t.b)
  return newTransposeTransform(t.d * inv_det, - (t.c * inv_det),
                                 (t.c * t.ty - t.tx * t.d) * inv_det,
                                 - (t.b * inv_det), t.a * inv_det,
                                 (t.tx * t.b - t.a * t.ty) * inv_det)

## Multiply two transformation matrices.

proc mult*(t1: Transform; t2: Transform): Transform {.inline, cdecl.} =
  return newTransposeTransform(t1.a * t2.a + t1.c * t2.b,
                                 t1.a * t2.c + t1.c * t2.d,
                                 t1.a * t2.tx + t1.c * t2.ty + t1.tx,
                                 t1.b * t2.a + t1.d * t2.b,
                                 t1.b * t2.c + t1.d * t2.d,
                                 t1.b * t2.tx + t1.d * t2.ty + t1.ty)

## Transform an absolute point. (i.e. a vertex)

proc point*(t: Transform; p: Vect): Vect {.inline, cdecl.} =
  #"cpTransformPoint"
  return v(t.a * p.x + t.c * p.y + t.tx, t.b * p.x + t.d * p.y + t.ty)

## Transform a vector (i.e. a normal)

proc vect*(t: Transform; v: Vect): Vect {.inline, cdecl.} =
  #"cpTransformVect"
  return v(t.a * v.x + t.c * v.y, t.b * v.x + t.d * v.y)

## Transform a BB.

proc bBB*(t: Transform; bb: BB): BB {.inline, cdecl.} =
  #"cpTransformbBB"
  var center: Vect = center(bb)
  var hw: Float = (bb.r - bb.l) * 0.5
  var hh: Float = (bb.t - bb.b) * 0.5
  var
    a: Float = t.a * hw
    b: Float = t.c * hh
    d: Float = t.b * hw
    e: Float = t.d * hh
  var hw_max: Float = fmax(fabs(a + b), fabs(a - b))
  var hh_max: Float = fmax(fabs(d + e), fabs(d - e))
  return newForExtentsBB(point(t, center), hw_max, hh_max)

## Create a transation matrix.

proc translate*(translate: Vect): Transform {.inline, cdecl.} =
  #"cpTransformTranslate"
  return newTransposeTransform(1.0, 0.0, translate.x, 0.0, 1.0, translate.y)

## Create a scale matrix.

proc scale*(scaleX: Float; scaleY: Float): Transform {.inline, cdecl.} =
  return newTransposeTransform(scaleX, 0.0, 0.0, 0.0, scaleY, 0.0)

## Create a rotation matrix.

proc rotate*(radians: Float): Transform {.inline, cdecl.} =
  #"cpTransformRotate"
  var rot: Vect = vforangle(radians)
  return newTransposeTransform(rot.x, - rot.y, 0.0, rot.y, rot.x, 0.0)

## Create a rigid transformation matrix. (transation + rotation)

proc rigid*(translate: Vect; radians: Float): Transform {.inline, cdecl.} =
  var rot: Vect = vforangle(radians)
  return newTransposeTransform(rot.x, - rot.y, translate.x, rot.y, rot.x,
                                 translate.y)

## Fast inverse of a rigid transformation matrix.

proc rigidInverse*(t: Transform): Transform {.inline, cdecl.} =
  #"cpTransformRigidInverse"
  return newTransposeTransform(t.d, - t.c, (t.c * t.ty - t.tx * t.d), - t.b,
                                 t.a, (t.tx * t.b - t.a * t.ty))

proc wrap*(outer: Transform; inner: Transform): Transform {.inline, cdecl.} =
  return mult(inverse(outer),
                         mult(inner, outer))

proc wrapInverse*(outer: Transform; inner: Transform): Transform {.inline, cdecl.} =
  return mult(outer,
                         mult(inner, inverse(outer)))

proc ortho*(bb: BB): Transform {.inline, cdecl.} =
  #"cpTransformOrtho"
  return newTransposeTransform(2.0 div (bb.r - bb.l), 0.0,
                                 - ((bb.r + bb.l) div (bb.r - bb.l)), 0.0,
                                 2.0 div (bb.t - bb.b),
                                 - ((bb.t + bb.b) div (bb.t - bb.b)))

proc boneScale*(v0: Vect; v1: Vect): Transform {.inline, cdecl.} =
  #"cpTransformBoneScale"
  var d: Vect = vsub(v1, v0)
  return newTransposeTransform(d.x, - d.y, v0.x, d.y, d.x, v0.y)

proc axialScale*(axis: Vect; pivot: Vect; scale: Float): Transform {.inline, cdecl.} =
  var A: Float = axis.x * axis.y * (scale - 1.0)
  var B: Float = vdot(axis, pivot) * (1.0 - scale)
  return newTransposeTransform(scale * axis.x * axis.x + axis.y * axis.y, A,
                                 axis.x * B, A,
                                 axis.x * axis.x + scale * axis.y * axis.y,
                                 axis.y * B)

## Spatial index bounding box callback function type.
## The spatial index calls this function and passes you a pointer to an object you added
## when it needs to get the bounding box associated with that object.

## Spatial index/object iterator callback function type.

## Spatial query callback function type.

## Spatial segment query callback function type.

## @private

## Allocate a spatial hash.

proc allocateSpaceHash*(): SpaceHash {.cdecl, importc: "cpSpaceHashAlloc".}
## Initialize a spatial hash.

proc initializeSpaceHash*(hash: SpaceHash; celldim: Float; numcells: cint; bbfunc: SpatialIndexBBFunc; staticIndex: SpatialIndex): SpatialIndex {.cdecl, importc: "cpSpaceHashInit".}
## Allocate and initialize a spatial hash.

proc newSpaceHash*(celldim: Float; cells: cint; bbfunc: SpatialIndexBBFunc; staticIndex: SpatialIndex): SpaceHash {.cdecl, importc: "cpSpaceHashNew".}
## Change the cell dimensions and table size of the spatial hash to tune it.
## The cell dimensions should roughly match the average size of your objects
## and the table size should be ~10 larger than the number of objects inserted.
## Some trial and error is required to find the optimum numbers for efficiency.

proc resize*(hash: SpaceHash; celldim: Float; numcells: cint) {.cdecl, importc: "cpSpaceHashResize".}

## Allocate a bounding box tree.

proc allocateBBTree*(): BBTree {.cdecl, importc: "cpBBTreeAlloc".}
## Initialize a bounding box tree.

proc initializeBBTree*(tree: BBTree; bbfunc: SpatialIndexBBFunc; staticIndex: SpatialIndex): SpatialIndex {.cdecl, importc: "cpBBTreeInit".}
## Allocate and initialize a bounding box tree.

proc newBBTree*(bbfunc: SpatialIndexBBFunc; staticIndex: SpatialIndex): BBTree {.cdecl, importc: "cpBBTreeNew".}
## Perform a static top down optimization of the tree.

proc optimize*(index: BBTree) {.cdecl, importc: "cpBBTreeOptimize".}
## Bounding box tree velocity callback function.
## This function should return an estimate for the object's velocity.

## Set the velocity function for the bounding box tree to enable temporal coherence.

proc `velocityFunc=`*(index: BBTree; `func`: BBTreeVelocityFunc) {.cdecl, importc: "cpBBTreeSetVelocityFunc".}

## Allocate a 1D sort and sweep broadphase.

proc allocateSweep1D*(): Sweep1D {.cdecl, importc: "cpSweep1DAlloc".}
## Initialize a 1D sort and sweep broadphase.

proc initializeSweep1D*(sweep: Sweep1D; bbfunc: SpatialIndexBBFunc; staticIndex: SpatialIndex): SpatialIndex {.cdecl, importc: "cpSweep1DInit".}
## Allocate and initialize a 1D sort and sweep broadphase.

proc newSweep1D*(bbfunc: SpatialIndexBBFunc; staticIndex: SpatialIndex): Sweep1D {.cdecl, importc: "cpSweep1DNew".}

## Destroy and free a spatial index.

proc destroy*(index: SpatialIndex) {.destroy, cdecl, importc: "cpSpatialIndexFree".}
proc destroy*(index: Sweep1D) {.destroy, cdecl, importc: "cpSpatialIndexFree".}
proc destroy*(index: SpaceHash) {.destroy, cdecl, importc: "cpSpatialIndexFree".}
proc destroy*(index: BBTree) {.destroy, cdecl, importc: "cpSpatialIndexFree".}
## Collide the objects in @c dynamicIndex against the objects in @c staticIndex using the query callback function.

proc collideStatic*(dynamicIndex: SpatialIndex; staticIndex: SpatialIndex; `func`: SpatialIndexQueryFunc; data: pointer) {.cdecl, importc: "cpSpatialIndexCollideStatic".}
## Destroy a spatial index.

proc finalize*(index: SpatialIndex) {.inline, cdecl.} =
  #"cpSpatialIndexDestroy"
  if index.klass: index.klass.destroy(index)

## Get the number of objects in the spatial index.

proc count*(index: SpatialIndex): cint {.inline, cdecl.} =
  #"cpSpatialIndexCount"
  return index.klass.count(index)

## Iterate the objects in the spatial index. @c func will be called once for each object.

proc each*(index: SpatialIndex; `func`: SpatialIndexIteratorFunc; data: pointer) {.inline, cdecl.} =
  index.klass.each(index, `func`, data)

## Returns true if the spatial index contains the given object.
## Most spatial indexes use hashed storage, so you must provide a hash value too.

proc contains*(index: SpatialIndex; obj: pointer; hashid: HashValue): bool {.inline, cdecl.} =
  return index.klass.contains(index, obj, hashid)

## Add an object to a spatial index.
## Most spatial indexes use hashed storage, so you must provide a hash value too.

proc insert*(index: SpatialIndex; obj: pointer; hashid: HashValue) {.inline, cdecl.} =
  index.klass.insert(index, obj, hashid)

## Remove an object from a spatial index.
## Most spatial indexes use hashed storage, so you must provide a hash value too.

proc remove*(index: SpatialIndex; obj: pointer; hashid: HashValue) {.inline, cdecl.} =
  index.klass.remove(index, obj, hashid)

## Perform a full reindex of a spatial index.

proc reindex*(index: SpatialIndex) {.inline, cdecl.} =
  #"cpSpatialIndexReindex"
  index.klass.reindex(index)

## Reindex a single object in the spatial index.

proc reindexObject*(index: SpatialIndex; obj: pointer; hashid: HashValue) {.inline, cdecl.} =
  index.klass.reindexObject(index, obj, hashid)

## Perform a rectangle query against the spatial index, calling @c func for each potential match.

proc query*(index: SpatialIndex; obj: pointer; bb: BB; `func`: SpatialIndexQueryFunc; data: pointer) {.inline, cdecl.} =
  index.klass.query(index, obj, bb, `func`, data)

## Perform a segment query against the spatial index, calling @c func for each potential match.

proc segmentQuery*(index: SpatialIndex; obj: pointer; a: Vect; b: Vect; t_exit: Float; `func`: SpatialIndexSegmentQueryFunc; data: pointer) {.inline, cdecl.} =
  index.klass.segmentQuery(index, obj, a, b, t_exit, `func`, data)

## Simultaneously reindex and find all colliding objects.
## @c func will be called once for each potentially overlapping pair of objects found.
## If the spatial index was initialized with a static index, it will collide it's objects against that as well.

proc reindexQuery*(index: SpatialIndex; `func`: SpatialIndexQueryFunc; data: pointer) {.inline, cdecl.} =
  index.klass.reindexQuery(index, `func`, data)

## Get the restitution (elasticity) that will be applied to the pair of colliding objects.

proc restitution*(arb: Arbiter): Float {.cdecl, importc: "cpArbiterGetRestitution".}
## Override the restitution (elasticity) that will be applied to the pair of colliding objects.

proc `restitution=`*(arb: Arbiter; restitution: Float) {.cdecl, importc: "cpArbiterSetRestitution".}
## Get the friction coefficient that will be applied to the pair of colliding objects.

proc friction*(arb: Arbiter): Float {.cdecl, importc: "cpArbiterGetFriction".}
## Override the friction coefficient that will be applied to the pair of colliding objects.

proc `friction=`*(arb: Arbiter; friction: Float) {.cdecl, importc: "cpArbiterSetFriction".}
proc surfaceVelocity*(arb: Arbiter): Vect {.cdecl, importc: "cpArbiterGetSurfaceVelocity".}
proc `surfaceVelocity=`*(arb: Arbiter; vr: Vect) {.cdecl, importc: "cpArbiterSetSurfaceVelocity".}
## Get the user data pointer associated with this pair of colliding objects.

proc userData*(arb: Arbiter): DataPointer {.cdecl, importc: "cpArbiterGetUserData".}
## Set a user data point associated with this pair of colliding objects.
## If you need to perform any cleanup for this pointer, you must do it yourself, in the separate callback for instance.

proc `userData=`*(arb: Arbiter; userData: DataPointer) {.cdecl, importc: "cpArbiterSetUserData".}
## Calculate the total impulse including the friction that was applied by this arbiter.
## This function should only be called from a post-solve, post-step or eachArbiter callback.

proc totalImpulse*(arb: Arbiter): Vect {.cdecl, importc: "cpArbiterTotalImpulse".}
## Calculate the amount of energy lost in a collision including static, but not dynamic friction.
## This function should only be called from a post-solve, post-step or eachArbiter callback.

proc totalKE*(arb: Arbiter): Float {.cdecl, importc: "cpArbiterTotalKE".}
## Mark a collision pair to be ignored until the two objects separate.
## Pre-solve and post-solve callbacks will not be called, but the separate callback will be called.

proc ignore*(arb: Arbiter): bool {.cdecl, importc: "cpArbiterIgnore".}
## Return the colliding shapes involved for this arbiter.
## The order of their Space.collision_type values will match
## the order set when the collision handler was registered.

proc shapes*(arb: Arbiter; a: ptr Shape; b: ptr Shape) {.cdecl, importc: "cpArbiterGetShapes".}
## A macro shortcut for defining and retrieving the shapes from an arbiter.
## Return the colliding bodies involved for this arbiter.
## The order of the Space.collision_type the bodies are associated with values will match
## the order set when the collision handler was registered.

proc bodies*(arb: Arbiter; a: ptr Body; b: ptr Body) {.cdecl, importc: "cpArbiterGetBodies".}
## A macro shortcut for defining and retrieving the bodies from an arbiter.
## A struct that wraps up the important collision data for an arbiter.

## Return a contact set from an arbiter.

proc contactPointSet*(arb: Arbiter): ContactPointSet {.cdecl, importc: "cpArbiterGetContactPointSet".}
## Replace the contact point set for an arbiter.
## This can be a very powerful feature, but use it with caution!

proc `contactPointSet=`*(arb: Arbiter; set: ptr ContactPointSet) {.cdecl, importc: "cpArbiterSetContactPointSet".}
## Returns true if this is the first step a pair of objects started colliding.

proc isFirstContact*(arb: Arbiter): bool {.cdecl, importc: "cpArbiterIsFirstContact".}
## Returns true if the separate callback is due to a shape being removed from the space.

proc isRemoval*(arb: Arbiter): bool {.cdecl, importc: "cpArbiterIsRemoval".}
## Get the number of contact points for this arbiter.

proc count*(arb: Arbiter): cint {.cdecl, importc: "cpArbiterGetCount".}
## Get the normal of the collision.

proc normal*(arb: Arbiter): Vect {.cdecl, importc: "cpArbiterGetNormal".}
## Get the position of the @c ith contact point on the surface of the first shape.

proc pointA*(arb: Arbiter; i: cint): Vect {.cdecl, importc: "cpArbiterGetPointA".}
## Get the position of the @c ith contact point on the surface of the second shape.

proc pointB*(arb: Arbiter; i: cint): Vect {.cdecl, importc: "cpArbiterGetPointB".}
## Get the depth of the @c ith contact point.

proc depth*(arb: Arbiter; i: cint): Float {.cdecl, importc: "cpArbiterGetDepth".}
## If you want a custom callback to invoke the wildcard callback for the first collision type, you must call this function explicitly.
## You must decide how to handle the wildcard's return value since it may disagree with the other wildcard handler's return value or your own.

proc callWildcardBeginA*(arb: Arbiter; space: Space): bool {.cdecl, importc: "cpArbiterCallWildcardBeginA".}
## If you want a custom callback to invoke the wildcard callback for the second collision type, you must call this function explicitly.
## You must decide how to handle the wildcard's return value since it may disagree with the other wildcard handler's return value or your own.

proc callWildcardBeginB*(arb: Arbiter; space: Space): bool {.cdecl, importc: "cpArbiterCallWildcardBeginB".}
## If you want a custom callback to invoke the wildcard callback for the first collision type, you must call this function explicitly.
## You must decide how to handle the wildcard's return value since it may disagree with the other wildcard handler's return value or your own.

proc callWildcardPreSolveA*(arb: Arbiter; space: Space): bool {.cdecl, importc: "cpArbiterCallWildcardPreSolveA".}
## If you want a custom callback to invoke the wildcard callback for the second collision type, you must call this function explicitly.
## You must decide how to handle the wildcard's return value since it may disagree with the other wildcard handler's return value or your own.

proc callWildcardPreSolveB*(arb: Arbiter; space: Space): bool {.cdecl, importc: "cpArbiterCallWildcardPreSolveB".}
## If you want a custom callback to invoke the wildcard callback for the first collision type, you must call this function explicitly.

proc callWildcardPostSolveA*(arb: Arbiter; space: Space) {.cdecl, importc: "cpArbiterCallWildcardPostSolveA".}
## If you want a custom callback to invoke the wildcard callback for the second collision type, you must call this function explicitly.

proc callWildcardPostSolveB*(arb: Arbiter; space: Space) {.cdecl, importc: "cpArbiterCallWildcardPostSolveB".}
## If you want a custom callback to invoke the wildcard callback for the first collision type, you must call this function explicitly.

proc callWildcardSeparateA*(arb: Arbiter; space: Space) {.cdecl, importc: "cpArbiterCallWildcardSeparateA".}
## If you want a custom callback to invoke the wildcard callback for the second collision type, you must call this function explicitly.

proc callWildcardSeparateB*(arb: Arbiter; space: Space) {.cdecl, importc: "cpArbiterCallWildcardSeparateB".}

## Rigid body velocity update function type.

## Rigid body position update function type.

## Allocate a Body.

proc allocateBody*(): Body {.cdecl, importc: "cpBodyAlloc".}
## Initialize a Body.

proc initializeBody*(body: Body; mass: Float; moment: Float): Body {.cdecl, importc: "cpBodyInit".}
## Allocate and initialize a Body.

proc newBody*(mass: Float; moment: Float): Body {.cdecl, importc: "cpBodyNew".}
## Allocate and initialize a Body, and set it as a kinematic body.

proc newKinematicBody*(): Body {.cdecl, importc: "cpBodyNewKinematic".}
## Allocate and initialize a Body, and set it as a static body.

proc newStaticBody*(): Body {.cdecl, importc: "cpBodyNewStatic".}
## Destroy a Body.

proc finalize*(body: Body) {.cdecl, importc: "cpBodyDestroy".}
## Destroy and free a Body.

proc destroy*(body: Body) {.destroy, cdecl, importc: "cpBodyFree".}
## Wake up a sleeping or idle body.

proc activate*(body: Body) {.cdecl, importc: "cpBodyActivate".}
## Wake up any sleeping or idle bodies touching a static body.

proc activateStatic*(body: Body; filter: Shape) {.cdecl, importc: "cpBodyActivateStatic".}
## Force a body to fall asleep immediately.

proc sleep*(body: Body) {.cdecl, importc: "cpBodySleep".}
## Force a body to fall asleep immediately along with other bodies in a group.

proc sleepWithGroup*(body: Body; group: Body) {.cdecl, importc: "cpBodySleepWithGroup".}
## Returns true if the body is sleeping.

proc isSleeping*(body: Body): bool {.cdecl, importc: "cpBodyIsSleeping".}
## Get the type of the body.

proc bodyType*(body: Body): BodyType {.cdecl, importc: "cpBodyGetType".}
## Set the type of the body.

proc `bodyType=`*(body: Body; `type`: BodyType) {.cdecl, importc: "cpBodySetType".}
## Get the space this body is added to.

proc space*(body: Body): Space {.cdecl, importc: "cpBodyGetSpace".}
## Get the mass of the body.

proc mass*(body: Body): Float {.cdecl, importc: "cpBodyGetMass".}
## Set the mass of the body.

proc `mass=`*(body: Body; m: Float) {.cdecl, importc: "cpBodySetMass".}
## Get the moment of inertia of the body.

proc moment*(body: Body): Float {.cdecl, importc: "cpBodyGetMoment".}
## Set the moment of inertia of the body.

proc `moment=`*(body: Body; i: Float) {.cdecl, importc: "cpBodySetMoment".}
## Set the position of a body.

proc position*(body: Body): Vect {.cdecl, importc: "cpBodyGetPosition".}
## Set the position of the body.

proc `position=`*(body: Body; pos: Vect) {.cdecl, importc: "cpBodySetPosition".}
## Get the offset of the center of gravity in body local coordinates.

proc centerOfGravity*(body: Body): Vect {.cdecl, importc: "cpBodyGetCenterOfGravity".}
## Set the offset of the center of gravity in body local coordinates.

proc `centerOfGravity=`*(body: Body; cog: Vect) {.cdecl, importc: "cpBodySetCenterOfGravity".}
## Get the velocity of the body.

proc velocity*(body: Body): Vect {.cdecl, importc: "cpBodyGetVelocity".}
## Set the velocity of the body.

proc `velocity=`*(body: Body; velocity: Vect) {.cdecl, importc: "cpBodySetVelocity".}
## Get the force applied to the body for the next time step.

proc force*(body: Body): Vect {.cdecl, importc: "cpBodyGetForce".}
## Set the force applied to the body for the next time step.

proc `force=`*(body: Body; force: Vect) {.cdecl, importc: "cpBodySetForce".}
## Get the angle of the body.

proc angle*(body: Body): Float {.cdecl, importc: "cpBodyGetAngle".}
## Set the angle of a body.

proc `angle=`*(body: Body; a: Float) {.cdecl, importc: "cpBodySetAngle".}
## Get the angular velocity of the body.

proc angularVelocity*(body: Body): Float {.cdecl, importc: "cpBodyGetAngularVelocity".}
## Set the angular velocity of the body.

proc `angularVelocity=`*(body: Body; angularVelocity: Float) {.cdecl, importc: "cpBodySetAngularVelocity".}
## Get the torque applied to the body for the next time step.

proc torque*(body: Body): Float {.cdecl, importc: "cpBodyGetTorque".}
## Set the torque applied to the body for the next time step.

proc `torque=`*(body: Body; torque: Float) {.cdecl, importc: "cpBodySetTorque".}
## Get the rotation vector of the body. (The x basis vector of it's transform.)

proc rotation*(body: Body): Vect {.cdecl, importc: "cpBodyGetRotation".}
## Get the user data pointer assigned to the body.

proc userData*(body: Body): DataPointer {.cdecl, importc: "cpBodyGetUserData".}
## Set the user data pointer assigned to the body.

proc `userData=`*(body: Body; userData: DataPointer) {.cdecl, importc: "cpBodySetUserData".}
## Set the callback used to update a body's velocity.

proc `velocityUpdateFunc=`*(body: Body; velocityFunc: BodyVelocityFunc) {.cdecl, importc: "cpBodySetVelocityUpdateFunc".}
## Set the callback used to update a body's position.
## NOTE: It's not generally recommended to override this unless you call the default position update function.

proc `positionUpdateFunc=`*(body: Body; positionFunc: BodyPositionFunc) {.cdecl, importc: "cpBodySetPositionUpdateFunc".}
## Default velocity integration function..

proc updateVelocity*(body: Body; gravity: Vect; damping: Float; dt: Float) {.cdecl, importc: "cpBodyUpdateVelocity".}
## Default position integration function.

proc updatePosition*(body: Body; dt: Float) {.cdecl, importc: "cpBodyUpdatePosition".}
## Convert body relative/local coordinates to absolute/world coordinates.

proc localToWorld*(body: Body; point: Vect): Vect {.cdecl, importc: "cpBodyLocalToWorld".}
## Convert body absolute/world coordinates to  relative/local coordinates.

proc worldToLocal*(body: Body; point: Vect): Vect {.cdecl, importc: "cpBodyWorldToLocal".}
## Apply a force to a body. Both the force and point are expressed in world coordinates.

proc applyForceAtWorldPoint*(body: Body; force: Vect; point: Vect) {.cdecl, importc: "cpBodyApplyForceAtWorldPoint".}
## Apply a force to a body. Both the force and point are expressed in body local coordinates.

proc applyForceAtLocalPoint*(body: Body; force: Vect; point: Vect) {.cdecl, importc: "cpBodyApplyForceAtLocalPoint".}
## Apply an impulse to a body. Both the impulse and point are expressed in world coordinates.

proc applyImpulseAtWorldPoint*(body: Body; impulse: Vect; point: Vect) {.cdecl, importc: "cpBodyApplyImpulseAtWorldPoint".}
## Apply an impulse to a body. Both the impulse and point are expressed in body local coordinates.

proc applyImpulseAtLocalPoint*(body: Body; impulse: Vect; point: Vect) {.cdecl, importc: "cpBodyApplyImpulseAtLocalPoint".}
## Get the velocity on a body (in world units) at a point on the body in world coordinates.

proc velocityAtWorldPoint*(body: Body; point: Vect): Vect {.cdecl, importc: "cpBodyGetVelocityAtWorldPoint".}
## Get the velocity on a body (in world units) at a point on the body in local coordinates.

proc velocityAtLocalPoint*(body: Body; point: Vect): Vect {.cdecl, importc: "cpBodyGetVelocityAtLocalPoint".}
## Get the amount of kinetic energy contained by the body.

proc kineticEnergy*(body: Body): Float {.cdecl, importc: "cpBodyKineticEnergy".}
## Body/shape iterator callback function type.

## Call @c func once for each shape attached to @c body and added to the space.

proc eachShape*(body: Body; `func`: BodyShapeIteratorFunc; data: pointer) {.cdecl, importc: "cpBodyEachShape".}
## Body/constraint iterator callback function type.

## Call @c func once for each constraint attached to @c body and added to the space.

proc eachConstraint*(body: Body; `func`: BodyConstraintIteratorFunc; data: pointer) {.cdecl, importc: "cpBodyEachConstraint".}
## Body/arbiter iterator callback function type.

## Call @c func once for each arbiter that is currently active on the body.

proc eachArbiter*(body: Body; `func`: BodyArbiterIteratorFunc; data: pointer) {.cdecl, importc: "cpBodyEachArbiter".}
## Point query info struct.

## Segment query info struct.

## Fast collision filtering type that is used to determine if two objects collide before calling collision or query callbacks.

## Collision filter value for a shape that will collide with anything except SHAPE_FILTER_NONE.

var SHAPE_FILTER_ALL* = ShapeFilter(group: (cast[Group](0)))

## Collision filter value for a shape that does not collide with anything.

var SHAPE_FILTER_NONE* = ShapeFilter(group: (cast[Group](0)))

## Create a new collision filter.

proc newShapeFilter*(group: Group; categories: Bitmask; mask: Bitmask): ShapeFilter {.inline, cdecl.} =
  var filter = ShapeFilter(group: group)
  return filter

## Destroy a shape.

proc finalize*(shape: Shape) {.cdecl, importc: "cpShapeDestroy".}
## Destroy and Free a shape.

proc destroy*(shape: Shape) {.destroy, cdecl, importc: "cpShapeFree".}
proc destroy*(shape: PolyShape) {.destroy, cdecl, importc: "cpShapeFree".}
proc destroy*(shape: SegmentShape) {.destroy, cdecl, importc: "cpShapeFree".}
proc destroy*(shape: CircleShape) {.destroy, cdecl, importc: "cpShapeFree".}
## Update, cache and return the bounding box of a shape based on the body it's attached to.

proc cacheBB*(shape: Shape): BB {.cdecl, importc: "cpShapeCacheBB".}
## Update, cache and return the bounding box of a shape with an explicit transformation.

proc update*(shape: Shape; transform: Transform): BB {.cdecl, importc: "cpShapeUpdate".}
## Perform a nearest point query. It finds the closest point on the surface of shape to a specific point.
## The value returned is the distance between the points. A negative distance means the point is inside the shape.

proc pointQuery*(shape: Shape; p: Vect; `out`: ptr PointQueryInfo): Float {.cdecl, importc: "cpShapePointQuery".}
## Perform a segment query against a shape. @c info must be a pointer to a valid SegmentQueryInfo structure.

proc segmentQuery*(shape: Shape; a: Vect; b: Vect; radius: Float; info: ptr SegmentQueryInfo): bool {.cdecl, importc: "cpShapeSegmentQuery".}
## Return contact information about two shapes.

proc sCollide*(a: Shape; b: Shape): ContactPointSet {.cdecl, importc: "cpShapesCollide".}
## The Space this body is added to.

proc space*(shape: Shape): Space {.cdecl, importc: "cpShapeGetSpace".}
## The Body this shape is connected to.

proc body*(shape: Shape): Body {.cdecl, importc: "cpShapeGetBody".}
## Set the Body this shape is connected to.
## Can only be used if the shape is not currently added to a space.

proc `body=`*(shape: Shape; body: Body) {.cdecl, importc: "cpShapeSetBody".}
## Get the mass of the shape if you are having Chipmunk calculate mass properties for you.

proc mass*(shape: Shape): Float {.cdecl, importc: "cpShapeGetMass".}
## Set the mass of this shape to have Chipmunk calculate mass properties for you.

proc `mass=`*(shape: Shape; mass: Float) {.cdecl, importc: "cpShapeSetMass".}
## Get the density of the shape if you are having Chipmunk calculate mass properties for you.

proc density*(shape: Shape): Float {.cdecl, importc: "cpShapeGetDensity".}
## Set the density  of this shape to have Chipmunk calculate mass properties for you.

proc `density=`*(shape: Shape; density: Float) {.cdecl, importc: "cpShapeSetDensity".}
## Get the calculated moment of inertia for this shape.

proc moment*(shape: Shape): Float {.cdecl, importc: "cpShapeGetMoment".}
## Get the calculated area of this shape.

proc area*(shape: Shape): Float {.cdecl, importc: "cpShapeGetArea".}
## Get the centroid of this shape.

proc centerOfGravity*(shape: Shape): Vect {.cdecl, importc: "cpShapeGetCenterOfGravity".}
## Get the bounding box that contains the shape given it's current position and angle.

proc bB*(shape: Shape): BB {.cdecl, importc: "cpShapeGetBB".}
## Get if the shape is set to be a sensor or not.

proc sensor*(shape: Shape): bool {.cdecl, importc: "cpShapeGetSensor".}
## Set if the shape is a sensor or not.

proc `sensor=`*(shape: Shape; sensor: bool) {.cdecl, importc: "cpShapeSetSensor".}
## Get the elasticity of this shape.

proc elasticity*(shape: Shape): Float {.cdecl, importc: "cpShapeGetElasticity".}
## Set the elasticity of this shape.

proc `elasticity=`*(shape: Shape; elasticity: Float) {.cdecl, importc: "cpShapeSetElasticity".}
## Get the friction of this shape.

proc friction*(shape: Shape): Float {.cdecl, importc: "cpShapeGetFriction".}
## Set the friction of this shape.

proc `friction=`*(shape: Shape; friction: Float) {.cdecl, importc: "cpShapeSetFriction".}
## Get the surface velocity of this shape.

proc surfaceVelocity*(shape: Shape): Vect {.cdecl, importc: "cpShapeGetSurfaceVelocity".}
## Set the surface velocity of this shape.

proc `surfaceVelocity=`*(shape: Shape; surfaceVelocity: Vect) {.cdecl, importc: "cpShapeSetSurfaceVelocity".}
## Get the user definable data pointer of this shape.

proc userData*(shape: Shape): DataPointer {.cdecl, importc: "cpShapeGetUserData".}
## Set the user definable data pointer of this shape.

proc `userData=`*(shape: Shape; userData: DataPointer) {.cdecl, importc: "cpShapeSetUserData".}
## Set the collision type of this shape.

proc collisionType*(shape: Shape): CollisionType {.cdecl, importc: "cpShapeGetCollisionType".}
## Get the collision type of this shape.

proc `collisionType=`*(shape: Shape; collisionType: CollisionType) {.cdecl, importc: "cpShapeSetCollisionType".}
## Get the collision filtering parameters of this shape.

proc filter*(shape: Shape): ShapeFilter {.cdecl, importc: "cpShapeGetFilter".}
## Set the collision filtering parameters of this shape.

proc `filter=`*(shape: Shape; filter: ShapeFilter) {.cdecl, importc: "cpShapeSetFilter".}
## Allocate a circle shape.

proc allocateCircleShape*(): CircleShape {.cdecl, importc: "cpCircleShapeAlloc".}
## Initialize a circle shape.

proc initializeCircleShape*(circle: CircleShape; body: Body; radius: Float; offset: Vect): CircleShape {.cdecl, importc: "cpCircleShapeInit".}
## Allocate and initialize a circle shape.

proc newCircleShape*(body: Body; radius: Float; offset: Vect): CircleShape {.cdecl, importc: "cpCircleShapeNew".}
## Get the offset of a circle shape.

proc offset*(shape: CircleShape): Vect {.cdecl, importc: "cpCircleShapeGetOffset".}
## Get the radius of a circle shape.

proc radius*(shape: CircleShape): Float {.cdecl, importc: "cpCircleShapeGetRadius".}
## Allocate a segment shape.

proc allocateSegmentShape*(): SegmentShape {.cdecl, importc: "cpSegmentShapeAlloc".}
## Initialize a segment shape.

proc initializeSegmentShape*(seg: SegmentShape; body: Body; a: Vect; b: Vect; radius: Float): SegmentShape {.cdecl, importc: "cpSegmentShapeInit".}
## Allocate and initialize a segment shape.

proc newSegmentShape*(body: Body; a: Vect; b: Vect; radius: Float): SegmentShape {.cdecl, importc: "cpSegmentShapeNew".}
## Let Chipmunk know about the geometry of adjacent segments to avoid colliding with endcaps.

proc `neighbors=`*(shape: SegmentShape; prev: Vect; next: Vect) {.cdecl, importc: "cpSegmentShapeSetNeighbors".}
## Get the first endpoint of a segment shape.

proc a*(shape: SegmentShape): Vect {.cdecl, importc: "cpSegmentShapeGetA".}
## Get the second endpoint of a segment shape.

proc b*(shape: SegmentShape): Vect {.cdecl, importc: "cpSegmentShapeGetB".}
## Get the normal of a segment shape.

proc normal*(shape: SegmentShape): Vect {.cdecl, importc: "cpSegmentShapeGetNormal".}
## Get the first endpoint of a segment shape.

proc radius*(shape: SegmentShape): Float {.cdecl, importc: "cpSegmentShapeGetRadius".}
## Allocate a polygon shape.

proc allocatePolyShape*(): PolyShape {.cdecl, importc: "cpPolyShapeAlloc".}
## Initialize a polygon shape with rounded corners.
## A convex hull will be created from the vertexes.

proc initializePolyShape*(poly: PolyShape; body: Body; count: cint; verts: ptr Vect; transform: Transform; radius: Float): PolyShape {.cdecl, importc: "cpPolyShapeInit".}
## Initialize a polygon shape with rounded corners.
## The vertexes must be convex with a counter-clockwise winding.

proc initializePolyShape*(poly: PolyShape; body: Body; count: cint; verts: ptr Vect; radius: Float): PolyShape {.cdecl, importc: "cpPolyShapeInitRaw".}
## Allocate and initialize a polygon shape with rounded corners.
## A convex hull will be created from the vertexes.

proc newPolyShape*(body: Body; count: cint; verts: ptr Vect; transform: Transform; radius: Float): PolyShape {.cdecl, importc: "cpPolyShapeNew".}
## Allocate and initialize a polygon shape with rounded corners.
## The vertexes must be convex with a counter-clockwise winding.

proc newPolyShape*(body: Body; count: cint; verts: ptr Vect; radius: Float): PolyShape {.cdecl, importc: "cpPolyShapeNewRaw".}
## Initialize a box shaped polygon shape with rounded corners.

proc initializeBoxShape*(poly: PolyShape; body: Body; width: Float; height: Float; radius: Float): PolyShape {.cdecl, importc: "cpBoxShapeInit".}
## Initialize an offset box shaped polygon shape with rounded corners.

proc initializeBoxShape*(poly: PolyShape; body: Body; box: BB; radius: Float): PolyShape {.cdecl, importc: "cpBoxShapeInit2".}
## Allocate and initialize a box shaped polygon shape.

proc newBoxShape*(body: Body; width: Float; height: Float; radius: Float): PolyShape {.cdecl, importc: "cpBoxShapeNew".}
## Allocate and initialize an offset box shaped polygon shape.

proc newBoxShape*(body: Body; box: BB; radius: Float): PolyShape {.cdecl, importc: "cpBoxShapeNew2".}
## Get the number of verts in a polygon shape.

proc count*(shape: PolyShape): cint {.cdecl, importc: "cpPolyShapeGetCount".}
## Get the @c ith vertex of a polygon shape.

proc vert*(shape: PolyShape; index: cint): Vect {.cdecl, importc: "cpPolyShapeGetVert".}
## Get the radius of a polygon shape.

proc radius*(shape: PolyShape): Float {.cdecl, importc: "cpPolyShapeGetRadius".}
## Callback function type that gets called before solving a joint.

## Callback function type that gets called after solving a joint.

## Destroy a constraint.

proc finalize*(constraint: Constraint) {.cdecl, importc: "cpConstraintDestroy".}
## Destroy and free a constraint.

proc destroy*(constraint: Constraint) {.destroy, cdecl, importc: "cpConstraintFree".}
proc destroy*(constraint: PinJoint) {.destroy, cdecl, importc: "cpConstraintFree".}
proc destroy*(constraint: DampedSpring) {.destroy, cdecl, importc: "cpConstraintFree".}
proc destroy*(constraint: RotaryLimitJoint) {.destroy, cdecl, importc: "cpConstraintFree".}
proc destroy*(constraint: SlideJoint) {.destroy, cdecl, importc: "cpConstraintFree".}
proc destroy*(constraint: GearJoint) {.destroy, cdecl, importc: "cpConstraintFree".}
proc destroy*(constraint: DampedRotarySpring) {.destroy, cdecl, importc: "cpConstraintFree".}
proc destroy*(constraint: GrooveJoint) {.destroy, cdecl, importc: "cpConstraintFree".}
proc destroy*(constraint: RatchetJoint) {.destroy, cdecl, importc: "cpConstraintFree".}
proc destroy*(constraint: SimpleMotor) {.destroy, cdecl, importc: "cpConstraintFree".}
proc destroy*(constraint: PivotJoint) {.destroy, cdecl, importc: "cpConstraintFree".}
## Get the Space this constraint is added to.

proc space*(constraint: Constraint): Space {.cdecl, importc: "cpConstraintGetSpace".}
## Get the first body the constraint is attached to.

proc bodyA*(constraint: Constraint): Body {.cdecl, importc: "cpConstraintGetBodyA".}
## Get the second body the constraint is attached to.

proc bodyB*(constraint: Constraint): Body {.cdecl, importc: "cpConstraintGetBodyB".}
## Get the maximum force that this constraint is allowed to use.

proc maxForce*(constraint: Constraint): Float {.cdecl, importc: "cpConstraintGetMaxForce".}
## Set the maximum force that this constraint is allowed to use. (defaults to Inf)

proc `maxForce=`*(constraint: Constraint; maxForce: Float) {.cdecl, importc: "cpConstraintSetMaxForce".}
## Get rate at which joint error is corrected.

proc errorBias*(constraint: Constraint): Float {.cdecl, importc: "cpConstraintGetErrorBias".}
## Set rate at which joint error is corrected.
## Defaults to pow(1.0 - 0.1, 60.0) meaning that it will
## correct 10% of the error every 1/60th of a second.

proc `errorBias=`*(constraint: Constraint; errorBias: Float) {.cdecl, importc: "cpConstraintSetErrorBias".}
## Get the maximum rate at which joint error is corrected.

proc maxBias*(constraint: Constraint): Float {.cdecl, importc: "cpConstraintGetMaxBias".}
## Set the maximum rate at which joint error is corrected. (defaults to Inf)

proc `maxBias=`*(constraint: Constraint; maxBias: Float) {.cdecl, importc: "cpConstraintSetMaxBias".}
## Get if the two bodies connected by the constraint are allowed to collide or not.

proc collideBodies*(constraint: Constraint): bool {.cdecl, importc: "cpConstraintGetCollideBodies".}
## Set if the two bodies connected by the constraint are allowed to collide or not. (defaults to False)

proc `collideBodies=`*(constraint: Constraint; collideBodies: bool) {.cdecl, importc: "cpConstraintSetCollideBodies".}
## Get the pre-solve function that is called before the solver runs.

proc preSolveFunc*(constraint: Constraint): ConstraintPreSolveFunc {.cdecl, importc: "cpConstraintGetPreSolveFunc".}
## Set the pre-solve function that is called before the solver runs.

proc `preSolveFunc=`*(constraint: Constraint; preSolveFunc: ConstraintPreSolveFunc) {.cdecl, importc: "cpConstraintSetPreSolveFunc".}
## Get the post-solve function that is called before the solver runs.

proc postSolveFunc*(constraint: Constraint): ConstraintPostSolveFunc {.cdecl, importc: "cpConstraintGetPostSolveFunc".}
## Set the post-solve function that is called before the solver runs.

proc `postSolveFunc=`*(constraint: Constraint; postSolveFunc: ConstraintPostSolveFunc) {.cdecl, importc: "cpConstraintSetPostSolveFunc".}
## Get the user definable data pointer for this constraint

proc userData*(constraint: Constraint): DataPointer {.cdecl, importc: "cpConstraintGetUserData".}
## Set the user definable data pointer for this constraint

proc `userData=`*(constraint: Constraint; userData: DataPointer) {.cdecl, importc: "cpConstraintSetUserData".}
## Get the last impulse applied by this constraint.

proc impulse*(constraint: Constraint): Float {.cdecl, importc: "cpConstraintGetImpulse".}
## Check if a constraint is a pin joint.

proc isPinJoint*(constraint: Constraint): bool {.cdecl, importc: "cpConstraintIsPinJoint".}
## Allocate a pin joint.

proc allocatePinJoint*(): PinJoint {.cdecl, importc: "cpPinJointAlloc".}
## Initialize a pin joint.

proc initializePinJoint*(joint: PinJoint; a: Body; b: Body; anchorA: Vect; anchorB: Vect): PinJoint {.cdecl, importc: "cpPinJointInit".}
## Allocate and initialize a pin joint.

proc newPinJoint*(a: Body; b: Body; anchorA: Vect; anchorB: Vect): PinJoint {.cdecl, importc: "cpPinJointNew".}
## Get the location of the first anchor relative to the first body.

proc anchorA*(constraint: PinJoint): Vect {.cdecl, importc: "cpPinJointGetAnchorA".}
## Set the location of the first anchor relative to the first body.

proc `anchorA=`*(constraint: PinJoint; anchorA: Vect) {.cdecl, importc: "cpPinJointSetAnchorA".}
## Get the location of the second anchor relative to the second body.

proc anchorB*(constraint: PinJoint): Vect {.cdecl, importc: "cpPinJointGetAnchorB".}
## Set the location of the second anchor relative to the second body.

proc `anchorB=`*(constraint: PinJoint; anchorB: Vect) {.cdecl, importc: "cpPinJointSetAnchorB".}
## Get the distance the joint will maintain between the two anchors.

proc dist*(constraint: PinJoint): Float {.cdecl, importc: "cpPinJointGetDist".}
## Set the distance the joint will maintain between the two anchors.

proc `dist=`*(constraint: PinJoint; dist: Float) {.cdecl, importc: "cpPinJointSetDist".}
## Check if a constraint is a slide joint.

proc isSlideJoint*(constraint: Constraint): bool {.cdecl, importc: "cpConstraintIsSlideJoint".}
## Allocate a slide joint.

proc allocateSlideJoint*(): SlideJoint {.cdecl, importc: "cpSlideJointAlloc".}
## Initialize a slide joint.

proc initializeSlideJoint*(joint: SlideJoint; a: Body; b: Body; anchorA: Vect; anchorB: Vect; min: Float; max: Float): SlideJoint {.cdecl, importc: "cpSlideJointInit".}
## Allocate and initialize a slide joint.

proc newSlideJoint*(a: Body; b: Body; anchorA: Vect; anchorB: Vect; min: Float; max: Float): SlideJoint {.cdecl, importc: "cpSlideJointNew".}
## Get the location of the first anchor relative to the first body.

proc anchorA*(constraint: SlideJoint): Vect {.cdecl, importc: "cpSlideJointGetAnchorA".}
## Set the location of the first anchor relative to the first body.

proc `anchorA=`*(constraint: SlideJoint; anchorA: Vect) {.cdecl, importc: "cpSlideJointSetAnchorA".}
## Get the location of the second anchor relative to the second body.

proc anchorB*(constraint: SlideJoint): Vect {.cdecl, importc: "cpSlideJointGetAnchorB".}
## Set the location of the second anchor relative to the second body.

proc `anchorB=`*(constraint: SlideJoint; anchorB: Vect) {.cdecl, importc: "cpSlideJointSetAnchorB".}
## Get the minimum distance the joint will maintain between the two anchors.

proc min*(constraint: SlideJoint): Float {.cdecl, importc: "cpSlideJointGetMin".}
## Set the minimum distance the joint will maintain between the two anchors.

proc `min=`*(constraint: SlideJoint; min: Float) {.cdecl, importc: "cpSlideJointSetMin".}
## Get the maximum distance the joint will maintain between the two anchors.

proc max*(constraint: SlideJoint): Float {.cdecl, importc: "cpSlideJointGetMax".}
## Set the maximum distance the joint will maintain between the two anchors.

proc `max=`*(constraint: SlideJoint; max: Float) {.cdecl, importc: "cpSlideJointSetMax".}
## Check if a constraint is a slide joint.

proc isPivotJoint*(constraint: Constraint): bool {.cdecl, importc: "cpConstraintIsPivotJoint".}
## Allocate a pivot joint

proc allocatePivotJoint*(): PivotJoint {.cdecl, importc: "cpPivotJointAlloc".}
## Initialize a pivot joint.

proc initializePivotJoint*(joint: PivotJoint; a: Body; b: Body; anchorA: Vect; anchorB: Vect): PivotJoint {.cdecl, importc: "cpPivotJointInit".}
## Allocate and initialize a pivot joint.

proc newPivotJoint*(a: Body; b: Body; pivot: Vect): PivotJoint {.cdecl, importc: "cpPivotJointNew".}
## Allocate and initialize a pivot joint with specific anchors.

proc newPivotJoint*(a: Body; b: Body; anchorA: Vect; anchorB: Vect): PivotJoint {.cdecl, importc: "cpPivotJointNew2".}
## Get the location of the first anchor relative to the first body.

proc anchorA*(constraint: PivotJoint): Vect {.cdecl, importc: "cpPivotJointGetAnchorA".}
## Set the location of the first anchor relative to the first body.

proc `anchorA=`*(constraint: PivotJoint; anchorA: Vect) {.cdecl, importc: "cpPivotJointSetAnchorA".}
## Get the location of the second anchor relative to the second body.

proc anchorB*(constraint: PivotJoint): Vect {.cdecl, importc: "cpPivotJointGetAnchorB".}
## Set the location of the second anchor relative to the second body.

proc `anchorB=`*(constraint: PivotJoint; anchorB: Vect) {.cdecl, importc: "cpPivotJointSetAnchorB".}
## Check if a constraint is a slide joint.

proc isGrooveJoint*(constraint: Constraint): bool {.cdecl, importc: "cpConstraintIsGrooveJoint".}
## Allocate a groove joint.

proc allocateGrooveJoint*(): GrooveJoint {.cdecl, importc: "cpGrooveJointAlloc".}
## Initialize a groove joint.

proc initializeGrooveJoint*(joint: GrooveJoint; a: Body; b: Body; groove_a: Vect; groove_b: Vect; anchorB: Vect): GrooveJoint {.cdecl, importc: "cpGrooveJointInit".}
## Allocate and initialize a groove joint.

proc newGrooveJoint*(a: Body; b: Body; groove_a: Vect; groove_b: Vect; anchorB: Vect): GrooveJoint {.cdecl, importc: "cpGrooveJointNew".}
## Get the first endpoint of the groove relative to the first body.

proc grooveA*(constraint: GrooveJoint): Vect {.cdecl, importc: "cpGrooveJointGetGrooveA".}
## Set the first endpoint of the groove relative to the first body.

proc `grooveA=`*(constraint: GrooveJoint; grooveA: Vect) {.cdecl, importc: "cpGrooveJointSetGrooveA".}
## Get the first endpoint of the groove relative to the first body.

proc grooveB*(constraint: GrooveJoint): Vect {.cdecl, importc: "cpGrooveJointGetGrooveB".}
## Set the first endpoint of the groove relative to the first body.

proc `grooveB=`*(constraint: GrooveJoint; grooveB: Vect) {.cdecl, importc: "cpGrooveJointSetGrooveB".}
## Get the location of the second anchor relative to the second body.

proc anchorB*(constraint: GrooveJoint): Vect {.cdecl, importc: "cpGrooveJointGetAnchorB".}
## Set the location of the second anchor relative to the second body.

proc `anchorB=`*(constraint: GrooveJoint; anchorB: Vect) {.cdecl, importc: "cpGrooveJointSetAnchorB".}
## Check if a constraint is a slide joint.

proc isDampedSpring*(constraint: Constraint): bool {.cdecl, importc: "cpConstraintIsDampedSpring".}
## Function type used for damped spring force callbacks.

## Allocate a damped spring.

proc allocateDampedSpring*(): DampedSpring {.cdecl, importc: "cpDampedSpringAlloc".}
## Initialize a damped spring.

proc initializeDampedSpring*(joint: DampedSpring; a: Body; b: Body; anchorA: Vect; anchorB: Vect; restLength: Float; stiffness: Float; damping: Float): DampedSpring {.cdecl, importc: "cpDampedSpringInit".}
## Allocate and initialize a damped spring.

proc newDampedSpring*(a: Body; b: Body; anchorA: Vect; anchorB: Vect; restLength: Float; stiffness: Float; damping: Float): DampedSpring {.cdecl, importc: "cpDampedSpringNew".}
## Get the location of the first anchor relative to the first body.

proc anchorA*(constraint: DampedSpring): Vect {.cdecl, importc: "cpDampedSpringGetAnchorA".}
## Set the location of the first anchor relative to the first body.

proc `anchorA=`*(constraint: DampedSpring; anchorA: Vect) {.cdecl, importc: "cpDampedSpringSetAnchorA".}
## Get the location of the second anchor relative to the second body.

proc anchorB*(constraint: DampedSpring): Vect {.cdecl, importc: "cpDampedSpringGetAnchorB".}
## Set the location of the second anchor relative to the second body.

proc `anchorB=`*(constraint: DampedSpring; anchorB: Vect) {.cdecl, importc: "cpDampedSpringSetAnchorB".}
## Get the rest length of the spring.

proc restLength*(constraint: DampedSpring): Float {.cdecl, importc: "cpDampedSpringGetRestLength".}
## Set the rest length of the spring.

proc `restLength=`*(constraint: DampedSpring; restLength: Float) {.cdecl, importc: "cpDampedSpringSetRestLength".}
## Get the stiffness of the spring in force/distance.

proc stiffness*(constraint: DampedSpring): Float {.cdecl, importc: "cpDampedSpringGetStiffness".}
## Set the stiffness of the spring in force/distance.

proc `stiffness=`*(constraint: DampedSpring; stiffness: Float) {.cdecl, importc: "cpDampedSpringSetStiffness".}
## Get the damping of the spring.

proc damping*(constraint: DampedSpring): Float {.cdecl, importc: "cpDampedSpringGetDamping".}
## Set the damping of the spring.

proc `damping=`*(constraint: DampedSpring; damping: Float) {.cdecl, importc: "cpDampedSpringSetDamping".}
## Get the damping of the spring.

proc springForceFunc*(constraint: DampedSpring): DampedSpringForceFunc {.cdecl, importc: "cpDampedSpringGetSpringForceFunc".}
## Set the damping of the spring.

proc `springForceFunc=`*(constraint: DampedSpring; springForceFunc: DampedSpringForceFunc) {.cdecl, importc: "cpDampedSpringSetSpringForceFunc".}
## Check if a constraint is a damped rotary springs.

proc isDampedRotarySpring*(constraint: Constraint): bool {.cdecl, importc: "cpConstraintIsDampedRotarySpring".}
## Function type used for damped rotary spring force callbacks.

## Allocate a damped rotary spring.

proc allocateDampedRotarySpring*(): DampedRotarySpring {.cdecl, importc: "cpDampedRotarySpringAlloc".}
## Initialize a damped rotary spring.

proc initializeDampedRotarySpring*(joint: DampedRotarySpring; a: Body; b: Body; restAngle: Float; stiffness: Float; damping: Float): DampedRotarySpring {.cdecl, importc: "cpDampedRotarySpringInit".}
## Allocate and initialize a damped rotary spring.

proc newDampedRotarySpring*(a: Body; b: Body; restAngle: Float; stiffness: Float; damping: Float): DampedRotarySpring {.cdecl, importc: "cpDampedRotarySpringNew".}
## Get the rest length of the spring.

proc restAngle*(constraint: DampedRotarySpring): Float {.cdecl, importc: "cpDampedRotarySpringGetRestAngle".}
## Set the rest length of the spring.

proc `restAngle=`*(constraint: DampedRotarySpring; restAngle: Float) {.cdecl, importc: "cpDampedRotarySpringSetRestAngle".}
## Get the stiffness of the spring in force/distance.

proc stiffness*(constraint: DampedRotarySpring): Float {.cdecl, importc: "cpDampedRotarySpringGetStiffness".}
## Set the stiffness of the spring in force/distance.

proc `stiffness=`*(constraint: DampedRotarySpring; stiffness: Float) {.cdecl, importc: "cpDampedRotarySpringSetStiffness".}
## Get the damping of the spring.

proc damping*(constraint: DampedRotarySpring): Float {.cdecl, importc: "cpDampedRotarySpringGetDamping".}
## Set the damping of the spring.

proc `damping=`*(constraint: DampedRotarySpring; damping: Float) {.cdecl, importc: "cpDampedRotarySpringSetDamping".}
## Get the damping of the spring.

proc springTorqueFunc*(constraint: DampedRotarySpring): DampedRotarySpringTorqueFunc {.cdecl, importc: "cpDampedRotarySpringGetSpringTorqueFunc".}
## Set the damping of the spring.

proc `springTorqueFunc=`*(constraint: DampedRotarySpring; springTorqueFunc: DampedRotarySpringTorqueFunc) {.cdecl, importc: "cpDampedRotarySpringSetSpringTorqueFunc".}
## Check if a constraint is a damped rotary springs.

proc isRotaryLimitJoint*(constraint: Constraint): bool {.cdecl, importc: "cpConstraintIsRotaryLimitJoint".}
## Allocate a damped rotary limit joint.

proc allocateRotaryLimitJoint*(): RotaryLimitJoint {.cdecl, importc: "cpRotaryLimitJointAlloc".}
## Initialize a damped rotary limit joint.

proc initializeRotaryLimitJoint*(joint: RotaryLimitJoint; a: Body; b: Body; min: Float; max: Float): RotaryLimitJoint {.cdecl, importc: "cpRotaryLimitJointInit".}
## Allocate and initialize a damped rotary limit joint.

proc newRotaryLimitJoint*(a: Body; b: Body; min: Float; max: Float): RotaryLimitJoint {.cdecl, importc: "cpRotaryLimitJointNew".}
## Get the minimum distance the joint will maintain between the two anchors.

proc min*(constraint: RotaryLimitJoint): Float {.cdecl, importc: "cpRotaryLimitJointGetMin".}
## Set the minimum distance the joint will maintain between the two anchors.

proc `min=`*(constraint: RotaryLimitJoint; min: Float) {.cdecl, importc: "cpRotaryLimitJointSetMin".}
## Get the maximum distance the joint will maintain between the two anchors.

proc max*(constraint: RotaryLimitJoint): Float {.cdecl, importc: "cpRotaryLimitJointGetMax".}
## Set the maximum distance the joint will maintain between the two anchors.

proc `max=`*(constraint: RotaryLimitJoint; max: Float) {.cdecl, importc: "cpRotaryLimitJointSetMax".}
## Check if a constraint is a damped rotary springs.

proc isRatchetJoint*(constraint: Constraint): bool {.cdecl, importc: "cpConstraintIsRatchetJoint".}
## Allocate a ratchet joint.

proc allocateRatchetJoint*(): RatchetJoint {.cdecl, importc: "cpRatchetJointAlloc".}
## Initialize a ratched joint.

proc initializeRatchetJoint*(joint: RatchetJoint; a: Body; b: Body; phase: Float; ratchet: Float): RatchetJoint {.cdecl, importc: "cpRatchetJointInit".}
## Allocate and initialize a ratchet joint.

proc newRatchetJoint*(a: Body; b: Body; phase: Float; ratchet: Float): RatchetJoint {.cdecl, importc: "cpRatchetJointNew".}
## Get the angle of the current ratchet tooth.

proc angle*(constraint: RatchetJoint): Float {.cdecl, importc: "cpRatchetJointGetAngle".}
## Set the angle of the current ratchet tooth.

proc `angle=`*(constraint: RatchetJoint; angle: Float) {.cdecl, importc: "cpRatchetJointSetAngle".}
## Get the phase offset of the ratchet.

proc phase*(constraint: RatchetJoint): Float {.cdecl, importc: "cpRatchetJointGetPhase".}
## Get the phase offset of the ratchet.

proc `phase=`*(constraint: RatchetJoint; phase: Float) {.cdecl, importc: "cpRatchetJointSetPhase".}
## Get the angular distance of each ratchet.

proc ratchet*(constraint: RatchetJoint): Float {.cdecl, importc: "cpRatchetJointGetRatchet".}
## Set the angular distance of each ratchet.

proc `ratchet=`*(constraint: RatchetJoint; ratchet: Float) {.cdecl, importc: "cpRatchetJointSetRatchet".}
## Check if a constraint is a damped rotary springs.

proc isGearJoint*(constraint: Constraint): bool {.cdecl, importc: "cpConstraintIsGearJoint".}
## Allocate a gear joint.

proc allocateGearJoint*(): GearJoint {.cdecl, importc: "cpGearJointAlloc".}
## Initialize a gear joint.

proc initializeGearJoint*(joint: GearJoint; a: Body; b: Body; phase: Float; ratio: Float): GearJoint {.cdecl, importc: "cpGearJointInit".}
## Allocate and initialize a gear joint.

proc newGearJoint*(a: Body; b: Body; phase: Float; ratio: Float): GearJoint {.cdecl, importc: "cpGearJointNew".}
## Get the phase offset of the gears.

proc phase*(constraint: GearJoint): Float {.cdecl, importc: "cpGearJointGetPhase".}
## Set the phase offset of the gears.

proc `phase=`*(constraint: GearJoint; phase: Float) {.cdecl, importc: "cpGearJointSetPhase".}
## Get the angular distance of each ratchet.

proc ratio*(constraint: GearJoint): Float {.cdecl, importc: "cpGearJointGetRatio".}
## Set the ratio of a gear joint.

proc `ratio=`*(constraint: GearJoint; ratio: Float) {.cdecl, importc: "cpGearJointSetRatio".}
## Opaque struct type for damped rotary springs.

## Check if a constraint is a damped rotary springs.

proc isSimpleMotor*(constraint: Constraint): bool {.cdecl, importc: "cpConstraintIsSimpleMotor".}
## Allocate a simple motor.

proc allocateSimpleMotor*(): SimpleMotor {.cdecl, importc: "cpSimpleMotorAlloc".}
## initialize a simple motor.

proc initializeSimpleMotor*(joint: SimpleMotor; a: Body; b: Body; rate: Float): SimpleMotor {.cdecl, importc: "cpSimpleMotorInit".}
## Allocate and initialize a simple motor.

proc newSimpleMotor*(a: Body; b: Body; rate: Float): SimpleMotor {.cdecl, importc: "cpSimpleMotorNew".}
## Get the rate of the motor.

proc rate*(constraint: SimpleMotor): Float {.cdecl, importc: "cpSimpleMotorGetRate".}
## Set the rate of the motor.

proc `rate=`*(constraint: SimpleMotor; rate: Float) {.cdecl, importc: "cpSimpleMotorSetRate".}
## Collision begin event function callback type.
## Returning false from a begin callback causes the collision to be ignored until
## the the separate callback is called when the objects stop colliding.

## Collision pre-solve event function callback type.
## Returning false from a pre-step callback causes the collision to be ignored until the next step.

## Collision post-solve event function callback type.

## Collision separate event function callback type.

## Struct that holds function callback pointers to configure custom collision handling.
## Collision handlers have a pair of types; when a collision occurs between two shapes that have these types, the collision handler functions are triggered.

## Allocate a Space.

proc allocateSpace*(): Space {.cdecl, importc: "cpSpaceAlloc".}
## Initialize a Space.

proc initializeSpace*(space: Space): Space {.cdecl, importc: "cpSpaceInit".}
## Allocate and initialize a Space.

proc newSpace*(): Space {.cdecl, importc: "cpSpaceNew".}
## Destroy a Space.

proc finalize*(space: Space) {.cdecl, importc: "cpSpaceDestroy".}
## Destroy and free a Space.

proc destroy*(space: Space) {.destroy, cdecl, importc: "cpSpaceFree".}
## Number of iterations to use in the impulse solver to solve contacts and other constraints.

proc iterations*(space: Space): cint {.cdecl, importc: "cpSpaceGetIterations".}
proc `iterations=`*(space: Space; iterations: cint) {.cdecl, importc: "cpSpaceSetIterations".}
## Gravity to pass to rigid bodies when integrating velocity.

proc gravity*(space: Space): Vect {.cdecl, importc: "cpSpaceGetGravity".}
proc `gravity=`*(space: Space; gravity: Vect) {.cdecl, importc: "cpSpaceSetGravity".}
## Damping rate expressed as the fraction of velocity bodies retain each second.
## A value of 0.9 would mean that each body's velocity will drop 10% per second.
## The default value is 1.0, meaning no damping is applied.
## @note This damping value is different than those of DampedSpring and DampedRotarySpring.

proc damping*(space: Space): Float {.cdecl, importc: "cpSpaceGetDamping".}
proc `damping=`*(space: Space; damping: Float) {.cdecl, importc: "cpSpaceSetDamping".}
## Speed threshold for a body to be considered idle.
## The default value of 0 means to let the space guess a good threshold based on gravity.

proc idleSpeedThreshold*(space: Space): Float {.cdecl, importc: "cpSpaceGetIdleSpeedThreshold".}
proc `idleSpeedThreshold=`*(space: Space; idleSpeedThreshold: Float) {.cdecl, importc: "cpSpaceSetIdleSpeedThreshold".}
## Time a group of bodies must remain idle in order to fall asleep.
## Enabling sleeping also implicitly enables the the contact graph.
## The default value of Inf disables the sleeping algorithm.

proc sleepTimeThreshold*(space: Space): Float {.cdecl, importc: "cpSpaceGetSleepTimeThreshold".}
proc `sleepTimeThreshold=`*(space: Space; sleepTimeThreshold: Float) {.cdecl, importc: "cpSpaceSetSleepTimeThreshold".}
## Amount of encouraged penetration between colliding shapes.
## Used to reduce oscillating contacts and keep the collision cache warm.
## Defaults to 0.1. If you have poor simulation quality,
## increase this number as much as possible without allowing visible amounts of overlap.

proc collisionSlop*(space: Space): Float {.cdecl, importc: "cpSpaceGetCollisionSlop".}
proc `collisionSlop=`*(space: Space; collisionSlop: Float) {.cdecl, importc: "cpSpaceSetCollisionSlop".}
## Determines how fast overlapping shapes are pushed apart.
## Expressed as a fraction of the error remaining after each second.
## Defaults to pow(1.0 - 0.1, 60.0) meaning that Chipmunk fixes 10% of overlap each frame at 60Hz.

proc collisionBias*(space: Space): Float {.cdecl, importc: "cpSpaceGetCollisionBias".}
proc `collisionBias=`*(space: Space; collisionBias: Float) {.cdecl, importc: "cpSpaceSetCollisionBias".}
## Number of frames that contact information should persist.
## Defaults to 3. There is probably never a reason to change this value.

proc collisionPersistence*(space: Space): Timestamp {.cdecl, importc: "cpSpaceGetCollisionPersistence".}
proc `collisionPersistence=`*(space: Space; collisionPersistence: Timestamp) {.cdecl, importc: "cpSpaceSetCollisionPersistence".}
## User definable data pointer.
## Generally this points to your game's controller or game state
## class so you can access it when given a Space reference in a callback.

proc userData*(space: Space): DataPointer {.cdecl, importc: "cpSpaceGetUserData".}
proc `userData=`*(space: Space; userData: DataPointer) {.cdecl, importc: "cpSpaceSetUserData".}
## The Space provided static body for a given Space.
## This is merely provided for convenience and you are not required to use it.

proc staticBody*(space: Space): Body {.cdecl, importc: "cpSpaceGetStaticBody".}
## Returns the current (or most recent) time step used with the given space.
## Useful from callbacks if your time step is not a compile-time global.

proc currentTimeStep*(space: Space): Float {.cdecl, importc: "cpSpaceGetCurrentTimeStep".}
## returns true from inside a callback when objects cannot be added/removed.

proc isLocked*(space: Space): bool {.cdecl, importc: "cpSpaceIsLocked".}
## Create or return the existing collision handler that is called for all collisions that are not handled by a more specific collision handler.

proc addDefaultCollisionHandler*(space: Space): ptr CollisionHandler {.cdecl, importc: "cpSpaceAddDefaultCollisionHandler".}
## Create or return the existing collision handler for the specified pair of collision types.
## If wildcard handlers are used with either of the collision types, it's the responibility of the custom handler to invoke the wildcard handlers.

proc addCollisionHandler*(space: Space; a: CollisionType; b: CollisionType): ptr CollisionHandler {.cdecl, importc: "cpSpaceAddCollisionHandler".}
## Create or return the existing wildcard collision handler for the specified type.

proc addWildcardHandler*(space: Space; `type`: CollisionType): ptr CollisionHandler {.cdecl, importc: "cpSpaceAddWildcardHandler".}
## Add a collision shape to the simulation.
## If the shape is attached to a static body, it will be added as a static shape.

proc addShape*(space: Space; shape: Shape): Shape {.cdecl, importc: "cpSpaceAddShape".}
## Add a rigid body to the simulation.

proc addBody*(space: Space; body: Body): Body {.cdecl, importc: "cpSpaceAddBody".}
## Add a constraint to the simulation.

proc addConstraint*(space: Space; constraint: Constraint): Constraint {.cdecl, importc: "cpSpaceAddConstraint".}
## Remove a collision shape from the simulation.

proc removeShape*(space: Space; shape: Shape) {.cdecl, importc: "cpSpaceRemoveShape".}
## Remove a rigid body from the simulation.

proc removeBody*(space: Space; body: Body) {.cdecl, importc: "cpSpaceRemoveBody".}
## Remove a constraint from the simulation.

proc removeConstraint*(space: Space; constraint: Constraint) {.cdecl, importc: "cpSpaceRemoveConstraint".}
## Test if a collision shape has been added to the space.

proc containsShape*(space: Space; shape: Shape): bool {.cdecl, importc: "cpSpaceContainsShape".}
## Test if a rigid body has been added to the space.

proc containsBody*(space: Space; body: Body): bool {.cdecl, importc: "cpSpaceContainsBody".}
## Test if a constraint has been added to the space.

proc containsConstraint*(space: Space; constraint: Constraint): bool {.cdecl, importc: "cpSpaceContainsConstraint".}
## Post Step callback function type.

## Schedule a post-step callback to be called when step() finishes.
## You can only register one callback per unique value for @c key.
## Returns true only if @c key has never been scheduled before.
## It's possible to pass @c NULL for @c func if you only want to mark @c key as being used.

proc addPostStepCallback*(space: Space; `func`: PostStepFunc; key: pointer; data: pointer): bool {.cdecl, importc: "cpSpaceAddPostStepCallback".}
## Nearest point query callback function type.

## Query the space at a point and call @c func for each shape found.

proc pointQuery*(space: Space; point: Vect; maxDistance: Float; filter: ShapeFilter; `func`: SpacePointQueryFunc; data: pointer) {.cdecl, importc: "cpSpacePointQuery".}
## Query the space at a point and return the nearest shape found. Returns NULL if no shapes were found.

proc pointQueryNearest*(space: Space; point: Vect; maxDistance: Float; filter: ShapeFilter; `out`: ptr PointQueryInfo): Shape {.cdecl, importc: "cpSpacePointQueryNearest".}
## Segment query callback function type.

## Perform a directed line segment query (like a raycast) against the space calling @c func for each shape intersected.

proc segmentQuery*(space: Space; start: Vect; `end`: Vect; radius: Float; filter: ShapeFilter; `func`: SpaceSegmentQueryFunc; data: pointer) {.cdecl, importc: "cpSpaceSegmentQuery".}
## Perform a directed line segment query (like a raycast) against the space and return the first shape hit. Returns NULL if no shapes were hit.

proc segmentQueryFirst*(space: Space; start: Vect; `end`: Vect; radius: Float; filter: ShapeFilter; `out`: ptr SegmentQueryInfo): Shape {.cdecl, importc: "cpSpaceSegmentQueryFirst".}
## Rectangle Query callback function type.

## Perform a fast rectangle query on the space calling @c func for each shape found.
## Only the shape's bounding boxes are checked for overlap, not their full shape.

proc bBQuery*(space: Space; bb: BB; filter: ShapeFilter; `func`: SpaceBBQueryFunc; data: pointer) {.cdecl, importc: "cpSpaceBBQuery".}
## Shape query callback function type.

## Query a space for any shapes overlapping the given shape and call @c func for each shape found.

proc shapeQuery*(space: Space; shape: Shape; `func`: SpaceShapeQueryFunc; data: pointer): bool {.cdecl, importc: "cpSpaceShapeQuery".}
## Space/body iterator callback function type.

## Call @c func for each body in the space.

proc eachBody*(space: Space; `func`: SpaceBodyIteratorFunc; data: pointer) {.cdecl, importc: "cpSpaceEachBody".}
## Space/body iterator callback function type.

## Call @c func for each shape in the space.

proc eachShape*(space: Space; `func`: SpaceShapeIteratorFunc; data: pointer) {.cdecl, importc: "cpSpaceEachShape".}
## Space/constraint iterator callback function type.

## Call @c func for each shape in the space.

proc eachConstraint*(space: Space; `func`: SpaceConstraintIteratorFunc; data: pointer) {.cdecl, importc: "cpSpaceEachConstraint".}
## Update the collision detection info for the static shapes in the space.

proc reindexStatic*(space: Space) {.cdecl, importc: "cpSpaceReindexStatic".}
## Update the collision detection data for a specific shape in the space.

proc reindexShape*(space: Space; shape: Shape) {.cdecl, importc: "cpSpaceReindexShape".}
## Update the collision detection data for all shapes attached to a body.

proc reindexShapesForBody*(space: Space; body: Body) {.cdecl, importc: "cpSpaceReindexShapesForBody".}
## Switch the space to use a spatial has as it's spatial index.

proc useSpatialHash*(space: Space; dim: Float; count: cint) {.cdecl, importc: "cpSpaceUseSpatialHash".}
## Step the space forward in time by @c dt.

proc step*(space: Space; dt: Float) {.cdecl, importc: "cpSpaceStep".}
## Color type to use with the space debug drawing API.

## Callback type for a function that draws a filled, stroked circle.

## Callback type for a function that draws a line segment.

## Callback type for a function that draws a thick line segment.

## Callback type for a function that draws a convex polygon.

## Callback type for a function that draws a dot.

## Callback type for a function that returns a color for a given shape. This gives you an opportunity to color shapes based on how they are used in your engine.

## Struct used with debugDraw() containing drawing callbacks and other drawing settings.

## Debug draw the current state of the space using the supplied drawing options.

proc debugDraw*(space: Space; options: ptr SpaceDebugDrawOptions) {.cdecl, importc: "cpSpaceDebugDraw".}
## Version string.

var VersionString* {.importc: "cpVersionString".}: cstring

## Calculate the moment of inertia for a circle.
## @c r1 and @c r2 are the inner and outer diameters. A solid circle has an inner diameter of 0.

proc momentForCircle*(m: Float; r1: Float; r2: Float; offset: Vect): Float {.cdecl, importc: "cpMomentForCircle".}
## Calculate area of a hollow circle.
## @c r1 and @c r2 are the inner and outer diameters. A solid circle has an inner diameter of 0.

proc areaForCircle*(r1: Float; r2: Float): Float {.cdecl, importc: "cpAreaForCircle".}
## Calculate the moment of inertia for a line segment.
## Beveling radius is not supported.

proc momentForSegment*(m: Float; a: Vect; b: Vect; radius: Float): Float {.cdecl, importc: "cpMomentForSegment".}
## Calculate the area of a fattened (capsule shaped) line segment.

proc areaForSegment*(a: Vect; b: Vect; radius: Float): Float {.cdecl, importc: "cpAreaForSegment".}
## Calculate the moment of inertia for a solid polygon shape assuming it's center of gravity is at it's centroid. The offset is added to each vertex.

proc momentForPoly*(m: Float; count: cint; verts: ptr Vect; offset: Vect; radius: Float): Float {.cdecl, importc: "cpMomentForPoly".}
## Calculate the signed area of a polygon. A Clockwise winding gives positive area.
## This is probably backwards from what you expect, but matches Chipmunk's the winding for poly shapes.

proc areaForPoly*(count: cint; verts: ptr Vect; radius: Float): Float {.cdecl, importc: "cpAreaForPoly".}
## Calculate the natural centroid of a polygon.

proc centroidForPoly*(count: cint; verts: ptr Vect): Vect {.cdecl, importc: "cpCentroidForPoly".}
## Calculate the moment of inertia for a solid box.

proc momentForBox*(m: Float; width: Float; height: Float): Float {.cdecl, importc: "cpMomentForBox".}
## Calculate the moment of inertia for a solid box.

proc momentForBox*(m: Float; box: BB): Float {.cdecl, importc: "cpMomentForBox2".}
## Calculate the convex hull of a given set of points. Returns the count of points in the hull.
## @c result must be a pointer to a @c Vect array with at least @c count elements. If @c verts == @c result, then @c verts will be reduced inplace.
## @c first is an optional pointer to an integer to store where the first vertex in the hull came from (i.e. verts[first] == result[0])
## @c tol is the allowed amount to shrink the hull when simplifying it. A tolerance of 0.0 creates an exact hull.

proc convexHull*(count: cint; verts: ptr Vect; result: ptr Vect; first: ptr cint; tol: Float): cint {.cdecl, importc: "cpConvexHull".}
## Convenience macro to work with convexHull.
## @c count and @c verts is the input array passed to convexHull().
## @c count_var and @c verts_var are the names of the variables the macro creates to store the result.
## The output vertex array is allocated on the stack using alloca() so it will be freed automatically, but cannot be returned from the current scope.
## Returns the closest point on the line segment ab, to the point p.

proc closetPointOnSegment*(p: Vect; a: Vect; b: Vect): Vect {.inline, cdecl.} =
  var delta: Vect = vsub(a, b)
  var t: Float = fclamp01(vdot(delta, vsub(p, b)) div
      vlengthsq(delta))
  return vadd(b, vmult(delta, t))



{.pop.}


proc `*`*(v: Vect; s: Float): Vect = vmult(v, s)
proc `+`*(v1, v2: Vect): Vect = vadd(v1, v2)
proc `-`*(v1, v2: Vect): Vect = vsub(v1, v2)
proc `==`*(v1, v2: Vect): bool = veql(v1, v2)
proc `-`*(v: Vect): Vect = vneg(v)
