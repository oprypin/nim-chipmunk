
import 
  chipmunk, 
  csfml, 
  math

when not defined(csfmlNoDestructors) or not defined(chipmunkNoDestructors):
    const errorMessage = "This example has to be compiled with the following flags: " &
                         "chipmunkNoDestructors, csfmlNoDestructors"
    {.fatal: errorMessage.} 

## Math's randomize
randomize()

const
  gravityStrength = 50.Float
  CTplanet = cast[CollisionType](1)
  CTgravity = cast[CollisionType](2)
  ScreenW = 640
  ScreenH = 480

## Global variables
var 
  space = newSpace()
  window = newRenderWindow(
    videoMode(ScreenW, ScreenH, 32), "Planets demo", WindowStyle.Default
  )
  screenArea = IntRect(left: 20, top: 20, width: ScreenW-20, height: ScreenH-20)
  circleObjects: seq[chipmunk.Shape] = newSeq[chipmunk.Shape]()
  segmentObjects: seq[chipmunk.Shape] = newSeq[chipmunk.Shape]()
  running = true
  event: Event
  clock = newClock()

## Helper procedures
proc floor(vec: Vect): Vector2f =
  result.x = vec.x.floor
  result.y = vec.y.floor

proc cp2sfml(vec: Vect): Vector2f =
  result.x = vec.x
  result.y = vec.y

proc initCircleShape(space: Space; shape: chipmunk.Shape; 
                     userData: pointer = nil): chipmunk.Shape {.discardable.} =
  result = space.addShape(shape)
  shape.userData = csfml.newCircleShape(cast[chipmunk.CircleShape](shape).radius, 30)
  let circleData = cast[csfml.CircleShape](shape.userData)
  circleData.origin = Vector2f(
    x:cast[chipmunk.CircleShape](shape).radius, 
    y:cast[chipmunk.CircleShape](shape).radius
  )
  circleData.fillColor = Green

proc drawCircle(win: RenderWindow, shape: chipmunk.Shape) =
    let circle = cast[csfml.CircleShape](shape.userData)
    circle.position = shape.body.position.floor()
    win.draw(circle)

proc drawSegment(win: RenderWindow, shape: chipmunk.Shape) =
    win.draw(cast[csfml.VertexArray](shape.userData))

proc randomPoint(rect: var IntRect): Vect =
  result.x = (random(rect.width) + rect.left).Float
  result.y = (random(rect.height) + rect.top).Float

proc addPlanet() =
  let
    mass = random(10_000)/10_000*10.0
    radius = mass * 2.0
    gravityRadius = radius * 8.8
    body = space.addBody(newBody(mass, momentForCircle(mass, 0.0, radius, vzero)))
    shape = initCircleShape(space, body.newCircleShape(radius, vzero))
    gravity = initCircleShape(space, body.newCircleShape(gravityRadius, vzero))
    gravityCircle = cast[csfml.CircleShape](gravity.userData)
  body.position = randomPoint(screenArea)
  shape.collisionType = CTplanet
  gravity.sensor = true
  gravity.collisionType = CTgravity
  gravityCircle.fillColor = Transparent
  gravityCircle.outlineColor = Blue
  gravityCircle.outlineThickness = 2.0
  circleObjects.add(shape)
  circleObjects.add(gravity)

## Presolver callback procedure 
## (Pre-Solve > collision happend, but has not been resolved yet)
## https://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#CollisionCallbacks
proc gravityApplicator(arb: Arbiter; space: Space; data: pointer): bool {.cdecl.} =
  var
    bodyA: Body
    bodyB: Body
    dist: Vect
  arb.bodies(addr(bodyA), addr(bodyB))
  dist = bodyA.position - bodyB.position
  bodyB.applyForceAtWorldPoint(
    dist * (1.0 / dist.vnormalize.vlength * gravityStrength),
    vzero
  )


## Startup initialization
window.frameRateLimit = 60
space.iterations = 20
## Add the collision callback to the presolver
var handler = space.addCollisionHandler(CTgravity, CTplanet)
handler.preSolveFunc = cast[CollisionPreSolveFunc](gravityApplicator)

## Add the planets and the borders
block:
  let borders = [Vect(x:0, y:0), Vect(x:0, y:ScreenH),
                 Vect(x:ScreenW, y:ScreenH), Vect(x:ScreenW, y:0)]
  for i in 0..3:
    var newSegment = space.addShape(space.staticBody.newSegmentShape(
      borders[i], borders[(i + 1) mod 4], 16.0)
    )
    newSegment.userData = csfml.newVertexArray(PrimitiveType.Lines, 2)
    let vertexData = cast[VertexArray](newSegment.userData)
    vertexData.getVertex(0).position = cast[SegmentShape](newSegment).a.cp2sfml()
    vertexData.getVertex(1).position = cast[SegmentShape](newSegment).b.cp2sfml()
    vertexData.getVertex(0).color = Blue
    vertexData.getVertex(1).color = Blue
    segmentObjects.add(newSegment)
  for i in 0..29:
    addPlanet()

## Main loop
while running:
  while window.pollEvent(event):
    if event.kind == EventType.Closed:
      running = false
      break
    elif event.kind == EventType.KeyPressed:
      if event.key.code == KeyCode.Escape:
        running = false
        break
        
  let dt = clock.restart.asSeconds / 100
  
  space.step(dt)
  window.clear(Black)
  for obj in circleObjects:
    window.drawCircle(obj)
  for obj in segmentObjects:
    window.drawSegment(obj)
  window.display()

## Cleanup
space.destroy()

