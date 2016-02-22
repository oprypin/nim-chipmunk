import 
  chipmunk, 
  csfml, 
  math

const
  Width = 800
  Height = 600

type 
  GameObjPtr = ref object
    circleSprite: csfml.CircleShape
    rectangleSprite: csfml.RectangleShape
    body: chipmunk.Body
    shape: chipmunk.Shape

## Helper functions
proc cp2sfml(v: Vect): Vector2f {.inline.} =
  result.x = v.x
  result.y = v.y

proc vectorToVec2f(a: Vect): Vector2f =
  result.x = a.x
  result.y = a.y

## Global variables
var 
  window = newRenderWindow(
    videoMode(Width, Height, 32), "Chipmunk Test", WindowStyle.Default
  )
  space = newSpace()
  gameobjects: seq[GameObjPtr] = @[]
  event: Event
  oldPos: Vect

window.framerateLimit = 60
space.gravity = Vect(x:8.9, y:82.3)
randomize()

## Set the filters that are used to determine which objects collide and which ones don't: 
## http://chipmunk-physics.net/release/ChipmunkLatest-Docs/#cpShape-Filtering
const
  cBorder = 0b0001.BitMask
  cBall = 0b0010.BitMask
  cBox = 0b0100.BitMask
  cBlueBall = 0b1000.BitMask

let
  FilterBorder = chipmunk.ShapeFilter(
    group: nil,
    categories: cBorder,
    mask: cBorder or cBall or cBox or cBlueBall
  )
  FilterBall = chipmunk.ShapeFilter(
    group:nil,
    categories: cBall,
    mask: cBorder or cBall
  )
  FilterBox = chipmunk.ShapeFilter(
    group:nil,
    categories: cBox,
    mask: cBorder or cBox or cBlueBall
  )
  FilterBlueBall = chipmunk.ShapeFilter(
    group:nil,
    categories: cBlueBall,
    mask: cBorder or cBox or cBlueBall
  )
## Predefined CollisionType single assignment variables
let
  ctBorder = cast[CollisionType](1)
  ctBall = cast[CollisionType](2)
  ctBox = cast[CollisionType](3)
  ctBlueBall = cast[CollisionType](4)

## Collision callback definition
proc ballCallback(a: Arbiter; space: Space; data: pointer): bool {.cdecl.} =
  echo("Inside callback")
  result = true
## Add collision callback only to the blue ball (when it hits the border)
var handler = space.addCollisionHandler(ctBorder, ctBlueBall)
handler.postSolveFunc = cast[CollisionpostSolveFunc](ballCallback)

## Add the borders
var borders: seq[Vect]
borders = @[
  Vect(x:0.0, y:0.0),
  Vect(x:Width, y:0.0),
  Vect(x:Width, y:Height),
  Vect(x:0.0, y:Height)]
var sfBorders = newVertexArray(PrimitiveType.LinesStrip, 4)
for i in 0..3:
  var shape = space.addShape(
    newSegmentShape(
        space.staticBody(), 
        borders[i], 
        borders[(i+1) mod 4], 
        1.0
      )
    )
  sfBorders.getVertex(i).position = borders[i].cp2sfml
  shape.filter = FilterBorder
  shape.collisionType = ctBorder

## Helper functions for creating shapes
proc newBall(mass = 10.0, radius = 10.0): GameObjPtr =
  let pos = Vect(x:100.0, y:30.0)
  new(result)
  result.rectangleSprite = nil
  result.circleSprite = newCircleShape()
  result.circleSprite.radius = radius
  result.circleSprite.origin = Vector2f(x: radius, y: radius)
  result.body = space.addBody(
    newBody(mass, momentForCircle(mass, 0.0, radius, vzero))
  )
  result.body.position = pos
  result.shape = space.addShape(
    newCircleShape(result.body, radius, vzero)
  )
  result.shape.filter = FilterBall
  result.shape.collisionType = ctBall

proc newBox(mass = 10.0, width = 10.0, height = 10.0,
            position = Vect(x:30.0, y:10.0)): GameObjPtr =
  new(result)
  result.circleSprite = nil
  result.rectangleSprite = newRectangleShape()
  result.rectangleSprite.size = Vector2f(x: width, y: height)
  result.rectangleSprite.origin = Vector2f(x: width/2, y: height/2)
  result.rectangleSprite.fillColor = Yellow
  result.body = space.addBody(newBody(mass, momentForBox(mass, width, height)))
  result.body.position = position
  result.shape = space.addShape(newBoxShape(result.body, width, height, radius=0.0))
  result.shape.filter = FilterBox
  result.shape.collisionType = ctBox

## Add the shapes to the space
for i in 0..20:
    gameobjects.add(newBall(50.0, 30.0))
for i in 0..10:
    gameobjects.add(newBox(50.0, 30.0, 30.0, Vect(x:600.0, y:50.0+float(i))))
var ball = newBall(10.0, 15.0)
ball.rectangleSprite = nil
ball.circleSprite.fillColor = Blue
ball.shape.filter = FilterBlueBall
ball.shape.collisionType = ctBlueBall
gameobjects.add(ball)


## The main loop
while window.open():
  while window.pollEvent(event):
    if event.kind == EventType.Closed:
      window.close()
      break
    elif event.kind == EventType.KeyPressed:
      if event.key.code == KeyCode.R:
        oldPos = ball.body.position
        echo("oldPos = ", repr(ball.body.position))
      elif event.key.code == KeyCode.O:
        ball.body.position = oldPos
      elif event.key.code == KeyCode.Escape:
        window.close()
        break
      
  space.step(1.0/60.0)
  window.clear(Black)
  for o in gameobjects: 
    if o.rectangleSprite == nil:
        o.circleSprite.position = o.body.position.vectorToVec2f
        o.circleSprite.rotation = o.body.angle.radToDeg()
        window.draw(o.circleSprite)
    else:
        o.rectangleSprite.position = o.body.position.vectorToVec2f
        o.rectangleSprite.rotation = o.body.angle.radToDeg()
        window.draw(o.rectangleSprite)
  window.draw(sfBorders)
  window.display()

for o in gameobjects:
  o.body.destroy()
  if o.rectangleSprite == nil:
    o.circleSprite.destroy()
  else:
    o.rectangleSprite.destroy()

space.destroy()