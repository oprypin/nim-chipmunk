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

var 
  window = newRenderWindow(
    videoMode(Width, Height, 32), "Chipmunk Test", WindowStyle.Default
  )
  space = newSpace()
  gameobjects: seq[GameObjPtr] = @[]

window.framerateLimit = 60
space.gravity = Vect(x:8.9, y:82.3)
randomize()

proc cp2sfml(v: Vect): Vector2f {.inline.} =
  result.x = v.x
  result.y = v.y

let
  ClBorder = 1
  ClBall = (1 or 2)
let
  CtBorder = cast[CollisionType](1)
  CtBall = cast[CollisionType](2)

proc borderballz(a: Arbiter; space: Space; data: pointer): bool {.cdecl.} =
  echo("Borderballz()")
  result = true
var handler = space.addCollisionHandler(CtBorder, CtBall)
handler.postSolveFunc = cast[CollisionpostSolveFunc](borderballz)

var borders: seq[Vect]
borders = @[
  Vect(x:0.0, y:0.0),
  Vect(x:500.0, y:0.0),
  Vect(x:500.0, y:500.0),
  Vect(x:0.0, y:500.0)]
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
  sfBorders[i].position = borders[i].cp2sfml
#  shape.layers = ClBorder
  shape.collisionType = CtBorder


proc vectorToVec2f(a: Vect): Vector2f =
  result.x = a.x
  result.y = a.y

proc floor(a: Vect): Vector2f =
  result.x = a.x.floor
  result.y = a.y.floor

proc newBall(mass = 10.0, radius = 10.0): GameObjPtr =
  let pos = Vect(x:20.0, y:30.0)
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
#  result.shape.setLayers(ClBall)
  result.shape.collisionType = CtBall

proc newBox(mass = 10.0, width = 10.0, height = 10.0,
            position = Vect(x:30.0, y:10.0)): GameObjPtr =
  new(result)
  result.circleSprite = nil
  result.rectangleSprite = newRectangleShape()
  result.rectangleSprite.size = Vector2f(x: width, y: height)
  result.rectangleSprite.origin = Vector2f(x: width/2, y: height/2)
  result.body = space.addBody(newBody(mass, momentForBox(mass, width, height)))
  result.body.position = position
  result.shape = space.addShape(newBoxShape(result.body, width, height, radius=0.0))
#  result.shape.setLayers(ClBall)
  result.shape.collisionType = CtBall

for i in 0..20:
    gameobjects.add(newBall(50.0, 30.0))
for i in 0..10:
    gameobjects.add(newBox(50.0, 30.0, 30.0, Vect(x:400.0, y:50.0+float(i))))
var ball = newBall(10.0, 15.0)
ball.rectangleSprite = nil
ball.circleSprite.fillColor = Blue
gameobjects.add(ball)

var 
  font = newFont("sansation.ttf")
  text = newText()
  event: Event
  clock = newClock()
  oldPos: Vect
text.characterSize = 18
text.font = font
text.strC = "Chipmunk2D TEST" 
var text2 = text.copy()
text2.position = text2.position + Vector2f(x:0.0, y:18.0)
while window.open():
  while window.pollEvent(event):
    if event.kind == EventType.Closed:
      window.close()
      break
    elif event.kind == EventType.KeyPressed:
      if event.key.code == KeyCode.R:
        text.strC = $ball.body.position
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
  window.draw(text)
  window.draw(text2)
  window.draw(sfBorders)
  window.display()

for o in gameobjects:
  o.body.destroy()
  if o.rectangleSprite == nil:
    o.circleSprite.destroy()
  else:
    o.rectangleSprite.destroy()

space.destroy()