import strutils
import chipmunk


var gravity = v(0, -100)

var space = newSpace()
space.gravity = gravity

var ground = newSegmentShape(space.staticBody, v(-20, 5), v(20, -5), 0)
ground.friction = 1.0
var discarded = space.addShape(ground)

var radius = 5.0
var mass = 1.0

var moment = momentForCircle(mass, 0, radius, vzero)

var ballBody = space.addBody(newBody(mass, moment))
ballBody.position = v(0, 15)

var ballShape = space.addShape(newCircleShape(ballBody, radius, vzero))
ballShape.friction = 0.7

var timeStep = 1.0/60.0

var time = 0.0
while time < 2:
  var pos = ballBody.position
  var vel = ballBody.velocity
  echo "Time is $#. ballBody is at ($#, $#). Its velocity is ($#, $#)".format(
    time, pos.x, pos.y, vel.x, vel.y
  )

  space.step(timeStep)

  time += timeStep

when defined chipmunkNoDestructors:
  ballShape.destroy()
  ballBody.destroy()
  ground.destroy()
  space.destroy()