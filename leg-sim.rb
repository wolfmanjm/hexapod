require 'opengl'
require 'glu'
require 'glut'
include Gl,Glu,Glut

require 'mqtt'


INERTIA_THRESHOLD = 1.0
INERTIA_FACTOR = 0.5
SCALE_FACTOR = 0.01
SCALE_INCREMENT = 0.1
TIMER_FREQUENCY_MILLIS = 20

$rotl = 1 * Math::PI / 180
$last_time = 0

$fXDiff = 0
$fYDiff = 0
$fZDiff = 0
$xLastIncr = 0
$yLastIncr = 0
$fXInertia = -0.5
$fYInertia = 0
$fXInertiaOld = 0
$fYInertiaOld = 0
$fScale = 1.0
$ftime = 0
$xLast = -1
$yLast = -1
$bmModifiers = 0
$rotate = false
$animate = false
$stream_angle= false
$stream_pos= false
$mqttcon= nil
$leg= 0

SQRT2 = Math.sqrt(2.0)
PI2 = Math::PI/2.0
PI4 = Math::PI/4.0
TAU = Math::PI * 2.0

# Lengths of leg segments, in mm
$coxa = 36.5  # between the hip and the knee
$femur = 32.5  # between the knee and the ankle
$tibia = 65.0  # between the ankle and the foot
$home= ($coxa + $femur)

$legx= $home
$legy= 0
$legz= -$tibia

$hip = 0
$knee = 0
$ankle = 0

def _solve_triangle(a, b, c)
  # Calculate an angle of a triangle using the law of cosines.
  a, b, c = a.abs, b.abs, c.abs
  if a + b < c or a + c < b or b + c < a
    raise "Impossible triangle: #{a}, #{b}, #{c}"
  end
  cos = (a ** 2 + b ** 2 - c ** 2) / (2.0 * a * b)
  return Math.acos(cos)
end

def _length(a, b)
  #Calculate the length of a vector.
  return Math.sqrt(a ** 2 + b ** 2)
end

def to_degrees(r)
  r += PI2
  while r > TAU
    r -= TAU
  end
  while r < 0.0
    r += TAU
  end
  r*180.0/Math::PI
end

def _inverse_kinematics(x, y, z)
  # Calculate the 3 angles of the leg, in degrees.
  f = _length(x, y) - $coxa
  d = _length(f, z)
  if d > $femur + $tibia
    raise "d out of range #{d}"
  end

  hip = Math.atan2(y, x)
  knee = _solve_triangle($femur, d, $tibia) - Math.atan2(-z, f)
  ankle = _solve_triangle($femur, $tibia, d) - PI2
  h= to_degrees(hip)
  k= to_degrees(knee)
  a= to_degrees(-ankle)
  print "x= #{x}, y= #{y}, z= #{z}, hip= #{h}, knee= #{k}, ankle= #{a}\n"

  return [h, k, a]
end

def _forward_kinematics(h, k, a)
  # Calculate the x,y,z of the foot given the angles of the leg joints, in degrees.

  # TODO

  print "x= #{x}, y= #{y}, z= #{z}\n"

  return [x, y, z]
end

def init
  $hip, $knee, $ankle =  _inverse_kinematics($legx, $legy, $legz)
  if !ARGV.empty?
    puts "Connecting to MQTT on #{ARGV[0]}"
    $mqttcon= MQTT::Client.connect(ARGV[0])
  end
end

play = lambda do
  this_time = glutGet(GLUT_ELAPSED_TIME)

  $rotl+=(this_time - $last_time) * -0.001
  $last_time = this_time

  glutPostRedisplay()
end

display = Proc.new do
  glLoadIdentity()
  glTranslatef(2.0, 0.0, -8.0)

  glRotatef($fYDiff, 1,0,0)
  glRotatef($fXDiff, 0,1,0)
  glRotatef($fZDiff, 0,0,1)

  glScalef($fScale, $fScale, $fScale)

  glClear(GL_COLOR_BUFFER_BIT)
  glPushMatrix()

  # base
  glTranslate(1.0, 0, 0.0)
  glPushMatrix()
  glScale(2.0, 4.0, 0.5)
  glColor3f(1,1,1)
  glutSolidCube(1.0)
  glPopMatrix()

  # coxa
  glTranslate(-1.0, 0.0, 0.0) # move to corner of base
  glRotate($hip+90, 0.0, 0.0, -1.0)
  glTranslate(1.0, 0.0, 0.0) # rotate around the end

  glPushMatrix()
  glScale(2.0, 0.1, 0.1)
  glColor3f(1,0,0)
  glutSolidCube(1.0)
  glPopMatrix()

  # femur
  fl= 2*0.89
  glTranslate(1.0, 0.0, 0.0)
  glRotate($knee-90, 0.0, -1.0, 0.0)
  glTranslate(fl/2.0, 0.0, 0.0)
  glPushMatrix()
  glScale(fl, 0.1, 0.1)
  glColor3f(0,1,0)
  glutSolidCube(1.0)
  glPopMatrix()

  # tibia
  tl= 2*1.78
  glTranslate(1.0, 0.0, 0.0)
  glRotate(-$ankle, 0.0, -1.0, 0.0)
  glTranslate(tl/2.0, 0.0, 0.0)
  glPushMatrix()
  glScale(tl, 0.1, 0.1)
  glColor3f(0,0,1)
  glutSolidCube(1.0)
  glPopMatrix()

  # foot
  glTranslate(tl/2.0, 0.0, 0.0)
  glPushMatrix()
  glColor3f(0,1,1)
  glutSolidSphere(0.1,50,50)
  glPopMatrix()

  glPopMatrix()
  glFlush()
  glutSwapBuffers()
end

reshape  = Proc.new do |w, h|
  glViewport(0, 0,  w,  h)
  glMatrixMode(GL_PROJECTION)
  glLoadIdentity()
  glOrtho(-4.0, 4.0, -4.0, 4.0, 0, 100)
  #gluPerspective(65.0,  w/h, 1.0, 20.0)
  glMatrixMode(GL_MODELVIEW)
  glLoadIdentity()
  #glTranslate(0.0, 0.0, -20.0)
end

keyboard = Proc.new do|key, x, y|
  case  (key)
  when ?t
    $fXDiff = 0
    $fYDiff = 0
    $fZDiff = 0
    glutPostRedisplay()
  when ?s
    $fXDiff = 0
    $fYDiff = -90
    $fZDiff = 0
    glutPostRedisplay()

  when ?f
    $fXDiff = 0
    $fYDiff = -90
    $fZDiff = 90
    glutPostRedisplay()

  when ?h
    $hip = ($hip + 5) % 360
    puts $hip
    glutPostRedisplay()
  when ?H
    $hip = ($hip - 5) % 360
    puts $hip
    glutPostRedisplay()
  when ?k
    $knee = ($knee + 5) % 360
    puts $knee
    glutPostRedisplay()
  when ?K
    $knee = ($knee - 5) % 360
    puts $knee
    glutPostRedisplay()
  when ?a
    $ankle = ($ankle + 5) % 360
    puts $ankle
    glutPostRedisplay()
  when ?A
    $ankle = ($ankle - 5) % 360
    puts $ankle
    glutPostRedisplay()

  when ?w
    $animate= !$animate

  when ?0, ?1, ?2, ?3, ?4, ?5
    $leg= key.to_i
    puts "Selected leg #{$leg}"

  when ?P
    $stream_angle= false
    $stream_pos= !$stream_pos
    puts "Position streaming #{$stream_pos ? 'enabled' : 'disabled'}"

  when ?R
    $stream_pos= false
    $stream_angle= !$stream_angle
    puts "Angle streaming #{$stream_angle ? 'enabled' : 'disabled'}"

  when ?q, ?\e
    exit(0)
  when ?\s # space
    $rotate = !$rotate

    if ($rotate==false)
      $fXInertiaOld = $fXInertia
      $fYInertiaOld = $fYInertia
    else
      $fXInertia = $fXInertiaOld
      $fYInertia = $fYInertiaOld

      # To prevent confusion, force some rotation
      if ($fXInertia == 0 && $fYInertia == 0)
        $fXInertia = -0.5
      end
    end
  when ?+
    $fScale += SCALE_INCREMENT
  when ?-
    $fScale -= SCALE_INCREMENT
  else
    puts "Keyboard commands:\n"
    puts "t - top view"
    puts "s - side view"
    puts "f - front view"
    puts "P - toggle Position streaming"
    puts "R - toggle Angle Streaming"
    puts "0-5 - select leg"

    puts "q, <esc> - Quit"
    puts "? - Help"

    puts "<home>     - reset zoom and rotation"
    puts "<space> or <click>        - stop rotation"
    puts "<+>, <-> or <ctrl + drag> - zoom model"
    puts "<drag>    - rotate model\n"
    puts "<arrow keys>    - move leg in X or Y\n"
    puts "<pgup> or <pgdown> - move leg in Z\n"
  end
end

motion = lambda do |x,y|
  if ($xLast != -1 || $yLast != -1)
    $xLastIncr = x - $xLast
    $yLastIncr = y - $yLast
    if ($bmModifiers & GLUT_ACTIVE_CTRL != 0)
      if ($xLast != -1)
        $fZDiff += $xLastIncr
        $fScale += $yLastIncr*SCALE_FACTOR
      end
    else
      if ($xLast != -1)
        $fXDiff += $xLastIncr
        $fYDiff += $yLastIncr
      end
    end
  end
  $xLast = x
  $yLast = y
end

mouse = lambda do |button,state,x,y|
  $bmModifiers = glutGetModifiers()
  if (button == GLUT_LEFT_BUTTON)
    if (state == GLUT_UP)
      $xLast = -1
      $yLast = -1
      if $xLastIncr > INERTIA_THRESHOLD
        $fXInertia = ($xLastIncr - INERTIA_THRESHOLD)*INERTIA_FACTOR
      end
      if -$xLastIncr > INERTIA_THRESHOLD
        $fXInertia = ($xLastIncr + INERTIA_THRESHOLD)*INERTIA_FACTOR
      end

      if $yLastIncr > INERTIA_THRESHOLD
        $fYInertia = ($yLastIncr - INERTIA_THRESHOLD)*INERTIA_FACTOR
      end
      if -$yLastIncr > INERTIA_THRESHOLD
        $fYInertia = ($yLastIncr + INERTIA_THRESHOLD)*INERTIA_FACTOR
      end
    else
      $fXInertia = 0
      $fYInertia = 0
    end
    $xLastIncr = 0
    $yLastIncr = 0
  end
end

special = lambda do |key,x,y|
  case key
  when GLUT_KEY_HOME
    $fXDiff = 0
    $fYDiff = 0
    $fZDiff = 0
    $xLastIncr = 0
    $yLastIncr = 0
    $fXInertia = -0.5
    $fYInertia = 0
    $fScale = 1.0
  when GLUT_KEY_LEFT
    $legx -= 1
  when GLUT_KEY_RIGHT
    $legx += 1
  when GLUT_KEY_UP
    $legy -= 1
  when GLUT_KEY_DOWN
    $legy += 1
  when GLUT_KEY_PAGE_UP
    $legz += 1
  when GLUT_KEY_PAGE_DOWN
    $legz -= 1
  end

  $hip, $knee, $ankle =  _inverse_kinematics($legx, $legy, $legz)
  $mqttcon.publish('quadruped/commands', "A #{$leg} #{$hip} #{$knee} #{$ankle}") if $stream_angle and $mqttcon
  $mqttcon.publish('quadruped/commands', "P #{$leg} #{$legx} #{$legy} #{$legz}") if $stream_pos and $mqttcon
end

timer = lambda do |value|
  $ftime += 0.01
  if $rotate
    $fXDiff += $fXInertia
    $fYDiff += $fYInertia
  end

  if $animate
    walk
  end

  glutTimerFunc(TIMER_FREQUENCY_MILLIS , timer, 0)
end

$dirx= 1
def walk
  $legx += $dirx
  if $legx > 30
    $dirx= -5
    $legz= -40
  elsif $legx < -30
    $legz= -$tibia
    $dirx= 1
  end

  $hip, $knee, $ankle =  _inverse_kinematics($legx, $legy, $legz)
end

glutInit
glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB)
glutInitWindowSize(1024, 768)
glutInitWindowPosition(100, 100)
glutCreateWindow($0)
# turn on default lighting
# glEnable(GL_LIGHTING)
# glEnable(GL_LIGHT0)
glClearColor(0.0, 0.0, 0.0, 0.0)
glShadeModel(GL_FLAT)

init()

glutIdleFunc(play)
glutDisplayFunc(display)
glutReshapeFunc(reshape)
glutKeyboardFunc(keyboard)
glutMotionFunc(motion)
glutMouseFunc(mouse)
glutSpecialFunc(special)
glutTimerFunc(TIMER_FREQUENCY_MILLIS , timer, 0)

glutMainLoop()
