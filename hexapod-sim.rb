require 'opengl'
require 'glu'
require 'glut'
include Gl,Glu,Glut

require 'mqtt'


INERTIA_THRESHOLD = 1.0
INERTIA_FACTOR = 0.5
SCALE_FACTOR = 0.01
SCALE_INCREMENT = 0.01
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
$fScale = 1.0/100.0
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

# robot base, coordinates of each vertex of the hexagon
$base_scale= 1.0
$basepoly= [
  [-46.75/$base_scale, 80.97/$base_scale, 0.0],
  [-93.5/$base_scale, 0.0/$base_scale, 0.0],
  [-46.75/$base_scale, -80.97/$base_scale, 0.0],
  [46.75/$base_scale, -80.97/$base_scale, 0.0],
  [93.5/$base_scale, 0.0/$base_scale, 0.0],
  [46.75/$base_scale, 80.97/$base_scale, 0.0]
]


# transform for leg position on body
class Numeric
  def degrees
    self * Math::PI / 180.0
  end
end

def transform(angle, x, y)
  tx= x *  Math::cos(angle.degrees) + y * Math::sin(angle.degrees)
  ty= x * -Math::sin(angle.degrees) + y * Math::cos(angle.degrees)
  [tx, ty]
end

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
  #print "x= #{x}, y= #{y}, z= #{z}, hip= #{h}, knee= #{k}, ankle= #{a}\n"

  return [h, k, a]
end

def _forward_kinematics(h, k, a)
  # Calculate the x,y,z of the foot given the angles of the leg joints, in degrees.

  # TODO

  print "x= #{x}, y= #{y}, z= #{z}\n"

  return [x, y, z]
end

#
# Test stuff
#

def do_rotate_leg(l, dr)
  x, y, z= $legs[l][:pos]

  # translate to robot coordinates
  x -= $basepoly[l][0]*$base_scale
  y += $basepoly[l][1]*$base_scale

  tx, ty= transform(dr, x, y)

  tx += $basepoly[l][0]*$base_scale
  ty -= $basepoly[l][1]*$base_scale

  $legs[l][:pos][0]= tx
  $legs[l][:pos][1]= ty
  $legs[l][:pos][2]= z

  rx, ry= transform($legs[l][:rot], tx, ty)
  hip, knee, ankle =  _inverse_kinematics(rx, ry, z)
  $legs[l][:hip]= hip
  $legs[l][:knee]= knee
  $legs[l][:ankle]= ankle
end

$liftleg= -1
$rcnt= 10
def walk
  dr= 0.5
  (0..5).each do |l|
    if l != $liftleg
      do_rotate_leg(l, dr)
    else
      do_rotate_leg(l, -dr*5)
    end
  end
  $rcnt+=1
  if $rcnt > 10
    $legs[$liftleg][:pos][2] -= 20 if($liftleg >= 0)
    $liftleg+=1
    $liftleg= 0 if($liftleg > 5)
    $legs[$liftleg][:pos][2] += 20
    $rcnt= 0
  end
end

#
# display stuff
#

play = lambda do
  this_time = glutGet(GLUT_ELAPSED_TIME)

  $rotl+=(this_time - $last_time) * -0.001
  $last_time = this_time

  glutPostRedisplay()
end

def drawHexagon
  ht= 0.1
  # Draw 3D hexagon.
  glColor3f(1,0,0)
  glBegin (GL_POLYGON)
    $basepoly.each do |v| # Top hexagon
      glVertex3fv(v[0], v[1], v[2]+ht)
    end
  glEnd()

  glColor3f(0,1,0)
  glBegin (GL_POLYGON)
    $basepoly.each do |v| # Bottom hexagon
      glVertex3fv(v[0], v[1], v[2]-ht)
    end
  glEnd()

  glColor3f(1,1,1)
  glBegin (GL_QUADS)
    (0..5).each do |i|
        v = (i + 1) % 6
        glVertex3fv($basepoly[i][0], $basepoly[i][1], $basepoly[i][2]+ht) # bottom
        glVertex3fv($basepoly[i][0], $basepoly[i][1], $basepoly[i][2]-ht) # top
        glVertex3fv($basepoly[v][0], $basepoly[v][1], $basepoly[v][2]-ht) # top
        glVertex3fv($basepoly[v][0], $basepoly[v][1], $basepoly[v][2]+ht) # bottom
    end
  glEnd()
end

def drawleg(n, tx, ty, tz)
  glPushMatrix()
  glTranslate(tx, ty, tz) # move to corner of base
  glRotate($legs[n][:pos_rot], 0,0,1) # and rotate accordingly

  # coxa
  glRotate($legs[n][:hip]+90, 0.0, 0.0, -1.0)
  glTranslate($coxa/2, 0.0, 0.0)
  glPushMatrix()
  glScale(1.0, 0.05, 0.05)
  glColor3f(1,0,0)
  glutSolidCube($coxa)
  glPopMatrix()

  # femur
  glTranslate($coxa/2, 0.0, 0.0)
  glRotate($legs[n][:knee]-90, 0.0, -1.0, 0.0)
  glTranslate($femur/2, 0.0, 0.0)
  glPushMatrix()
  glScale(1.0, 0.05, 0.05)
  glColor3f(0,1,0)
  glutSolidCube($femur)
  glPopMatrix()

  # tibia
  glTranslate($femur/2, 0.0, 0.0)
  glRotate(-$legs[n][:ankle], 0.0, -1.0, 0.0)
  glTranslate($tibia/2, 0.0, 0.0)
  glPushMatrix()
  glScale(1.0, 0.05, 0.05)
  glColor3f(0,0,1)
  glutSolidCube($tibia)
  glPopMatrix()

  # foot
  glTranslate($tibia/2, 0.0, 0.0)
  glPushMatrix()
  glColor3f(0,1,1)
  glutSolidSphere(5,50,50)
  glPopMatrix()

  glPopMatrix()
end

display = Proc.new do
  glLoadIdentity()
  glTranslatef(0.0, 0.0, -10.0)

  glRotatef($fYDiff, 1,0,0)
  glRotatef($fXDiff, 0,1,0)
  glRotatef($fZDiff, 0,0,1)

  glScalef($fScale, $fScale, $fScale)

  #glClear(GL_COLOR_BUFFER_BIT)
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

  glPushMatrix()

  # base
  drawHexagon

  #legs
  (0..$legs.size-1).each do |i|
    drawleg(i, $basepoly[i][0], $basepoly[i][1], 0.0)
  end

  glPopMatrix()
  glFlush()
  glutSwapBuffers()
end

reshape  = Proc.new do |w, h|
  vp = 0.8
  aspect = w.to_f/h.to_f

  glViewport(0, 0, w, h)
  glMatrixMode(GL_PROJECTION)
  glLoadIdentity()

  glFrustum(-vp, vp, -vp / aspect, vp / aspect, 3, 200)

  glMatrixMode(GL_MODELVIEW)
  glLoadIdentity()
  glTranslatef(0.0, 0.0, -20.0)


  # glViewport(0, 0,  w,  h)
  # glMatrixMode(GL_PROJECTION)
  # glLoadIdentity()
  # #glOrtho(-4.0, 4.0, -4.0, 4.0, 0, 100)
  # gluPerspective(65.0,  w/h, 1.0, 100.0)
  # glMatrixMode(GL_MODELVIEW)
  # glLoadIdentity()
  # glTranslate(0.0, 0.0, -20.0)
end

$kmap= {?h => [:hip, 5], ?H => [:hip, -5], ?a => [:ankle, 5], ?A => [:ankle, -5], ?k => [:knee, 5], ?K => [:knee, -5]}
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

  when ?h, ?H, ?k, ?K, ?a, ?A
    k= $kmap[key]
    $legs[$leg][k[0]] += k[1]
    puts $legs[$leg][k[0]]
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

  # when ?R
  #   $stream_pos= false
  #   $stream_angle= !$stream_angle
  #   puts "Angle streaming #{$stream_angle ? 'enabled' : 'disabled'}"

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

  elsif button == 3
      $fScale += (SCALE_INCREMENT/10)
  elsif button == 4
      $fScale -= (SCALE_INCREMENT/10)
  end
end

special = lambda do |key,x,y|
  legx= $legs[$leg][:pos][0]
  legy= $legs[$leg][:pos][1]
  legz= $legs[$leg][:pos][2]
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
    legx -= 1
  when GLUT_KEY_RIGHT
    legx += 1
  when GLUT_KEY_UP
    legy -= 1
  when GLUT_KEY_DOWN
    legy += 1
  when GLUT_KEY_PAGE_UP
    legz += 1
  when GLUT_KEY_PAGE_DOWN
    legz -= 1
  end

  tx, ty= transform($legs[$leg][:rot], legx, legy)

  hip, knee, ankle =  _inverse_kinematics(tx, ty, legz)
  $legs[$leg][:hip]= hip
  $legs[$leg][:knee]= knee
  $legs[$leg][:ankle]= ankle

  $legs[$leg][:pos][0]= legx
  $legs[$leg][:pos][1]= legy
  $legs[$leg][:pos][2]= legz

  # FIXME
  #$mqttcon.publish('quadruped/commands', "A #{$leg} #{$hip} #{$knee} #{$ankle}") if $stream_angle and $mqttcon
  #$mqttcon.publish('quadruped/commands', "P #{$leg} #{$legx} #{$legy} #{$legz}") if $stream_pos and $mqttcon
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


#
# Setup
#

$legs=[
  {pos:[0,0,0], rot: 60, pos_rot: -60, hip: 0, knee: 0, ankle: 0}, # front left
  {pos:[0,0,0], rot: 0, pos_rot: 0, hip: 0, knee: 0, ankle: 0}, # middle left
  {pos:[0,0,0], rot: -60, pos_rot: 60, hip: 0, knee: 0, ankle: 0}, # back left

  {pos:[0,0,0], rot: -120, pos_rot: 120, hip: 0, knee: 0, ankle: 0}, # back right
  {pos:[0,0,0], rot: 180, pos_rot: 180, hip: 0, knee: 0, ankle: 0}, # middle right
  {pos:[0,0,0], rot: 120, pos_rot: -120, hip: 0, knee: 0, ankle: 0}, # front right
]

# setup initial leg positions
# calculate initial angles
$legs.each do |l|
  # home position for each leg
  l[:pos][0], l[:pos][1] = transform(l[:pos_rot], $home, 0)
  l[:pos][2]= -$tibia

  tx, ty= transform(l[:rot], l[:pos][0], l[:pos][1])
  tz= l[:pos][2]
  l[:hip], l[:knee], l[:ankle] =  _inverse_kinematics(tx, ty, tz)
end

if !ARGV.empty?
  puts "Connecting to MQTT on #{ARGV[0]}"
  $mqttcon= MQTT::Client.connect(ARGV[0])
end


glutInit
#glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB)
glutInitDisplayMode( GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE)

glutInitWindowSize(1024, 768)
glutInitWindowPosition(200, 200)
glutCreateWindow($0)
# turn on default lighting
# glEnable(GL_LIGHTING)
# glEnable(GL_LIGHT0)
glClearColor(0.0, 0.0, 0.0, 0.0)
glShadeModel(GL_FLAT)

glutIdleFunc(play)
glutDisplayFunc(display)
glutReshapeFunc(reshape)
glutKeyboardFunc(keyboard)
glutMotionFunc(motion)
glutMouseFunc(mouse)
glutSpecialFunc(special)
glutTimerFunc(TIMER_FREQUENCY_MILLIS , timer, 0)

glDepthFunc(GL_LESS)
glEnable(GL_DEPTH_TEST)


glutMainLoop()
