require 'mqtt'
require 'ruby-sdl-ffi'
require 'timers'

::SDL.Init( ::SDL::INIT_JOYSTICK )

raise "No SDL joystick available" if  ::SDL.NumJoysticks == 0

$joystick = ::SDL.JoystickOpen(0)

naxis= ::SDL.JoystickNumAxes($joystick)
nhats= ::SDL.JoystickNumHats($joystick)
nbuttons= ::SDL.JoystickNumButtons($joystick)

if !ARGV.empty?
  puts "Connecting to MQTT on #{ARGV[0]}"
  $mqttcon= MQTT::Client.connect(ARGV[0])
end

# Axis 0: X -left +right -32768 - 32768
# Axis 1: Y -forward +back -32768 - 32768
# Axis 2: twist -ccw +cw -32768 - 32768
# Axis 3: slider -forward +back -32768 - 32768
# hat bit0 up, bit1 right, bit2 down, bit3 left
# Button 0: trigger (not working)
# Button 1: thumb back
# Button 2: thumb side top
# Button 3: thunb side bottom
# Button 4: base top left
# Button 5: base top right
# Button 6: base bottom right
# Button 7: base bottom left
$axis_map= {0 => :x, 1 => :y, 2 => :a, 3 => :b, 4 => :c, 5 => :d}

$button_state= [].fill(0, 0, nbuttons)
$axis_state= [].fill(99999, 0, naxis)
$hat_state= [].fill(9999, 0, nhats)

def handle_button(b, v)
  if v != $button_state[b]
    $button_state[b]= v
    $mqttcon.publish('quadruped/commands', "G #{b}") if $mqttcon && v == 1
  end
end

def handle_axis(a, v)
  p= (v/328.0).round # convert to a percentage -100 0 100
  if p != $axis_state[a]
    $axis_state[a]= p
    case $axis_map[a]
    when :x
      $mqttcon.publish('quadruped/commands', "X #{-p}") if $mqttcon

    when :y
      $mqttcon.publish('quadruped/commands', "Y #{-p}") if $mqttcon

    when :a # rotate
      $mqttcon.publish('quadruped/commands', "R #{p}") if $mqttcon

    when :b # speed
      $mqttcon.publish('quadruped/commands', "S #{(-p + 100)/2}") if $mqttcon
    end
  end
end

def handle_hat(h, v)
  if v != $hat_state[h]
    $hat_state[h]= v
    if $mqttcon && (v & 1) != 0 then $mqttcon.publish('quadruped/commands', "U 100") end
    if $mqttcon && (v & 2) != 0 then $mqttcon.publish('quadruped/commands', "L 100") end
    if $mqttcon && (v & 4) != 0 then $mqttcon.publish('quadruped/commands', "U -100") end
    if $mqttcon && (v & 8) != 0 then $mqttcon.publish('quadruped/commands', "L -100") end
  end
end

timers = Timers::Group.new
timers.every(0.1) do

  ::SDL.JoystickUpdate
  naxis.times do |i|
    #puts "axis #{i} #{::SDL.JoystickGetAxis(@joystick, i)}"
    handle_axis(i, ::SDL.JoystickGetAxis($joystick, i))
  end

  nhats.times do |i|
    #puts "hat #{i} #{::SDL.JoystickGetHat($joystick, i)}"
    handle_hat(i, ::SDL.JoystickGetHat($joystick, i))
  end

  nbuttons.times do |i|
    #puts "button #{i} #{::SDL.JoystickGetButton($joystick, i)}"
    handle_button(i, ::SDL.JoystickGetButton($joystick, i))
  end
end

loop { timers.wait }

#MQTT::Client.disconnect() if $mqttcon
