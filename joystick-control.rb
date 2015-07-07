require "rjoystick"
require 'mqtt'

#
# ARGV[0] => /dev/input/jsX
#$dev= ARGV[1]
if !ARGV.empty?
  puts "Connecting to MQTT on #{ARGV[0]}"
  $mqttcon= MQTT::Client.connect(ARGV[0])
end

$dev= "/dev/input/js0"
k = Rjoystick::Device.new($dev)
if k.nil?
  raise "No Joystick found at #{$dev}"
end

# puts k.axes
# puts k.buttons
# puts k.name
# puts k.version
# puts k.axes_maps

# Axis 0: X -left +right -32768 - 32768
# Axis 1: Y -forward +back -32768 - 32768
# Axis 2: twist -ccw +cw -32768 - 32768
# Axis 3: slider -forward +back -32768 - 32768
# Axis 4: hat -left +right -32768 or 32768
# Axis 5: hat -up +down -32768 or 32768
# Button 0: trigger (not working)
# Button 1: thumb back
# Button 2: thumb side top
# Button 3: thunb side bottom
# Button 4: base top left
# Button 5: base top right
# Button 6: base bottom right
# Button 7: base bottom left
$axis_map= {0 => :x, 1 => :y, 2 => :a, 3 => :b, 4 => :c, 5 => :d}
running= true
lastv= 0
while running
  e = k.event
  if e.type == Rjoystick::Event::JSBUTTON
    #puts "Button: #{e.number} #{e.value}"

    case e.number
    when 1
      $mqttcon.publish('quadruped/commands', "G 0") if $mqttcon && e.value == 1
    when 2
      $mqttcon.publish('quadruped/commands', "G 1") if $mqttcon && e.value == 1
    when 3
      $mqttcon.publish('quadruped/commands', "G 2") if $mqttcon && e.value == 1
    when 7
      running= false
    end
  end
  if e.type == Rjoystick::Event::JSAXIS
    v= (e.value/328.0).round # convert to a percentage -100 0 100
    #puts "Axis #{$axis_map[e.number]} #{v}"
    case $axis_map[e.number]
    when :x
      $mqttcon.publish('quadruped/commands', "X #{-v}") if $mqttcon

    when :y
      $mqttcon.publish('quadruped/commands', "Y #{-v}") if $mqttcon

    when :a # rotate
      $mqttcon.publish('quadruped/commands', "R #{v}") if $mqttcon

    when :b # speed
      if lastv != v
        $mqttcon.publish('quadruped/commands', "S #{-v}") if $mqttcon
        lastv= v
      end

    when :d # hat up/down
      $mqttcon.publish('quadruped/commands', "U #{-v}") if $mqttcon
    end
  end
end

k.close
#MQTT::Client.disconnect() if $mqttcon
