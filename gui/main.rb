require 'Qt'
require './movement_group.rb'
require 'mqtt'

if !ARGV.empty?
  puts "Connecting to MQTT on #{ARGV[0]}"
  $mqttcon= MQTT::Client.connect(ARGV[0])
  ARGV.shift
end

app = Qt::Application.new(ARGV) do
	Qt::Widget.new do
		self.window_title = 'Hexapod Control'
		resize(800,600)

        movement = MovementGroup.new("Move")
        movement.mqttcon= $mqttcon

        button = Qt::PushButton.new('Quit') do
            connect(SIGNAL :clicked) { Qt::Application.instance.quit }
        end

        self.layout = Qt::VBoxLayout.new do
            addWidget(movement)
            addWidget(button, 0, Qt::AlignRight)
        end

		show
    end

	exec
end






