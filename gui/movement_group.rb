
class MovementGroup < Qt::GroupBox

    slots   'valueChangedX(int)',
            'valueChangedY(int)',
            'valueChangedA(int)',
            'valueChangedStride(int)',
            'valueChangedHeight(int)',
            'setServo(int)'



    attr_accessor :mqttcon

    def initialize(title, parent = nil)
        super(title, parent)
        $me= self
        @sliderx = Qt::Slider.new(Qt::Horizontal)
        @sliderx.focusPolicy = Qt::StrongFocus
        @sliderx.tickPosition = Qt::Slider::TicksBothSides
        @sliderx.tickInterval = 10
        @sliderx.singleStep = 1
        @sliderx.minimum = -100
        @sliderx.maximum = 100

        @slidery = Qt::Slider.new(Qt::Horizontal)
        @slidery.focusPolicy = Qt::StrongFocus
        @slidery.tickPosition = Qt::Slider::TicksBothSides
        @slidery.tickInterval = 10
        @slidery.singleStep = 1
        @slidery.minimum = -100
        @slidery.maximum = 100

        @xvalue = Qt::SpinBox.new do |s|
            s.range = -100..100
            s.singleStep = 1
        end

        @yvalue = Qt::SpinBox.new do |s|
            s.range = -100..100
            s.singleStep = 1
        end

        @dial = Qt::Dial.new {
            setFocusPolicy(Qt::StrongFocus)
            setMinimum(-100)
            setMaximum(100)
            setNotchesVisible(true)
            #setNotchSize(4)
            #setNotchTarget(10.0)
            setSingleStep(1)
            setWrapping(false)
        }
        @avalue = Qt::SpinBox.new do |s|
            s.range = -100..100
            s.singleStep = 1
        end

        @xlabel= Qt::Label.new("X")
        @ylabel= Qt::Label.new("Y")
        @alabel= Qt::Label.new("A")

        @ga= {None: 1, Wave: 2, Tripod: 3, TripodRotate: 4, WaveRotate: 5, Home: 7}
        grb= @ga.collect do |n|
            Qt::RadioButton.new(n[0].to_s) {
                connect(SIGNAL :clicked) { $me.setGait(n[1]) }
            }
        end
        grb[0].checked= true

        gaits = Qt::GroupBox.new("Gaits") do
            buttons= Qt::HBoxLayout.new do
                (0...grb.size).each { |i| addWidget(grb[i]) }
                #addStretch(1)
            end
            setLayout(buttons)
        end

        @stride = Qt::SpinBox.new do
            setValue(50)
            setMinimum(0)
            setMaximum(100)
            setSuffix("%")
        end

        @height = Qt::SpinBox.new do
            setValue(0)
            setMinimum(-100)
            setMaximum(100)
            setSuffix("%")
        end

        reset_button = Qt::PushButton.new('Reset') do |w|
            w.connect(SIGNAL :clicked) { @sliderx.value= 0; @slidery.value= 0; @dial.value= 0; }
        end

        @controls= Qt::GridLayout.new do |l|
            l.addWidget(@xlabel, 0, 0); l.addWidget(@sliderx, 0, 1); l.addWidget(@xvalue, 0, 2)
            l.addWidget(@ylabel, 1, 0); l.addWidget(@slidery, 1, 1); l.addWidget(@yvalue, 1, 2)
            l.addWidget(@alabel, 2, 0); l.addWidget(@dial, 2, 1); l.addWidget(@avalue, 2, 2)
            l.addWidget(reset_button, 3, 0)
        end

        @aux= Qt::FormLayout.new do |l|
            l.addRow(tr("Stride"), @stride)
            l.addRow(tr("Height"), @height)
        end

        connect(@sliderx, SIGNAL('valueChanged(int)'), self, SLOT('valueChangedX(int)'))
        connect(@sliderx, SIGNAL('valueChanged(int)'), @xvalue, SLOT('setValue(int)'))
        connect(@slidery, SIGNAL('valueChanged(int)'), self, SLOT('valueChangedY(int)'))
        connect(@slidery, SIGNAL('valueChanged(int)'), @yvalue, SLOT('setValue(int)'))
        connect(@dial, SIGNAL('valueChanged(int)'), self, SLOT('valueChangedA(int)'))
        connect(@dial, SIGNAL('valueChanged(int)'), @avalue, SLOT('setValue(int)'))
        connect(@stride, SIGNAL('valueChanged(int)'), self, SLOT('valueChangedStride(int)'))
        connect(@height, SIGNAL('valueChanged(int)'), self, SLOT('valueChangedHeight(int)'))

        layout= Qt::VBoxLayout.new do |l|
            l.addLayout(@controls)
            l.addLayout(@aux)
            l.addStretch(1)
            l.addWidget(gaits)
            #l.addWidget(setup_direct_moves)
        end
        setLayout(layout)
    end

    # panel to set the angle on a specific servo
    def setup_direct_moves
        servo_buttons= []
        18.times do |i|
           servo_buttons << Qt::CheckBox.new(i.to_s)
        end
        direct_moves = Qt::GroupBox.new("Direct Moves") do
            l= Qt::VBoxLayout.new do
                r= 0
                c= 0
                gl= Qt::GridLayout.new do
                    servo_buttons.each do |w|
                        addWidget(w, r, c)
                        c+=1
                        if c >= 9
                            c= 0
                            r+=1
                        end
                    end
                end
                addLayout(gl)

                angle= Qt::SpinBox.new do
                    setValue(90)
                    setMinimum(0)
                    setMaximum(200)
                    setSuffix("°")
                end
                addWidget(angle)
                connect(angle, SIGNAL('valueChanged(int)'), $me, SLOT('setServo(int)'))
            end

            setLayout(l)
        end

        @servo_buttons= servo_buttons
        direct_moves
    end

    def setServo(a)
        @servo_buttons.each do |b|
            if b.isChecked
                @mqttcon.publish('quadruped/commands', "A #{b.text} #{a}") if @mqttcon
            end
        end
    end

    def setGait(g)
        puts "gait set to #{g}"
        @mqttcon.publish('quadruped/commands', "G #{g}") if @mqttcon
    end

    def valueChangedX(value)
        puts "X #{value}"
        @mqttcon.publish('quadruped/commands', "X #{value}") if @mqttcon
    end

    def valueChangedY(value)
        puts "Y #{value}"
        @mqttcon.publish('quadruped/commands', "Y #{value}") if @mqttcon
    end

    def valueChangedA(value)
        puts "A #{value}"
        @mqttcon.publish('quadruped/commands', "R #{value}") if @mqttcon
    end

    def valueChangedStride(value)
        puts "Stride #{value}"
        @mqttcon.publish('quadruped/commands', "S #{value}") if @mqttcon
    end

    def valueChangedHeight(value)
        puts "Height #{value}"
        @mqttcon.publish('quadruped/commands', "H #{value}") if @mqttcon
    end

end
