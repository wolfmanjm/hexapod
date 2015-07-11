
class MovementGroup < Qt::GroupBox

    slots   'valueChangedX(int)',
            'valueChangedY(int)',
            'valueChangedA(int)',
            'valueChangedSpeed(int)',
            'valueChangedHeight(int)'



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

        @slidery = Qt::Slider.new(Qt::Vertical)
        @slidery.focusPolicy = Qt::StrongFocus
        @slidery.tickPosition = Qt::Slider::TicksBothSides
        @slidery.tickInterval = 10
        @slidery.singleStep = 1
        @slidery.minimum = -100
        @slidery.maximum = 100

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

        ga= ["Wave", "Tripod", "WaveRotate", "TripodRotate"]
        grb= ga.collect do |n|
            Qt::RadioButton.new(n) {
                connect(SIGNAL :clicked) { $me.setGait(n) }
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

        @speed = Qt::SpinBox.new do
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

        @speedset= Qt::FormLayout.new do |l|
            # l.setRowWrapPolicy(Qt::FormLayout::DontWrapRows)
            # l.setFieldGrowthPolicy(Qt::FormLayout::FieldsStayAtSizeHint)
            # l.setFormAlignment(Qt::AlignHCenter | Qt::AlignTop)
            # l.setLabelAlignment(Qt::AlignLeft)
            l.addRow(tr("Speed"), @speed)
            l.addRow(tr("Height"), @height)
        end

        connect(@sliderx, SIGNAL('valueChanged(int)'), self, SLOT('valueChangedX(int)'))
        connect(@slidery, SIGNAL('valueChanged(int)'), self, SLOT('valueChangedY(int)'))
        connect(@dial, SIGNAL('valueChanged(int)'), self, SLOT('valueChangedA(int)'))
        connect(@speed, SIGNAL('valueChanged(int)'), self, SLOT('valueChangedSpeed(int)'))
        connect(@height, SIGNAL('valueChanged(int)'), self, SLOT('valueChangedHeight(int)'))

        @slidersLayout = Qt::VBoxLayout.new do |l|
            l.addWidget(@sliderx)
            l.addWidget(@slidery)
            l.addWidget(@dial)
            l.addLayout(@speedset)
            l.addWidget(gaits)
        end
        setLayout(@slidersLayout)
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

    def valueChangedSpeed(value)
        puts "Speed #{value}"
        @mqttcon.publish('quadruped/commands', "S #{value}") if @mqttcon
    end

    def valueChangedHeight(value)
        puts "Height #{value}"
        @mqttcon.publish('quadruped/commands', "H #{value}") if @mqttcon
    end

end
