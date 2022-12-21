import sensor, image, time, math, pyb


#Voor deze robot hebben wij een OpenMV M7 camera gebruikt, twee L298N motor controllers en vier LL130 DC motoren.

timer = pyb.Timer(4, freq=1000)
enaR = timer.channel(1, pyb.Timer.PWM, pin=pyb.Pin('P7'))
enaL = timer.channel(2, pyb.Timer.PWM, pin=pyb.Pin('P8'))
in1R = pyb.Pin('P4', pyb.Pin.OUT_PP, pyb.Pin.PULL_NONE)
in2R = pyb.Pin('P3', pyb.Pin.OUT_PP, pyb.Pin.PULL_NONE)
in1L = pyb.Pin('P2', pyb.Pin.OUT_PP, pyb.Pin.PULL_NONE)
in2L = pyb.Pin('P1', pyb.Pin.OUT_PP, pyb.Pin.PULL_NONE)
#LED = pyb.Pin("LED_BLUE", pyb.Pin.OUT_PP)


timerLED = pyb.Timer(2, freq=1000)
timerLED.counter()
timerLED.freq(4)
timerLED.callback(lambda t: pyb.LED(3).toggle())


def pwmTest():
    for i in range(0, 3):
        in1R(1)
        in2R(0)
        in1L(1)
        in2L(0)

        enaR.pulse_width_percent(25)
        enaL.pulse_width_percent(25)
        pyb.delay(2000)
        enaR.pulse_width_percent(50)
        enaL.pulse_width_percent(50)
        pyb.delay(2000)
        enaR.pulse_width_percent(75)
        enaL.pulse_width_percent(75)
        pyb.delay(2000)
        enaR.pulse_width_percent(100)
        enaL.pulse_width_percent(100)
        pyb.delay(2000)

def forward():
    print('forward')

    in1R(1)
    in2R(0)
    in1L(1)
    in2L(0)

    enaR.pulse_width_percent(80)
    enaL.pulse_width_percent(80)


def stop():
    in1R(0)
    in2R(0)
    in1L(0)
    in2L(0)

    for i in range(0, 8):

        timerLED = pyb.Timer(2, freq=1000)
        timerLED.counter()
        timerLED.freq(8)
        timerLED.callback(lambda t: pyb.LED(1).toggle())


# de robot naar links draaien.
# Voor erg scherpe hoeken kan je tegelijkertijd
# ook de linkerwielen achteruit laten draaien door
# in2L op 1 te zetten. Het tegenovergestelde geldt
# voor scherpe hoeken naar rechts.
def left():
    print('left')
    in1R(1)
    in2R(0)
    in1L(0)
    in2L(0)

    enaL.pulse_width_percent(80)


def right():
    print('right')
    in1R(0)
    in2R(0)
    in1L(1)
    in2L(0)

    enaR.pulse_width_percent(80)


# Dit programma werkt het beste als de camera naar de grond kijkt met een hoek tussen de 45 en 50 graden.



# Nu ingesteld op zwarte lijn. Gebruik [(128, 255)] voor een witte lijn.
GRAYSCALE_THRESHOLD = [(0, 64)]


ROIS = [ # [ROI, weight]
        (0, 100, 160, 20, 0.7), # Deze getallen kunnen aangepast worden voor
        (0,  50, 160, 20, 0.3), # je eigen specifieke robot afhankelijk van
        (0,   0, 160, 20, 0.1)  # hoe die in elkaar zit.
       ]

# Bereken de gewichtsdeler (we berekenen dit zodat je gewichten niet hoeft op te tellen tot 1).
weight_sum = 0
for r in ROIS: weight_sum += r[4] # r[4] is het roi gewicht.

# Camera setup...
sensor.reset() # Initialize the camera sensor.
sensor.set_pixformat(sensor.GRAYSCALE) # use grayscale.
sensor.set_framesize(sensor.QQVGA) # use QQVGA for speed.
sensor.skip_frames(time = 2000) # Let new settings take affect.
sensor.set_auto_gain(False) # must be turned off for color tracking
sensor.set_auto_whitebal(False) # must be turned off for color tracking
clock = time.clock() # Tracks FPS.

while(True):
    clock.tick() # Track elapsed milliseconds between snapshots().
    img = sensor.snapshot() # Take a picture and return the image.

    centroid_sum = 0

    for r in ROIS:
        blobs = img.find_blobs(GRAYSCALE_THRESHOLD, roi=r[0:4], merge=True) # r[0:4] is roi tuple.

        if blobs:
            # Find the blob with the most pixels.
            largest_blob = max(blobs, key=lambda b: b.pixels())

            # Draw a rect around the blob.
            img.draw_rectangle(largest_blob.rect())
            img.draw_cross(largest_blob.cx(),
                           largest_blob.cy())

            centroid_sum += largest_blob.cx() * r[4] # r[4] is the roi weight.

    center_pos = (centroid_sum / weight_sum) # Determine center of line.

    # Convert the center_pos to a deflection angle. We're using a non-linear
    # operation so that the response gets stronger the farther off the line we
    # are. Non-linear operations are good to use on the output of algorithms
    # like this to cause a response "trigger".
    deflection_angleRAD = 0

    # The 80 is from half the X res, the 60 is from half the Y res. The
    # equation below is just computing the angle of a triangle where the
    # opposite side of the triangle is the deviation of the center position
    # from the center and the adjacent side is half the Y res. This limits
    # the angle output to around -45 to 45. (It's not quite -45 and 45).
    deflection_angleRAD = -math.atan((center_pos-80)/60)

    # Convert angle in radians to degrees.
    deflection_angleDEG = math.degrees(deflection_angleRAD)

    # Nu heb je een hoek die geeft hoeveel graden de robot moet draaien om recht voor de lijn te komen
    print("Turn Angle: %f" % deflection_angleDEG)

    print(clock.fps()) # Note: Your OpenMV Cam runs about half as fast while
    # connected to your computer. The FPS should increase once disconnected.


    if -10 <= deflection_angleDEG <= 10:
        forward() #naar voren als de lijn binnen +/- 10 graden zit

    elif 10 <= deflection_angleDEG <= 50:
        left() # naar links bij tussen de 10 en 50 graden

    elif -50 <= deflection_angleDEG <= -10:
        right() # naar rechts
