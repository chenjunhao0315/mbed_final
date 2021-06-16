import pyb, sensor, image, time, math, tf
enable_lens_corr = False # turn on for straighter lines...
sensor.reset()
sensor.set_pixformat(sensor.RGB565) # grayscale is faster
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time = 2000)
clock = time.clock()

#THRESHOLD = [(44, 100, -21, 63, -6, 127)]
THRESHOLD = [(51, 68, -19, 42, -18, 31)]
#THRESHOLD = [(90, 100, -21, 63, -6, 127)]

uart = pyb.UART(3,9600,timeout_char=1000)
uart.init(9600,bits=8,parity = None, stop=1, timeout_char=1000)

# All lines also have `x1()`, `y1()`, `x2()`, and `y2()` methods to get their end-points
# and a `line()` method to get all the above as one 4 value tuple for `draw_line()`.

f_x = (2.8 / 3.984) * 160 # find_apriltags defaults to this if not set
f_y = (2.8 / 2.952) * 120 # find_apriltags defaults to this if not set
c_x = 160 * 0.5 # find_apriltags defaults to this if not set (the image.w * 0.5)
c_y = 120 * 0.5 # find_apriltags defaults to this if not set (the image.h * 0.5)

def degrees(radians):
   return (180 * radians) / math.pi

scl = 6.52

net = tf.load('person_detection')
labels = ['unsure', 'person', 'no_person']

while(True):
   clock.tick()
   img = sensor.snapshot()
   if enable_lens_corr: img.lens_corr(1.8) # for 2.8mm lens...

   # `merge_distance` controls the merging of nearby lines. At 0 (the default), no
   # merging is done. At 1, any line 1 pixel away from another is merged... and so
   # on as you increase this value. You may wish to merge lines as line segment
   # detection produces a lot of line segment results.

   # `max_theta_diff` controls the maximum amount of rotation difference between
   # any two lines about to be merged. The default setting allows for 15 degrees.

   line = img.get_regression(THRESHOLD, pixels_threshold = 40, roi = [0, 0, img.width(), 50], robust = True, area_threshold = 4)
   #img.draw_rectangle(0, 0, img.width(), 50)
   if line:
       #img.draw_line(line.line(), color = (255, 0, 0))
       info = (line.x1(), line.x2(), line.y1(), line.y2(), line.theta(), line.rho())
       print(line)
       uart.write(("l%3d%3d%3d%3d%4d%4de" % info).encode())

   for tag in img.find_apriltags(fx=f_x, fy=f_y, cx=c_x, cy=c_y): # defaults to TAG36H11
       #img.draw_rectangle(tag.rect(), color = (255, 0, 0))
       #img.draw_cross(tag.cx(), tag.cy(), color = (0, 255, 0))
       print_args = (tag.id(), int(tag.x_translation() * scl), int(tag.y_translation() * scl), int(tag.z_translation() * scl), \
             int(degrees(tag.x_rotation())), int(degrees(tag.y_rotation())), int(degrees(tag.z_rotation())))
       # Translation units are unknown. Rotation units are in degrees.
       print("Id : %3d Tx: %3d, Ty %3d, Tz %3d, Rx %3d, Ry %3d, Rz %3d" % print_args)
       uart.write(("a%3d%4d%4d%4d%3d%3d%3de" % print_args).encode())

   img.to_grayscale()

   for obj in net.classify(img, min_scale=1.0, scale_mul=0.5, x_overlap=0.0, y_overlap=0.0):
       #print(obj.output().index(max(obj.output())))
       print(obj.output())
       if obj.output()[2] > 0.63 or obj.output()[0] > 0.63 or obj.output()[1] < 0.8:
           print("No Person")
           uart.write(("p0").encode())
       else:
           print("Person")
           uart.write(("p1").encode())
       #print("**********\nDetections at [x=%d,y=%d,w=%d,h=%d]" % obj.rect())
       #for i in range(len(obj.output())):
             #print("%s = %f" % (labels[i], obj.output()[i]))
       #img.draw_rectangle(obj.rect())
       #img.draw_string(obj.x()+3, obj.y()-1, labels[obj.output().index(max(obj.output()))], mono_space = False)


   print("FPS %f" % clock.fps())
