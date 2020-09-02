from pynput.keyboard import Key, Listener
from tkinter.colorchooser import *
from tkinter import *
import numpy as np
import random
import math
import time

sleep_time = 0
gravity = 9.807  # meters (100 pixels) /sec^2
earth_mass = 5.98 * 10 ** 14  # 24 in real life


class Shape:
    def __init__(self, position, points, center_of_mass, mass=1, bounciness=0, friction=0.5,
                 angle=0, color='blue', outline='', static=False):
        self.canvas = canvas
        self.tk = tk
        shapes.append(self)

        self.x, self.y = position
        self.mass = mass
        self.bounciness = bounciness
        self.friction = friction
        self.vel = np.array([0, 0])
        self.acc = np.array([0, 0])
        self.gravity = gravity
        self.cent_m = center_of_mass
        self.ang_mom = 0
        self.em = earth_mass
        self.static = static
        self.center = np.array([0, 0])
        self.contact_points = []
        self.shapes = []
        self.calculated = False
        self.ground = False
        self.touching_static = False
        self.start_angle = angle
        self.angle = 0

        self.mx = None
        self.my = None
        self.prev_mx = None
        self.prev_my = None
        self.mouse = False
        self.selected = False
        self.delete = False
        self.shift_mouse = False
        self.selec_rect = None

        self.color = color
        self.outline = outline
        self.points = np.array(points)
        self.ori_points = self.points
        self.ori_cent_m = self.cent_m
        self.body = self.canvas.create_polygon(self.get_coords(), fill=color, outline=outline)

    def get_coords(self):
        return [tuple(point) for point in (np.array(self.points) + (self.x, self.y))]

    def check_inside_shape(self, point, shape):
        count = 0
        tri_l = self.triangle_list(shape, coords=True)
        for tri in tri_l:
            if self.check_inside_triangle(point, tri):
                count += 1

        return count % 2 == 1

    def check_collision(self):
        points = []
        shapess = []
        self.ground = False
        self.touching_static = False
        direct_ground = False
        for p in self.points:
            if p[0] + self.vel[0] + self.x >= width and not self.mouse:
                points.append(p)
                self.vel[0] *= -self.bounciness
                self.vel[0] -= 0.01
                self.move(width - (p[0] + self.vel[0] + self.x), 0)
            if p[0] + self.vel[0] + self.x <= 0 and not self.mouse:
                points.append(p)
                self.vel[0] *= -self.bounciness
                self.vel[0] += 0.01
                self.move(-(p[0] + self.vel[0] + self.x), 0)
            if p[1] + self.vel[1] + self.y >= height and not self.mouse:
                points.append(p)
                self.vel[1] *= -self.bounciness
                self.vel[0] *= 1 - self.friction
                self.move(0, height - (p[1] + self.vel[1] + self.y))
                self.ground = True
                direct_ground = True

            for s in shapes:
                if s == self:
                    continue

                if s.calculated:
                    shape = s.points + np.array([s.x, s.y])
                else:
                    shape = s.points + s.vel + np.array([s.x, s.y])

                if self.check_inside_shape((p[0] + self.vel[0] + self.x, p[1] + self.vel[1] + self.y), shape):
                    s.contact_points.append(np.array(p) + np.array([self.x, self.y]) + self.vel)
                    s.shapes.append(self)
                    points.append(p)
                    shapess.append(s)

                    b = (self.bounciness + s.bounciness) / 2

                    # Move outside of triangle
                    before_distance = math.sqrt(((self.x + self.cent_m[0]) - (s.x + s.cent_m[0])) ** 2 + (
                                (self.y + self.cent_m[1]) - (s.y + s.cent_m[1])) ** 2)
                    after_distance = math.sqrt(
                        ((self.x + self.cent_m[0] + self.vel[0]) - (s.x + s.vel[0] + s.cent_m[0])) ** 2 + (
                                    (self.y + self.cent_m[1] + self.vel[1]) - (s.y + s.vel[1] + s.cent_m[1])) ** 2)

                    if after_distance <= before_distance or self.ground or s.ground:
                        if self.ground or s.ground and abs(self.vel[0]) + abs(
                                s.vel[0]) < 0.1 or \
                                self.static or s.static or self.touching_static or \
                                self.touching_static or s.touching_static and not self.mouse and not s.mouse:
                            radius = 1
                            _break = False
                            while True:
                                for angle in range(0, 360, 90):
                                    x = math.cos(angle / 360 * math.pi * 2) * radius
                                    y = math.sin(angle / 360 * math.pi * 2) * radius
                                    point = (p[0] + self.vel[0] + x, p[1] + self.vel[1] + y)

                                    if not self.check_inside_shape(point + np.array([self.x, self.y]), shape):
                                        self.vel = self.vel * -b
                                        s.vel = s.vel * -b
                                        if x + y > 1 and not self.ground and not s.ground:
                                            if not self.static and not s.static and not self.touching_static and not s.touching_static:
                                                self.move(x / 2, y / 2)
                                                s.move(-x / 2, -y / 2)
                                            elif not self.static and not self.touching_static:
                                                self.move(x, y)
                                            elif not s.static and not s.touching_static:
                                                s.move(-x, -y)

                                        _break = True
                                        break
                                if _break:
                                    break
                                radius += 1
                                # print(direct_ground)
                                if not direct_ground:
                                    # print('oui')
                                    self.vel = np.array([0, 0])
                                s.vel = np.array([0, 0])
                                if s.ground or self.ground:
                                    self.ground = True
                                    s.ground = True
                                if s.static or self.static or s.touching_static or self.touching_static:
                                    self.touching_static = True
                                    s.touching_static = True

                        else:
                            self.ground = False
                            s.ground = False
                            if not self.static and not s.static:
                                # Conservation of momentum
                                temp = self.vel[0]
                                self.vel[0] = (self.mass - s.mass) / (self.mass + s.mass) * self.vel[
                                    0] + 2 * s.mass / \
                                              (self.mass + s.mass) * s.vel[0]
                                s.vel[0] = 2 * self.mass / (self.mass + s.mass) * temp + (s.mass - self.mass) / \
                                           (self.mass + s.mass) * temp
                                temp = self.vel[1]
                                self.vel[1] = (self.mass - s.mass) / (self.mass + s.mass) * self.vel[
                                    1] + 2 * s.mass / \
                                              (self.mass + s.mass) * s.vel[1]
                                s.vel[1] = 2 * self.mass / (self.mass + s.mass) * temp + (s.mass - self.mass) / \
                                           (self.mass + s.mass) * temp
                            else:
                                pass

        if len(points) > 0:
            return (points, shapess)
        else:
            return ([], [])

    def triangle_list(self, shape, coords=False):
        tri_l = []
        if not coords:
            for i in range(0, len(shape.points) - 1):
                tri_l.append(np.array([shape.points[0], shape.points[i + 1], shape.points[i]]) +
                             np.array([shape.x, shape.y]))
        else:
            for i in range(1, len(shape)-1):
                tri_l.append(np.array((shape[0], shape[i + 1], shape[i])))

        return tri_l

    def check_inside_triangle(self, P, tri):
        A = tri[0]
        B = tri[1]
        C = tri[2]
        s1 = C[1] - A[1]
        s2 = C[0] - A[0]
        s3 = B[1] - A[1]
        s4 = P[1] - A[1]

        w1 = (A[0] * s1 + s4 * s2 - P[0] * s1) / (s3 * s2 - (B[0] - A[0]) * s1)
        w2 = (s4 - w1 * s3) / s1

        return w1 >= 0 and w2 >= 0 and w1 + w2 <= 1

    def move(self, x, y):
        self.canvas.move(self.body, x, y)
        self.x += x
        self.y += y

    def rotate(self):
        self.points = rotate(self.points, self.ang_mom, self.center)
        self.cent_m = rotate(self.cent_m, self.ang_mom, self.center)

        self.canvas.delete(self.body)
        self.body = self.canvas.create_polygon(self.get_coords(), fill=self.color, outline=self.outline)
        self.angle += self.ang_mom

    def applyForce(self, force):
        self.acc = np.add(self.acc, force / self.mass)

    def set(self, x, y, reset=True):
        self.move(x - self.x, y - self.y)
        if reset:
            self.angle = 0
            self.acc = np.array([0, 0])
            self.vel = np.array([0, 0])
            self.points = self.ori_points
            self.cent_m = self.ori_cent_m
            self.center = self.cent_m
            self.ang_mom = self.start_angle
            self.rotate()
            self.ang_mom = 0
            self.canvas.delete(self.body)
            self.body = self.canvas.create_polygon(self.get_coords(), fill=self.color, outline=self.outline)

    def select(self):
        self.selected = True
        if not self.selec_rect and not running:
            self.selec_rect = self.canvas.create_polygon(self.get_coords(), fill='', outline='red')

    def mouse_m(self, event):
        self.mx = event.x
        self.my = event.y

    def mouse_p(self, event):
        self.mx = event.x
        self.my = event.y
        self.prev_mx = self.mx
        self.prev_my = self.my

        in_radius = self.check_inside_shape((event.x, event.y), np.array(self.points) + np.array([self.x, self.y]))
        if in_radius:
            if mouse_state == 'normal' and self.selected or running:
                self.mouse = True
            elif mouse_state == 'normal' and not self.selected and not running:
                self.select()
            return True
        elif mouse_state == 'normal' and self.selected and shift_selection:
            self.shift_mouse = True
        else:
            self.shift_mouse = False
        if not in_radius:
            return False

    def mouse_r(self, event):
        self.mouse = False

    def update(self):
        global moving, x_shift, y_shift, running, shapes

        if self.delete:
            self.canvas.delete(self.body)
            del shapes[shapes.index(self)]
            self.canvas.delete(self.selec_rect)
        else:
            if self.mouse and not running:
                x_shift = self.mx - self.prev_mx
                y_shift = self.my - self.prev_my
                moving = True
                self.move(x_shift, y_shift)
            elif self.shift_mouse and moving:
                self.move(x_shift, y_shift)

            if not running:
                self.set(self.x, self.y, reset=True)
                if self.selected:
                    self.canvas.delete(self.selec_rect)
                    self.selec_rect = self.canvas.create_polygon(self.get_coords(), fill='', outline='red')
            else:
                if not self.static:
                    grav = np.array([0, self.gravity * self.em * self.mass / 637200000 ** 2])
                    self.applyForce(grav)
                    self.acc = np.multiply(self.acc, np.array([0.9, 0.9]))
                    self.ang_mom *= 0.95
                    self.vel = np.add(self.vel, self.acc)


                if self.mouse:
                    x_shift = self.mx - self.prev_mx
                    y_shift = self.my - self.prev_my
                    self.vel = np.array([x_shift, y_shift])

                self.center = self.cent_m
                centers = []
                ang_moms = 0
                cp = self.check_collision()

                if self.mouse and running:
                    cp = (cp[0] + [(self.mx - self.x, self.my - self.y)], cp[1])

                self.speed = math.sqrt(self.vel[0] ** 2 + self.vel[1] ** 2)

                if cp != ([], []) or len(self.contact_points) > 0:
                    if self.mouse:
                        self.speed /= 5

                    points, shapess = cp
                    self.contact_points = [np.array(p) - np.array([self.x, self.y]) for p in self.contact_points]
                    self.contact_points += points
                    self.shapes += shapess
                    for point in self.contact_points:
                        # Calculate angular momentum
                        distance = math.sqrt((point[0] - self.cent_m[0]) ** 2 + (point[1] - self.cent_m[1]) ** 2)

                        ang = distance / 7500 * (self.speed + 1)
                        if point[0] > self.cent_m[0]:
                            self.ang_mom += ang
                            ang_moms += 1
                        else:
                            self.ang_mom -= ang
                            ang_moms -= 1
                        centers.append(point)

                    self.center = (sum(list(map(lambda x: x[0], centers))) / len(centers),
                                   sum(list(map(lambda y: y[1], centers))) / len(centers))

                    if len(self.contact_points) > 1 and abs(ang_moms) != len(self.contact_points) and abs(self.ang_mom) < 0.05:
                        self.ang_mom = 0

                if not self.static:
                    self.move(self.vel[0], self.vel[1])
                    self.rotate()

            self.prev_mx = self.mx
            self.prev_my = self.my
            self.contact_points = []
            self.shapess = []
            self.calculated = True


class start_stop_btn:
    def __init__(self, tk, x, y, image_start, image_stop):
        self.state = False
        self.image_start = image_start
        self.image_stop = image_stop
        self.bt = Button(tk, image=self.image_start, relief=FLAT, cursor='hand2', command=lambda: self.ch_state(),
                         width=60, height=50)
        self.bt.place(x=x, y=y, anchor='center')

    def ch_state(self):
        global running, shapes, canvas

        if not self.state:
            self.bt.config(image=self.image_stop)
            self.state = True
            running = True
            for s in shapes:
                s.start_x = s.x
                s.start_y = s.y
                s.start_angle = s.angle
                try:
                    s.selected = False
                    canvas.delete(s.selec_rect)
                    s.selec_rect = None
                except:
                    pass
        else:
            self.bt.config(image=self.image_start)
            self.state = False
            running = False
            for s in shapes:
                s.set(s.start_x, s.start_y, reset=True)


def mouse_m(event):
    global canvas, start_x, start_y, rect_outline, ov_outline, end_x, end_y, ctr

    if mouse_state == 'normal' or running:
        for b in shapes:
            b.mouse_m(event)
    if (mouse_state == 'cr_ball' or mouse_state == 'cr_wall' or (mouse_state == 'normal' and ctr)) and not running:
        try:
            canvas.delete(rect_outline)
            canvas.delete(ov_outline)
        except:
            pass
        if mouse_state == 'cr_ball':
            if event.x - start_x > 0:
                x_term = 1
            else:
                x_term = -1
            if event.y - start_y > 0:
                y_term = 1
            else:
                y_term = -1
            if abs(event.x - start_x) > abs(event.y - start_y):
                size_x = x_term * abs(event.x - start_x)
                size_y = y_term * abs(event.x - start_x)
            else:
                size_x = x_term * abs(event.y - start_y)
                size_y = y_term * abs(event.y - start_y)
            end_x = start_x + size_x
            end_y = start_y + size_y
            ov_outline = canvas.create_oval(start_x, start_y, end_x, end_y, width=3)
            rect_outline = canvas.create_rectangle(start_x, start_y, end_x, end_y, width=1)
        elif mouse_state == 'cr_wall' or (mouse_state == 'normal' and ctr):
            rect_outline = canvas.create_rectangle(start_x, start_y, event.x, event.y, width=3)


def mouse_p(event):
    global start_x, start_y, shapes, shift_selection, ctr

    mouse1 = False
    mouse2 = False
    for b in shapes:
        if b.check_inside_shape((event.x, event.y), np.array(b.points) + np.array([b.x, b.y])):
            mouse1 = True
            if shift:
                shift_selection = True
    for w in walls:
        if w.x1 < mouse_x < w.x2 and w.y1 < mouse_y < w.y2:
            mouse2 = True
            if shift:
                shift_selection = True
    if not mouse1 and not mouse2 and shift_selection:
        shift_selection = False
    if not mouse1 and not shift and not shift_selection:
        for b in shapes:
            try:
                b.selected = False
                canvas.delete(b.selec_rect)
            except:
                pass
    if not mouse2 and not shift and not shift_selection:
        for w in walls:
            try:
                w.selected = False
                canvas.delete(w.selec_rect)
            except:
                pass
    if mouse_state == 'normal' or running:
        for b in shapes:
            out = b.mouse_p(event)
            if not out and not shift and not shift_selection:
                b.selected = False
                try:
                    canvas.delete(b.selec_rect)
                except:
                    pass
        for w in walls:
            out = w.mouse_p(event)
            if not out and not shift and not shift_selection:
                w.selected = False
                try:
                    canvas.delete(w.selec_rect)
                except:
                    pass
    if (mouse_state == 'cr_wall' or (mouse_state == 'normal' and ctr)) and not running:
        start_x = event.x
        start_y = event.y


def mouse_r(event):
    global canvas, start_x, start_y, end_x, end_y, color, moving, shift_selection, ctr

    moving = False
    ctr = False
    if mouse_state == 'normal':
        for b in shapes:
            tk.config(cursor='arrow')
            b.mouse_r(event)
        # for w in walls:
        #     tk.config(cursor='arrow')
        #     w.mouse_r(event)
    # if mouse_state == 'cr_ball' and not running:
    #     try:
    #         canvas.delete(rect_outline)
    #         canvas.delete(ov_outline)
    #     except:
    #         pass
        # try:
        #     create_ball(canvas, start_x-(start_x-end_x)/2, start_y-(start_y-end_y)/2, start_x-end_x, abs(start_x-end_x), color)
        # except TypeError:
        #     pass
    if (mouse_state == 'cr_wall' or mouse_state == 'normal') and not running:
        try:
            canvas.delete(rect_outline)
        except:
            pass
        if mouse_state == 'normal':
            try:
                for b in shapes:
                    if min([start_x, event.x]) < b.x < max([start_x, event.x]) and min([start_y, event.y]) < b.y < max(
                            [start_y, event.y]):
                        shift_selection = True
                        b.select()
                # for w in walls:
                #     x1, y1, *c = canvas.coords(w.body)
                #     cx = (x1 + c[0]) / 2
                #     cy = (y1 + c[1]) / 2
                #     if min([start_x, event.x]) < cx < max([start_x, event.x]) and min([start_y, event.y]) < cy < max(
                #             [start_y, event.y]):
                #         shift_selection = True
                #         w.select()
            except:
                pass
        # elif mouse_state == 'cr_wall':
        #     try:
        #         create_wall(canvas, start_x, start_y, event.x, event.y, color)
        #     except TypeError:
        #         pass
    start_x, start_y, end_x, end_y = None, None, None, None


# def create_ball(canvas, x, y, s, m, c):
#     global shapes
#
#     b = Ball(canvas, x, y, s, s, mass=m,
#              color=c)
#     shapes.append(b)
# def create_wall(canvas, x1, y1, x2, y2, c):
#     global walls
#
#     w = Wall(canvas, x1, y1, x2, y2, color=c)
#     walls.append(w)
def change_mouse_state(state):
    global mouse_state

    mouse_state = state


def _ask_color():
    global color

    color = askcolor()[1]


def rotate(point, angle, center):
    rot_2d = np.array([[math.cos(angle), -math.sin(angle)],
                       [math.sin(angle), math.cos(angle)]])
    return np.matmul(np.array(point) - np.array(center), rot_2d) + np.array(center)


def on_press(key):
    global shift, ctr

    if key == Key.delete:
        for i, b in enumerate(shapes):
            if b.selected:
                b.delete = True
        for i, w in enumerate(walls):
            if w.selected:
                w.delete = True
    elif key == Key.shift:
        shift = True
    elif key == Key.ctrl_l:
        ctr = True


def on_release(key):
    global shift, ctr

    if key == Key.shift:
        shift = False


tk = Tk()
tk.title('PhysicsSimulator')
canvas = Canvas(tk, width=950, height=650)
canvas.pack()

shapes = []
walls = []

running = False
shift = False
x_shift = None
y_shift = None
moving = False
shift_selection = False
ctr = False
mouse_state = 'normal'
rect_outline = None
ov_outline = None
start_x = None
start_y = None
end_x = None
end_y = None
color = 'blue'

def gen_color():
    COLORS = ['snow', 'ghost white', 'white smoke', 'gainsboro', 'floral white', 'old lace',
        'linen', 'antique white', 'papaya whip', 'blanched almond', 'bisque', 'peach puff',
        'navajo white', 'lemon chiffon', 'mint cream', 'azure', 'alice blue', 'lavender',
        'lavender blush', 'misty rose', 'dark slate gray', 'dim gray', 'slate gray',
        'light slate gray', 'gray', 'light grey', 'midnight blue', 'navy', 'cornflower blue', 'dark slate blue',
        'slate blue', 'medium slate blue', 'light slate blue', 'medium blue', 'royal blue',  'blue',
        'dodger blue', 'deep sky blue', 'sky blue', 'light sky blue', 'steel blue', 'light steel blue',
        'light blue', 'powder blue', 'pale turquoise', 'dark turquoise', 'medium turquoise', 'turquoise',
        'cyan', 'light cyan', 'cadet blue', 'medium aquamarine', 'aquamarine', 'dark green', 'dark olive green',
        'dark sea green', 'sea green', 'medium sea green', 'light sea green', 'pale green', 'spring green',
        'lawn green', 'medium spring green', 'green yellow', 'lime green', 'yellow green',
        'forest green', 'olive drab', 'dark khaki', 'khaki', 'pale goldenrod', 'light goldenrod yellow',
        'light yellow', 'yellow', 'gold', 'light goldenrod', 'goldenrod', 'dark goldenrod', 'rosy brown',
        'indian red', 'saddle brown', 'sandy brown',
        'dark salmon', 'salmon', 'light salmon', 'orange', 'dark orange',
        'coral', 'light coral', 'tomato', 'orange red', 'red', 'hot pink', 'deep pink', 'pink', 'light pink',
        'pale violet red', 'maroon', 'medium violet red', 'violet red',
        'medium orchid', 'dark orchid', 'dark violet', 'blue violet', 'purple', 'medium purple',
        'thistle', 'snow2', 'snow3',
        'snow4', 'seashell2', 'seashell3', 'seashell4', 'AntiqueWhite1', 'AntiqueWhite2',
        'AntiqueWhite3', 'AntiqueWhite4', 'bisque2', 'bisque3', 'bisque4', 'PeachPuff2',
        'PeachPuff3', 'PeachPuff4', 'NavajoWhite2', 'NavajoWhite3', 'NavajoWhite4',
        'LemonChiffon2', 'LemonChiffon3', 'LemonChiffon4', 'cornsilk2', 'cornsilk3',
        'cornsilk4', 'ivory2', 'ivory3', 'ivory4', 'honeydew2', 'honeydew3', 'honeydew4',
        'LavenderBlush2', 'LavenderBlush3', 'LavenderBlush4', 'MistyRose2', 'MistyRose3',
        'MistyRose4', 'azure2', 'azure3', 'azure4', 'SlateBlue1', 'SlateBlue2', 'SlateBlue3',
        'SlateBlue4', 'RoyalBlue1', 'RoyalBlue2', 'RoyalBlue3', 'RoyalBlue4', 'blue2', 'blue4',
        'DodgerBlue2', 'DodgerBlue3', 'DodgerBlue4', 'SteelBlue1', 'SteelBlue2',
        'SteelBlue3', 'SteelBlue4', 'DeepSkyBlue2', 'DeepSkyBlue3', 'DeepSkyBlue4',
        'SkyBlue1', 'SkyBlue2', 'SkyBlue3', 'SkyBlue4', 'LightSkyBlue1', 'LightSkyBlue2',
        'LightSkyBlue3', 'LightSkyBlue4', 'SlateGray1', 'SlateGray2', 'SlateGray3',
        'SlateGray4', 'LightSteelBlue1', 'LightSteelBlue2', 'LightSteelBlue3',
        'LightSteelBlue4', 'LightBlue1', 'LightBlue2', 'LightBlue3', 'LightBlue4',
        'LightCyan2', 'LightCyan3', 'LightCyan4', 'PaleTurquoise1', 'PaleTurquoise2',
        'PaleTurquoise3', 'PaleTurquoise4', 'CadetBlue1', 'CadetBlue2', 'CadetBlue3',
        'CadetBlue4', 'turquoise1', 'turquoise2', 'turquoise3', 'turquoise4', 'cyan2', 'cyan3',
        'cyan4', 'DarkSlateGray1', 'DarkSlateGray2', 'DarkSlateGray3', 'DarkSlateGray4',
        'aquamarine2', 'aquamarine4', 'DarkSeaGreen1', 'DarkSeaGreen2', 'DarkSeaGreen3',
        'DarkSeaGreen4', 'SeaGreen1', 'SeaGreen2', 'SeaGreen3', 'PaleGreen1', 'PaleGreen2',
        'PaleGreen3', 'PaleGreen4', 'SpringGreen2', 'SpringGreen3', 'SpringGreen4',
        'green2', 'green3', 'green4', 'chartreuse2', 'chartreuse3', 'chartreuse4',
        'OliveDrab1', 'OliveDrab2', 'OliveDrab4', 'DarkOliveGreen1', 'DarkOliveGreen2',
        'DarkOliveGreen3', 'DarkOliveGreen4', 'khaki1', 'khaki2', 'khaki3', 'khaki4',
        'LightGoldenrod1', 'LightGoldenrod2', 'LightGoldenrod3', 'LightGoldenrod4',
        'LightYellow2', 'LightYellow3', 'LightYellow4', 'yellow2', 'yellow3', 'yellow4',
        'gold2', 'gold3', 'gold4', 'goldenrod1', 'goldenrod2', 'goldenrod3', 'goldenrod4',
        'DarkGoldenrod1', 'DarkGoldenrod2', 'DarkGoldenrod3', 'DarkGoldenrod4',
        'RosyBrown1', 'RosyBrown2', 'RosyBrown3', 'RosyBrown4', 'IndianRed1', 'IndianRed2',
        'IndianRed3', 'IndianRed4', 'sienna1', 'sienna2', 'sienna3', 'sienna4', 'burlywood1',
        'burlywood2', 'burlywood3', 'burlywood4', 'wheat1', 'wheat2', 'wheat3', 'wheat4', 'tan1',
        'tan2', 'tan4', 'chocolate1', 'chocolate2', 'chocolate3', 'firebrick1', 'firebrick2',
        'firebrick3', 'firebrick4', 'brown1', 'brown2', 'brown3', 'brown4', 'salmon1', 'salmon2',
        'salmon3', 'salmon4', 'LightSalmon2', 'LightSalmon3', 'LightSalmon4', 'orange2',
        'orange3', 'orange4', 'DarkOrange1', 'DarkOrange2', 'DarkOrange3', 'DarkOrange4',
        'coral1', 'coral2', 'coral3', 'coral4', 'tomato2', 'tomato3', 'tomato4', 'OrangeRed2',
        'OrangeRed3', 'OrangeRed4', 'red2', 'red3', 'red4', 'DeepPink2', 'DeepPink3', 'DeepPink4',
        'HotPink1', 'HotPink2', 'HotPink3', 'HotPink4', 'pink1', 'pink2', 'pink3', 'pink4',
        'LightPink1', 'LightPink2', 'LightPink3', 'LightPink4', 'PaleVioletRed1',
        'PaleVioletRed2', 'PaleVioletRed3', 'PaleVioletRed4', 'maroon1', 'maroon2',
        'maroon3', 'maroon4', 'VioletRed1', 'VioletRed2', 'VioletRed3', 'VioletRed4',
        'magenta2', 'magenta3', 'magenta4', 'orchid1', 'orchid2', 'orchid3', 'orchid4', 'plum1',
        'plum2', 'plum3', 'plum4', 'MediumOrchid1', 'MediumOrchid2', 'MediumOrchid3',
        'MediumOrchid4', 'DarkOrchid1', 'DarkOrchid2', 'DarkOrchid3', 'DarkOrchid4',
        'purple1', 'purple2', 'purple3', 'purple4', 'MediumPurple1', 'MediumPurple2',
        'MediumPurple3', 'MediumPurple4', 'thistle1', 'thistle2', 'thistle3', 'thistle4',
        'grey' + str(random.randint(1, 99))]

    return random.choice(COLORS)


shapess = [[[(-25, -14), (0, -25), (0, 0), (20, 0), (2, 25), (-30, 2)], (-7, 1)],
           [[(-20, -20), (-20, 20), (20, 20), (20, -20)], (0, 0)],
           [[(-20, -20), (20, -20), (0, 20)], (0, -6)],
           [[(-30, -10), (30, -10), (30, 10), (-30, 10)], (0, 0)],
           [[(-10, -11), (0, -30), (10, -11), (30, -7), (16, 9), (19, 30), (0, 21), (-19, 30), (-16, 9), (-30, -7)], (0, 0)],
           [[(-35, -15), (35, -15), (25, 15), (-25, 15)], (0, 0)]]

for _ in range(15):
    i = random.randint(0, len(shapess) - 1)
    x = random.randint(50, 900)
    y = random.randint(50, 600)
    Shape((x, y), shapess[i][0], shapess[i][1], mass=5,
          color=gen_color(),
          outline='black', bounciness=random.uniform(0, 0.5), angle=random.randint(0, 360))

Shape((475, 500), [(-70, -35), (50, -20), (50, 25), (-70, 25)], (0, 0), mass=10, color=gen_color(),
               outline='black', static=True)

Shape((100, 150), [(-30, -30), (60, -50), (50, 80), (-40, 70)], (0, 0), mass=10, color=gen_color(),
               outline='black', static=True)

Shape((800, 200), [(0, -50), (50, 0), (25, 50), (-25, 50), (-50, 0)], (0, 0), mass=10, color=gen_color(),
               outline='black', static=True)

ssb = start_stop_btn(tk, 900, 610, PhotoImage(file='Assets/play_btn.gif'), PhotoImage(file='Assets/stop_btn.gif'))
nrml_bt = Button(tk, text='edit', command=lambda: change_mouse_state('normal'), cursor='hand2').place(x=250, y=5)
# cr_ball = Button(tk, text='create ball', command=lambda: change_mouse_state('cr_ball'), cursor='hand2').place(x=335, y=5)
# cr_wall = Button(tk, text='create wall', command=lambda: change_mouse_state('cr_wall'), cursor='hand2').place(x=420, y=5)
# color_bt = Button(text='Select Color', command=_ask_color, cursor='hand2').place(x=20, y=100)

canvas.bind('<B1-Motion>', mouse_m)
canvas.bind('<Button-1>', mouse_p)
canvas.bind('<ButtonRelease-1>', mouse_r)
listener = Listener(on_press=on_press, on_release=on_release)
listener.start()

while True:
    try:
        tk.update()
        width = canvas.winfo_width()
        height = canvas.winfo_height()
        mouse_x = tk.winfo_pointerx() - tk.winfo_rootx()
        mouse_y = tk.winfo_pointery() - tk.winfo_rooty()
        mouse = False
        for b in shapes:
            b.calculated = True
        for b in shapes:
            b.update()
            if b.check_inside_shape((mouse_x, mouse_y), np.array(b.points) + np.array(
                    [b.x, b.y])) and mouse_state == 'normal' and not running:
                if b.selected:
                    tk.config(cursor='fleur')
                else:
                    tk.config(cursor='hand2')
                mouse = True
        if not running and mouse_state == 'cr_ball' or mouse_state == 'cr_wall':
            tk.config(cursor='crosshair')
        elif not mouse and mouse_state == 'normal':
            tk.config(cursor='arrow')
        time.sleep(sleep_time)
    except:
        break
