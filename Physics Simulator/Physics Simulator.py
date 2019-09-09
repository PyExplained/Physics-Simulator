from tkinter import *
import numpy as np
import math
import time

gravity = 9.807 * 100 # pixels/s^2
earth_mass = 5.98 * 10**2 #24

class Molecule:
    def __init__(self, canvas, x, y, w, h, mass=1, color='blue'):
        global gravity, earth_mass

        self.canvas = canvas

        self.x = x
        self.y = y
        self.mass = mass
        # self.vel = np.array([random.uniform(-2, 2), 0])
        self.vel = np.array([0, 0])
        self.acc = np.array([0, 0])
        self.w = w
        self.h = h
        self.gravity = gravity
        self.em = earth_mass
        self.mouse = False
        self.mx = 0
        self.my = 0
        self.calc = False

        self.body = self.canvas.create_oval(x-w/2, y-h/2, x+w/2, y+h/2, fill=color)
    def applyForce(self, force):
        self.acc = np.add(self.acc, force/self.mass)
    def mouse_m(self, event):
        self.mx = event.x
        self.my = event.y
    def mouse_p(self, event):
        if math.sqrt((event.x - self.cx)**2 + (event.y - self.cy)**2) <= 25:
            self.mouse = True
    def mouse_r(self, event):
        self.mouse = False
    def update(self):
        global molecules

        self.x, self.y, *c = self.canvas.coords(self.body)
        self.cx = (self.x + c[0]) / 2
        self.cy = (self.y + c[1]) / 2

        if not self.mouse:
            self.acc = np.array([0, 0])

            grav = np.array([0, self.gravity * self.em * self.mass / (6.38 * 10**6)])
            self.applyForce(grav)
            # drag = np.array([1.2754, 0])
            # self.applyForce(drag)
            self.acc = np.multiply(self.acc, np.array([0.9, 0.9]))

            self.vel = np.add(self.vel, self.acc)
        else:
            self.vel[0] = self.mx - self.cx
            self.vel[1] = self.my - self.cy

        if not self.calc:
            for m in molecules:
                if m == self:
                    continue
                x1, y1, x2, y2 = self.canvas.coords(m.body)
                cx = (x1 + x2) / 2
                cy = (y1 + y2) / 2
                if math.sqrt((cx-self.cx)**2 + (cy-self.cy)**2) <= self.w/2 + m.w/2:
                    temp = self.vel[0]
                    self.vel[0] = (self.mass - m.mass) / (self.mass + m.mass) * self.vel[0] + 2 * m.mass / \
                                  (self.mass + m.mass) * m.vel[0]
                    m.vel[0] = 2 * self.mass / (self.mass + m.mass) * temp + (m.mass - self.mass) / \
                               (self.mass + m.mass) * temp
                    temp = self.vel[1]
                    self.vel[1] = (self.mass - m.mass) / (self.mass + m.mass) * self.vel[1] + 2 * m.mass / \
                                  (self.mass + m.mass) * m.vel[1]
                    m.vel[1] = 2 * self.mass / (self.mass + m.mass) * temp + (m.mass - self.mass) / \
                               (self.mass + m.mass) * temp
                    m.calc = True

        if c[1] + self.vel[1] > self.canvas.winfo_height():
            self.vel[1] *= -0.8
            self.vel[0] *= 0.9
            self.canvas.move(self.body, self.vel[0], self.canvas.winfo_height()-c[1])
        if c[0] + self.vel[0] > self.canvas.winfo_width():
            self.vel[0] *= -0.8
            self.canvas.move(self.body, self.canvas.winfo_width()-c[0], self.vel[1])
        elif self.x + self.vel[0] < 0:
            self.vel[0] *= -0.8
            self.canvas.move(self.body, 0 - self.x, self.vel[1])
        if not c[1] + self.vel[1] > self.canvas.winfo_height() and not c[0] + self.vel[0] > self.canvas.winfo_width() and not self.x + self.vel[0] < 0:
            self.canvas.move(self.body, self.vel[0], self.vel[1])
        self.calc = False

def mouse_m(event):
    for m in molecules:
        m.mouse_m(event)
def mouse_p(event):
    for m in molecules:
        m.mouse_p(event)
def mouse_r(event):
    for m in molecules:
        m.mouse_r(event)

tk = Tk()
canvas = Canvas(tk, width=700, height=700)
canvas.pack()

import random
molecules = []
for _ in range(25):
    s = random.randint(5,15)
    m = Molecule(canvas, random.randint(0, 700), random.randint(0,700), s, s, mass=s/5, color=random.choice(['red','orange','blue','yellow','green']))
    molecules.append(m)

canvas.bind('<B1-Motion>', mouse_m)
canvas.bind('<Button-1>', mouse_p)
canvas.bind('<ButtonRelease-1>', mouse_r)

while True:
    try:
        tk.update()
        for m in molecules:
            m.update()
        time.sleep(0.01)
    except:
        break
