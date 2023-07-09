import PIL
from PIL import Image, ImageDraw
import math
import numpy as np
import os
import glob

import matplotlib.pyplot as plt
import matplotlib.animation as animation

# キャンパスの大きさ
CANVAS_WIDTH = 300
CANVAS_HEIGHT = 300

GROUND = -80

PI = math.pi


class Robo1leg:
    def __init__(self):
        self.theta1 = PI / 8
        self.theta2 = - PI / 8
        self.theta3 = PI / 2
        self.l1 = 50
        self.l2 = 40
        self.l3 = 20

    def draw(self, canvas):
        draw = ImageDraw.Draw(canvas)
        # 本体
        cx0, cy0, cz0 = robo2canvas(0, 0, 0)
        draw.ellipse((cx0 - 10, cy0 - 10, cx0 + 10, cy0 + 10), fill=(0, 0, 0))

        # 足1
        x1 = self.l1 * math.sin(self.theta1)
        z1 = - self.l1 * math.cos(self.theta1)
        cx1, cy1, cz1 = robo2canvas(x1, 0, z1)
        draw.line((cx0, cy0, cx1, cy1), fill=(255, 0, 0), width=3)

        # 関節
        draw.ellipse((cx1 - 5, cy1 - 5, cx1 + 5, cy1 + 5), fill=(0, 0, 0))

        # 足2
        x2 = x1 + self.l2 * math.sin(self.theta2 + self.theta1)
        z2 = z1 - self.l2 * math.cos(self.theta2 + self.theta1)
        cx2, cy2, cz2 = robo2canvas(x2, 0, z2)
        draw.line((cx1, cy1, cx2, cy2), fill=(0, 255, 0), width=3)

        # 関節
        draw.ellipse((cx2 - 5, cy2 - 5, cx2 + 5, cy2 + 5), fill=(0, 0, 0))

        # 足3
        x3 = x2 + self.l3 * math.sin(self.theta3 + self.theta2 + self.theta1)
        z3 = z2 - self.l3 * math.cos(self.theta3 + self.theta2 + self.theta1)
        cx3, cy3, cz3 = robo2canvas(x3, 0, z3)
        draw.line((cx2, cy2, cx3, cy3), fill=(0, 0, 255), width=3)

    def get_leg_pos(self):
        x1 = self.l1 * math.sin(self.theta1)
        z1 = - self.l1 * math.cos(self.theta1)
        x2 = x1 + self.l2 * math.sin(self.theta2 + self.theta1)
        z2 = z1 - self.l2 * math.cos(self.theta2 + self.theta1)
        x3 = x2 + self.l3 * math.sin(self.theta3 + self.theta2 + self.theta1)
        z3 = z2 - self.l3 * math.cos(self.theta3 + self.theta2 + self.theta1)

        return x3, z3

    def inverse_kinematics(self, x, y, z):
        theta1 = math.atan2(
            (self.l1 + self.l2 * math.cos(self.theta2)) * x + self.l2 * math.sin(self.theta2) * z,
            (self.l2 * math.sin(self.theta2) * x - (self.l1 + self.l2 * math.cos(self.theta2)) * z)
        )
        tmp = (x**2 + z**2 - self.l1**2 - self.l2**2) / (2 * self.l1 * self.l2)
        theta2 = math.acos(tmp)
        theta3 = (PI / 2) - theta1 - theta2

        return theta1, theta2, theta3

class World:
    def __init__(self):
        # 画像内でxは右が正、zは上が正、yは手前が正
        self.origin_x = int(CANVAS_WIDTH / 2)
        self.origin_z = int(CANVAS_HEIGHT / 2)
        self.output_dir = 'result'

        self.robo_x = 0
        self.robo_y = 0
        self.robo_z = 0
        self.robo = None

    def add_robo(self, robo):
        self.robo = robo

    def draw(self, time):
        canvas = Image.new('RGB', (CANVAS_WIDTH, CANVAS_HEIGHT), (255, 255, 255))
        self.robo.draw(canvas)

        gx, gy, gz = robo2canvas(0, 0, GROUND)
        draw = ImageDraw.Draw(canvas)
        draw.line((0, gy, CANVAS_WIDTH, gy), fill=(0, 0, 0), width=1)

        file_name = os.path.join(self.output_dir, 'result_{0:03d}.png'.format(time))
        canvas.save(file_name, 'PNG')



def robo2canvas(robo_x, robo_y, robo_z):
    """ロボの座標を画像上のx, yに変換する。"""
    rot_mat = np.array([[1, 0, 0, CANVAS_WIDTH / 2],
                        [0, 0, -1, CANVAS_HEIGHT / 2],
                        [0, 0, 0, 0]])

    pos = rot_mat.dot(np.array([robo_x, robo_y, robo_z, 1]))
    return pos[0], pos[1], pos[2]

def create_animation(output):
    fig = plt.figure()

    pictures = glob.glob("result/*.png")

    ims = []
    for i in range(len(pictures)):
        # 読み込んで付け加えていく
        tmp = Image.open(pictures[i])
        ims.append([plt.imshow(tmp)])

    ani = animation.ArtistAnimation(fig, ims, interval=500)
    ani.save(os.path.join('result', output))


if __name__ == '__main__':
    robo = Robo1leg()
    world = World()
    world.add_robo(robo)

    px0, pz0 = 40, GROUND
    theta1, theta2, theta3 = robo.inverse_kinematics(px0, 0, pz0)
    robo.theta1 = theta1
    robo.theta2 = theta2
    robo.theta3 = theta3

    print("robo pos = ", px0, pz0)
    pxf = - px0
    pzf = pz0
    step = 16

    for t in range(0, step):

        # zを一定にして逆運動学で目標位置を求める
        px = px0 + (pxf - px0) * t / step
        pz = pz0 + (pzf - pz0) * t / step
        print("target pos = ", px, pz)

        theta1, theta2, theta3 = robo.inverse_kinematics(px, 0, pz)
        robo.theta1 = theta1
        robo.theta2 = theta2
        robo.theta3 = theta3
        world.draw(t)

    create_animation('result.gif')
