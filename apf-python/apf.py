import numpy as np
import matplotlib.pyplot as plt
import math
from matplotlib.animation import FuncAnimation

GNUM = 200  # 地图的边长


def clamp(v, min_value, max_value):
    if v < min_value:
        return min_value
    elif v > max_value:
        return max_value
    return v


def get_dis(x1, x2, y1, y2):
    return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5


class OwnShip(object):
    def __init__(self, lo, la, v, h, ta):
        self.lon = lo
        self.lat = la
        self.speed = v
        self.hdg = h
        self.turn_ang_tol = ta

    def update(self, new_lo, new_la, new_v, new_h):
        self.lon = new_lo
        self.lat = new_la
        self.speed = new_v
        self.hdg = new_h

    def move(self, delta_lo, delta_la):
        self.lon += delta_lo
        self.lat += delta_la

    def turn_to(self, new_h):
        self.hdg = new_h


class TargetShip(object):
    def __init__(self, m, lo, la, pr):
        self.m_msi = m
        self.lon = lo
        self.lat = la
        self.potential_filed_l_range = pr

    def __hash__(self):
        return hash(self.m_msi + str(self.lon) + str(self.lat))

    def __eq__(self, other):
        if self.m_msi == other.m_msi and self.lon == other.lon and self.lat == other.lat:
            return True
        else:
            return False


class Line(object):
    def __init__(self, p1, p2, pr):
        # A、B、C 为直线段的一般方程 Ax + By + C = 0 的系数
        self.p1 = p1
        self.p2 = p2
        self.potential_filed_l_range = pr
        self.A = p2[1] - p1[1]
        self.B = p1[0] - p2[0]
        self.C = p2[0] * p1[1] - p1[0] * p2[1]
        self.len = np.sqrt(self.B * self.B + self.A *
                           self.A)   # line segment length

    def convert_to_point(self, point_list):
        num = int(self.len / 10)
        if self.B == 0:
            step = (self.p2[1] - self.p1[1]) / num
            for i in range(num):
                point_list.append(
                    TargetShip("0000", self.p1[0], self.p1[1] + i * step, 0, 0, self.potential_filed_l_range))
        else:
            step = (self.p2[0] - self.p1[0]) / num
            for i in range(num):
                tmp_x = self.p1[0] + step * i
                point_list.append(TargetShip("0000", tmp_x, -self.A * tmp_x / self.B - self.C / self.B,
                                             self.potential_filed_l_range))
        point_list.append(TargetShip(
            "0000", self.p2[0], self.p2[1], self.potential_filed_l_range))


class Goal(object):
    def __init__(self, lo, la):
        self.lon = lo
        self.lat = la


def find_path(os, goal, obs_list, f_att, f_rep, step_len, iter_num):
    path = []                         # 保存我船走过的每个点的坐标
    hdg = []                          # 经过计算给出的建议航向

    arr_att = np.zeros([GNUM, GNUM])  # 地图每一点的总引力
    arr_rep = np.zeros([GNUM, GNUM])  # 地图每一点的总斥力
    arr_tol = np.zeros([GNUM, GNUM])  # 地图每一点的总合力

    # caculate the repulsive force
    for k in range(len(obs_list)):
        obs = obs_list[k]
        for i in range(GNUM):
            for j in range(GNUM):
                dis = get_dis(i, obs.lon, j, obs.lat)
                if dis < obs.potential_filed_l_range:
                    if dis < 1:
                        dis = 1
                    arr_rep[i][j] += 0.5 * f_rep * \
                        (1 / dis - 1 / obs.potential_filed_l_range) ** 0.5

    # caculate the attractive force and total force
    for i in range(GNUM):
        for j in range(GNUM):
            dis = get_dis(i, goal.lon, j, goal.lat)
            if dis < 10:
                arr_att[i][j] = 0.5 * f_att * dis ** 2
            else:
                arr_att[i][j] = 10 * f_att * dis - 0.5 * f_att * 100
            arr_tol[i][j] = arr_att[i][j] + arr_rep[i][j]

    f_tol = np.gradient(arr_tol)  # 计算合力的梯度  三维数组 [0:x方向梯度, 1:y方向梯度][x][y]

    for i in range(iter_num):
        if get_dis(os.lon, goal.lon, os.lat, goal.lat) < 5:
            break
        path.append([os.lon, os.lat])

        # 合力方向与梯度方向相反
        F_sum = np.array(
            [-f_tol[0][int(os.lon)][int(os.lat)], -f_tol[1][int(os.lon)][int(os.lat)]])
        new_h = math.degrees(math.atan2(F_sum[1], F_sum[0]))  # 计算向量方向的角度

        # 确保其不会在一次迭代中转向角度过大
        if abs(new_h - os.hdg) >= os.turn_ang_tol:
            # new_h = os.hdg + os.turn_ang_tol
            # 根据转向后的势能决定转向方向
            forward_len = 3
            tmp1 = os.speed * step_len * forward_len * np.array([math.cos(math.radians(os.hdg + 30)),
                                                                 math.sin(math.radians(os.hdg + 30))]) + np.array(
                [os.lon, os.lat])
            tmp2 = os.speed * step_len * forward_len * np.array([math.cos(math.radians(os.hdg - 30)),
                                                                 math.sin(math.radians(os.hdg - 30))]) + np.array(
                [os.lon, os.lat])
            tmp1[0] = clamp(int(tmp1[0]), 0, GNUM - 1)
            tmp1[1] = clamp(int(tmp1[1]), 0, GNUM - 1)
            tmp2[0] = clamp(int(tmp2[0]), 0, GNUM - 1)
            tmp2[1] = clamp(int(tmp2[1]), 0, GNUM - 1)
            if arr_tol[int(tmp1[0])][int(tmp1[1])] < arr_tol[int(tmp2[0])][int(tmp2[1])]:
                new_h = os.hdg + os.turn_ang_tol
            else:
                new_h = os.hdg - os.turn_ang_tol

        # 计算我船的下一步位置
        nxt = os.speed * step_len * np.array([math.cos(math.radians(new_h)),
                                              math.sin(math.radians(new_h))])
        os.move(nxt[0], nxt[1])
        hdg.append(new_h)
        os.turn_to(new_h)

    path.append([goal.lon, goal.lat])
    path = np.array(path)
    return path, arr_tol


if __name__ == "__main__":
    f_att = 5     # 引力的增益系数
    f_rep = 20000  # 斥力的增益系数
    step_len = 3     # 步长
    iter_num = 50000  # 最大循环迭代次数

    os = OwnShip(10, 10, 1, 0, 15)
    goal = Goal(180, 180)

    obs_list = [TargetShip("000", 30, 50, 20),
                TargetShip("001", 80, 60, 30),
                TargetShip("010", 150, 160, 30)]

    line_list = [Line([50, 100], [100, 100], 15),
                 Line([50, 100], [90, 80], 15),
                 Line([100, 100], [90, 80], 15)]

    line_num = len(line_list)
    for line in line_list:
        line.convert_to_point(obs_list)

    obs_list = list(set(obs_list))  # 去重

    path, arr_tol = find_path(
        os, goal, obs_list, f_att, f_rep, step_len, iter_num)

    def init():
        plt.axis([0, GNUM, 0, GNUM])
        plt.plot(goal.lon, goal.lat, 'rv')
        plt.plot(path[0, 0], path[0, 1], 'rv')
        x = np.arange(0, GNUM)
        y = np.arange(0, GNUM)
        z = np.transpose(arr_tol)
        plt.pcolormesh(x, y, z)
        for i in range(line_num):
            l = line_list[i]
            plt.plot([l.p1[0], l.p2[0]], [l.p1[1], l.p2[1]],
                     color="r", linewidth=1)

    def update(t):
        # plt.plot(path[t, 0], path[t, 1], 'w.', markersize=0.1)   # 'w.' 表示绘制白色点
        circle = plt.Circle((path[t, 0], path[t, 1]), 0.3, color='w')
        plt.gcf().gca().add_artist(circle)

    fig = plt.figure(1)
    ani = FuncAnimation(fig, update, frames=np.arange(
        0, len(path)), init_func=init,  interval=0,  repeat=False)    # interval 单位：毫秒
    # ani.save('apf_simulate.gif', writer='pillow', fps=30,  dpi=200)
    plt.show()
