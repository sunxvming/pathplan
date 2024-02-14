import sys
import numpy as np
import uuid
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
import matplotlib.pyplot as plt


INTERVAL = 100  # unit:ms


class Ship:

    def __init__(self, x, y, v, h, da, df, goal_x=-10, goal_y=-10, dt=0.1, m=1000, t=50, tau=5, maxTOL=10):
        self.id = str(uuid.uuid4())
        self.m = m                          # 质量
        self.default_df = df                         # 默认推进力
        self.command_df = df                         # 推进力指令
        self.command_da = da                         # 艏向角速度指令, 表示施加到物体上的力矩
        self.t = t                          # 转向时间常数, 类似于质量的一个常量
        self.dt = dt                         # 时间步长
        self.vel = v                          # 速度 velocity /vəˈlɒsəti/
        self.tau = tau                        # 加速时间常数
        self.pos = np.array([x, y])           # 位置
        self.heading = np.radians(h)              # 艏向
        self.angular_acc = 0.0                        # 角加速度
        self.acc = 0.0                        # 加速度
        self.history = []                         # 历史位置记录列表
        self.k = 1.0                        # 稳态速度比例系数，初始值为1.0
        self.steady_vel = self.k * self.command_df   # 稳态速度
        self.goal_pos = np.array([goal_x, goal_y])  # 目标点坐标
        self.maxTOL = maxTOL                     # 船舶能够进行最大转向的角度,  Turn to Own Length
        self.adviceCourse = 0.0                        # 建议航向
        self.att_coef = 1                          # 引力的增益系数
        self.rep_coef = 100000                     # 斥力增益系数
        self.ruleCPA = 15                         # 安全范围参考值,Closest Point of Approach
        self.emCPA = 10                         # 紧急避碰范围
        self.f_att = 0.0                        # 引力
        self.f_rep = 0.0                        # 斥力
        self.all_clear = True                       # 周围无碰撞风险
        self.can_change_speed = True
        self.speed_change_time = -10.0
        self.min_dis = -10.0           # 距离他船的最小距离

    def update(self, dt):
        self.steady_vel = self.k * self.command_df
        # 计算加速度和角加速度
        if self.vel < self.steady_vel:  # 当船速小于稳态速度时，计算加速度和角加速度
            # 1000：将command_df从千牛（kN）转换为牛顿（N），因为质量m通常是以千克（kg）表示的
            # a=F/m   因为m=1000kg， 所以推力就等于加速度,1秒内达到稳态速度
            self.acc = self.command_df * 1000 / self.m
            self.angular_acc = self.command_da / self.t
        else:
            # 如果船体速度超过稳态速度，则给船体一个反向的加速度
            self.acc = -(self.vel - self.steady_vel) / self.tau   # 加速度会趋向0
            self.angular_acc = self.command_da / self.t

        # print("self.command_da:", self.command_da)
        # 计算速度和艏向
        self.vel += self.acc * dt     # 1秒内达到稳态速度
        self.heading += self.angular_acc * dt

        # 计算稳态速度和位移
        self.pos += self.vel * \
            np.array([np.cos(self.heading), np.sin(self.heading)]) * dt
        self.history.append(self.pos.copy())  # 添加当前位置到历史位置列表中


class Line:
    def __init__(self, x1, y1, x2, y2):
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2
        self.a = y2 - y1
        self.b = x1 - x2
        self.c = x2 * y1 - x1 * y2
        self.len = np.sqrt(self.b * self.b + self.a * self.a)

    def covert_to_point(self, ships):
        num = int(self.len / 10)
        if self.b == 0:
            step = (self.y1 - self.y1) / num
            for i in range(num):
                ships.append(Ship(self.x1, self.x1 + i * step, 0, 0, 0, 0))
        else:
            step = (self.x2 - self.x1) / num
            for i in range(num):
                tmp_x = self.x1 + i * step
                ships.append(Ship(tmp_x, - self.a * tmp_x /
                             self.b - self.c / self.b, 0, 0, 0, 0))
        ships.append(Ship(self.x2, self.y2, 0, 0, 0, 0))


def get_dis(x1, y1, x2, y2):
    return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5


def trans_angle(angle):
    return np.degrees(((angle + np.pi) % (2 * np.pi)) - np.pi)


class SimulationWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('船舶模拟')
        self.setFixedSize(800, 600)
        self.ships = []
        self.static_obs = []
        self.lines = []
        self.timer = QTimer(self)
        self.current = 0.0     # 当前时间
        self.dt = INTERVAL / 1000  # delta time unit:s
        self.os_vel = []
        self.os_hdg = []
        self.os_min_dis = []

        self.ts_agent = False
        self.speed_change = False

        self.start_button = QPushButton('开始模拟')
        self.start_button.clicked.connect(self.start_simulation)
        self.pause_button = QPushButton('暂停模拟')
        self.pause_button.clicked.connect(self.pause_simulation)
        self.ts_agent_check = QCheckBox("他船代理(避碰规则也作用于他船)")
        self.ts_agent_check.toggled.connect(self.ts_agent_change)
        self.speed_change_check = QCheckBox("速度改变(转角时改变速度)")
        self.speed_change_check.toggled.connect(self.speed_change_change)
        self.ship_button = QPushButton("选择避碰场景", self)
        self.ship_button.clicked.connect(self.select_ship_file)
        self.map_button = QPushButton("选择海图", self)
        self.map_button.clicked.connect(self.select_map_file)
        # 创建画布
        self.canvas = QLabel(self)
        self.canvas.setGeometry(0, 0, 780, 580)
        pixmap = QPixmap(780, 580)
        pixmap.fill(Qt.white)
        self.canvas.setPixmap(pixmap)

        # 布局
        layout = QGridLayout()
        layout.addWidget(self.ts_agent_check, 1, 1)
        layout.addWidget(self.speed_change_check, 2, 1)
        layout.addWidget(self.ship_button, 1, 2)
        layout.addWidget(self.map_button, 2, 2)
        layout.addWidget(self.start_button, 1, 3)
        layout.addWidget(self.pause_button, 2, 3)
        layout.addWidget(self.canvas, 3, 0, 3, 5)
        self.setLayout(layout)

    def select_ship_file(self):
        file_dialog = QFileDialog()
        file_path, _ = file_dialog.getOpenFileName(
            self, "选择场景文件", "", "All Files (*);;Text Files (*.txt)")
        if file_path:
            with open(file_path, 'r') as file:
                lines = file.readlines()
                for j in range(1, len(lines)):
                    values = lines[j].strip().split(',')
                    x = float(values[0])
                    y = float(values[1])
                    v = float(values[2])
                    h = float(values[3])
                    da = float(values[4])
                    df = float(values[5])
                    goal_x = float(values[6])
                    goal_y = float(values[7])

                    ship = Ship(x, y, v, h, da, df, goal_x, goal_y)
                    if j == 1:
                        self.ownShip = ship
                    else:
                        self.ships.append(ship)

    def select_map_file(self):
        file_dialog = QFileDialog()
        file_path, _ = file_dialog.getOpenFileName(
            self, "选择海图文件", "", "All Files (*);;Text Files (*.txt)")
        if file_path:
            with open(file_path, 'r') as file:
                lines = file.readlines()
                for j in range(1, len(lines)):
                    values = lines[j].strip().split(',')
                    x1 = float(values[0])
                    y1 = float(values[1])
                    x2 = float(values[2])
                    y2 = float(values[3])
                    self.lines.append(Line(x1, y1, x2, y2))
            for line in self.lines:
                line.covert_to_point(self.static_obs)

    def ts_agent_change(self):
        self.ts_agent = not self.ts_agent
        print("他船代理：", self.ts_agent)

    def speed_change_change(self):
        self.speed_change = not self.speed_change
        print("速度变化：", self.speed_change)

    def start_simulation(self):
        # 开始模拟
        self.os_vel.append(self.ownShip.vel)
        self.os_hdg.append(np.degrees(self.ownShip.heading))
        self.timer.timeout.connect(self.update_simulation)
        self.timer.start(INTERVAL)  # 每隔INTERVAL ms更新一次

    def pause_simulation(self):
        # 暂停模拟，并显示速度和艏向曲线
        self.timer.stop()
        fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(10, 3))
        ax1.plot(self.os_vel)
        ax1.set_title("Velocity")
        ax1.set_xlabel("time step(0.1s)")
        ax1.set_ylabel("os.spd(/s)")
        ax2.plot(self.os_hdg)
        ax2.set_title('Heading')
        ax2.set_ylim((-180, 180))
        ax2.set_xlabel("time step(0.1s)")
        ax2.set_ylabel("os.hdg(°)")
        ax3.plot(self.os_min_dis)
        ax3.set_title('Minimum Distance')
        ax3.set_xlabel("time step(0.1s)")
        ax3.set_ylabel("os.min_dis")
        plt.tight_layout()
        plt.show()

    def avoid_calculate(self, ownShip):
        goal_dis = get_dis(
            ownShip.pos[0], ownShip.pos[1], ownShip.goal_pos[0], ownShip.goal_pos[1])
        if goal_dis < 2:
            ownShip.command_df = 0
            # ownShip.vel = 0
            return
        # 计算船只当前位置与目标的向量
        goal_vec = np.array(
            [(ownShip.goal_pos[0] - ownShip.pos[0]), (ownShip.goal_pos[1] - ownShip.pos[1])])
        # 计算引力
        if goal_dis < 50:
            ownShip.f_att = ownShip.att_coef * goal_vec
        else:
            ownShip.f_att = ownShip.rep_coef * \
                (1 / goal_dis - 1 / 50) ** 2 * goal_vec

        # 计算斥力
        ownShip.f_rep = np.array([float(0.0), float(0.0)])

        ownShip.all_clear = True
        if ownShip.id != self.ownShip.id:   # 他船
            self.force_calculate(ownShip, self.ownShip)

        for ship in self.ships:
            if ship.id == ownShip.id:  # 排除自己
                continue
            self.force_calculate(ownShip, ship)

        for obs in self.static_obs:
            self.force_calculate(ownShip, obs)

        # 测试是否可以恢复推力
        if self.current - ownShip.speed_change_time > 5:
            ownShip.can_change_speed = True
        else:
            ownShip.can_change_speed = False

        if ownShip.all_clear:  # 周围无碰撞风险
            F_sum = ownShip.f_att
            if ownShip.command_df != ownShip.default_df and ownShip.can_change_speed:
                ownShip.speed_change_time = self.current
                ownShip.command_df = ownShip.default_df    # 恢复推进力
        else:    # 周围有碰撞风险, 采用斥力
            F_sum = ownShip.f_rep
        ownShip.adviceCourse = np.arctan2(F_sum[1], F_sum[0])
        self.rudder_calculate(ownShip)

    def force_calculate(self, ownShip, ship):
        goal_dis = get_dis(
            ownShip.pos[0], ownShip.pos[1], ownShip.goal_pos[0], ownShip.goal_pos[1])
        goal_vec = np.array(
            [(ownShip.goal_pos[0] - ownShip.pos[0]), (ownShip.goal_pos[1] - ownShip.pos[1])])

        ts_dis = get_dis(
            ownShip.pos[0], ownShip.pos[1], ship.pos[0], ship.pos[1])
        if ownShip.min_dis < 0 or ts_dis < ownShip.min_dis:
            ownShip.min_dis = ts_dis
        ts_vec = np.array(
            [(-ownShip.pos[0] + ship.pos[0]) / ts_dis, (-ownShip.pos[1] + ship.pos[1]) / ts_dis])
        # 我船与他船连线角度
        tmp_angle = np.arctan2(ts_vec[1], ts_vec[0])
        # 我船他船艏向夹角
        tmp_delta_hdg = trans_angle(ownShip.heading - ship.heading)
        # 他船艏向与我船他船连线向量夹角，用以衡量两船相对位置
        tmp_delta_angle_ts = trans_angle(tmp_angle - ship.heading)
        # 我船艏向与他船我船连线向量夹角，用以衡量两船相对位置
        tmp_delta_angle_os = trans_angle(tmp_angle - ownShip.heading)

        # 计算DCPA和TCPA  CPA(Closest Point of Approach)两船最近交汇点
        # TCPA: 最近会遇时间
        # DCPA: 船舶最近会遇距离
        dx = ownShip.pos[0] - ship.pos[0]
        dy = ownShip.pos[1] - ship.pos[1]
        # x，y方向上的相对速度
        dvx = ownShip.vel * np.cos(ownShip.heading) - \
            ship.vel * np.cos(ship.heading)
        dvy = ownShip.vel * np.sin(ownShip.heading) - \
            ship.vel * np.sin(ship.heading)
        a = dvx * dvx + dvy * dvy
        b = dx * dvx + dy * dvy
        if a == 0:   # 相对速度为0，无碰撞风险
            tcpa = -b / 0.001
        else:
            tcpa = - b / a
        dcpa = np.sqrt((dx + tcpa * dvx) ** 2 + (dy + tcpa * dvy) ** 2)

        # 计算实际安全避碰范围
        tmp_ruleCPA = ownShip.ruleCPA * (
            (abs(np.sin(np.radians(tmp_delta_hdg))) + 1) ** ((ownShip.vel / 10) * (ship.vel / 10)))

        # tmp_ruleCPA = ownShip.ruleCPA
        print("dis:", int(ts_dis), "tcpa:", int(tcpa), "dcpa:",
              int(dcpa), "tmp_ruleCPA:", int(tmp_ruleCPA), "angular_acc:", int(trans_angle(self.ownShip.angular_acc)))

        if ts_dis <= ownShip.emCPA or (dcpa <= tmp_ruleCPA and 5 > tcpa > 0):
            d0 = ownShip.ruleCPA
            # 障碍物的斥力1，方向由障碍物指向我船
            f_rep_abs = abs(ownShip.rep_coef * (1 / ts_dis - 1 / d0)
                            ** 2 * goal_dis ** 2 / ts_dis ** 2)  # 斥力大小
            f_rep_ob1 = -f_rep_abs * ts_vec  # 斥力向量
            # 障碍物的斥力2，方向由我船指向目标点，目的是在到达终点时合力为0，解决局部最优和目标不可达问题
            f_rep_ob2 = 1 / 2 * ownShip.att_coef * \
                (1 / ts_dis - 1 / d0) ** 2 * goal_dis * goal_vec  # 斥力向量
            # 障碍物的斥力3，方向指向我船右侧，使我船做出避碰决策时倾向进行右转, 垂直于船右侧的力
            f_rep_ob3 = 100 * f_rep_abs * np.array(
                [np.cos(ownShip.heading + np.pi / 2), np.sin(ownShip.heading + np.pi / 2)])
            # 障碍物的斥力4，方向指向他船船尾，避免发生船头横越
            f_rep_ob4 = f_rep_abs * np.array(
                [np.cos(-ship.heading), np.sin(-ship.heading)])

            # 改进后的障碍物合斥力计算
            if ts_dis > ownShip.emCPA:
                if ship.vel == 0:
                    print("静态")
                    ownShip.all_clear = False
                    ownShip.f_rep += f_rep_ob1
                    ownShip.f_rep += f_rep_ob2
                elif ship.vel != 0:
                    if abs(tmp_delta_angle_ts) <= 67.5 and abs(tmp_delta_hdg) <= 67.5:
                        print("追越")
                        ownShip.all_clear = False
                        ownShip.f_rep += f_rep_ob1
                        ownShip.f_rep += f_rep_ob2
                    elif abs(tmp_delta_angle_os) <= 67.5 and abs(tmp_delta_hdg) <= 67.5:
                        print("被追越")
                        if not self.ts_agent:
                            ownShip.all_clear = False
                            ownShip.f_rep += f_rep_ob1
                            ownShip.f_rep += f_rep_ob2
                    elif abs(tmp_delta_angle_ts) >= 157.5 and abs(tmp_delta_hdg) >= 157.5:
                        print("对遇")
                        ownShip.all_clear = False
                        ownShip.f_rep += f_rep_ob1
                        ownShip.f_rep += f_rep_ob2
                        ownShip.f_rep += f_rep_ob3
                    elif tmp_delta_angle_os > 0:
                        print("右舷交叉")
                        ownShip.all_clear = False
                        ownShip.f_rep += f_rep_ob1
                        ownShip.f_rep += f_rep_ob2
                        ownShip.f_rep += f_rep_ob3
                    elif tmp_delta_angle_os < 0:
                        print("左舷交叉")
                        if not self.ts_agent:
                            ownShip.all_clear = False
                            ownShip.f_rep += f_rep_ob1
                            ownShip.f_rep += f_rep_ob2
                            ownShip.f_rep += f_rep_ob4
                    else:
                        print("未定义")
                        ownShip.all_clear = False
                        ownShip.f_rep += f_rep_ob1
                        ownShip.f_rep += f_rep_ob2
                        if not self.ts_agent:
                            ownShip.f_rep += f_rep_ob4
            elif ts_dis <= ownShip.emCPA:
                print("紧急")
                ownShip.all_clear = False
                ownShip.f_rep += 100 * f_rep_ob1
                ownShip.f_rep += f_rep_ob2

    def rudder_calculate(self, ownShip):
        max_rudder = 20.0
        min_rudder = 1
        delta = trans_angle(-ownShip.heading + ownShip.adviceCourse)

        if delta >= ownShip.maxTOL:
            if delta >= 90.0 and self.speed_change and ownShip.can_change_speed:
                if ownShip.command_df == ownShip.default_df:
                    ownShip.speed_change_time = self.current
                ownShip.command_df = ownShip.default_df / 10  # 降低推进力
            delta = ownShip.maxTOL
        elif delta <= -ownShip.maxTOL:
            if delta <= -90.0 and self.speed_change and ownShip.can_change_speed:
                if ownShip.command_df == ownShip.default_df:
                    ownShip.speed_change_time = self.current
                ownShip.command_df = ownShip.default_df / 10  # 降低推进力
            delta = -ownShip.maxTOL

        if delta < -min_rudder and abs(delta) < 180:
            rudder = -max_rudder * abs(delta) / ownShip.maxTOL
        elif delta > min_rudder and delta > 270:
            rudder = -max_rudder * abs(-delta + 360) / ownShip.maxTOL
        elif delta < -min_rudder and abs(delta) > 180:
            rudder = max_rudder * abs(delta + 360) / ownShip.maxTOL
        elif abs(delta) < min_rudder:
            rudder = 0
        else:
            rudder = max_rudder * abs(delta) / ownShip.maxTOL

        ownShip.command_da = rudder

    def update_simulation(self):
        # ============== 更新状态 ==================
        self.current += self.dt
        # 本船
        self.avoid_calculate(self.ownShip)
        self.ownShip.update(self.dt)
        # 他船
        for ship in self.ships:
            if ship.command_df > 0:
                if self.ts_agent:
                    self.avoid_calculate(ship)
                ship.update(self.dt)

        # 记录我船数据
        self.os_vel.append(self.ownShip.vel)
        self.os_hdg.append(np.degrees(self.ownShip.heading))
        self.os_min_dis.append(self.ownShip.min_dis)

        self.ownShip.min_dis = -1
        for ship in self.ships:
            ship.min_dis = -1

        # ============== 绘制 ==================
        pixmap = QPixmap(780, 580)
        pixmap.fill(Qt.white)
        painter = QPainter(pixmap)

        # 绘制本船目标点
        pen = QPen(Qt.green, 5)
        pen.setStyle(Qt.SolidLine)
        painter.setPen(pen)
        painter.drawPoint(int(self.ownShip.goal_pos[0]), int(
            self.ownShip.goal_pos[1]))

        # 绘制本船，红色的三角，三角的顶点指向艏向
        pen = QPen(Qt.red, 2)
        pen.setStyle(Qt.SolidLine)
        painter.setPen(pen)
        path = QPainterPath()
        path.moveTo(int(self.ownShip.pos[0]), int(self.ownShip.pos[1]))
        path.lineTo(
            int(self.ownShip.pos[0] - 15 * np.cos(self.ownShip.heading) +
                5 * np.cos(self.ownShip.heading + np.pi / 2)),
            int(self.ownShip.pos[1] - 15 * np.sin(self.ownShip.heading) + 5 * np.sin(self.ownShip.heading + np.pi / 2)))
        path.lineTo(
            int(self.ownShip.pos[0] - 15 * np.cos(self.ownShip.heading) -
                5 * np.cos(self.ownShip.heading + np.pi / 2)),
            int(self.ownShip.pos[1] - 15 * np.sin(self.ownShip.heading) - 5 * np.sin(self.ownShip.heading + np.pi / 2)))
        path.lineTo(int(self.ownShip.pos[0]), int(self.ownShip.pos[1]))
        painter.drawPath(path)

        # 本船绘制起点到终点的虚线
        pen = QPen(Qt.red, 1)
        pen.setStyle(Qt.DotLine)
        painter.setPen(pen)
        painter.drawLine(int(self.ownShip.goal_pos[0]), int(self.ownShip.goal_pos[1]),
                         int(self.ownShip.history[0][0]), int(self.ownShip.history[0][1]))

        # 绘制本船历史轨迹
        if len(self.ownShip.history) > 1:
            pen = QPen(Qt.black, 1)
            pen.setStyle(Qt.DashDotLine)
            painter.setPen(pen)
            for i in range(1, len(self.ownShip.history)):
                p1 = QPoint(
                    int(self.ownShip.history[i - 1][0]), int(self.ownShip.history[i - 1][1]))
                p2 = QPoint(int(self.ownShip.history[i][0]), int(
                    self.ownShip.history[i][1]))
                painter.drawLine(p1, p2)

        # 绘制他船
        for ship in self.ships:
            # 终点
            pen = QPen(Qt.blue, 2)
            pen.setStyle(Qt.SolidLine)
            painter.setPen(pen)
            painter.drawPoint(int(ship.goal_pos[0]), int(ship.goal_pos[1]))

            # 绘制他船，蓝色的三角，三角的顶点指向艏向
            path = QPainterPath()
            path.moveTo(int(ship.pos[0]), int(ship.pos[1]))
            path.lineTo(int(ship.pos[0] - 15 * np.cos(ship.heading) + 5 * np.cos(ship.heading + np.pi / 2)),
                        int(ship.pos[1] - 15 * np.sin(ship.heading) + 5 * np.sin(ship.heading + np.pi / 2)))
            path.lineTo(int(ship.pos[0] - 15 * np.cos(ship.heading) - 5 * np.cos(ship.heading + np.pi / 2)),
                        int(ship.pos[1] - 15 * np.sin(ship.heading) - 5 * np.sin(ship.heading + np.pi / 2)))
            path.lineTo(int(ship.pos[0]), int(ship.pos[1]))
            painter.drawPath(path)

            # 绘制起点到终点的虚线
            pen = QPen(Qt.blue, 1)
            pen.setStyle(Qt.DotLine)
            painter.setPen(pen)
            if ship.vel > 0:
                painter.drawLine(int(ship.goal_pos[0]), int(ship.goal_pos[1]),
                                 int(ship.history[0][0]), int(ship.history[0][1]))

            # 绘制他船历史轨迹
            if len(ship.history) > 1:
                pen = QPen(Qt.gray, 1)
                pen.setStyle(Qt.DashLine)
                painter.setPen(pen)
                for i in range(1, len(ship.history)):
                    p1 = QPoint(
                        int(ship.history[i - 1][0]), int(ship.history[i - 1][1]))
                    p2 = QPoint(int(ship.history[i][0]), int(
                        ship.history[i][1]))
                    painter.drawLine(p1, p2)

        # 绘制障碍物
        for obs in self.static_obs:
            pen = QPen(Qt.blue, 5)
            pen.setStyle(Qt.SolidLine)
            painter.setPen(pen)
            painter.drawPoint(int(obs.pos[0]), int(obs.pos[1]))
        painter.end()

        self.canvas.setPixmap(pixmap)
        self.canvas.update()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    widget = SimulationWidget()
    widget.show()
    sys.exit(app.exec_())
