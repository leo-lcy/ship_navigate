import math
import random
import sys
import time
from dataclasses import dataclass
from typing import Optional
import json

from PySide6.QtCore import QSize, QTimer
from PySide6.QtGui import QColor
from PySide6.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QScrollArea, QHBoxLayout, QLineEdit, \
    QPushButton, QMessageBox
from pydash import find_index

from adg import build_adg, to_adg_key
from domain import MapConfig, MapReq, x_y_to_index, ShipTask, State, state_to_cell, distance_of_two_points, \
    MapfResult
from ecbs import ECBS
from ui_ship import ShipWidget

cell_size = 20
obstacles_file = '/Users/leo/PycharmProjects/ship_nevigate/obstacles.json'
ship_tasks_file = '/Users/leo/PycharmProjects/ship_nevigate/ship_tasks.json'


def add_widget_with_label(layout, widget, label_text):
    hbox = QHBoxLayout()
    label = QLabel(label_text)
    hbox.addWidget(label)
    hbox.addWidget(widget)
    layout.addLayout(hbox)


@dataclass
class Node:
    state: State
    parent: Optional['Node']
    g: float
    h: float
    f: float


@dataclass
class ShipExePath:
    s2Index: int  # s2 在 path 里的 index
    adgKey: str
    timeStart: int
    timeEnd: int
    startOn: int
    s1: State  # 起始
    s2: State  # 目标
    x: int  # 当前位置坐标
    y: int  # 当前位置坐标
    head: int  # 当前朝向
    rotateDuration: int  # 完成这一步的仿真时间
    moveDuration: int  # 完成这一步的仿真时间
    waitDuration: int  # 完成这一步的仿真时间
    p: float  # 完成度 0 ~ 1
    holding: bool  # 等其他车同步


@dataclass
class ShipPosition:
    x: int
    y: int
    head: int


class MapfUi:
    def __init__(self):
        self.map_config = MapConfig()
        self.map_req = MapReq(self.map_config, set())

        self.ship_colors: dict[str, QColor] = {}
        self.reset_ship_colors()

        self.tasks: dict[str, ShipTask] = {}
        self.target_to_ship: dict[int, str] = {}

        self.plan: Optional[MapfResult] = None

        self.ship_widgets: dict[str, ShipWidget] = {}  # by ship name

        self.adg_nodes: dict[str, list[str]] = {}
        self.finished_adg_nodes: set[str] = set()

        self.simulation = False
        self.sim_speed = 4
        self.current_time = 0
        self.max_time = 0
        self.simulation_start = 0
        self.stepDurationVar = .1
        self.sim_ships: dict[str, ShipExePath] = {}

        # 创建一个垂直布局
        main_layout = QVBoxLayout()
        self.main_layout = main_layout

        # 创建一个滚动区域
        scroll_area = QScrollArea()

        # 创建一个包含大量内容的部件
        content_widget = QWidget()
        content_layout = QVBoxLayout()

        button_layout = QHBoxLayout()

        random_obstacle_btn = QPushButton('扫描地图')
        button_layout.addWidget(random_obstacle_btn)
        random_obstacle_btn.clicked.connect(self.random_obstacles)

        init_targets_btn = QPushButton('确定起点和终点')
        button_layout.addWidget(init_targets_btn)
        init_targets_btn.clicked.connect(self.init_targets)

        resolve_btn = QPushButton('规划路线')
        button_layout.addWidget(resolve_btn)
        resolve_btn.clicked.connect(self.resolve)

        sim_btn = QPushButton('开始仿真')
        self.sim_btn = sim_btn
        button_layout.addWidget(sim_btn)
        sim_btn.clicked.connect(self.toggle_sim)

        content_layout.addLayout(button_layout)

        input_line_1 = QHBoxLayout()

        ship_num_edit = QLineEdit(str(self.map_config.shipNum))
        add_widget_with_label(input_line_1, ship_num_edit, '无人艇数量:')

        ship_num_edit.textChanged.connect(self.update_ship_num)

        map_dim_x_edit = QLineEdit(str(self.map_config.mapDimX))
        add_widget_with_label(input_line_1, map_dim_x_edit, '地图宽度:')

        map_dim_y_edit = QLineEdit(str(self.map_config.mapDimY))
        add_widget_with_label(input_line_1, map_dim_y_edit, '地图高度:')

        content_layout.addLayout(input_line_1, 0)

        input_line_2 = QHBoxLayout()

        w_edit = QLineEdit(str(self.map_config.w))
        add_widget_with_label(input_line_2, w_edit, 'ω:')

        target_num_edit = QLineEdit(str(self.map_config.targetNum))
        add_widget_with_label(input_line_2, target_num_edit, '目标点数目:')

        content_layout.addLayout(input_line_2, 0)

        map_grid = QWidget()
        self.map_grid = map_grid
        map_grid.setFixedSize(QSize(self.map_config.mapDimX * (cell_size + 1),
                                    self.map_config.mapDimY * (cell_size + 1)))

        self.map_cells = []
        self.rebuild_map_cells()

        map_grid_container = QHBoxLayout()
        map_grid_container.addStretch(1)  # 左边弹性空间
        map_grid_container.addWidget(map_grid)  # 中间的地图
        map_grid_container.addStretch(1)  # 右边弹性空间
        content_layout.addLayout(map_grid_container)

        content_widget.setLayout(content_layout)

        # 将内容部件设置到滚动区域中
        scroll_area.setWidget(content_widget)
        scroll_area.setWidgetResizable(True)
        self.scroll_area = scroll_area

        # 将滚动区域添加到主布局中
        main_layout.addWidget(scroll_area)

        # 创建主窗口
        main_window = QWidget()
        self.main_window = main_window

        main_window.setWindowTitle('无人艇路径规划仿真')
        # main_window.setGeometry(100, 100, 400, 300)

        # 将布局设置到主窗口
        main_window.setLayout(main_layout)

        self.timer = QTimer(main_window)
        self.timer.timeout.connect(self.sim_loop)
        self.timer.start(1000)

    def update_ship_num(self, text: str):
        self.map_config.shipNum = int(text)
        self.reset_ship_colors()

    def reset_ship_colors(self):
        self.ship_colors = {}
        for i in range(self.map_config.shipNum):
            hue = i * 137.508  # use golden angle approximation
            color = QColor()
            color.setHsl(int(hue), int(255 * .7), int(255 * .5))
            self.ship_colors[str(i)] = color

    def init_obstacles(self):
        """
        通过文件导入障碍物信息（使用坐标）
        """
        obstacles = set()
        file_path = obstacles_file

        try:
            # 读取文件中的障碍物信息
            with open(file_path, 'r') as file:
                obstacle_data = json.load(file)
                obstacles_list = obstacle_data.get("obstacles", [])

                # 将每个障碍物的坐标转换为索引，并加入障碍物集合
                for obstacle in obstacles_list:
                    x = obstacle.get("x")
                    y = obstacle.get("y")
                    if x is not None and y is not None:
                        # 转换为索引，假设地图宽度是 config.mapDimX
                        index = y * self.map_config.mapDimX + x
                        obstacles.add(index)

            # 将导入的障碍物信息赋值给 map_req 对象
            self.map_req.obstacles = obstacles

        except FileNotFoundError:
            print(f"错误: 文件 {file_path} 未找到！")
        except json.JSONDecodeError:
            print(f"错误: 文件 {file_path} 格式不正确！")
        except Exception as e:
            print(f"发生错误: {e}")

        # 初始化其他属性
        self.tasks = {}
        self.target_to_ship = {}
        self.plan = None

    def random_obstacles(self):
        print("random_obstacles")
        self.init_obstacles()
        self.rebuild_map_cells()

    def rebuild_map_cells(self):
        for cell in self.map_cells:
            cell.setParent(None)
            cell.deleteLater()

        self.map_cells = []

        x_n = self.map_config.mapDimX
        y_n = self.map_config.mapDimY
        print(f"rebuild_map_cells, x={x_n}, y={y_n}")

        for x in range(x_n):
            for y in range(y_n):
                index = x_y_to_index(x, y, x_n)
                obstacle = index in self.map_req.obstacles
                ship = self.target_to_ship.get(index)
                fill = "#87CEEB"
                if obstacle:
                    fill = "#666"
                elif ship is not None:
                    r, g, b, a = self.ship_colors[ship].getRgb()
                    color = QColor(r, g, b, 128)
                    fill = color_to_css_rgba(color)
                    print(f"ship {ship} at {x}, {y}, color {fill}")

                cell = QWidget(self.map_grid)
                self.map_cells.append(cell)
                cell.setFixedSize(QSize(cell_size, cell_size))
                cell.setStyleSheet(f"background-color: {fill};")
                cell.move(x * (cell_size + 1), y * (cell_size + 1))
                cell.show()

        print("rebuild_map_cells, done")

    def init_targets(self):
        tasks: dict[str, ShipTask] = {}
        target_to_ship: dict[int, str] = {}

        used_indexes: set[int] = set()

        # 从文件加载机器人起点和目标信息
        with open(ship_tasks_file, 'r') as f:
            task_config = json.load(f)

        ships = task_config['ships']

        config = self.map_config
        cell_num = config.mapDimX * config.mapDimY

        # 验证空间是否足够
        if config.shipNum * 2 >= cell_num - len(self.map_req.obstacles):
            QMessageBox.warning(self.main_window, "Warning", "No enough room.")
            return

        # 将障碍物添加到已使用索引中
        used_indexes.update(self.map_req.obstacles)

        for ship in ships:
            ship_id = str(ship['ship_id'])  # 将 ship_id 转换为字符串
            start_position = ship['start']
            target_positions = ship['targets']

            # 确保起点在可用区域内
            from_index = start_position[0] + start_position[1] * config.mapDimX
            if from_index in self.map_req.obstacles or from_index in used_indexes:
                QMessageBox.warning(self.main_window, "Warning", f"Ship {ship_id} start position is blocked.")
                return
            used_indexes.add(from_index)

            s1 = State(
                x=start_position[0],
                y=start_position[1],
                head=0,
                timeNum=0,
                timeStart=-1,
                timeEnd=-1
            )
            task = ShipTask(name=str(ship_id), fromState=state_to_cell(s1), toStates=[])
            tasks[str(ship_id)] = task

            bad_indexes: set[int] = set()
            retry_target = 0

            for target in target_positions:
                to_index = target[0] + target[1] * config.mapDimX
                if to_index in self.map_req.obstacles or to_index in used_indexes or to_index in bad_indexes:
                    bad_indexes.add(to_index)
                    continue

                s2 = State(x=target[0], y=target[1], head=0, timeNum=0, timeStart=-1, timeEnd=-1)
                if not self.test_path(ship_id, s1, s2):
                    bad_indexes.add(to_index)
                    continue

                used_indexes.add(to_index)
                task.toStates.append(state_to_cell(s2))

                target_to_ship[to_index] = str(ship_id)

            if len(task.toStates) == 0:
                QMessageBox.warning(self.main_window, "Warning", f"Ship {ship_id} has no valid targets.")
                return

        self.tasks = tasks
        self.target_to_ship = target_to_ship
        self.plan = None

        print(f"tasks: {tasks}")

        self.rebuild_map_cells()
        self.update_ships_ui()

    def test_path(self, ship_name: int, from_state: State, to_state: State) -> bool:
        print(f"find path: {ship_name}, {from_state} {to_state}")
        if from_state.x == to_state.x and from_state.y == to_state.y:
            return True
        open_set: list[Node] = [Node(state=from_state, g=0, h=0, f=0, parent=None)]
        close_set: list[Node] = []
        expanded_count = 0
        while open_set:
            expanded_count += 1
            top = open_set.pop(0)
            print(f"expanded: [{expanded_count}]R={ship_name}|x={top.state.x}|y={top.state.y}|f={top.f}")
            if top.state.x == to_state.x and top.state.y == to_state.y:
                return True
            close_set.append(top)
            neighbors = self.get_neighbors(top.state)
            for n in neighbors:
                g = top.g + 1
                h = distance_of_two_points(n.x, n.y, to_state.x, to_state.y)
                f = g + h
                open_index = find_index(open_set, lambda node: node.state.x == n.x and node.state.y == n.y)
                if open_index >= 0 and g < open_set[open_index].g:
                    open_set.pop(open_index)
                    open_index = -1
                close_index = find_index(close_set, lambda node: node.state.x == n.x and node.state.y == n.y)
                if close_index >= 0 and g < close_set[close_index].g:
                    close_set.pop(close_index)
                    close_index = -1
                if open_index < 0 and close_index < 0:
                    new_node = Node(state=n, g=g, h=h, f=f, parent=top)
                    open_set.append(new_node)

            open_set = sorted(open_set, key=lambda node: node.f)
        return False

    def get_neighbors(self, state: State) -> list[State]:
        neighbors: list[State] = []
        self.add_neighbor(neighbors, state, 0, 1)
        self.add_neighbor(neighbors, state, 0, -1)
        self.add_neighbor(neighbors, state, 1, 0)
        self.add_neighbor(neighbors, state, -1, 0)

        return neighbors

    def add_neighbor(self, neighbors: list[State], state: State, dx: int, dy: int) -> None:
        if state.x + dx < 0 or state.x + dx >= self.map_req.config.mapDimX:
            return
        if state.y + dy < 0 or state.y + dy >= self.map_req.config.mapDimY:
            return
        if ((state.y + dy) * self.map_req.config.mapDimX + state.x + dx) in self.map_req.obstacles:
            return
        neighbors.append(
            State(
                x=state.x + dx, y=state.y + dy, head=0,
                timeNum=0, timeStart=-1, timeEnd=-1
            )
        )

    def resolve(self):
        resolver = ECBS(
            w=self.map_config.w,
            map_dim_x=self.map_config.mapDimX,
            map_dim_y=self.map_config.mapDimY,
            obstacles=self.map_req.obstacles,
            tasks=self.tasks,
        )
        r = resolver.search()
        self.plan = r
        print("Plan: " + str(r))

    def toggle_sim(self):
        if self.simulation:
            self.stop_sim()
        else:
            if not (self.plan and self.plan.ok):
                QMessageBox.warning(self.main_window, "Warning", "No ok plan.")

            self.sim_ships = {}
            for ship_name, plan in self.plan.plans.items():
                self.sim_ships[ship_name] = self.build_ship_exe_path(ship_name, 1, plan.path[0], plan.path[1])

            self.simulation = True
            self.sim_btn.setText('停止仿真')
            self.adg_nodes = build_adg(self.plan)
            self.finished_adg_nodes.clear()

    def stop_sim(self):
        self.simulation = False
        self.sim_btn.setText('开始仿真')

    def sim_loop(self):
        if not self.simulation:
            return
        now = round(time.time() * 1000)
        ship_names = self.plan.plans.keys()

        # 第一轮循环，先推下进度
        for ship_name in ship_names:
            sim_ship = self.sim_ships[ship_name]
            duration = sim_ship.rotateDuration + sim_ship.moveDuration + sim_ship.waitDuration
            if duration <= 0:
                duration = 1000
            sim_ship.p = (now - sim_ship.startOn) / duration * self.sim_speed
            if sim_ship.p >= 1:
                sim_ship.p = 1
                sim_ship.holding = True
                print(f"done ADG node {sim_ship.adgKey}")
                self.finished_adg_nodes.add(sim_ship.adgKey)
                if sim_ship.timeEnd > self.current_time:
                    self.current_time = sim_ship.timeEnd

            rp = self.get_position(sim_ship)
            sim_ship.x = rp.x
            sim_ship.y = rp.y
            sim_ship.head = rp.head

        # 分配下一步
        all_done = True
        for ship_name in ship_names:
            sim_ship = self.sim_ships[ship_name]
            if sim_ship.p < 1:
                all_done = False
                continue
            path = self.plan.plans[ship_name].path
            if not path:
                continue
            next_index = sim_ship.s2Index + 1
            s1 = path[sim_ship.s2Index] if sim_ship.s2Index < len(path) else None
            s2 = path[next_index] if next_index < len(path) else None
            if not s1 or not s2:
                continue
            all_done = False
            adg_key = to_adg_key(ship_name, next_index)
            dependents = self.adg_nodes.get(adg_key)
            dependents_all_pass = True
            if dependents:
                for d in dependents:
                    if d not in self.finished_adg_nodes:
                        dependents_all_pass = False
                        break
            if dependents_all_pass:
                self.sim_ships[ship_name] = self.build_ship_exe_path(ship_name, next_index, s1, s2)
                print(f"release ADG node {sim_ship.adgKey}")

        self.update_ships_ui()

        if all_done:
            print(f"Sim done")
            self.stop_sim()
            return

    def build_ship_exe_path(self, ship_name: str, s2_index: int, s1: State, s2: State) -> ShipExePath:
        # 需要转的角度，初始，-270 ~ +270
        d_head = abs(s2.head - s1.head)
        # 270 改成 90
        if d_head > 180:
            d_head = 90
        d_head /= 90
        rotate_time_num = math.ceil(d_head)
        move_time_num = abs(s1.x - s2.x + s1.y - s2.y)
        wait_time_num = s2.timeNum - rotate_time_num - move_time_num
        # 每 90 度 1 秒
        rotate_duration = rotate_time_num * 1000 * (1 + random.random() * self.stepDurationVar)
        # 旋转，则移动为 0
        move_duration = move_time_num * 1000 * (1 + random.random() * self.stepDurationVar)
        return ShipExePath(
            s2Index=s2_index,
            adgKey=to_adg_key(ship_name, s2_index),
            timeStart=s1.timeStart or 0,  # 取开始点的
            timeEnd=s2.timeEnd or 0,  # 取结束点的
            startOn=round(time.time() * 1000),
            s1=s1, s2=s2,
            rotateDuration=int(rotate_duration),
            moveDuration=int(move_duration),
            waitDuration=wait_time_num * 1000,
            p=0,
            x=s1.x * (cell_size + 1),
            y=s1.y * (cell_size + 1),
            head=s1.head,
            holding=False
        )

    def get_position(self, sim_ship: ShipExePath) -> ShipPosition:
        s1 = sim_ship.s1
        s2 = sim_ship.s2
        p_rotate = 1
        p_move = 1
        time_pass = round(time.time() * 1000) - sim_ship.startOn
        if sim_ship.rotateDuration > 0:
            p_rotate = time_pass / sim_ship.rotateDuration * self.sim_speed
            if p_rotate > 1:
                p_rotate = 1
        if sim_ship.moveDuration > 0:
            p_move = (time_pass - sim_ship.rotateDuration * self.sim_speed) / sim_ship.moveDuration * self.sim_speed
            if p_move > 1:
                p_move = 1
            if p_move < 0:
                p_move = 0
        return ShipPosition(
            x=round((s1.x + p_move * (s2.x - s1.x)) * (cell_size + 1)),
            y=round((s1.y + p_move * (s2.y - s1.y)) * (cell_size + 1)),
            head=round(s1.head + (s2.head - s1.head) * p_rotate)
        )

    def update_ships_ui(self):
        for (ship_name, r_ui) in self.ship_widgets.items():
            r_ui.setParent(None)
            r_ui.deleteLater()

        self.ship_widgets = {}

        ship_positions: dict[str, ShipPosition] = {}
        if self.simulation and self.plan.ok:
            for ship_name, sim_ship in self.sim_ships.items():
                ship_positions[ship_name] = ShipPosition(x=sim_ship.x,
                                                           y=sim_ship.y,
                                                           head=sim_ship.head)
        else:
            for ship_name, task in self.tasks.items():
                ship_positions[ship_name] = ShipPosition(
                    x=task.fromState.x * (cell_size + 1),
                    y=task.fromState.y * (cell_size + 1),
                    head=0)

        ri = 0
        for (ship_name, p) in ship_positions.items():
            print(f"ship {ship_name} position: {p}")

            color = self.ship_colors[ship_name]
            r_ui = ShipWidget(cell_size, p.x, p.y, p.head, color, self.map_grid)
            self.ship_widgets[ship_name] = r_ui
            r_ui.show()

            ri += 1


def color_to_css_rgba(color: QColor):
    r = color.red()
    g = color.green()
    b = color.blue()
    a = color.alpha() / 255.0  # 将透明度从 0 - 255 范围转换到 0 - 1 范围
    return f"rgba({r}, {g}, {b}, {a})"


def main():
    app = QApplication(sys.argv)

    mapf_ui = MapfUi()

    # 显示主窗口
    mapf_ui.main_window.show()

    sys.exit(app.exec())


if __name__ == '__main__':
    main()
