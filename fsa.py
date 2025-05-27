import heapq
import time
from dataclasses import replace
from math import ceil
from typing import Optional, Callable

from pydash import remove

from domain import State, TargetManyPlanResult, TargetOnePlanResult, LowNode, OpenLowNode, FocalLowNode, Cell, \
    state_to_cell


class FSA:
    """
    有界最优，Focal Search A*
    f, g, h 单位是时间，表示时间成本
    """

    def __init__(self, ship_name: str, high_node_id: int, w: float,
                 map_dim_x: int, map_dim_y: int, obstacles: set[int],
                 move_unit_cost: float = 1.0, rotate_unit_cost: float = 1.0,
                 neighbor_validator: Optional[Callable[[State, State], bool]] = None,
                 focal_state_heuristic2: Optional[Callable[[State, float], float]] = None,
                 focal_transition_heuristic2: Optional[Callable[[State, State, float, float], float]] = None):
        """
        :param ship_name:
        :param high_node_id:
        :param w:
        :param map_dim_x:
        :param map_dim_y:
        :param obstacles:
        :param move_unit_cost:
        :param rotate_unit_cost:
        :param neighbor_validator:
        :param focal_state_heuristic2:
        :param focal_transition_heuristic2:
        """
        self.ship_name = ship_name
        self.high_node_id = high_node_id
        self.w = w
        self.map_dim_x = map_dim_x
        self.map_dim_y = map_dim_y
        self.obstacles = obstacles
        self.move_unit_cost = move_unit_cost
        self.rotate_unit_cost = rotate_unit_cost
        self.neighbor_validator = neighbor_validator
        self.focal_state_heuristic2 = focal_state_heuristic2
        self.focal_transition_heuristic2 = focal_transition_heuristic2

    def search(self, start_state: State, goal_states: list[State], goal_stop_time: int) -> TargetManyPlanResult:
        """
        多目标搜索
        :param start_state:
        :param goal_states:
        :param goal_stop_time: how many steps ship stop when goal reached
        :return:
        """
        start_on = time.time()

        from_state = start_state
        ok = True
        reason = ""

        steps: list[TargetOnePlanResult] = []
        path: list[State] = []

        expanded_count = 0
        cost = 0.0
        min_f = 0.0
        time_num = 0

        for ti, goal_state in enumerate(goal_states):
            sr = self.search_one(time_num, from_state, goal_state, goal_stop_time, -1)
            expanded_count += sr.expandedCount

            if not sr.ok:
                ok = False
                reason = sr.reason
                break

            steps.append(sr)
            path.extend(sr.path)

            cost += sr.cost
            min_f += sr.minF
            time_num += sr.timeNum

            from_state = goal_state

        return TargetManyPlanResult(
            self.ship_name,
            ok,
            reason,
            cost=cost,
            minF=min_f,
            expandedCount=expanded_count,
            planCost=time.time() - start_on,
            timeNum=time_num, timeStart=0, timeEnd=time_num,
            steps=steps, path=path,
        )

    def search_one(self, time_offset: int, start_state: State,
                   goal_state: State, goal_stop_time: int, last_goal_constraint: int = -1) -> TargetOnePlanResult:
        """
        单目标搜索
        :param time_offset: 起始时刻
        :param start_state:
        :param goal_state:
        :param goal_stop_time:
        :param last_goal_constraint: 如果目标位置被约束了，last_goal_constraint 是被约束的最后一个时刻
        :return:
        """
        print(f"Search one: offset={time_offset}, start={start_state}, goal={goal_state}")

        start_on = time.time()
        expanded_count = 0

        open_set: list[OpenLowNode] = []
        focal_set: list[FocalLowNode] = []
        closed_set: set[Cell] = set()  # by location only
        # 待展开，未展开（不在 close 里），那一定在 open 里吧，所以不需要这个
        processed_nodes: dict[Cell, LowNode] = {}  # by location

        # noinspection PyTypeChecker
        start_node = LowNode(
            replace(start_state, timeStart=time_offset, timeEnd=time_offset, timeNum=1),  # 时间从 timeOffset
            None,
            f=self.admissible_heuristic(start_state, goal_state),
            g=0.0,
            focalHeuristic=0.0,
        )

        heapq.heappush(open_set, OpenLowNode(start_node))
        heapq.heappush(focal_set, FocalLowNode(start_node))

        # TODO eq 对吗
        processed_nodes[state_to_cell(start_node.state)] = start_node

        min_f = start_node.f

        while open_set:
            # update focal list
            old_min_f = min_f
            min_f = open_set[0].n.f  # 读取最小的节点，但不删除

            focal_set = []
            old_bound = old_min_f * self.w
            bound = min_f * self.w
            for node in open_set:
                if node.n.f <= bound:
                    heapq.heappush(focal_set, FocalLowNode(node.n))

            # TODO 有这一步，可以不要上一步？
            if min_f > old_min_f:  # top.n.f 值肯定在增大
                for node in open_set:
                    # 之前不在 focal_set，本轮新加入 focal_set 的节点
                    if old_bound < node.n.f <= bound:
                        heapq.heappush(focal_set, FocalLowNode(node.n))
                    if node.n.f > bound:
                        break

            # 取出下一个有界最优启发值最低的节点
            print(f"debug: focal set size before pop: {len(focal_set)}")
            min_focal_n: FocalLowNode = heapq.heappop(focal_set)
            min_focal_s = min_focal_n.n.state
            print(f"min focal node: {min_focal_s}")
            expanded_count += 1

            if expanded_count > self.map_dim_x * self.map_dim_y:
                print("展开过多，是 BUG？")
                return TargetOnePlanResult(
                    self.ship_name,
                    False,
                    "展开过多",
                    planCost=time.time() - start_on,
                    fromState=start_state,
                    toState=goal_state,
                )

            if min_focal_s.is_same_location(goal_state) and min_focal_s.timeEnd > last_goal_constraint:
                # 达到时间在最后一次目标点被约束的时刻后
                # 到达后等待一段时间
                last_state = min_focal_s
                path = [last_state]
                curr_node = min_focal_n.n.parent
                while curr_node:
                    path.append(curr_node.state)
                    curr_node = curr_node.parent
                path.reverse()

                goal_stop_time2 = goal_stop_time if goal_stop_time > 0 else 1

                # 最后追加一个原地等待的，模拟动作时间
                # noinspection PyTypeChecker
                action_state = replace(last_state,
                                       timeStart=last_state.timeEnd + 1,
                                       timeEnd=last_state.timeEnd + goal_stop_time2,
                                       timeNum=goal_stop_time2)
                path.append(action_state)

                return TargetOnePlanResult(
                    self.ship_name,
                    True,
                    cost=min_focal_n.n.g + goal_stop_time2,
                    minF=min_focal_n.n.f + goal_stop_time2,
                    planCost=time.time() - start_on,
                    expandedCount=expanded_count,
                    timeNum=action_state.timeEnd,
                    timeStart=time_offset,
                    timeEnd=action_state.timeEnd,
                    fromState=start_state,
                    toState=goal_state,
                    path=path)

            # TODO 直接删除底层数据机构，堆还正常吗？
            remove(open_set, lambda n: n.n == min_focal_n.n)

            # 加入 close 就不用在 processedStates
            # TODO key not in processed_nodes
            processed_nodes.pop(state_to_cell(min_focal_n.n.state), None)
            closed_set.add(state_to_cell(min_focal_n.n.state))

            # traverse neighbors
            neighbors = self.get_neighbors(min_focal_s, goal_state, goal_stop_time)
            for neighbor in neighbors:
                # 只要经过这个位置，不管时间、朝向
                neighbor_cell = state_to_cell(neighbor)
                if neighbor_cell in closed_set:
                    continue

                g = min_focal_n.n.g + neighbor.timeNum
                old_node = processed_nodes.get(neighbor_cell)
                if old_node is None:
                    f = g + self.admissible_heuristic(neighbor, goal_state)
                    focal_heuristic = (min_focal_n.n.focalHeuristic +
                                       self.focal_state_heuristic(neighbor, g) +
                                       self.focal_transition_heuristic(min_focal_s, neighbor, min_focal_n.n.g, g))
                    node = LowNode(neighbor, min_focal_n.n, f=f, focalHeuristic=focal_heuristic, g=g)
                    heapq.heappush(open_set, OpenLowNode(node))
                    processed_nodes[state_to_cell(node.state)] = node
                    if node.f <= min_f * self.w:
                        heapq.heappush(focal_set, FocalLowNode(node))
                else:
                    # We found this node before with a better path
                    if g >= old_node.g:
                        continue
                    old_g = old_node.g
                    olg_f = old_node.f
                    # update f and g
                    node = replace(old_node, g=g, f=old_node.f + g - old_g)

                    # 肯定已在 openSet，重新添加引发排序
                    remove(open_set, lambda n: n.n == node)
                    heapq.heappush(open_set, OpenLowNode(node))
                    if old_node.f <= min_f * self.w < olg_f:
                        heapq.heappush(focal_set, FocalLowNode(node))

        return TargetOnePlanResult(
            self.ship_name,
            False,
            "找不到路径",
            planCost=time.time() - start_on,
            expandedCount=expanded_count,
            fromState=start_state,
            toState=goal_state,
        )

    def admissible_heuristic(self, from_state: State, target_state: State) -> float:
        """
        两点之间的直线路径
        vs 欧氏距离
        """
        return float(abs(from_state.x - target_state.x) + abs(from_state.y - target_state.y)) / self.move_unit_cost

    def focal_state_heuristic(self, to_state: State, to_state_g: float) -> float:
        """
        故意 inadmissible 的启发式
        """
        if self.focal_state_heuristic2 is not None:
            return self.focal_state_heuristic2(to_state, to_state_g)
        else:
            return to_state_g

    def focal_transition_heuristic(self, from_state: State, to_state: State,
                                   from_state_g: float, to_state_g: float) -> float:
        if self.focal_transition_heuristic2 is not None:
            return self.focal_transition_heuristic2(from_state, to_state, from_state_g, to_state_g)
        else:
            return to_state_g - from_state_g

    def get_neighbors(self, from_state: State, goal_state: State, goal_stop_time: int) -> list[State]:
        neighbors = []
        self.add_valid_neighbor(neighbors, from_state, goal_state, goal_stop_time, 0, 0,
                                from_state.head)  # waiting
        self.add_valid_neighbor(neighbors, from_state, goal_state, goal_stop_time, 1, 0, 0)
        self.add_valid_neighbor(neighbors, from_state, goal_state, goal_stop_time, -1, 0, 180)
        self.add_valid_neighbor(neighbors, from_state, goal_state, goal_stop_time, 0, 1, 90)
        self.add_valid_neighbor(neighbors, from_state, goal_state, goal_stop_time, 0, -1, 270)
        return neighbors

    def add_valid_neighbor(self, neighbors: list[State],
                           from_state: State, goal_state: State, goal_stop_time: int, dx: int, dy: int, to_head: int):
        """
        toHead 目标车头朝向
        """
        x = from_state.x + dx
        y = from_state.y + dy

        if (x < 0 or x >= self.map_dim_x or y < 0 or y >= self.map_dim_y
                or self.state_to_index(x, y) in self.obstacles):
            return

        # 需要转的角度，初始，-270 ~ +270
        d_head = abs(to_head - from_state.head)

        # 270 改成 90
        if d_head > 180:
            d_head = 90

        d_head /= 90

        # 耗时，也作为 g 的增量
        # 假设 dx/dy 1 是 1 米
        time_num = ceil(float(abs(dx + dy)) / self.move_unit_cost + d_head / self.rotate_unit_cost)

        if time_num < 1:
            time_num = 1  # 原地等待

        new_state = State(x=x,
                          y=y,
                          head=to_head,
                          timeStart=from_state.timeEnd + 1,
                          timeEnd=from_state.timeEnd + time_num,
                          timeNum=time_num)

        # 最后一步要等待
        test_state = new_state
        if goal_state.is_same_location(new_state):
            # noinspection PyTypeChecker
            test_state = replace(new_state,
                                 timeEnd=new_state.timeEnd + goal_stop_time,
                                 timeNum=new_state.timeNum + goal_stop_time)

        if self.neighbor_validator is not None and not self.neighbor_validator(from_state, test_state):
            return

        neighbors.append(new_state)

    def state_to_index(self, x: int, y: int):
        return x + y * self.map_dim_x
