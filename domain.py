import math
from dataclasses import dataclass, field
from typing import Optional

from dataclasses_json import dataclass_json


@dataclass_json
@dataclass(frozen=True)
class Location:
    """
    frozen 才是 hashable 的，可以放入 set。
    """
    x: int
    y: int


@dataclass_json
@dataclass(frozen=True)
class Cell(Location):
    pass


@dataclass_json
@dataclass(frozen=True)
class TimeSteps:
    timeStart: int  # 开始去这个位置的第一个时间步
    timeEnd: int  # 到达这个位置的时刻


def is_time_overlay(start1: int, end1: int, start2: int, end2: int):
    return start1 <= end2 and start2 <= end1


@dataclass_json
@dataclass(frozen=True)
class State(Location, TimeSteps):
    """
    允许跨多个时间步
    车头方向，
    0，x 正，向右
    90，y 正，向下
    180，x 负，向左
    270，y 负，向上
    """
    head: int
    timeNum: int

    def is_same_location(self, o: Location):
        return self.x == o.x and self.y == o.y

    def is_time_overlay(self, o: TimeSteps):
        return is_time_overlay(self.timeStart, self.timeEnd, o.timeStart, o.timeEnd)

    def desc_loc_head(self) -> str:
        return f"({self.x},{self.y},{self.head})"


def cell_to_state(c: Cell) -> State:
    return State(x=c.x, y=c.y, timeStart=0, timeEnd=0, timeNum=0, head=0)


def state_to_cell(s: State) -> Cell:
    return Cell(s.x, s.y)


@dataclass_json
@dataclass
class TargetOnePlanResult:
    """
    单目标
    """
    shipName: str
    ok: bool = True
    reason: str = None
    cost: float = 0.0
    minF: float = 0.0
    expandedCount: int = 0
    planCost: float = 0  # 秒
    timeNum: int = 0
    timeStart: int = -1
    timeEnd: int = -1
    fromState: State = None
    toState: State = None
    path: list[State] = None
    extra: any = None


@dataclass_json
@dataclass
class TargetManyPlanResult:
    """
    多目标
    """
    shipName: str
    ok: bool = True
    reason: str = None
    cost: float = 0.0
    minF: float = 0.0
    expandedCount: int = 0
    planCost: float = 0  # 秒
    timeNum: int = 0
    timeStart: int = -1
    timeEnd: int = -1
    steps: list[TargetOnePlanResult] = None
    path: list[State] = None  # 总路径
    extra: any = None


@dataclass_json
@dataclass
class ShipTask:
    name: str
    fromState: Cell
    toStates: list[Cell]
    stopTimes: int = 1  # 停多久


@dataclass_json
@dataclass
class MapfReq:
    w: float
    mapDimX: int
    mapDimY: int  # y 向下为正
    obstacles: set[int]
    tasks: dict[str, ShipTask]  # ship name ->
    goalStops: int = 0


@dataclass_json
@dataclass
class MapfResult:
    ok: bool = True
    plans: dict[str, TargetManyPlanResult] = None
    timeCost: float = 0.0


@dataclass_json
@dataclass(frozen=True)
class LowNode:
    state: State
    parent: Optional['LowNode'] = None
    f: float = 0.0
    focalHeuristic: float = 0.0
    g: float = 0.0  # 到这个节点的实际成本


@dataclass_json
@dataclass(frozen=True)
class VertexConstraint(TimeSteps, Location):
    pass


@dataclass_json
@dataclass(frozen=True)
class EdgeConstraint(TimeSteps):
    x1: int
    y1: int
    x2: int
    y2: int


@dataclass_json
@dataclass(frozen=True)
class Constraints:
    shipName: str
    vertexConstraints: field(default_factory=list)
    edgeConstraints: field(default_factory=list)


@dataclass_json
@dataclass(frozen=True)
class HighNode:
    id: int  # 节点 ID
    parentId: int
    solution: dict[str, TargetManyPlanResult]  # ship name ->
    constraints: dict[str, Constraints]  # ship name ->
    cost: float
    lb: float
    focalHeuristic: float


class OpenHighNode:

    def __init__(self, n: HighNode):
        self.n = n

    def __lt__(self, other: 'OpenHighNode'):
        # if (cost != n.cost)
        return self.n.cost < other.n.cost


class FocalHighNode:

    def __init__(self, n: HighNode):
        self.n = n

    def __lt__(self, other: 'FocalHighNode'):
        if self.n.focalHeuristic != other.n.focalHeuristic:
            return self.n.focalHeuristic < other.n.focalHeuristic
        return self.n.cost < other.n.cost


class OpenLowNode:
    """
    排序：lowest fScore，highest gScore
    """

    def __init__(self, n: LowNode):
        self.n = n

    def __lt__(self, other: 'OpenLowNode'):
        if self.n.f != other.n.f:
            return self.n.f < other.n.f
        return other.n.g < self.n.g


class FocalLowNode:
    """
    # Sort order (see "Improved Solvers for Bounded-Suboptimal Multi-Agent Path Finding" by Cohen et. al.)
    # 1. lowest focalHeuristic
    # 2. lowest fScore
    # 3. highest gScore
    """

    def __init__(self, n: LowNode):
        self.n = n

    def __lt__(self, other: 'FocalLowNode'):
        if self.n.focalHeuristic != other.n.focalHeuristic:
            return self.n.focalHeuristic < other.n.focalHeuristic
        elif self.n.f != other.n.f:
            return self.n.f < other.n.f
        return other.n.g < self.n.g


@dataclass_json
@dataclass
class MapConfig:
    shipNum: int = 3
    mapDimX: int = 20
    mapDimY: int = 10
    w: float = 1.5
    targetNum: int = 2

@dataclass_json
@dataclass
class MapReq:
    config: MapConfig
    obstacles: set[int]


def x_y_to_index(x: int, y: int, map_dim_x: int) -> int:
    return y * map_dim_x + x

def distance_of_two_points(x1: int, y1: int, x2: int, y2: int) -> float:
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)