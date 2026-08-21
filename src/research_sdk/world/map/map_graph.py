# nodes and edges of map


from dataclasses import dataclass


@dataclass
class MapNode:
    id: int
    x: float
    y: float
    kind: str = "generic"


@dataclass
class MapEdge:
    start_id: int
    end_id: int
    cost: float
    clearance: float | None = None