from typing import TypeAlias

CellPath: TypeAlias = tuple[list[tuple[int, int]], list[float]]
CIDPath: TypeAlias = tuple[list[int], list[float]]

TimeSteps: TypeAlias = list[float]
Vert: TypeAlias = tuple[float, float]
Poly: TypeAlias = list[Vert]
VertPath: TypeAlias = tuple[list[Vert], TimeSteps]
