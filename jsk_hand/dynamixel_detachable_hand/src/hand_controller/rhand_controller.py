from __future__ import annotations

from .ros2_hand_controller import HandInterface


class RHandInterface(HandInterface):
    def __init__(self, groupname: str = "rhand", **kwargs) -> None:
        super().__init__("rhand", groupname=groupname, **kwargs)

    def open_holder(
        self,
        *,
        wait: bool = True,
        tm: float = 1.0,
        velocity: float = 0.5,
        acceleration: float = 0.0,
        effort: float = 0.0,
    ):
        return self.move_hand(-0.1, wait=wait, tm=tm, velocity=velocity, acceleration=acceleration, effort=effort)

    def close_holder(
        self,
        *,
        wait: bool = True,
        tm: float = 1.0,
        velocity: float = 0.5,
        acceleration: float = 0.0,
        effort: float = 0.0,
    ):
        return self.move_hand(0.08, wait=wait, tm=tm, velocity=velocity, acceleration=acceleration, effort=effort)
