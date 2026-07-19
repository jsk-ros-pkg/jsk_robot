from __future__ import annotations

from .ros2_hand_controller import HandInterface


class LHandInterface(HandInterface):
    def __init__(self, groupname: str = "lhand", **kwargs) -> None:
        super().__init__("lhand", groupname=groupname, **kwargs)
