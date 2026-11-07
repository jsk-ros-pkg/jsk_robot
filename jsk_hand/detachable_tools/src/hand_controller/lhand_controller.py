from __future__ import annotations

from .hand_interface import HandInterface


class LHandInterface(HandInterface):
    def __init__(self, groupname: str = "lhand", **kwargs) -> None:
        super().__init__("lhand", groupname=groupname, **kwargs)
