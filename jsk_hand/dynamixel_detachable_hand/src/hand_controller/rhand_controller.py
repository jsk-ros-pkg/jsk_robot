from __future__ import annotations

from .hand_interface import HandInterface


class RHandInterface(HandInterface):
    def __init__(self, groupname: str = "rhand", **kwargs) -> None:
        super().__init__("rhand", groupname=groupname, **kwargs)
