"""Startup-only ID discovery. Runtime motor I/O belongs to ros2_control."""

from .description import TOOLS, describe_hand, normalize_side


def detacher_available(side, ids):
    """Whether all detacher motors defined by the URDF responded."""
    motors = describe_hand(side, 'detacher', attached=False).motors
    return {m.id for m in motors}.issubset(set(ids))


def select_tool(side, ids, include_detacher=True):
    """Resolve IDs using the URDF motor definitions; generic aliases gripper."""
    side = normalize_side(side)
    ids = set(ids)
    detacher_ids = {m.id for m in describe_hand(side, 'detacher', attached=False).motors}
    matches = []
    known = set(detacher_ids)
    for tool in TOOLS:
        if tool == 'generic':
            continue  # same motor as gripper; prefer its full model
        motor_ids = {m.id for m in describe_hand(side, tool, include_detacher=False).motors}
        known.update(motor_ids)
        if motor_ids and motor_ids.issubset(ids):
            matches.append(tool)
    if ids - known:
        raise RuntimeError(f'{side}: unexpected motor IDs {sorted(ids - known)}; check side/ID assignment')
    if len(matches) > 1:
        raise RuntimeError(f'{side}: multiple tools detected: {matches}')
    if not matches and (not include_detacher or not detacher_ids.issubset(ids)):
        raise RuntimeError(f'{side}: no usable tool or enabled detacher responded')
    return matches[0] if matches else 'detacher'


def scan_ids(port_name, baud_rate=57600):
    """Read Protocol 2 IDs without writing motor registers; always release port."""
    from dynamixel_sdk import COMM_SUCCESS, COMM_RX_TIMEOUT, PacketHandler, PortHandler

    port = PortHandler(port_name)
    packet = PacketHandler(2.0)
    try:
        if not port.openPort():
            raise RuntimeError(f'Cannot open {port_name}')
        if not port.setBaudRate(int(baud_rate)):
            raise RuntimeError(f'Cannot set baud rate on {port_name}')
        devices, result = packet.broadcastPing(port)
        if result not in (COMM_SUCCESS, COMM_RX_TIMEOUT):
            raise RuntimeError(f'{port_name}: {packet.getTxRxResult(result)}')
        return set(devices)
    finally:
        port.closePort()
