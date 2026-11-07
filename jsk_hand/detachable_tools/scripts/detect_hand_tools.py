#!/usr/bin/env python3
"""Scan both buses once and emit launch data after releasing both ports."""
import argparse
import json

from detachable_tools_common.tool_detection import detacher_available, scan_ids, select_tool


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--left-port-name', default='/dev/lhand')
    parser.add_argument('--right-port-name', default='/dev/rhand')
    parser.add_argument('--baud-rate', type=int, default=57600)
    parser.add_argument('--include-detacher', type=str.lower, choices=('true', 'false'), default='true')
    args = parser.parse_args()
    detections = {}
    for side, port in [('lhand', args.left_port_name), ('rhand', args.right_port_name)]:
        ids = scan_ids(port, args.baud_rate)
        tool = select_tool(side, ids, args.include_detacher == 'true')
        enabled = args.include_detacher == 'true' and detacher_available(side, ids)
        detections[side] = dict(include_detacher=str(enabled).lower(), tool=tool, attached=str(tool != 'detacher').lower(),
                                detected_ids=','.join(str(i) for i in sorted(ids)))
    print(json.dumps(detections))


if __name__ == '__main__':
    main()
