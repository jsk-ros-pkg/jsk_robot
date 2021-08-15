#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import argparse
import yaml

from graphviz import Digraph


def main():

    parser = argparse.ArgumentParser()
    parser.add_argument('--filename',required=True)
    parser.add_argument('--output',required=True)
    args = parser.parse_args()

    filename = args.filename
    output = args.output

    with open(filename, 'r') as f:
        map_data = yaml.load(f)

    edges = map_data['edges']

    list_node = []
    list_behavior_type = []
    list_color_for_bt = []

    for edge in edges:

        if edge['from'] not in list_node:
            list_node.append(edge['from'])

        if edge['to'] not in list_node:
            list_node.append(edge['to'])

        if edge['behavior_type'] not in list_behavior_type:
            list_behavior_type.append(edge['behavior_type'])

    for index, behavior_type in enumerate(list_behavior_type):
        list_color_for_bt.append('{} 1.0 1.0'.format(1.0 * index / len(list_behavior_type)))

    dg = Digraph(format='svg')

    for node in list_node:
        dg.node(node)

    for edge in edges:
        dg.attr('edge', color=list_color_for_bt[list_behavior_type.index(edge['behavior_type'])])
        dg.edge(edge['from'], edge['to'], label=edge['behavior_type'].split('.')[2])

    dg.render(output)


if __name__ == '__main__':
    main()
