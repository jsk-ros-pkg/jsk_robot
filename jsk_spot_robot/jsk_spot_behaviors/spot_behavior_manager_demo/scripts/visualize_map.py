#!/usr/bin/env python3
# -*- encoding: utf-8 -*-

import argparse
import yaml

from graphviz import Digraph

import gi
gi.require_version('Gtk','3.0')
from gi.repository import Gtk

import xdot


class MyDotwindow(xdot.DotWindow):

    def __init__(self):
        xdot.DotWindow.__init__(self)
        self.dotwidget.connect('clicked', self.on_url_clicked)

    def on_url_clicked(self, widget, url, event):
        dialog = Gtk.MessageDialog(
                parent=self,
                buttons=Gtk.ButtonsType.OK,
                message_format='{} clicked'.format(url))
        dialog.connect('response', lambda dialog, response: dialog.destroy())
        dialog.run()
        return True


def main():

    parser = argparse.ArgumentParser()
    parser.add_argument('filename')
    parser.add_argument('--output')
    args = parser.parse_args()

    filename = args.filename

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


    if args.output:
        dg.render(args.output)
    else:
        window = MyDotwindow()
        window.set_dotcode(dg.source.encode('utf-8'))
        window.connect('delete-event', Gtk.main_quit)
        Gtk.main()


if __name__ == '__main__':
    main()
