#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import datetime
import os
from StringIO import StringIO
from utils import UniqueStringGenerator


class Graph(object):
    def __init__(self):
        self.s = StringIO()
        self.num = 1
        self.table = {}
    def node(self, name, label=None, color="white"):
        self.table[name] = str(self.num)
        self.num += 1
        self.s.write("  %s" % self.table[name])
        if label is None:
            label = name
        label = label.replace("<", "\<").replace(">", "\>")
        self.s.write(" [")
        self.s.write("label=\"%s\"," % label)
        self.s.write("fillcolor=\"%s\"," % color)
        self.s.write("style=\"solid,filled\"")
        self.s.write("];" + os.linesep)
    def edge(self, f, t):
        self.s.write(" %s -> %s" % (self.table[f], self.table[t]))
        self.s.write(";" + os.linesep)
    def save(self, fn):
        with open(fn, 'w') as f:
            f.write("digraph sample {" + os.linesep)
            f.write("  node [shape = record];" + os.linesep)
            f.write(self.s.getvalue())
            f.write("}" + os.linesep)


class GraphWriter(object):
    def __init__(self, root_node):
        self.root = root_node
        self.graph = Graph()
    def node_attr(self, n):
        return n.name + "\\n" + "\\l".join(["%s: %s" % (k, str(v)) for k,v in n.properties.items()]) + "\\l"
    def parse_node(self, n):
        color = "white"
        if "actionResult" in n.properties:
            result = n.properties["actionResult"]
            if result == "SUCCESS":
                color = "#ccffff" # pastel blue
            else:
                color = "#ffcccc"
        self.graph.node(n.name, label=self.node_attr(n), color=color)
        if n.parent is not None:
            self.graph.edge(n.name, n.parent.name)
        for c in n.children:
            self.parse_node(c)
    def parse(self):
        self.parse_node(self.root)
    def save_pdf(self, dest, preview=False):
        base, _ = os.path.splitext(dest)
        dot_path = base + ".dot"
        pdf_path = base + ".pdf"
        self.graph.save(dot_path)
        os.system("dot -Kdot -Tpdf %s > %s" % (dot_path, pdf_path))
        if preview:
            os.system("gnome-open %s" % pdf_path)
