#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>


import argparse
import os
import sys
from jsk_owl_exporter import *


def list_tasks(db_addr, simple=False):
    client = get_mongo_client(db_addr)
    tasks = list_logged_tasks(client)
    fmt = "{:<13}  ({:<6} docs, {:<10} - {:<10})"
    if len(tasks) > 0:
        for t in tasks:
            if simple:
                print t["task"]
            else:
                print fmt.format(t["task"], t["msg_count"], t["from"], t["till"])
        return True
    else: return False

def info_task(db_addr, task_id):
    client = get_mongo_client(db_addr)
    info = get_logged_task_info(client, task_id)
    stat = info.pop("Data")

    # print result
    fmt = "{:<13} | {:<30}"
    for t, s in info.items():
        print fmt.format(t, s)
    print "Statistics"
    statfmt = "  {:<50} | {:>6}"
    print statfmt.format("Data Type", "Size")
    for t, s in stat.items():
        print statfmt.format(t, s)
    return True

def export_task(db_addr, task_id, output_dir):
    client = get_mongo_client(db_addr)
    agg = MongoAggregator(client, task_id)
    node = agg.aggregate()
    writer = OWLWriter(node)
    save_path = os.path.join(output_dir, "log.owl")
    writer.to_file(save_path)
    print "saved to %s" % save_path
    return True

def graph_task(db_addr, task_id, output_dir, preview):
    client = get_mongo_client(db_addr)
    agg = MongoAggregator(client, task_id)
    node = agg.aggregate()
    writer = GraphWriter(node)
    writer.parse()
    save_path = os.path.join(output_dir, "%s.pdf" % task_id)
    writer.save_pdf(save_path, preview)
    print "saved to %s" % save_path
    return True

def exec_command(args):
    if args.command == "list":
        return list_tasks(args.db, args.simple)
    elif args.command == "info":
        return info_task(args.db, args.task)
    elif args.command == "export":
        return export_task(args.db, args.task, args.output)
    elif args.command == "graph":
        return graph_task(args.db, args.task, args.output, args.preview)


if __name__ == '__main__':
    p = argparse.ArgumentParser(description="OWL Exporter from mongo database")

    p.add_argument("--db", default="mongodb://localhost:27017/jsk_robot_lifelog/pr1012",
                   type=str, help="address for database")

    verbs = ["list", "info", "graph", "export"]
    sp = p.add_subparsers(title="command",
                          metavar="[" + " | ".join(verbs) + "]",
                          dest="command")

    cp = sp.add_parser("list", description="list logged tasks")
    cp.add_argument("-s", "--simple", action="store_true",
                    help="Show only task IDs")

    cp = sp.add_parser("info", description="show logged task information and statistics")
    cp.add_argument("task", type=str, help="task ID")

    cp = sp.add_parser("graph", description="export graph of episodic memory")
    cp.add_argument("task", type=str, help="task ID")
    cp.add_argument("-o", "--output", default=os.getcwd(), type=str,
                    help="output directory of exported graph")
    cp.add_argument("-p", "--preview", action="store_true",
                    help="preview pdf")

    cp = sp.add_parser("export", description="export ontology from task logged data")
    cp.add_argument("task", type=str, help="task ID")
    cp.add_argument("-o", "--output", default=os.getcwd(), type=str,
                    help="output directory of exported log.owl")

    args = p.parse_args()
    sys.exit(exec_command(args))

