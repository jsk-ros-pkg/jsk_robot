#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

from bson.son import SON
import calendar
import datetime
import pymongo
import random
import string
import time


class UniqueStringGenerator(object):
    def __init__(self, strlen=8):
        self.strlen = strlen
        self.issued = set()
    def gen(self):
        while True:
            s = ''.join([random.choice(string.ascii_letters + string.digits) for i in range(self.strlen)])
            if s not in self.issued:
                self.issued |= set(s)
                return s

def is_simulation_time(t):
    return t.tm_year < 1990

def epoch_time_to_datetime(e):
    t = time.gmtime(e)
    if is_simulation_time(t):
        return datetime.datetime(t.tm_year, t.tm_mon, t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec)
    else:
        return datetime.datetime.fromtimestamp(time.mktime(time.gmtime(e)))

def datetime_to_epoch_time(dt, local=True):
    t = dt.timetuple()
    if is_simulation_time(t):
        return calendar.timegm(t)
    else:
        return int(time.mktime(t))

def parse_mongodb_address(s):
    host = "localhost"
    port = 27017
    db = "test"
    col = "test"
    if s.startswith("mongodb://"):
        s = s[len("mongodb://"):]
    s = s.split("/")
    if len(s) > 3:
        return None
    elif len(s) >= 1:
        ss = s[0].split(":")
        if len(ss) >= 1:
            host = ss[0]
        elif len(ss) == 2:
            port = int(ss[1])
        else:
            return None
    if len(s) >= 2:
        db = s[1]
    if len(s) >= 3:
        col = s[2]
    return { "host": host,
             "port": port,
             "db"  : db,
             "col" : col }

def get_mongo_client(address):
    addr = parse_mongodb_address(address)
    if addr is None:
        return None
    c = pymongo.MongoClient(addr["host"], addr["port"])
    return c[addr["db"]][addr["col"]]

def list_logged_tasks(client):
    res = client.aggregate([
        { "$sort" : SON([("_id", pymongo.ASCENDING)])},
        { "$group": { "_id": "$_meta.task_id", "msgs": { "$sum": 1 }}},
    ])
    tasks = []
    if res["ok"] == 1.0:
        for tid in res["result"]:
            task_id = tid["_id"]
            if isinstance(task_id, str) or isinstance(task_id, unicode):
                cur = client.find({"_meta.task_id": task_id }).sort([("$natural", pymongo.ASCENDING)]).limit(1)
                task_start = "N/A"
                if cur.alive:
                    task_start_time = cur.next()["_id"].generation_time
                    task_start = task_start_time.strftime("%Y/%m/%d %H:%M:%S")
                cur = client.find({"_meta.task_id": task_id }).sort([("$natural", pymongo.DESCENDING)]).limit(1)
                task_end = "N/A"
                if cur.alive:
                    task_end = cur.next()["_id"].generation_time.strftime("%Y/%m/%d %H:%M:%S")
                tasks.append({
                    "task": task_id,
                    "msg_count": tid["msgs"],
                    "from": task_start,
                    "till": task_end,
                    "sort_key": task_start_time,
                })
    return sorted(tasks, key=lambda x: x["sort_key"])

def get_logged_task_info(client, task_id):
    # task name
    task_name = client.find_one({
        "_meta.task_id": task_id,
        "_meta.task_name" : { "$exists": True }})
    if task_name is not None:
        task_name = task_name["_meta"]["task_name"]

    # date
    task_start = client.find({ "_meta.task_id": task_id }).sort([("$natural", pymongo.ASCENDING)]).limit(1)
    if task_start.alive:
        task_start = task_start.next()["_meta"]["inserted_at"].strftime("%Y/%m/%d %H:%M:%S")
    else:
        task_start = "N/A"
    task_end = client.find({ "_meta.task_id": task_id }).sort([("$natural", pymongo.DESCENDING)]).limit(1)
    if task_end.alive:
        task_end = task_end.next()["_meta"]["inserted_at"].strftime("%Y/%m/%d %H:%M:%S")
    else:
        task_end = "N/A"

    # count
    data_size = client.find({"_meta.task_id": task_id }).count()

    # stat
    stat = client.aggregate([
        { "$match" : { "_meta.task_id": task_id }},
        { "$group" : { "_id": "$_meta.stored_type",
                       "size": { "$sum" : 1 }}},
        { "$sort" : SON([("_id", pymongo.ASCENDING)])},
    ])
    if stat["ok"] != 1.0:
        stat = None
    else:
        stat = {d["_id"]: d["size"] for d in stat["result"]}

    return {
        "Task ID": task_id,
        "Task Name": task_name,
        "Date": task_start + " - " + task_end,
        "Data Size": data_size,
        "Data": stat,
    }

