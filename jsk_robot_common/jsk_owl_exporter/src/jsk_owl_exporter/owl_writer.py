#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import datetime
import lxml.etree
import lxml.builder
import os
import sys
from utils import UniqueStringGenerator, datetime_to_epoch_time

class OWLWriterMeta(object):
    VERSION = 0.93

class OWLWriter(OWLWriterMeta):
    def __init__(self, root_node, nsmap=None):
        self.root = root_node
        self.strgen = UniqueStringGenerator(8)
        self.nsmap = {
            None: "http://knowrob.org/kb/cram_log.owl#",
            "owl": "http://www.w3.org/2002/07/owl#",
            "xsd": "http://www.w3.org/2001/XMLSchema#",
            "knowrob": "http://knowrob.org/kb/knowrob.owl#",
            "rdfs": "http://www.w3.org/2000/01/rdf-schema#",
            "rdf": "http://www.w3.org/1999/02/22-rdf-syntax-ns#",
            "log": "http://knowrob.org/kb/cram_log.owl#",
        }
        if nsmap is not None:
            self.nsmap.update(nsmap)
        self.rdf = lxml.builder.ElementMaker(namespace=self.nsmap["rdf"],
                                             nsmap=self.nsmap)
        self.owl = lxml.builder.ElementMaker(namespace=self.nsmap["owl"],
                                             nsmap=self.nsmap)
        self.knowrob = lxml.builder.ElementMaker(namespace=self.nsmap["knowrob"],
                                                 nsmap=self.nsmap)
        self.doc = self.rdf.RDF()
        self.doc.base = "http://knowrob.org/kb/cram_log.owl"

        self.timepoints = []

    def add_attrib(self, elem, ns, name, val_ns, val):
        if val_ns is not None:
            val = "&%s;%s" % (val_ns, val)
        elem.attrib["{%s}%s" % (self.nsmap[ns], name)] = val
        return elem

    def gen_owl_import(self, uris):
        o = self.owl.Ontology()
        self.add_attrib(o, "rdf", "about", None, self.nsmap[None])
        for uri in uris:
            i = self.owl.imports()
            self.add_attrib(i, "rdf", "resource", None, uri)
            o.append(i)
        self.doc.append(o)

    def gen_property_definitions(self):
        self.doc.append(lxml.etree.Comment("Property Definitions"))

        for prop in self.root.prop_key_set():
            op = self.owl.ObjectProperty()
            self.add_attrib(op, "rdf", "about", "knowrob", prop)
            self.doc.append(op)

    def gen_class_definitions(self):
        self.doc.append(lxml.etree.Comment("Class Definitions"))

        for prop in self.root.type_set():
            cls = self.owl.Class()
            self.add_attrib(cls, "rdf", "about", "knowrob", prop)
            self.doc.append(cls)

    def gen_event_individual_recursive(self, n):
        for c in n.children:
            self.gen_event_individual_recursive(c)
        elem = self.owl.namedIndividual()
        self.add_attrib(elem, "rdf", "about", "log", n.name)

        # rdf:type
        etype = self.rdf.type()
        self.add_attrib(etype, "rdf", "resource", "knowrob", n.ntype)
        elem.append(etype)

        # properties
        for tag, value in n.properties.items():
            p = self.knowrob(tag)
            if isinstance(value, list):
                self.add_attrib(p, "rdf", "datatype", "xsd", "string")
                p.text = " ".join(value)
            elif isinstance(value, datetime.datetime):
                print value, datetime_to_epoch_time(value)
                n = "timepoint_%d" % datetime_to_epoch_time(value)
                print n
                if n not in self.timepoints:
                    self.timepoints.append(n)
                self.add_attrib(p, "rdf", "resource", "log", n)
            else:
                self.add_attrib(p, "rdf", "resource", "log", value)
            elem.append(p)

        self.doc.append(elem)

    def gen_event_individuals(self):
        self.doc.append(lxml.etree.Comment("Event Individuals"))
        self.gen_event_individual_recursive(self.root)

    def gen_object_individuals(self):
        pass

    def gen_human_individuals(self):
        pass

    def gen_image_individuals(self):
        pass

    def gen_designator_individuals(self):
        pass

    def gen_failure_individuals(self):
        pass

    def gen_timepoint_individuals(self):
        self.doc.append(lxml.etree.Comment("Timepoint Individuals"))
        for name in self.timepoints:
            tp = self.owl.NamedIndividual()
            self.add_attrib(tp, "rdf", "about", "log", name)
            typ = self.rdf.type()
            self.add_attrib(typ, "rdf", "resource", "knowrob", "TimePoint")
            tp.append(typ)
            self.doc.append(tp)

    def gen_metadata_individual(self):
        self.doc.append(lxml.etree.Comment("Meta Data Individual"))
        meta = self.owl.NamedIndividual()
        for tag, value in self.root.properties.items():
            p = self.knowrob(tag)
            self.add_attrib(p, "rdf", "datatype", "xsd", "string")
            p.text = value
            meta.append(p)

        start = self.knowrob.startTime()
        self.add_attrib(start, "rdf", "resource", "log", self.timepoints[0])
        meta.append(start)

        end = self.knowrob.endTime()
        self.add_attrib(end, "rdf", "resource", "log", self.timepoints[-1])
        meta.append(end)

        self.doc.append(meta)

    def gen_parameter_annotation_information(self):
        pass

    def to_string(self, pretty_print=True):
        # define entity references
        docstr = ["<!DOCTYPE rdf:RDF ["]
        docstr += ["<!ENTITY %s \"%s\" >" % (k,v) for k,v in self.nsmap.items() if k is not None]
        docstr += ["]>"]
        doctype = os.linesep.join(docstr)

        self.gen_owl_import([
            "package://knowrob_common/owl/knowrob.owl"
        ])

        self.gen_property_definitions()
        self.gen_class_definitions()

        self.gen_event_individuals()
        self.gen_object_individuals()
        self.gen_human_individuals()
        self.gen_image_individuals()
        self.gen_designator_individuals()
        self.gen_failure_individuals()
        self.gen_timepoint_individuals()
        self.gen_metadata_individual()
        self.gen_parameter_annotation_information()

        # generate xml string
        body = lxml.etree.tostring(self.doc,
                                   encoding="utf-8",
                                   xml_declaration=True,
                                   pretty_print=pretty_print,
                                   with_comments=True,
                                   doctype=doctype)

        # unescape entity references
        for key in self.nsmap.keys():
            body = body.replace("&amp;%s;" % key, "&%s;" % key)
        return body

    def to_file(self, dest, pretty_print=True):
        with open(dest, "w") as f:
            f.write(self.to_string(pretty_print=pretty_print))

if __name__ == '__main__':
    w = OWLWriter()
    print w.to_string()
