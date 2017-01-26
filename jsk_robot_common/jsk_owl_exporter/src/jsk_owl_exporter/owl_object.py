#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import os
import StringIO

class OWLProperty(object):
    def __init__(self, tag=None, data_type=None, resource=None, content=None):
        self.tag = tag
        self.data_type = data_type
        self.resource = resource
        self.content = content

class OWLObject(object):
    def __init__(self):
        self.__id = None
        self.__type = None
        self.types = set()
        self.properties = []

        self.static_resources = {}
        self.static_properties = {}

    def add_static_resource(self, key, resource):
        self.static_resources[key] = resource
    def add_static_property(self, key, prop):
        self.static_properties[key] = prop
    def add_data_property(self, tag, data_type, content):
        prop = OWLProperty(tag=tag, data_type=data_type, content=content)
        self.properties.append(prop)
        self.issue_property(tag)
    def add_resource_property(self, tag, resource):
        prop = OWLProperty(tag=tag, resource=resource)
        self.properties.append(prop)
        self.issue_property(tag)
    def add_content_property(self, tag, content):
        prop = OWLProperty(tag=tag, content=content)
        self.properties.append(prop)
        self.issue_property(tag)

    @property
    def id(self):
        self.__id
    @id.setter
    def id(self, i):
        self.__id = i

    @property
    def type(self):
        self.__type
    @type.setter
    def type(self, t):
        self.__type = t
        self.types.add(t)

    def issued_properties(self):
        tags = list(set([p.tag for p in self.properties]))
        for i, t in enumerate(tags):
            ts = t.split(':')
            if len(ts) > 1:
                tags[i] = "&%s;%s" % (ts[0], ts[1])
        return tags

    def issued_types(self):
        return list(self.types)

    def owl_string(self, idnent, space=2):
        os = StringIO.StringIO()
        def write(indent_level, line):
            os.write(' ' * (indent + indent_level) * space)
            os.write(line + os.linesep)

        write(0, '<owl:NamedIndividual rdf:about="%s">' % self.id)

        if self.type is not None:
            write(1, '<rdf:type rdf:resource="%s"/>' % self.type)

        for p in self.properties:
            if p.data_type is not None:
                write(1, '<%s rdf:datatype="%s">%s</%s>' % (p.tag, p.data_type, p.content, p.tag))
            elif p.resource is not None:
                write(1, '<%s rdf:resource="%s"/>' % (p.tag, p.resource))
            else:
                write(1, '<%s>%s</%s>' % (p.tag, p.content, p.tag))

        for k, v in self.static_resources.items():
            write(1, '<knowrob:%s rdf:resource="%s"/>' % (k, v))

        for k, v in self.static_properties.items():
            write(1, '<knowrob:%s rdf:datatype="&xsd;string>%s</knowrob:%s>' % (k, v, k))

        write(0, '</owl:NamedIndividual>')

        return os.getvalue()
