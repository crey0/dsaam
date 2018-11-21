# Copyright © 2018 CNRS
# All rights reserved.

# @author Christophe Reymann

#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:

#  1. Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#  2. Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.

#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
#  TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
#  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
#  BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
#  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
#  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
#  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
#  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
#  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.

from __future__ import print_function, division, absolute_import

from collections import Sequence
from enum import Enum


class RecordType(Sequence):

    __slots__ = ()

    def __init__(self, *a, **kw):

        slots = self.__slots__
        i = -1
        if a:
            for i, (k, v) in enumerate(zip(slots, a)):
                setattr(self, k, v)

        try:
            for k in slots[i+1:]:
                setattr(self, k, kw[k])
        except KeyError as e:
            raise KeyError("Missing constructor argument {} in class {}"
                           .format(e, self.__class__))

    def __str__(self):
        cname = self.__class__.__name__
        items = ', '.join('{}={}'.format(k, repr(getattr(self, k)))
                          for k in self.__slots__)
        return '{}({})'.format(cname, items)

    __repr__ = __str__

    def __getitem__(self, item):
        return getattr(self, self.__slots__[item])

    def __setitem__(self, item, value):
        return setattr(self, self.__slots__[item], value)

    def __len__(self):
        return len(self.__slots__)


def record(name, members):
    if isinstance(members, str):
        members = members.split()
    members = tuple(members)
    ret = type(name, (RecordType,), {'__slots__': members})
    ret.__module__ = RecordType.__module__
    return ret


InFlow = record('InFlow', ['name', 'time', 'dt', 'qsize', 'callback', 'time_callback'])
OutFlow = record('OutFlow', ['name', 'time', 'dt', 'qsize', 'sinks', 'ftype'])
Sink = record('Sink', ['name', 'callback'])

FlowType = Enum('FlowType', 'PRED OBS')
