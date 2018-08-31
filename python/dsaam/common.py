from __future__ import print_function, division, absolute_import

from collections import Sequence
from enum import Enum


class NamedMutableSequence(Sequence):
    __slots__ = ()

    def __init__(self, *a, **kw):
        slots = self.__slots__

        i=-1
        if a:
            for i, (k, v) in enumerate(zip(slots, a)):
                setattr(self, k, v)
        
        try:
            for k in slots[i+1:]:
                setattr(self, k, kw[k])
        except KeyError as e:
            raise KeyError("Missing constructor argument {} in class {}"\
                           .format(e, self.__class__))
        
    def __str__(self):
        clsname = self.__class__.__name__
        values = ', '.join('%s=%r' % (k, getattr(self, k))
                           for k in self.__slots__)
        return '%s(%s)' % (clsname, values)

    __repr__ = __str__

    def __getitem__(self, item):
        return getattr(self, self.__slots__[item])

    def __setitem__(self, item, value):
        return setattr(self, self.__slots__[item], value)

    def __len__(self):
        return len(self.__slots__)

def namedlist(name, members):
    if isinstance(members, str):
        members = members.split()
    members = tuple(members)
    ret = type(name, (NamedMutableSequence,), {'__slots__': members})  
    ret.__module__ = NamedMutableSequence.__module__
    return ret

InFlow = namedlist('InFlow', ['name', 'time', 'dt', 'qsize', 'callback', 'time_callback'])
OutFlow = namedlist('OutFlow', ['name', 'time', 'dt', 'qsize', 'sinks', 'ftype'])
Sink = namedlist('Sink', ['name', 'callback'])

FlowType = Enum('FlowType', 'PRED OBS')
