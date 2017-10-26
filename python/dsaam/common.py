from __future__ import print_function, division, absolute_import

from collections import Sequence


class NamedMutableSequence(Sequence):
    __slots__ = ()

    def __init__(self, *a, **kw):
        slots = self.__slots__
        for k in slots:
            setattr(self, k, kw.get(k))

        if a:
            for k, v in zip(slots, a):
                setattr(self, k, v)

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
    return type(name, (NamedMutableSequence,), {'__slots__': members})  
        
InFlow = namedlist('InFlow', ['name', 'time', 'dt', 'qsize', 'callback', 'time_callback'])
OutFlow = namedlist('OutFlow', ['name', 'time', 'dt', 'qsize', 'sinks'])
Sink = namedlist('Sink', ['name', 'callback'])
