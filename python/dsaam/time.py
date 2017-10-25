from __future__ import print_function, division, absolute_import

NS_IN_SECOND = 1000000000
    
class Time:
    def __init__(self, sec=0, nanos=0):
        assert type(sec) is int,\
            "Time.__init__ : Expected sec=int first parameter, got {}"\
            .format(sec.__class__)
        assert type(nanos) is int,\
            "Time.__init__ : Expected nanos=int second parameter, got {}"\
            .format(nanos.__class__)

        self.sec = sec + nanos // NS_IN_SECOND
        self.nanos = nanos % NS_IN_SECOND

        
    def __call__(self, other):
        assert isinstance(self, Time),\
            "Time.__call__ : Expected Time parameter, got {}".format(other.__class__)
        self.sec = other.sec
        self.nanos = other.nanos

    def to_nanos(self):
        return self.sec * NS_IN_SECOND + self.nanos

    def __add__(self, other):
        nanos = self.nanos % NS_IN_SECOND + other.nanos % NS_IN_SECOND
        carry = nanos // NS_IN_SECOND + self.nanos //  NS_IN_SECOND + other.nanos // NS_IN_SECOND
        return Time(self.sec + other.sec + carry, nanos % NS_IN_SECOND)

    def __sub__(self, other):
        nanos = self.nanos % NS_IN_SECOND - other.nanos % NS_IN_SECOND
        carry = nanos // NS_IN_SECOND + self.nanos //  NS_IN_SECOND - other.nanos // NS_IN_SECOND
        return Time(self.sec - other.sec + carry, nanos % NS_IN_SECOND)

    def __mul__(self, other):
        nanos = self.nanos % NS_IN_SECOND * other
        carry = nanos // NS_IN_SECOND + self.nanos // NS_IN_SECOND
        return Time(self.sec * other + carry, nanos % NS_IN_SECOND)
    
    def __lt__(self, other):
        return self.sec < other.sec or (self.sec == other.sec and self.nanos < other.nanos)

    def __le__(self, other):
        return self.__lt__(other) or self.__eq__(other)

    def __eq__(self, other):
        return isinstance(other, Time) and self.sec == other.sec and self.nanos == other.nanos

    def __ne__(self, other):
        return not self.__eq__(other)

    def __gt__(self, other):
        return not self.__le__(other)

    def __ge__(self, other):
        return not self.__lt__(other)

    def __repr__(self):
        return "Time(sec={}, nanos={})".format(self.sec, self.nanos)

    def __str__(self):
        return "{}:{}".format(self.sec, self.nanos)
