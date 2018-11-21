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
