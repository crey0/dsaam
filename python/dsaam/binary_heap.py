import operator

class BinaryHeap:
    class Handle:
        def __init__(self, index, value):
            self.index = index
            self.value = value
                     
    def __init__(self, compare=operator.lt):
        self._heap = []
        self.compare=compare

    def push(self, value):
        h = BinaryHeap.Handle(len(self._heap), value)
        self._heap.append(h)
        self.siftup(h)
        return h

    def pop(self):
        hf = self._heap[0]
        hb = self._heap[-1]
        self.swap_nodes(hf, hb)
        self.siftdown(hb)
        self._heap.pop()

    def top(self):
        return self._heap[0].value

    def father(self, h):
        return self._heap[(h.index - 1) // 2]

    def left(self, h):
        return self._heap[2 * h.index + 1]

    def right(self, h):
        return self._heap[2 * h.index + 2]

    def child_count(self, h):
        size = len(self._heap)
        if 2 * h.index + 1 < size:
            return min(size - 2 * h.index - 1, 2)
        else:
            return 0

    def best_child(self, h):
        n = self.child_count(h)
        if(n==0): return None
        elif(n==1): return self.left(h)
        else:
            l = self.left(h)
            r = self.right(h)
            if self.compare(l.value, r.value):
                return r
            else:
                return l

    def swap_nodes(self, a, b):
        self._heap[b.index] = a
        self._heap[a.index] = b
        a_idx = a.index
        a.index = b.index
        b.index = a_idx

    def update(self, h):
        if h.index != 0 and self.compare(self.father(h).value, h.value):
            self.siftup(h)
        else:
            self.siftdown(h)

    def siftup(self, h):
        father = self.father(h)
        while h.index != 0 and self.compare(father.value, h.value):
            self.swap_nodes(h, father)
            father = self.father(h)

    def siftdown(self, h):
        c = self.best_child(h)
        while (c is not None and self.compare(h.value, c.value)):
            self.swap_nodes(h, c)
            c = self.best_child(h)
