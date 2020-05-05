import time
class TimeIt():
    ''' simple class to instrument code for profiling '''
    def __init__(self):
        self.events = []
        self.mark('start')
    def mark(self, name):
        self.events.append((name, time.time()))
    def show(self):
        for i in range(1, len(self.events)):
            name = self.events[i][0]
            delta = self.events[i][1] - self.events[i-1][1]
            print('event: {} delta: {:6.3f}'.format(name, delta))
