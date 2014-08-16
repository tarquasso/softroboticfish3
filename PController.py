#!/usr/bin/python

class PController:
    def __init__(self, p, setPoint):
        self.p=p
        self.setPoint=setPoint
    def update(self, current):
        error=self.setPoint-current
        return current+(error*self.p)

if (__name__=="__main__"):
    pc=PController(0.5, 20.0)
    start=1.0
    count=0
    print 'starting at ' + str(start) + ' and ending at 20.0'
    while (start<19 or start>21):
        start=pc.update(start)
        count+=1
        print str(start)
    print 'converged in ' + str(count) + ' iterations'
