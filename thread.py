
#!/usr/bin/python
# -*- coding: UTF-8 -*-
 
import threading
import time
 
a=100
lock = threading.Lock()
def print_time1( threadName, delay):
	global a
	while 1:
		time.sleep(1)
		lock.acquire()
		a +=1
		lock.release()
		print "%s: %s" % (threadName, time.ctime(time.time()) )

def print_time2( threadName, delay):
	global a
	while 1:
		time.sleep(1)
		lock.acquire()
		print "%d, %s: %s" % (a, threadName, time.ctime(time.time()) )
		lock.release()
def main():
	try:
	   t1 = threading.Thread(target=print_time1, name='LoopThread', args=('LoopThread1',1,))
	   t2 = threading.Thread(target=print_time2, name='LoopThread', args=('LoopThread2', 1,))
	except:
	   print "Error: unable to start thread"
	t1.setDaemon(True)
	t2.setDaemon(True)
	t1.start()
	t2.start()
	# t2.join()
	count = 0
	while 1:
		time.sleep(1)
		count +=1
		if count>10:
			break
if __name__ == '__main__':
	main()
