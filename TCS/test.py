#!/usr/bin/env python

import subprocess
import re
import os
import sys
import platform





tasklog = open('task_list.log', 'r')
tasklist=[]
tasklist_amount=0
tasklist_counter=0
tasklist_finish_flag=True
tasklist_env = os.environ.copy()

for eachline in tasklog:
    line = eachline.strip('\n').split(' ')
    # python TASK.py [args] [timeout in second]
    tasklist.append(['python', str(line[0])+'.py', ' '.join(line[1:-1])])
    tasklist_amount+=1
tasklog.close()

print tasklist[0]

#call the first task
# subprocess.call(' '.join(tasklist[0]), env=tasklist_env)
subprocess.Popen(tasklist[0], env=tasklist_env)
print "task executed"
# subprocess.call('', shell=True, env=tasklist_env)


