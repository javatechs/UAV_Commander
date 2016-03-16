import sys
import subprocess

print sys.argv

arg = ['head','-n',1,'default_offboard.py']
subprocess.Popen(str(arg[i]) for i in range(0,len(arg)))
