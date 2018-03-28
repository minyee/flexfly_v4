#from sst.macro import *
import sys
import os
import sst
import sst.macro
#from . import setupTopology 
from setupTopology import *
from setupTopologySimplified import *

isSoFile = True
idx = 1

while isSoFile:
	next = sys.argv[idx]
	if next.endswith(".so"):
		if not os.path.isfile(next):
  			sys.exit("Invalid library specified %s" % next)
		folder, lib = os.path.split(next)
		if not folder: folder = os.getcwd()
		old = os.environ["SST_LIB_PATH"] 
		print "folder is %s" % folder
		print old + ":" + folder
		os.environ["SST_LIB_PATH"] = old + ":" + folder
		sys.path.append(folder)
		importer = ".".join(lib[3:].split(".")[:-1]) #chop off lib and last so
		importer = "sst." + importer
		cmd = "import %s" % importer
		exec(cmd)
		del sys.argv[idx]
	else: isSoFile = False
	idx += 1
if False:
	setupTopology()
else:
	setupTopologySimplified()
