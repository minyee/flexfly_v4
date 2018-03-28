# This library contains the code for flexfly topology modelling in sstmacro
# Note that this library is meant to work with the sstmac that is built with the
# standalone core (i.e the sst-core that comes by default with sstmacro itself and not the external core).
# A lot of methods in the topology class has been reimplemented to fit the interface for 
# topology classes in SSTMacro.

#
# Written by Jason Teh, 03/28/2018

# Also this version actually directly wires pisces switches to one another, so as to make 
# for a much for realistic network topology, using the form_virtual_topology function and 
# the intergroup connectivity