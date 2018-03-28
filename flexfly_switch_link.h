#ifndef SWITCH_LINK
#define SWITCH_LINK

enum Link_Type {Electrical, Optical};

struct switch_link {
  sstmac::switch_id src_sid;
  sstmac::switch_id dest_sid; // switch_id of the destination switch
  int src_outport;
  int dest_inport; // port of the destination switch
  Link_Type type; 
};
#endif