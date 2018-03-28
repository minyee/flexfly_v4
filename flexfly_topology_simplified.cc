/**
 * Author: Min Yee Teh
 */
#include <iostream>	
#include <algorithm>
#include <vector>
#include <unordered_map>
#include <iostream>
#include <sstmac/common/node_address.h>
#include <sprockit/sim_parameters.h>
#include <stdlib.h>
#include <sstmac/hardware/topology/topology.h>
#include <stack>
#include "flexfly_topology_simplified.h"
#include "link_stealing.h"
#include <queue>
namespace sstmac {
namespace hw {

 // NOTE: the switch with the same id as the max_switch_id_ is the optical switch.
 flexfly_topology_simplified::flexfly_topology_simplified(sprockit::sim_parameters* params) : 
                              structured_topology(params,InitMaxPortsIntra::I_Remembered, 
                                                  InitGeomEjectID::I_Remembered) {
  std::cout << "flexfly_topology_simplified called" << std::endl;
  num_groups_ = params->get_int_param("groups"); // controls g
 	switches_per_group_ = params->get_int_param("switches_per_group"); // controls a
 	nodes_per_switch_ = params->get_int_param("nodes_per_switch");
  traffic_matrix_filename_ = params->get_optional_param("traffic_matrix_filename", "traffic_matrix.txt");
  link_stealing_ = params->get_bool_param("link_steal");
  cheat_optical_bandwidth_ = params->get_optional_bool_param("cheat", false);
  max_switch_id_ = (num_groups_ * switches_per_group_) - 1;
  group_connectivity_matrix_.resize(num_groups_);
  for (int i = 0; i < num_groups_; i++) {
    group_connectivity_matrix_[i].resize(num_groups_);
    for (int j = 0; j < num_groups_; j++) {
      if (i == j) {
        group_connectivity_matrix_[i][j] = 0;
      } else {
        group_connectivity_matrix_[i][j] = 1;
      }
    }
  }
 	
  
  
  g2g_traffic_matrix_.resize(num_groups_);
  for (int i = 0; i < num_groups_; i++) {
    g2g_traffic_matrix_[i].resize(num_groups_);
    std::fill(g2g_traffic_matrix_[i].begin(), g2g_traffic_matrix_[i].end(), 0);
  }

  /*
   * link_stealking algorithm is applied here to group_connectivity_matrix;
   * begin
   */
  if (params->get_bool_param("link_steal")) {
    matrix_float float_mat;
    std::string filename = params->get_param("bandwidth_allocation_matrix_filename");
    read_float_matrix(filename, float_mat);
    for (int i = 0; i < num_groups_; i++) {
      for (int j = 0; j < num_groups_; j++) {
        group_connectivity_matrix_[i][j] = (int) float_mat[i][j];
        std::cout << std::to_string((int) float_mat[i][j]) << ", ";
      }
      std::cout << std::endl;
    }
  }
  setup_flexfly_topology_simplified();
  route_topology();
  int diam = diameter();
  std::cout << "The diameter of this topology is: " << std::to_string(diam) << std::endl;
  std::cout << "got out of flexfly_topol constructor?" << std::endl;
 }

 flexfly_topology_simplified::~flexfly_topology_simplified() {
    for (const std::pair<switch_id, std::vector<switch_link*>> elem : switch_outport_connection_map_) {
      const std::vector<switch_link*>& conn_vector = elem.second;  
      for (auto const&  switch_link_ptr : conn_vector) {
        if (switch_link_ptr)
          free(switch_link_ptr);
      }
    }

    std::ofstream file;
    file.open(traffic_matrix_filename_);
    file << std::to_string(num_groups_) << std::endl;
    for (int i = 0; i < num_groups_; i++) {
      for (int j = 0; j < num_groups_; j++) {
        file << std::to_string(g2g_traffic_matrix_[i][j]) << " ";
      }
      file << std::endl;
    }
    file.close();

    std::cout << "Deconstructor flexfly_topology_simplified is called" << std::endl;
 }

 /**
  * the implementation is the one where the optical radix is the same as the # of 
  * groups, and there will be switches_per_group_ number of optical switches
  */
 void flexfly_topology_simplified::setup_flexfly_topology_simplified() {
   // first step: connect all the switches within the same group together
  //std::cout << "3 " << std::endl;
  int last_used_outport[max_switch_id_ + 1];
  int last_used_inport[max_switch_id_ + 1];
  std::memset(&last_used_outport, 0, sizeof(int) * (max_switch_id_ + 1));
  std::memset(&last_used_inport, 0, sizeof(int) * (max_switch_id_ + 1));
   for (int group = 0; group < num_groups_; group++) {
     switch_id group_offset = group * switches_per_group_;
     for (int index = 0; index < switches_per_group_ - 1; index++) {
       switch_id swid = group_offset + index;
       for (int target_index = index + 1; target_index < switches_per_group_; target_index++) {
         switch_id target_swid = group_offset + target_index;
         connect_switches(swid, last_used_outport[swid], target_swid, last_used_inport[target_swid], Electrical);
         last_used_outport[swid]++;
         last_used_inport[target_swid]++;
         connect_switches(target_swid, last_used_outport[target_swid], swid, last_used_inport[swid], Electrical);
         last_used_outport[target_swid]++;
         last_used_inport[swid]++;
       }
     }  
   }
   // second step: connect all the switches within groups to the optical switches
   form_virtual_topology(false, last_used_inport, last_used_outport);
 };

 /**
  * IMPORTANT: This function will route the minimal path
  **/
 void flexfly_topology_simplified::minimal_route_to_switch(switch_id src_switch_addr, 
 												switch_id dst_switch_addr, 
 												routable::path& path) const {
  
  int src_group = group_from_swid(src_switch_addr);
  int dst_group = group_from_swid(dst_switch_addr);
  auto iter = switch_outport_connection_map_.find(src_switch_addr);
  assert(iter != switch_outport_connection_map_.end());
  int ports = (iter->second).size();
  path.vc = 0;
 
  for (int i = 0; i < ports; i++) {
    if ((iter->second)[i]->dest_sid == routing_table_[src_switch_addr][dst_switch_addr]) {
      path.set_local_outport((iter->second)[i]->src_outport);
      path.set_outport((iter->second)[i]->src_outport);
  
      return;
    }
  }
  
 };

 int flexfly_topology_simplified::get_output_port(int src_switch, int dst_switch) const {
  //std::cout << "7 " << std::endl;
  auto sl_iter = switch_outport_connection_map_.find(src_switch);
  assert(sl_iter != switch_outport_connection_map_.end());
  auto slv = sl_iter->second;
  //int i = 0;
  for (auto sl : slv) {
    if (sl->dest_sid == dst_switch)
      return sl->src_outport;
    //i++; 
  }
  spkt_abort_printf("Get output port function in flexfly_topology_simplified has failed ");
  return -1;
 };

 /**
  * checks if a given switch id in the topology is an optical switch or not
  */
 bool flexfly_topology_simplified::is_optical_switch(switch_id swid) const {
  //std::cout << "8 " << std::endl;
  if (!valid_switch_id(swid)) {
 		return false;
 	} 
  //std::cout << "got out of 8" << std::endl;
 	return swid == max_switch_id_;
 };

 /**
  * checks if a given switch id in the topology is an electrical switch or not
  */
 bool flexfly_topology_simplified::is_electrical_switch(switch_id swid) const {
  //std::cout << "9" << std::endl;
  return valid_switch_id(swid) && !is_optical_switch(swid);
 };

  // DONE
 void flexfly_topology_simplified::configure_individual_port_params(switch_id src,
                                   sprockit::sim_parameters* switch_params) const {
  if (!valid_switch_id(src)) return;
  sprockit::sim_parameters* link_params = switch_params->get_namespace("link");
  
  double electrical_bw = link_params->get_bandwidth_param("electrical_link_bandwidth");
  double optical_bw = link_params->get_bandwidth_param("optical_link_bandwidth"); 
  // default to large buffer size
  long credits = switch_params->get_optional_byte_length_param("buffer_size", 100e9); 
  credits = 100e9;
  //double super_high_bw = 10e10;
  auto adjacency_vector = switch_outport_connection_map_.find(src);
  assert(adjacency_vector != switch_outport_connection_map_.end());
  int port_count = (adjacency_vector->second).size();
  for (int i = 0; i < port_count; i++) {
    if (group_from_swid((adjacency_vector->second)[i]->dest_sid) == group_from_swid(src)) { // if same group, use electrical
      topology::setup_port_params(i, credits, electrical_bw, link_params, switch_params);
    } else {
      int src_group = group_from_swid(src);
      int dst_group = group_from_swid((adjacency_vector->second)[i]->dest_sid);
      int num_connections = std::max(group_connectivity_matrix_[src_group][dst_group], 1);
      num_connections = cheat_optical_bandwidth_ ? num_connections : 1;
      topology::setup_port_params(i, credits, num_connections * optical_bw, link_params, switch_params);
    }
  }
 };

 void flexfly_topology_simplified::configure_nonuniform_switch_params(switch_id src, sprockit::sim_parameters* switch_params) const {
    switch_params->add_param_override("id", int(src));
    switch_params->add_param_override("model", "pisces");
    switch_params->add_param_override("switches_per_group", int(switches_per_group_));
    switch_params->add_param_override("num_groups", int(num_groups_));
    switch_params->add_param_override("total_radix", int(switches_per_group_ + nodes_per_switch_)); 
    sprockit::sim_parameters* ej_params = switch_params->get_namespace("ejection");
    long num_credits = ej_params->get_byte_length_param("credits");
  }
 



 /**
  * @Brief Given a source switch and a destination switch, connects the source switch with the 
  * dest switch
  * NOTE: This member function should form a single switch_link
  */
  void flexfly_topology_simplified::connect_switches(switch_id src, int src_outport, switch_id dst, int dst_inport, Link_Type ltype) {
    std::vector<switch_link*>& src_outport_connection_vector = switch_outport_connection_map_[src];
    std::vector<switch_link*>& dst_inport_connection_vector = switch_inport_connection_map_[dst];
    switch_link *conns = new switch_link();
    conns->src_sid = src;
    conns->dest_sid = dst;
    conns->dest_inport = dst_inport; 
    conns->src_outport = src_outport;
    conns->type = ltype;
    switch_outport_connection_map_[src].push_back(conns);
    switch_inport_connection_map_[dst].push_back(conns);
    //assert(conns->src_outport == src_outport_connection_vector.size() - 1);
    //assert(conns->dest_inport == dst_inport_connection_vector.size() - 1);
    return;
 }


void flexfly_topology_simplified::connected_outports(const switch_id src, 
                                            std::vector<topology::connection>& conns) const {
  //std::cout << "13" << std::endl;
  std::unordered_map<switch_id, std::vector<switch_link*>>::const_iterator got = switch_outport_connection_map_.find(src);

  int cidx = 0;
  if (got != switch_outport_connection_map_.end()) {
    const std::vector<switch_link*>& switch_link_vectors = got->second;
    conns.resize(switch_link_vectors.size());
    for (switch_link* current_switch_link : switch_link_vectors) {
      //conns[topology::connection()];
      conns[cidx].src = src;
      conns[cidx].dst = current_switch_link->dest_sid;
      conns[cidx].src_outport = current_switch_link->src_outport; 
      conns[cidx].dst_inport = current_switch_link->dest_inport;
      //conns[cidx].link_type = current_switch_link->type;
      cidx++;
    }
  }
  //std::cout << "got out of 13" << std::endl;
  //std::cout << "cidx is: " << std::to_string(cidx) << std::endl;
}


bool flexfly_topology_simplified::switch_id_slot_filled(switch_id sid) const {
  //std::cout << "14 " << std::endl;
  return (sid <= max_switch_id_);
}

 void flexfly_topology_simplified::configure_vc_routing(std::map<routing::algorithm_t, int>& m) const {
  m.insert({routing::minimal, 1});
  m.insert({routing::minimal_adaptive, 1});
  m.insert({routing::valiant, 1});
  m.insert({routing::ugal, 1});
  return;
 };

  switch_id flexfly_topology_simplified::node_to_ejection_switch(node_id addr, uint16_t& port) const {
    switch_id swid = addr / nodes_per_switch_; // this gives us the switch id of the switch node addr is connected to
    //std::unordered_map<switch_id, std::vector<switch_link*>>::const_iterator tmp_iter = switch_outport_connection_map_.find(swid);
    //const std::vector<switch_link*>& conn_vector = tmp_iter->second;
    //port = std::max((int) (conn_vector.size() - 1), 0) + ((int) swid) * nodes_per_switch_; // CHECK THIS AGAIN
    auto iter = switch_outport_connection_map_.find(swid);
    auto outport_connections = iter->second;
    port = outport_connections.size() + (addr % nodes_per_switch_);
    return swid;
  };
  
  switch_id flexfly_topology_simplified::node_to_injection_switch(node_id addr, uint16_t& port) const {
    //std::cout << "17 " << std::endl;
    return node_to_ejection_switch(addr, port);
  };

  /**
   * NOTE: This method does not include the hop to an optical switch
   **/
  int flexfly_topology_simplified::minimal_distance(switch_id src, switch_id dst) const {
    //std::cout << "src switch id: " << std::to_string(src) << " dst switch id: " << std::to_string(dst) << std::endl;
    //std::cout << "18 " << std::endl;
    int src_group = group_from_swid(src);
    int dst_group = group_from_swid(dst);
    if (src == dst) { // same switch
      return 0;
    } else if (src_group == dst_group) { // same group
      return 1;
    } else { // different group but can reach either by 1 global and 1 local or 1 local and then 1 global
      if (group_connectivity_matrix_[src_group][dst_group] > 0) {
        return 3;
      } else {
        return 5;
      }
    }
  };

  int flexfly_topology_simplified::num_hops_to_node(node_id src, node_id dst) const {
    //std::cout << "19 " << std::endl;
    int src_swid = src / (nodes_per_switch_);
    int dst_swid = dst / (nodes_per_switch_);
    int min_dist = minimal_distance(src_swid, dst_swid);
    return min_dist + 2; // added by 2 because each node is 1 hop away from it's switch
  };

  void flexfly_topology_simplified::nodes_connected_to_injection_switch(switch_id swid, 
                                                              std::vector<injection_port>& nodes) const {
    if (swid > max_switch_id_) return;
    auto iter = switch_outport_connection_map_.find(swid);
    int port_offset = (iter->second).size();
    nodes.resize(nodes_per_switch_);
    for (int i = 0; i < nodes_per_switch_; i++) {
      int node_id = swid * nodes_per_switch_ + i;
      int port_ind = port_offset + i;
      nodes[i].nid = node_id;
      nodes[i].port = port_ind;
    }
    //std::cout << "did i get out " << std::endl;
  };

  void flexfly_topology_simplified::nodes_connected_to_ejection_switch(switch_id swid, 
                                                              std::vector<injection_port>& nodes) const { 
    if (swid > max_switch_id_) return;
    auto iter = switch_outport_connection_map_.find(swid);
    int port_offset = (iter->second).size();
    nodes.resize(nodes_per_switch_);
    for (int i = 0; i < nodes_per_switch_; i++) {
      int node_id = swid * nodes_per_switch_ + i;
      int port_ind = port_offset + i;
      nodes[i].nid = node_id;
      nodes[i].port = port_ind;
    }
  };

  // returns the group id of a given switch
  int flexfly_topology_simplified::group_from_swid(switch_id swid) const {
    return swid / (switches_per_group_);
  };

  /**
   * prints out the all the connections for each switch
   */
  void flexfly_topology_simplified::print_port_connection_for_switch(switch_id swid) const {
    //std::cout << "1000" << std::endl;
    std::unordered_map<switch_id, std::vector<switch_link*>>::const_iterator tmp_iter = 
                                        switch_outport_connection_map_.find(swid);
    
    if (tmp_iter == switch_outport_connection_map_.end()) {
      return;
    }

    const std::vector<switch_link*>& connection_vector = tmp_iter->second;

    std::string message;
    int i = 0;
    for (switch_link* sl_ptr : connection_vector) {
      // check if null, if null have to throw an error 
      if (sl_ptr) {
        message += ("Dest switch_id: " + std::to_string(sl_ptr->dest_sid));
        message += (" Dest inport: " + std::to_string(sl_ptr->dest_inport));
        message += " Link type: ";
        if (sl_ptr->type == Electrical) {
          message += ("ELECTRICAL\n");
        } else {
          message += ("OPTICAL\n");
        }
      } else {
        spkt_abort_printf("A switch link with swid: %d is null\n", swid);
      }
      i++;
    }
    std::cout << message; 
  };


  /**
   * @brief num_endpoints To be distinguished slightly from nodes.
   * Multiple nodes can be grouped together with a netlink.  The netlink
   * is then the network endpoint that injects to the switch topology
   * @return
   */
  int flexfly_topology_simplified::num_netlinks() const {
    //std::cout << "22 " << std::endl;
    //std::cout << "num_netlinks?" << std::endl;
    return 1;
  }; 

  switch_id flexfly_topology_simplified::max_netlink_id() const {
    //std::cout << "23 " << std::endl;
    //std::cout << "max_netlink_id?" << std::endl;
    return 0;
    //return max_switch_id_;
  };

  bool flexfly_topology_simplified::netlink_id_slot_filled(node_id nid) const {
    //std::cout << "24 " << std::endl;
    //std::cout << "netlink_id_slot_filled?" << std::endl;

    return (nid < (num_groups_ * switches_per_group_ * nodes_per_switch_));
  };

  /**
     For a given node, determine the injection switch
     All messages from this node inject into the network
     through this switch
     @param nodeaddr The node to inject to
     @param switch_port [inout] The port on the switch the node injects on
     @return The switch that injects from the node
  */
  switch_id flexfly_topology_simplified::netlink_to_injection_switch(
        netlink_id nodeaddr, uint16_t& switch_port) const {
    
    return max_switch_id_;
  };

  /**
     For a given node, determine the ejection switch
     All messages to this node eject into the network
     through this switch
     @param nodeaddr The node to eject from
     @param switch_port [inout] The port on the switch the node ejects on
     @return The switch that ejects` into the node
  */
  switch_id flexfly_topology_simplified::netlink_to_ejection_switch(
        netlink_id nodeaddr, uint16_t& switch_port) const {
    return max_switch_id_;
  };

  
  bool flexfly_topology_simplified::node_to_netlink(node_id nid, node_id& net_id, int& offset) const {
    //std::cout << "27 " << std::endl;
    
    net_id = nid ;
    offset = 0 ;
    return false;
    //offset = 0;
    //net_id = nid;
    
  };

  switch_id flexfly_topology_simplified::node_to_switch(node_id nid) const {
    //std::cout << "28 " << std::endl;
    switch_id swid = nid / (nodes_per_switch_);
    return swid;
  };

  /**
   * given two group id's, group1 and group2, returns the number of intergroup links 
   * 
   */
  int flexfly_topology_simplified::num_links_between_groups(int group1, int group2) const {
    //std::cout << "29 " << std::endl;
    return group_connectivity_matrix_[group1][group2];
  };

  int flexfly_topology_simplified::diameter() const {
    int max_so_far = 0;
    // do dfs and bfs for each node, and then find the maximum distance.
    for (int i = 0; i < num_groups_; i++) {
      std::stack<int> st;
      st.push(i);
      std::vector<bool> visited(num_groups_);
      std::vector<int> distance(num_groups_);
      std::fill(visited.begin(), visited.end(), false);
      std::fill(distance.begin(), distance.end(), INT_MAX);
      visited[i] = true;
      distance[i] = 0;
      while (!st.empty()) {
        int curr = st.top();
        st.pop();
        visited[curr] = true;
        for (int j = 0; j < num_groups_; j++) {
          if (curr == j || visited[j] || group_connectivity_matrix_[curr][j] == 0 ) continue;
          st.push(j);
          if (distance[j] > distance[curr] + 1) {
            distance[j] = distance[curr] + 1;
          }
        }
      }

      for (auto dist : distance) {
        if (dist > max_so_far) {
          max_so_far = dist;
        }
      }
    }
    return (max_so_far == INT_MAX) ? INT_MAX : (max_so_far + 2); // add two because we are only couting the group diameter 
  };

  void flexfly_topology_simplified::form_virtual_topology(bool deterministic, int* last_used_inport, int* last_used_outport) {
    uint32_t seed = 100;
    std::srand(seed);
    std::unordered_map<int, std::vector<switch_id>> switches_in_group_out;
    std::unordered_map<int, std::vector<switch_id>> switches_in_group_in;
    

    /*
     * preprocessing stage
     */
    for (int i = 0; i < num_groups_; i++) {

      switches_in_group_out.insert({i, std::vector<switch_id>(0)});
      auto iter_out = switches_in_group_out.find(i);
      switches_in_group_in.insert({i, std::vector<switch_id>(0)});
      auto iter_in = switches_in_group_in.find(i);
      switch_id offset = i * switches_per_group_;
      for (int j = 0; j < switches_per_group_; j++) {
        switch_id curr_id = offset + j;
        (iter_out->second).push_back(curr_id);
        (iter_in->second).push_back(curr_id);
      }
    }
    /*
     * forming connection, main stage
     */
    for (int i = 0; i < num_groups_; i++) {
      auto iter_src = switches_in_group_out.find(i);
      for (int j = 0; j < num_groups_; j++) {
        if (i == j) continue;
        auto iter_dst = switches_in_group_in.find(j);
        int demand = group_connectivity_matrix_[i][j];
        while (demand > 0) {
          if (iter_src->second.size() == 0 || iter_dst->second.size() == 0) {
              break;
          }
          int src_index = 0;
          int dst_index = 0;
          if (!deterministic) {
            
            src_index = rand() % iter_src->second.size();
            dst_index = rand() % iter_dst->second.size();
          } 
          switch_id src_switch = (iter_src->second)[src_index];
          switch_id dst_switch = (iter_dst->second)[dst_index];
          connect_switches(src_switch, last_used_outport[src_switch], dst_switch, last_used_inport[dst_switch], Optical);
          last_used_outport[src_switch]++;
          last_used_inport[dst_switch]++;

          (iter_src->second).erase((iter_src->second).begin() + src_index);
          (iter_dst->second).erase((iter_dst->second).begin() + dst_index);
          demand--; 
        }
      }
    }
  };

  void flexfly_topology_simplified::route_topology() {
    std::cout << "got into route_topology" << std::endl;
    routing_table_.resize(max_switch_id_ + 1);
    for (int i = 0; i <= max_switch_id_; i++) {
      routing_table_[i].resize(max_switch_id_ + 1);
      int group = group_from_swid(i);
      std::cout << "routing in switch:" << std::to_string(i) << std::endl;
      route_individual_switch(i);
    }
    std::cout << "got out of route_topology" << std::endl;
  };

  void flexfly_topology_simplified::route_individual_switch(switch_id src) {
    std::queue<switch_id> q;
    std::vector<bool> visited(max_switch_id_ + 1);
    std::vector<int> distance(max_switch_id_ + 1);
    std::vector<switch_id> parent(max_switch_id_ + 1);
    std::fill(distance.begin(), distance.end(), INT_MAX);
    std::fill(visited.begin(), visited.end(), false);
    q.push(src);
    distance[src] = 0;
    std::srand(1000);
    while (!q.empty()) {
      switch_id curr = q.front();
      q.pop();
      auto iter_vect = switch_outport_connection_map_.find(curr);
      for (auto conn : iter_vect->second) {
        

        if (distance[conn->dest_sid] > distance[curr] + 1) {
          distance[conn->dest_sid] = distance[curr] + 1;
          parent[conn->dest_sid] = curr;
        } else if (distance[conn->dest_sid] == distance[curr] + 1) {
          int decider = rand() % 2;
          if (decider) {
            parent[conn->dest_sid] = curr;
          }
        }
        if (!visited[conn->dest_sid] && (ignore_groups_.find(group_from_swid(conn->dest_sid)) == ignore_groups_.end())) {
          q.push(conn->dest_sid);
        }
      }
      visited[curr] = true;
    }
    // postprocessing of the routing_table_
    for (int i = 0; i <= max_switch_id_; i++) {
      if (i == src || !visited[i]) continue;
      switch_id curr_id = i;
      switch_id parent_id = parent[i];
      while (parent_id != src) {
        curr_id = parent_id;
        parent_id = parent[curr_id];
      }
      routing_table_[src][i] = curr_id;
    }
  };

}
}

