

// to build:  g++ ./main.cc -W -Wall -o dykstra_ecmp

#include<queue>
#include<set>
#include<map>
#include<vector>
#include<cstdio>
#include<climits>
#include<cstdint>
#include<iostream>
#include<sstream>
#include<functional>
#include<algorithm>
#include<sys/time.h>

#undef DEBUG

// node id
using NodeId = std::uint16_t;

// weight/cost
using WeightType = float;

// max/infinity
const WeightType MAX_WEIGHT = 0xFFFF;
const WeightType DEF_WEIGHT = 1.0;
const WeightType DEF_BIAS   = 0.0;

template <typename T>
inline std::string containerToString(const T & items)
 {
   std::stringstream ss;
   size_t n = 0;
   for(const auto & e : items)
     {
       if(n++ == 0)
          ss << e;
       else
          ss << ", " << e;
     }
    return ss.str();
  }


class WeightEntry
{
 public:
   WeightEntry(WeightType weight, WeightType bias = 0):
    weight_{weight},
    bias_{bias}
   { }

   WeightEntry():
    weight_{},
    bias_{}
   { }

   WeightType getWeight() const
    {  return weight_ + bias_; }

   void setWeight(const WeightType weight)
    {  weight_ = weight; }

   void setBias(const WeightType bias)
    {  bias_ = bias; }

   bool operator > (const WeightEntry & other) const
    {
      return (getWeight() > other.getWeight());
    }

 private:
   WeightType weight_, bias_;
};


using Weights = std::map<NodeId, WeightEntry>;
using NodeIds = std::set<NodeId>;
using Parents = std::map<NodeId, NodeIds>;
using Path    = std::vector<NodeId>;
using Paths   = std::vector<Path>;

using Link    = std::pair<WeightEntry, NodeId>;
using Links   = std::vector<Link>;
struct LinkCmp {
   bool operator() (const Link & a, const Link & b) const
    {
      return (a.first.getWeight() > b.first.getWeight());
    }
 };


std::string toString(const Parents & parents)
 {
   std::stringstream ss;
   ss << "parents:\n";

   for(const auto & [nodeId, nodeIds] : parents)
    {
       if(nodeIds.empty() == true)
        {
           ss << "\tnone" << std::endl;
        }
       else
        {
           ss << "\tu: " << nodeId  << " = {" << containerToString<NodeIds>(nodeIds) << "}" << std::endl;
        }
    }

   ss << "\n--------------------------------\n";
   return ss.str();
 }

class Topology 
{ 
  public: 
    Topology();

    void addEdge(NodeId u, NodeId v, WeightType wv, WeightType bias = 0.0); 

    void addEdge(NodeId u, NodeId v, WeightType wv, WeightType wu, WeightType bias = 0.0); 

    void getCosts(); 

    void getCost(const NodeId s); 

    const NodeIds & getDestinations() const;

    void findAllPaths(NodeId curr,
                      NodeId start,
                      const Parents & parents,
                      Path & currentPath, 
                      Paths & allPaths);

  private:
    std::map<NodeId, Links> topology_; 

    NodeIds destinations_;
}; 


Topology::Topology()
{ } 


void Topology::addEdge(NodeId u, 
                       NodeId v, 
                       WeightType wv,
                       WeightType wu,
                       WeightType bias) 
{ 
   // link cost u --> v
   topology_[u].emplace_back(WeightEntry{wv, bias}, v); 

   // link cost v --> u
   topology_[v].emplace_back(WeightEntry{wu, bias}, u); 

   destinations_.insert(v);
   destinations_.insert(u);
} 


void Topology::addEdge(NodeId u, 
                       NodeId v, 
                       WeightType wv,
                       WeightType bias)
{ 
   // link cost u --> v
   topology_[u].emplace_back(WeightEntry{wv, bias}, v); 

   destinations_.insert(v);
} 


void Topology::getCosts() 
{ 
   for(auto & entry : topology_)
    {
      getCost(entry.first);
    }
}


const NodeIds & Topology::getDestinations() const
{  return destinations_; }


void Topology::getCost(const NodeId src) 
{ 
   if(topology_.count(src) == 0)
    {
      return;
    }

   // priority queue
   std::priority_queue<Link, Links, LinkCmp> priorityQueue;

   // for multipath
   Parents parents;

   // final cost to each node
   Weights distance; 

   for(const auto & dst : destinations_)
    {      
       if(dst == src)
        {               
          // cost to self is 0
          distance[src] = WeightEntry{};
        }                         
       else                       
        {              
          // init cost to INF
          distance[dst] = WeightEntry{MAX_WEIGHT};
        }  
    }

   // insert self node with 0 weight to bootstrap the search
   priorityQueue.emplace(WeightType{0}, src); 

   // search adjacent links
   while(!priorityQueue.empty()) 
    { 
      const auto d = priorityQueue.top().first; 
      const auto u = priorityQueue.top().second; 

      priorityQueue.pop(); 

      if(d > distance[u])
       {
         continue;
       }

      for(auto & edge : topology_[u]) 
       { 
         const auto w = edge.first.getWeight();
         const auto v = edge.second; 
         const auto du_w = distance[u].getWeight() + w;
         const auto dv   = distance[v].getWeight();

#ifdef DEBUG
         std::cout << "u= " << u << ", v= " << v << ", w= " << w << ", du_w= " << du_w << ", dv= " << dv << std::endl;
#endif

         // check for lower cost
         if(du_w < dv) 
           { 
              distance[v].setWeight(du_w);

              // add new vertex and continue search
              priorityQueue.emplace(du_w, v); 

              parents[v].clear();
              parents[v].insert(u);
           }
          else if(du_w == dv) 
           {
              parents[v].insert(u);
           }
        } 
     } 

 std::cout << toString(parents);

 for(const auto dst : destinations_)
  {
     Paths allPaths;
     Path  currentPath;

     if(src != dst)
      {
        findAllPaths(dst, src, parents, currentPath, allPaths);
                  
        std::cout << "src " << src << ", dst " << dst;

        if(allPaths.size() == 0)
         {
           std::cout << " no route";
         }
        else
         {
           for(const auto & path : allPaths)
            {             
              for(size_t i = 0; i < path.size(); ++i)
               {
                  if(i == 0)
                    std::cout << "\n\t";

                  std::cout << path[i] << (i == path.size() - 1 ? "" : " -> ");
               }
            }
         }   

       std::cout << "\n--------------------------------\n";
     }
  }
} 


void Topology::findAllPaths(NodeId curr,
                            NodeId start,
                            const Parents & parents,
                            Path & currentPath, 
                            Paths & allPaths)
{
    currentPath.push_back(curr);

    if (curr == start)
    {
        // Reverse path to get it from start to destination
        auto path = currentPath;
        std::reverse(path.begin(), path.end());
        allPaths.push_back(path);
    }
    else
    {
        // Recur for all predecessors
        const auto it = parents.find(curr);
        if (it != parents.end())
        {
            for (const auto & parent : it->second)
            {
                findAllPaths(parent, start, parents, currentPath, allPaths);
            }
        }
    }
    currentPath.pop_back(); // backtrack
}



int main(void) 
{ 
   Topology topology; 

/*  topology
             3 - 4 ----- 9
            /   /       /
       1 - 2 - 5 - 6 - 7   99 - 11
            \ /         \  /
             8           10 
*/
   topology.addEdge(1, 2, DEF_WEIGHT);        

   topology.addEdge(2, 3, DEF_WEIGHT); 

   topology.addEdge(2, 5, DEF_WEIGHT, DEF_WEIGHT, DEF_BIAS); 

   topology.addEdge(2, 8, DEF_WEIGHT);

   topology.addEdge(3, 4, DEF_WEIGHT); 

   topology.addEdge(5, 4, DEF_WEIGHT);

   topology.addEdge(4, 9, DEF_WEIGHT);

   topology.addEdge(5, 6, DEF_WEIGHT); 

   topology.addEdge(6, 7, DEF_WEIGHT);

   topology.addEdge(5, 8, DEF_WEIGHT, DEF_WEIGHT, DEF_BIAS); 

   topology.addEdge(7, 9, DEF_WEIGHT, DEF_WEIGHT, DEF_BIAS); 

   topology.addEdge(7, 10, DEF_WEIGHT); 

   topology.addEdge(10, 99, DEF_WEIGHT);

   topology.addEdge(99, 11, DEF_WEIGHT);
 
   timeval t1, t2, t3;
 
   gettimeofday(&t1, NULL);

   topology.getCosts();
   
   gettimeofday(&t2, NULL);

   timersub(&t2, &t1, &t3);

   printf("time %lu:%06lu\n", t3.tv_sec, t3.tv_usec);

   return 0; 
} 

