#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/link-monitor-module.h"
#include "ns3/traffic-control-module.h"
#include "ns3/tcp-resequence-buffer.h"
#include "ns3/gnuplot.h"
#include "ns3/flow-id-tag.h"
#include "ns3/ipv4-flow-probe.h"
#include "ns3/netanim-module.h"
#include "ns3/cdf.h"
#include "ns3/ipv4-tlb-probing.h"

#include <vector>
#include <list>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <map>
#include <utility>
#include <set>
#include <string>

class Flow
{
   public:
      int src;   // srcID
      int dest;  // destID
      uint16_t port;
      uint32_t size;
      double start;  //startTime
      Flow(int src,int dest,uint16_t port,uint32_t size,double start);
};

Flow::Flow(int src,int dest,uint16_t port,uint32_t size,double start){
    this->src=src;
    this->dest=dest;
    this->port=port;
    this->size=size;
    this->start=start;
}

#define LINK_CAPACITY_BASE    1000000000          // 1Gbps
#define BUFFER_SIZE 600                           // 250 packets

#define RED_QUEUE_MARKING 65 		        	  // 65 Packets (available only in DcTcp)

// The flow port range, each flow will be assigned a random port number within this range
#define PORT_START 10000
#define PORT_END 50000

#define PACKET_SIZE 1400

#define PRESTO_RATIO 10

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("flowgenerator");

// Port from Traffic Generator
// Acknowledged to https://github.com/HKUST-SING/TrafficGenerator/blob/master/src/common/common.c
double poission_gen_interval(double avg_rate)
{
    if (avg_rate > 0)
       return -logf(1.0 - (double)rand() / RAND_MAX) / avg_rate;
    else
       return 0;
}

template<typename T>
T rand_range (T min, T max)
{
    return min + ((double)max - min) * rand () / RAND_MAX;
}

std::vector<Flow> flows;

std::ofstream out ("flow.txt", std::ios::out|std::ios::app);

void install_applications (int fromLeafId, NodeContainer servers, double requestRate, struct cdf_table *cdfTable,
        long &flowCount, long &totalFlowSize, int SERVER_COUNT, int LEAF_COUNT, double START_TIME, double END_TIME, double FLOW_LAUNCH_END_TIME, uint32_t applicationPauseThresh, uint32_t applicationPauseTime)
{
    NS_LOG_INFO ("Install applications:");
    for (int i = 0; i < SERVER_COUNT; i++)
    {
        int fromServerIndex = fromLeafId * SERVER_COUNT + i;

        double startTime = START_TIME + poission_gen_interval (requestRate);
        //std::cout<<startTime<<std::endl;
        while (startTime < FLOW_LAUNCH_END_TIME)
        {
            flowCount ++;
            uint16_t port = rand_range (PORT_START, PORT_END);

            int destServerIndex = fromServerIndex;
	    while (destServerIndex >= fromLeafId * SERVER_COUNT && destServerIndex < fromLeafId * SERVER_COUNT + SERVER_COUNT)
            {
		    destServerIndex = rand_range (0, SERVER_COUNT * LEAF_COUNT);
            }

	    Ptr<Node> destServer = servers.Get (destServerIndex);
	    Ptr<Ipv4> ipv4 = destServer->GetObject<Ipv4> ();
	    Ipv4InterfaceAddress destInterface = ipv4->GetAddress (1,0);
	    Ipv4Address destAddress = destInterface.GetLocal ();

            BulkSendHelper source ("ns3::TcpSocketFactory", InetSocketAddress (destAddress, port));
            uint32_t flowSize = gen_random_cdf (cdfTable);

            totalFlowSize += flowSize;
 	    source.SetAttribute ("SendSize", UintegerValue (PACKET_SIZE));
            source.SetAttribute ("MaxBytes", UintegerValue(flowSize));
            source.SetAttribute ("DelayThresh", UintegerValue (applicationPauseThresh));
            source.SetAttribute ("DelayTime", TimeValue (MicroSeconds (applicationPauseTime)));

            // Install apps
            ApplicationContainer sourceApp = source.Install (servers.Get (fromServerIndex));
            sourceApp.Start (Seconds (startTime));
            sourceApp.Stop (Seconds (END_TIME));

            // Install packet sinks
            PacketSinkHelper sink ("ns3::TcpSocketFactory",InetSocketAddress (Ipv4Address::GetAny (), port));
            ApplicationContainer sinkApp = sink.Install (servers. Get (destServerIndex));
            sinkApp.Start (Seconds (START_TIME));
            sinkApp.Stop (Seconds (END_TIME));

            //out<<fromServerIndex<<","<<destServerIndex<<","<<port<<","<<flowSize<<","<<startTime<<std::endl;
            //out<<fromServerIndex<<" "<<destServerIndex<<" "<<port<<" "<<flowSize<<" "<<startTime<<std::endl;
            Flow f(fromServerIndex,destServerIndex,port,flowSize,startTime);
            flows.push_back(f);
            //std::cout<<"Flow from server: " << fromServerIndex << " to server: "<< destServerIndex << " on port: " << port << " with flow size: "<< flowSize << "start time: " <<startTime<<std::endl;
            startTime += poission_gen_interval (requestRate);
        }
    }
}

int main (int argc, char *argv[])
{
#if 1
    //LogComponentEnable ("flowgenerator", LOG_LEVEL_INFO);
#endif

    // Command line parameters parsing
    std::string id = "0";
    unsigned randomSeed = 0;
    std::string cdfFileName = "examples/load-balance/VL2_CDF.txt";
    double load = 0.7;
    std::string transportProt = "Tcp";

    // The simulation starting and ending time
    double START_TIME = 0.0;
    double END_TIME = 0.1;

    double FLOW_LAUNCH_END_TIME = 0.1;

    uint32_t linkLatency = 10;

    bool asymCapacity = false;

    uint32_t asymCapacityPoss = 40;  // 40 %

    int SERVER_COUNT = 4;
    int SPINE_COUNT = 8;
    int LEAF_COUNT = 8;
    int LINK_COUNT = 1;

    uint64_t spineLeafCapacity = 10;
    uint64_t leafServerCapacity = 10;

    uint32_t applicationPauseThresh = 0;
    uint32_t applicationPauseTime = 1000;

    bool enableLargeDupAck = false;

    bool enableRandomDrop = false;
    double randomDropRate = 0.005; // 0.5%

    uint32_t blackHoleMode = 0; // When the black hole is enabled, the
    std::string blackHoleSrcAddrStr = "10.1.1.1";
    std::string blackHoleSrcMaskStr = "255.255.255.0";
    std::string blackHoleDestAddrStr = "10.1.2.0";
    std::string blackHoleDestMaskStr = "255.255.255.0";

    bool asymCapacity2 = false;

    bool enableLargeSynRetries = false;

    bool enableLargeDataRetries = false;

    bool enableFastReConnection = false;

    CommandLine cmd;
    cmd.AddValue ("ID", "Running ID", id);
    cmd.AddValue ("StartTime", "Start time of the simulation", START_TIME);
    cmd.AddValue ("EndTime", "End time of the simulation", END_TIME);
    cmd.AddValue ("FlowLaunchEndTime", "End time of the flow launch period", FLOW_LAUNCH_END_TIME);
    cmd.AddValue ("randomSeed", "Random seed, 0 for random generated", randomSeed);
    cmd.AddValue ("cdfFileName", "File name for flow distribution", cdfFileName);
    cmd.AddValue ("load", "Load of the network, 0.0 - 1.0", load);
    cmd.AddValue ("transportProt", "Transport protocol to use: Tcp, DcTcp", transportProt);
    cmd.AddValue ("linkLatency", "Link latency, should be in MicroSeconds", linkLatency);

    cmd.AddValue ("asymCapacity", "Whether the capacity is asym, which means some link will have only 1/10 the capacity of others", asymCapacity);
    cmd.AddValue ("asymCapacityPoss", "The possibility that a path will have only 1/10 capacity", asymCapacityPoss);

    cmd.AddValue ("serverCount", "The Server count", SERVER_COUNT);
    cmd.AddValue ("spineCount", "The Spine count", SPINE_COUNT);
    cmd.AddValue ("leafCount", "The Leaf count", LEAF_COUNT);
    cmd.AddValue ("linkCount", "The Link count", LINK_COUNT);

    cmd.AddValue ("spineLeafCapacity", "Spine <-> Leaf capacity in Gbps", spineLeafCapacity);
    cmd.AddValue ("leafServerCapacity", "Leaf <-> Server capacity in Gbps", leafServerCapacity);

    cmd.AddValue ("applicationPauseThresh", "How many packets can pass before we have delay, 0 for disable", applicationPauseThresh);
    cmd.AddValue ("applicationPauseTime", "The time for a delay, in MicroSeconds", applicationPauseTime);

    cmd.AddValue ("enableLargeDupAck", "Whether to set the ReTxThreshold to a very large value to mask reordering", enableLargeDupAck);

    cmd.AddValue ("enableRandomDrop", "Whether the Spine-0 to other leaves has the random drop problem", enableRandomDrop);
    cmd.AddValue ("randomDropRate", "The random drop rate when the random drop is enabled", randomDropRate);

    cmd.AddValue ("blackHoleMode", "The packet black hole mode, 0 to disable, 1 src, 2 dest, 3 src/dest pair", blackHoleMode);
    cmd.AddValue ("blackHoleSrcAddr", "The packet black hole source address", blackHoleSrcAddrStr);
    cmd.AddValue ("blackHoleSrcMask", "The packet black hole source mask", blackHoleSrcMaskStr);
    cmd.AddValue ("blackHoleDestAddr", "The packet black hole destination address", blackHoleDestAddrStr);
    cmd.AddValue ("blackHoleDestMask", "The packet black hole destination mask", blackHoleDestMaskStr);

    cmd.AddValue ("asymCapacity2", "Whether the Spine0-Leaf0's capacity is asymmetric", asymCapacity2);

    cmd.AddValue ("enableLargeSynRetries", "Whether the SYN packet would retry thousands of times", enableLargeSynRetries);
    cmd.AddValue ("enableFastReConnection", "Whether the SYN gap will be very small when reconnecting", enableFastReConnection);
    cmd.AddValue ("enableLargeDataRetries", "Whether the data retransmission will be more than 6 times", enableLargeDataRetries);

    cmd.Parse (argc, argv);

    uint64_t SPINE_LEAF_CAPACITY = spineLeafCapacity * LINK_CAPACITY_BASE;
    uint64_t LEAF_SERVER_CAPACITY = leafServerCapacity * LINK_CAPACITY_BASE;
    Time LINK_LATENCY = MicroSeconds (linkLatency);

    Ipv4Address blackHoleSrcAddr = Ipv4Address (blackHoleSrcAddrStr.c_str ());
    Ipv4Mask blackHoleSrcMask = Ipv4Mask (blackHoleSrcMaskStr.c_str ());
    Ipv4Address blackHoleDestAddr = Ipv4Address (blackHoleDestAddrStr.c_str ());
    Ipv4Mask blackHoleDestMask = Ipv4Mask (blackHoleDestMaskStr.c_str ());


    if (load < 0.0 || load >= 1.0)
    {
        NS_LOG_ERROR ("The network load should within 0.0 and 1.0");
        return 0;
    }

    if (transportProt.compare ("DcTcp") == 0)
    {
	NS_LOG_INFO ("Enabling DcTcp");
        Config::SetDefault ("ns3::TcpL4Protocol::SocketType", TypeIdValue (TcpDCTCP::GetTypeId ()));
        Config::SetDefault ("ns3::RedQueueDisc::Mode", StringValue ("QUEUE_MODE_BYTES"));
    	Config::SetDefault ("ns3::RedQueueDisc::MeanPktSize", UintegerValue (PACKET_SIZE));
        Config::SetDefault ("ns3::RedQueueDisc::QueueLimit", UintegerValue (BUFFER_SIZE * PACKET_SIZE));
        //Config::SetDefault ("ns3::QueueDisc::Quota", UintegerValue (BUFFER_SIZE));
        Config::SetDefault ("ns3::RedQueueDisc::Gentle", BooleanValue (false));
    }

    NS_LOG_INFO ("Config parameters");
    Config::SetDefault ("ns3::TcpSocket::SegmentSize", UintegerValue(PACKET_SIZE));
    Config::SetDefault ("ns3::TcpSocket::DelAckCount", UintegerValue (0));
    if (enableFastReConnection)
    {
        Config::SetDefault ("ns3::TcpSocket::ConnTimeout", TimeValue (MicroSeconds (40)));
    }
    else
    {
        Config::SetDefault ("ns3::TcpSocket::ConnTimeout", TimeValue (MilliSeconds (5)));
    }
    Config::SetDefault ("ns3::TcpSocket::InitialCwnd", UintegerValue (10));
    Config::SetDefault ("ns3::TcpSocketBase::MinRto", TimeValue (MilliSeconds (5)));
    Config::SetDefault ("ns3::TcpSocketBase::ClockGranularity", TimeValue (MicroSeconds (100)));
    Config::SetDefault ("ns3::RttEstimator::InitialEstimation", TimeValue (MicroSeconds (80)));
    Config::SetDefault ("ns3::TcpSocket::SndBufSize", UintegerValue (160000000));
    Config::SetDefault ("ns3::TcpSocket::RcvBufSize", UintegerValue (160000000));

    if (enableLargeDupAck)
    {
        Config::SetDefault ("ns3::TcpSocketBase::ReTxThreshold", UintegerValue (1000));
    }

    if (enableLargeSynRetries)
    {
        Config::SetDefault ("ns3::TcpSocket::ConnCount", UintegerValue (10000));
    }

    if (enableLargeDataRetries)
    {
        Config::SetDefault ("ns3::TcpSocket::DataRetries", UintegerValue (10000));
    }

    NodeContainer spines;
    spines.Create (SPINE_COUNT);
    NodeContainer leaves;
    leaves.Create (LEAF_COUNT);
    NodeContainer servers;
    servers.Create (SERVER_COUNT * LEAF_COUNT);

    NS_LOG_INFO ("Install Internet stacks");
    InternetStackHelper internet;
    Ipv4GlobalRoutingHelper globalRoutingHelper;

    internet.SetRoutingHelper (globalRoutingHelper);
    Config::SetDefault ("ns3::Ipv4GlobalRouting::PerflowEcmpRouting", BooleanValue(true));
    NS_LOG_INFO("Enabling Per Flow ECMP");

    internet.Install (servers);
    internet.Install (spines);
    internet.Install (leaves);

    NS_LOG_INFO ("Install channels and assign addresses");

    PointToPointHelper p2p;
    Ipv4AddressHelper ipv4;

    TrafficControlHelper tc;
    if (transportProt.compare ("DcTcp") == 0)
    {
        tc.SetRootQueueDisc ("ns3::RedQueueDisc", "MinTh", DoubleValue (RED_QUEUE_MARKING * PACKET_SIZE),
                                                  "MaxTh", DoubleValue (RED_QUEUE_MARKING * PACKET_SIZE));
    }

    NS_LOG_INFO ("Configuring servers");
    // Setting servers
    p2p.SetDeviceAttribute ("DataRate", DataRateValue (DataRate (LEAF_SERVER_CAPACITY)));
    p2p.SetChannelAttribute ("Delay", TimeValue(LINK_LATENCY));
    if (transportProt.compare ("Tcp") == 0)
    {
     	p2p.SetQueue ("ns3::DropTailQueue", "MaxPackets", UintegerValue (BUFFER_SIZE));
    }
    else
    {
	p2p.SetQueue ("ns3::DropTailQueue", "MaxPackets", UintegerValue (10));
    }

    ipv4.SetBase ("10.1.0.0", "255.255.255.0");

    std::vector<Ipv4Address> leafNetworks (LEAF_COUNT);

    std::vector<Ipv4Address> serverAddresses (SERVER_COUNT * LEAF_COUNT);

    std::map<std::pair<int, int>, uint32_t> leafToSpinePath;
    std::map<std::pair<int, int>, uint32_t> spineToLeafPath;

    std::vector<Ptr<Ipv4TLBProbing> > probings (SERVER_COUNT * LEAF_COUNT);

    for (int i = 0; i < LEAF_COUNT; i++)
    {
	    Ipv4Address network = ipv4.NewNetwork ();
        leafNetworks[i] = network;

        for (int j = 0; j < SERVER_COUNT; j++)
        {
            int serverIndex = i * SERVER_COUNT + j;
            NodeContainer nodeContainer = NodeContainer (leaves.Get (i), servers.Get (serverIndex));
            NetDeviceContainer netDeviceContainer = p2p.Install (nodeContainer);

            if (transportProt.compare ("DcTcp") == 0)
            {
		    NS_LOG_INFO ("Install RED Queue for leaf: " << i << " and server: " << j);
	            tc.Install (netDeviceContainer);
            }
            Ipv4InterfaceContainer interfaceContainer = ipv4.Assign (netDeviceContainer);

            NS_LOG_INFO ("Leaf - " << i << " is connected to Server - " << j << " with address "
                    << interfaceContainer.GetAddress(0) << " <-> " << interfaceContainer.GetAddress (1)
                    << " with port " << netDeviceContainer.Get (0)->GetIfIndex () << " <-> " << netDeviceContainer.Get (1)->GetIfIndex ());

            serverAddresses [serverIndex] = interfaceContainer.GetAddress (1);
            if (transportProt.compare ("Tcp") == 0)
            {
                tc.Uninstall (netDeviceContainer);
            }
        }
    }

    NS_LOG_INFO ("Configuring switches");
    // Setting up switches
    p2p.SetDeviceAttribute ("DataRate", DataRateValue (DataRate (SPINE_LEAF_CAPACITY)));
    std::set<std::pair<uint32_t, uint32_t> > asymLink; // set< (A, B) > Leaf A -> Spine B is asymmetric

    for (int i = 0; i < LEAF_COUNT; i++)
    {
        for (int j = 0; j < SPINE_COUNT; j++)
        {

        for (int l = 0; l < LINK_COUNT; l++)
        {
            bool isAsymCapacity = false;

            if (asymCapacity && static_cast<uint32_t> (rand () % 100) < asymCapacityPoss)
            {
                isAsymCapacity = true;
            }

            if (asymCapacity2 && i == 0 && j ==0)
            {
                isAsymCapacity = true;
            }

            // TODO
            uint64_t spineLeafCapacity = SPINE_LEAF_CAPACITY;

            if (isAsymCapacity)
            {
                spineLeafCapacity = SPINE_LEAF_CAPACITY / 5;
                asymLink.insert (std::make_pair (i, j));
                asymLink.insert (std::make_pair (j, i));
            }

            p2p.SetDeviceAttribute ("DataRate", DataRateValue (DataRate (spineLeafCapacity)));
            ipv4.NewNetwork ();

            NodeContainer nodeContainer = NodeContainer (leaves.Get (i), spines.Get (j));
            NetDeviceContainer netDeviceContainer = p2p.Install (nodeContainer);
	    if (transportProt.compare ("DcTcp") == 0)
            {
		NS_LOG_INFO ("Install RED Queue for leaf: " << i << " and spine: " << j);
                if (enableRandomDrop)
                {
                    if (j == 0)
                    {
                        Config::SetDefault ("ns3::RedQueueDisc::DropRate", DoubleValue (0.0));
                        tc.Install (netDeviceContainer.Get (0)); // Leaf to Spine Queue
                        Config::SetDefault ("ns3::RedQueueDisc::DropRate", DoubleValue (randomDropRate));
                        tc.Install (netDeviceContainer.Get (1)); // Spine to Leaf Queue
                    }
                    else
                    {
                        Config::SetDefault ("ns3::RedQueueDisc::DropRate", DoubleValue (0.0));
	                tc.Install (netDeviceContainer);
                    }
                }
                else if (blackHoleMode != 0)
                {
                    if (j == 0)
                    {
                        Config::SetDefault ("ns3::RedQueueDisc::BlackHole", UintegerValue (0));
                        tc.Install (netDeviceContainer.Get (0)); // Leaf to Spine Queue
                        Config::SetDefault ("ns3::RedQueueDisc::BlackHole", UintegerValue (blackHoleMode));
                        tc.Install (netDeviceContainer.Get (1)); // Spine to Leaf Queue
                        Ptr<TrafficControlLayer> tc = netDeviceContainer.Get (1)->GetNode ()->GetObject<TrafficControlLayer> ();
                        Ptr<QueueDisc> queueDisc = tc->GetRootQueueDiscOnDevice (netDeviceContainer.Get (1));
                        Ptr<RedQueueDisc> redQueueDisc = DynamicCast<RedQueueDisc> (queueDisc);
                        redQueueDisc->SetBlackHoleSrc (blackHoleSrcAddr, blackHoleSrcMask);
                        redQueueDisc->SetBlackHoleDest (blackHoleDestAddr, blackHoleDestMask);
                    }
                    else
                    {
                        Config::SetDefault ("ns3::RedQueueDisc::BlackHole", UintegerValue (0));
                        tc.Install (netDeviceContainer);
                    }
                }
                else
                {
	                tc.Install (netDeviceContainer);
                }
            }
            Ipv4InterfaceContainer ipv4InterfaceContainer = ipv4.Assign (netDeviceContainer);
            NS_LOG_INFO ("Leaf - " << i << " is connected to Spine - " << j << " with address "
                    << ipv4InterfaceContainer.GetAddress(0) << " <-> " << ipv4InterfaceContainer.GetAddress (1)
                    << " with port " << netDeviceContainer.Get (0)->GetIfIndex () << " <-> " << netDeviceContainer.Get (1)->GetIfIndex ()
                    << " with data rate " << spineLeafCapacity);

            if (transportProt.compare ("Tcp") == 0)
            {
                tc.Uninstall (netDeviceContainer);
            }
        }
        }
    }


    NS_LOG_INFO ("Populate global routing tables");
    Ipv4GlobalRoutingHelper::PopulateRoutingTables ();

    double oversubRatio = static_cast<double>(SERVER_COUNT * LEAF_SERVER_CAPACITY) / (SPINE_LEAF_CAPACITY * SPINE_COUNT * LINK_COUNT);
    NS_LOG_INFO ("Over-subscription ratio: " << oversubRatio);

    NS_LOG_INFO ("Initialize CDF table");
    struct cdf_table* cdfTable = new cdf_table ();
    init_cdf (cdfTable);
    load_cdf (cdfTable, cdfFileName.c_str ());

    NS_LOG_INFO ("Calculating request rate");
    double requestRate = load * LEAF_SERVER_CAPACITY * SERVER_COUNT / oversubRatio / (8 * avg_cdf (cdfTable)) / SERVER_COUNT;
    NS_LOG_INFO ("Average request rate: " << requestRate << " per second");

    NS_LOG_INFO ("Initialize random seed: " << randomSeed);
    if (randomSeed == 0)
    {
        srand ((unsigned)time (NULL));
    }
    else
    {
        srand (randomSeed);
    }

    NS_LOG_INFO ("Create applications");

    long flowCount = 0;
    long totalFlowSize = 0;

    for (int fromLeafId = 0; fromLeafId < LEAF_COUNT; fromLeafId ++)
    {
        install_applications(fromLeafId, servers, requestRate, cdfTable, flowCount, totalFlowSize, SERVER_COUNT, LEAF_COUNT, START_TIME, END_TIME, FLOW_LAUNCH_END_TIME, applicationPauseThresh, applicationPauseTime);
    }
    
   std::cout<<flowCount<<std::endl;

    for(std::size_t i=0;i<flows.size();i++){
       // std::cout<<flows[i].src<<" "<<flows[i].dest<<" "<<flows[i].port<<" "<<flows[i].size<<" "<<flows[i].start<<std::endl;
    }    
    std::cout<<"\n"<<std::endl;
    for(std::size_t i=0;i<flows.size()-1;i++){
        for(std::size_t j=0;j<flows.size()-1-i;j++){
            if(flows[j].start>flows[j+1].start){
                std::swap(flows[j],flows[j+1]);
            }
        }
    }
    for(std::size_t i=0;i<flows.size();i++){
        out<<flows[i].src<<" "<<flows[i].dest<<" "<<flows[i].port<<" "<<flows[i].size<<" "<<flows[i].start<<std::endl;
        //std::cout<<flows[i].src<<" "<<flows[i].dest<<" "<<flows[i].port<<" "<<flows[i].size<<" "<<flows[i].start<<std::endl;
    }
    out.close();
     
}


