#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/traffic-control-module.h"
#include "ns3/gnuplot.h"
#include "ns3/flow-id-tag.h"
#include "ns3/ipv4-flow-probe.h"
#include "ns3/netanim-module.h"
#include "ns3/cdf.h"

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
#include <stdlib.h>
#include <stdio.h>
#include <time.h>

#define LINK_CAPACITY_BASE    1000000000          // 1Gbps
#define BUFFER_SIZE 600                           // 250 packets

#define RED_QUEUE_MARKING 65 		        	  // 65 Packets (available only in DcTcp)

// The flow port range, each flow will be assigned a random port number within this range
#define PORT_START 10000
#define PORT_END 50000

#define PACKET_SIZE 1400

#define PRESTO_RATIO 10

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("wcmp");

class Box
{
   public:
      int start_time;    
      int end_time1;      
      int delay1;
      int pythondelay;
      int end_time2;
      int delay2; 
      int all;
      Box(int,int,int,int,int,int,int);
};
Box::Box(int s,int e1,int d1,int pd,int e2,int d2,int a):start_time(s),end_time1(e1),delay1(d1),pythondelay(pd),end_time2(e2),delay2(d2),all(a) {}
std::vector<Box> box;

//server send flow information 回调函数
void 
send(Ptr<Socket> sock,std::string str)
{ 
//   uint8_t buffer[sizeof(str)] ;//也可以根据字符串大小创建包
  uint8_t buffer[255] ;
  uint32_t len = str.length();
  for(uint32_t i=0;i<len;i++)
  {
    buffer[i]=str[i];//char 与 uint_8逐个赋值
  }
  buffer[len]='\0';
  Ptr<Packet> p = Create<Packet>(buffer,sizeof(buffer));//把buffer写入到包内
  sock->Send(p);
//  std::cout<<"send"<<std::endl;
} 

//host receive flow information 回调函数
NodeContainer servers;
NodeContainer cs;
Ipv4InterfaceContainer csi;
std::vector<Ipv4Address> serverAddresses (16);
void
CSRecvString(Ptr<Socket> sock)
{
    Address from;
    Ptr<Packet> packet = sock->RecvFrom (from);
    packet->RemoveAllPacketTags ();
    packet->RemoveAllByteTags ();
    InetSocketAddress address = InetSocketAddress::ConvertFrom (from);

    // uint8_t data[sizeof(packet)];
    uint8_t data[255];
    packet->CopyData(data,sizeof(data));//将包内数据写入到data内
    std::cout <<sock->GetNode()->GetId()<<" "<<"receive : '" << data <<"' from "<<address.GetIpv4()<< std::endl; 
    char a[sizeof(data)];
    for(uint32_t i=0;i<sizeof(data);i++){
        a[i]=data[i];
    }
    std::string strres = std::string(a);
    std::istringstream iss(strres);
    int sid;
    double start;
    iss >> sid >>start;
 //   std::cout<<sid<<" "<<start<<std::endl;
    Time start_time = Seconds(start);
    Time end_time1 = Simulator::Now();
    int delay1 = end_time1.GetMicroSeconds()-start_time.GetMicroSeconds();
    std::cout<<"server to cs delay:"<<delay1<<"microseconds"<<std::endl;

    /*call python and read python delay*/
    char result[100];
    FILE *fp;
    std::string cmd = "python3 -W ignore 1.py";
    fp = popen(cmd.c_str(),"r");
    if(fp == NULL){
        std::cout << "ERROR"  << std::endl;
        exit(1); 
    }
    fgets(result,sizeof(result)-1,fp);
//    std::cout << "result:"<<result<<std::endl;
    std::string resultstring = std::string(result);
    pclose(fp); 
    std::istringstream is(resultstring);
    double p;
    is >> p;
    Time pythondelay = Seconds(p);
    int pdelay = pythondelay.GetMicroSeconds();
    std::cout << "pythondelay:"<<pdelay<<std::endl;

    //cs send action to server sid
    TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");
    Ptr<Socket> Send_sock = Socket::CreateSocket(cs.Get(0), tid);
    InetSocketAddress RecvAddr = InetSocketAddress(serverAddresses[sid], 10);
    Send_sock->Connect(RecvAddr);
    std::string action = std::to_string(start_time.GetMicroSeconds())+ " " + std::to_string(delay1) +" "+std::to_string(pdelay)+" "+std::to_string(end_time1.GetMicroSeconds());
    Simulator::Schedule(Simulator::Now(),&send,Send_sock,action);  
}


void
ServerRecvString(Ptr<Socket> sock)//回调函数
{
    Address from;
    Ptr<Packet> packet = sock->RecvFrom (from);
    packet->RemoveAllPacketTags ();
    packet->RemoveAllByteTags ();
    InetSocketAddress address = InetSocketAddress::ConvertFrom (from);

    // uint8_t data[sizeof(packet)];
    uint8_t data[255];
    packet->CopyData(data,sizeof(data));//将包内数据写入到data内
    int start_time,delay1,pythondelay,end_time1;
    std::cout <<sock->GetNode()->GetId()<<" "<<"receive : '" << data <<"' from "<<address.GetIpv4 ()<<std::endl;  
    char a[sizeof(data)];
    for(uint32_t i=0;i<sizeof(data);i++){
        a[i]=data[i];
    }
    std::string strres = std::string(a);
    std::istringstream iss(strres);
    iss >> start_time >>delay1>>pythondelay>>end_time1;
    Time end_time2 = Simulator::Now();
    int delay2 = end_time2.GetMicroSeconds()-end_time1;
    int all = end_time2.GetMicroSeconds()-start_time;
    std::ofstream outfile; 
    outfile.open("result.txt", std::ios::app);
    if(!outfile.is_open ())
        std::cout << "Open file failure" << std::endl;
    outfile <<delay1<<"\t"<<pythondelay<<"\t"<<delay2<<"\t"<<all<<std::endl; 

}


int main (int argc, char *argv[])
{
#if 1
   // LogComponentEnable ("wcmp", LOG_LEVEL_ALL);
#endif
    // Command line parameters parsing
    std::string id = "0";
    unsigned randomSeed = 0;
    std::string transportProt = "Tcp";

    // The simulation starting and ending time
    double START_TIME = 0.0;
    double END_TIME = 1000.0;

    double FLOW_LAUNCH_END_TIME = END_TIME;

    uint32_t linkLatency = 10;

    int SERVER_COUNT = 4;
    int SPINE_COUNT = 4;
    int LEAF_COUNT = 4;
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

    CommandLine cmd;
    cmd.AddValue ("ID", "Running ID", id);
    cmd.AddValue ("StartTime", "Start time of the simulation", START_TIME);
    cmd.AddValue ("EndTime", "End time of the simulation", END_TIME);
    cmd.AddValue ("FlowLaunchEndTime", "End time of the flow launch period", FLOW_LAUNCH_END_TIME);
    cmd.AddValue ("randomSeed", "Random seed, 0 for random generated", randomSeed);
    cmd.AddValue ("transportProt", "Transport protocol to use: Tcp, DcTcp", transportProt);
    cmd.AddValue ("linkLatency", "Link latency, should be in MicroSeconds", linkLatency);

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

    cmd.Parse (argc, argv);

    uint64_t SPINE_LEAF_CAPACITY = spineLeafCapacity * LINK_CAPACITY_BASE;
    uint64_t LEAF_SERVER_CAPACITY = leafServerCapacity * LINK_CAPACITY_BASE;
    Time LINK_LATENCY = MicroSeconds (linkLatency);

    Ipv4Address blackHoleSrcAddr = Ipv4Address (blackHoleSrcAddrStr.c_str ());
    Ipv4Mask blackHoleSrcMask = Ipv4Mask (blackHoleSrcMaskStr.c_str ());
    Ipv4Address blackHoleDestAddr = Ipv4Address (blackHoleDestAddrStr.c_str ());
    Ipv4Mask blackHoleDestMask = Ipv4Mask (blackHoleDestMaskStr.c_str ());

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
    Config::SetDefault ("ns3::TcpSocket::ConnTimeout", TimeValue (MilliSeconds (5)));
    Config::SetDefault ("ns3::TcpSocket::InitialCwnd", UintegerValue (10));
    Config::SetDefault ("ns3::TcpSocketBase::MinRto", TimeValue (MilliSeconds (5)));
    Config::SetDefault ("ns3::TcpSocketBase::ClockGranularity", TimeValue (MicroSeconds (100)));
    Config::SetDefault ("ns3::RttEstimator::InitialEstimation", TimeValue (MicroSeconds (80)));
    Config::SetDefault ("ns3::TcpSocket::SndBufSize", UintegerValue (160000000));
    Config::SetDefault ("ns3::TcpSocket::RcvBufSize", UintegerValue (160000000));

    NodeContainer spines;
    spines.Create (SPINE_COUNT);
    NodeContainer leaves;
    leaves.Create (LEAF_COUNT);
    //NodeContainer servers;
    servers.Create (SERVER_COUNT * LEAF_COUNT);

    cs.Create (1);

    NS_LOG_INFO ("Install Internet stacks");
    InternetStackHelper internet;
    Ipv4GlobalRoutingHelper globalRoutingHelper;
    internet.SetRoutingHelper (globalRoutingHelper);
    internet.Install (servers);
    internet.Install (spines);
    internet.Install (leaves);
    internet.Install (cs);

    Config::SetDefault ("ns3::Ipv4GlobalRouting::PerflowEcmpRouting", BooleanValue(true));
    NS_LOG_INFO("Enabling Per Flow ECMP");

    NS_LOG_INFO ("Install channels and assign addresses");

    PointToPointHelper p2p;
    Ipv4AddressHelper ipv4;

    //connect spine0 to central server
    NodeContainer cs_s0 = NodeContainer(cs.Get(0),spines.Get(0));
    NetDeviceContainer cs_s0_device = p2p.Install(cs_s0);
    Ipv4AddressHelper c_s_address;
    c_s_address.SetBase("10.1.73.0", "255.255.255.0");
    csi = c_s_address.Assign(cs_s0_device);

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
    std::map<std::pair<int, int>, uint32_t> leafToSpinePath;
    std::map<std::pair<int, int>, uint32_t> spineToLeafPath;
    
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

    for (int i = 0; i < LEAF_COUNT; i++)
    {
        for (int j = 0; j < SPINE_COUNT; j++)
        {

        for (int l = 0; l < LINK_COUNT; l++)
        {

            // TODO
            uint64_t spineLeafCapacity = SPINE_LEAF_CAPACITY;

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


    NS_LOG_INFO ("Initialize random seed: " << randomSeed);
    if (randomSeed == 0)
    {
        srand ((unsigned)time (NULL));
    }
    else
    {
        srand (randomSeed);
    }

    //-----------------FlowMonitor-DELAY---------------------

    //DelayMonitor(&flowHelper, flowMonitor);

    //set trace
    //Config::Connect("/NodeList/*/ApplicationList/*/$ns3::BulkSendApplication/Tx",MakeCallback(&sendPackets));
     //Config::Connect("/NodeList/*/ApplicationList/*/$ns3::PacketSink/Rx",MakeCallback (&recvPackets));
    //double period = 0.1;
    //Config::ConnectWithoutContext("/NodeList/*/ApplicationList/*/$ns3::PacketSink/Rx",MakeCallback (&ReceivePacket));
    //Simulator::Schedule(Seconds(period), &Throughput);
    //ThroughputMonitor(&flowHelper,flowMonitor,id);

    //CS接收端
    TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");
    Ptr<Socket> Recv_sock = Socket::CreateSocket(cs.Get (0), tid);
    InetSocketAddress addr = InetSocketAddress(csi.GetAddress(0), 9);
    Recv_sock->Bind(addr);
    Recv_sock->SetRecvCallback(MakeCallback(&CSRecvString)); 

    //server send to cs
    InetSocketAddress RecvAddr = InetSocketAddress(csi.GetAddress(0), 9);
    std::vector<Ptr<Socket>> Send_sock (SERVER_COUNT * LEAF_COUNT);
    for(int i=0;i<16;i++){
        Send_sock[i] = Socket::CreateSocket(servers.Get(i), tid);
        Send_sock[i]->Connect(RecvAddr);
    }
    double t = 0.01;
    for(int j =0;j<63;j++){
        for(int i=0;i<16;i++){
            std::string s= std::to_string(i)+" "+std::to_string(t);
            Simulator::Schedule(Seconds(t),&send,Send_sock[i],s); 
            t += 0.01;
        }
    }
    
    //server reciive action from cs
    //server0接收端
    Ptr<Socket> Recv_sock0 = Socket::CreateSocket(servers.Get(0), tid);
    InetSocketAddress addr0 = InetSocketAddress(serverAddresses[0], 10);
    Recv_sock0->Bind(addr0);
    Recv_sock0->SetRecvCallback(MakeCallback(&ServerRecvString));
    //server1接收端
    Ptr<Socket> Recv_sock1 = Socket::CreateSocket(servers.Get(1), tid);
    InetSocketAddress addr1 = InetSocketAddress(serverAddresses[1], 10);
    Recv_sock1->Bind(addr1);
    Recv_sock1->SetRecvCallback(MakeCallback(&ServerRecvString));
    //server2接收端
    Ptr<Socket> Recv_sock2 = Socket::CreateSocket(servers.Get(2), tid);
    InetSocketAddress addr2 = InetSocketAddress(serverAddresses[2], 10);
    Recv_sock2->Bind(addr2);
    Recv_sock2->SetRecvCallback(MakeCallback(&ServerRecvString));
    //server3接收端
    Ptr<Socket> Recv_sock3 = Socket::CreateSocket(servers.Get(3), tid);
    InetSocketAddress addr3 = InetSocketAddress(serverAddresses[3], 10);
    Recv_sock3->Bind(addr3);
    Recv_sock3->SetRecvCallback(MakeCallback(&ServerRecvString));
    //server4接收端
    Ptr<Socket> Recv_sock4 = Socket::CreateSocket(servers.Get(4), tid);
    InetSocketAddress addr4 = InetSocketAddress(serverAddresses[4], 10);
    Recv_sock4->Bind(addr4);
    Recv_sock4->SetRecvCallback(MakeCallback(&ServerRecvString));
    //server5接收端
    Ptr<Socket> Recv_sock5 = Socket::CreateSocket(servers.Get(5), tid);
    InetSocketAddress addr5 = InetSocketAddress(serverAddresses[5], 10);
    Recv_sock5->Bind(addr5);
    Recv_sock5->SetRecvCallback(MakeCallback(&ServerRecvString));
    //server6接收端
    Ptr<Socket> Recv_sock6 = Socket::CreateSocket(servers.Get(6), tid);
    InetSocketAddress addr6 = InetSocketAddress(serverAddresses[6], 10);
    Recv_sock6->Bind(addr6);
    Recv_sock6->SetRecvCallback(MakeCallback(&ServerRecvString));
    //server7接收端
    Ptr<Socket> Recv_sock7 = Socket::CreateSocket(servers.Get(7), tid);
    InetSocketAddress addr7 = InetSocketAddress(serverAddresses[7], 10);
    Recv_sock7->Bind(addr7);
    Recv_sock7->SetRecvCallback(MakeCallback(&ServerRecvString));
    //server8接收端
    Ptr<Socket> Recv_sock8 = Socket::CreateSocket(servers.Get(8), tid);
    InetSocketAddress addr8 = InetSocketAddress(serverAddresses[8], 10);
    Recv_sock8->Bind(addr8);
    Recv_sock8->SetRecvCallback(MakeCallback(&ServerRecvString));
    //server9接收端
    Ptr<Socket> Recv_sock9 = Socket::CreateSocket(servers.Get(9), tid);
    InetSocketAddress addr9 = InetSocketAddress(serverAddresses[9], 10);
    Recv_sock9->Bind(addr9);
    Recv_sock9->SetRecvCallback(MakeCallback(&ServerRecvString));
    //server10接收端
    Ptr<Socket> Recv_sock10 = Socket::CreateSocket(servers.Get(10), tid);
    InetSocketAddress addr10 = InetSocketAddress(serverAddresses[10], 10);
    Recv_sock10->Bind(addr10);
    Recv_sock10->SetRecvCallback(MakeCallback(&ServerRecvString));
    //server11接收端
    Ptr<Socket> Recv_sock11 = Socket::CreateSocket(servers.Get(11), tid);
    InetSocketAddress addr11 = InetSocketAddress(serverAddresses[11], 10);
    Recv_sock11->Bind(addr11);
    Recv_sock11->SetRecvCallback(MakeCallback(&ServerRecvString));
    //server12接收端
    Ptr<Socket> Recv_sock12 = Socket::CreateSocket(servers.Get(12), tid);
    InetSocketAddress addr12 = InetSocketAddress(serverAddresses[12], 10);
    Recv_sock12->Bind(addr12);
    Recv_sock12->SetRecvCallback(MakeCallback(&ServerRecvString));
    //server13接收端
    Ptr<Socket> Recv_sock13 = Socket::CreateSocket(servers.Get(13), tid);
    InetSocketAddress addr13 = InetSocketAddress(serverAddresses[13], 10);
    Recv_sock13->Bind(addr13);
    Recv_sock13->SetRecvCallback(MakeCallback(&ServerRecvString));
    //server14接收端
    Ptr<Socket> Recv_sock14 = Socket::CreateSocket(servers.Get(14), tid);
    InetSocketAddress addr14 = InetSocketAddress(serverAddresses[14], 10);
    Recv_sock14->Bind(addr14);
    Recv_sock14->SetRecvCallback(MakeCallback(&ServerRecvString));
    //server15接收端
    Ptr<Socket> Recv_sock15 = Socket::CreateSocket(servers.Get(15), tid);
    InetSocketAddress addr15 = InetSocketAddress(serverAddresses[15], 10);
    Recv_sock15->Bind(addr15);
    Recv_sock15->SetRecvCallback(MakeCallback(&ServerRecvString));
 
    NS_LOG_INFO ("Enabling flow monitor");

    Ptr<FlowMonitor> flowMonitor;
    FlowMonitorHelper flowHelper;
    flowMonitor = flowHelper.InstallAll();
    flowMonitor->CheckForLostPackets ();

    NS_LOG_INFO ("Start simulation");
    Simulator::Stop (Seconds (END_TIME));

    Simulator::Run ();

    flowMonitor->SerializeToXmlFile("result.xml", true, true);

    Simulator::Destroy ();
    NS_LOG_INFO ("Stop simulation");
}
