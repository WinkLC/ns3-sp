#include <iostream>
#include <fstream>
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/core-module.h"
#include "ns3/applications-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/point-to-point-module.h"

using namespace ns3;
NS_LOG_COMPONENT_DEFINE ("simpleTCP");

//global variables
static std::ofstream window;

static void
CwndTracer (uint32_t oldval , uint32_t newval)
{
    window << ns3::Simulator:: Now().GetSeconds() <<"" << newval /1024 <<std:: endl;
}

int main (int argc , char *argv[])
{
        LogComponentEnable ("simpleTCP", LOG_LEVEL_ALL);
        // Set up some default values for the simulation.
        Config::SetDefault ("ns3::OnOffApplication::PacketSize", UintegerValue (1024));
        Config::SetDefault ("ns3::OnOffApplication::DataRate", DataRateValue (2000000));
       // Config::SetDefault ("ns3::DropTailQueue::MaxPackets", UintegerValue (20));
        CommandLine cmd;
        cmd.Parse (argc , argv);
        window.open("window.tr");

        NS_LOG_INFO ("Creating nodes..");
        // We create three nodes
        Ptr <Node > n0 = CreateObject <Node > (); 
        Ptr <Node > n1 = CreateObject <Node > ();
        Ptr <Node > n2 = CreateObject <Node > ();
        NS_LOG_INFO ("CreatingTopology..");
	NodeContainer n0n1;
	NodeContainer n1n2;
	n0n1.Add(n0);
	n0n1.Add(n1);
	n1n2.Add(n1);
	n1n2.Add(n2);
	// Container for all nodes
	NodeContainer contAllNodes;
	contAllNodes.Add(n0);
	contAllNodes.Add(n1);
	contAllNodes.Add(n2);
	// Point2point network 1
	PointToPointHelper p2pNet1;
	p2pNet1.SetDeviceAttribute ("DataRate", DataRateValue (2000000));
	p2pNet1.SetChannelAttribute ("Delay", TimeValue ( MilliSeconds (5)));
	// Point2point network 2
	PointToPointHelper p2pNet2;
	p2pNet2. SetDeviceAttribute ("DataRate", DataRateValue (1000000));
	p2pNet2. SetChannelAttribute ("Delay", TimeValue (MilliSeconds (5)));
	// NetDeviceContainers
	NetDeviceContainer dev1 = p2pNet1.Install (n0n1);
	NetDeviceContainer dev2 = p2pNet2.Install (n1n2);
	// Install Internet stack in all nodes
	InternetStackHelper stack;
	stack.Install (contAllNodes);
	NS_LOG_INFO ("AddIPaddresses..");
	Ipv4AddressHelper ipv4;
	// First Network
	ipv4. SetBase (" 194.57.1.0", "255.255.255.0");
	Ipv4InterfaceContainer ipIfaceN0N1 = ipv4.Assign (dev1);
	// Second Network
	ipv4. SetBase (" 194.57.2.0", "255.255.255.0");
	Ipv4InterfaceContainer ipIfaceN1N2 = ipv4.Assign (dev2);
	Ipv4GlobalRoutingHelper::PopulateRoutingTables ();
	// Create TCP Sink
	uint16_t port = 10600;
	Address SinkLocalAddress(InetSocketAddress (Ipv4Address:: GetAny (), port));
	PacketSinkHelper sinkHelper ("ns3::TcpSocketFactory",SinkLocalAddress);
	ApplicationContainer sinkApp = sinkHelper .Install ( n1n2.Get (1));
	sinkApp.Start (Seconds (0.0));
	sinkApp. Stop ( Seconds (1000.0));
	// Create Application for generating packets
	OnOffHelper clientHelper ("ns3::TcpSocketFactory", Address ());
	clientHelper.SetAttribute ("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
	clientHelper.SetAttribute ("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
	ApplicationContainer clientApps;
	AddressValue remoteAddress ( InetSocketAddress ( ipIfaceN1N2.GetAddress (1),port));
	clientHelper.SetAttribute ("Remote", remoteAddress);
	clientApps.Add( clientHelper. Install (n0n1.Get(0)));
	clientApps. Start (Seconds (1.0));
	clientApps. Stop ( Seconds (300.0));
	// Set default Socket type to one of the Tcp Sockets
	//Config::SetDefault ("ns3::TcpL4Protocol::SocketType",TypeIdValue(TypeId::LookupByName ("ns3::TcpTahoe")));
	Config::ConnectWithoutContext("/NodeList/0/$ns3::TcpL4Protocol/SocketList/0/CongestionWindow",MakeCallback(&CwndTracer));
	NS_LOG_INFO ("ConfigureTracing.");
	AsciiTraceHelper ascii;
	PointToPointHelper ptp;
	ptp.EnableAsciiAll (ascii. CreateFileStream("tcp.tr"));
	// Simulation.
	NS_LOG_INFO ("Running Simulation..");
	Simulator:: Stop ( Seconds(1000));
	Simulator::Run ();
	Simulator::Destroy ();
	NS_LOG_INFO (" Done!.");
}
