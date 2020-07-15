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
#include <time.h>
// The CDF in TrafficGenerator
/*
extern "C"
{
#include "cdf.h"
}
*/
#define LINK_CAPACITY_BASE    1000000000          // 1Gbps
#define BUFFER_SIZE 600                           // 250 packets

#define RED_QUEUE_MARKING 65 		        	  // 65 Packets (available only in DcTcp)

// The flow port range, each flow will be assigned a random port number within this range
#define PORT_START 10000
#define PORT_END 50000

// Adopted from the simulation from WANG PENG
// Acknowledged to https://williamcityu@bitbucket.org/williamcityu/2016-socc-simulation.git
#define PACKET_SIZE 1400

#define PRESTO_RATIO 10

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("wcmp");

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

void install_applications (int fromLeafId, NodeContainer servers, double requestRate, struct cdf_table *cdfTable,
        long &flowCount, long &totalFlowSize, int SERVER_COUNT, int LEAF_COUNT, double START_TIME, double END_TIME, double FLOW_LAUNCH_END_TIME, uint32_t applicationPauseThresh, uint32_t applicationPauseTime)
{
    NS_LOG_INFO ("Install applications:");
    for (int i = 0; i < SERVER_COUNT; i++)
    {
        int fromServerIndex = fromLeafId * SERVER_COUNT + i;

        double startTime = START_TIME + poission_gen_interval (requestRate);
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
       
            /*
            NS_LOG_INFO ("\tFlowid:"<<flowCount<<" Flow from server: " << fromServerIndex << " to server: "
                    << destServerIndex << " on port: " << port << " with flow size: "
                    << flowSize << " [start time: " << startTime <<"]");*/
            startTime += poission_gen_interval (requestRate);
        }
    }
}

void recvPackets(std::string context,Ptr<const Packet> p,const Address &from){   
    std::cout<<from<<std::endl;
    //std::cout<<tcpH<<std::endl;
    //uint16_t a = (uint16_t)tcpH.GetFlags();
    //std::cout<<a<<std::endl;
/*
    TcpHeader tcpHeader;
    packet->PeekHeader(tcpHeader);
    uint8_t tcpflags = tcpHeader.GetFlags ();
    if (tcpHeader.GetFlags () & TcpHeader::ACK)
    {
         std::cout<<"caught ack packet"<<std::endl;
    }
    if (tcpflags == (TcpHeader::FIN))
    {
        std::cout<<"caught fin packet"<<std::endl;
    }
*/
    //uint8_t flag = th.GetFlags();
    //Ipv4FlowProbeTag tag;
    //packet->FindFirstMatchingByteTag(tag);
    //uint32_t fid = tag.GetFlowId ();
    //uint32_t psize = tag.GetPacketSize();
}

void sendPakcets(std::string path,Ptr<const Packet> packet);
/*
uint32_t pktcounter = 0;
uint32_t oldcounter = 0;
void ReceivePacket (Ptr<const Packet> packet, const Address &from)
{
    //NS_LOG_DEBUG ("Received one packet!");
    uint32_t size = packet->GetSize();
    pktcounter +=size ;
}

void Throughput()
{
  double period = 0.1;
  double throughput = (pktcounter - oldcounter)/ period;
  NS_LOG_INFO("Throughput (bytes/sec) = " << throughput);
  oldcounter = pktcounter; 
  Simulator::Schedule(Seconds(period), &Throughput);
}
*/

/*
void SendInfoToCS(int period)
{
  Simulator::Schedule(Seconds(period), &Throughput, period);
}*/


/*
int step = -1;
double period = 10.0;
double throught[20]={0.0};

void ThroughputMonitor (FlowMonitorHelper* fmhelper, Ptr<FlowMonitor> flowMon,std::string id)
{
    if(step>=0)
    {
        std::list<std::string> activelist;
        std::list<std::string> finishedlist;
        std::map<FlowId, FlowMonitor::FlowStats> flowStats = flowMon->GetFlowStats();
        Ptr<Ipv4FlowClassifier> classing = DynamicCast<Ipv4FlowClassifier> (fmhelper->GetClassifier());
        double th = 0.0;
        for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator stats = flowStats.begin (); stats != flowStats.end (); ++stats)
        {	
            Ipv4FlowClassifier::FiveTuple fiveTuple = classing->FindFlow (stats->first);
		
		//std::cout<<stats->first<<","<<fiveTuple.sourceAddress<<","<<fiveTuple.destinationAddress<<","<<fiveTuple.sourcePort<<","<<fiveTuple.destinationPort<<<<","<<fiveTuple.protocol<<endl;
		//std::cout<<"Flow ID			: " << stats->first <<" ; "<< fiveTuple.sourceAddress <<" -----> "<<fiveTuple.destinationAddress<<std::endl;
		//std::cout<<"Tx Packets = " << stats->second.txPackets<<std::endl;
		//std::cout<<"Rx Packets = " << stats->second.rxPackets<<std::endl;
		//std::cout<<"Duration		: "<<stats->second.timeLastRxPacket.GetSeconds()-stats->second.timeFirstTxPacket.GetSeconds()<<std::endl;
		//std::cout<<"Last Received Packet	: "<< stats->second.timeLastRxPacket.GetSeconds()<<" Seconds"<<std::endl;
		//std::cout<<"Throughput: " << stats->second.rxBytes * 8.0 / (stats->second.timeLastRxPacket.GetSeconds()-stats->second.timeFirstTxPacket.GetSeconds())/1024/1024  << " Mbps"<<std::endl;
		//std::cout<<"---------------------------------------------------------------------------"<<std::endl;
		
            if(!(stats->first % 2 == 0)){
                std::ostringstream oss;
                if(stats->second.rxPackets < stats->second.txPackets){
                    oss << fiveTuple.sourceAddress<<","<<fiveTuple.destinationAddress<<","<<fiveTuple.sourcePort<<","<<fiveTuple.destinationPort<<","<<"tcp";
                    if(activelist.size()<10){
                         activelist.push_back(oss.str());
                     }
                     else{
                         activelist.pop_front();
                         activelist.push_back(oss.str());
                     }
                     oss.clear();
                //std::cout<<stats->first<<","<<fiveTuple.sourceAddress<<","<<fiveTuple.destinationAddress<<","<<fiveTuple.sourcePort<<","<<fiveTuple.destinationPort<<","<<"tcp"<<std::endl;
                 }
                 else{
                     th += stats->second.rxBytes/ (stats->second.timeLastRxPacket.GetSeconds()-stats->second.timeFirstTxPacket.GetSeconds());
                     oss<< fiveTuple.sourceAddress<<","<<fiveTuple.destinationAddress<<","<<fiveTuple.sourcePort<<","<<fiveTuple.destinationPort<<",tcp,"<<
                     stats->second.rxBytes<<","<<(stats->second.timeLastRxPacket.GetSeconds()-stats->second.timeFirstTxPacket.GetSeconds());
                     if(finishedlist.size()<100){
                         finishedlist.push_back(oss.str());
                     }
                     else{
                         finishedlist.pop_front();
                         finishedlist.push_back(oss.str());
                     }
                     oss.clear();
                    // std::cout<<stats->first<<","<<fiveTuple.sourceAddress<<","<<fiveTuple.destinationAddress<<","<<fiveTuple.sourcePort<<","<<fiveTuple.destinationPort<<","<<"tcp,"<<
                    // stats->second.rxBytes<<","<<(stats->second.timeLastRxPacket.GetSeconds()-stats->second.timeFirstTxPacket.GetSeconds())<<std::endl;
                }
            }
        }
        throught[step] = th;
        std::cout<<th<<std::endl;
        while(activelist.size()<10){
            activelist.push_back("0,0,0,0,0");          
        }
        while(finishedlist.size()<100){
             finishedlist.push_back("0,0,0,0,0,0,0");
        }
        activelist.splice (activelist.end(), finishedlist);
        std::list<std::string>::iterator itor;
        itor = activelist.begin();
        std::stringstream stateFilename;
        stateFilename<<"state-"<<id<<"-"<<step<<".txt";

        std::ofstream out (stateFilename.str ().c_str (), std::ios::out|std::ios::app);
        while(itor!=activelist.end())
        {
            //std::cout<< *itor++<<std::endl;
            out <<*itor++<<std::endl;
        }
        out.close(); 
        if(step > 0){
            std::stringstream rewardFilename;
            rewardFilename<<"reward-"<<id<<"-"<<step-1<<".txt";
            std::ofstream out (rewardFilename.str ().c_str (), std::ios::out|std::ios::app);
            double reward = throught[step]/throught[step-1];
            out<<reward<<std::endl;
            out.close();
        }
    }
    step++;
    Simulator::Schedule(Seconds(period),&ThroughputMonitor, fmhelper, flowMon, id);
}
*/

//记录开始时间
Time start_time=Seconds(0.01);
//记录结束时间
Time end_time1;
Time end_time2;

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
int callnum=0;
int servernum = 64;
NodeContainer servers;
NodeContainer cs;
Ipv4InterfaceContainer csi;
std::vector<Ipv4Address> serverAddresses (64);
int delay1=0;
int delay2=0;
int pythondelay=0;
void
CSRecvString(Ptr<Socket> sock)
{
    callnum++;
    Address from;
    Ptr<Packet> packet = sock->RecvFrom (from);
    packet->RemoveAllPacketTags ();
    packet->RemoveAllByteTags ();
    InetSocketAddress address = InetSocketAddress::ConvertFrom (from);

    // uint8_t data[sizeof(packet)];
    uint8_t data[255];
    packet->CopyData(data,sizeof(data));//将包内数据写入到data内
    std::cout <<sock->GetNode()->GetId()<<" "<<"receive : '" << data <<"' from "<<address.GetIpv4()<< std::endl; 
    if(callnum == servernum){
        end_time1 = Simulator::Now();
        delay1 = end_time1.GetMicroSeconds()-start_time.GetMicroSeconds();
        std::cout<<"finish:"<<delay1<<"ms"<<std::endl;
        time_t c_start, c_end;
        c_start = clock();
        system("python3 1.py");
        c_end = clock(); 
        pythondelay=difftime(c_end,c_start);
        std::cout<<"python use"<<pythondelay<<"ms" <<std::endl;     

        TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");

        Ptr<Socket> Send_sock = Socket::CreateSocket(cs.Get(0), tid);
        InetSocketAddress RecvAddr = InetSocketAddress(serverAddresses[0], 10);
        Send_sock->Connect(RecvAddr);
        Simulator::Schedule(Simulator::Now(),&send,Send_sock,"cs send to 0");  

        Ptr<Socket> Send_sock1 = Socket::CreateSocket(cs.Get(0), tid);
        InetSocketAddress RecvAddr1 = InetSocketAddress(serverAddresses[1], 10);
        Send_sock1->Connect(RecvAddr1);
        Simulator::Schedule(Simulator::Now(),&send,Send_sock1,"cs send to 1"); 

        Ptr<Socket> Send_sock2 = Socket::CreateSocket(cs.Get(0), tid);
        InetSocketAddress RecvAddr2 = InetSocketAddress(serverAddresses[2], 10);
        Send_sock2->Connect(RecvAddr2);
        Simulator::Schedule(Simulator::Now(),&send,Send_sock2,"cs send to 2");  

        Ptr<Socket> Send_sock3 = Socket::CreateSocket(cs.Get(0), tid);
        InetSocketAddress RecvAddr3 = InetSocketAddress(serverAddresses[3], 10);
        Send_sock3->Connect(RecvAddr3);
        Simulator::Schedule(Simulator::Now(),&send,Send_sock3,"cs send to 3"); 

        Ptr<Socket> Send_sock4 = Socket::CreateSocket(cs.Get(0), tid);
        InetSocketAddress RecvAddr4 = InetSocketAddress(serverAddresses[4], 10);
        Send_sock4->Connect(RecvAddr4);
        Simulator::Schedule(Simulator::Now(),&send,Send_sock4,"cs send to 4");  

        Ptr<Socket> Send_sock5 = Socket::CreateSocket(cs.Get(0), tid);
        InetSocketAddress RecvAddr5 = InetSocketAddress(serverAddresses[5], 10);
        Send_sock5->Connect(RecvAddr5);
        Simulator::Schedule(Simulator::Now(),&send,Send_sock5,"cs send to 5"); 

        Ptr<Socket> Send_sock6 = Socket::CreateSocket(cs.Get(0), tid);
        InetSocketAddress RecvAddr6 = InetSocketAddress(serverAddresses[6], 10);
        Send_sock6->Connect(RecvAddr6);
        Simulator::Schedule(Simulator::Now(),&send,Send_sock6,"cs send to 6");  

        Ptr<Socket> Send_sock7 = Socket::CreateSocket(cs.Get(0), tid);
        InetSocketAddress RecvAddr7 = InetSocketAddress(serverAddresses[7], 10);
        Send_sock7->Connect(RecvAddr7);
        Simulator::Schedule(Simulator::Now(),&send,Send_sock7,"cs send to 7"); 

        Ptr<Socket> Send_sock8 = Socket::CreateSocket(cs.Get(0), tid);
        InetSocketAddress RecvAddr8 = InetSocketAddress(serverAddresses[8], 10);
        Send_sock8->Connect(RecvAddr8);
        Simulator::Schedule(Simulator::Now(),&send,Send_sock8,"cs send to 8"); 

        Ptr<Socket> Send_sock9 = Socket::CreateSocket(cs.Get(0), tid);
        InetSocketAddress RecvAddr9 = InetSocketAddress(serverAddresses[9], 10);
        Send_sock9->Connect(RecvAddr9);
        Simulator::Schedule(Simulator::Now(),&send,Send_sock9,"cs send to 9");  

        Ptr<Socket> Send_sock10 = Socket::CreateSocket(cs.Get(0), tid);
        InetSocketAddress RecvAddr10 = InetSocketAddress(serverAddresses[10], 10);
        Send_sock10->Connect(RecvAddr10);
        Simulator::Schedule(Simulator::Now(),&send,Send_sock10,"cs send to 10"); 

        Ptr<Socket> Send_sock11= Socket::CreateSocket(cs.Get(0), tid);
        InetSocketAddress RecvAddr11 = InetSocketAddress(serverAddresses[11], 10);
        Send_sock11->Connect(RecvAddr11);
        Simulator::Schedule(Simulator::Now(),&send,Send_sock11,"cs send to 11");  

        Ptr<Socket> Send_sock12 = Socket::CreateSocket(cs.Get(0), tid);
        InetSocketAddress RecvAddr12 = InetSocketAddress(serverAddresses[12], 10);
        Send_sock12->Connect(RecvAddr12);
        Simulator::Schedule(Simulator::Now(),&send,Send_sock12,"cs send to 12"); 

        Ptr<Socket> Send_sock13 = Socket::CreateSocket(cs.Get(0), tid);
        InetSocketAddress RecvAddr13 = InetSocketAddress(serverAddresses[13], 10);
        Send_sock13->Connect(RecvAddr13);
        Simulator::Schedule(Simulator::Now(),&send,Send_sock13,"cs send to 13");  

        Ptr<Socket> Send_sock14 = Socket::CreateSocket(cs.Get(0), tid);
        InetSocketAddress RecvAddr14 = InetSocketAddress(serverAddresses[14], 10);
        Send_sock14->Connect(RecvAddr14);
        Simulator::Schedule(Simulator::Now(),&send,Send_sock14,"cs send to 14"); 
      
        Ptr<Socket> Send_sock15 = Socket::CreateSocket(cs.Get(0), tid);
        InetSocketAddress RecvAddr15 = InetSocketAddress(serverAddresses[15], 10);
        Send_sock15->Connect(RecvAddr15);
        Simulator::Schedule(Simulator::Now(),&send,Send_sock15,"cs send to 15");  
//16-31
        Ptr<Socket> Send_sock16 = Socket::CreateSocket(cs.Get(0), tid);
        InetSocketAddress RecvAddr16 = InetSocketAddress(serverAddresses[16], 10);
        Send_sock16->Connect(RecvAddr16);
        Simulator::Schedule(Simulator::Now(),&send,Send_sock16,"cs send to 16");  

        Ptr<Socket> Send_sock17 = Socket::CreateSocket(cs.Get(0), tid);
        InetSocketAddress RecvAddr17 = InetSocketAddress(serverAddresses[17], 10);
        Send_sock17->Connect(RecvAddr17);
        Simulator::Schedule(Simulator::Now(),&send,Send_sock17,"cs send to 17"); 

        Ptr<Socket> Send_sock18 = Socket::CreateSocket(cs.Get(0), tid);
        InetSocketAddress RecvAddr18 = InetSocketAddress(serverAddresses[18], 10);
        Send_sock18->Connect(RecvAddr18);
        Simulator::Schedule(Simulator::Now(),&send,Send_sock18,"cs send to 18");  

        Ptr<Socket> Send_sock19 = Socket::CreateSocket(cs.Get(0), tid);
        InetSocketAddress RecvAddr19 = InetSocketAddress(serverAddresses[19], 10);
        Send_sock19->Connect(RecvAddr19);
        Simulator::Schedule(Simulator::Now(),&send,Send_sock19,"cs send to 19"); 

        Ptr<Socket> Send_sock20 = Socket::CreateSocket(cs.Get(0), tid);
        InetSocketAddress RecvAddr20 = InetSocketAddress(serverAddresses[20], 10);
        Send_sock20->Connect(RecvAddr20);
        Simulator::Schedule(Simulator::Now(),&send,Send_sock20,"cs send to 20");  

        Ptr<Socket> Send_sock21 = Socket::CreateSocket(cs.Get(0), tid);
        InetSocketAddress RecvAddr21 = InetSocketAddress(serverAddresses[21], 10);
        Send_sock21->Connect(RecvAddr21);
        Simulator::Schedule(Simulator::Now(),&send,Send_sock21,"cs send to 21");  

        Ptr<Socket> Send_sock22 = Socket::CreateSocket(cs.Get(0), tid);
        InetSocketAddress RecvAddr22 = InetSocketAddress(serverAddresses[22], 10);
        Send_sock22->Connect(RecvAddr22);
        Simulator::Schedule(Simulator::Now(),&send,Send_sock22,"cs send to 22");  

        Ptr<Socket> Send_sock23 = Socket::CreateSocket(cs.Get(0), tid);
        InetSocketAddress RecvAddr23 = InetSocketAddress(serverAddresses[23], 10);
        Send_sock23->Connect(RecvAddr23);
        Simulator::Schedule(Simulator::Now(),&send,Send_sock23,"cs send to 23");  

        Ptr<Socket> Send_sock24 = Socket::CreateSocket(cs.Get(0), tid);
        InetSocketAddress RecvAddr24 = InetSocketAddress(serverAddresses[24], 10);
        Send_sock24->Connect(RecvAddr24);
        Simulator::Schedule(Simulator::Now(),&send,Send_sock24,"cs send to 24");  

        Ptr<Socket> Send_sock25 = Socket::CreateSocket(cs.Get(0), tid);
        InetSocketAddress RecvAddr25 = InetSocketAddress(serverAddresses[25], 10);
        Send_sock25->Connect(RecvAddr25);
        Simulator::Schedule(Simulator::Now(),&send,Send_sock25,"cs send to 25");  

        Ptr<Socket> Send_sock26 = Socket::CreateSocket(cs.Get(0), tid);
        InetSocketAddress RecvAddr26 = InetSocketAddress(serverAddresses[26], 10);
        Send_sock26->Connect(RecvAddr26);
        Simulator::Schedule(Simulator::Now(),&send,Send_sock26,"cs send to 26");  

        Ptr<Socket> Send_sock27 = Socket::CreateSocket(cs.Get(0), tid);
        InetSocketAddress RecvAddr27 = InetSocketAddress(serverAddresses[27], 10);
        Send_sock27->Connect(RecvAddr27);
        Simulator::Schedule(Simulator::Now(),&send,Send_sock27,"cs send to 27");  
  
        Ptr<Socket> Send_sock28 = Socket::CreateSocket(cs.Get(0), tid);
        InetSocketAddress RecvAddr28 = InetSocketAddress(serverAddresses[28], 10);
        Send_sock28->Connect(RecvAddr28);
        Simulator::Schedule(Simulator::Now(),&send,Send_sock28,"cs send to 28");  

        Ptr<Socket> Send_sock29 = Socket::CreateSocket(cs.Get(0), tid);
        InetSocketAddress RecvAddr29 = InetSocketAddress(serverAddresses[29], 10);
        Send_sock29->Connect(RecvAddr29);
        Simulator::Schedule(Simulator::Now(),&send,Send_sock29,"cs send to 29");  
  
        Ptr<Socket> Send_sock30 = Socket::CreateSocket(cs.Get(0), tid);
        InetSocketAddress RecvAddr30 = InetSocketAddress(serverAddresses[30], 10);
        Send_sock30->Connect(RecvAddr30);
        Simulator::Schedule(Simulator::Now(),&send,Send_sock30,"cs send to 30");  

        Ptr<Socket> Send_sock31 = Socket::CreateSocket(cs.Get(0), tid);
        InetSocketAddress RecvAddr31 = InetSocketAddress(serverAddresses[31], 10);
        Send_sock31->Connect(RecvAddr31);
        Simulator::Schedule(Simulator::Now(),&send,Send_sock31,"cs send to 31");  
        Ptr<Socket> Send_sock32= Socket::CreateSocket(cs.Get(0), tid);
        InetSocketAddress RecvAddr32 = InetSocketAddress(serverAddresses[32], 10);
        Send_sock32->Connect(RecvAddr32);
        Simulator::Schedule(Simulator::Now(),&send,Send_sock32,"cs send to 32");  

        Ptr<Socket> Send_sock33 = Socket::CreateSocket(cs.Get(0), tid);
        InetSocketAddress RecvAddr33 = InetSocketAddress(serverAddresses[33], 10);
        Send_sock33->Connect(RecvAddr33);
        Simulator::Schedule(Simulator::Now(),&send,Send_sock33,"cs send to 33"); 

        Ptr<Socket> Send_sock34 = Socket::CreateSocket(cs.Get(0), tid);
        InetSocketAddress RecvAddr34 = InetSocketAddress(serverAddresses[34], 10);
        Send_sock34->Connect(RecvAddr34);
        Simulator::Schedule(Simulator::Now(),&send,Send_sock34,"cs send to 34");  

        Ptr<Socket> Send_sock35 = Socket::CreateSocket(cs.Get(0), tid);
        InetSocketAddress RecvAddr35 = InetSocketAddress(serverAddresses[35], 10);
        Send_sock35->Connect(RecvAddr35);
        Simulator::Schedule(Simulator::Now(),&send,Send_sock35,"cs send to 35"); 
      
        Ptr<Socket> Send_sock36 = Socket::CreateSocket(cs.Get(0), tid);
        InetSocketAddress RecvAddr36 = InetSocketAddress(serverAddresses[36], 10);
        Send_sock36->Connect(RecvAddr36);
        Simulator::Schedule(Simulator::Now(),&send,Send_sock36,"cs send to 36");  

        Ptr<Socket> Send_sock37 = Socket::CreateSocket(cs.Get(0), tid);
        InetSocketAddress RecvAddr37 = InetSocketAddress(serverAddresses[37], 10);
        Send_sock37->Connect(RecvAddr37);
        Simulator::Schedule(Simulator::Now(),&send,Send_sock37,"cs send to 37");  

        Ptr<Socket> Send_sock38 = Socket::CreateSocket(cs.Get(0), tid);
        InetSocketAddress RecvAddr38 = InetSocketAddress(serverAddresses[38], 10);
        Send_sock38->Connect(RecvAddr38);
        Simulator::Schedule(Simulator::Now(),&send,Send_sock38,"cs send to 38"); 

        Ptr<Socket> Send_sock39 = Socket::CreateSocket(cs.Get(0), tid);
        InetSocketAddress RecvAddr39 = InetSocketAddress(serverAddresses[39], 10);
        Send_sock39->Connect(RecvAddr39);
        Simulator::Schedule(Simulator::Now(),&send,Send_sock39,"cs send to 39");  

        Ptr<Socket> Send_sock40 = Socket::CreateSocket(cs.Get(0), tid);
        InetSocketAddress RecvAddr40 = InetSocketAddress(serverAddresses[40], 10);
        Send_sock40->Connect(RecvAddr40);
        Simulator::Schedule(Simulator::Now(),&send,Send_sock40,"cs send to 40"); 

        Ptr<Socket> Send_sock41 = Socket::CreateSocket(cs.Get(0), tid);
        InetSocketAddress RecvAddr41 = InetSocketAddress(serverAddresses[41], 10);
        Send_sock41->Connect(RecvAddr41);
        Simulator::Schedule(Simulator::Now(),&send,Send_sock41,"cs send to 41");  

        Ptr<Socket> Send_sock42 = Socket::CreateSocket(cs.Get(0), tid);
        InetSocketAddress RecvAddr42 = InetSocketAddress(serverAddresses[42], 10);
        Send_sock42->Connect(RecvAddr42);
        Simulator::Schedule(Simulator::Now(),&send,Send_sock42,"cs send to 42");  

        Ptr<Socket> Send_sock43 = Socket::CreateSocket(cs.Get(0), tid);
        InetSocketAddress RecvAddr43 = InetSocketAddress(serverAddresses[43], 10);
        Send_sock43->Connect(RecvAddr43);
        Simulator::Schedule(Simulator::Now(),&send,Send_sock43,"cs send to 43");  

        Ptr<Socket> Send_sock44 = Socket::CreateSocket(cs.Get(0), tid);
        InetSocketAddress RecvAddr44 = InetSocketAddress(serverAddresses[44], 10);
        Send_sock44->Connect(RecvAddr44);
        Simulator::Schedule(Simulator::Now(),&send,Send_sock44,"cs send to 44");  

        Ptr<Socket> Send_sock45 = Socket::CreateSocket(cs.Get(0), tid);
        InetSocketAddress RecvAddr45 = InetSocketAddress(serverAddresses[45], 10);
        Send_sock45->Connect(RecvAddr45);
        Simulator::Schedule(Simulator::Now(),&send,Send_sock45,"cs send to 45");  

        Ptr<Socket> Send_sock46 = Socket::CreateSocket(cs.Get(0), tid);
        InetSocketAddress RecvAddr46 = InetSocketAddress(serverAddresses[46], 10);
        Send_sock46->Connect(RecvAddr46);
        Simulator::Schedule(Simulator::Now(),&send,Send_sock46,"cs send to 46");  

        Ptr<Socket> Send_sock47 = Socket::CreateSocket(cs.Get(0), tid);
        InetSocketAddress RecvAddr47 = InetSocketAddress(serverAddresses[47], 10);
        Send_sock47->Connect(RecvAddr47);
        Simulator::Schedule(Simulator::Now(),&send,Send_sock47,"cs send to 47");  

        Ptr<Socket> Send_sock48 = Socket::CreateSocket(cs.Get(0), tid);
        InetSocketAddress RecvAddr48 = InetSocketAddress(serverAddresses[48], 10);
        Send_sock48->Connect(RecvAddr48);
        Simulator::Schedule(Simulator::Now(),&send,Send_sock48,"cs send to 48");  
  
        Ptr<Socket> Send_sock49 = Socket::CreateSocket(cs.Get(0), tid);
        InetSocketAddress RecvAddr49 = InetSocketAddress(serverAddresses[49], 10);
        Send_sock49->Connect(RecvAddr49);
        Simulator::Schedule(Simulator::Now(),&send,Send_sock49,"cs send to 49");  

        Ptr<Socket> Send_sock50 = Socket::CreateSocket(cs.Get(0), tid);
        InetSocketAddress RecvAddr50 = InetSocketAddress(serverAddresses[50], 10);
        Send_sock50->Connect(RecvAddr50);
        Simulator::Schedule(Simulator::Now(),&send,Send_sock50,"cs send to 50");  
  
        Ptr<Socket> Send_sock51 = Socket::CreateSocket(cs.Get(0), tid);
        InetSocketAddress RecvAddr51 = InetSocketAddress(serverAddresses[51], 10);
        Send_sock51->Connect(RecvAddr51);
        Simulator::Schedule(Simulator::Now(),&send,Send_sock51,"cs send to 51");  

        Ptr<Socket> Send_sock52 = Socket::CreateSocket(cs.Get(0), tid);
        InetSocketAddress RecvAddr52 = InetSocketAddress(serverAddresses[52], 10);
        Send_sock52->Connect(RecvAddr52);
        Simulator::Schedule(Simulator::Now(),&send,Send_sock52,"cs send to 52");  
        Ptr<Socket> Send_sock53= Socket::CreateSocket(cs.Get(0), tid);
        InetSocketAddress RecvAddr53 = InetSocketAddress(serverAddresses[53], 10);
        Send_sock53->Connect(RecvAddr53);
        Simulator::Schedule(Simulator::Now(),&send,Send_sock53,"cs send to 53");  

        Ptr<Socket> Send_sock54 = Socket::CreateSocket(cs.Get(0), tid);
        InetSocketAddress RecvAddr54 = InetSocketAddress(serverAddresses[54], 10);
        Send_sock54->Connect(RecvAddr54);
        Simulator::Schedule(Simulator::Now(),&send,Send_sock54,"cs send to 54"); 

        Ptr<Socket> Send_sock55 = Socket::CreateSocket(cs.Get(0), tid);
        InetSocketAddress RecvAddr55 = InetSocketAddress(serverAddresses[55], 10);
        Send_sock55->Connect(RecvAddr55);
        Simulator::Schedule(Simulator::Now(),&send,Send_sock55,"cs send to 55");  

        Ptr<Socket> Send_sock56 = Socket::CreateSocket(cs.Get(0), tid);
        InetSocketAddress RecvAddr56 = InetSocketAddress(serverAddresses[56], 10);
        Send_sock56->Connect(RecvAddr56);
        Simulator::Schedule(Simulator::Now(),&send,Send_sock56,"cs send to 56"); 
      
        Ptr<Socket> Send_sock57 = Socket::CreateSocket(cs.Get(0), tid);
        InetSocketAddress RecvAddr57 = InetSocketAddress(serverAddresses[57], 10);
        Send_sock57->Connect(RecvAddr57);
        Simulator::Schedule(Simulator::Now(),&send,Send_sock57,"cs send to 57");  

        Ptr<Socket> Send_sock58 = Socket::CreateSocket(cs.Get(0), tid);
        InetSocketAddress RecvAddr58 = InetSocketAddress(serverAddresses[58], 10);
        Send_sock58->Connect(RecvAddr58);
        Simulator::Schedule(Simulator::Now(),&send,Send_sock58,"cs send to 58");  

        Ptr<Socket> Send_sock59 = Socket::CreateSocket(cs.Get(0), tid);
        InetSocketAddress RecvAddr59 = InetSocketAddress(serverAddresses[59], 10);
        Send_sock59->Connect(RecvAddr59);
        Simulator::Schedule(Simulator::Now(),&send,Send_sock59,"cs send to 59"); 

        Ptr<Socket> Send_sock60 = Socket::CreateSocket(cs.Get(0), tid);
        InetSocketAddress RecvAddr60 = InetSocketAddress(serverAddresses[60], 10);
        Send_sock60->Connect(RecvAddr60);
        Simulator::Schedule(Simulator::Now(),&send,Send_sock60,"cs send to 60");  

        Ptr<Socket> Send_sock61 = Socket::CreateSocket(cs.Get(0), tid);
        InetSocketAddress RecvAddr61 = InetSocketAddress(serverAddresses[61], 10);
        Send_sock61->Connect(RecvAddr61);
        Simulator::Schedule(Simulator::Now(),&send,Send_sock61,"cs send to 61"); 

        Ptr<Socket> Send_sock62 = Socket::CreateSocket(cs.Get(0), tid);
        InetSocketAddress RecvAddr62 = InetSocketAddress(serverAddresses[62], 10);
        Send_sock62->Connect(RecvAddr62);
        Simulator::Schedule(Simulator::Now(),&send,Send_sock62,"cs send to 62");  

        Ptr<Socket> Send_sock63 = Socket::CreateSocket(cs.Get(0), tid);
        InetSocketAddress RecvAddr63 = InetSocketAddress(serverAddresses[63], 10);
        Send_sock63->Connect(RecvAddr63);
        Simulator::Schedule(Simulator::Now(),&send,Send_sock63,"cs send to 63");  
    
/*  
        //cs send to server	
   	std::vector<InetSocketAddress> RecvAddr(servernum);
        std::vector<Ptr<Socket>> Send_sock (servernum);
        
   	for(int i =0;i<servernum;i++){
           RecvAddr[i] = InetSocketAddress(serverAddresses[i], 10);
    	   Send_sock[i] = Socket::CreateSocket(cs.Get(0), tid);
      	   Send_sock[i]->Connect(RecvAddr[i]);
      	   std::string s= "cs send action to server"+std::to_string(i);
       	   Simulator::Schedule(Simulator::Now(),&send,Send_sock[i],s); 
    	}
*/
    }
    
/*   
    char a[sizeof(data)];
    for(uint32_t i=0;i<sizeof(data);i++){
        a[i]=data[i];
    }
    std::string strres = std::string(a);
    std::cout<<"接受到的字符串为 "<<strres<<std::endl;
*/
}

int servercallnum=0;
void
ServerRecvString(Ptr<Socket> sock)//回调函数
{
    servercallnum++;
    Address from;
    Ptr<Packet> packet = sock->RecvFrom (from);
    packet->RemoveAllPacketTags ();
    packet->RemoveAllByteTags ();
    InetSocketAddress address = InetSocketAddress::ConvertFrom (from);

    // uint8_t data[sizeof(packet)];
    uint8_t data[255];
    packet->CopyData(data,sizeof(data));//将包内数据写入到data内
    std::cout <<sock->GetNode()->GetId()<<" "<<"receive : '" << data <<"' from "<<address.GetIpv4 ()<<std::endl;  
    if(servercallnum==64){
        end_time2 = Simulator::Now();
        delay2 = end_time2.GetMicroSeconds()-end_time1.GetMicroSeconds();
        std::cout<<"delay1:"<<delay1<<"ms"<<std::endl;
        std::cout<<"delay2:"<<delay2<<"ms"<<std::endl;
        std::cout<<"pythondelay2:"<<pythondelay<<"ms"<<std::endl;
        std::cout<<"ns3delay:"<<end_time2.GetMicroSeconds()-start_time.GetMicroSeconds()<<"ms"<<std::endl;
        int alldelay = delay1+delay2+pythondelay;
        std::cout<<"alldelay:"<<alldelay<<"ms"<<std::endl;
        
    }
}


int main (int argc, char *argv[])
{
#if 1
    //LogComponentEnable ("wcmp", LOG_LEVEL_ALL);
#endif
    // Command line parameters parsing
    std::string id = "0";
    unsigned randomSeed = 0;
    std::string cdfFileName = "examples/load-balance/VL2_CDF.txt";
    double load = 0.01;
    std::string transportProt = "Tcp";

    // The simulation starting and ending time
    double START_TIME = 0.0;
    double END_TIME = 100.0;

    double FLOW_LAUNCH_END_TIME = END_TIME;

    uint32_t linkLatency = 10;

    int SERVER_COUNT = 4;
    int SPINE_COUNT = 16;
    int LEAF_COUNT = 16;
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
    cmd.AddValue ("cdfFileName", "File name for flow distribution", cdfFileName);
    cmd.AddValue ("load", "Load of the network, 0.0 - 1.0", load);
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
    c_s_address.SetBase("10.3.1.0", "255.255.255.0");
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
/*
    for (int fromLeafId = 0; fromLeafId < LEAF_COUNT; fromLeafId ++)
    {
        install_applications(fromLeafId, servers, requestRate, cdfTable, flowCount, totalFlowSize, SERVER_COUNT, LEAF_COUNT, START_TIME, END_TIME, FLOW_LAUNCH_END_TIME, applicationPauseThresh, applicationPauseTime);
    }
    std::cout<<"Total flow: " << flowCount<<std::endl;
*/
    //NS_LOG_INFO ("Total flow: " << flowCount);

    NS_LOG_INFO ("Actual average flow size: " << static_cast<double> (totalFlowSize) / flowCount);

    //-----------------FlowMonitor-DELAY---------------------

    //DelayMonitor(&flowHelper, flowMonitor);

    //set trace
    //Config::Connect("/NodeList/*/ApplicationList/*/$ns3::BulkSendApplication/Tx",MakeCallback(&sendPackets));
     //Config::Connect("/NodeList/*/ApplicationList/*/$ns3::PacketSink/Rx",MakeCallback (&recvPackets));
    //double period = 0.1;
    //Config::ConnectWithoutContext("/NodeList/*/ApplicationList/*/$ns3::PacketSink/Rx",MakeCallback (&ReceivePacket));
    //Simulator::Schedule(Seconds(period), &Throughput);
    //ThroughputMonitor(&flowHelper,flowMonitor,id);

    servernum = SERVER_COUNT * LEAF_COUNT;
    //CS接收端
    TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");
    Ptr<Socket> Recv_sock = Socket::CreateSocket(cs.Get (0), tid);
    InetSocketAddress addr = InetSocketAddress(csi.GetAddress(0), 9);
    Recv_sock->Bind(addr);
    Recv_sock->SetRecvCallback(MakeCallback(&CSRecvString)); //设置回调函数
    //server send to cs
    InetSocketAddress RecvAddr = InetSocketAddress(csi.GetAddress(0), 9);
    std::vector<Ptr<Socket>> Send_sock (SERVER_COUNT * LEAF_COUNT);

    for(int i =0;i<SERVER_COUNT * LEAF_COUNT;i++){
        Send_sock[i] = Socket::CreateSocket(servers.Get(i), tid);
        Send_sock[i]->Connect(RecvAddr);
        std::string s= "server"+std::to_string(i);
        Simulator::Schedule(Seconds(0.01),&send,Send_sock[i],s); 
    }

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
    //server16接收端
    Ptr<Socket> Recv_sock16 = Socket::CreateSocket(servers.Get(16), tid);
    InetSocketAddress addr16 = InetSocketAddress(serverAddresses[16], 10);
    Recv_sock16->Bind(addr16);
    Recv_sock16->SetRecvCallback(MakeCallback(&ServerRecvString));
    //server17接收端
    Ptr<Socket> Recv_sock17 = Socket::CreateSocket(servers.Get(17), tid);
    InetSocketAddress addr17 = InetSocketAddress(serverAddresses[17], 10);
    Recv_sock17->Bind(addr17);
    Recv_sock17->SetRecvCallback(MakeCallback(&ServerRecvString));
    //server18接收端
    Ptr<Socket> Recv_sock18 = Socket::CreateSocket(servers.Get(18), tid);
    InetSocketAddress addr18 = InetSocketAddress(serverAddresses[18], 10);
    Recv_sock18->Bind(addr18);
    Recv_sock18->SetRecvCallback(MakeCallback(&ServerRecvString));
    //server19接收端
    Ptr<Socket> Recv_sock19 = Socket::CreateSocket(servers.Get(19), tid);
    InetSocketAddress addr19 = InetSocketAddress(serverAddresses[19], 10);
    Recv_sock19->Bind(addr19);
    Recv_sock19->SetRecvCallback(MakeCallback(&ServerRecvString));
    //server20接收端
    Ptr<Socket> Recv_sock20 = Socket::CreateSocket(servers.Get(20), tid);
    InetSocketAddress addr20 = InetSocketAddress(serverAddresses[20], 10);
    Recv_sock20->Bind(addr20);
    Recv_sock20->SetRecvCallback(MakeCallback(&ServerRecvString));
    //server21接收端
    Ptr<Socket> Recv_sock21 = Socket::CreateSocket(servers.Get(21), tid);
    InetSocketAddress addr21 = InetSocketAddress(serverAddresses[21], 10);
    Recv_sock21->Bind(addr21);
    Recv_sock21->SetRecvCallback(MakeCallback(&ServerRecvString));
    //server22接收端
    Ptr<Socket> Recv_sock22 = Socket::CreateSocket(servers.Get(22), tid);
    InetSocketAddress addr22 = InetSocketAddress(serverAddresses[22], 10);
    Recv_sock22->Bind(addr22);
    Recv_sock22->SetRecvCallback(MakeCallback(&ServerRecvString));
    //server23接收端
    Ptr<Socket> Recv_sock23 = Socket::CreateSocket(servers.Get(23), tid);
    InetSocketAddress addr23 = InetSocketAddress(serverAddresses[23], 10);
    Recv_sock23->Bind(addr23);
    Recv_sock23->SetRecvCallback(MakeCallback(&ServerRecvString));
    //server24接收端
    Ptr<Socket> Recv_sock24 = Socket::CreateSocket(servers.Get(24), tid);
    InetSocketAddress addr24 = InetSocketAddress(serverAddresses[24], 10);
    Recv_sock24->Bind(addr24);
    Recv_sock24->SetRecvCallback(MakeCallback(&ServerRecvString));
    //server25接收端
    Ptr<Socket> Recv_sock25 = Socket::CreateSocket(servers.Get(25), tid);
    InetSocketAddress addr25 = InetSocketAddress(serverAddresses[25], 10);
    Recv_sock25->Bind(addr25);
    Recv_sock25->SetRecvCallback(MakeCallback(&ServerRecvString));
    //server26接收端
    Ptr<Socket> Recv_sock26 = Socket::CreateSocket(servers.Get(26), tid);
    InetSocketAddress addr26 = InetSocketAddress(serverAddresses[26], 10);
    Recv_sock26->Bind(addr26);
    Recv_sock26->SetRecvCallback(MakeCallback(&ServerRecvString));
    //server27接收端
    Ptr<Socket> Recv_sock27 = Socket::CreateSocket(servers.Get(27), tid);
    InetSocketAddress addr27 = InetSocketAddress(serverAddresses[27], 10);
    Recv_sock27->Bind(addr27);
    Recv_sock27->SetRecvCallback(MakeCallback(&ServerRecvString));
    //server28接收端
    Ptr<Socket> Recv_sock28 = Socket::CreateSocket(servers.Get(28), tid);
    InetSocketAddress addr28 = InetSocketAddress(serverAddresses[28], 10);
    Recv_sock28->Bind(addr28);
    Recv_sock28->SetRecvCallback(MakeCallback(&ServerRecvString));
    //server29接收端
    Ptr<Socket> Recv_sock29 = Socket::CreateSocket(servers.Get(29), tid);
    InetSocketAddress addr29 = InetSocketAddress(serverAddresses[29], 10);
    Recv_sock29->Bind(addr29);
    Recv_sock29->SetRecvCallback(MakeCallback(&ServerRecvString));
    //server30接收端
    Ptr<Socket> Recv_sock30 = Socket::CreateSocket(servers.Get(30), tid);
    InetSocketAddress addr30 = InetSocketAddress(serverAddresses[30], 10);
    Recv_sock30->Bind(addr30);
    Recv_sock30->SetRecvCallback(MakeCallback(&ServerRecvString));

    //server31接收端
    Ptr<Socket> Recv_sock31 = Socket::CreateSocket(servers.Get(31), tid);
    InetSocketAddress addr31 = InetSocketAddress(serverAddresses[31], 10);
    Recv_sock31->Bind(addr31);
    Recv_sock31->SetRecvCallback(MakeCallback(&ServerRecvString));
    //server32接收端
    Ptr<Socket> Recv_sock32 = Socket::CreateSocket(servers.Get(32), tid);
    InetSocketAddress addr32 = InetSocketAddress(serverAddresses[32], 10);
    Recv_sock32->Bind(addr32);
    Recv_sock32->SetRecvCallback(MakeCallback(&ServerRecvString));
    //server33接收端
    Ptr<Socket> Recv_sock33 = Socket::CreateSocket(servers.Get(33), tid);
    InetSocketAddress addr33 = InetSocketAddress(serverAddresses[33], 10);
    Recv_sock33->Bind(addr33);
    Recv_sock33->SetRecvCallback(MakeCallback(&ServerRecvString));
    //server34接收端
    Ptr<Socket> Recv_sock34 = Socket::CreateSocket(servers.Get(34), tid);
    InetSocketAddress addr34 = InetSocketAddress(serverAddresses[34], 10);
    Recv_sock34->Bind(addr34);
    Recv_sock34->SetRecvCallback(MakeCallback(&ServerRecvString));
    //server35接收端
    Ptr<Socket> Recv_sock35 = Socket::CreateSocket(servers.Get(35), tid);
    InetSocketAddress addr35 = InetSocketAddress(serverAddresses[35], 10);
    Recv_sock35->Bind(addr35);
    Recv_sock35->SetRecvCallback(MakeCallback(&ServerRecvString));
    //server36接收端
    Ptr<Socket> Recv_sock36 = Socket::CreateSocket(servers.Get(36), tid);
    InetSocketAddress addr36 = InetSocketAddress(serverAddresses[36], 10);
    Recv_sock36->Bind(addr36);
    Recv_sock36->SetRecvCallback(MakeCallback(&ServerRecvString));
    //server37接收端
    Ptr<Socket> Recv_sock37 = Socket::CreateSocket(servers.Get(37), tid);
    InetSocketAddress addr37 = InetSocketAddress(serverAddresses[37], 10);
    Recv_sock37->Bind(addr37);
    Recv_sock37->SetRecvCallback(MakeCallback(&ServerRecvString));
    //server38接收端
    Ptr<Socket> Recv_sock38 = Socket::CreateSocket(servers.Get(38), tid);
    InetSocketAddress addr38 = InetSocketAddress(serverAddresses[38], 10);
    Recv_sock38->Bind(addr38);
    Recv_sock38->SetRecvCallback(MakeCallback(&ServerRecvString));
    //server39接收端
    Ptr<Socket> Recv_sock39 = Socket::CreateSocket(servers.Get(39), tid);
    InetSocketAddress addr39 = InetSocketAddress(serverAddresses[39], 10);
    Recv_sock39->Bind(addr39);
    Recv_sock39->SetRecvCallback(MakeCallback(&ServerRecvString));
    //server40接收端
    Ptr<Socket> Recv_sock40 = Socket::CreateSocket(servers.Get(40), tid);
    InetSocketAddress addr40 = InetSocketAddress(serverAddresses[40], 10);
    Recv_sock40->Bind(addr40);
    Recv_sock40->SetRecvCallback(MakeCallback(&ServerRecvString));
    //server41接收端
    Ptr<Socket> Recv_sock41 = Socket::CreateSocket(servers.Get(41), tid);
    InetSocketAddress addr41 = InetSocketAddress(serverAddresses[41], 10);
    Recv_sock41->Bind(addr41);
    Recv_sock41->SetRecvCallback(MakeCallback(&ServerRecvString));
    //server42接收端
    Ptr<Socket> Recv_sock42 = Socket::CreateSocket(servers.Get(42), tid);
    InetSocketAddress addr42 = InetSocketAddress(serverAddresses[42], 10);
    Recv_sock42->Bind(addr42);
    Recv_sock42->SetRecvCallback(MakeCallback(&ServerRecvString));
    //server43接收端
    Ptr<Socket> Recv_sock43 = Socket::CreateSocket(servers.Get(43), tid);
    InetSocketAddress addr43 = InetSocketAddress(serverAddresses[43], 10);
    Recv_sock43->Bind(addr43);
    Recv_sock43->SetRecvCallback(MakeCallback(&ServerRecvString));
    //server44接收端
    Ptr<Socket> Recv_sock44 = Socket::CreateSocket(servers.Get(44), tid);
    InetSocketAddress addr44 = InetSocketAddress(serverAddresses[44], 10);
    Recv_sock44->Bind(addr44);
    Recv_sock44->SetRecvCallback(MakeCallback(&ServerRecvString));
    //server45接收端
    Ptr<Socket> Recv_sock45 = Socket::CreateSocket(servers.Get(45), tid);
    InetSocketAddress addr45 = InetSocketAddress(serverAddresses[45], 10);
    Recv_sock45->Bind(addr45);
    Recv_sock45->SetRecvCallback(MakeCallback(&ServerRecvString));
	//server46接收端
    Ptr<Socket> Recv_sock46 = Socket::CreateSocket(servers.Get(46), tid);
    InetSocketAddress addr46 = InetSocketAddress(serverAddresses[46], 10);
    Recv_sock46->Bind(addr46);
    Recv_sock46->SetRecvCallback(MakeCallback(&ServerRecvString));
    //server47接收端
    Ptr<Socket> Recv_sock47 = Socket::CreateSocket(servers.Get(47), tid);
    InetSocketAddress addr47 = InetSocketAddress(serverAddresses[47], 10);
    Recv_sock47->Bind(addr47);
    Recv_sock47->SetRecvCallback(MakeCallback(&ServerRecvString));
    //server48接收端
    Ptr<Socket> Recv_sock48 = Socket::CreateSocket(servers.Get(48), tid);
    InetSocketAddress addr48 = InetSocketAddress(serverAddresses[48], 10);
    Recv_sock48->Bind(addr48);
    Recv_sock48->SetRecvCallback(MakeCallback(&ServerRecvString));
    //server49接收端
    Ptr<Socket> Recv_sock49 = Socket::CreateSocket(servers.Get(49), tid);
    InetSocketAddress addr49 = InetSocketAddress(serverAddresses[49], 10);
    Recv_sock49->Bind(addr49);
    Recv_sock49->SetRecvCallback(MakeCallback(&ServerRecvString));
    //server50接收端
    Ptr<Socket> Recv_sock50 = Socket::CreateSocket(servers.Get(50), tid);
    InetSocketAddress addr50 = InetSocketAddress(serverAddresses[50], 10);
    Recv_sock50->Bind(addr50);
    Recv_sock50->SetRecvCallback(MakeCallback(&ServerRecvString));
    //server51接收端
    Ptr<Socket> Recv_sock51 = Socket::CreateSocket(servers.Get(51), tid);
    InetSocketAddress addr51 = InetSocketAddress(serverAddresses[51], 10);
    Recv_sock51->Bind(addr51);
    Recv_sock51->SetRecvCallback(MakeCallback(&ServerRecvString));
    //server52接收端
    Ptr<Socket> Recv_sock52 = Socket::CreateSocket(servers.Get(52), tid);
    InetSocketAddress addr52 = InetSocketAddress(serverAddresses[52], 10);
    Recv_sock52->Bind(addr52);
    Recv_sock52->SetRecvCallback(MakeCallback(&ServerRecvString));
    //server53接收端
    Ptr<Socket> Recv_sock53 = Socket::CreateSocket(servers.Get(53), tid);
    InetSocketAddress addr53 = InetSocketAddress(serverAddresses[53], 10);
    Recv_sock53->Bind(addr53);
    Recv_sock53->SetRecvCallback(MakeCallback(&ServerRecvString));
    //server54接收端
    Ptr<Socket> Recv_sock54 = Socket::CreateSocket(servers.Get(54), tid);
    InetSocketAddress addr54 = InetSocketAddress(serverAddresses[54], 10);
    Recv_sock54->Bind(addr54);
    Recv_sock54->SetRecvCallback(MakeCallback(&ServerRecvString));
    //server55接收端
    Ptr<Socket> Recv_sock55 = Socket::CreateSocket(servers.Get(55), tid);
    InetSocketAddress addr55 = InetSocketAddress(serverAddresses[55], 10);
    Recv_sock55->Bind(addr55);
    Recv_sock55->SetRecvCallback(MakeCallback(&ServerRecvString));
    //server56接收端
    Ptr<Socket> Recv_sock56 = Socket::CreateSocket(servers.Get(56), tid);
    InetSocketAddress addr56 = InetSocketAddress(serverAddresses[56], 10);
    Recv_sock56->Bind(addr56);
    Recv_sock56->SetRecvCallback(MakeCallback(&ServerRecvString));
    //server57接收端
    Ptr<Socket> Recv_sock57 = Socket::CreateSocket(servers.Get(57), tid);
    InetSocketAddress addr57 = InetSocketAddress(serverAddresses[57], 10);
    Recv_sock57->Bind(addr57);
    Recv_sock57->SetRecvCallback(MakeCallback(&ServerRecvString));
    //server58接收端
    Ptr<Socket> Recv_sock58 = Socket::CreateSocket(servers.Get(58), tid);
    InetSocketAddress addr58 = InetSocketAddress(serverAddresses[58], 10);
    Recv_sock58->Bind(addr58);
    Recv_sock58->SetRecvCallback(MakeCallback(&ServerRecvString));
    //server59接收端
    Ptr<Socket> Recv_sock59 = Socket::CreateSocket(servers.Get(59), tid);
    InetSocketAddress addr59 = InetSocketAddress(serverAddresses[59], 10);
    Recv_sock59->Bind(addr59);
    Recv_sock59->SetRecvCallback(MakeCallback(&ServerRecvString));
    //server60接收端
    Ptr<Socket> Recv_sock60 = Socket::CreateSocket(servers.Get(60), tid);
    InetSocketAddress addr60 = InetSocketAddress(serverAddresses[60], 10);
    Recv_sock60->Bind(addr60);
    Recv_sock60->SetRecvCallback(MakeCallback(&ServerRecvString));
	//server61接收端
    Ptr<Socket> Recv_sock61 = Socket::CreateSocket(servers.Get(61), tid);
    InetSocketAddress addr61 = InetSocketAddress(serverAddresses[61], 10);
    Recv_sock61->Bind(addr61);
    Recv_sock61->SetRecvCallback(MakeCallback(&ServerRecvString));
    //server62接收端
    Ptr<Socket> Recv_sock62 = Socket::CreateSocket(servers.Get(62), tid);
    InetSocketAddress addr62 = InetSocketAddress(serverAddresses[62], 10);
    Recv_sock62->Bind(addr62);
    Recv_sock62->SetRecvCallback(MakeCallback(&ServerRecvString));
	//server63接收端
    Ptr<Socket> Recv_sock63 = Socket::CreateSocket(servers.Get(63), tid);
    InetSocketAddress addr63 = InetSocketAddress(serverAddresses[63], 10);
    Recv_sock63->Bind(addr63);
    Recv_sock63->SetRecvCallback(MakeCallback(&ServerRecvString));
    //int period = 10;
    //Simulator::Schedule(Seconds(period), &SendInfoToCS,period);
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
    free_cdf (cdfTable);
    NS_LOG_INFO ("Stop simulation");
}
