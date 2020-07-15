#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"

#include <iostream>
#include <string>

using namespace ns3;
using namespace std;

NS_LOG_COMPONENT_DEFINE ("FirstScriptExample");

void 
send(Ptr<Socket> sock,string str)
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
} 

void
RecvString(Ptr<Socket> sock)//回调函数
{
   
    Address from;
    Ptr<Packet> packet = sock->RecvFrom (from);
    packet->RemoveAllPacketTags ();
    packet->RemoveAllByteTags ();
    InetSocketAddress address = InetSocketAddress::ConvertFrom (from);

    // uint8_t data[sizeof(packet)];
    uint8_t data[255];
    packet->CopyData(data,sizeof(data));//将包内数据写入到data内
    cout <<sock->GetNode()->GetId()<<" "<<"receive : '" << data <<"' from "<<address.GetIpv4 ()<< endl;  
    
    char a[sizeof(data)];
    for(uint32_t i=0;i<sizeof(data);i++){
        a[i]=data[i];
    }
    string strres = string(a);
    cout<<"接受到的字符串为 "<<strres<<endl;
 
}


int
main (int argc, char *argv[])
{
  CommandLine cmd;
  cmd.Parse (argc, argv);
  
  Time::SetResolution (Time::NS);
  // LogComponentEnable ("UdpEchoClientApplication", LOG_LEVEL_INFO);
  // LogComponentEnable ("UdpEchoServerApplication", LOG_LEVEL_INFO);

  NodeContainer nodes;
  nodes.Create (2);

  PointToPointHelper pointToPoint;
  pointToPoint.SetDeviceAttribute ("DataRate", StringValue ("5Mbps"));
  pointToPoint.SetChannelAttribute ("Delay", StringValue ("2ms"));

  NetDeviceContainer devices;
  devices = pointToPoint.Install (nodes);

  InternetStackHelper stack;
  stack.Install (nodes);
  Ipv4AddressHelper address;
  address.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer interfaces = address.Assign (devices);

      
  //UDP传输

  //接收端
  TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");
  Ptr<Socket> Recv_sock = Socket::CreateSocket(nodes.Get (0), tid);
  // InetSocketAddress addr = InetSocketAddress(Ipv4Address::GetAny(), 10000);
  InetSocketAddress addr = InetSocketAddress(interfaces.GetAddress(0), 10000);
  Recv_sock->Bind(addr);
  Recv_sock->SetRecvCallback(MakeCallback(&RecvString)); //设置回调函数

  //发送端
  Ptr<Socket> Send_sock = Socket::CreateSocket(nodes.Get (1), tid);
  InetSocketAddress RecvAddr = InetSocketAddress(interfaces.GetAddress(0), 10000);
  Send_sock->Connect(RecvAddr);

  Simulator::Schedule(Seconds(2),&send,Send_sock,"Hello Ns3!");  

  Simulator::Run ();
  Simulator::Stop(Seconds(5));
  Simulator::Destroy ();
  return 0;
}
