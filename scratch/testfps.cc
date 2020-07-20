#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"
#include <iostream>
#include <string>

using namespace ns3;
using namespace std;

NS_LOG_COMPONENT_DEFINE ("FirstScriptExample");

NodeContainer nodes;
Ipv4InterfaceContainer interfaces;
Ptr<Socket> Send_sock1;
Ptr<Socket> Send_sock;

int flow[100][6];
void serversend(Ptr<Socket> sock,int id)
{
  string str = to_string(id);
  Time send_time = Simulator::Now();
  flow[id][2] = send_time.GetMicroSeconds();
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

void cssend(Ptr<Socket> sock,string str)
{
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
ServerRecvString(Ptr<Socket> sock)//回调函数
{
    Address from;
    Ptr<Packet> packet = sock->RecvFrom (from);
    packet->RemoveAllPacketTags ();
    packet->RemoveAllByteTags ();
    InetSocketAddress address = InetSocketAddress::ConvertFrom (from);
    uint8_t data[255];
    packet->CopyData(data,sizeof(data));
    int sid;
    std::cout <<sock->GetNode()->GetId()<<" "<<"receive : '" << data <<"' from "<<address.GetIpv4 ()<<std::endl;  
    char a[sizeof(data)];
    for(uint32_t i=0;i<sizeof(data);i++){
        a[i]=data[i];
    }
    std::string strres = std::string(a);
    std::istringstream iss(strres);
    iss>>sid;

    Time action_arrive = Simulator::Now();
    flow[sid][5] = action_arrive.GetMicroSeconds();
    sid +=1;
    if(sid < 100){
        if(action_arrive.GetMicroSeconds() >= flow[sid][1]){
            serversend(Send_sock,sid);
        }
        else{
            int diff = flow[sid][1]-action_arrive.GetMicroSeconds();
            Simulator::Schedule(MicroSeconds(diff),&serversend,Send_sock,sid); 
        }
    }     
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
    std::cout <<sock->GetNode()->GetId()<<" "<<"receive : '" << data <<"' from "<<address.GetIpv4 ()<<std::endl;
    char a[sizeof(data)];
    for(uint32_t i=0;i<sizeof(data);i++){
        a[i]=data[i];
    }
    std::string strres = std::string(a);
    std::istringstream iss(strres);
    int sid;
    iss >> sid;
    Time state_arrive = Simulator::Now();
    flow[sid][3]= state_arrive.GetMicroSeconds();

    /*call python and read python delay*/
    char result[100];
    FILE *fp;
    string cmd = "python3 -W ignore 1.py";
    fp = popen(cmd.c_str(),"r");
    if(fp == NULL){
        std::cout << "ERROR"  << std::endl;
        exit(1); 
    }
    fgets(result,sizeof(result)-1,fp);
//    std::cout << "result:"<<result<<std::endl;
    string resultstring = string(result);
    pclose(fp); 
    istringstream is(resultstring);
    double p;
    is >> p;
    Time pythondelay = Seconds(p);
    int pdelay = pythondelay.GetMicroSeconds();


    flow[sid][4]= pdelay+state_arrive.GetMicroSeconds();;
    //cs send action to server sid
    string action = to_string(sid);
    Simulator::Schedule(pythondelay,&cssend,Send_sock1,action);  
}


int
main (int argc, char *argv[])
{
  CommandLine cmd;
  cmd.Parse (argc, argv);
  
  Time::SetResolution (Time::NS);
  nodes.Create (2);

  PointToPointHelper pointToPoint;
  pointToPoint.SetDeviceAttribute ("DataRate", StringValue ("10Gbps"));
  pointToPoint.SetChannelAttribute ("Delay", StringValue ("10000ns"));

  NetDeviceContainer devices;
  devices = pointToPoint.Install (nodes);

  InternetStackHelper stack;
  stack.Install (nodes);
  Ipv4AddressHelper address;
  address.SetBase ("10.1.1.0", "255.255.255.0");
  interfaces = address.Assign (devices);

  //UDP传输

  //0 cs 接收端
  TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");
  Ptr<Socket> Recv_sock = Socket::CreateSocket(nodes.Get (0), tid);
  // InetSocketAddress addr = InetSocketAddress(Ipv4Address::GetAny(), 10000);
  InetSocketAddress addr = InetSocketAddress(interfaces.GetAddress(0), 10000);
  Recv_sock->Bind(addr);
  Recv_sock->SetRecvCallback(MakeCallback(&RecvString)); //设置回调函数

  //1 server发送端
  Send_sock = Socket::CreateSocket(nodes.Get (1), tid);
  InetSocketAddress RecvAddr = InetSocketAddress(interfaces.GetAddress(0), 10000);
  Send_sock->Connect(RecvAddr);

  int t = 0;
  for(int i=0;i<100;i++){
     flow[i][0] = t;
     t =t + 1000000/50;
     flow[i][1] = t - 10000/50;
  }
  Simulator::Schedule(MicroSeconds(flow[0][1]),&serversend,Send_sock,0);  

  //0 cs send socket
  Send_sock1 = Socket::CreateSocket(nodes.Get(0), tid);
  InetSocketAddress RecvAddr1 = InetSocketAddress(interfaces.GetAddress(1), 10);
  Send_sock1->Connect(RecvAddr1);

  //1 server 接收端
  Ptr<Socket> Recv_sock0 = Socket::CreateSocket(nodes.Get(1), tid);
  InetSocketAddress addr0 = InetSocketAddress(interfaces.GetAddress(1), 10);
  Recv_sock0->Bind(addr0);
  Recv_sock0->SetRecvCallback(MakeCallback(&ServerRecvString));

  Simulator::Run ();
  Simulator::Stop(Seconds(500));
  Simulator::Destroy ();

  std::ofstream outfile; 
  outfile.open("100result.txt", std::ios::app);
  if(!outfile.is_open ())
     std::cout << "Open file failure" << std::endl;
  for(int i=0;i<100;i++)
      outfile <<flow[i][0]<<"\t"<<flow[i][1]<<"\t"<<flow[i][2]<<"\t"<<flow[i][3]<<"\t"<<flow[i][4]<<"\t"<<flow[i][5]<<std::endl; 
  return 0;
}
