#include <ros/ros.h>
#include <iostream>
#include <sys/socket.h>
#include <netdb.h>
#include <string>

using namespace std;

uint64_t pack754(double f, unsigned bits, unsigned expbits)
{
    long double fnorm;
    int shift;
    long long sign, exp, significand;
    unsigned significandbits = bits - expbits - 1; // -1 for sign bit

    if (f == 0.0) return 0; // get this special case out of the way

    // check sign and begin normalization
    if (f < 0) { sign = 1; fnorm = -f; }
    else { sign = 0; fnorm = f; }

    // get the normalized form of f and track the exponent
    shift = 0;
    while(fnorm >= 2.0) { fnorm /= 2.0; shift++; }
    while(fnorm < 1.0) { fnorm *= 2.0; shift--; }
    fnorm = fnorm - 1.0;

    // calculate the binary form (non-float) of the significand data
    significand = fnorm * ((1LL<<significandbits) + 0.5f);

    // get the biased exponent
    exp = shift + ((1<<(expbits-1)) - 1); // shift + bias

    // return the final answer
    return (sign<<(bits-1)) | (exp<<(bits-expbits-1)) | significand;
}

double unpack754(uint64_t i, unsigned bits, unsigned expbits)
{
    long double result;
    long long shift;
    unsigned bias;
    unsigned significandbits = bits - expbits - 1; // -1 for sign bit

    if (i == 0) return 0.0;

    // pull the significand
    result = (i&((1LL<<significandbits)-1)); // mask
    result /= (1LL<<significandbits); // convert back to float
    result += 1.0f; // add the one back on

    // deal with the exponent
    bias = (1<<(expbits-1)) - 1;
    shift = ((i>>significandbits)&((1LL<<expbits)-1)) - bias;
    while(shift > 0) { result *= 2.0; shift--; }
    while(shift < 0) { result /= 2.0; shift++; }

    // sign it
    result *= (i>>(bits-1))&1? -1.0: 1.0;

    return result;
}

unsigned char * serialize_uint_64(unsigned char *buffer, uint64_t value)
{
  buffer[0] = value >> 56;
  buffer[1] = value >> 48;
  buffer[2] = value >> 40;
  buffer[3] = value >> 32;
  buffer[4] = value >> 24;
  buffer[5] = value >> 16;
  buffer[6] = value >> 8;
  buffer[7] = value;
  return buffer + 8;
}

unsigned char* deserialize_uint_64(unsigned char *buffer, uint64_t* value)
{
  *value = (uint64_t)buffer[0]<<56;
  *value = *value + ((uint64_t)buffer[1]<<48);
  *value = *value + ((uint64_t)buffer[2]<<40);  
  *value = *value + ((uint64_t)buffer[3]<<32);
  *value = *value + ((uint64_t)buffer[4]<<24);
  *value = *value + ((uint64_t)buffer[5]<<16);
  *value = *value + ((uint64_t)buffer[6]<<8); 
  *value = *value + ((uint64_t)buffer[7]);    
  return buffer + 8;
}

void send_msg_struct_cl(int socketfd, int i, unsigned char* buffer, double v, double gamma, int len)
{
  unsigned char* point;
  ssize_t bytes_sent;
  uint64_t temp_64;
  temp_64 = pack754(v,64,11);
  point = serialize_uint_64(buffer,temp_64);
  temp_64 = pack754(gamma,64,11);
  point = serialize_uint_64(point,temp_64); 
  bytes_sent = send(socketfd, buffer, len, 0);
  cout<<"\nFrom :"<<i<<" bytes_sent: "<<bytes_sent<<"\n";
}

//void recv_msg_struct(int new_sd, int count, unsigned char* buffer, x_and_u* msg_recv, int len)
void recv_msg_struct(int new_sd, int count, unsigned char* buffer, double* v, double* gamma, int len)
{
  unsigned char* point;
  ssize_t bytes_recv;
  uint64_t temp_64;
  bytes_recv = recv(new_sd, buffer, len, 0);
  point = deserialize_uint_64(buffer, &temp_64);
  cout<<"bytes received: "<<bytes_recv<<endl;
  *v = unpack754(temp_64,64,11);
  point = deserialize_uint_64(point,&temp_64); 
  *gamma = unpack754(temp_64,64,11);
  //cout<<"\nFrom client "<<count<<":\n";
  //cout<<"msg x: "<<msg_recv->x<<"\n";
  //cout<<"msg u: "<<msg_recv->u<<"\n"; 
}

void create_socket_server(const char* s, int& sockets_server)
{
 int status;
 struct addrinfo host_info;       
 struct addrinfo *host_info_list; 
   
 memset(&host_info, 0, sizeof host_info);
 
 cout<<"Setting up the structs..."<<endl;
 
 host_info.ai_family = AF_UNSPEC;
 host_info.ai_flags = AI_PASSIVE;     
 host_info.ai_socktype = SOCK_STREAM; 
 
 status = getaddrinfo(s, "5532", &host_info, &host_info_list);

 if (status != 0)  
 {
   cout<<"getaddrinfo error: " << gai_strerror(status);
 }
 cout<<"\nCreating a socket \n";
 int socketfd;
 socketfd = socket(host_info_list->ai_family, host_info_list->ai_socktype, host_info_list->ai_protocol);
 if (socketfd == -1)
 {
   cout<<"socket error \n";
 }
 cout<<"Binding socket..."<<endl;

 int yes = 1;
 status = setsockopt(socketfd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int));
 status = bind(socketfd, host_info_list->ai_addr, host_info_list->ai_addrlen);
 if (status == -1)  
 {
   cout<<"bind error"<<endl;
 } 
 cout << "Listening for connections..."<<endl;
 //cout<<"before listen.\n";
 status =  listen(socketfd, 5);
 //cout<<"after listen.\n";
 if (status == -1)  
 {
   cout<< "listen error"<<endl;
 }
 //cout<<"after if.\n";
 int new_sd;
 struct sockaddr_storage their_addr;
 socklen_t addr_size = sizeof(their_addr);
 //cout<<"before accept.\n";
 new_sd = accept(socketfd, (struct sockaddr *)&their_addr, &addr_size);
 if (new_sd == -1)
 {
    cout << "listen error" << std::endl ;
 }
 else
 {
   cout << "Connection accepted. Using new socketfd : "<<new_sd<<endl;
   //mtx.lock();
   //connect_type = 's';
   //mtx.unlock();
 }
 sockets_server = new_sd;
}

void create_socket_client(const char* s, int& socket_client)
{
  int status;
  struct addrinfo host_info;
  struct addrinfo *host_info_list;

  memset(&host_info, 0, sizeof host_info);

  host_info.ai_family = AF_UNSPEC;
  host_info.ai_socktype = SOCK_STREAM;
  status = getaddrinfo(s, "5532", &host_info, &host_info_list);
  cout<<"status: "<<status<<endl;
  if (status != 0)
  {
    cout<<"getaddrinfo error"<<endl<<gai_strerror(status);
  }
  cout<<"Creating a socket \n";
  int socketfd;
  socketfd = socket(host_info_list->ai_family, host_info_list->ai_socktype, host_info_list->ai_protocol);
  if (socketfd == -1)
  {
    cout<<"socket error \n";
  }
  bool error_once = 0;
  while(1)
  {
  status =  connect(socketfd, host_info_list->ai_addr, host_info_list->ai_addrlen);
  if (status == -1 && error_once == 0)
  {
    cout<<"connect error \n";
    error_once = 1;
  }
  else if (status == 0)
  {
    //mtx.lock();
    //connect_type = 'c';
    //mtx.unlock();
    cout << "send()ing message..."  <<endl;
    cout<<"status: "<<status<<"\n";
    break;
  }
  }
  socket_client = socketfd;
}

int main(int argc, char** argv)
{
ros::init(argc, argv, "test_tcp");
double v = 1.0, gamma = 0.5;
ros::spin();
return 0;
}