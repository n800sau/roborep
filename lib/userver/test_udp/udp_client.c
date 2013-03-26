#include <sys/socket.h>
#include <netinet/in.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

int main(int argc, char**argv)
{
   int sockfd,n;
   struct sockaddr_in servaddr,cliaddr;
   char sendline[1000];
   char recvline[1000];

   if (argc != 3)
   {
      printf("usage:  udpcli <IP address> <port>\n");
      exit(1);
   }

   sockfd=socket(AF_INET,SOCK_DGRAM,0);

   bzero(&servaddr,sizeof(servaddr));
   servaddr.sin_family = AF_INET;
   servaddr.sin_addr.s_addr=inet_addr(argv[1]);
   servaddr.sin_port=htons(atoi(argv[2]));

   while (fgets(sendline, 10000,stdin) != NULL)
   {
		printf("Sending...\n");
      sendto(sockfd,sendline,strlen(sendline),0, (struct sockaddr *)&servaddr,sizeof(servaddr));
		printf("Sent\n");
		printf("Reading...\n");
      n=recvfrom(sockfd,recvline,100,0,NULL,NULL);
		printf("Read %d bytes\n", n);
      recvline[n]=0;
      fputs(recvline,stdout);
   }
}

