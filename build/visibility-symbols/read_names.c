#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <tlhelp32.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static HANDLE process;
static int read_at(DWORD address,void *out,SIZE_T size) {
 SIZE_T got; return ReadProcessMemory(process,(void *)(ULONG_PTR)address,out,size,&got)&&got==size;
}
static DWORD word(DWORD address) { DWORD v=0;read_at(address,&v,4);return v; }
int main(int argc,char **argv) {
 MODULEENTRY32 entry={0};HANDLE snapshot;DWORD base=0,count,i; char name[129];
 if(argc!=4)return 2;
 snapshot=CreateToolhelp32Snapshot(TH32CS_SNAPMODULE|TH32CS_SNAPMODULE32,strtoul(argv[1],0,10));
 entry.dwSize=sizeof(entry);
 if(Module32First(snapshot,&entry))do {
  if(!_stricmp(entry.szModule,"NC-TK17-PhysX.dll"))base=(DWORD)(ULONG_PTR)entry.modBaseAddr;
 }while(Module32Next(snapshot,&entry));
 CloseHandle(snapshot);if(!base)return 3;
 process=OpenProcess(PROCESS_VM_READ|PROCESS_QUERY_INFORMATION,FALSE,strtoul(argv[1],0,10));
 if(!process){printf("OpenProcess error=%lu\n",GetLastError());return 4;}
 count=word(base+strtoul(argv[2],0,16));printf("count=%lu\n",count);if(count>16384)return 5;
 for(i=0;i<count;i++) {
  DWORD address=base+strtoul(argv[3],0,16)+i*140,obj,meta,prop,get;BYTE code[6]={0};
  memset(name,0,sizeof(name));if(!read_at(address,name,128))continue;
  if(!strstr(name,"genital01_SG"))continue;
  obj=word(address+128);meta=word(obj-0x18);prop=word(meta+0x2cc);get=word(prop+0x80);
  read_at(get,code,6);
  printf("name=%s obj=%08lx visibility=%lu supported=%lu getter=%02x%02x%02x%02x%02x%02x\n",name,obj,word(obj+0x20),word(prop-0x1c),code[0],code[1],code[2],code[3],code[4],code[5]);
 }

 { MEMORY_BASIC_INFORMATION info; ULONG_PTR address=0; BYTE *buffer=malloc(1024*1024+256); unsigned hits=0;
   const char *needle="body_subdiv_cageShape__body_genital01_SG"; unsigned needle_size=strlen(needle);
   while(address<0x80000000u && hits<40 && VirtualQueryEx(process,(void *)address,&info,sizeof(info))) {
    ULONG_PTR next=(ULONG_PTR)info.BaseAddress+info.RegionSize;
    if(next<=address)break;
    if(info.State==MEM_COMMIT && !(info.Protect&(PAGE_GUARD|PAGE_NOACCESS))) {
     ULONG_PTR cursor=address;
     while(cursor<next && hits<40) {
      SIZE_T length=next-cursor,got=0,k;if(length>1024*1024)length=1024*1024;
      if(ReadProcessMemory(process,(void *)cursor,buffer,length,&got)) {
       for(k=0;k+needle_size<=got;++k)if(buffer[k]=='b' && !memcmp(buffer+k,needle,needle_size)) {
        SIZE_T start=k,end=k+needle_size;
        while(start>0 && k-start<100 && buffer[start-1]>=32 && buffer[start-1]<127)--start;
        while(end<got && end-k<180 && buffer[end]>=32 && buffer[end]<127)++end;
        printf("string=%08lx %.*s\n",(DWORD)(cursor+start),(int)(end-start),buffer+start);++hits;
       }
      }
      cursor+=length;
     }
    }
    address=next;
   }
   free(buffer);
 }
 CloseHandle(process);return 0;
}
