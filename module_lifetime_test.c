#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#define REQUIRE(x) do { if(!(x)) { fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#x); return 1; } } while(0)
typedef int (*callback_t)(void);
typedef int (*install_t)(int,callback_t *);
int main(int argc,char **argv)
{
    HMODULE module;
    install_t install;
    callback_t callback=NULL;
    int pinned,i;
    REQUIRE(argc==3);
    pinned=atoi(argv[2]);
    module=LoadLibraryA(argv[1]); REQUIRE(module);
    install=(install_t)(void*)GetProcAddress(module,"install"); REQUIRE(install);
    REQUIRE(!install(-1,&callback)); REQUIRE(!callback);
    REQUIRE(install(pinned,&callback)); REQUIRE(callback && callback()==42);
    REQUIRE(FreeLibrary(module));
    if(!pinned) {
        REQUIRE(GetModuleHandleA(argv[1])==NULL);
        puts("PASS: unprotected control unloads (stale callback deliberately not invoked)");
        return 0;
    }
    REQUIRE(GetModuleHandleA(argv[1])==module);
    for(i=0;i<256;i++) {
        REQUIRE(FreeLibrary(module));
        REQUIRE(GetModuleHandleA(argv[1])==module);
        REQUIRE(callback()==42);
    }
    REQUIRE(install(1,&callback)); REQUIRE(callback()==42);
    puts("PASS: pinned callback survives repeated FreeLibrary; failed pin publishes no callback");
    return 0;
}
