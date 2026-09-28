#include <assert.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
static int count;
static char message[512];
static void log_line(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vsnprintf(message, sizeof(message), fmt, args);
    va_end(args);
    count++;
}
#include "physx_game_mode.h"
int main(int argc, char **argv)
{
    if (argc == 2) {
        char line[8192];
        FILE *input = fopen(argv[1], "r");
        assert(input);
        while (fgets(line, sizeof(line), input)) {
            int before = count;
            physx_game_mode_log_line(line);
            if (before != count) puts(message);
        }
        fclose(input);
        assert(count > 0);
        return 0;
    }
    physx_game_mode_log_line(NULL);
    physx_game_mode_log_line("include \"LUA/VAR_Mode___PoseEdit|0\";");
    physx_game_mode_log_line("openstream: file: \"LUA/VAR_Mode___PoseEdit|0\" doesn't exist");
    physx_game_mode_log_line("ExecuteFile 'LUA/VAR_Mode___|0'");
    physx_game_mode_log_line("ExecuteFile 'LUA/VAR_Mode___PoseEdit|0' extra");
    physx_game_mode_log_line("ExecuteFile 'LUA/VAR_Mode___Bad mode|0'");
    physx_game_mode_log_line("ExecuteFile 'LUA/VAR_Mode___PoseEdit|");
    assert(count == 0);
    physx_game_mode_log_line("ExecuteFile 'LUA/VAR_Mode___Start|0'");
    assert(count == 1 && strstr(message, "previous=\"unknown\""));
    physx_game_mode_log_line("ExecuteFile 'LUA/VAR_Mode___Start|0'");
    assert(count == 1);
    const char *modes[] = {"Full", "PoseEdit", "Customizer", "Photo", "Sequencer", "SequencerExtended", "Story", "Start"};
    for (unsigned i=0; i<sizeof(modes)/sizeof(modes[0]); i++) {
        char line[128], expected[100];
        snprintf(line, sizeof(line), "  ExecuteFile 'LUA/VAR_Mode___%s|0'\r\n", modes[i]);
        snprintf(expected, sizeof(expected), "native_ui_mode=\"%s\"", modes[i]);
        physx_game_mode_log_line(line);
        assert(count == (int)i+2 && strstr(message, expected));
        if (i) {
            snprintf(expected, sizeof(expected), "previous=\"%s\"", modes[i-1]);
            assert(strstr(message, expected));
        }
    }
    puts("PASS: native mode transitions, duplicate suppression, malformed input and script/error false positives");
    return 0;
}
